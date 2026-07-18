use super::*;

/// Axis-aligned bounding box of a solid from its B-Rep vertices, as
/// `(min, max)`. Exact for polygonal solids; a conservative-enough estimate for
/// curved ones (used only for cheap overlap pre-tests). `None` if vertexless.
pub fn solid_aabb(solid: &KernelSolid) -> Option<([f32; 3], [f32; 3])> {
    let (lo, hi) = solid.bounding_box().corners()?;
    Some((
        [lo.x() as f32, lo.y() as f32, lo.z() as f32],
        [hi.x() as f32, hi.y() as f32, hi.z() as f32],
    ))
}

/// Whether two AABBs overlap (or touch within `eps`). Used to skip boolean
/// attempts between solids that can't possibly interact.
pub fn aabbs_overlap(a: &([f32; 3], [f32; 3]), b: &([f32; 3], [f32; 3]), eps: f32) -> bool {
    (0..3).all(|k| a.0[k] - eps <= b.1[k] && b.0[k] - eps <= a.1[k])
}

/// Whether `outer` fully encloses `inner` (allowing `eps` slack on every side).
/// A boolean union `a ∪ b` must contain `a`, so its AABB must contain `a`'s —
/// used as a cheap sanity check to reject a degenerate union that would
/// otherwise silently delete the body it merged into.
pub fn aabb_contains(outer: &([f32; 3], [f32; 3]), inner: &([f32; 3], [f32; 3]), eps: f32) -> bool {
    (0..3).all(|k| outer.0[k] - eps <= inner.0[k] && inner.1[k] <= outer.1[k] + eps)
}

/// Whether topology-separated solid components still form one physically
/// connected piece of material by sharing boundary vertices. Boolean union can
/// retain a seam between tangent/adjacent sketch regions (for example a capsule
/// made from a rectangle and circle), even though those regions are one body.
/// A genuinely separated pair such as concentric rings has no shared boundary
/// point and remains multiple bodies.
pub fn components_form_connected_material(parts: &[KernelSolid]) -> bool {
    if parts.len() <= 1 {
        return true;
    }

    let vertices: Vec<_> = parts.iter().map(KernelSolid::vertices).collect();
    let meshes: Vec<_> = parts.iter().map(MockMesh::from_solid).collect();
    let vertex_near_surface = |vertex_mesh: &MockMesh, surface_mesh: &MockMesh| {
        vertex_mesh.vertices.chunks_exact(6).any(|vertex| {
            let point = [vertex[0], vertex[1], vertex[2]];
            surface_mesh.indices.chunks_exact(3).any(|triangle| {
                let position = |index: u32| {
                    let offset = index as usize * 6;
                    [
                        surface_mesh.vertices[offset],
                        surface_mesh.vertices[offset + 1],
                        surface_mesh.vertices[offset + 2],
                    ]
                };
                crate::mock_kernel::point_triangle_distance(
                    point,
                    position(triangle[0]),
                    position(triangle[1]),
                    position(triangle[2]),
                ) <= 1.0e-4
            })
        })
    };
    let touches = |a: usize, b: usize| {
        let aabb_overlap = match (solid_aabb(&parts[a]), solid_aabb(&parts[b])) {
            (Some(a), Some(b)) => aabbs_overlap(&a, &b, 1.0e-4),
            _ => false,
        };
        aabb_overlap
            && (vertices[a].iter().any(|va| {
                vertices[b]
                    .iter()
                    .any(|vb| va.point().distance(&vb.point()) <= 1.0e-4)
            }) || vertex_near_surface(&meshes[a], &meshes[b])
                || vertex_near_surface(&meshes[b], &meshes[a]))
    };

    let mut reached = vec![false; parts.len()];
    reached[0] = true;
    let mut pending = vec![0usize];
    while let Some(current) = pending.pop() {
        for (candidate, connected) in reached.iter_mut().enumerate() {
            if !*connected && touches(current, candidate) {
                *connected = true;
                pending.push(candidate);
            }
        }
    }
    reached.into_iter().all(|connected| connected)
}

/// Package several already-validated components into one solid container. This
/// is the honest fallback when a boolean union cannot sew tangent components:
/// their individual shells remain intact, while evaluation can still retain
/// their one-body material connectivity and tessellate the shared seam away.
pub fn aggregate_solid_components(parts: &[KernelSolid]) -> Option<KernelSolid> {
    (!parts.is_empty()).then(|| {
        let faces = parts.iter().flat_map(|part| part.shell().faces());
        KernelSolid::new(openrcad::topo::Shell::from_faces(faces))
    })
}

// ---------------------------------------------------------------------------
// openrcad Solid builders
// ---------------------------------------------------------------------------

/// Axis, radius, and axial extent of a cylindrical face on a solid, resolved for
/// the thread feature. `origin`/`dir` define the axis line; `radius` is the wall
/// radius; `axial_min`/`axial_max` bound the threaded span along `dir` (measured
/// from `origin`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderFaceInfo {
    pub origin: [f32; 3],
    pub dir: [f32; 3],
    pub radius: f32,
    pub axial_min: f32,
    pub axial_max: f32,
    /// True when the face orientation reverses the cylinder's intrinsic radial
    /// normal, as on the wall of a drilled hole.
    pub internal: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConeFaceInfo {
    pub origin: [f32; 3],
    pub dir: [f32; 3],
    pub reference_radius: f32,
    pub semi_angle: f32,
    pub axial_min: f32,
    pub axial_max: f32,
    pub internal: bool,
}

impl ConeFaceInfo {
    pub fn radius_at(self, axial: f32) -> f32 {
        self.reference_radius + axial * self.semi_angle.tan()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereFaceInfo {
    pub center: [f32; 3],
    pub radius: f32,
    pub internal: bool,
}

/// Find the cylindrical face of `solid` whose wall passes closest to `centroid`
/// (the captured pick point) and report its axis/radius/axial-extent. The extent
/// is measured from the tessellated vertices that lie on that cylinder, so it
/// matches the visible face span. `None` if the solid has no cylindrical face.
pub fn cylinder_face_near(solid: &KernelSolid, centroid: [f32; 3]) -> Option<CylinderFaceInfo> {
    let radial_dist = |p: [f32; 3], origin: [f32; 3], dir: [f32; 3]| -> (f32, f32) {
        let rel = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
        let t = rel[0] * dir[0] + rel[1] * dir[1] + rel[2] * dir[2];
        let perp = [
            rel[0] - dir[0] * t,
            rel[1] - dir[1] * t,
            rel[2] - dir[2] * t,
        ];
        (
            (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt(),
            t,
        )
    };

    let mut best: Option<(f32, [f32; 3], [f32; 3], f32, bool)> = None;
    for face in solid.shell().faces() {
        let Some(GeomSurface::Cylinder(cyl)) = face.surface() else {
            continue;
        };
        let p = cyl.position();
        let loc = p.location();
        let d = p.direction();
        let dl = (d.x() * d.x() + d.y() * d.y() + d.z() * d.z()).sqrt();
        if dl < 1e-9 {
            continue;
        }
        let origin = [loc.x() as f32, loc.y() as f32, loc.z() as f32];
        let dir = [
            (d.x() / dl) as f32,
            (d.y() / dl) as f32,
            (d.z() / dl) as f32,
        ];
        let r = cyl.radius() as f32;
        let (radial, _) = radial_dist(centroid, origin, dir);
        let err = (radial - r).abs();
        if best.as_ref().map_or(true, |b| err < b.0) {
            best = Some((
                err,
                origin,
                dir,
                r,
                face.orientation() == Orientation::Reversed,
            ));
        }
    }
    let (_, origin, dir, radius, internal) = best?;

    // Axial extent from the tessellated vertices that lie on this cylinder.
    let mesh = try_display_mesh_from_part(solid)?;
    let tol = 0.05 * radius.max(1.0) + 0.05;
    let mut amin = f32::INFINITY;
    let mut amax = f32::NEG_INFINITY;
    for v in mesh.vertices.chunks(6) {
        let (radial, t) = radial_dist([v[0], v[1], v[2]], origin, dir);
        if (radial - radius).abs() <= tol {
            amin = amin.min(t);
            amax = amax.max(t);
        }
    }
    if !amin.is_finite() || amax - amin <= 1e-3 {
        return None;
    }
    Some(CylinderFaceInfo {
        origin,
        dir,
        radius,
        axial_min: amin,
        axial_max: amax,
        internal,
    })
}

/// Resolve the conical face whose analytic support is closest to a captured
/// pick point, including the exact axial trim recovered from vertices on that
/// support surface.
pub fn cone_face_near(solid: &KernelSolid, centroid: [f32; 3]) -> Option<ConeFaceInfo> {
    let mut best: Option<(f32, ConeFaceInfo)> = None;
    for face in solid.shell().faces() {
        let Some(GeomSurface::Cone(cone)) = face.surface() else {
            continue;
        };
        let position = cone.position();
        let location = position.location();
        let direction = position.direction();
        let origin = [
            location.x() as f32,
            location.y() as f32,
            location.z() as f32,
        ];
        let dir = [
            direction.x() as f32,
            direction.y() as f32,
            direction.z() as f32,
        ];
        let relative = [
            centroid[0] - origin[0],
            centroid[1] - origin[1],
            centroid[2] - origin[2],
        ];
        let axial = relative[0] * dir[0] + relative[1] * dir[1] + relative[2] * dir[2];
        let radial = magnitude3([
            relative[0] - dir[0] * axial,
            relative[1] - dir[1] * axial,
            relative[2] - dir[2] * axial,
        ]);
        let info = ConeFaceInfo {
            origin,
            dir,
            reference_radius: cone.ref_radius() as f32,
            semi_angle: cone.semi_angle() as f32,
            axial_min: f32::INFINITY,
            axial_max: f32::NEG_INFINITY,
            internal: face.orientation() == Orientation::Reversed,
        };
        let error = (radial - info.radius_at(axial)).abs();
        if best.as_ref().is_none_or(|(best, _)| error < *best) {
            best = Some((error, info));
        }
    }
    let (_, mut info) = best?;
    let scale = info.reference_radius.abs().max(1.0);
    let tolerance = scale * 0.05 + 0.05;
    for vertex in solid.vertices() {
        let point = vertex.point();
        let relative = [
            point.x() as f32 - info.origin[0],
            point.y() as f32 - info.origin[1],
            point.z() as f32 - info.origin[2],
        ];
        let axial =
            relative[0] * info.dir[0] + relative[1] * info.dir[1] + relative[2] * info.dir[2];
        let radial = magnitude3([
            relative[0] - info.dir[0] * axial,
            relative[1] - info.dir[1] * axial,
            relative[2] - info.dir[2] * axial,
        ]);
        if (radial - info.radius_at(axial)).abs() <= tolerance {
            info.axial_min = info.axial_min.min(axial);
            info.axial_max = info.axial_max.max(axial);
        }
    }
    (info.axial_min.is_finite() && info.axial_max - info.axial_min > 1.0e-4).then_some(info)
}

pub fn sphere_face_near(solid: &KernelSolid, centroid: [f32; 3]) -> Option<SphereFaceInfo> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| {
            let GeomSurface::Sphere(sphere) = face.surface()? else {
                return None;
            };
            let center = sphere.center();
            let info = SphereFaceInfo {
                center: [center.x() as f32, center.y() as f32, center.z() as f32],
                radius: sphere.radius() as f32,
                internal: face.orientation() == Orientation::Reversed,
            };
            let error = (magnitude3([
                centroid[0] - info.center[0],
                centroid[1] - info.center[1],
                centroid[2] - info.center[2],
            ]) - info.radius)
                .abs();
            Some((error, info))
        })
        .min_by(|(left, _), (right, _)| left.total_cmp(right))
        .map(|(_, info)| info)
}

fn magnitude3(vector: [f32; 3]) -> f32 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

pub fn solid_has_cylindrical_face(solid: &KernelSolid) -> bool {
    solid
        .shell()
        .faces()
        .iter()
        .any(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
}

pub fn preserves_cylindrical_faces(reference: &KernelSolid, candidate: &KernelSolid) -> bool {
    let reference = cylinder_identity_signatures(reference);
    if reference.is_empty() {
        return true;
    }
    let candidate = cylinder_identity_signatures(candidate);
    reference
        .iter()
        .all(|sig| candidate.iter().any(|cand| cand == sig))
}

pub fn preserved_cylindrical_face_ids(
    reference: &KernelSolid,
    candidate: &KernelSolid,
) -> std::collections::HashSet<u32> {
    let reference = cylinder_identity_signatures(reference);
    if reference.is_empty() {
        return std::collections::HashSet::new();
    }
    let mut out = std::collections::HashSet::new();
    for (i, face) in candidate.shell().faces().iter().enumerate() {
        let Some(GeomSurface::Cylinder(cyl)) = face.surface() else {
            continue;
        };
        if reference
            .iter()
            .any(|sig| *sig == cylinder_identity_signature(cyl))
        {
            out.insert(i as u32);
        }
    }
    out
}

pub(crate) fn cylinder_identity_signatures(
    solid: &KernelSolid,
) -> Vec<(i64, i64, i64, i64, i64, i64, i64)> {
    let mut out = Vec::new();
    for face in solid.shell().faces() {
        let Some(GeomSurface::Cylinder(cyl)) = face.surface() else {
            continue;
        };
        let sig = cylinder_identity_signature(cyl);
        if !out.contains(&sig) {
            out.push(sig);
        }
    }
    out
}

pub(crate) fn cylinder_identity_signature(
    cyl: &CylindricalSurface,
) -> (i64, i64, i64, i64, i64, i64, i64) {
    let q = |v: f64| (v * 1.0e4).round() as i64;
    let p = cyl.position();
    let d = p.direction();
    let (mut dx, mut dy, mut dz) = (d.x(), d.y(), d.z());
    let lead = if dx.abs() > 1e-9 {
        dx
    } else if dy.abs() > 1e-9 {
        dy
    } else {
        dz
    };
    if lead < 0.0 {
        dx = -dx;
        dy = -dy;
        dz = -dz;
    }
    let loc = p.location();
    let t = loc.x() * dx + loc.y() * dy + loc.z() * dz;
    let (fx, fy, fz) = (loc.x() - dx * t, loc.y() - dy * t, loc.z() - dz * t);
    (q(fx), q(fy), q(fz), q(dx), q(dy), q(dz), q(cyl.radius()))
}

pub(crate) fn render_mesh_is_closed_manifold(mesh: &MockMesh) -> bool {
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            quant(mesh.vertices[b]),
            quant(mesh.vertices[b + 1]),
            quant(mesh.vertices[b + 2]),
        )
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges.values().all(|&count| count == 2)
}

pub(crate) fn render_mesh_normals_follow_winding(mesh: &MockMesh) -> bool {
    for tri in mesh.indices.chunks_exact(3) {
        let p = |i: u32| {
            let b = i as usize * 6;
            [
                mesh.vertices[b] as f64,
                mesh.vertices[b + 1] as f64,
                mesh.vertices[b + 2] as f64,
            ]
        };
        let n = |i: u32| {
            let b = i as usize * 6;
            [
                mesh.vertices[b + 3] as f64,
                mesh.vertices[b + 4] as f64,
                mesh.vertices[b + 5] as f64,
            ]
        };
        let a = p(tri[0]);
        let b = p(tri[1]);
        let c = p(tri[2]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let winding = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let navg = [
            (n(tri[0])[0] + n(tri[1])[0] + n(tri[2])[0]) / 3.0,
            (n(tri[0])[1] + n(tri[1])[1] + n(tri[2])[1]) / 3.0,
            (n(tri[0])[2] + n(tri[1])[2] + n(tri[2])[2]) / 3.0,
        ];
        if winding[0] * navg[0] + winding[1] * navg[1] + winding[2] * navg[2] < -1.0e-7 {
            return false;
        }
    }
    true
}

pub(crate) fn loop_is_concave(loop_pts: &[(f32, f32)]) -> bool {
    if loop_pts.len() < 4 {
        return false;
    }
    let pts: Vec<(f64, f64)> = loop_pts
        .iter()
        .map(|&(x, y)| (x as f64, y as f64))
        .collect();
    let area = pts
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let q = pts[(i + 1) % pts.len()];
            p.0 * q.1 - p.1 * q.0
        })
        .sum::<f64>();
    let winding = area.signum();
    if winding == 0.0 {
        return false;
    }

    for i in 0..pts.len() {
        let a = pts[(i + pts.len() - 1) % pts.len()];
        let b = pts[i];
        let c = pts[(i + 1) % pts.len()];
        let ab = (b.0 - a.0, b.1 - a.1);
        let bc = (c.0 - b.0, c.1 - b.1);
        let cross = ab.0 * bc.1 - ab.1 * bc.0;
        if cross * winding < -1.0e-7 {
            return true;
        }
    }
    false
}
