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

// ---------------------------------------------------------------------------
// openrcad Solid builders
// ---------------------------------------------------------------------------

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
