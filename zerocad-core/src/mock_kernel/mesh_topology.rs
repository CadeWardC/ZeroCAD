use super::*;

/// The boundary loops of one mesh face (`fid` in [`MockMesh::face_ids`]) — its
/// outer wire plus any hole rims — projected into `cs`'s 2D plane as line
/// segments, ready to join a sketch's region detection as reference geometry.
///
/// Extraction is purely mesh-based: among the face's triangles, an undirected
/// edge used by exactly ONE triangle is on the face boundary (internal
/// triangulation diagonals are shared by two). The edges are chained into
/// closed loops, then collinear runs are merged so a subdivided straight rim
/// comes back as one segment; genuinely curved rims (hole circles) keep their
/// tessellation chords, matching how region detection flattens drawn circles.
/// Shared by the GUI (sketch-on-face capture) and the evaluator (re-deriving
/// the outline from wherever the face is after the body changes) so both
/// produce bit-identical curves for the same mesh face.
pub fn mesh_face_boundary_2d(
    mesh: &MockMesh,
    fid: u32,
    cs: &crate::geometry::CoordinateSystem,
) -> crate::sketch::SketchCurves {
    use crate::geometry::Vec3;
    use std::collections::HashMap;

    let mut out = crate::sketch::SketchCurves::new();
    let ntris = mesh.indices.len() / 3;
    let vpos = |vi: usize| -> Vec3 {
        let i = vi * 6;
        Vec3::new(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2])
    };
    let q = |p: Vec3| -> (i64, i64, i64) {
        let s = |v: f32| (v as f64 * 1.0e4).round() as i64;
        (s(p.x), s(p.y), s(p.z))
    };

    // Undirected edge use-count among this face's triangles.
    let mut edge_use: HashMap<((i64, i64, i64), (i64, i64, i64)), (u32, Vec3, Vec3)> =
        HashMap::new();
    for t in 0..ntris {
        if mesh.face_ids.get(t).copied() != Some(fid) {
            continue;
        }
        for k in 0..3 {
            let a = vpos(mesh.indices[t * 3 + k] as usize);
            let b = vpos(mesh.indices[t * 3 + (k + 1) % 3] as usize);
            let (ka, kb) = (q(a), q(b));
            if ka == kb {
                continue;
            }
            let (key, pa, pb) = if ka < kb {
                ((ka, kb), a, b)
            } else {
                ((kb, ka), b, a)
            };
            edge_use.entry(key).or_insert((0, pa, pb)).0 += 1;
        }
    }
    // Deterministic boundary-edge order (HashMap iteration is randomized; the
    // loops themselves are canonicalized below, this keeps multi-loop
    // discovery order stable too).
    let mut boundary: Vec<(((i64, i64, i64), (i64, i64, i64)), Vec3, Vec3)> = edge_use
        .iter()
        .filter(|(_, (c, _, _))| *c == 1)
        .map(|(key, (_, a, b))| (*key, *a, *b))
        .collect();
    boundary.sort_by_key(|(k, _, _)| *k);
    let boundary: Vec<(Vec3, Vec3)> = boundary.into_iter().map(|(_, a, b)| (a, b)).collect();

    // Chain the loose boundary edges into closed loops.
    let mut adj: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (i, (a, b)) in boundary.iter().enumerate() {
        adj.entry(q(*a)).or_default().push(i);
        adj.entry(q(*b)).or_default().push(i);
    }
    let to2 = |p: Vec3| -> (f32, f32) {
        let rel = p.sub(cs.origin);
        (rel.dot(cs.u), rel.dot(cs.v))
    };
    let mut loops: Vec<Vec<(f32, f32)>> = Vec::new();
    let mut used = vec![false; boundary.len()];
    for start in 0..boundary.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let (a0, b0) = boundary[start];
        let mut pts: Vec<Vec3> = vec![a0, b0];
        let start_key = q(a0);
        let mut cursor = q(b0);
        while cursor != start_key {
            let Some(&ei) = adj
                .get(&cursor)
                .and_then(|cands| cands.iter().find(|&&e| !used[e]))
            else {
                break;
            };
            used[ei] = true;
            let (a, b) = boundary[ei];
            let nxt = if q(a) == cursor { b } else { a };
            cursor = q(nxt);
            if cursor != start_key {
                pts.push(nxt);
            }
        }
        if pts.len() < 3 || cursor != start_key {
            continue; // open chain — not a valid face loop
        }

        // Merge collinear runs (2D): keep a vertex only where the loop
        // actually turns, so a refinement-subdivided straight rim projects
        // as one clean segment.
        let p2: Vec<(f32, f32)> = pts.iter().map(|&p| to2(p)).collect();
        let n = p2.len();
        let mut keep: Vec<(f32, f32)> = Vec::new();
        for i in 0..n {
            let prev = p2[(i + n - 1) % n];
            let cur = p2[i];
            let next = p2[(i + 1) % n];
            let (ux, uy) = (cur.0 - prev.0, cur.1 - prev.1);
            let (vx, vy) = (next.0 - cur.0, next.1 - cur.1);
            let lens = (ux.hypot(uy) * vx.hypot(vy)).max(1.0e-9);
            let sin_turn = (ux * vy - uy * vx) / lens;
            let dot = ux * vx + uy * vy;
            if sin_turn.abs() > 1.0e-3 || dot < 0.0 {
                keep.push(cur);
            }
        }
        if keep.len() < 3 {
            continue;
        }
        loops.push(keep);
    }

    // Canonicalize every loop — CCW winding, starting at the lexicographically
    // smallest vertex — and order the loops by that start vertex. The emitted
    // curves must be BIT-IDENTICAL for the same face regardless of the mesh's
    // triangle/hash order: callers hash-compare a stored outline against a
    // re-derived one to decide whether the face actually changed.
    for keep in &mut loops {
        let n = keep.len();
        let area: f32 = (0..n)
            .map(|i| {
                let a = keep[i];
                let b = keep[(i + 1) % n];
                a.0 * b.1 - b.0 * a.1
            })
            .sum();
        if area < 0.0 {
            keep.reverse();
        }
        let start = keep
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);
        keep.rotate_left(start);
    }
    loops.sort_by(|a, b| {
        a.first()
            .partial_cmp(&b.first())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for keep in &loops {
        for i in 0..keep.len() {
            out.add_line(keep[i], keep[(i + 1) % keep.len()]);
        }
    }
    out
}

/// Build a hidden-line-ready wireframe from a tessellated mesh (the interleaved
/// `[x,y,z,nx,ny,nz]` `vertices`, `indices`, and one `face_ids` entry per
/// triangle). Returns `(edge_vertices, edge_indices, edge_face_normals)` in the
/// same layout the analytic box/extrusion wireframes use.
///
/// Only *feature* edges are kept: a triangle edge shared by two triangles with
/// **different** face ids (the crease between two B-Rep faces), or a lone mesh
/// boundary edge. Internal triangulation diagonals — shared by two triangles of
/// the *same* face — are dropped. Each kept edge records its (up to two)
/// adjacent face normals so the renderer hides it when both faces turn away.
///
/// Deriving edges from triangles rather than the raw B-Rep is what fixes the
/// boolean "stray lines": a degenerate zero-area fin face produces no triangle,
/// so it contributes no edge, and back edges now get proper hidden-line removal
/// instead of x-raying through the body.
/// Group B-rep faces that lie on the *same* analytic surface (cylinder, torus,
/// cone, or adjacent coplanar planes), returning one group id per face in
/// `solid.shell().faces()` order (which matches the mesh's `face_id`s). The
/// kernel emits a cylindrical wall — a bored hole, a round boss — as 3
/// arc-faces (thirds), a circular-rim fillet/chamfer band as one torus/cone
/// sector per rim fragment, and a full revolve's planar cap as 3 pie wedges
/// (the same 120° stations); the seams between those sectors are construction
/// artifacts, not design edges: drawn, they make a hole read as a notched
/// circle, a rim fillet read as a segmented band, and a revolved disc read as
/// sliced pie. Faces sharing a group are recognised as one surface so those
/// seams can be suppressed (and the whole band selects as one face). Every
/// other face (and each distinct surface) gets its own id, so only true
/// same-surface faces match.
///
/// Planar faces group only when they are BOTH coplanar and edge-adjacent
/// (union-find over shared boundary spans) — never by plane identity alone.
/// Two disjoint coplanar faces (the two top lands of a U-shaped part) are
/// distinct design faces and must keep separate ids; a revolve cap's wedges
/// share their radial seam edges, so they merge. Analytic curved surfaces keep
/// the identity-only rule: a boolean can fragment one cylinder wall into
/// non-adjacent pieces that must still read as one surface. Adjacent ruled
/// surfaces are grouped only across a shared ruling when their curved rails are
/// tangent-continuous. This lets consecutive glyph Bezier spans read and select
/// as one curved wall without merging a curve into a straight wall or hiding a
/// real corner.
pub(crate) fn cylinder_surface_groups(solid: &KernelSolid) -> Vec<u32> {
    // Quantized surface identity. Cylinder: (axis-foot xyz, axis-dir xyz,
    // radius, 0). Torus: (centre xyz, axis-dir xyz, major radius, minor
    // radius). Cone: (apex xyz, axis-dir xyz, tan(semi-angle), 0). A leading
    // tag keeps the kinds from ever colliding.
    type SurfSig = (u8, i64, i64, i64, i64, i64, i64, i64, i64);
    /// A point quantized to the same grid, for geometric edge-span matching.
    type QPnt = (i64, i64, i64);
    let q = |v: f64| (v * 1.0e4).round() as i64;
    // Sign-normalize a direction so +axis and -axis hash the same; returns the
    // (possibly flipped) components and whether it flipped.
    let canon_dir = |d: openrcad::foundation::Dir| -> (f64, f64, f64, bool) {
        let (dx, dy, dz) = (d.x(), d.y(), d.z());
        let lead = if dx.abs() > 1e-9 {
            dx
        } else if dy.abs() > 1e-9 {
            dy
        } else {
            dz
        };
        if lead < 0.0 {
            (-dx, -dy, -dz, true)
        } else {
            (dx, dy, dz, false)
        }
    };
    // A cylinder's identity is its axis *line* + radius, independent of which
    // generator/location names the axis. Canonicalize the axis point to the foot
    // of the perpendicular from the origin, and the direction to a fixed sign.
    let cyl_sig = |s: &CylindricalSurface| -> SurfSig {
        let p = s.position();
        let (dx, dy, dz, _) = canon_dir(p.direction());
        let loc = p.location();
        let t = loc.x() * dx + loc.y() * dy + loc.z() * dz; // (loc·d)
        let (fx, fy, fz) = (loc.x() - dx * t, loc.y() - dy * t, loc.z() - dz * t);
        (
            0,
            q(fx),
            q(fy),
            q(fz),
            q(dx),
            q(dy),
            q(dz),
            q(s.radius()),
            0,
        )
    };
    // A torus is symmetric under flipping its axis, so its identity is centre +
    // axis line + both radii; the centre already lies on the axis.
    let torus_sig = |s: &openrcad::geom::ToroidalSurface| -> SurfSig {
        let p = s.position();
        let (dx, dy, dz, _) = canon_dir(p.direction());
        let c = p.location();
        (
            1,
            q(c.x()),
            q(c.y()),
            q(c.z()),
            q(dx),
            q(dy),
            q(dz),
            q(s.major_radius()),
            q(s.minor_radius()),
        )
    };
    // A cone's identity is its apex + axis line + slope. Flipping the axis
    // negates the semi-angle (widening in +Z becomes narrowing), so negate the
    // slope when the direction was sign-flipped.
    let cone_sig = |s: &openrcad::geom::ConicalSurface| -> Option<SurfSig> {
        let p = s.position();
        let slope = s.semi_angle().tan();
        if slope.abs() <= 1.0e-9 {
            return None; // degenerate (a cylinder in disguise); don't group.
        }
        let (dx, dy, dz, flipped) = canon_dir(p.direction());
        let slope = if flipped { -slope } else { slope };
        // r(v) = ref_radius + v·tanα = 0 → apex at v = −ref_radius/tanα along
        // the ORIGINAL direction.
        let v = -s.ref_radius() / s.semi_angle().tan();
        let d0 = p.direction();
        let loc = p.location();
        let apex = (
            loc.x() + d0.x() * v,
            loc.y() + d0.y() * v,
            loc.z() + d0.z() * v,
        );
        Some((
            2,
            q(apex.0),
            q(apex.1),
            q(apex.2),
            q(dx),
            q(dy),
            q(dz),
            q(slope),
            0,
        ))
    };

    // A plane's identity is its normal *line* direction + signed offset from
    // the origin along that (sign-canonicalized) normal. Used only as the
    // coplanarity test for the adjacency merge below — never as a standalone
    // grouping key.
    let plane_sig = |s: &Plane| -> SurfSig {
        let (dx, dy, dz, _) = canon_dir(s.normal());
        let loc = s.location();
        let d = loc.x() * dx + loc.y() * dy + loc.z() * dz;
        (3, q(dx), q(dy), q(dz), q(d), 0, 0, 0, 0)
    };

    enum Kind {
        /// Curved analytic surface: group by identity alone.
        Analytic(SurfSig),
        /// Plane: group only with coplanar edge-adjacent neighbours.
        Planar(SurfSig),
        /// Swept curved rail: group only with tangent edge-adjacent neighbours.
        Ruled,
        /// Ungroupable: a fresh, unshareable id.
        Other,
    }
    let faces = solid.shell().faces();
    let kinds: Vec<Kind> = faces
        .iter()
        .map(|face| match face.surface() {
            Some(GeomSurface::Cylinder(c)) => Kind::Analytic(cyl_sig(c)),
            Some(GeomSurface::Torus(t)) => Kind::Analytic(torus_sig(t)),
            Some(GeomSurface::Cone(c)) => cone_sig(c).map_or(Kind::Other, Kind::Analytic),
            Some(GeomSurface::Plane(p)) => Kind::Planar(plane_sig(p)),
            Some(GeomSurface::Ruled(_)) => Kind::Ruled,
            _ => Kind::Other,
        })
        .collect();

    // Union-find over adjacency-sensitive faces. Planes join when coplanar;
    // ruled surfaces join only at a shared sweep-direction edge where their
    // rails meet tangentially. Spans match geometrically (quantized endpoints +
    // curve midpoint), not by EdgeId — sew usually unifies coincident edges but
    // adjacent faces may still hold independent copies. The midpoint keeps a
    // straight chord from ever matching an arc between the same endpoints.
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let qp = |p: &Pnt| (q(p.x()), q(p.y()), q(p.z()));
    let edge_key = |edge: &Edge| {
        let a = edge.source().point();
        let b = edge.target().point();
        let (qa, qb) = (qp(&a), qp(&b));
        if qa == qb {
            return None;
        }
        let mid = edge
            .curve()
            .map(|c| c.point(0.5 * (edge.first() + edge.last())))
            .unwrap_or_else(|| {
                Pnt::new(
                    0.5 * (a.x() + b.x()),
                    0.5 * (a.y() + b.y()),
                    0.5 * (a.z() + b.z()),
                )
            });
        let (lo, hi) = if qa <= qb { (qa, qb) } else { (qb, qa) };
        Some((lo, hi, qp(&mid)))
    };
    #[derive(Clone, Copy)]
    struct SmoothRuledBoundary {
        normal: [f64; 3],
        inward_tangent: [f64; 3],
    }
    #[derive(Clone, Copy)]
    struct EdgeOwner {
        face: usize,
        ruled: Option<SmoothRuledBoundary>,
    }
    let unit = |vector: GeomVec| {
        vector
            .normalized()
            .map(|direction| [direction.x(), direction.y(), direction.z()])
    };
    let mut edge_owners: HashMap<(QPnt, QPnt, QPnt), Vec<EdgeOwner>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        match (&kinds[fi], face.surface()) {
            (Kind::Planar(_), _) => {
                for wire in face.wires() {
                    for edge in wire.edges() {
                        if let Some(key) = edge_key(&edge) {
                            edge_owners.entry(key).or_default().push(EdgeOwner {
                                face: fi,
                                ruled: None,
                            });
                        }
                    }
                }
            }
            (Kind::Ruled, Some(GeomSurface::Ruled(ruled))) => {
                // A prism's lateral ruled patch has two pcurve edges at constant
                // U, spanning V=0..1. Those are its sweep-direction boundaries.
                // Evaluate the surface tangent plane there and orient dU toward
                // the patch interior; two genuinely smooth neighbouring spans
                // have equal tangent planes and opposing inward dU directions.
                for wire in face.wires() {
                    struct Candidate {
                        key: (QPnt, QPnt, QPnt),
                        u: f64,
                        normal: [f64; 3],
                        tangent: [f64; 3],
                    }
                    let edges = wire.edges();
                    let mut candidates = Vec::new();
                    for (edge_index, edge) in edges.iter().enumerate() {
                        let Some(key) = edge_key(edge) else {
                            continue;
                        };
                        let Some(pcurve) = wire.pcurve(edge_index) else {
                            continue;
                        };
                        let uv0 = pcurve.point_at_fraction(0.0);
                        let uv1 = pcurve.point_at_fraction(1.0);
                        let u_scale = 1.0 + uv0.x().abs().max(uv1.x().abs());
                        if (uv1.x() - uv0.x()).abs() > 1.0e-8 * u_scale
                            || (uv1.y() - uv0.y()).abs() < 0.5
                        {
                            continue;
                        }
                        let u = 0.5 * (uv0.x() + uv1.x());
                        let v = 0.5 * (uv0.y() + uv1.y());
                        let (_, du, dv) = ruled.d1(u, v);
                        let Some(normal) = unit(du.cross(&dv)) else {
                            continue;
                        };
                        let Some(tangent) = unit(du) else {
                            continue;
                        };
                        candidates.push(Candidate {
                            key,
                            u,
                            normal,
                            tangent,
                        });
                    }
                    for (candidate_index, candidate) in candidates.iter().enumerate() {
                        let Some(other) = candidates
                            .iter()
                            .enumerate()
                            .filter(|(index, _)| *index != candidate_index)
                            .max_by(|(_, left), (_, right)| {
                                (left.u - candidate.u)
                                    .abs()
                                    .total_cmp(&(right.u - candidate.u).abs())
                            })
                            .map(|(_, other)| other)
                        else {
                            continue;
                        };
                        let inward_sign = (other.u - candidate.u).signum();
                        if inward_sign == 0.0 {
                            continue;
                        }
                        let inward_tangent = [
                            candidate.tangent[0] * inward_sign,
                            candidate.tangent[1] * inward_sign,
                            candidate.tangent[2] * inward_sign,
                        ];
                        edge_owners
                            .entry(candidate.key)
                            .or_default()
                            .push(EdgeOwner {
                                face: fi,
                                ruled: Some(SmoothRuledBoundary {
                                    normal: candidate.normal,
                                    inward_tangent,
                                }),
                            });
                    }
                }
            }
            _ => {}
        }
    }
    for owners in edge_owners.values() {
        for (k, &left) in owners.iter().enumerate() {
            for &right in &owners[k + 1..] {
                let (i, j) = (left.face, right.face);
                let joins = if let (Kind::Planar(si), Kind::Planar(sj)) = (&kinds[i], &kinds[j]) {
                    si == sj
                } else if matches!((&kinds[i], &kinds[j]), (Kind::Ruled, Kind::Ruled)) {
                    left.ruled.zip(right.ruled).is_some_and(|(a, b)| {
                        let normal_dot = a.normal[0] * b.normal[0]
                            + a.normal[1] * b.normal[1]
                            + a.normal[2] * b.normal[2];
                        let inward_dot = a.inward_tangent[0] * b.inward_tangent[0]
                            + a.inward_tangent[1] * b.inward_tangent[1]
                            + a.inward_tangent[2] * b.inward_tangent[2];
                        // About 2.6 degrees of angular slack covers f32-derived
                        // glyph control points while preserving intentional kinks.
                        normal_dot.abs() >= 0.999 && inward_dot <= -0.999
                    })
                } else {
                    false
                };
                if joins {
                    let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                    if ri != rj {
                        parent[rj] = ri;
                    }
                }
            }
        }
    }

    let mut groups = Vec::with_capacity(faces.len());
    let mut seen: HashMap<SurfSig, u32> = HashMap::new();
    let mut adjacent_roots: HashMap<usize, u32> = HashMap::new();
    let mut next = 0u32;
    for fi in 0..faces.len() {
        let mut fresh = || {
            let g = next;
            next += 1;
            g
        };
        let id = match &kinds[fi] {
            Kind::Analytic(sig) => *seen.entry(*sig).or_insert_with(fresh),
            Kind::Planar(_) | Kind::Ruled => {
                let root = find(&mut parent, fi);
                *adjacent_roots.entry(root).or_insert_with(fresh)
            }
            Kind::Other => fresh(),
        };
        groups.push(id);
    }
    groups
}

pub(crate) fn mesh_feature_edges(
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
    surface_group: &[u32],
) -> (Vec<f32>, Vec<u32>, Vec<f32>, Vec<(u32, u32)>) {
    // Quantize a vertex position so the independent per-face copies of a shared
    // corner collapse to one key (1e-4 mm, matching the watertightness test).
    let key = |idx: usize| -> (i64, i64, i64) {
        let b = idx * 6;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    let pos = |idx: usize| -> [f32; 3] {
        let b = idx * 6;
        [vertices[b], vertices[b + 1], vertices[b + 2]]
    };
    // The crease test deliberately uses the *smoothed* per-vertex normal
    // (`solid_to_flat_mesh` runs `smooth_vertex_normals`, which blends normals
    // across shallow creases). That makes adjacent facets of a curved/boolean'd
    // surface look nearly identical, so their seams are suppressed and the round
    // reads as one face — while a genuine sharp edge (≥30°, beyond the smoothing
    // cap) keeps each face's distinct normal and still draws. A planar face's
    // vertices share one normal, so the first vertex's is fine.
    let nrm = |idx: usize| -> [f32; 3] {
        let b = idx * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    struct EdgeRec {
        pa: [f32; 3],
        pb: [f32; 3],
        tris: u32,
        faces: Vec<(u32, [f32; 3])>, // distinct (face id, that face's normal)
    }
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), EdgeRec> = HashMap::new();

    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let n = nrm(tri[0] as usize);
        for &(i, j) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (vi, vj) = (tri[i] as usize, tri[j] as usize);
            let (ka, kb) = (key(vi), key(vj));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            let rec = edges.entry(k).or_insert_with(|| EdgeRec {
                pa: pos(vi),
                pb: pos(vj),
                tris: 0,
                faces: Vec::new(),
            });
            rec.tris += 1;
            if !rec.faces.iter().any(|(f, _)| *f == fid) {
                rec.faces.push((fid, n));
            }
        }
    }

    let mut edge_vertices: Vec<f32> = Vec::new();
    let mut edge_indices: Vec<u32> = Vec::new();
    let mut edge_face_normals: Vec<f32> = Vec::new();
    // Per kept segment: the canonical pair of surface ids it borders. Aligned with
    // each `edge_indices` pair, this is what `group_edge_segments` chains on so an
    // arc's chords (constant surface pair along the whole arc) become one edge.
    let mut edge_pairs: Vec<(u32, u32)> = Vec::new();
    for rec in edges.values() {
        // Keep only a crease between two *distinct* B-rep faces. This drops both
        // internal triangulation diagonals (two triangles of the SAME face) and
        // lone boundary edges (a single triangle owns the edge). Every ZeroCAD
        // body is a closed solid, so a watertight tessellation has no genuine
        // boundary edge — a single-triangle edge is a crack/sliver left by a
        // fragile boolean, and drawing it is exactly the "stray spray". Real
        // design edges are always shared by ≥2 faces, so they're untouched.
        if rec.faces.len() < 2 {
            continue;
        }
        // Suppress a *same-surface* seam: the straight longitudinal edge where two
        // arc-faces of ONE analytic cylinder meet (the kernel splits a cylindrical
        // wall into thirds). It's a construction artifact, not a design edge — drawn,
        // it makes a bored hole / round boss look notched. Recognised by surface
        // identity rather than normals (the per-face representative normals here are
        // too coarse: each 120° arc-face's sample normal can sit ~60° off the seam).
        if rec.faces.len() == 2 {
            let g0 = surface_group.get(rec.faces[0].0 as usize);
            let g1 = surface_group.get(rec.faces[1].0 as usize);
            if g0.is_some() && g0 == g1 {
                continue;
            }
        }
        // Suppress tangent-continuous boundaries, including the construction seam
        // where a circular end joins a straight profile. The B-Rep faces remain
        // intact; this only removes their misleading display/selectable edge.
        // Genuine corners, chamfers, and cylinder rims exceed this threshold.
        if rec.faces.len() >= 2 {
            let n0 = rec.faces[0].1;
            let n1 = rec.faces[1].1;
            let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0);
            const CREASE_COS: f32 = 0.95; // cos(~18°)
            if dot > CREASE_COS {
                continue;
            }
        }
        // Drop a degenerate zero-length edge (collapsed by a sliver triangle): it
        // would render as a stray dot/spike and never as a real line.
        let d2 = (rec.pa[0] - rec.pb[0]).powi(2)
            + (rec.pa[1] - rec.pb[1]).powi(2)
            + (rec.pa[2] - rec.pb[2]).powi(2);
        if d2 < 1.0e-12 {
            continue;
        }
        let a = (edge_vertices.len() / 3) as u32;
        edge_vertices.extend_from_slice(&rec.pa);
        edge_vertices.extend_from_slice(&rec.pb);
        edge_indices.push(a);
        edge_indices.push(a + 1);
        // Two adjacent face normals; duplicate the lone one on a boundary edge.
        let n0 = rec.faces[0].1;
        let n1 = rec.faces.get(1).map_or(n0, |(_, n)| *n);
        edge_face_normals.extend_from_slice(&n0);
        edge_face_normals.extend_from_slice(&n1);
        // Bordering surface pair, canonicalized so arc-faces of one cylinder match.
        let f0 = rec.faces[0].0;
        let f1 = rec.faces.get(1).map_or(f0, |(f, _)| *f);
        edge_pairs.push(canonical_surface_pair(f0, f1, surface_group));
    }

    (edge_vertices, edge_indices, edge_face_normals, edge_pairs)
}

/// Canonical, order-independent key for the pair of *surfaces* an edge borders.
///
/// Each face id is mapped through `surface_group` so the arc-faces a cylinder is
/// split into collapse to one surface — then the smaller id is placed first.
/// Segments that share this key and touch are chords of the same topological
/// edge (see [`group_edge_segments`]).
pub(crate) fn canonical_surface_pair(f0: u32, f1: u32, surface_group: &[u32]) -> (u32, u32) {
    let g0 = surface_group.get(f0 as usize).copied().unwrap_or(f0);
    let g1 = surface_group.get(f1 as usize).copied().unwrap_or(f1);
    if g0 <= g1 {
        (g0, g1)
    } else {
        (g1, g0)
    }
}

/// Group edge **segments** into topological edges, returning one group id per
/// segment (parallel to the `edge_indices` pairs).
///
/// Two segments that meet at a shared (welded) endpoint are placed in the same
/// group when:
/// * `surface_pairs` is given (B-Rep solids) — they border the same surface pair.
///   An arc keeps one pair along its whole length, so its chords chain into one
///   edge; a corner where the pair changes splits them.
/// * `surface_pairs` is `None` (analytic primitive wireframes, which carry no
///   face provenance) — exactly two segments meet there and pass through nearly
///   straight (tangent-continuous). This chains a rim circle's chords while
///   leaving a box's 90° corners as separate edges.
pub(crate) fn group_edge_segments(
    edge_vertices: &[f32],
    edge_indices: &[u32],
    surface_pairs: Option<&[(u32, u32)]>,
) -> Vec<u32> {
    let n = edge_indices.len() / 2;
    if n == 0 {
        return Vec::new();
    }

    // Weld endpoints by quantized position (1e-4 mm, matching the wireframe build).
    let vkey = |vi: u32| -> (i64, i64, i64) {
        let b = vi as usize * 3;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            q(edge_vertices[b]),
            q(edge_vertices[b + 1]),
            q(edge_vertices[b + 2]),
        )
    };
    let pos = |vi: u32| -> [f64; 3] {
        let b = vi as usize * 3;
        [
            edge_vertices[b] as f64,
            edge_vertices[b + 1] as f64,
            edge_vertices[b + 2] as f64,
        ]
    };

    // Welded vertex -> the segments incident to it.
    let mut at: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for s in 0..n {
        at.entry(vkey(edge_indices[s * 2])).or_default().push(s);
        at.entry(vkey(edge_indices[s * 2 + 1])).or_default().push(s);
    }

    // Union-find over segments.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let union = |parent: &mut [usize], a: usize, b: usize| {
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb {
            parent[ra.max(rb)] = ra.min(rb);
        }
    };

    // Direction of segment `s` pointing away from welded vertex `key`.
    let dir_away = |s: usize, key: (i64, i64, i64)| -> Option<[f64; 3]> {
        let (a, b) = (edge_indices[s * 2], edge_indices[s * 2 + 1]);
        let (from, to) = if vkey(a) == key { (a, b) } else { (b, a) };
        let (p, q) = (pos(from), pos(to));
        let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        (l > 1e-9).then(|| [d[0] / l, d[1] / l, d[2] / l])
    };

    for (key, segs) in &at {
        match surface_pairs {
            Some(pairs) => {
                // Chain every pair of incident segments that border the same surfaces.
                for i in 0..segs.len() {
                    for j in (i + 1)..segs.len() {
                        if pairs[segs[i]] == pairs[segs[j]] {
                            union(&mut parent, segs[i], segs[j]);
                        }
                    }
                }
            }
            None => {
                // Chain any pair of incident segments that pass through nearly
                // straight (tangent-continuous). Their outward dirs point opposite
                // for a smooth pass; ~25° slack chains coarse rim arcs yet splits
                // corners. Pairwise (not degree-2 only) so a rim still chains
                // *through* a silhouette strut's junction — the two rim chords are
                // tangent while the strut, branching off, joins neither.
                for i in 0..segs.len() {
                    for j in (i + 1)..segs.len() {
                        if let (Some(d0), Some(d1)) =
                            (dir_away(segs[i], *key), dir_away(segs[j], *key))
                        {
                            let dot = d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2];
                            if dot < -0.9 {
                                union(&mut parent, segs[i], segs[j]);
                            }
                        }
                    }
                }
            }
        }
    }

    // Densely renumber roots into stable group ids.
    let mut group = vec![0u32; n];
    let mut remap: HashMap<usize, u32> = HashMap::new();
    let mut next = 0u32;
    for (s, g) in group.iter_mut().enumerate() {
        let r = find(&mut parent, s);
        *g = *remap.entry(r).or_insert_with(|| {
            let id = next;
            next += 1;
            id
        });
    }
    group
}

/// One [`MeshFaceRef`] per distinct `face_id`, with its area-weighted centroid
/// and outward normal computed from the triangle mesh. This is the geometric half
/// of a face's identity; the durable *name* (`topology`) is stamped separately by
/// whichever feature created the face. Faces are returned in ascending `face_id`
/// order (via `BTreeMap`) so the result is deterministic.
pub(crate) fn mesh_face_refs(
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
) -> Vec<MeshFaceRef> {
    use std::collections::BTreeMap;
    // Per face id: (Σ area-weighted normal, Σ area-weighted centroid, Σ area).
    let mut acc: BTreeMap<u32, ([f64; 3], [f64; 3], f64)> = BTreeMap::new();
    let tri_count = indices.len() / 3;
    for t in 0..tri_count {
        let Some(&fid) = face_ids.get(t) else {
            continue;
        };
        let pos = |k: usize| -> [f64; 3] {
            let i = indices[t * 3 + k] as usize * 6;
            [
                vertices[i] as f64,
                vertices[i + 1] as f64,
                vertices[i + 2] as f64,
            ]
        };
        let a = pos(0);
        let b = pos(1);
        let c = pos(2);
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        // Cross product; its magnitude is twice the triangle area, so summing it
        // directly gives an area-weighted (unnormalized) face normal.
        let n = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let area = 0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let cen = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        let e = acc.entry(fid).or_insert(([0.0; 3], [0.0; 3], 0.0));
        for k in 0..3 {
            e.0[k] += n[k];
            e.1[k] += cen[k] * area;
        }
        e.2 += area;
    }
    acc.into_iter()
        .map(|(face_id, (nsum, csum, area))| {
            let nlen = (nsum[0] * nsum[0] + nsum[1] * nsum[1] + nsum[2] * nsum[2]).sqrt();
            let normal = if nlen > 0.0 {
                [
                    (nsum[0] / nlen) as f32,
                    (nsum[1] / nlen) as f32,
                    (nsum[2] / nlen) as f32,
                ]
            } else {
                [0.0, 0.0, 0.0]
            };
            let centroid = if area > 0.0 {
                [
                    (csum[0] / area) as f32,
                    (csum[1] / area) as f32,
                    (csum[2] / area) as f32,
                ]
            } else {
                [0.0, 0.0, 0.0]
            };
            MeshFaceRef {
                face_id,
                centroid,
                normal,
                topology: None,
            }
        })
        .collect()
}

/// Stamp each edge with the durable **names of its two adjacent faces** (its
/// face-owner pair). In the persistent-naming model an edge is identified by the
/// pair of faces it separates, so once faces carry names an edge inherits a stable
/// identity for free — one that survives a boolean even when the edge's own id is
/// re-derived. Requires `face_refs` to be named first (planar faces only; a curved
/// adjacency is left unpaired and falls back to the edge's own id).
pub(crate) fn populate_edge_adjacent_face_names(mesh: &mut MockMesh) {
    // Named planar faces as (normal, centroid, name).
    let named: Vec<([f32; 3], [f32; 3], String)> = mesh
        .face_refs
        .iter()
        .filter_map(|f| {
            let name = f.topology.as_ref().and_then(|t| t.face_id.clone())?;
            Some((f.normal, f.centroid, name))
        })
        .collect();
    if named.is_empty() {
        return;
    }
    // The face whose outward normal matches `n` and whose plane contains `mid`.
    let find = |n: [f32; 3], mid: [f32; 3]| -> Option<String> {
        let mut best: Option<(f32, &str)> = None;
        for (fnrm, fc, name) in &named {
            let ndot = n[0] * fnrm[0] + n[1] * fnrm[1] + n[2] * fnrm[2];
            if ndot < 0.99 {
                continue;
            }
            let dist = ((mid[0] - fc[0]) * fnrm[0]
                + (mid[1] - fc[1]) * fnrm[1]
                + (mid[2] - fc[2]) * fnrm[2])
                .abs();
            if dist > 1.0e-2 {
                continue;
            }
            if best.as_ref().is_none_or(|(bd, _)| dist < *bd) {
                best = Some((dist, name.as_str()));
            }
        }
        best.map(|(_, name)| name.to_string())
    };
    for e in &mut mesh.edge_refs {
        let mid = [
            (e.p0[0] + e.p1[0]) * 0.5,
            (e.p0[1] + e.p1[1]) * 0.5,
            (e.p0[2] + e.p1[2]) * 0.5,
        ];
        let (Some(a), Some(b)) = (find(e.n1, mid), find(e.n2, mid)) else {
            continue;
        };
        if a == b {
            continue;
        }
        let mut pair = [a, b];
        pair.sort();
        match &mut e.topology {
            Some(t) => {
                t.adjacent_face_ids = pair.to_vec();
                if t.producer_feature_id.is_none() {
                    t.producer_feature_id = current_feature_context();
                }
            }
            None => {
                e.topology = Some(MeshTopologyEdgeRef {
                    adjacent_face_ids: pair.to_vec(),
                    producer_feature_id: current_feature_context(),
                    ..Default::default()
                })
            }
        }
    }
}

/// The geometric normals of the (up to) two triangles of the display mesh
/// sharing the chord `a`–`b`, matched by quantized vertex position. `None`
/// unless exactly two distinct-normal triangles touch the chord.
///
/// The per-segment `edge_face_normals` a wireframe stores can be face
/// REPRESENTATIVE directions on analytic (smooth-cylinder) tessellations — a
/// wall normal sampled somewhere ELSE around the rim, up to 180° from this
/// chord. Anything that reads a normal's radial *sign* at the chord (the
/// fillet/chamfer preview ribbon does) then flips. The chord's own adjacent
/// triangles are always local, so prefer them.
pub(crate) fn chord_adjacent_triangle_normals(
    vertices: &[f32],
    indices: &[u32],
    a: [f32; 3],
    b: [f32; 3],
) -> Option<([f32; 3], [f32; 3])> {
    let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(p[0]), q(p[1]), q(p[2]))
    };
    let (ka, kb) = (qkey(a), qkey(b));
    let vert = |vi: u32| -> [f32; 3] {
        let o = vi as usize * 6;
        [vertices[o], vertices[o + 1], vertices[o + 2]]
    };
    let mut normals: Vec<[f32; 3]> = Vec::new();
    for tri in indices.chunks_exact(3) {
        let ps = [vert(tri[0]), vert(tri[1]), vert(tri[2])];
        let ks = [qkey(ps[0]), qkey(ps[1]), qkey(ps[2])];
        if !(ks.contains(&ka) && ks.contains(&kb)) {
            continue;
        }
        let u = [
            ps[1][0] - ps[0][0],
            ps[1][1] - ps[0][1],
            ps[1][2] - ps[0][2],
        ];
        let v = [
            ps[2][0] - ps[0][0],
            ps[2][1] - ps[0][1],
            ps[2][2] - ps[0][2],
        ];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len <= 1.0e-9 {
            continue;
        }
        let n = [n[0] / len, n[1] / len, n[2] / len];
        // Coplanar neighbors (two cap triangles) count once.
        let dup = normals
            .iter()
            .any(|m| m[0] * n[0] + m[1] * n[1] + m[2] * n[2] > 0.999);
        if !dup {
            normals.push(n);
        }
        if normals.len() > 2 {
            return None;
        }
    }
    (normals.len() == 2).then(|| (normals[0], normals[1]))
}

pub(crate) fn mesh_edge_refs_from_groups(
    vertices: &[f32],
    indices: &[u32],
    edge_vertices: &[f32],
    edge_indices: &[u32],
    edge_face_normals: &[f32],
    edge_groups: &[u32],
) -> Vec<MeshEdgeRef> {
    let seg_count = edge_indices.len() / 2;
    if seg_count == 0 || edge_groups.len() != seg_count {
        return Vec::new();
    }

    let mut by_group: HashMap<u32, Vec<usize>> = HashMap::new();
    for (seg, &group) in edge_groups.iter().enumerate() {
        by_group.entry(group).or_default().push(seg);
    }

    let mut out = Vec::with_capacity(by_group.len());
    for (group, segs) in by_group {
        let Some(&first) = segs.first() else {
            continue;
        };
        let vpos = |seg: usize, which: usize| -> [f32; 3] {
            let vi = edge_indices[seg * 2 + which] as usize * 3;
            [
                edge_vertices[vi],
                edge_vertices[vi + 1],
                edge_vertices[vi + 2],
            ]
        };
        let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
            let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
            (q(p[0]), q(p[1]), q(p[2]))
        };

        let mut uses: HashMap<(i64, i64, i64), (u32, [f32; 3])> = HashMap::new();
        let mut pts = Vec::new();
        for &seg in &segs {
            for which in 0..2 {
                let p = vpos(seg, which);
                uses.entry(qkey(p)).or_insert((0, p)).0 += 1;
                if !pts.iter().any(|q: &[f32; 3]| dist3(*q, p) <= 1.0e-4) {
                    pts.push(p);
                }
            }
        }

        let mut ends: Vec<[f32; 3]> = uses
            .values()
            .filter(|(count, _)| *count == 1)
            .map(|(_, p)| *p)
            .collect();
        ends.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let closed = ends.len() < 2;
        let (p0, p1) = if ends.len() >= 2 {
            (ends[0], ends[1])
        } else {
            (vpos(first, 0), vpos(first, 1))
        };

        let fo = first * 6;
        if edge_face_normals.len() < fo + 6 {
            continue;
        }
        let mut n1 = [
            edge_face_normals[fo],
            edge_face_normals[fo + 1],
            edge_face_normals[fo + 2],
        ];
        let mut n2 = [
            edge_face_normals[fo + 3],
            edge_face_normals[fo + 4],
            edge_face_normals[fo + 5],
        ];
        // The stored pair can be face-representative (sampled far from this
        // chord on smooth walls); anything reading a radial SIGN off n1/n2 at
        // p0/p1 — the edge-mod preview ribbon — then flips. Re-localize to the
        // chord's own two adjacent display triangles when they resolve.
        if let Some((t1, t2)) =
            chord_adjacent_triangle_normals(vertices, indices, vpos(first, 0), vpos(first, 1))
        {
            n1 = t1;
            n2 = t2;
        }
        let curve = edge_curve_hint_from_points(&pts, p0, p1, n1, n2, closed);
        let curve_kind = match curve {
            Some(EdgeCurveHint::Circle { .. }) => Some("circle".to_string()),
            Some(EdgeCurveHint::Line) => Some("line".to_string()),
            None => None,
        };
        let edge_id = Some(format!("mesh:{group}"));
        out.push(MeshEdgeRef {
            group,
            p0,
            p1,
            n1,
            n2,
            curve,
            topology: Some(MeshTopologyEdgeRef {
                topology_version: Some(0),
                edge_id,
                curve_kind,
                adjacent_surface_kinds: vec!["unknown".to_string(), "unknown".to_string()],
                producer_feature_id: current_feature_context(),
                ..MeshTopologyEdgeRef::default()
            }),
        });
    }
    out.sort_by_key(|edge| edge.group);
    out
}

pub(crate) fn dist3(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

pub(crate) fn edge_curve_hint_from_points(
    points: &[[f32; 3]],
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
    closed: bool,
) -> Option<EdgeCurveHint> {
    if points.len() < 3 {
        return Some(EdgeCurveHint::Line);
    }

    let pts: Vec<Vec3> = points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    let p0v = Vec3::new(p0[0], p0[1], p0[2]);
    let p1v = Vec3::new(p1[0], p1[1], p1[2]);
    let mut axes = vec![
        Vec3::new(n1[0], n1[1], n1[2]).normalize(),
        Vec3::new(n2[0], n2[1], n2[2]).normalize(),
    ];
    let mut chord_axis = Vec3::ZERO;
    for i in 1..pts.len() {
        for j in (i + 1)..pts.len() {
            let axis = pts[i].sub(pts[0]).cross(pts[j].sub(pts[0]));
            if axis.length() > chord_axis.length() {
                chord_axis = axis;
            }
        }
    }
    axes.push(chord_axis.normalize());

    let mut best: Option<(f32, EdgeCurveHint)> = None;
    for axis in axes {
        if axis.length() < 0.5 {
            continue;
        }
        if let Some((score, hint)) = fit_circle_hint_on_axis(&pts, p0v, p1v, axis, closed) {
            if best
                .as_ref()
                .is_none_or(|(best_score, _)| score < *best_score)
            {
                best = Some((score, hint));
            }
        }
    }
    best.map(|(_, hint)| hint).or(Some(EdgeCurveHint::Line))
}

pub(crate) fn fit_circle_hint_on_axis(
    pts: &[Vec3],
    p0: Vec3,
    p1: Vec3,
    axis: Vec3,
    closed: bool,
) -> Option<(f32, EdgeCurveHint)> {
    let axis = axis.normalize();
    let base = if axis.dot(Vec3::X).abs() < 0.9 {
        Vec3::X
    } else {
        Vec3::Y
    };
    let u = base.sub(axis.mul(base.dot(axis))).normalize();
    if u.length() < 0.5 {
        return None;
    }
    let v = axis.cross(u).normalize();
    let origin = pts[0];
    let project = |p: Vec3| -> (f32, f32, f32) {
        let d = p.sub(origin);
        (d.dot(u), d.dot(v), d.dot(axis))
    };
    if pts
        .iter()
        .map(|&p| project(p).2.abs())
        .fold(0.0f32, f32::max)
        > 0.05
    {
        return None;
    }

    let projected: Vec<(f32, f32)> = pts
        .iter()
        .map(|&p| {
            let (x, y, _) = project(p);
            (x, y)
        })
        .collect();
    let mut circle = None;
    'outer: for i in 0..projected.len() {
        for j in (i + 1)..projected.len() {
            for k in (j + 1)..projected.len() {
                if let Some(c) =
                    circle_from_three_points_2d(projected[i], projected[j], projected[k])
                {
                    circle = Some(c);
                    break 'outer;
                }
            }
        }
    }
    let (cx, cy, radius) = circle?;
    if radius <= 1.0e-4 {
        return None;
    }
    let residual = projected
        .iter()
        .map(|&(x, y)| ((x - cx).hypot(y - cy) - radius).abs())
        .fold(0.0f32, f32::max);
    if residual > (0.01 * radius).max(0.05) {
        return None;
    }

    let center = origin.add(u.mul(cx)).add(v.mul(cy));
    let mut x_dir = p0.sub(center).normalize();
    if x_dir.length() < 0.5 {
        x_dir = pts[0].sub(center).normalize();
    }
    x_dir = x_dir.sub(axis.mul(x_dir.dot(axis))).normalize();
    if x_dir.length() < 0.5 {
        return None;
    }
    let y_dir = axis.cross(x_dir).normalize();
    let angle = |p: Vec3| {
        let d = p.sub(center);
        d.dot(y_dir).atan2(d.dot(x_dir))
    };
    let end = if closed {
        std::f32::consts::TAU
    } else {
        let raw_end = angle(p1);
        let forward = normalize_positive(raw_end);
        let reverse = forward - std::f32::consts::TAU;
        let contains = |span_end: f32| -> bool {
            pts.iter()
                .all(|&p| angle_in_span_f32(angle(p), 0.0, span_end, 0.08))
        };
        if contains(forward) {
            forward
        } else if contains(reverse) {
            reverse
        } else {
            raw_end
        }
    };

    Some((
        residual,
        EdgeCurveHint::Circle {
            center: [center.x, center.y, center.z],
            axis: [axis.x, axis.y, axis.z],
            x_dir: [x_dir.x, x_dir.y, x_dir.z],
            radius,
            start: 0.0,
            end,
            closed,
        },
    ))
}

pub(crate) fn normalize_positive(mut a: f32) -> f32 {
    while a < 0.0 {
        a += std::f32::consts::TAU;
    }
    while a >= std::f32::consts::TAU {
        a -= std::f32::consts::TAU;
    }
    a
}

pub(crate) fn angle_in_span_f32(angle: f32, start: f32, end: f32, tol: f32) -> bool {
    let span = end - start;
    if span.abs() >= std::f32::consts::TAU - tol {
        return true;
    }
    if span >= 0.0 {
        let mut rel = angle - start;
        while rel < -tol {
            rel += std::f32::consts::TAU;
        }
        while rel > std::f32::consts::TAU + tol {
            rel -= std::f32::consts::TAU;
        }
        rel <= span + tol
    } else {
        let mut rel = start - angle;
        while rel < -tol {
            rel += std::f32::consts::TAU;
        }
        while rel > std::f32::consts::TAU + tol {
            rel -= std::f32::consts::TAU;
        }
        rel <= -span + tol
    }
}

#[cfg(test)]
mod surface_group_tests {
    use super::*;

    /// Axis-adjacent rectangle in the XZ plane (y=0), the standard revolve
    /// profile: x from `x0` to `x1`, z from `z0` to `z1`.
    fn rect_profile(x0: f64, z0: f64, x1: f64, z1: f64) -> Face {
        let p = [
            Pnt::new(x0, 0.0, z0),
            Pnt::new(x1, 0.0, z0),
            Pnt::new(x1, 0.0, z1),
            Pnt::new(x0, 0.0, z1),
        ];
        let edges: Vec<Edge> = (0..4)
            .map(|i| Edge::between_points(p[i], p[(i + 1) % 4]))
            .collect();
        Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                p[0],
                Dir::dy(),
            ))),
            Wire::from_edges(edges),
        )
    }

    /// A full revolve's flat annular cap is emitted as three 120° pie wedges
    /// (the kernel's thirds). Coplanar + edge-adjacent, so they must share ONE
    /// surface group — the seams are construction artifacts, and the disc must
    /// read/select as a single face. A washer has exactly 4 true surfaces.
    #[test]
    fn revolved_washer_caps_group_as_one_face_each() {
        let face = rect_profile(1.0, 0.0, 2.0, 3.0);
        let solid = openrcad::algo::revolve::revolve_operation(
            &face,
            Pnt::origin(),
            Dir::dz(),
            std::f64::consts::TAU,
        )
        .expect("washer revolve")
        .value;
        let faces = solid.shell().faces();
        assert_eq!(faces.len(), 12, "4 profile edges x 3 stations");
        let groups = cylinder_surface_groups(&solid);
        let distinct: HashSet<u32> = groups.iter().copied().collect();
        assert_eq!(
            distinct.len(),
            4,
            "outer wall, inner wall, top cap, bottom cap; got groups {groups:?}"
        );
        // Each cap's three wedges carry one shared id.
        for (target_z, name) in [(0.0f64, "bottom"), (3.0, "top")] {
            let ids: HashSet<u32> = faces
                .iter()
                .enumerate()
                .filter(|(_, f)| {
                    matches!(
                        f.surface(),
                        Some(GeomSurface::Plane(p))
                            if (p.location().z() - target_z).abs() < 1e-9
                    )
                })
                .map(|(i, _)| groups[i])
                .collect();
            assert_eq!(ids.len(), 1, "{name} cap wedges must share one group");
        }
    }

    /// Coplanar but NON-adjacent planar faces must keep distinct groups: the
    /// two arm-tip faces of an extruded U both lie on y=3 but are separated by
    /// the notch — grouping them would make a click select both. Plane
    /// identity alone is never enough; adjacency is required.
    #[test]
    fn disjoint_coplanar_faces_stay_separate() {
        let pts = [
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 3.0),
            (2.0, 3.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 3.0),
            (0.0, 3.0),
        ];
        let edges: Vec<Edge> = (0..pts.len())
            .map(|i| {
                let (ax, ay) = pts[i];
                let (bx, by) = pts[(i + 1) % pts.len()];
                Edge::between_points(Pnt::new(ax, ay, 0.0), Pnt::new(bx, by, 0.0))
            })
            .collect();
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges(edges),
        );
        let solid = openrcad::algo::prism::prism_operation(&face, GeomVec::new(0.0, 0.0, 2.0))
            .expect("U prism")
            .value;
        let faces = solid.shell().faces();
        let groups = cylinder_surface_groups(&solid);
        // The two arm-tip laterals on the plane y=3.
        let tip_ids: Vec<u32> = faces
            .iter()
            .enumerate()
            .filter(|(_, f)| {
                matches!(
                    f.surface(),
                    Some(GeomSurface::Plane(p))
                        if p.normal().y().abs() > 0.99 && (p.location().y() - 3.0).abs() < 1e-9
                )
            })
            .map(|(i, _)| groups[i])
            .collect();
        assert_eq!(tip_ids.len(), 2, "expected the two arm-tip faces");
        assert_ne!(
            tip_ids[0], tip_ids[1],
            "disjoint coplanar faces must not share a group"
        );
    }

    fn quadratic_edge(poles: [Pnt; 3]) -> Edge {
        let curve = BSplineCurve::new(2, poles.to_vec(), None, vec![0.0, 1.0], vec![3, 3]);
        Edge::new(
            Some(GeomCurve::bspline(curve)),
            0.0,
            1.0,
            Vertex::new(poles[0]),
            Vertex::new(poles[2]),
        )
    }

    fn two_curve_prism(second_control: Pnt) -> KernelSolid {
        let start = Pnt::new(0.0, 0.0, 0.0);
        let curve_start = Pnt::new(2.0, 0.0, 0.0);
        let join = Pnt::new(3.0, 1.0, 0.0);
        let curve_end = Pnt::new(2.0, 2.0, 0.0);
        let upper_left = Pnt::new(0.0, 2.0, 0.0);
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                start,
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(start, curve_start),
                quadratic_edge([curve_start, Pnt::new(3.0, 0.0, 0.0), join]),
                quadratic_edge([join, second_control, curve_end]),
                Edge::between_points(curve_end, upper_left),
                Edge::between_points(upper_left, start),
            ]),
        );
        openrcad::algo::prism::prism_operation(&face, GeomVec::new(0.0, 0.0, 2.0))
            .expect("curved profile prism")
            .value
    }

    #[test]
    fn tangent_curved_prism_spans_select_as_one_face_without_a_vertical_seam() {
        let solid = two_curve_prism(Pnt::new(3.0, 2.0, 0.0));
        let faces = solid.shell().faces();
        let groups = cylinder_surface_groups(&solid);
        let ruled: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter_map(|(index, face)| {
                matches!(face.surface(), Some(GeomSurface::Ruled(_))).then_some(index)
            })
            .collect();
        assert_eq!(ruled.len(), 2, "fixture must produce two curved wall spans");
        assert_eq!(
            groups[ruled[0]], groups[ruled[1]],
            "tangent-connected Bezier walls must share one selectable face"
        );

        let mesh = MockMesh::from_solid(&solid);
        let has_join_seam = mesh.edge_indices.chunks_exact(2).any(|segment| {
            let point = |index: u32| {
                let base = index as usize * 3;
                [
                    mesh.edge_vertices[base],
                    mesh.edge_vertices[base + 1],
                    mesh.edge_vertices[base + 2],
                ]
            };
            let a = point(segment[0]);
            let b = point(segment[1]);
            (a[0] - 3.0).abs() < 1.0e-4
                && (a[1] - 1.0).abs() < 1.0e-4
                && (b[0] - 3.0).abs() < 1.0e-4
                && (b[1] - 1.0).abs() < 1.0e-4
                && (a[2] - b[2]).abs() > 1.0
        });
        assert!(
            !has_join_seam,
            "the internal Bezier-span ruling must not be drawn/selectable"
        );
    }

    #[test]
    fn sharp_curved_prism_spans_remain_separate_faces() {
        let solid = two_curve_prism(Pnt::new(2.0, 1.2, 0.0));
        let faces = solid.shell().faces();
        let groups = cylinder_surface_groups(&solid);
        let ruled: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter_map(|(index, face)| {
                matches!(face.surface(), Some(GeomSurface::Ruled(_))).then_some(index)
            })
            .collect();
        assert_eq!(ruled.len(), 2);
        assert_ne!(
            groups[ruled[0]], groups[ruled[1]],
            "a real corner between curved spans must remain a selectable edge"
        );
    }
}
