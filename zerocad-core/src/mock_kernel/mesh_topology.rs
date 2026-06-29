use super::*;

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
/// Group B-rep faces that lie on the *same* analytic cylinder, returning one
/// group id per face in `solid.shell().faces()` order (which matches the mesh's
/// `face_id`s). The kernel emits a cylindrical wall — a bored hole, a round boss
/// — as 3 arc-faces (thirds), and the straight longitudinal seams between them
/// are a construction artifact, not design edges: drawn, they make a hole read
/// as a notched/faceted circle. Faces sharing a group are recognised as one
/// surface so those seams can be suppressed. Every non-cylinder face (and each
/// distinct cylinder) gets its own id, so only true co-cylindrical faces match.
pub(crate) fn cylinder_surface_groups(solid: &KernelSolid) -> Vec<u32> {
    // Quantized (axis-foot xyz, axis-dir xyz, radius) — a cylinder's identity.
    type CylSig = (i64, i64, i64, i64, i64, i64, i64);
    let q = |v: f64| (v * 1.0e4).round() as i64;
    // A cylinder's identity is its axis *line* + radius, independent of which
    // generator/location names the axis. Canonicalize the axis point to the foot
    // of the perpendicular from the origin, and the direction to a fixed sign.
    let sig = |s: &CylindricalSurface| -> CylSig {
        let p = s.position();
        let d = p.direction();
        let (mut dx, mut dy, mut dz) = (d.x(), d.y(), d.z());
        // Sign-normalize the direction so +axis and -axis hash the same.
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
        let t = loc.x() * dx + loc.y() * dy + loc.z() * dz; // (loc·d)
        let (fx, fy, fz) = (loc.x() - dx * t, loc.y() - dy * t, loc.z() - dz * t);
        (q(fx), q(fy), q(fz), q(dx), q(dy), q(dz), q(s.radius()))
    };

    let mut groups = Vec::with_capacity(solid.shell().faces().len());
    let mut seen: HashMap<CylSig, u32> = HashMap::new();
    let mut next = 0u32;
    for face in solid.shell().faces() {
        let id = match face.surface() {
            Some(GeomSurface::Cylinder(c)) => *seen.entry(sig(c)).or_insert_with(|| {
                let g = next;
                next += 1;
                g
            }),
            // Non-cylinder: a fresh, unshareable id.
            _ => {
                let g = next;
                next += 1;
                g
            }
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

    // Classify each B-rep face as flat or curved by whether its triangles' stored
    // (smoothed) normals vary. A fillet/cylinder face is curved; box and cap faces
    // are flat. Used below so a fillet's *tangent boundary* — where its curved face
    // meets a flat one with nearly-equal normals — is kept as a real edge (the
    // top/bottom line of the round), while a faceted fallback's flat-facet seams
    // (also shallow) stay suppressed.
    let mut face_ref_n: HashMap<u32, [f32; 3]> = HashMap::new();
    let mut face_curved: HashMap<u32, bool> = HashMap::new();
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let r = *face_ref_n
            .entry(fid)
            .or_insert_with(|| nrm(tri[0] as usize));
        for &v in tri {
            let n = nrm(v as usize);
            // ~2.5°: a flat B-rep face's vertices share one (anchored) normal, so
            // it never trips this; a curved face's normals fan out and do.
            const CURVE_COS: f32 = 0.999;
            if r[0] * n[0] + r[1] * n[1] + r[2] * n[2] < CURVE_COS {
                face_curved.insert(fid, true);
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
        // Suppress the *facet-boundary* lines of a curved surface: a crease whose
        // two faces meet at a shallow dihedral (their outward normals nearly
        // agree) is a tessellation seam of a fillet / boolean'd cylinder, not a
        // design edge. Hiding it lets the round read as one smooth face, while
        // genuine edges (box corners at 90°, chamfer bevels at 45°, …) — whose
        // normals differ well past the threshold — still draw. The crease is kept
        // only when the normals diverge by more than ~`CREASE_COS` (≈18°).
        if rec.faces.len() >= 2 {
            let n0 = rec.faces[0].1;
            let n1 = rec.faces[1].1;
            let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0);
            const CREASE_COS: f32 = 0.95; // cos(~18°)
                                          // A curved face (fillet/cylinder) meets its neighbour along a *tangent*
                                          // edge whose normals nearly agree — yet it's a real design edge (the
                                          // top/bottom of a fillet, a cylinder's rim), so any shallow crease that
                                          // touches a curved face is kept. Only a shallow crease between two
                                          // genuinely flat faces is a faceted tessellation seam to hide.
                                          // (A *same-surface* seam — two arc-faces of one cylinder — is handled
                                          // separately above via `surface_group`; here the representative
                                          // per-face normals are too coarse to recognise it.)
            let touches_curved = rec
                .faces
                .iter()
                .any(|(fid, _)| face_curved.get(fid).copied().unwrap_or(false));
            if dot > CREASE_COS && !touches_curved {
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

pub(crate) fn mesh_edge_refs_from_groups(
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
        let n1 = [
            edge_face_normals[fo],
            edge_face_normals[fo + 1],
            edge_face_normals[fo + 2],
        ];
        let n2 = [
            edge_face_normals[fo + 3],
            edge_face_normals[fo + 4],
            edge_face_normals[fo + 5],
        ];
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
