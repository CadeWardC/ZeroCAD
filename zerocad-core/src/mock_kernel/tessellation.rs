use super::*;

/// Tessellation chordal tolerance (in model units / mm). 0.05mm produces a
/// smooth cylinder without explosive triangle counts. Will become a user-facing
/// setting in a later phase.
pub(crate) const TESS_TOL: f64 = 0.05;

/// Tessellation angular tolerance (radians) handed to `openrcad::mesh::tessellate`.
/// ~7.5° (2π/48) — one facet per `CIRCLE_SEGS` slice, so a tessellated cylinder
/// wall matches the analytic rim wireframe's smoothness and a 90° fillet corner
/// gets ~12 facets. The old 0.5 rad (~28.6°) gave a fillet corner only ~3-4
/// facets, so an analytic corner still *rendered* visibly segmented. Angular
/// (not chordal) tolerance keeps the facet density uniform across radii.
pub(crate) const TESS_ANGLE: f64 = 0.13;

/// Build a wire from a closed 2D boundary loop (`cs`-plane coordinates),
/// reconstructing circular arcs from co-circular runs so an extruded sketch arc
/// becomes one smooth cylindrical wall. Returns `None` for fewer than 3 distinct
/// points.
/// Emit the analytic arc covering rotated vertices `[cs_v ..= ce_v]` of `rp` (on
/// circle `circ = (cx, cy, r)` in sketch-plane coords) into `edges`, split into
/// <=`MAX_ARC_PIECE` sub-arcs. Shared by the sample-refit [`loop_to_wire`] and the
/// exact-arc [`loop_to_wire_with_arcs`].
fn emit_arc_edges(
    edges: &mut Vec<Edge>,
    rp: &[(f64, f64)],
    cs_v: usize,
    ce_v: usize,
    circ: (f64, f64, f64),
    cs: &crate::geometry::CoordinateSystem,
) {
    let to_pnt = |p: (f64, f64)| -> Pnt {
        let q = cs.unproject(p.0 as f32, p.1 as f32);
        Pnt::new(q.x as f64, q.y as f64, q.z as f64)
    };
    // The arc sense below is decided from the 2D signed area in (u, v), so the
    // axis reference must be the normal CONSISTENT with that 2D orientation:
    // u × v. The stored `cs.n` can disagree — the ground/top plane constant is
    // left-handed (u=X, v=Z, n=+Y but X×Z = −Y) — and using it flipped every
    // fillet arc into a scallop on ground-plane sketches (the "inverted
    // corners on commit" bug). Same derivation as `build_extrusion_solid`.
    let axis_n = Dir::new(
        (cs.u.y * cs.v.z - cs.u.z * cs.v.y) as f64,
        (cs.u.z * cs.v.x - cs.u.x * cs.v.z) as f64,
        (cs.u.x * cs.v.y - cs.u.y * cs.v.x) as f64,
    );
    let dir3d = |d: (f64, f64)| -> GeomVec {
        GeomVec::new(
            cs.u.x as f64 * d.0 + cs.v.x as f64 * d.1,
            cs.u.y as f64 * d.0 + cs.v.y as f64 * d.1,
            cs.u.z as f64 * d.0 + cs.v.z as f64 * d.1,
        )
    };
    let m = rp.len();
    let (cx, cy, r) = circ;
    let start2d = rp[cs_v % m];
    let end2d = rp[ce_v % m];

    // Sense: signed area of the covered chain about the centre picks which way
    // `Circle::point(t)` (CCW about +main) must turn to trace it.
    let mut signed = 0.0;
    for k in cs_v..ce_v {
        let p = rp[k % m];
        let q = rp[(k + 1) % m];
        signed += (p.0 - cx) * (q.1 - cy) - (p.1 - cy) * (q.0 - cx);
    }
    let main = if signed >= 0.0 {
        axis_n
    } else {
        axis_n.reversed()
    };

    let center3d = to_pnt((cx, cy));
    let xd = dir3d((start2d.0 - cx, start2d.1 - cy));
    let mv = GeomVec::from_dir(main);
    let xperp = xd - mv * xd.dot(&mv);
    let Some(xdir) = Dir::from_vec(&xperp) else {
        // Degenerate (start coincident with centre): fall back to chords.
        for k in cs_v..ce_v {
            edges.push(Edge::between_points(
                to_pnt(rp[k % m]),
                to_pnt(rp[(k + 1) % m]),
            ));
        }
        return;
    };
    let circle = Circle::new(Ax3::new_axes(center3d, main, xdir), r);
    let ydir = mv.cross(&GeomVec::from_dir(xdir));
    let ang = |p2d: (f64, f64)| -> f64 {
        let w = dir3d((p2d.0 - cx, p2d.1 - cy));
        w.dot(&ydir).atan2(w.dot(&GeomVec::from_dir(xdir)))
    };
    let t0 = ang(start2d);
    let mut t1 = ang(end2d);
    while t1 <= t0 + 1e-9 {
        t1 += 2.0 * std::f64::consts::PI;
    }
    let span = t1 - t0;
    let pieces = ((span / MAX_ARC_PIECE).ceil() as usize).max(1);
    let mut prev_v = Vertex::new(to_pnt(start2d));
    for k in 1..=pieces {
        let ts = t0 + span * ((k - 1) as f64 / pieces as f64);
        let te = t0 + span * (k as f64 / pieces as f64);
        let end_v = if k == pieces {
            Vertex::new(to_pnt(end2d))
        } else {
            Vertex::new(circle.point(te))
        };
        edges.push(Edge::new(
            Some(GeomCurve::circle(circle)),
            ts,
            te,
            prev_v.clone(),
            end_v.clone(),
        ));
        prev_v = end_v;
    }
}

/// Like [`loop_to_wire`], but given the EXACT arc circles the boundary contains
/// (from analytic sketch fillet arcs). Each boundary point is classified by which
/// known circle it lies on; a contiguous same-circle run becomes an exact arc edge
/// and everything else a straight edge. Robust for MULTI-arc profiles (rounded
/// rectangles, slots) that [`loop_to_wire`]'s sample refit can't segment — their
/// arc↔line junctions are tangent-continuous, so it finds no sharp-corner
/// separator and emits chords. With no arc circles (or none matching) it delegates
/// to [`loop_to_wire`], so drawn-circle refit is unchanged.
pub(crate) fn loop_to_wire_with_arcs(
    loop_pts: &[(f32, f32)],
    arc_circles: &[((f32, f32), f32)],
    cs: &crate::geometry::CoordinateSystem,
) -> Option<Wire> {
    if arc_circles.is_empty() {
        return loop_to_wire(loop_pts, cs);
    }
    // Dedup coincident consecutive (and wrap-around) points, matching `loop_to_wire`.
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(loop_pts.len());
    for &(u, v) in loop_pts {
        let p = (u as f64, v as f64);
        if pts
            .last()
            .is_none_or(|q: &(f64, f64)| (q.0 - p.0).hypot(q.1 - p.1) > 1e-7)
        {
            pts.push(p);
        }
    }
    if pts.len() >= 2 {
        let (f, l) = (pts[0], *pts.last().unwrap());
        if (f.0 - l.0).hypot(f.1 - l.1) <= 1e-7 {
            pts.pop();
        }
    }
    let n = pts.len();
    if n < 3 {
        return None;
    }

    // Classify each point by the nearest known arc circle it lies on (or None for a
    // straight point). A fillet arc's endpoints lie exactly on its circle; the
    // straight sides between fillets do not (except at the shared endpoints).
    let classify = |p: (f64, f64)| -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for (k, &((cx, cy), r)) in arc_circles.iter().enumerate() {
            let d = ((p.0 - cx as f64).hypot(p.1 - cy as f64) - r as f64).abs();
            let tol = (0.02 * r as f64).max(1.0e-2);
            if d <= tol && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, k));
            }
        }
        best.map(|(_, k)| k)
    };
    let cid: Vec<Option<usize>> = pts.iter().map(|&p| classify(p)).collect();

    // Rotate to a circle-transition point (its class differs from the previous
    // point's) so an arc run never wraps the array end. A rounded rectangle's
    // straight sides are single segments whose two endpoints both lie on
    // (different) fillet circles, so there may be NO unclassified point — but
    // adjacent corners are different circles, so a transition always exists. No
    // transition at all means one circle everywhere (a full circle drawn as arcs)
    // → sample path.
    let Some(off) = (0..n).find(|&i| cid[i] != cid[(i + n - 1) % n]) else {
        return loop_to_wire(loop_pts, cs);
    };
    let rp: Vec<(f64, f64)> = (0..n).map(|i| pts[(i + off) % n]).collect();
    let rc: Vec<Option<usize>> = (0..n).map(|i| cid[(i + off) % n]).collect();

    // Collect arc runs: maximal contiguous same-circle spans of >= 2 points.
    let mut runs: Vec<(usize, usize, (f64, f64, f64))> = Vec::new();
    let mut i = 0;
    while i < n {
        if let Some(k) = rc[i] {
            let mut j = i;
            while j + 1 < n && rc[j + 1] == Some(k) {
                j += 1;
            }
            if j > i {
                let ((cx, cy), r) = arc_circles[k];
                runs.push((i, j, (cx as f64, cy as f64, r as f64)));
                i = j + 1;
                continue;
            }
        }
        i += 1;
    }
    if runs.is_empty() {
        return loop_to_wire(loop_pts, cs);
    }

    let to_pnt = |p: (f64, f64)| -> Pnt {
        let q = cs.unproject(p.0 as f32, p.1 as f32);
        Pnt::new(q.x as f64, q.y as f64, q.z as f64)
    };
    let mut edges: Vec<Edge> = Vec::new();
    let mut cur = 0usize;
    let mut ri = 0usize;
    while cur < n {
        if ri < runs.len() && runs[ri].0 == cur {
            let (a, b, circ) = runs[ri];
            emit_arc_edges(&mut edges, &rp, a, b, circ, cs);
            cur = b;
            ri += 1;
        } else {
            edges.push(Edge::between_points(
                to_pnt(rp[cur % n]),
                to_pnt(rp[(cur + 1) % n]),
            ));
            cur += 1;
        }
    }
    (edges.len() >= 2).then(|| Wire::from_edges(edges))
}

pub(crate) fn loop_to_wire(
    loop_pts: &[(f32, f32)],
    cs: &crate::geometry::CoordinateSystem,
) -> Option<Wire> {
    // Dedup coincident consecutive (and wrap-around) points in 2D.
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(loop_pts.len());
    for &(u, v) in loop_pts {
        let p = (u as f64, v as f64);
        if pts
            .last()
            .is_none_or(|q: &(f64, f64)| (q.0 - p.0).hypot(q.1 - p.1) > 1e-7)
        {
            pts.push(p);
        }
    }
    if pts.len() >= 2 {
        let (f, l) = (pts[0], *pts.last().unwrap());
        if (f.0 - l.0).hypot(f.1 - l.1) <= 1e-7 {
            pts.pop();
        }
    }
    let n = pts.len();
    if n < 3 {
        return None;
    }

    let to_pnt = |p: (f64, f64)| -> Pnt {
        let q = cs.unproject(p.0 as f32, p.1 as f32);
        Pnt::new(q.x as f64, q.y as f64, q.z as f64)
    };

    // Per-vertex co-circularity: the circle through (prev, cur, next), gated so a
    // sharp polygon corner classifies as a non-arc vertex (a run separator).
    let arc_vert: Vec<Option<(f64, f64, f64)>> = (0..n)
        .map(|i| {
            let prev = pts[(i + n - 1) % n];
            let cur = pts[i];
            let next = pts[(i + 1) % n];
            if turn_angle_2d(prev, cur, next) > ARC_MAX_TURN {
                return None;
            }
            circumcircle_2d(prev, cur, next)
        })
        .collect();

    let mut edges: Vec<Edge> = Vec::new();

    // Pick a rotation start at a non-arc (corner) vertex so arc runs never wrap
    // the array boundary. With no corner the loop is a full circle (or an
    // ellipse, whose osculating circles never agree).
    match arc_vert.iter().position(|a| a.is_none()) {
        Some(off) => {
            let rp: Vec<(f64, f64)> = (0..n).map(|i| pts[(i + off) % n]).collect();
            let ra: Vec<Option<(f64, f64, f64)>> =
                (0..n).map(|i| arc_vert[(i + off) % n]).collect();

            // Collect arc runs (covered vertex ranges) over the interior 1..n.
            let mut runs: Vec<(usize, usize, (f64, f64, f64))> = Vec::new();
            let mut i = 1;
            while i < n {
                let Some(ci) = ra[i] else {
                    i += 1;
                    continue;
                };
                let mut j = i;
                while j < n - 1 {
                    match (ra[j], ra[j + 1]) {
                        (Some(a), Some(b)) if same_circle(a, b) => j += 1,
                        _ => break,
                    }
                }
                if j - i + 1 >= ARC_MIN_RUN {
                    let (cs_v, ce_v) = (i - 1, j + 1);
                    let mid = (cs_v + ce_v) / 2;
                    let circ =
                        circumcircle_2d(rp[cs_v % n], rp[mid % n], rp[ce_v % n]).unwrap_or(ci);
                    runs.push((cs_v, ce_v, circ));
                }
                i = j + 1;
            }

            // Walk the loop, emitting arcs for runs and line edges between them.
            let mut cur = 0usize;
            let mut ri = 0usize;
            while cur < n {
                if ri < runs.len() && runs[ri].0 == cur {
                    let (a, b, circ) = runs[ri];
                    emit_arc_edges(&mut edges, &rp, a, b, circ, cs);
                    cur = b;
                    ri += 1;
                } else {
                    edges.push(Edge::between_points(
                        to_pnt(rp[cur % n]),
                        to_pnt(rp[(cur + 1) % n]),
                    ));
                    cur += 1;
                }
            }
        }
        None => {
            // No corners. A full circle has one shared circle; an ellipse does not.
            let avg = {
                let (mut sx, mut sy, mut sr) = (0.0, 0.0, 0.0);
                for c in arc_vert.iter().flatten() {
                    sx += c.0;
                    sy += c.1;
                    sr += c.2;
                }
                (sx / n as f64, sy / n as f64, sr / n as f64)
            };
            let is_circle = arc_vert.iter().flatten().all(|c| same_circle(*c, avg));
            if is_circle {
                let circ = circumcircle_2d(pts[0], pts[n / 3], pts[2 * n / 3]).unwrap_or(avg);
                emit_arc_edges(&mut edges, &pts, 0, n, circ, cs);
            } else {
                for i in 0..n {
                    edges.push(Edge::between_points(
                        to_pnt(pts[i]),
                        to_pnt(pts[(i + 1) % n]),
                    ));
                }
            }
        }
    }

    (edges.len() >= 2).then(|| Wire::from_edges(edges))
}

/// Build a wire directly from the region's sampled polyline. This is the stable
/// path for visible sketch-extrude meshes: cap tessellation sees only straight
/// edges, so mixed rectangle/circle regions cannot produce analytic-arc cap
/// spikes in the preview render.
pub(crate) fn loop_to_polyline_wire(
    loop_pts: &[(f32, f32)],
    cs: &crate::geometry::CoordinateSystem,
) -> Option<Wire> {
    if loop_pts.len() < 3 {
        return None;
    }
    let pts: Vec<Pnt> = loop_pts
        .iter()
        .map(|&(u, v)| {
            let p = cs.unproject(u, v);
            Pnt::new(p.x as f64, p.y as f64, p.z as f64)
        })
        .collect();
    let n = pts.len();
    let edges: Vec<Edge> = (0..n)
        .map(|i| Edge::between_points(pts[i], pts[(i + 1) % n]))
        .collect();
    Some(Wire::from_edges(edges))
}

pub(crate) fn build_extrusion_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f64,
    cs: &crate::geometry::CoordinateSystem,
    reconstruct_arcs: bool,
) -> Option<KernelSolid> {
    build_extrusion_solid_arcs(points, holes, depth, cs, reconstruct_arcs, &[])
}

/// [`build_extrusion_solid`] plus the EXACT arc circles the boundary contains (from
/// analytic sketch fillet arcs). When `reconstruct_arcs`, the wire builder uses
/// them ([`loop_to_wire_with_arcs`]) to sweep each fillet to an exact cylindrical
/// wall — robust for rounded rectangles / multi-arc profiles the sample refit
/// can't segment. `arc_circles` empty ⇒ identical to `build_extrusion_solid`.
pub(crate) fn build_extrusion_solid_arcs(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f64,
    cs: &crate::geometry::CoordinateSystem,
    reconstruct_arcs: bool,
    arc_circles: &[((f32, f32), f32)],
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f64::EPSILON {
        return None;
    }

    let to_pnt = |u: f32, v: f32| -> Pnt {
        let p = cs.unproject(u, v);
        Pnt::new(p.x as f64, p.y as f64, p.z as f64)
    };
    // Boolean solids can reconstruct circular arcs so swept circle fragments
    // become smooth cylindrical walls. Visible sketch-extrude meshes keep the
    // sampled polyline: OpenRCAD's cap tessellator is more reliable on compound
    // circle/line regions when the cap boundary contains only straight chords.
    let make_wire = |loop_pts: &[(f32, f32)]| -> Option<Wire> {
        if reconstruct_arcs {
            loop_to_wire_with_arcs(loop_pts, arc_circles, cs)
        } else {
            loop_to_polyline_wire(loop_pts, cs)
        }
    };

    // A planar face on the sketch frame: outer boundary plus holes as inner
    // wires. `prism` orients its caps from the face's declared plane normal vs.
    // the sweep direction, so that normal must agree with the outer loop's actual
    // winding — otherwise the shell comes out with mixed (some inward) face
    // normals. ZeroCAD's XZ/YZ sketch frames are left-handed (u × v = −n), so a
    // CCW-in-(u,v) loop there faces −cs.n, not +cs.n. Derive the plane normal
    // straight from the 3D winding (Newell's method) so it is always consistent,
    // for any frame handedness; the sweep still runs along cs.n·depth, and
    // `prism` reconciles the sign.
    let outer = make_wire(points)?;
    let inners: Vec<Wire> = holes.iter().filter_map(|h| make_wire(h)).collect();
    let pts3: Vec<Pnt> = points.iter().map(|(u, v)| to_pnt(*u, *v)).collect();
    let normal = newell_normal(&pts3)?;
    let plane = GeomSurface::plane(Plane::from_point_normal(pts3[0], normal));
    let face = if inners.is_empty() {
        Face::new(Some(plane), outer)
    } else {
        Face::with_wires(Some(plane), Some(outer), inners, Orientation::Forward)
    };
    let sweep = GeomVec::new(
        cs.n.x as f64 * depth,
        cs.n.y as f64 * depth,
        cs.n.z as f64 * depth,
    );
    prism(&face, sweep).ok()
}

/// Unit normal of a planar 3D loop via Newell's method — robust to the loop's
/// winding and to which axis it spans. `None` for a degenerate (collinear or
/// zero-area) loop.
pub(crate) fn newell_normal(pts: &[Pnt]) -> Option<Dir> {
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0f64, 0.0, 0.0);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        nx += (a.y() - b.y()) * (a.z() + b.z());
        ny += (a.z() - b.z()) * (a.x() + b.x());
        nz += (a.x() - b.x()) * (a.y() + b.y());
    }
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    (len > 1e-12).then(|| Dir::new(nx / len, ny / len, nz / len))
}

// ---------------------------------------------------------------------------
// Tessellation → flat interleaved vertex buffer
// ---------------------------------------------------------------------------

pub(crate) fn solid_to_flat_mesh(
    solid: &KernelSolid,
    correct_boolean_bevels: bool,
    correct_mixed_triangle_normals: bool,
) -> (Vec<f32>, Vec<u32>, Vec<u32>) {
    // `gpu_mesh` unwelds each triangle into three vertices carrying that
    // triangle's flat face normal, plus a per-triangle source-face id — exactly
    // the interleaved layout (minus the f32 normal smoothing) we want. Each
    // vertex copy belongs to a single triangle, so the per-vertex→face mapping
    // `smooth_vertex_normals` relies on holds.
    let mesh = tessellate(solid, TESS_TOL, TESS_ANGLE);
    let gpu = mesh.gpu_mesh();

    let vcount = gpu.positions.len() / 3;
    let mut vertices: Vec<f32> = Vec::with_capacity(vcount * 6);
    for i in 0..vcount {
        let p = i * 3;
        vertices.extend_from_slice(&[
            gpu.positions[p],
            gpu.positions[p + 1],
            gpu.positions[p + 2],
            gpu.normals[p],
            gpu.normals[p + 1],
            gpu.normals[p + 2],
        ]);
    }
    let mut indices = gpu.indices;
    let face_ids = gpu.face_ids;

    // Normalize the shell to outward-facing normals.
    if correct_boolean_bevels {
        // Boolean / fillet results: `sew` aligns the loop *winding* across faces
        // but the stored plane normals can still diverge, so the shell arrives
        // with a MIX of inward/outward faces (a union drops a face, a cut's hole
        // walls turn invisible). A whole-shell or centroid test can't fix a mix —
        // and a centroid test is plain wrong for a hole wall (which legitimately
        // faces the centroid). Orient robustly by triangle adjacency + signed
        // volume instead, then recompute flat normals from the corrected winding.
        orient_mesh_outward(&mut vertices, &mut indices);
    } else {
        // Analytic primitives / sketch prisms: a per-triangle centroid repair
        // (the inverted cap is local and the geometry is convex enough).
        enforce_outward_normals(&mut vertices, &indices, correct_mixed_triangle_normals);
    }

    // Smooth the normals across shallow creases so a curved surface — an
    // analytic fillet cylinder, or a boolean'd / many-sided extruded cylinder
    // wall — shades as ONE smooth face. Sharp features (90° box corners, 45°
    // chamfers) meet past the crease angle and keep distinct normals, so they
    // stay crisp. Crucially this is *face-aware*: a genuinely flat B-rep face is
    // anchored, so its normal survives unbent right up to a tangent fillet line
    // (a fillet is tangent to its neighbours, so plain crease smoothing would
    // otherwise drag the flat face's edge normals into the round and shade the
    // flat face as a slope). Pairs with the renderer's Gouraud (per-vertex)
    // shading and `mesh_feature_edges`' matching crease filter, which hides the
    // facet-boundary lines.
    apply_analytic_cylinder_normals(solid, &mut vertices, &indices, &face_ids);
    smooth_vertex_normals(&mut vertices, &indices, &face_ids, SHADE_CREASE_COS);
    align_normals_to_winding(&mut vertices, &indices);
    apply_analytic_cylinder_normals(solid, &mut vertices, &indices, &face_ids);

    (vertices, indices, face_ids)
}

/// `cos` of the crease angle (~30°) below which adjacent faces are treated as one
/// smooth surface for shading. Above it (chamfer bevels at 45°, box corners at
/// 90°) the crease is a real edge and the faces keep independent normals.
pub(crate) const SHADE_CREASE_COS: f32 = 0.866;

/// Replace each *curved-face* vertex normal with the average of the normals of
/// all vertices sharing its position whose normal lies within the crease angle
/// (`crease_cos`). This is per-vertex normal smoothing with a crease threshold:
/// a fillet cylinder's tessellation normals (a few degrees apart) blend into a
/// smooth gradient, while a sharp edge — whose two faces' normals diverge past
/// the threshold — keeps each face's own normal, so it still reads as an edge.
///
/// It is **face-aware**: a genuinely flat B-rep face (all its triangles share
/// one normal) is *anchored* — its vertices keep their exact face normal even
/// where they sit on a tangent fillet line. Without this anchor, a fillet (which
/// is tangent to its neighbour faces) would bleed its curving normals into the
/// flat face along that line and shade the flat face as a slope. The round's own
/// vertices are still free to average toward the flat normal there, so the
/// junction stays smooth from the fillet side while the flat face stays flat.
///
/// Operates on the interleaved `[x,y,z,nx,ny,nz]` buffer in place; `face_ids`
/// gives the B-rep face of each triangle in `indices`.
pub(crate) fn smooth_vertex_normals(
    vertices: &mut [f32],
    indices: &[u32],
    face_ids: &[u32],
    crease_cos: f32,
) {
    let vcount = vertices.len() / 6;
    if vcount == 0 {
        return;
    }
    // Weld vertices by quantized position (1e-4 mm, as elsewhere).
    let key = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    let nrm = |i: usize| -> [f32; 3] {
        let b = i * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    // Map each vertex to its B-rep face (a vertex copy is only referenced by
    // triangles of the one face that appended it — see `solid_to_flat_mesh`),
    // then collect each face's vertices.
    let mut vert_face: Vec<Option<u32>> = vec![None; vcount];
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        for &vi in tri {
            vert_face[vi as usize] = Some(fid);
        }
    }
    let mut face_verts: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, f) in vert_face.iter().enumerate() {
        if let Some(f) = f {
            face_verts.entry(*f).or_default().push(i);
        }
    }
    // A face is flat when all its vertices' normals agree (within the crease
    // angle of the face's first normal). Such faces are anchored: a flat design
    // face stays flat; a faceted-fallback fillet's individual flat facets also
    // anchor (so that path keeps its old per-facet look), while a true analytic
    // fillet cylinder — whose normals genuinely vary — is left smoothable.
    let mut flat_face: HashMap<u32, bool> = HashMap::new();
    for (f, verts) in &face_verts {
        let n0 = nrm(verts[0]);
        let flat = verts.iter().all(|&i| {
            let n = nrm(i);
            (n0[0] * n[0] + n0[1] * n[1] + n0[2] * n[2]) >= crease_cos
        });
        flat_face.insert(*f, flat);
    }

    let mut groups: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for i in 0..vcount {
        groups.entry(key(i)).or_default().push(i);
    }
    let mut smoothed = vec![[0.0f32; 3]; vcount];
    for members in groups.values() {
        for &i in members {
            let ni = nrm(i);
            // Anchor: a vertex on a flat face keeps its exact normal.
            let anchored = vert_face[i]
                .and_then(|f| flat_face.get(&f).copied())
                .unwrap_or(false);
            if anchored {
                smoothed[i] = ni;
                continue;
            }
            let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
            for &j in members {
                let nj = nrm(j);
                let same_face = vert_face[i].is_some() && vert_face[i] == vert_face[j];
                let required_dot = if same_face { crease_cos } else { 0.995 };
                if ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2] >= required_dot {
                    sx += nj[0];
                    sy += nj[1];
                    sz += nj[2];
                }
            }
            let len = (sx * sx + sy * sy + sz * sz).sqrt();
            smoothed[i] = if len > 1.0e-6 {
                [sx / len, sy / len, sz / len]
            } else {
                ni
            };
        }
    }
    for (i, n) in smoothed.iter().enumerate() {
        let b = i * 6;
        vertices[b + 3] = n[0];
        vertices[b + 4] = n[1];
        vertices[b + 5] = n[2];
    }
}

/// Flip triangle normals in `vertices` (interleaved pos+normal) when they point
/// inward, judged against the direction from the mesh centroid to that triangle.
/// Robustly orient a tessellated **closed** mesh so every triangle winds (and is
/// normalled) outward — correct even for non-convex / holed solids where a
/// centroid test fails. Boolean results arrive orientation-inconsistent (`sew`
/// aligns winding but stored plane normals diverge), which a whole-shell flip
/// can't repair: it leaves a union missing a face and a cut's hole walls
/// back-facing (so the hole — and the whole cut — looks like it never happened).
///
/// Two passes: (1) flood-fill the triangles, flipping any neighbour that
/// traverses a shared edge the *same* way (a consistently-oriented manifold
/// traverses every shared edge in opposite directions); (2) flip the entire
/// shell if its signed volume came out negative (inside-out). Finally each
/// triangle's flat normal is recomputed from its corrected winding so normal and
/// winding always agree. `gpu_mesh` unwelds every triangle (3 private vertices),
/// so flipping one never disturbs a neighbour.
pub(crate) fn orient_mesh_outward(vertices: &mut [f32], indices: &mut [u32]) {
    let tris = indices.len() / 3;
    if tris == 0 {
        return;
    }
    let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
    let vkey = |vi: u32| -> (i64, i64, i64) {
        let b = vi as usize * 6;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    // Per-triangle vertex keys in stored winding order.
    let tkeys: Vec<[(i64, i64, i64); 3]> = (0..tris)
        .map(|t| {
            [
                vkey(indices[t * 3]),
                vkey(indices[t * 3 + 1]),
                vkey(indices[t * 3 + 2]),
            ]
        })
        .collect();
    // Undirected edge -> incident triangles.
    let mut edge_tris: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
    for (t, k) in tkeys.iter().enumerate() {
        for ei in 0..3 {
            let (a, b) = (k[ei], k[(ei + 1) % 3]);
            edge_tris
                .entry(if a <= b { (a, b) } else { (b, a) })
                .or_default()
                .push(t);
        }
    }
    // The three directed edges of a triangle given its current flip state.
    let directed = |t: usize, flip: bool| -> [((i64, i64, i64), (i64, i64, i64)); 3] {
        let k = &tkeys[t];
        let o = if flip { [0usize, 2, 1] } else { [0, 1, 2] };
        let v = [k[o[0]], k[o[1]], k[o[2]]];
        [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])]
    };
    let mut flipped = vec![false; tris];
    let mut visited = vec![false; tris];
    // Component id per triangle. A join can leave a body as several disjoint
    // watertight shells (a box with a smooth-cylinder boss fused on top often
    // tessellates as two components that only touch). Winding consistency
    // propagates only WITHIN a component, so the inside/outside sign must be
    // decided per component — a single global flip would orient the larger shell
    // correctly and leave the smaller one inside-out (its faces then back-face
    // cull and "disappear" on screen).
    let mut comp = vec![usize::MAX; tris];
    let mut ncomp = 0usize;
    for seed in 0..tris {
        if visited[seed] {
            continue;
        }
        visited[seed] = true;
        comp[seed] = ncomp;
        let mut queue = std::collections::VecDeque::from([seed]);
        while let Some(t) = queue.pop_front() {
            for &(ta, tb) in &directed(t, flipped[t]) {
                let u = if ta <= tb { (ta, tb) } else { (tb, ta) };
                let Some(adj) = edge_tris.get(&u) else {
                    continue;
                };
                for &t2 in adj {
                    if t2 == t || visited[t2] {
                        continue;
                    }
                    // Consistent ⇔ t2 traverses this edge OPPOSITE to t. If t2
                    // (unflipped) traverses it the SAME way (ta→tb), flip it.
                    flipped[t2] = directed(t2, false).contains(&(ta, tb));
                    visited[t2] = true;
                    comp[t2] = ncomp;
                    queue.push_back(t2);
                }
            }
        }
        ncomp += 1;
    }
    // Signed volume (×6) per component with flips applied; negative ⇒ that shell is
    // inside-out and its whole component must be flipped.
    let pos = |vi: u32| -> [f64; 3] {
        let b = vi as usize * 6;
        [
            vertices[b] as f64,
            vertices[b + 1] as f64,
            vertices[b + 2] as f64,
        ]
    };
    let mut comp_vol = vec![0.0f64; ncomp];
    for t in 0..tris {
        let (i0, i1, i2) = if flipped[t] {
            (indices[t * 3], indices[t * 3 + 2], indices[t * 3 + 1])
        } else {
            (indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2])
        };
        let (a, b, c) = (pos(i0), pos(i1), pos(i2));
        comp_vol[comp[t]] += a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    // Apply winding flips, then recompute each triangle's flat normal so the
    // normal agrees with the corrected winding (discarding any inconsistent
    // stored normal). Smoothing later blends genuinely-curved faces.
    for t in 0..tris {
        if flipped[t] ^ (comp_vol[comp[t]] < 0.0) {
            indices.swap(t * 3 + 1, t * 3 + 2);
        }
        let (i0, i1, i2) = (
            indices[t * 3] as usize,
            indices[t * 3 + 1] as usize,
            indices[t * 3 + 2] as usize,
        );
        let p = |i: usize| [vertices[i * 6], vertices[i * 6 + 1], vertices[i * 6 + 2]];
        let (a, b, c) = (p(i0), p(i1), p(i2));
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let nn = if len > 1.0e-12 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [
                vertices[i0 * 6 + 3],
                vertices[i0 * 6 + 4],
                vertices[i0 * 6 + 5],
            ]
        };
        for &vi in &[i0, i1, i2] {
            vertices[vi * 6 + 3] = nn[0];
            vertices[vi * 6 + 4] = nn[1];
            vertices[vi * 6 + 5] = nn[2];
        }
    }
}

pub(crate) fn align_normals_to_winding(vertices: &mut [f32], indices: &[u32]) {
    for tri in indices.chunks_exact(3) {
        let p = |i: u32| {
            let b = i as usize * 6;
            [
                vertices[b] as f64,
                vertices[b + 1] as f64,
                vertices[b + 2] as f64,
            ]
        };
        let n = |i: u32| {
            let b = i as usize * 6;
            [
                vertices[b + 3] as f64,
                vertices[b + 4] as f64,
                vertices[b + 5] as f64,
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
        if winding[0] * navg[0] + winding[1] * navg[1] + winding[2] * navg[2] < 0.0 {
            for &vi in tri {
                let b = vi as usize * 6;
                vertices[b + 3] = -vertices[b + 3];
                vertices[b + 4] = -vertices[b + 4];
                vertices[b + 5] = -vertices[b + 5];
            }
        }
    }
}

pub(crate) fn enforce_outward_normals(vertices: &mut [f32], indices: &[u32], per_triangle: bool) {
    let vcount = vertices.len() / 6;
    if vcount == 0 || indices.is_empty() {
        return;
    }

    let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
    for v in 0..vcount {
        cx += vertices[v * 6];
        cy += vertices[v * 6 + 1];
        cz += vertices[v * 6 + 2];
    }
    let inv = 1.0 / vcount as f32;
    let (cx, cy, cz) = (cx * inv, cy * inv, cz * inv);

    let flip_triangle = |vertices: &mut [f32], tri: &[u32]| {
        for &vi in tri {
            let b = vi as usize * 6;
            vertices[b + 3] = -vertices[b + 3];
            vertices[b + 4] = -vertices[b + 4];
            vertices[b + 5] = -vertices[b + 5];
        }
    };

    let mut orient = 0.0f32;
    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize * 6;
        let i1 = tri[1] as usize * 6;
        let i2 = tri[2] as usize * 6;
        let tcx = (vertices[i0] + vertices[i1] + vertices[i2]) / 3.0 - cx;
        let tcy = (vertices[i0 + 1] + vertices[i1 + 1] + vertices[i2 + 1]) / 3.0 - cy;
        let tcz = (vertices[i0 + 2] + vertices[i1 + 2] + vertices[i2 + 2]) / 3.0 - cz;
        let dot = vertices[i0 + 3] * tcx + vertices[i0 + 4] * tcy + vertices[i0 + 5] * tcz;
        if per_triangle {
            if dot < 0.0 {
                flip_triangle(vertices, tri);
            }
        } else {
            orient += dot;
        }
    }

    if !per_triangle && orient < 0.0 {
        for v in 0..vcount {
            vertices[v * 6 + 3] = -vertices[v * 6 + 3];
            vertices[v * 6 + 4] = -vertices[v * 6 + 4];
            vertices[v * 6 + 5] = -vertices[v * 6 + 5];
        }
    }
}

pub(crate) fn apply_analytic_cylinder_normals(
    solid: &KernelSolid,
    vertices: &mut [f32],
    indices: &[u32],
    face_ids: &[u32],
) {
    let faces = solid.shell().faces();
    let cylinders: Vec<Option<CylindricalSurface>> = faces
        .iter()
        .map(|face| match face.surface() {
            Some(GeomSurface::Cylinder(cyl)) => Some(*cyl),
            _ => None,
        })
        .collect();

    let current_normal = |vertices: &[f32], vi: u32| -> [f32; 3] {
        let b = vi as usize * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    let radial_normal = |vertices: &[f32], vi: u32, cyl: &CylindricalSurface| -> Option<[f32; 3]> {
        let b = vi as usize * 6;
        let p = Pnt::new(
            vertices[b] as f64,
            vertices[b + 1] as f64,
            vertices[b + 2] as f64,
        );
        let axis = GeomVec::from_dir(cyl.position().direction());
        let from_axis = p - cyl.position().location();
        let radial = from_axis.subtracted(&axis.multiplied(from_axis.dot(&axis)));
        let len = radial.magnitude();
        if len <= 1.0e-12 {
            return None;
        }
        Some([
            (radial.x() / len) as f32,
            (radial.y() / len) as f32,
            (radial.z() / len) as f32,
        ])
    };

    let mut face_signs = vec![0.0f32; cylinders.len()];
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0) as usize;
        let Some(Some(cyl)) = cylinders.get(fid) else {
            continue;
        };
        if face_signs[fid] != 0.0 {
            continue;
        }

        for &vi in tri {
            let Some(n) = radial_normal(vertices, vi, cyl) else {
                continue;
            };
            let reference = current_normal(vertices, vi);
            let dot = n[0] * reference[0] + n[1] * reference[1] + n[2] * reference[2];
            if dot.abs() > 1.0e-6 {
                face_signs[fid] = if dot < 0.0 { -1.0 } else { 1.0 };
                break;
            }
        }
    }
    for sign in &mut face_signs {
        if *sign == 0.0 {
            *sign = 1.0;
        }
    }

    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0) as usize;
        let Some(Some(cyl)) = cylinders.get(fid) else {
            continue;
        };
        let sign = face_signs.get(fid).copied().unwrap_or(1.0);

        for &vi in tri {
            let Some(n) = radial_normal(vertices, vi, cyl) else {
                continue;
            };
            let b = vi as usize * 6;
            vertices[b + 3] = n[0] * sign;
            vertices[b + 4] = n[1] * sign;
            vertices[b + 5] = n[2] * sign;
        }
    }
}

pub(crate) fn add_missing_straight_brep_edges(
    solid: &KernelSolid,
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
    surface_group: &[u32],
    edge_vertices: &mut Vec<f32>,
    edge_indices: &mut Vec<u32>,
    edge_face_normals: &mut Vec<f32>,
    edge_pairs: &mut Vec<(u32, u32)>,
) {
    let edge_key_from_points = |a: [f32; 3], b: [f32; 3]| {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        let ka = (q(a[0]), q(a[1]), q(a[2]));
        let kb = (q(b[0]), q(b[1]), q(b[2]));
        if ka <= kb {
            (ka, kb)
        } else {
            (kb, ka)
        }
    };

    let mut existing = HashSet::new();
    for pair in edge_indices.chunks_exact(2) {
        let ia = pair[0] as usize * 3;
        let ib = pair[1] as usize * 3;
        let a = [
            edge_vertices[ia],
            edge_vertices[ia + 1],
            edge_vertices[ia + 2],
        ];
        let b = [
            edge_vertices[ib],
            edge_vertices[ib + 1],
            edge_vertices[ib + 2],
        ];
        existing.insert(edge_key_from_points(a, b));
    }

    let mut face_normal: HashMap<u32, [f32; 3]> = HashMap::new();
    let mut face_count: HashMap<u32, u32> = HashMap::new();
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let b = tri[0] as usize * 6;
        let n = [vertices[b + 3], vertices[b + 4], vertices[b + 5]];
        let sum = face_normal.entry(fid).or_insert([0.0, 0.0, 0.0]);
        sum[0] += n[0];
        sum[1] += n[1];
        sum[2] += n[2];
        *face_count.entry(fid).or_insert(0) += 1;
    }
    for (fid, n) in &mut face_normal {
        let count = face_count.get(fid).copied().unwrap_or(1) as f32;
        n[0] /= count;
        n[1] /= count;
        n[2] /= count;
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 1.0e-6 {
            n[0] /= len;
            n[1] /= len;
            n[2] /= len;
        }
    }

    let mut topo_edges: HashMap<
        ((i64, i64, i64), (i64, i64, i64)),
        ([f32; 3], [f32; 3], Vec<u32>),
    > = HashMap::new();
    for (fid, face) in solid.shell().faces().iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                let a = edge.start().point();
                let b = edge.end().point();
                let pa = [a.x() as f32, a.y() as f32, a.z() as f32];
                let pb = [b.x() as f32, b.y() as f32, b.z() as f32];
                let d2 =
                    (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2);
                if d2 < 1.0e-8 || !edge_is_straight(&edge) {
                    continue;
                }
                let rec = topo_edges
                    .entry(edge_key_from_points(pa, pb))
                    .or_insert_with(|| (pa, pb, Vec::new()));
                if !rec.2.contains(&(fid as u32)) {
                    rec.2.push(fid as u32);
                }
            }
        }
    }

    for (key, (pa, pb, faces)) in topo_edges {
        if existing.contains(&key) || faces.len() < 2 {
            continue;
        }
        // Don't re-introduce a same-surface cylinder seam (see `mesh_feature_edges`):
        // a straight longitudinal edge whose two faces are arc-faces of one cylinder.
        if faces.len() == 2 {
            let g0 = surface_group.get(faces[0] as usize);
            let g1 = surface_group.get(faces[1] as usize);
            if g0.is_some() && g0 == g1 {
                continue;
            }
        }
        let d2 = (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2);
        if d2 < 1.0e-6 {
            continue;
        }
        let a = (edge_vertices.len() / 3) as u32;
        edge_vertices.extend_from_slice(&pa);
        edge_vertices.extend_from_slice(&pb);
        edge_indices.push(a);
        edge_indices.push(a + 1);

        let n0 = faces
            .get(0)
            .and_then(|fid| face_normal.get(fid).copied())
            .unwrap_or([0.0, 0.0, 1.0]);
        let n1 = faces
            .get(1)
            .and_then(|fid| face_normal.get(fid).copied())
            .unwrap_or(n0);
        edge_face_normals.extend_from_slice(&n0);
        edge_face_normals.extend_from_slice(&n1);
        // Each restored straight edge is one B-Rep edge: tag it with its bordering
        // surface pair so it joins the same grouping pass as the feature edges.
        let f0 = faces.first().copied().unwrap_or(0);
        let f1 = faces.get(1).copied().unwrap_or(f0);
        edge_pairs.push(canonical_surface_pair(f0, f1, surface_group));
    }
}

/// Draw each **curved** B-Rep edge (a cylinder rim, a fillet-blend boundary, a
/// drawn arc) as a *smooth* analytic polyline sampled from its exact curve — the
/// resolution-independent rim `build_oriented_cylinder_wireframe` gives an analytic
/// primitive, but derived straight from any boolean/solid B-Rep. Returns the set of
/// canonical surface pairs it drew, so the caller can drop the coarse tessellation
/// chords along those same edges (otherwise a rim renders twice: smooth + faceted).
///
/// The per-sample hidden-line normals are computed **analytically** on the curved
/// side (a cylinder's outward radial at that point, oriented to the face's meshed
/// normal) so the back half of a rim hides just as it does for the primitive path;
/// the planar side keeps its constant face normal.
pub(crate) fn add_analytic_curved_brep_edges(
    solid: &KernelSolid,
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
    surface_group: &[u32],
    edge_vertices: &mut Vec<f32>,
    edge_indices: &mut Vec<u32>,
    edge_face_normals: &mut Vec<f32>,
    edge_pairs: &mut Vec<(u32, u32)>,
) -> HashSet<(u32, u32)> {
    // Average (smoothed) outward normal per B-rep face id, from the triangle mesh —
    // used to orient the analytic radial and as the planar side's constant normal.
    let mut face_normal: HashMap<u32, [f32; 3]> = HashMap::new();
    let mut face_count: HashMap<u32, u32> = HashMap::new();
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let b = tri[0] as usize * 6;
        let n = [vertices[b + 3], vertices[b + 4], vertices[b + 5]];
        let s = face_normal.entry(fid).or_insert([0.0, 0.0, 0.0]);
        s[0] += n[0];
        s[1] += n[1];
        s[2] += n[2];
        *face_count.entry(fid).or_insert(0) += 1;
    }
    for (fid, n) in face_normal.iter_mut() {
        let c = face_count.get(fid).copied().unwrap_or(1) as f32;
        n[0] /= c;
        n[1] /= c;
        n[2] /= c;
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if l > 1.0e-6 {
            n[0] /= l;
            n[1] /= l;
            n[2] /= l;
        }
    }

    let key = |a: [f32; 3], b: [f32; 3]| {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        let ka = (q(a[0]), q(a[1]), q(a[2]));
        let kb = (q(b[0]), q(b[1]), q(b[2]));
        if ka <= kb {
            (ka, kb)
        } else {
            (kb, ka)
        }
    };

    // A curved edge is shared by its two adjacent faces; collect its finely-sampled
    // polyline once (keyed by endpoints so the two faces' copies merge) plus the
    // face ids it borders.
    #[allow(clippy::type_complexity)]
    let mut curved: HashMap<((i64, i64, i64), (i64, i64, i64)), (Vec<[f32; 3]>, Vec<u32>)> =
        HashMap::new();
    for (fid, face) in solid.shell().faces().iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                if edge_is_straight(&edge) {
                    continue;
                }
                let a = edge.start().point();
                let b = edge.end().point();
                let pa = [a.x() as f32, a.y() as f32, a.z() as f32];
                let pb = [b.x() as f32, b.y() as f32, b.z() as f32];
                let rec = curved
                    .entry(key(pa, pb))
                    .or_insert_with(|| (sample_curved_edge_polyline(&edge), Vec::new()));
                if !rec.1.contains(&(fid as u32)) {
                    rec.1.push(fid as u32);
                }
            }
        }
    }

    let mut drawn: HashSet<(u32, u32)> = HashSet::new();
    for (_k, (samples, faces)) in curved {
        if faces.len() < 2 || samples.len() < 2 {
            continue;
        }
        // A same-surface curved seam (should not arise for a rim) is a construction
        // artifact — skip it, mirroring `mesh_feature_edges`.
        if faces.len() == 2 {
            let g0 = surface_group.get(faces[0] as usize);
            let g1 = surface_group.get(faces[1] as usize);
            if g0.is_some() && g0 == g1 {
                continue;
            }
        }
        let f0 = faces[0];
        let f1 = faces[1];
        let pair = canonical_surface_pair(f0, f1, surface_group);
        drawn.insert(pair);
        for w in samples.windows(2) {
            let p = w[0];
            let q = w[1];
            let mid = [
                (p[0] + q[0]) * 0.5,
                (p[1] + q[1]) * 0.5,
                (p[2] + q[2]) * 0.5,
            ];
            let n0 = curved_edge_side_normal(solid, f0, mid, &face_normal);
            let n1 = curved_edge_side_normal(solid, f1, mid, &face_normal);
            let base = (edge_vertices.len() / 3) as u32;
            edge_vertices.extend_from_slice(&p);
            edge_vertices.extend_from_slice(&q);
            edge_indices.push(base);
            edge_indices.push(base + 1);
            edge_face_normals.extend_from_slice(&n0);
            edge_face_normals.extend_from_slice(&n1);
            edge_pairs.push(pair);
        }
    }
    drawn
}

/// Sample a curved edge's exact curve into a smooth polyline (uniform in the
/// curve parameter — for a circle that is uniform in angle). Density matches the
/// primitive rim (`CYL_WIRE_SEGS` per full turn) scaled by the arc's span.
fn sample_curved_edge_polyline(edge: &Edge) -> Vec<[f32; 3]> {
    let Some(curve) = edge.curve() else {
        return Vec::new();
    };
    let a = edge.first();
    let b = edge.last();
    let span = (b - a).abs();
    let full = std::f64::consts::TAU;
    let segs = (((span / full) * CYL_WIRE_SEGS as f64).ceil() as usize).clamp(2, 256);
    (0..=segs)
        .map(|i| {
            let t = a + (b - a) * (i as f64 / segs as f64);
            let p = curve.point(t);
            [p.x() as f32, p.y() as f32, p.z() as f32]
        })
        .collect()
}

/// Outward normal of face `fid` at `point` for hidden-line removal: a cylinder's
/// local radial (oriented to the face's meshed normal, so a bore points inward and
/// a boss outward), otherwise the face's constant meshed normal.
fn curved_edge_side_normal(
    solid: &KernelSolid,
    fid: u32,
    point: [f32; 3],
    face_normal: &HashMap<u32, [f32; 3]>,
) -> [f32; 3] {
    let base = face_normal.get(&fid).copied().unwrap_or([0.0, 0.0, 1.0]);
    let shell = solid.shell();
    let faces = shell.faces();
    let Some(face) = faces.get(fid as usize) else {
        return base;
    };
    if let Some(GeomSurface::Cylinder(cyl)) = face.surface() {
        let pos = cyl.position();
        let o = pos.location();
        let d = pos.direction();
        let (ox, oy, oz) = (o.x() as f32, o.y() as f32, o.z() as f32);
        let (dx, dy, dz) = (d.x() as f32, d.y() as f32, d.z() as f32);
        let rel = [point[0] - ox, point[1] - oy, point[2] - oz];
        let t = rel[0] * dx + rel[1] * dy + rel[2] * dz;
        let radial = [rel[0] - dx * t, rel[1] - dy * t, rel[2] - dz * t];
        let l = (radial[0] * radial[0] + radial[1] * radial[1] + radial[2] * radial[2]).sqrt();
        if l > 1.0e-6 {
            let mut r = [radial[0] / l, radial[1] / l, radial[2] / l];
            if r[0] * base[0] + r[1] * base[1] + r[2] * base[2] < 0.0 {
                r = [-r[0], -r[1], -r[2]];
            }
            return r;
        }
    }
    base
}

pub(crate) fn edge_is_straight(edge: &Edge) -> bool {
    let Some(curve) = edge.curve() else {
        return true;
    };
    let first = edge.first();
    let last = edge.last();
    let mid = 0.5 * (first + last);
    let p0 = curve.point(first);
    let p1 = curve.point(last);
    let pm = curve.point(mid);
    let chord = p1 - p0;
    let len = chord.magnitude();
    if len <= 1.0e-9 {
        return false;
    }
    let along = chord / len;
    let d = pm - p0;
    let closest = p0 + along * d.dot(&along);
    pm.distance(&closest) < 1.0e-4
}

// ---------------------------------------------------------------------------
// Analytical wireframes (unchanged behavior from the prior mock kernel)
// ---------------------------------------------------------------------------
