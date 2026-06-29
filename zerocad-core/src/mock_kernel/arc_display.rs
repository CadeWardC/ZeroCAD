use super::*;

/// Maximum central angle (radians) for a single reconstructed arc edge. Longer
/// arcs are split into equal pieces — mirroring `make_cylinder`'s 3×120° wall so
/// each cylindrical lateral face stays a comfortable sub-half-circle the
/// tessellator handles cleanly.
pub(crate) const MAX_ARC_PIECE: f64 = 0.75 * std::f64::consts::PI; // 135°

/// Per-vertex turn beyond which a boundary vertex is a true corner, not a point
/// on a smooth arc. A `CIRCLE_SEGS`-faceted circle turns ~7.5°/vertex and a
/// sketch fillet arc <=3.6°; an octagon turns 45° and a hexagon 60°. ~23°
/// cleanly separates real arcs from polygons the user wants left faceted.
pub(crate) const ARC_MAX_TURN: f64 = 0.40;

/// Minimum count of consecutive co-circular interior vertices for a run to count
/// as an arc, so a stray pair of points can't masquerade as one.
pub(crate) const ARC_MIN_RUN: usize = 3;

pub(crate) fn profile_has_reconstructable_arcs(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
) -> bool {
    loop_has_reconstructable_arcs(points) || holes.iter().any(|h| loop_has_reconstructable_arcs(h))
}

pub(crate) fn loop_has_reconstructable_arcs(loop_pts: &[(f32, f32)]) -> bool {
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
        let (first, last) = (pts[0], *pts.last().unwrap());
        if (first.0 - last.0).hypot(first.1 - last.1) <= 1e-7 {
            pts.pop();
        }
    }

    let n = pts.len();
    if n < 3 {
        return false;
    }

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

    match arc_vert.iter().position(|a| a.is_none()) {
        Some(off) => {
            let ra: Vec<Option<(f64, f64, f64)>> =
                (0..n).map(|i| arc_vert[(i + off) % n]).collect();
            let mut i = 1;
            while i < n {
                let Some(_) = ra[i] else {
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
                    return true;
                }
                i = j + 1;
            }
            false
        }
        None => {
            let avg = {
                let (mut sx, mut sy, mut sr) = (0.0, 0.0, 0.0);
                for c in arc_vert.iter().flatten() {
                    sx += c.0;
                    sy += c.1;
                    sr += c.2;
                }
                (sx / n as f64, sy / n as f64, sr / n as f64)
            };
            arc_vert.iter().flatten().all(|c| same_circle(*c, avg))
        }
    }
}

pub(crate) fn arc_display_targets(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
) -> HashMap<(i64, i64), (f64, f64)> {
    let mut targets = HashMap::new();
    add_arc_display_targets(points, &mut targets);
    for h in holes {
        add_arc_display_targets(h, &mut targets);
    }
    targets
}

pub(crate) fn add_arc_display_targets(
    loop_pts: &[(f32, f32)],
    targets: &mut HashMap<(i64, i64), (f64, f64)>,
) {
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
        let (first, last) = (pts[0], *pts.last().unwrap());
        if (first.0 - last.0).hypot(first.1 - last.1) <= 1e-7 {
            pts.pop();
        }
    }

    let n = pts.len();
    if n < 3 {
        return;
    }

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

    let key = |p: (f64, f64)| -> (i64, i64) {
        (
            (p.0 * 10_000.0).round() as i64,
            (p.1 * 10_000.0).round() as i64,
        )
    };

    match arc_vert.iter().position(|a| a.is_none()) {
        Some(off) => {
            let rp: Vec<(f64, f64)> = (0..n).map(|i| pts[(i + off) % n]).collect();
            let ra: Vec<Option<(f64, f64, f64)>> =
                (0..n).map(|i| arc_vert[(i + off) % n]).collect();
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
                    for k in (cs_v + 1)..ce_v {
                        targets.insert(key(rp[k % n]), (circ.0, circ.1));
                    }
                }
                i = j + 1;
            }
        }
        None => {
            let avg = {
                let (mut sx, mut sy, mut sr) = (0.0, 0.0, 0.0);
                for c in arc_vert.iter().flatten() {
                    sx += c.0;
                    sy += c.1;
                    sr += c.2;
                }
                (sx / n as f64, sy / n as f64, sr / n as f64)
            };
            if arc_vert.iter().flatten().all(|c| same_circle(*c, avg)) {
                for &p in &pts {
                    targets.insert(key(p), (avg.0, avg.1));
                }
            }
        }
    }
}

pub(crate) fn apply_arc_display_targets(
    mesh: &mut MockMesh,
    targets: &HashMap<(i64, i64), (f64, f64)>,
    cs: &crate::geometry::CoordinateSystem,
) {
    if targets.is_empty() {
        return;
    }

    let key = |u: f32, v: f32| -> (i64, i64) {
        (
            (u as f64 * 10_000.0).round() as i64,
            (v as f64 * 10_000.0).round() as i64,
        )
    };
    let circle_candidates: Vec<((f64, f64), f64)> = targets
        .iter()
        .filter_map(|(&(ku, kv), &(cu, cv))| {
            let u = ku as f64 / 10_000.0;
            let v = kv as f64 / 10_000.0;
            let r = (u - cu).hypot(v - cv);
            (r > 1e-9).then_some(((cu, cv), r))
        })
        .collect();

    for vtx in mesh.vertices.chunks_exact_mut(6) {
        let p = Vec3::new(vtx[0], vtx[1], vtx[2]);
        let (u, v) = cs.project(p);
        let center = targets.get(&key(u, v)).copied().or_else(|| {
            circle_candidates.iter().find_map(|&((cu, cv), r)| {
                let d = (u as f64 - cu).hypot(v as f64 - cv);
                ((d - r).abs() <= (0.02 * r).max(0.05)).then_some((cu, cv))
            })
        });
        let Some((cu, cv)) = center else {
            continue;
        };
        let current = Vec3::new(vtx[3], vtx[4], vtx[5]);
        if current.dot(cs.n).abs() > 0.5 {
            continue;
        }
        let du = u as f64 - cu;
        let dv = v as f64 - cv;
        let len = du.hypot(dv);
        if len <= 1e-9 {
            continue;
        }
        let mut radial =
            cs.u.mul((du / len) as f32)
                .add(cs.v.mul((dv / len) as f32))
                .normalize();
        let radial_dot = current.dot(radial);
        if radial_dot.abs() < 0.90 {
            continue;
        }
        if radial_dot < 0.0 {
            radial = radial.mul(-1.0);
        }
        vtx[3] = radial.x;
        vtx[4] = radial.y;
        vtx[5] = radial.z;
    }

    merge_arc_wall_face_ids(mesh, &circle_candidates, cs);
    suppress_arc_internal_struts(mesh, targets, cs);
}

pub(crate) fn merge_arc_wall_face_ids(
    mesh: &mut MockMesh,
    circle_candidates: &[((f64, f64), f64)],
    cs: &crate::geometry::CoordinateSystem,
) {
    if mesh.face_ids.is_empty() || circle_candidates.is_empty() {
        return;
    }

    let pos = |vi: u32, vertices: &[f32]| -> Vec3 {
        let b = vi as usize * 6;
        Vec3::new(vertices[b], vertices[b + 1], vertices[b + 2])
    };
    let normal = |vi: u32, vertices: &[f32]| -> Vec3 {
        let b = vi as usize * 6;
        Vec3::new(vertices[b + 3], vertices[b + 4], vertices[b + 5])
    };
    let on_circle = |p: Vec3, center: (f64, f64), radius: f64| -> bool {
        let (u, v) = cs.project(p);
        let d = (u as f64 - center.0).hypot(v as f64 - center.1);
        (d - radius).abs() <= (0.02 * radius).max(0.05)
    };

    let mut remap: HashMap<(i64, i64, i64), u32> = HashMap::new();
    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let side_wall = tri
            .iter()
            .all(|&vi| normal(vi, &mesh.vertices).dot(cs.n).abs() < 0.5);
        if !side_wall {
            continue;
        }

        let Some(&((cu, cv), radius)) = circle_candidates.iter().find(|&&((cu, cv), r)| {
            tri.iter()
                .all(|&vi| on_circle(pos(vi, &mesh.vertices), (cu, cv), r))
        }) else {
            continue;
        };

        let sig = (
            (cu * 10_000.0).round() as i64,
            (cv * 10_000.0).round() as i64,
            (radius * 10_000.0).round() as i64,
        );
        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
        remap
            .entry(sig)
            .and_modify(|existing| *existing = (*existing).min(fid))
            .or_insert(fid);
    }

    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let side_wall = tri
            .iter()
            .all(|&vi| normal(vi, &mesh.vertices).dot(cs.n).abs() < 0.5);
        if !side_wall {
            continue;
        }

        let Some(&((cu, cv), radius)) = circle_candidates.iter().find(|&&((cu, cv), r)| {
            tri.iter()
                .all(|&vi| on_circle(pos(vi, &mesh.vertices), (cu, cv), r))
        }) else {
            continue;
        };

        let sig = (
            (cu * 10_000.0).round() as i64,
            (cv * 10_000.0).round() as i64,
            (radius * 10_000.0).round() as i64,
        );
        if let Some(&fid) = remap.get(&sig) {
            if let Some(face_id) = mesh.face_ids.get_mut(t) {
                *face_id = fid;
            }
        }
    }
}

pub(crate) fn suppress_arc_internal_struts(
    mesh: &mut MockMesh,
    targets: &HashMap<(i64, i64), (f64, f64)>,
    cs: &crate::geometry::CoordinateSystem,
) {
    let edge_count = mesh.edge_indices.len() / 2;
    if edge_count == 0 {
        return;
    }

    let key = |p: Vec3| -> (i64, i64) {
        let (u, v) = cs.project(p);
        (
            (u as f64 * 10_000.0).round() as i64,
            (v as f64 * 10_000.0).round() as i64,
        )
    };
    let pos = |edge_vertices: &[f32], vi: u32| -> Vec3 {
        let b = vi as usize * 3;
        Vec3::new(edge_vertices[b], edge_vertices[b + 1], edge_vertices[b + 2])
    };

    let old_indices = std::mem::take(&mut mesh.edge_indices);
    let old_normals = std::mem::take(&mut mesh.edge_face_normals);
    let old_groups = std::mem::take(&mut mesh.edge_groups);
    let has_normals = old_normals.len() >= edge_count * 6;
    let has_groups = old_groups.len() == edge_count;

    let mut edge_indices = Vec::with_capacity(old_indices.len());
    let mut edge_face_normals = Vec::with_capacity(old_normals.len());
    let mut edge_groups = Vec::with_capacity(old_groups.len());
    for e in 0..edge_count {
        let ia = old_indices[e * 2];
        let ib = old_indices[e * 2 + 1];
        let a = pos(&mesh.edge_vertices, ia);
        let b = pos(&mesh.edge_vertices, ib);
        let ka = key(a);
        let kb = key(b);
        let d = b.sub(a);
        let axial = d.dot(cs.n).abs();
        let lateral = d.sub(cs.n.mul(d.dot(cs.n))).length();
        let internal_arc_strut =
            ka == kb && targets.contains_key(&ka) && axial > 1e-4 && lateral < 0.05;
        if internal_arc_strut {
            continue;
        }

        edge_indices.push(ia);
        edge_indices.push(ib);
        if has_normals {
            edge_face_normals.extend_from_slice(&old_normals[e * 6..e * 6 + 6]);
        }
        if has_groups {
            edge_groups.push(old_groups[e]);
        }
    }

    mesh.edge_indices = edge_indices;
    mesh.edge_face_normals = if has_normals {
        edge_face_normals
    } else {
        old_normals
    };
    mesh.edge_groups = if has_groups { edge_groups } else { old_groups };
}
