//! Coplanar face merging for boolean results.
//!
//! A boolean keeps faces split along the intersection imprint, so a union of two
//! boxes comes back with the shared top split into several coplanar strips
//! (14 faces instead of 6) — each seam drawn as a spurious wireframe line. This
//! pass merges adjacent faces that lie on the same plane with the same outward
//! side back into one trimmed face.
//!
//! After [`sew`](crate::sew::sew), two coplanar adjacent faces already reference
//! the **same** `EdgeId` along their shared seam, so the merge is edge
//! cancellation: within a coplanar group, an edge used by two faces is interior
//! (drop it); an edge used once is a real boundary (keep it). The kept boundary
//! co-edges are then re-traced into the merged face's outer loop and holes.
//!
//! The pass is wrapped in a hard safety net: if the merged solid is not
//! watertight + healthy, or does not actually reduce the face count, the original
//! solid is returned unchanged — so merging can only ever improve a result.

use std::collections::HashMap;
use std::sync::Arc;

use openrcad_foundation::{Pnt, Vec as FVec};
use openrcad_geom::{GeomCurve, GeomSurface, Line};
use openrcad_topo::arena::{
    BRep, EdgeData, EdgeId, FaceData, FaceId, LoopData, OrientedEdge, ShellData, VertexId,
};
use openrcad_topo::{Orientation, Shell, Solid};

/// A point quantized to a fine integer grid so coincident positions compare equal.
type QPoint = (i64, i64, i64);

fn quantize(p: &Pnt) -> QPoint {
    const GRID: f64 = 1.0e6;
    (
        (p.x() * GRID).round() as i64,
        (p.y() * GRID).round() as i64,
        (p.z() * GRID).round() as i64,
    )
}

/// Merge coplanar adjacent planar faces of `solid` into single trimmed faces.
///
/// Returns a solid with the merge applied when it is watertight, healthy, and has
/// strictly fewer faces; otherwise returns `solid` unchanged.
pub fn merge_coplanar_faces(solid: &Solid) -> Solid {
    let mut brep = (**solid.brep()).clone();
    let mut face_ids: Vec<FaceId> = solid.shell().faces().iter().map(|f| f.id()).collect();
    let original_count = face_ids.len();

    do_merge(&mut brep, &mut face_ids);

    if face_ids.len() >= original_count {
        return solid.clone();
    }

    // Collapse the collinear sub-edges the imprint left along each merged face's
    // boundary (e.g. a [0,10]+[10,20]+[20,30] run becomes one [0,30] edge). This
    // both cleans the topology and restores whole edges so endpoint-based edge
    // selection (used for fillet/chamfer) can find them on a boolean result.
    merge_collinear_edges(&mut brep, &face_ids);

    brep.retain_faces(&face_ids);
    let shell_id = brep.shells.insert(ShellData { faces: face_ids });
    let merged = Solid::new(Shell::from_id(Arc::new(brep), shell_id));

    if merged.is_watertight() && merged.health_report().is_healthy() {
        merged
    } else {
        solid.clone()
    }
}

/// Group planar faces by support plane + outward side, merge each group, and
/// rewrite `face_ids` with the merged faces (curved/lone faces pass through).
fn do_merge(brep: &mut BRep, face_ids: &mut Vec<FaceId>) {
    // Key a face by its effective outward normal and signed plane offset.
    let qf = |x: f64| (x * 1.0e6).round() as i64;
    let mut groups: HashMap<(i64, i64, i64, i64), Vec<FaceId>> = HashMap::new();
    let mut passthrough: Vec<FaceId> = Vec::new();

    for &fid in face_ids.iter() {
        let Some(fd) = brep.faces.get(fid) else {
            continue;
        };
        let Some(GeomSurface::Plane(plane)) = &fd.surface else {
            passthrough.push(fid);
            continue;
        };
        let mut n = plane.normal();
        if fd.orientation == Orientation::Reversed {
            n = n.reversed();
        }
        let off = FVec::from_dir(n).dot(&(plane.location() - Pnt::origin()));
        let key = (qf(n.x()), qf(n.y()), qf(n.z()), qf(off));
        groups.entry(key).or_default().push(fid);
    }

    let mut result = passthrough;
    for (_key, members) in groups {
        if members.len() < 2 {
            result.extend(members);
            continue;
        }
        match try_merge_group(brep, &members) {
            Some(new_faces) => result.extend(new_faces),
            None => result.extend(members),
        }
    }
    *face_ids = result;
}

/// Collapse runs of collinear straight edges that meet at a degree-2 vertex into
/// single edges, updating every loop that uses them. Repeats until stable.
///
/// Only merges two edges when their shared vertex is used by exactly those two
/// edges (no perpendicular face ends there), both are straight lines, and they
/// continue in the same direction — so a genuine corner is never flattened.
fn merge_collinear_edges(brep: &mut BRep, face_ids: &[FaceId]) {
    loop {
        let loop_ids = relevant_loops(brep, face_ids);
        // Distinct edges incident to each vertex used by the merged faces.
        let mut vertex_edges: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
        for &lid in &loop_ids {
            let Some(l) = brep.loops.get(lid) else { continue };
            for oe in &l.edges {
                let Some(e) = brep.edges.get(oe.id) else {
                    continue;
                };
                for v in [e.start, e.end] {
                    let entry = vertex_edges.entry(v).or_default();
                    if !entry.contains(&oe.id) {
                        entry.push(oe.id);
                    }
                }
            }
        }

        let candidate = vertex_edges.iter().find_map(|(&v, edges)| {
            if edges.len() != 2 {
                return None;
            }
            let (e1, e2) = (edges[0], edges[1]);
            collinear_merge_endpoints(brep, v, e1, e2).map(|(a, b)| (v, e1, e2, a, b))
        });

        let Some((v, e1, e2, a, b)) = candidate else {
            break;
        };
        apply_collinear_merge(brep, &loop_ids, v, e1, e2, a, b);
    }
}

/// All loop ids referenced by `face_ids` (outer + inner wires).
fn relevant_loops(brep: &BRep, face_ids: &[FaceId]) -> Vec<openrcad_topo::arena::LoopId> {
    let mut out = Vec::new();
    for &fid in face_ids {
        let Some(fd) = brep.faces.get(fid) else {
            continue;
        };
        if let Some(w) = fd.outer_wire {
            out.push(w);
        }
        out.extend(&fd.inner_wires);
    }
    out
}

/// If edges `e1` and `e2` (meeting at `v`) are straight, collinear, and continue
/// in the same direction, return their far endpoints `(a, b)` — the span of the
/// merged edge. `None` if they form a corner, are curved, or would collapse to a
/// point or an already-existing edge.
fn collinear_merge_endpoints(
    brep: &BRep,
    v: VertexId,
    e1: EdgeId,
    e2: EdgeId,
) -> Option<(VertexId, VertexId)> {
    if e1 == e2 {
        return None;
    }
    let ed1 = brep.edges.get(e1)?;
    let ed2 = brep.edges.get(e2)?;
    // Both must be straight lines.
    if !matches!(ed1.curve, Some(GeomCurve::Line(_))) || !matches!(ed2.curve, Some(GeomCurve::Line(_)))
    {
        return None;
    }
    let a = if ed1.start == v { ed1.end } else { ed1.start };
    let b = if ed2.start == v { ed2.end } else { ed2.start };
    if a == b || a == v || b == v {
        return None;
    }
    let pa = brep.vertices.get(a)?.point;
    let pv = brep.vertices.get(v)?.point;
    let pb = brep.vertices.get(b)?.point;
    let d1 = (pv - pa).normalized()?;
    let d2 = (pb - pv).normalized()?;
    // Same direction (a -> v -> b is a straight run, not a fold-back or corner).
    if d1.dot(&d2) < 1.0 - 1e-9 {
        return None;
    }
    // Don't create a duplicate of an edge that already joins a and b.
    if brep
        .edges
        .values()
        .any(|e| (e.start == a && e.end == b) || (e.start == b && e.end == a))
    {
        return None;
    }
    Some((a, b))
}

/// Replace edges `e1`,`e2` (meeting at `v`) with one straight edge `a -> b`
/// across every relevant loop, then delete `e1`, `e2`, and `v`.
fn apply_collinear_merge(
    brep: &mut BRep,
    loop_ids: &[openrcad_topo::arena::LoopId],
    v: VertexId,
    e1: EdgeId,
    e2: EdgeId,
    a: VertexId,
    b: VertexId,
) {
    let pa = brep.vertices[a].point;
    let pb = brep.vertices[b].point;
    let disp = pb - pa;
    let length = disp.magnitude();
    let Some(dir) = disp.normalized() else {
        return;
    };
    let tol = brep.edges[e1].tolerance.max(brep.edges[e2].tolerance);
    let e_new = brep.edges.insert(EdgeData {
        curve: Some(GeomCurve::line(Line::from_point_dir(pa, dir))),
        first: 0.0,
        last: length,
        start: a,
        end: b,
        tolerance: tol,
    });

    for &lid in loop_ids {
        let Some(l) = brep.loops.get(lid) else { continue };
        let n = l.edges.len();
        // Find the adjacent pair {e1, e2}.
        let mut pair = None;
        for i in 0..n {
            let (cur, nxt) = (l.edges[i].id, l.edges[(i + 1) % n].id);
            if (cur == e1 && nxt == e2) || (cur == e2 && nxt == e1) {
                pair = Some(i);
                break;
            }
        }
        let Some(i) = pair else { continue };
        let j = (i + 1) % n;
        // The merged co-edge runs from the pair's entry vertex to its exit.
        let entry_vid = oriented_start_vertex(brep, &l.edges[i]);
        let oe_new = OrientedEdge {
            id: e_new,
            orientation: if entry_vid == a {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
        };
        let new_edges: Vec<OrientedEdge> = (0..n)
            .filter_map(|k| {
                if k == i {
                    Some(oe_new)
                } else if k == j {
                    None
                } else {
                    Some(l.edges[k])
                }
            })
            .collect();
        if let Some(l) = brep.loops.get_mut(lid) {
            l.edges = new_edges;
        }
    }

    brep.edges.remove(e1);
    brep.edges.remove(e2);
    brep.vertices.remove(v);
}

/// The traversal-start vertex of a co-edge (respecting orientation).
fn oriented_start_vertex(brep: &BRep, oe: &OrientedEdge) -> VertexId {
    let e = &brep.edges[oe.id];
    match oe.orientation {
        Orientation::Reversed => e.end,
        _ => e.start,
    }
}

/// Natural endpoint positions (start, end) of an edge.
fn edge_endpoints(brep: &BRep, id: EdgeId) -> Option<(Pnt, Pnt)> {
    let e = brep.edges.get(id)?;
    let s = brep.vertices.get(e.start)?.point;
    let t = brep.vertices.get(e.end)?.point;
    Some((s, t))
}

/// Oriented (traversal start, traversal end) positions of a co-edge.
fn oriented_points(brep: &BRep, oe: &OrientedEdge) -> Option<(Pnt, Pnt)> {
    let (s, t) = edge_endpoints(brep, oe.id)?;
    Some(match oe.orientation {
        Orientation::Reversed => (t, s),
        _ => (s, t),
    })
}

/// Attempt to merge one coplanar group. Returns the merged face id(s), or `None`
/// if the boundary cannot be cleanly re-traced (caller keeps the originals).
fn try_merge_group(brep: &mut BRep, members: &[FaceId]) -> Option<Vec<FaceId>> {
    let rep = brep.faces.get(members[0])?.clone();
    let GeomSurface::Plane(plane) = rep.surface.as_ref()? else {
        return None;
    };
    let plane = *plane;

    // 1. Count co-edge usage across every member loop; boundary = used once.
    let mut count: HashMap<EdgeId, u32> = HashMap::new();
    let mut all: Vec<OrientedEdge> = Vec::new();
    for &fid in members {
        let fd = brep.faces.get(fid)?;
        let mut wires = Vec::new();
        if let Some(w) = fd.outer_wire {
            wires.push(w);
        }
        wires.extend(&fd.inner_wires);
        for w in wires {
            let l = brep.loops.get(w)?;
            for oe in &l.edges {
                *count.entry(oe.id).or_insert(0) += 1;
                all.push(*oe);
            }
        }
    }
    let boundary: Vec<OrientedEdge> = all.into_iter().filter(|oe| count[&oe.id] == 1).collect();
    if boundary.len() < 3 {
        return None;
    }

    // 2. Re-trace the boundary co-edges into closed loops.
    let loops = retrace_loops(brep, &boundary)?;

    // 3. Project each loop to the plane's UV and classify outer-vs-hole by
    //    containment nesting (frame-independent: a box plane's Ax3 can be
    //    left-handed about its normal, so signed area is unreliable). A loop is
    //    a hole when it sits inside an odd number of the group's other loops.
    //    Co-edge winding is left untouched — it is already correct from the
    //    original faces — so only the outer/hole *role* is decided here.
    let loc = plane.location();
    let xdir = FVec::from_dir(plane.position().x_direction());
    let ydir = FVec::from_dir(plane.position().y_direction());
    let uv = |p: Pnt| -> (f64, f64) {
        let d = p - loc;
        (d.dot(&xdir), d.dot(&ydir))
    };

    struct LoopInfo {
        oedges: Vec<OrientedEdge>,
        poly: Vec<(f64, f64)>,
        outer: bool,
    }
    let mut infos: Vec<LoopInfo> = Vec::with_capacity(loops.len());
    for oedges in loops {
        let mut poly = Vec::with_capacity(oedges.len());
        for oe in &oedges {
            let (s, _) = oriented_points(brep, oe)?;
            poly.push(uv(s));
        }
        infos.push(LoopInfo {
            oedges,
            poly,
            outer: true,
        });
    }
    for i in 0..infos.len() {
        let test = infos[i].poly[0];
        let depth = (0..infos.len())
            .filter(|&j| j != i && point_in_polygon(test, &infos[j].poly))
            .count();
        infos[i].outer = depth % 2 == 0;
    }

    let outer_idx: Vec<usize> = (0..infos.len()).filter(|&i| infos[i].outer).collect();
    let hole_idx: Vec<usize> = (0..infos.len()).filter(|&i| !infos[i].outer).collect();
    if outer_idx.is_empty() {
        return None;
    }

    // 4. Build a face per outer loop, attaching each hole to the outer that
    //    contains it. A hole contained by no outer means a mis-trace -> bail.
    let mut new_faces = Vec::with_capacity(outer_idx.len());
    let mut outer_holes: HashMap<usize, Vec<usize>> = HashMap::new();
    for &hi in &hole_idx {
        let test = infos[hi].poly[0];
        let owner = outer_idx
            .iter()
            .copied()
            .find(|&oi| point_in_polygon(test, &infos[oi].poly));
        match owner {
            Some(oi) => outer_holes.entry(oi).or_default().push(hi),
            None => return None,
        }
    }

    for &oi in &outer_idx {
        let outer_loop = brep.loops.insert(LoopData {
            edges: infos[oi].oedges.clone(),
        });
        let inner_wires: Vec<_> = outer_holes
            .get(&oi)
            .into_iter()
            .flatten()
            .map(|&hi| {
                brep.loops.insert(LoopData {
                    edges: infos[hi].oedges.clone(),
                })
            })
            .collect();
        let fid = brep.faces.insert(FaceData {
            surface: rep.surface.clone(),
            outer_wire: Some(outer_loop),
            inner_wires,
            orientation: rep.orientation,
        });
        new_faces.push(fid);
    }
    Some(new_faces)
}

/// Re-trace a set of boundary co-edges into closed loops by chaining each
/// co-edge's traversal-end to the next's traversal-start (matched by quantized
/// position). Returns `None` if the boundary is not a disjoint union of simple
/// cycles (a vertex with more than one outgoing co-edge, or an open chain).
fn retrace_loops(brep: &BRep, oedges: &[OrientedEdge]) -> Option<Vec<Vec<OrientedEdge>>> {
    let n = oedges.len();
    let mut ends: Vec<(QPoint, QPoint)> = Vec::with_capacity(n);
    let mut by_start: HashMap<QPoint, Vec<usize>> = HashMap::new();
    for (i, oe) in oedges.iter().enumerate() {
        let (s, e) = oriented_points(brep, oe)?;
        let (qs, qe) = (quantize(&s), quantize(&e));
        ends.push((qs, qe));
        by_start.entry(qs).or_default().push(i);
    }
    // Simple-cycle precondition: exactly one outgoing co-edge per used vertex.
    if by_start.values().any(|v| v.len() != 1) {
        return None;
    }

    let mut used = vec![false; n];
    let mut loops = Vec::new();
    for start in 0..n {
        if used[start] {
            continue;
        }
        let mut chain = Vec::new();
        let mut cur = start;
        loop {
            used[cur] = true;
            chain.push(oedges[cur]);
            let (_, cur_end) = ends[cur];
            let next = by_start.get(&cur_end)?.first().copied()?;
            if next == start {
                break;
            }
            if used[next] {
                return None; // chain rejoined a different loop -> not simple
            }
            cur = next;
        }
        loops.push(chain);
    }
    Some(loops)
}

/// Even-odd ray-crossing point-in-polygon test in 2D (UV).
fn point_in_polygon(p: (f64, f64), poly: &[(f64, f64)]) -> bool {
    let (px, py) = p;
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if ((yi > py) != (yj > py))
            && (px < (xj - xi) * (py - yi) / (yj - yi + f64::EPSILON) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}
