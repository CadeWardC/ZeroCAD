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
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Line};
use openrcad_topo::arena::{
    BRep, EdgeData, EdgeId, FaceData, FaceId, LoopData, OrientedEdge, ShellData, VertexId,
};
use openrcad_topo::{BRepBuilder, Orientation, Shell, Solid};

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

/// Merge adjacent cocylindrical faces (same axis frame, radius, and orientation)
/// of `solid` into one face by dropping the shared seam edge(s).
///
/// A boolean Cut of a vertical cylinder out of a box corner leaves the concave
/// wall split into two faces wherever the cut arc crosses a `make_cylinder` rim
/// seam (the lateral wall is built in thirds at 0/120/240°). Re-uniting them gives
/// one selectable face and lets the fillet's flush-trim corner path see a single
/// cut cap instead of bailing on its two-cap guard and deforming the cut.
///
/// Returns the merged solid only when it is watertight, healthy, and has strictly
/// fewer faces; otherwise returns `solid` unchanged — so it can only ever improve a
/// boolean result.
pub fn merge_cocylindrical_faces(solid: &Solid) -> Solid {
    let mut brep = (**solid.brep()).clone();
    let mut face_ids: Vec<FaceId> = solid.shell().faces().iter().map(|f| f.id()).collect();
    let original_count = face_ids.len();

    do_merge_cocylindrical(&mut brep, &mut face_ids);

    if face_ids.len() >= original_count {
        return solid.clone();
    }

    // Re-use the collinear-line cleanup so any straight side edges the imprint left
    // split are restored to whole edges (arc rims are left as-is).
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

/// Heal T-junctions: split any edge at a vertex that lies in its interior.
///
/// A boolean keeps each face's boundary edges separate (a box's top face and its
/// side face own *coincident but distinct* edges along their shared rim). When an
/// imprint splits one of them at a new vertex but not its coincident twin — e.g. a
/// boss footprint splits the box-top edge at y=6,14 but the perpendicular box-side
/// edge only at y=14 — the two edges no longer share endpoints and the shell opens
/// (free edges). This is the production-kernel "imprint vertices onto coincident
/// edges" heal: wherever an existing vertex sits on another edge's interior, split
/// that edge **at that vertex** so the two become topologically shared.
///
/// Safety-gated like the merges: returned only when it makes the solid watertight
/// and healthy; otherwise the input is returned unchanged, so it can only help.
pub fn heal_tjunctions(solid: &Solid, tol: f64) -> Solid {
    // Only relevant when the shell is open; a watertight solid has no T-junctions
    // to heal, and re-running the scan would be wasted work on every boolean.
    if solid.is_watertight() {
        return solid.clone();
    }

    let face_ids: Vec<FaceId> = solid.shell().faces().iter().map(|f| f.id()).collect();
    let mut builder = BRepBuilder::from_brep((**solid.brep()).clone());

    // Repeatedly split the first edge that has an interior coincident vertex.
    // Each split turns that vertex into a shared endpoint of the two sub-edges, so
    // it can never be re-selected for them — the loop is bounded by edges×vertices,
    // capped here as a hard backstop.
    let max_splits = builder.brep().edges.len() * 4 + 16;
    let mut splits = 0;
    'outer: loop {
        if splits >= max_splits {
            break;
        }
        let edge_ids: Vec<EdgeId> = builder.brep().edges.keys().collect();
        for e_id in edge_ids {
            let Some(e) = builder.brep().edges.get(e_id).cloned() else {
                continue;
            };
            let Some(curve) = e.curve.clone() else {
                continue;
            };
            let sp = builder.brep().vertices[e.start].point;
            let ep = builder.brep().vertices[e.end].point;
            let (lo, hi) = if e.first <= e.last {
                (e.first, e.last)
            } else {
                (e.last, e.first)
            };

            let mut chosen = None;
            for (vid, vd) in builder.brep().vertices.iter() {
                if vid == e.start || vid == e.end {
                    continue;
                }
                let vp = vd.point;
                if vp.distance(&sp) <= tol || vp.distance(&ep) <= tol {
                    continue;
                }
                let t = crate::boolean::project_point_on_curve(&vp, &curve, e.first, e.last);
                if t <= lo + tol || t >= hi - tol {
                    continue;
                }
                if curve.point(t).distance(&vp) > tol {
                    continue;
                }
                chosen = Some((vid, t));
                break;
            }

            if let Some((vid, t)) = chosen {
                builder.split_edge(e_id, vid, t);
                splits += 1;
                continue 'outer; // edges changed — restart the scan
            }
        }
        break; // no edge had an interior vertex
    }

    if splits == 0 {
        return solid.clone();
    }

    builder.brep_mut().retain_faces(&face_ids);
    let shell_id = builder
        .brep_mut()
        .shells
        .insert(ShellData { faces: face_ids });
    let healed = Solid::new(Shell::from_id(builder.build(), shell_id));

    if healed.is_watertight() && healed.health_report().is_healthy() {
        healed
    } else {
        solid.clone()
    }
}

/// Group cylinder faces by axis frame + radius + orientation, merge each group,
/// and rewrite `face_ids` with the merged faces (other faces pass through).
fn do_merge_cocylindrical(brep: &mut BRep, face_ids: &mut Vec<FaceId>) {
    let qf = |x: f64| (x * 1.0e6).round() as i64;
    // (axis location, axis direction, radius, orientation side).
    type CylKey = (i64, i64, i64, i64, i64, i64, i64, u8);
    let mut groups: HashMap<CylKey, Vec<FaceId>> = HashMap::new();
    let mut passthrough: Vec<FaceId> = Vec::new();

    for &fid in face_ids.iter() {
        let Some(fd) = brep.faces.get(fid) else {
            continue;
        };
        let Some(GeomSurface::Cylinder(cyl)) = &fd.surface else {
            passthrough.push(fid);
            continue;
        };
        let loc = cyl.position().location();
        let dir = cyl.position().direction();
        let side: u8 = if fd.orientation == Orientation::Reversed {
            1
        } else {
            0
        };
        let key = (
            qf(loc.x()),
            qf(loc.y()),
            qf(loc.z()),
            qf(dir.x()),
            qf(dir.y()),
            qf(dir.z()),
            qf(cyl.radius()),
            side,
        );
        groups.entry(key).or_default().push(fid);
    }

    // Process groups in sorted-key order, not HashMap order: `try_merge_cyl_group`
    // allocates new FaceIds, so a random iteration order would renumber the result
    // differently each process run and make the whole boolean non-deterministic.
    let mut groups: Vec<(CylKey, Vec<FaceId>)> = groups.into_iter().collect();
    groups.sort_by_key(|(k, _)| *k);

    let mut result = passthrough;
    for (_key, members) in groups {
        if members.len() < 2 {
            result.extend(members);
            continue;
        }
        match try_merge_cyl_group(brep, &members) {
            Some(new_faces) => result.extend(new_faces),
            None => result.extend(members),
        }
    }
    *face_ids = result;
}

/// Merge one cocylindrical group: cancel the shared seam edge(s), re-trace the
/// boundary into a single loop, and rebuild one face. Returns `None` (keep the
/// originals) for anything but the clean, single-loop, non-periodic case — the
/// safety gate in the caller absorbs anything this conservatively skips.
fn try_merge_cyl_group(brep: &mut BRep, members: &[FaceId]) -> Option<Vec<FaceId>> {
    let rep = brep.faces.get(members[0])?.clone();

    // 1. Count co-edge usage across member loops; an edge used twice is the shared
    //    seam (drop it), used once is a real boundary (keep it).
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
    // Nothing is actually shared between the members -> not adjacent, don't merge.
    if !count.values().any(|&c| c >= 2) {
        return None;
    }
    let boundary: Vec<OrientedEdge> = all.into_iter().filter(|oe| count[&oe.id] == 1).collect();
    if boundary.len() < 3 {
        return None;
    }

    // 2. Re-trace the boundary into closed loops; require exactly one (a partial
    //    cut wall). Holes / multi-loop cases are left to the safety gate.
    let loops = retrace_loops(brep, &boundary)?;
    if loops.len() != 1 {
        return None;
    }
    let outer = loops.into_iter().next()?;

    // A full 360° wrap (e.g. a whole cylinder's three lateral faces) drops all its
    // seams and re-traces into TWO rim loops, so it is already rejected above by the
    // single-loop requirement. A partial cut wall always retraces into exactly one
    // loop with two real side edges (the plane∩cylinder cuts, which are BSplines,
    // not lines) — so no further guard is needed; the watertight/health/fewer-faces
    // gate in the caller is the backstop.

    // Build a single face on the shared cylinder surface.
    let outer_loop = brep.loops.insert(LoopData { edges: outer });
    let fid = brep.faces.insert(FaceData {
        surface: rep.surface.clone(),
        outer_wire: Some(outer_loop),
        inner_wires: Vec::new(),
        orientation: rep.orientation,
    });
    Some(vec![fid])
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

    // Sorted-key order, not HashMap order — see the note in `do_merge_cocylindrical`:
    // new FaceIds allocated here must be reproducible across runs.
    let mut groups: Vec<_> = groups.into_iter().collect();
    groups.sort_by_key(|(k, _)| *k);

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
    // Vertices approved as merge candidates whose merge did not actually reduce
    // the edge count (e.g. a degenerate zero-length pair that `apply_collinear_merge`
    // no-ops on). Retiring them guarantees termination: every iteration either
    // removes an edge or permanently blocks one vertex. This is what lets the pick
    // order be deterministic (sorted) without risking an infinite loop — a random
    // HashMap order previously dodged these candidates only by luck.
    let mut blocked: std::collections::HashSet<VertexId> = std::collections::HashSet::new();
    loop {
        let loop_ids = relevant_loops(brep, face_ids);
        // Distinct edges incident to each vertex used by the merged faces.
        let mut vertex_edges: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
        for &lid in &loop_ids {
            let Some(l) = brep.loops.get(lid) else {
                continue;
            };
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

        // Deterministic pick: the smallest VertexId among the mergeable,
        // non-blocked vertices. Pinning the order keeps the surviving edge's
        // identity stable run to run (downstream selection / fillet edge
        // resolution depend on it), instead of varying with the HashMap seed.
        let mut verts: Vec<VertexId> = vertex_edges.keys().copied().collect();
        verts.sort();
        let candidate = verts.into_iter().find_map(|v| {
            if blocked.contains(&v) {
                return None;
            }
            let edges = &vertex_edges[&v];
            if edges.len() != 2 {
                return None;
            }
            let (e1, e2) = (edges[0], edges[1]);
            collinear_merge_endpoints(brep, v, e1, e2).map(|(a, b)| (v, e1, e2, a, b))
        });

        let Some((v, e1, e2, a, b)) = candidate else {
            break;
        };
        let before = brep.edges.len();
        apply_collinear_merge(brep, &loop_ids, v, e1, e2, a, b);
        if brep.edges.len() >= before {
            // No progress (a degenerate candidate `apply_collinear_merge` no-ops
            // on) — retire this vertex so the deterministic pick can't loop on it.
            blocked.insert(v);
        }
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
    if !matches!(ed1.curve, Some(GeomCurve::Line(_)))
        || !matches!(ed2.curve, Some(GeomCurve::Line(_)))
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
        let Some(l) = brep.loops.get(lid) else {
            continue;
        };
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
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi + f64::EPSILON) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax2, Dir};
    use openrcad_primitives::{make_box, make_cylinder};

    /// Count Ø8 (radius-4) vertical-axis cylinder faces.
    fn vertical_cyl_faces(s: &Solid, r: f64) -> usize {
        s.shell()
            .faces()
            .iter()
            .filter(|f| match f.surface() {
                Some(GeomSurface::Cylinder(c)) => {
                    c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - r).abs() < 1e-6
                }
                _ => false,
            })
            .count()
    }

    /// A corner cut whose 180->270° arc crosses a rim seam (here rotated to 90° so a
    /// seam lands at 210°) splits the concave wall into two cocylindrical faces; the
    /// merge re-unites them into one.
    #[test]
    fn corner_cut_crossing_seam_merges_to_one_wall() {
        use core::f64::consts::PI;
        let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let xdir = Dir::new((PI / 2.0).cos(), (PI / 2.0).sin(), 0.0);
        let axis = Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), xdir);
        let cyl = make_cylinder(&axis, 4.0, 12.0);
        let cut = crate::boolean(&cube, &cyl, crate::BooleanOp::Cut);

        assert!(cut.is_watertight() && cut.health_report().is_healthy());
        // Seam-crossing -> the wall starts as two cocylindrical fragments; the merge
        // leaves exactly one.
        assert_eq!(
            vertical_cyl_faces(&cut, 4.0),
            1,
            "the corner-cut wall must be a single cylinder face after the merge"
        );
    }

    /// Negative control: the pass must NOT merge the three lateral faces of a plain
    /// cylinder into a degenerate periodic face (the full-wrap guard rejects it).
    #[test]
    fn plain_cylinder_lateral_faces_are_left_alone() {
        let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 4.0, 10.0);
        let before = cyl.shell().faces().len();
        let after = merge_cocylindrical_faces(&cyl);
        assert_eq!(
            after.shell().faces().len(),
            before,
            "a plain cylinder (full 360° wall) must be left unchanged"
        );
    }
}
