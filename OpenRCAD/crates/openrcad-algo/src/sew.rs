use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use openrcad_foundation::{tolerance, Pnt, Vec as FVec};
use openrcad_geom::{Curve, GeomSurface, Plane};
use openrcad_topo::arena::{LoopId, OrientedEdge, ShellData};
use openrcad_topo::{BRep, EdgeId, Face, FaceId, Orientation, Shell, VertexId};

/// A point quantized to a fine integer grid (see [`quantize`]).
type QPoint = (i64, i64, i64);

/// Quantize a point to a fine integer grid so coincident endpoints from
/// independently-built edges compare equal (matches `manifold_report`'s grid).
fn quantize(p: &Pnt) -> QPoint {
    const GRID: f64 = 1.0e6;
    (
        (p.x() * GRID).round() as i64,
        (p.y() * GRID).round() as i64,
        (p.z() * GRID).round() as i64,
    )
}

fn loops_for_faces(brep: &BRep, face_ids: &[FaceId]) -> HashSet<LoopId> {
    let mut loops = HashSet::new();
    for &face_id in face_ids {
        let Some(face) = brep.faces.get(face_id) else {
            continue;
        };
        if let Some(outer) = face.outer_wire {
            loops.insert(outer);
        }
        loops.extend(face.inner_wires.iter().copied());
    }
    loops
}

/// Re-order and re-orient a loop's co-edges into a single contiguous chain, so
/// each co-edge's traversal-end meets the next's traversal-start — the defining
/// invariant of a B-Rep loop.
///
/// The split/partition/imprint passes can emit a loop whose edges are all present
/// and form a closed cycle but with one co-edge's orientation flipped, leaving the
/// loop non-contiguous (`LoopNotContiguous`) even though it is geometrically
/// closed. This greedily threads the edges by quantized endpoint position to
/// restore the chain.
///
/// Returns `None` (caller leaves the loop untouched) when the loop is **already
/// contiguous** — so correct loops keep their exact winding — or when the edges
/// cannot be threaded into one closed cycle (a genuine gap or a pinch point),
/// so nothing is silently corrupted. Preserves the first co-edge's orientation,
/// and therefore the loop's winding sense.
fn rethread_loop(brep: &BRep, edges: &[OrientedEdge]) -> Option<Vec<OrientedEdge>> {
    let n = edges.len();
    if n < 3 {
        return None;
    }
    // Natural endpoint positions (start, end) of an edge.
    let endpoints = |oe: &OrientedEdge| -> Option<(QPoint, QPoint)> {
        let e = brep.edges.get(oe.id)?;
        let ps = brep.vertices.get(e.start)?.point;
        let pe = brep.vertices.get(e.end)?.point;
        Some((quantize(&ps), quantize(&pe)))
    };
    // Oriented (traversal start, traversal end) of a co-edge.
    let oriented = |oe: &OrientedEdge| -> Option<(QPoint, QPoint)> {
        let (s, e) = endpoints(oe)?;
        Some(match oe.orientation {
            Orientation::Reversed => (e, s),
            _ => (s, e),
        })
    };

    // Fast path: already a contiguous chain → leave untouched (exact winding).
    let mut contiguous = true;
    for i in 0..n {
        let (_, cur_end) = oriented(&edges[i])?;
        let (next_start, _) = oriented(&edges[(i + 1) % n])?;
        if cur_end != next_start {
            contiguous = false;
            break;
        }
    }
    if contiguous {
        return None;
    }

    // Thread the edges, keeping the first co-edge as the anchor.
    let mut remaining: Vec<OrientedEdge> = edges.to_vec();
    let first = remaining.remove(0);
    let (start_key, mut cur_end) = oriented(&first)?;
    let mut result: Vec<OrientedEdge> = Vec::with_capacity(n);
    result.push(first);

    while !remaining.is_empty() {
        let mut picked = None;
        for (i, oe) in remaining.iter().enumerate() {
            let (es, ee) = match endpoints(oe) {
                Some(v) => v,
                None => continue,
            };
            if es == cur_end {
                picked = Some((
                    i,
                    OrientedEdge {
                        id: oe.id,
                        orientation: Orientation::Forward,
                    },
                    ee,
                ));
                break;
            } else if ee == cur_end {
                picked = Some((
                    i,
                    OrientedEdge {
                        id: oe.id,
                        orientation: Orientation::Reversed,
                    },
                    es,
                ));
                break;
            }
        }
        let (i, oe, next_end) = picked?;
        remaining.remove(i);
        result.push(oe);
        cur_end = next_end;
    }

    // The chain must close back onto the first co-edge's start.
    (cur_end == start_key).then_some(result)
}

/// The traversal-start points of a loop's co-edges, in loop order — the polygon
/// that defines the loop's winding (each co-edge contributes the vertex it leaves
/// from, honouring its orientation flag).
fn loop_traversal_points(brep: &BRep, loop_id: LoopId) -> Vec<Pnt> {
    let mut pts = Vec::new();
    let Some(l) = brep.loops.get(loop_id) else {
        return pts;
    };
    for oe in &l.edges {
        let Some(e) = brep.edges.get(oe.id) else {
            continue;
        };
        let v = match oe.orientation {
            Orientation::Reversed => e.end,
            _ => e.start,
        };
        if let Some(vd) = brep.vertices.get(v) {
            pts.push(vd.point);
        }
    }
    pts
}

/// Newell area-weighted normal of an ordered point polygon, or `None` if the
/// polygon is degenerate (fewer than 3 points or near-zero area).
fn newell_normal(points: &[Pnt]) -> Option<FVec> {
    if points.len() < 3 {
        return None;
    }
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..points.len() {
        let p = points[i];
        let q = points[(i + 1) % points.len()];
        nx += (p.y() - q.y()) * (p.z() + q.z());
        ny += (p.z() - q.z()) * (p.x() + q.x());
        nz += (p.x() - q.x()) * (p.y() + q.y());
    }
    let n = FVec::new(nx, ny, nz);
    (n.magnitude() > tolerance::CONFUSION).then_some(n)
}

/// True if every undirected boundary edge (matched by quantized endpoint
/// position) is used by exactly two face loops — the watertight 2-manifold
/// condition. Open intermediate shells return `false`.
fn shell_is_closed(brep: &BRep, face_ids: &[FaceId]) -> bool {
    let mut counts: HashMap<(QPoint, QPoint), usize> = HashMap::new();
    for &f_id in face_ids {
        let Some(f) = brep.faces.get(f_id) else {
            continue;
        };
        let mut wires = Vec::new();
        if let Some(w) = f.outer_wire {
            wires.push(w);
        }
        wires.extend(&f.inner_wires);
        for w_id in wires {
            let Some(l) = brep.loops.get(w_id) else {
                continue;
            };
            for oe in &l.edges {
                let Some(e) = brep.edges.get(oe.id) else {
                    continue;
                };
                let (Some(a), Some(b)) = (brep.vertices.get(e.start), brep.vertices.get(e.end))
                else {
                    continue;
                };
                let (qa, qb) = (quantize(&a.point), quantize(&b.point));
                let key = if qa <= qb { (qa, qb) } else { (qb, qa) };
                *counts.entry(key).or_insert(0) += 1;
            }
        }
    }
    !counts.is_empty() && counts.values().all(|&c| c == 2)
}

/// Signed volume of the shell via the divergence theorem: sum of the signed
/// tetrahedra `(origin, p0, pi, pi+1)` over a fan triangulation of each face's
/// outer loop (in its effective, flag-adjusted winding). Positive when the shell
/// faces outward. Only the sign is used, so the coarse boundary-loop fan (which
/// approximates curved faces by their corner polygon) is sufficient.
fn signed_volume(brep: &BRep, face_ids: &[FaceId]) -> f64 {
    let mut v6 = 0.0;
    for &f_id in face_ids {
        let Some(f) = brep.faces.get(f_id) else {
            continue;
        };
        let Some(outer) = f.outer_wire else {
            continue;
        };
        let mut pts = loop_traversal_points(brep, outer);
        if f.orientation == Orientation::Reversed {
            pts.reverse();
        }
        if pts.len() < 3 {
            continue;
        }
        let a = pts[0];
        for i in 1..pts.len() - 1 {
            let b = pts[i];
            let c = pts[i + 1];
            v6 += a.x() * (b.y() * c.z() - b.z() * c.y()) - a.y() * (b.x() * c.z() - b.z() * c.x())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
        }
    }
    v6 / 6.0
}

/// Canonicalize the sewn shell's orientation so every face's effective normal is
/// reliable and (for a closed shell) points outward.
///
/// Step 7's BFS makes the flag-adjusted loop *windings* mutually consistent, but
/// never reconciles them with each face's stored surface normal, and never
/// orients the shell as a whole — so a watertight result can still carry faces
/// whose effective normal (`orientation × surface.normal()`) disagrees with the
/// winding, or whose shell faces inward. This pass closes both gaps:
///
/// 1. **Per-face normal/winding agreement.** For each planar face, re-point the
///    stored plane normal to agree with its loop winding's Newell normal. A plane
///    normal is independent of the winding and can be flipped without changing the
///    face's geometry, so this only touches the *stored normal*, never the
///    orientation flag — leaving step 7's winding consistency intact. (Curved
///    surfaces carry an intrinsic normal sense and are handled by step 2 alone.)
/// 2. **Global outward pass.** If the shell is closed and its signed volume is
///    negative, flip every face's orientation flag so the shell faces outward.
///    Flipping all flags together preserves both the per-face agreement and the
///    cross-edge winding consistency.
fn canonicalize_shell_orientation(brep: &mut BRep, face_ids: &[FaceId]) {
    // 1. Align each planar face's stored normal with its loop winding.
    for &f_id in face_ids {
        let Some(outer) = brep.faces.get(f_id).and_then(|f| f.outer_wire) else {
            continue;
        };
        let pts = loop_traversal_points(brep, outer);
        let Some(w) = newell_normal(&pts) else {
            continue;
        };
        let plane = match &brep.faces[f_id].surface {
            Some(GeomSurface::Plane(p)) => *p,
            _ => continue,
        };
        let n = plane.normal();
        let dot = n.x() * w.x() + n.y() * w.y() + n.z() * w.z();
        if dot < 0.0 {
            let flipped = Plane::from_point_normal(plane.location(), n.reversed());
            brep.faces[f_id].surface = Some(GeomSurface::plane(flipped));
        }
    }

    // 2. Orient the whole shell outward — only when it is closed (an open
    //    intermediate shell has no well-defined inside).
    if !shell_is_closed(brep, face_ids) {
        return;
    }
    if signed_volume(brep, face_ids) < 0.0 {
        for &f_id in face_ids {
            if let Some(f) = brep.faces.get_mut(f_id) {
                f.orientation = f.orientation.reversed();
            }
        }
    }
}

/// Sew a collection of faces into a single shell, joining edges within `tol`.
///
/// Merges vertices within `tol` by clustering them and updating edge endpoints.
/// Also merges edges that share the same endpoints (in either direction) and have compatible curves.
pub fn sew(faces: &[Face], tol: f64) -> Shell {
    if faces.is_empty() {
        return Shell::default();
    }
    let debug = std::env::var_os("OPENRCAD_SEW_DEBUG").is_some();

    // 1. Merge all faces into a single BRep.
    let mut brep = BRep::new();
    let mut face_ids = Vec::with_capacity(faces.len());
    let mut merged = HashMap::new();

    for face in faces {
        let ptr = Arc::as_ptr(face.brep()) as usize;
        let map = merged.entry(ptr).or_insert_with(|| brep.merge(face.brep()));
        let new_face_id = map.faces[&face.id()];

        // Sync face's orientation in BRep data to match handle
        if let Some(merged_face_data) = brep.faces.get_mut(new_face_id) {
            merged_face_data.orientation = face.orientation();
        }
        face_ids.push(new_face_id);
    }

    // `BRep::merge` copies the full source arena for each face. Count boundaries
    // using only the loops of the selected faces so orphan faces do not make real
    // shell boundaries look already paired while sewing.
    let active_loops = loops_for_faces(&brep, &face_ids);

    // 2. Identify free boundaries (referenced by at most one loop) and collect boundary vertices.
    let mut edge_counts_initial = HashMap::new();
    for &loop_id in &active_loops {
        let Some(l_data) = brep.loops.get(loop_id) else {
            continue;
        };
        for oe in &l_data.edges {
            *edge_counts_initial.entry(oe.id).or_insert(0) += 1;
        }
    }
    let mut boundary_vertices = HashSet::new();
    for (e_id, &count) in &edge_counts_initial {
        if count <= 1 {
            if let Some(e) = brep.edges.get(*e_id) {
                boundary_vertices.insert(e.start);
                boundary_vertices.insert(e.end);
            }
        }
    }

    // Cluster vertices within `tol` using a Disjoint Set (Union-Find) - free boundary focused.
    let vertex_keys: Vec<VertexId> = brep.vertices.keys().collect();
    let n_vertices = vertex_keys.len();
    let mut parent: Vec<usize> = (0..n_vertices).collect();

    fn find(i: usize, parent: &mut [usize]) -> usize {
        let mut root = i;
        while parent[root] != root {
            root = parent[root];
        }
        let mut curr = i;
        while curr != root {
            let nxt = parent[curr];
            parent[curr] = root;
            curr = nxt;
        }
        root
    }

    fn find_no_compress(i: usize, parent: &[usize]) -> usize {
        let mut root = i;
        while parent[root] != root {
            root = parent[root];
        }
        root
    }

    fn union(i: usize, j: usize, parent: &mut [usize]) {
        let root_i = find(i, parent);
        let root_j = find(j, parent);
        if root_i != root_j {
            parent[root_i] = root_j;
        }
    }

    for i in 0..n_vertices {
        let v1_id = vertex_keys[i];
        if !boundary_vertices.contains(&v1_id) {
            continue;
        }
        let p1 = brep.vertices[v1_id].point;
        for j in (i + 1)..n_vertices {
            let v2_id = vertex_keys[j];
            if !boundary_vertices.contains(&v2_id) {
                continue;
            }
            let p2 = brep.vertices[v2_id].point;
            // Tolerant modeling (CLAUDE.md principle #4): two boundary vertices
            // coincide if they are within the *global* sewing tolerance OR within
            // the sum of their own per-entity uncertainty radii. The latter lets
            // "dirty" imported geometry — whose vertices carry large local
            // tolerances — heal even under a tight global `tol`, while clean
            // geometry (CONFUSION-tolerance vertices) is unaffected.
            let t1 = brep.vertices[v1_id].tolerance;
            let t2 = brep.vertices[v2_id].tolerance;
            let merge_tol = tol.max(t1 + t2);
            if p1.distance(&p2) <= merge_tol {
                union(i, j, &mut parent);
            }
        }
    }

    // Map each vertex to its representative vertex and compute weighted averages and tolerance.
    let mut v_map = HashMap::with_capacity(n_vertices);
    let mut cluster_sums = vec![openrcad_foundation::Xyz::new(0.0, 0.0, 0.0); n_vertices];
    let mut cluster_counts = vec![0; n_vertices];

    for i in 0..n_vertices {
        let root = find(i, &mut parent);
        let p = brep.vertices[vertex_keys[i]].point;
        cluster_sums[root] += p.coord();
        cluster_counts[root] += 1;
    }

    // Compute averaged points.
    let mut avg_points = HashMap::new();
    for i in 0..n_vertices {
        let root = find(i, &mut parent);
        if i == root {
            let count = cluster_counts[root] as f64;
            let avg_xyz = cluster_sums[root] / count;
            let avg_pnt = Pnt::from_xyz(avg_xyz);
            avg_points.insert(vertex_keys[root], avg_pnt);
        }
    }

    // Map.
    for i in 0..n_vertices {
        let root = find(i, &mut parent);
        let rep_v_id = vertex_keys[root];
        v_map.insert(vertex_keys[i], rep_v_id);
    }

    // Propagate tolerances to representative vertices.
    let mut max_tols = HashMap::new();
    for i in 0..n_vertices {
        let v_id = vertex_keys[i];
        let rep_id = v_map[&v_id];
        let p = brep.vertices[v_id].point;
        let rep_p = avg_points[&rep_id];
        let dist = p.distance(&rep_p);
        let v_tol = brep.vertices[v_id].tolerance;
        let entry = max_tols.entry(rep_id).or_insert(0.0);
        *entry = f64::max(*entry, v_tol + dist);
    }

    // Apply representative points and tolerances.
    for i in 0..n_vertices {
        let v_id = vertex_keys[i];
        if v_id == v_map[&v_id] {
            if let Some(v_data) = brep.vertices.get_mut(v_id) {
                v_data.point = avg_points[&v_id];
                v_data.tolerance = max_tols[&v_id];
            }
        }
    }

    // Remove non-representative vertices.
    for i in 0..n_vertices {
        let v_id = vertex_keys[i];
        if v_map[&v_id] != v_id {
            brep.vertices.remove(v_id);
        }
    }

    // 3. Update all EdgeData start/end vertex references.
    for (_, e_data) in &mut brep.edges {
        e_data.start = v_map[&e_data.start];
        e_data.end = v_map[&e_data.end];
    }

    // 4. Count edge occurrences across loops to identify free boundary edges.
    let mut edge_counts = HashMap::new();
    for &loop_id in &active_loops {
        let Some(l_data) = brep.loops.get(loop_id) else {
            continue;
        };
        for oe in &l_data.edges {
            *edge_counts.entry(oe.id).or_insert(0) += 1;
        }
    }

    // 5. Merge edges that share endpoints (in either direction) and have compatible curves.
    let edge_keys: Vec<EdgeId> = brep.edges.keys().collect();
    let n_edges = edge_keys.len();
    let mut e_parent: Vec<usize> = (0..n_edges).collect();
    let mut reverse_merge = vec![false; n_edges]; // Track if merged reversed

    for i in 0..n_edges {
        let e1_id = edge_keys[i];
        let e1 = &brep.edges[e1_id];
        let count1 = *edge_counts.entry(e1_id).or_insert(0);

        for j in (i + 1)..n_edges {
            let e2_id = edge_keys[j];
            let e2 = &brep.edges[e2_id];
            let count2 = *edge_counts.entry(e2_id).or_insert(0);

            // Merge if edges are identical, or if both are free boundaries.
            let both_free = count1 <= 1 && count2 <= 1;
            let endpoints_match_same = e1.start == e2.start && e1.end == e2.end;
            let endpoints_match_opp = e1.start == e2.end && e1.end == e2.start;
            if debug && both_free && !(endpoints_match_same || endpoints_match_opp) {
                let a1 = brep.vertices[e1.start].point;
                let b1 = brep.vertices[e1.end].point;
                let a2 = brep.vertices[e2.start].point;
                let b2 = brep.vertices[e2.end].point;
                let same = a1.distance(&a2).max(b1.distance(&b2));
                let opp = a1.distance(&b2).max(b1.distance(&a2));
                let near = same.min(opp);
                if near < 1e-2 {
                    eprintln!(
                        "sew near free endpoints {:?}/{:?} near={near:.8} same={same:.8} opp={opp:.8} tol={tol:.8}",
                        e1_id, e2_id
                    );
                }
            }

            if endpoints_match_same || endpoints_match_opp {
                // If they are not free boundaries, only merge if they are exact duplicates in same direction.
                if !both_free && !endpoints_match_same {
                    continue;
                }

                // Check curve compatibility within tolerance.
                let mut curve_mismatch = None;
                let curves_match = match (&e1.curve, &e2.curve) {
                    (None, None) => true,
                    (Some(c1), Some(c2)) => {
                        // Sample start, mid, end.
                        let t1_mid = 0.5 * (e1.first + e1.last);
                        let t2_mid = 0.5 * (e2.first + e2.last);
                        let p1_start = c1.point(e1.first);
                        let p1_mid = c1.point(t1_mid);
                        let p1_end = c1.point(e1.last);

                        let (p2_start, p2_mid, p2_end) = if endpoints_match_same {
                            (c2.point(e2.first), c2.point(t2_mid), c2.point(e2.last))
                        } else {
                            (c2.point(e2.last), c2.point(t2_mid), c2.point(e2.first))
                        };

                        let d_start = p1_start.distance(&p2_start);
                        let d_mid = p1_mid.distance(&p2_mid);
                        let d_end = p1_end.distance(&p2_end);
                        curve_mismatch = Some((d_start, d_mid, d_end, tol));
                        d_start <= tol && d_mid <= tol && d_end <= tol
                    }
                    _ => false,
                };
                if debug && both_free && !curves_match {
                    if let Some((d_start, d_mid, d_end, curve_tol)) = curve_mismatch {
                        if d_start.min(d_mid).min(d_end) < 1e-2 {
                            eprintln!(
                                "sew free curve mismatch {:?}/{:?} ds={d_start:.8} dm={d_mid:.8} de={d_end:.8} tol={curve_tol:.8}",
                                e1_id, e2_id
                            );
                        }
                    }
                }

                if curves_match {
                    let root_i = find_no_compress(i, &e_parent);
                    let root_j = find_no_compress(j, &e_parent);
                    if root_i != root_j {
                        e_parent[root_i] = root_j;
                        if endpoints_match_opp {
                            reverse_merge[root_i] = !reverse_merge[root_i];
                        }
                    }
                }
            }
        }
    }

    // Map edges to their representative edge and update tolerances.
    let mut e_map = HashMap::with_capacity(n_edges);
    let mut edge_reversals = HashMap::new();
    for i in 0..n_edges {
        let root = find_no_compress(i, &e_parent);
        let e_id = edge_keys[i];
        let rep_id = edge_keys[root];
        e_map.insert(e_id, rep_id);

        // Track orientation mapping.
        let mut is_rev = false;
        let mut curr = i;
        while e_parent[curr] != curr {
            if reverse_merge[curr] {
                is_rev = !is_rev;
            }
            curr = e_parent[curr];
        }
        edge_reversals.insert(e_id, is_rev);
    }

    // Propagate tolerances for merged edges using: tol_new = max(tol_1, tol_2) + d / 2
    for i in 0..n_edges {
        let e_id = edge_keys[i];
        let rep_id = e_map[&e_id];
        if e_id != rep_id {
            let e_tol = brep.edges[e_id].tolerance;
            let rep_tol = brep.edges[rep_id].tolerance;

            let mut d = 0.0;
            let e1 = &brep.edges[e_id];
            let e2 = &brep.edges[rep_id];
            if let (Some(c1), Some(c2)) = (&e1.curve, &e2.curve) {
                let t1_mid = 0.5 * (e1.first + e1.last);
                let t2_mid = 0.5 * (e2.first + e2.last);
                let p1_start = c1.point(e1.first);
                let p1_mid = c1.point(t1_mid);
                let p1_end = c1.point(e1.last);

                let endpoints_match_same = e1.start == e2.start && e1.end == e2.end;
                let (p2_start, p2_mid, p2_end) = if endpoints_match_same {
                    (c2.point(e2.first), c2.point(t2_mid), c2.point(e2.last))
                } else {
                    (c2.point(e2.last), c2.point(t2_mid), c2.point(e2.first))
                };
                let d_start = p1_start.distance(&p2_start);
                let d_mid = p1_mid.distance(&p2_mid);
                let d_end = p1_end.distance(&p2_end);
                d = d_start.max(d_mid).max(d_end);
            }

            if let Some(rep_edge) = brep.edges.get_mut(rep_id) {
                rep_edge.tolerance = f64::max(rep_tol, e_tol) + d / 2.0;
            }
        }
    }

    // Remove non-representative edges.
    for i in 0..n_edges {
        let e_id = edge_keys[i];
        if e_map[&e_id] != e_id {
            brep.edges.remove(e_id);
        }
    }

    // 6. Update LoopData edge references and invert orientation if merged opposite.
    for (_, l_data) in &mut brep.loops {
        for oe in &mut l_data.edges {
            let orig_id = oe.id;
            let new_id = e_map[&orig_id];
            oe.id = new_id;
            if edge_reversals[&orig_id] {
                oe.orientation = oe.orientation.reversed();
            }
        }
    }

    // 6.5 Re-thread every loop into a contiguous, consistently-oriented chain.
    // The coplanar imprint/partition passes can leave a closed loop with one
    // co-edge's orientation flipped, which fails the `LoopNotContiguous` validator
    // even though the loop is geometrically closed. Restore the loop invariant by
    // threading the edges by endpoint position; this is a no-op on loops that are
    // already contiguous (their winding is preserved exactly).
    let loop_ids: Vec<_> = brep.loops.keys().collect();
    for l_id in loop_ids {
        let edges = match brep.loops.get(l_id) {
            Some(l) => l.edges.clone(),
            None => continue,
        };
        if let Some(threaded) = rethread_loop(&brep, &edges) {
            if let Some(l) = brep.loops.get_mut(l_id) {
                l.edges = threaded;
            }
        }
    }

    // 7. Perform normal orientation propagation across faces using BFS.
    let mut face_adj: HashMap<FaceId, Vec<(FaceId, EdgeId)>> = HashMap::new();
    for (f_id, f_data) in &brep.faces {
        let mut wires = Vec::new();
        if let Some(w) = f_data.outer_wire {
            wires.push(w);
        }
        wires.extend(&f_data.inner_wires);

        for w_id in wires {
            if let Some(l_data) = brep.loops.get(w_id) {
                for oe in &l_data.edges {
                    face_adj.entry(f_id).or_default().push((f_id, oe.id));
                }
            }
        }
    }

    // Cross-link adjacent faces sharing the same edge.
    let mut edge_to_faces: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for (&f_id, adj_list) in &face_adj {
        for &(_, e_id) in adj_list {
            edge_to_faces.entry(e_id).or_default().push(f_id);
        }
    }

    let face_set: HashSet<FaceId> = face_ids.iter().copied().collect();
    let mut visited_faces = HashSet::new();
    let mut queue = VecDeque::new();

    for &start_face_id in &face_ids {
        if visited_faces.contains(&start_face_id) {
            continue;
        }
        queue.push_back(start_face_id);
        visited_faces.insert(start_face_id);

        while let Some(curr_f_id) = queue.pop_front() {
            let (curr_f_orientation, curr_wires) = {
                let curr_f_data = &brep.faces[curr_f_id];
                let mut wires = Vec::new();
                if let Some(w) = curr_f_data.outer_wire {
                    wires.push(w);
                }
                wires.extend(&curr_f_data.inner_wires);
                (curr_f_data.orientation, wires)
            };

            // Find current face's oriented edge directions.
            let mut curr_edge_dirs = HashMap::new();
            for w_id in curr_wires {
                if let Some(l_data) = brep.loops.get(w_id) {
                    for oe in &l_data.edges {
                        curr_edge_dirs.insert(oe.id, oe.orientation);
                    }
                }
            }

            for (&e_id, &curr_orientation) in &curr_edge_dirs {
                if let Some(adj_faces) = edge_to_faces.get(&e_id) {
                    for &adj_f_id in adj_faces {
                        if !face_set.contains(&adj_f_id) {
                            continue;
                        }
                        if visited_faces.contains(&adj_f_id) {
                            continue;
                        }

                        // Find orientation of adj_f_id on this edge.
                        let (adj_f_orientation, adj_wires) = {
                            let adj_f_data = &brep.faces[adj_f_id];
                            let mut wires = Vec::new();
                            if let Some(w) = adj_f_data.outer_wire {
                                wires.push(w);
                            }
                            wires.extend(&adj_f_data.inner_wires);
                            (adj_f_data.orientation, wires)
                        };

                        let mut adj_orientation = None;
                        for w_id in adj_wires {
                            if let Some(l_data) = brep.loops.get(w_id) {
                                for oe in &l_data.edges {
                                    if oe.id == e_id {
                                        adj_orientation = Some(oe.orientation);
                                        break;
                                    }
                                }
                            }
                        }

                        if let Some(adj_orientation) = adj_orientation {
                            // In a manifold shell, sharing faces should traverse the edge in opposite orientations
                            // when their normals are aligned. We must adjust the loop edge orientations by the
                            // face orientations to get the effective traversal directions.
                            let e1 = if curr_f_orientation == Orientation::Reversed {
                                curr_orientation.reversed()
                            } else {
                                curr_orientation
                            };

                            let e2 = if adj_f_orientation == Orientation::Reversed {
                                adj_orientation.reversed()
                            } else {
                                adj_orientation
                            };

                            let normal_needs_flip = e1 == e2;
                            if normal_needs_flip {
                                if let Some(adj_face_data) = brep.faces.get_mut(adj_f_id) {
                                    adj_face_data.orientation =
                                        adj_face_data.orientation.reversed();
                                }
                            }
                        }

                        visited_faces.insert(adj_f_id);
                        queue.push_back(adj_f_id);
                    }
                }
            }
        }
    }

    // 7.5 Canonicalize orientation: make every face's stored normal agree with
    // its winding and (for a closed shell) orient the whole shell outward. Step 7
    // only makes windings mutually consistent; this reconciles them with the
    // stored surface normals and fixes a globally-inward shell.
    canonicalize_shell_orientation(&mut brep, &face_ids);

    // 8. Garbage-collect orphan entities. `BRep::merge` copies *every* entity of
    // each source arena, so faces assembled from edges borrowed from another
    // face's arena (prism/sweep laterals do exactly this) drag that whole face
    // in as an orphan. Pruning to the faces the shell actually references keeps
    // the arena consistent with the shell — without it, a later partition that
    // removes a loop shared with an orphan leaves a dangling reference that
    // panics every full-arena traversal (BVH build, validation).
    brep.retain_faces(&face_ids);

    // 9. Create the final Shell in the BRep.
    let shell_id = brep.shells.insert(ShellData { faces: face_ids });

    Shell::from_id(Arc::new(brep), shell_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_geom::{GeomCurve, Line};
    use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};

    /// A unit square in the Z=0 plane at `offset_x`, every boundary vertex built
    /// with the given per-entity `vtol`.
    fn make_square_face_with_vtol(offset_x: f64, vtol: f64) -> Face {
        let corners = [
            Pnt::new(offset_x, 0.0, 0.0),
            Pnt::new(offset_x + 1.0, 0.0, 0.0),
            Pnt::new(offset_x + 1.0, 1.0, 0.0),
            Pnt::new(offset_x, 1.0, 0.0),
        ];
        let verts: Vec<Vertex> = corners
            .iter()
            .map(|p| Vertex::new_with_tolerance(*p, vtol))
            .collect();
        let mut edges = Vec::new();
        for i in 0..4 {
            let a = corners[i];
            let b = corners[(i + 1) % 4];
            let dir = (b - a).normalized().expect("non-degenerate edge");
            let line = GeomCurve::line(Line::from_point_dir(a, dir));
            edges.push(Edge::new(
                Some(line),
                0.0,
                a.distance(&b),
                verts[i].clone(),
                verts[(i + 1) % 4].clone(),
            ));
        }
        Face::new(None, Wire::from_edges(edges))
    }

    #[test]
    fn test_sewing_respects_per_entity_tolerance() {
        // Two squares separated by a 1e-3 gap. The global sewing tolerance is a
        // tight 1e-6 (would NOT heal the gap on its own), but the boundary
        // vertices carry a large 1e-3 local tolerance, so per-entity tolerance
        // alone must close the seam.
        let f1 = make_square_face_with_vtol(0.0, 1e-3);
        let f2 = make_square_face_with_vtol(1.001, 1e-3);

        let shell = sew(&[f1, f2], 1e-6);
        let solid = Solid::new(shell);
        assert_eq!(
            solid.vertex_count(),
            6,
            "shared vertices should have merged"
        );
        assert_eq!(solid.edge_count(), 7);

        // Clean geometry (default CONFUSION tolerance) under the same tight
        // global tolerance stays unsewn — per-entity tolerance only *widens*
        // merging, never forces it on confident vertices.
        let g1 = make_square_face_with_vtol(0.0, openrcad_foundation::tolerance::CONFUSION);
        let g2 = make_square_face_with_vtol(1.001, openrcad_foundation::tolerance::CONFUSION);
        let shell_clean = sew(&[g1, g2], 1e-6);
        let solid_clean = Solid::new(shell_clean);
        assert_eq!(
            solid_clean.vertex_count(),
            8,
            "tight tol keeps clean seam open"
        );
    }

    fn make_square_face(offset_x: f64) -> Face {
        let w = Wire::from_edges([
            Edge::between_points(
                Pnt::new(offset_x, 0.0, 0.0),
                Pnt::new(offset_x + 1.0, 0.0, 0.0),
            ),
            Edge::between_points(
                Pnt::new(offset_x + 1.0, 0.0, 0.0),
                Pnt::new(offset_x + 1.0, 1.0, 0.0),
            ),
            Edge::between_points(
                Pnt::new(offset_x + 1.0, 1.0, 0.0),
                Pnt::new(offset_x, 1.0, 0.0),
            ),
            Edge::between_points(Pnt::new(offset_x, 1.0, 0.0), Pnt::new(offset_x, 0.0, 0.0)),
        ]);
        Face::new(None, w)
    }

    #[test]
    fn test_sewing_heals_gap() {
        let f1 = make_square_face(0.0);
        // Gap of 0.001 (1e-3)
        let f2 = make_square_face(1.001);

        // Sew with small tolerance (1e-5) -> no connection.
        let shell_unsewn = sew(&[f1.clone(), f2.clone()], 1e-5);
        let solid_unsewn = Solid::new(shell_unsewn);
        assert_eq!(solid_unsewn.vertex_count(), 8);
        assert_eq!(solid_unsewn.edge_count(), 8);

        // Sew with larger tolerance (2e-3) -> connected.
        let shell_sewn = sew(&[f1, f2], 2e-3);
        let solid_sewn = Solid::new(shell_sewn);
        // Shared side vertices are merged. Sewn: 6 vertices, 7 edges.
        assert_eq!(solid_sewn.vertex_count(), 6);
        assert_eq!(solid_sewn.edge_count(), 7);
        // Two coplanar faces stitched along a shared seam: the merged topology
        // must remain structurally valid (no dangling edge refs left by the
        // edge-merge pass, loops still contiguous).
        solid_sewn.assert_valid();

        // Verify that in the underlying BRep, we actually have exactly 7 edge entities in the arena
        assert_eq!(solid_sewn.brep().edges.len(), 7);
    }

    #[test]
    fn test_sewing_aligns_normals() {
        let f1 = make_square_face(0.0);
        let f2 = make_square_face(1.001).reversed(); // orientation is Reversed

        // Sew them together
        let shell_sewn = sew(&[f1, f2], 2e-3);

        // Let's check the orientations of the faces inside the sewn shell.
        // They must be consistent.
        let faces = shell_sewn.faces();
        assert_eq!(faces.len(), 2);

        let f1_sewn = &faces[0];
        let f2_sewn = &faces[1];

        // Since we oriented them consistently, one of them must have flipped to align with the other.
        // Let's verify their effective normal direction matches.
        // For two squares adjacent in X (X=0..1 and X=1..2):
        // The shared edge is at X=1. Face 1 traverses it UP, Face 2 traverses it DOWN.
        // So their raw loop edge orientations are opposite.
        // Since they are opposite, they are consistent if and only if both faces have the same orientation value.
        // Let's verify they both have the same orientation!
        assert_eq!(f1_sewn.orientation(), f2_sewn.orientation());
    }

    /// Effective outward normal of a planar face, or `None` if non-planar.
    fn effective_planar_normal(face: &Face) -> Option<openrcad_foundation::Dir> {
        let n = match face.surface()? {
            GeomSurface::Plane(p) => p.normal(),
            _ => return None,
        };
        Some(if face.orientation() == Orientation::Reversed {
            n.reversed()
        } else {
            n
        })
    }

    /// For a convex solid, every planar face's effective normal must point away
    /// from the solid centroid (i.e. outward).
    fn assert_planar_normals_outward(solid: &Solid) {
        let verts = solid.vertices();
        let n = verts.len() as f64;
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for v in &verts {
            cx += v.point().x();
            cy += v.point().y();
            cz += v.point().z();
        }
        let centroid = Pnt::new(cx / n, cy / n, cz / n);

        let mut checked = 0;
        for face in solid.shell().faces() {
            let Some(en) = effective_planar_normal(&face) else {
                continue;
            };
            let pts: Vec<Pnt> = face
                .outer_wire()
                .unwrap()
                .edges()
                .iter()
                .map(|e| e.start().point())
                .collect();
            let m = pts.len() as f64;
            let fc = Pnt::new(
                pts.iter().map(|p| p.x()).sum::<f64>() / m,
                pts.iter().map(|p| p.y()).sum::<f64>() / m,
                pts.iter().map(|p| p.z()).sum::<f64>() / m,
            );
            let out = fc - centroid;
            let dot = out.x() * en.x() + out.y() * en.y() + out.z() * en.z();
            assert!(
                dot > 0.0,
                "face effective normal points inward (dot = {dot})"
            );
            checked += 1;
        }
        assert!(checked > 0, "expected at least one planar face to check");
    }

    #[test]
    fn sew_keeps_outward_box_outward() {
        // A clean primitive box is already outward — canonicalization is a no-op.
        let solid = openrcad_primitives::make_box(&Pnt::origin(), 2.0, 3.0, 4.0);
        let faces = solid.shell().faces();
        let shell = sew(&faces, openrcad_foundation::tolerance::CONFUSION * 10.0);
        let resewn = Solid::new(shell);
        assert!(resewn.is_watertight());
        assert!(resewn.health_report().is_healthy());
        assert_planar_normals_outward(&resewn);
    }

    #[test]
    fn sew_reorients_inverted_box_outward() {
        // Reverse every face so the shell faces *inward*; sew's global outward
        // pass (Fix #1, step 2) must flip it back so all normals point outward.
        let solid = openrcad_primitives::make_box(&Pnt::origin(), 2.0, 3.0, 4.0);
        let inverted: Vec<Face> = solid.shell().faces().iter().map(|f| f.reversed()).collect();
        let shell = sew(&inverted, openrcad_foundation::tolerance::CONFUSION * 10.0);
        let fixed = Solid::new(shell);
        assert!(fixed.is_watertight());
        assert!(fixed.health_report().is_healthy());
        assert_planar_normals_outward(&fixed);
    }

    #[test]
    fn test_sewing_disconnected_components() {
        let f1 = make_square_face(0.0);
        let f2 = make_square_face(1.001);
        let f3 = make_square_face(10.0);
        let f4 = make_square_face(11.001);

        let shell = sew(&[f1, f2, f3, f4], 2e-3);
        assert_eq!(shell.faces().len(), 4);

        // Both disjoint pairs should have merged vertices/edges internally.
        // In the BRep, we should have 12 vertices total: 6 for first pair, 6 for second pair.
        // And 14 edges total: 7 for first, 7 for second.
        let brep = shell.brep();
        assert_eq!(brep.vertices.len(), 12);
        assert_eq!(brep.edges.len(), 14);
    }
}
