//! Imprinting an intersection curve onto a face and partitioning it.
//!
//! This is the shared primitive the boolean engine and (eventually) the blend
//! overflow tracer both build on: given a face and a curve that lies on (or
//! crosses) it, commit the curve as edges and split the face into the resulting
//! sub-regions.
//!
//! [`imprint_curve_on_face`] records open curve segments as split edges and lets
//! the boolean partition pass consume the complete per-face graph at once. This
//! matters for curved faces where one real intersection can arrive as several
//! trimmed pieces before it reaches both boundaries.

use openrcad_foundation::Pnt;
use openrcad_geom::{Curve, GeomCurve, Surface};
use openrcad_topo::arena::{EdgeData, EdgeId, OrientedEdge, VertexData, VertexId};
use openrcad_topo::{BRepBuilder, Face, FaceId, LoopId, Orientation};

use crate::boolean::project_point_on_curve;
use crate::intersect::{curve_curve, is_inside_trimming_loops, uv_of};

fn imprint_tolerance(tol: f64) -> f64 {
    tol.max(1e-4)
}

/// Imprint `curve` onto `face_id` and return the resulting sub-face ids.
///
/// Handles a closed curve lying fully inside the loop by drilling a hole (the
/// face gains an inner wire and the enclosed disk becomes its own face). Open
/// segments are returned as split edges to be partitioned together after all
/// face-pair intersections have been imprinted.
pub(crate) fn imprint_curve_on_face(
    builder: &mut BRepBuilder,
    face_id: FaceId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    force_queue_clean_crosscuts: bool,
    tol: f64,
) -> (Vec<FaceId>, Vec<EdgeId>) {
    let face_data = match builder.brep().faces.get(face_id) {
        Some(d) => d.clone(),
        None => return (vec![face_id], Vec::new()),
    };
    let outer_loop_id: LoopId = match face_data.outer_wire {
        Some(w) => w,
        None => return (vec![face_id], Vec::new()),
    };

    let surface = match &face_data.surface {
        Some(s) => s,
        None => return (vec![face_id], Vec::new()),
    };

    // 1. Gather all edges of the face (outer loop + inner loops)
    let mut face_edges = Vec::new();
    face_edges.extend(builder.brep().loops[outer_loop_id].edges.iter().copied());
    for &inner_id in &face_data.inner_wires {
        face_edges.extend(builder.brep().loops[inner_id].edges.iter().copied());
    }

    // 2. Find all intersections of `curve` within `[first, last]` with the face edges
    let mut intersections = Vec::new();
    for oe in &face_edges {
        let e_id = oe.id;
        let e_data = &builder.brep().edges[e_id];
        if let Some(e_curve) = &e_data.curve {
            let pts = curve_curve(e_curve, curve, tol);
            for pt in pts {
                let t_edge = project_point_on_curve(&pt, e_curve, e_data.first, e_data.last);
                if e_curve.point(t_edge).distance(&pt) > 1e-6 {
                    continue;
                }

                let t_curve = project_point_on_curve(&pt, curve, first, last);
                if curve.point(t_curve).distance(&pt) > 1e-6 {
                    continue; // outside the segment bounds
                }

                if intersections
                    .iter()
                    .any(|(_, _, _, q): &(_, _, _, Pnt)| q.distance(&pt) < tol)
                {
                    continue;
                }
                intersections.push((e_id, t_edge, t_curve, pt));
            }
        }
    }

    intersections.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

    // 3. Build sub-intervals based on intersection parameters
    let mut params = Vec::new();
    params.push((first, None)); // (t, Option<(EdgeId, t_edge, Pnt)>)
    for &(e_id, t_edge, t_curve, pt) in &intersections {
        if (t_curve - first).abs() < tol {
            params[0].1 = Some((e_id, t_edge, pt));
        } else if (t_curve - last).abs() < tol {
            // Will match later
        } else if t_curve > first + tol && t_curve < last - tol {
            params.push((t_curve, Some((e_id, t_edge, pt))));
        }
    }
    let mut last_info = None;
    for &(e_id, t_edge, t_curve, pt) in &intersections {
        if (t_curve - last).abs() < tol {
            last_info = Some((e_id, t_edge, pt));
        }
    }
    params.push((last, last_info));

    // 4. Filter sub-intervals to those whose interior lies inside the face
    let probe = Face::from_id(
        std::sync::Arc::new(builder.brep().clone()),
        face_id,
        face_data.orientation,
    );
    let mut inside_segments = Vec::new();
    for i in 0..(params.len() - 1) {
        let t_start = params[i].0;
        let t_end = params[i + 1].0;
        if t_end - t_start < tol {
            continue;
        }
        let t_mid = 0.5 * (t_start + t_end);
        let mid_p = curve.point(t_mid);
        let (u, v) = uv_of(surface, &mid_p);
        if is_inside_trimming_loops(u, v, &probe) {
            inside_segments.push((t_start, params[i].1, t_end, params[i + 1].1));
        }
    }

    if inside_segments.is_empty() {
        return (vec![face_id], Vec::new());
    }

    // Case B: A closed curve lying entirely inside the face cuts a hole
    let segment_is_closed =
        curve.is_closed() && curve.point(first).distance(&curve.point(last)) < tol;
    if intersections.is_empty() && segment_is_closed {
        if let Some(faces) = cut_hole(builder, face_id, curve, first, last, tol) {
            return (faces, Vec::new());
        }
    }

    // Case A: Clean 2-point cross-cut of the face. Standalone clean cuts use
    // the legacy immediate split path; if this face already has queued open
    // split edges, keep this edge queued too so the complete graph partitions
    // together instead of stranding earlier pending edges on a removed face.
    if inside_segments.len() == 1 {
        let (t_start, start_info, t_end, end_info) = inside_segments[0];
        let start_info = start_info.or_else(|| {
            let p = curve.point(t_start);
            boundary_hit_for_point(builder, &face_edges, &p, tol)
        });
        let end_info = end_info.or_else(|| {
            let p = curve.point(t_end);
            boundary_hit_for_point(builder, &face_edges, &p, tol)
        });
        if let (Some((e1_id, t1_edge, pt1)), Some((e2_id, t2_edge, pt2))) = (start_info, end_info) {
            if pt1.distance(&pt2) > tol {
                // Check if this chord is already a boundary edge of the face
                let mut is_boundary = false;
                for oe in &face_edges {
                    let e_id = oe.id;
                    let Some(ed) = builder.brep().edges.get(e_id) else {
                        continue;
                    };
                    if let Some(ec) = &ed.curve {
                        let t_mid = 0.5 * (t_start + t_end);
                        let mid_p = curve.point(t_mid);
                        let t = project_point_on_curve(&mid_p, ec, ed.first, ed.last);
                        if mid_p.distance(&ec.point(t)) < tol {
                            is_boundary = true;
                            break;
                        }
                    }
                }

                if !is_boundary {
                    let v1 = resolve_or_split(builder, e1_id, t1_edge, &pt1, tol);
                    let v2 = resolve_or_split(builder, e2_id, t2_edge, &pt2, tol);
                    if v1 != v2 {
                        let split_edge_data = EdgeData {
                            curve: Some(curve.clone()),
                            first: t_start,
                            last: t_end,
                            start: v1,
                            end: v2,
                            tolerance: imprint_tolerance(tol),
                        };
                        let split_edge_id = builder.brep_mut().edges.insert(split_edge_data);
                        if force_queue_clean_crosscuts {
                            return (vec![face_id], vec![split_edge_id]);
                        }
                        let (f1, f2) = builder.split_face(face_id, &[split_edge_id]);
                        return (vec![f1, f2], Vec::new());
                    }
                }
            }
        }
    }

    // Case C: Generalized / Partial imprint
    // Insert all inside segments as split edges.
    let mut split_edges = Vec::new();
    for (t_start, start_info, t_end, end_info) in inside_segments {
        let start_info = start_info.or_else(|| {
            let p = curve.point(t_start);
            boundary_hit_for_point(builder, &face_edges, &p, tol)
        });
        let end_info = end_info.or_else(|| {
            let p = curve.point(t_end);
            boundary_hit_for_point(builder, &face_edges, &p, tol)
        });
        // Check if this segment is already a boundary edge of the face
        let mut is_boundary = false;
        for oe in &face_edges {
            let e_id = oe.id;
            let Some(ed) = builder.brep().edges.get(e_id) else {
                continue;
            };
            if let Some(ec) = &ed.curve {
                let t_mid = 0.5 * (t_start + t_end);
                let mid_p = curve.point(t_mid);
                let t = project_point_on_curve(&mid_p, ec, ed.first, ed.last);
                if mid_p.distance(&ec.point(t)) < tol {
                    is_boundary = true;
                    break;
                }
            }
        }
        if is_boundary {
            continue;
        }

        let v_start = if let Some((e_id, t_edge, pt)) = start_info {
            resolve_or_split(builder, e_id, t_edge, &pt, tol)
        } else {
            let p = curve.point(t_start);
            resolve_after_prior_split(builder, &p, tol)
        };

        let v_end = if let Some((e_id, t_edge, pt)) = end_info {
            resolve_or_split(builder, e_id, t_edge, &pt, tol)
        } else {
            let p = curve.point(t_end);
            resolve_after_prior_split(builder, &p, tol)
        };

        if v_start == v_end {
            continue;
        }
        if pending_split_edge_exists(builder, v_start, v_end, curve, t_start, t_end, tol) {
            continue;
        }

        let split_edge_data = EdgeData {
            curve: Some(curve.clone()),
            first: t_start,
            last: t_end,
            start: v_start,
            end: v_end,
            tolerance: imprint_tolerance(tol),
        };
        let split_edge_id = builder.brep_mut().edges.insert(split_edge_data);
        split_edges.push(split_edge_id);
    }

    (vec![face_id], split_edges)
}

fn pending_split_edge_exists(
    builder: &BRepBuilder,
    v_start: VertexId,
    v_end: VertexId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    tol: f64,
) -> bool {
    let snap_tol = tol.max(1e-4);
    let mid = curve.point(0.5 * (first + last));
    builder.brep().edges.iter().any(|(_, edge)| {
        let endpoints_match = (edge.start == v_start && edge.end == v_end)
            || (edge.start == v_end && edge.end == v_start);
        if !endpoints_match {
            return false;
        }
        let Some(existing_curve) = edge.curve.as_ref() else {
            return false;
        };
        let existing_mid = existing_curve.point(0.5 * (edge.first + edge.last));
        mid.distance(&existing_mid) <= snap_tol
    })
}

/// Return the loop vertex at `pt` on edge `e_id`: reuse an existing endpoint if
/// `pt` coincides with one, otherwise split the edge there to create a fresh
/// vertex. Prevents zero-length edges when a face crossing lands on a vertex
/// that pass A already inserted.
fn resolve_or_split(
    builder: &mut BRepBuilder,
    e_id: EdgeId,
    t_edge: f64,
    pt: &Pnt,
    tol: f64,
) -> VertexId {
    if !builder.brep().edges.contains_key(e_id) {
        return resolve_after_prior_split(builder, pt, tol);
    }
    let ed = builder.brep().edges[e_id].clone();
    let s_pt = builder.brep().vertices[ed.start].point;
    let e_pt = builder.brep().vertices[ed.end].point;
    if pt.distance(&s_pt) < tol {
        return ed.start;
    }
    if pt.distance(&e_pt) < tol {
        return ed.end;
    }
    let v = builder.brep_mut().vertices.insert(VertexData {
        point: *pt,
        tolerance: imprint_tolerance(tol),
    });
    builder.split_edge(e_id, v, t_edge);
    v
}

fn boundary_hit_for_point(
    builder: &BRepBuilder,
    face_edges: &[OrientedEdge],
    pt: &Pnt,
    tol: f64,
) -> Option<(EdgeId, f64, Pnt)> {
    let snap_tol = tol.max(1e-4);
    for oe in face_edges {
        let Some(ed) = builder.brep().edges.get(oe.id) else {
            continue;
        };
        let Some(curve) = ed.curve.as_ref() else {
            continue;
        };
        let t = project_point_on_curve(pt, curve, ed.first, ed.last);
        let q = curve.point(t);
        if q.distance(pt) <= snap_tol {
            return Some((oe.id, t, q));
        }
    }
    None
}

/// Resolve a point after an earlier endpoint split removed the original edge id
/// captured during intersection gathering. Find the current sub-edge that still
/// contains `pt`, split it when needed, or finally create/reuse a coincident
/// vertex if the point is already detached from a boundary.
fn resolve_after_prior_split(builder: &mut BRepBuilder, pt: &Pnt, tol: f64) -> VertexId {
    let snap_tol = tol.max(1e-4);
    if let Some(v) = find_vertex_at(builder, pt, snap_tol) {
        return v;
    }

    let candidate = builder
        .brep()
        .edges
        .iter()
        .filter_map(|(id, ed)| {
            let curve = ed.curve.as_ref()?;
            let t = project_point_on_curve(pt, curve, ed.first, ed.last);
            (curve.point(t).distance(pt) <= snap_tol).then_some((id, t))
        })
        .next();

    if let Some((id, t)) = candidate {
        return resolve_or_split(builder, id, t, pt, tol);
    }

    builder.brep_mut().vertices.insert(VertexData {
        point: *pt,
        tolerance: imprint_tolerance(tol),
    })
}

fn find_vertex_at(builder: &BRepBuilder, pt: &Pnt, tol: f64) -> Option<VertexId> {
    builder
        .brep()
        .vertices
        .iter()
        .find_map(|(v_id, v_data)| (v_data.point.distance(pt) < tol).then_some(v_id))
}

/// Cut a hole bounded by the closed `curve` into `face_id`, provided the curve
/// lies on the face's surface and inside its outer loop. The face gains the
/// curve as an inner wire; the enclosed disk is returned as a new face. Returns
/// `None` if the curve does not belong to this face (so the caller leaves it
/// untouched).
fn cut_hole(
    builder: &mut BRepBuilder,
    face_id: FaceId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    tol: f64,
) -> Option<Vec<FaceId>> {
    use openrcad_topo::arena::{FaceData, LoopData};

    let face_data = builder.brep().faces.get(face_id)?.clone();
    let surface = face_data.surface.clone()?;
    let cmin = first;
    let cmax = last;
    if !cmin.is_finite() || !cmax.is_finite() || (cmax - cmin).abs() < tol {
        return None;
    }

    // The curve must lie on this face's surface.
    for k in 0..4 {
        let t = cmin + (cmax - cmin) * (k as f64) / 4.0;
        let p = curve.point(t);
        let (u, v) = uv_of(&surface, &p);
        if surface.point(u, v).distance(&p) > 1e-4 {
            return None;
        }
    }

    // Skip if a hole bounded by this same curve already exists (idempotency).
    for &inner in &face_data.inner_wires {
        if let Some(&oe) = builder.brep().loops[inner].edges.first() {
            let e = oe.id;
            if builder.brep().edges[e].curve.as_ref() == Some(curve) {
                return None;
            }
        }
    }

    // The hole must lie inside the face's outer loop.
    let mid = curve.point(0.5 * (cmin + cmax));
    let (mu, mv) = uv_of(&surface, &mid);
    let probe = Face::from_id(
        std::sync::Arc::new(builder.brep().clone()),
        face_id,
        face_data.orientation,
    );
    if !is_inside_trimming_loops(mu, mv, &probe) {
        return None;
    }

    // Split the closed curve into three arcs (keeps edge endpoints distinct).
    let breaks = [
        cmin,
        cmin + (cmax - cmin) / 3.0,
        cmin + 2.0 * (cmax - cmin) / 3.0,
        cmax,
    ];
    let verts: Vec<_> = (0..3)
        .map(|i| {
            builder.brep_mut().vertices.insert(VertexData {
                point: curve.point(breaks[i]),
                tolerance: openrcad_foundation::tolerance::CONFUSION,
            })
        })
        .collect();
    let arc_ids: Vec<_> = (0..3)
        .map(|i| {
            builder.brep_mut().edges.insert(EdgeData {
                curve: Some(curve.clone()),
                first: breaks[i],
                last: breaks[i + 1],
                start: verts[i],
                end: verts[(i + 1) % 3],
                tolerance: openrcad_foundation::tolerance::CONFUSION,
            })
        })
        .collect();

    let inner_edges: Vec<_> = arc_ids
        .iter()
        .map(|&id| OrientedEdge {
            id,
            orientation: Orientation::Forward,
        })
        .collect();
    let disk_edges: Vec<_> = inner_edges
        .iter()
        .rev()
        .map(|oe| OrientedEdge {
            id: oe.id,
            orientation: oe.orientation.reversed(),
        })
        .collect();

    let inner_loop = builder
        .brep_mut()
        .loops
        .insert(LoopData { edges: inner_edges });
    let disk_loop = builder
        .brep_mut()
        .loops
        .insert(LoopData { edges: disk_edges });

    builder
        .brep_mut()
        .faces
        .get_mut(face_id)
        .unwrap()
        .inner_wires
        .push(inner_loop);
    let disk = builder.brep_mut().faces.insert(FaceData {
        surface: face_data.surface.clone(),
        outer_wire: Some(disk_loop),
        inner_wires: Vec::new(),
        orientation: face_data.orientation,
    });
    for (_, shell) in &mut builder.brep_mut().shells {
        if shell.faces.contains(&face_id) {
            shell.faces.push(disk);
        }
    }

    Some(vec![face_id, disk])
}
