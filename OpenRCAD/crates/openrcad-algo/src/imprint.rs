//! Imprinting an intersection curve onto a face and partitioning it.
//!
//! This is the shared primitive the boolean engine and (eventually) the blend
//! overflow tracer both build on: given a face and a curve that lies on (or
//! crosses) it, commit the curve as edges and split the face into the resulting
//! sub-regions.
//!
//! Today [`imprint_curve_on_face`] reproduces the two cases the boolean split
//! pass has always handled — a clean two-point cross-cut, and a closed curve
//! drilling a hole — and leaves every other topology (entry-only, weave, open
//! interior curves) untouched. Those "partial imprint" cases are generalised in
//! a later milestone; the function is extracted here first so the
//! generalisation lands in one place and both call sites (booleans now, blends
//! later) share it.

use openrcad_foundation::Pnt;
use openrcad_geom::{Curve, GeomCurve, Surface};
use openrcad_topo::arena::{EdgeData, EdgeId, OrientedEdge, VertexData, VertexId};
use openrcad_topo::{BRepBuilder, Face, FaceId, LoopId, Orientation};

use crate::boolean::project_point_on_curve;
use crate::intersect::{curve_curve, is_inside_trimming_loops, uv_of};

/// Imprint `curve` onto `face_id` and return the resulting sub-face ids.
///
/// Handles a clean two-point cross-cut (the curve enters and exits the outer
/// loop) by inserting a split edge and bisecting the face, and a closed curve
/// lying fully inside the loop by drilling a hole (the face gains an inner wire
/// and the enclosed disk becomes its own face). Any other topology currently
/// leaves the face unchanged — returned as `vec![face_id]` — which the boolean
/// pipeline tolerates (the face simply is not partitioned by this curve).
pub(crate) fn imprint_curve_on_face(
    builder: &mut BRepBuilder,
    face_id: FaceId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
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

    // Case A: Clean 2-point cross-cut of the face.
    // If there is exactly one inside segment, and both endpoints are boundary intersections,
    // and the endpoints resolve to distinct vertices, we split immediately.
    if inside_segments.len() == 1 {
        let (t_start, start_info, t_end, end_info) = inside_segments[0];
        if let (Some((e1_id, t1_edge, pt1)), Some((e2_id, t2_edge, pt2))) = (start_info, end_info) {
            if pt1.distance(&pt2) > tol {
                // Check if this chord is already a boundary edge of the face
                let mut is_boundary = false;
                for oe in &face_edges {
                    let e_id = oe.id;
                    let ed = &builder.brep().edges[e_id];
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
                            tolerance: openrcad_foundation::tolerance::CONFUSION,
                        };
                        let split_edge_id = builder.brep_mut().edges.insert(split_edge_data);
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
        // Check if this segment is already a boundary edge of the face
        let mut is_boundary = false;
        for oe in &face_edges {
            let e_id = oe.id;
            let ed = &builder.brep().edges[e_id];
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
            let mut matched = None;
            for (v_id, v_data) in &builder.brep().vertices {
                if v_data.point.distance(&p) < tol {
                    matched = Some(v_id);
                    break;
                }
            }
            matched.unwrap_or_else(|| {
                builder.brep_mut().vertices.insert(VertexData {
                    point: p,
                    tolerance: openrcad_foundation::tolerance::CONFUSION,
                })
            })
        };

        let v_end = if let Some((e_id, t_edge, pt)) = end_info {
            resolve_or_split(builder, e_id, t_edge, &pt, tol)
        } else {
            let p = curve.point(t_end);
            let mut matched = None;
            for (v_id, v_data) in &builder.brep().vertices {
                if v_data.point.distance(&p) < tol {
                    matched = Some(v_id);
                    break;
                }
            }
            matched.unwrap_or_else(|| {
                builder.brep_mut().vertices.insert(VertexData {
                    point: p,
                    tolerance: openrcad_foundation::tolerance::CONFUSION,
                })
            })
        };

        if v_start == v_end {
            continue;
        }

        let split_edge_data = EdgeData {
            curve: Some(curve.clone()),
            first: t_start,
            last: t_end,
            start: v_start,
            end: v_end,
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        };
        let split_edge_id = builder.brep_mut().edges.insert(split_edge_data);
        split_edges.push(split_edge_id);
    }

    (vec![face_id], split_edges)
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
        tolerance: openrcad_foundation::tolerance::CONFUSION,
    });
    builder.split_edge(e_id, v, t_edge);
    v
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
