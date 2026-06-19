use crate::BooleanOp;
use openrcad_foundation::{Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::arena::EdgeId;
use openrcad_topo::{BRepBuilder, Face, FaceId, HealthReport, Solid};
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::bvh::Bvh;
use crate::sew::sew;

/// Which operand failed preflight validation for a boolean operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanInput {
    /// The solid being operated on.
    Object,
    /// The tool solid.
    Tool,
}

/// Structured boolean failure for applications that need recoverable modeling.
#[derive(Clone, Debug, PartialEq)]
pub enum BooleanError {
    /// One of the input solids is structurally invalid or not watertight.
    InvalidInput {
        /// The invalid operand.
        input: BooleanInput,
        /// Topology health diagnostics for that operand.
        report: HealthReport,
    },
    /// The boolean implementation panicked before it could produce a result.
    Panicked,
    /// The resulting solid failed structural health checks.
    InvalidOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
    /// The resulting solid is structurally valid but has open/free boundary
    /// edges, so it is unsafe to treat as a closed CAD solid.
    NonWatertightOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
}

impl core::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidInput { input, report } => {
                write!(f, "invalid boolean {input:?} input: {report:?}")
            }
            Self::Panicked => write!(f, "boolean operation panicked"),
            Self::InvalidOutput { report } => {
                write!(f, "boolean produced invalid topology: {report:?}")
            }
            Self::NonWatertightOutput { report } => {
                write!(f, "boolean produced a non-watertight solid: {report:?}")
            }
        }
    }
}

impl std::error::Error for BooleanError {}

/// Apply `op` and reject invalid inputs, panics, or unhealthy/non-watertight
/// outputs. This is the preferred entry point for CAD applications, where a
/// failed feature should be diagnosable instead of cached as a bad body.
pub fn boolean_checked(object: &Solid, tool: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    validate_operand(BooleanInput::Object, object)?;
    validate_operand(BooleanInput::Tool, tool)?;

    let result = catch_unwind(AssertUnwindSafe(|| boolean(object, tool, op)))
        .map_err(|_| BooleanError::Panicked)?;
    validate_output(result)
}

/// Apply `op` between `object` and `tool`.
pub fn boolean(object: &Solid, tool: &Solid, op: BooleanOp) -> Solid {
    let tol = 1e-5;

    // 1. Build staging builders
    let mut builder_obj = BRepBuilder::from_brep((**object.brep()).clone());
    let mut builder_tool = BRepBuilder::from_brep((**tool.brep()).clone());

    // 2. Perform intersection and splitting
    let faces_obj = object.shell().faces();
    let faces_tool = tool.shell().faces();
    let bvh_obj = Bvh::build(&faces_obj);
    let bvh_tool = Bvh::build(&faces_tool);
    let pairs = Bvh::overlapping_pairs(&bvh_obj, &bvh_tool);

    // A. Split all boundary edges at mutual intersection points
    for &(f_obj_id, f_tool_id) in &pairs {
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        for w_obj in f_obj.wires() {
            for e_obj in w_obj.edges() {
                for w_tool in f_tool.wires() {
                    for e_tool in w_tool.edges() {
                        if let (Some(c_obj), Some(c_tool)) = (e_obj.curve(), e_tool.curve()) {
                            let pts = crate::intersect::curve_curve(c_obj, c_tool, tol);
                            for pt in pts {
                                try_split_edge(
                                    &mut builder_obj,
                                    e_obj.id(),
                                    c_obj,
                                    e_obj.first(),
                                    e_obj.last(),
                                    &pt,
                                    tol,
                                );
                                try_split_edge(
                                    &mut builder_tool,
                                    e_tool.id(),
                                    c_tool,
                                    e_tool.first(),
                                    e_tool.last(),
                                    &pt,
                                    tol,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    // B. Split faces along intersection curves.
    //
    // Each original face is split only along the curves arising from the tool
    // faces it actually overlaps, and only its own descendant sub-faces are
    // re-split — never the whole builder. This keeps the work proportional to
    // the real intersections instead of exploding combinatorially across pairs.
    let mut obj_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_obj.iter().map(|f| (f.id(), vec![f.id()])).collect();
    let mut tool_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_tool.iter().map(|f| (f.id(), vec![f.id()])).collect();

    let mut splitting_edges_obj: std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>> =
        std::collections::HashMap::new();
    let mut splitting_edges_tool: std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>> =
        std::collections::HashMap::new();

    for &(f_obj_id, f_tool_id) in &pairs {
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        let s_obj = f_obj.surface().unwrap();
        let s_tool = f_tool.surface().unwrap();

        if surfaces_are_coplanar(s_obj, s_tool, tol) {
            // Coplanar faces: split f_obj along f_tool's boundary edges, and
            // vice versa — restricted to each face's own descendants.
            for w_tool in f_tool.wires() {
                for e_tool in w_tool.edges() {
                    if let Some(c_tool) = e_tool.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(c_tool, e_tool.first(), e_tool.last(), &f_obj, tol);
                        for (first, last) in intervals {
                            let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                            split_tracked(&mut builder_obj, sub, c_tool, first, last, &mut splitting_edges_obj, tol);
                        }
                    }
                }
            }
            for w_obj in f_obj.wires() {
                for e_obj in w_obj.edges() {
                    if let Some(c_obj) = e_obj.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(c_obj, e_obj.first(), e_obj.last(), &f_tool, tol);
                        for (first, last) in intervals {
                            let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                            split_tracked(&mut builder_tool, sub, c_obj, first, last, &mut splitting_edges_tool, tol);
                        }
                    }
                }
            }
        } else {
            // Intersecting surfaces: split each face along the trimmed intersection curves
            let curves = crate::intersect::surface_surface_curves(&f_obj, &f_tool, tol);
            for (curve, first, last) in curves {
                let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                split_tracked(&mut builder_obj, sub, &curve, first, last, &mut splitting_edges_obj, tol);
                let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                split_tracked(&mut builder_tool, sub, &curve, first, last, &mut splitting_edges_tool, tol);
            }
        }
    }

    // Partition faces that have accumulated splitting edges
    let run_partition = |builder: &mut BRepBuilder, sub: &mut Vec<FaceId>, split_map: &mut std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>>| {
        let mut new_sub = Vec::new();
        for &fid in sub.iter() {
            if let Some(edges) = split_map.remove(&fid) {
                if builder.brep().faces.contains_key(fid) {
                    let partitioned = builder.partition_face(fid, &edges);
                    new_sub.extend(partitioned);
                } else {
                    new_sub.push(fid);
                }
            } else {
                new_sub.push(fid);
            }
        }
        *sub = new_sub;
    };

    for sub in obj_sub.values_mut() {
        run_partition(&mut builder_obj, sub, &mut splitting_edges_obj);
    }
    for sub in tool_sub.values_mut() {
        run_partition(&mut builder_tool, sub, &mut splitting_edges_tool);
    }

    // 3. Classify all split faces
    let mut kept_faces = Vec::new();

    let brep_obj = builder_obj.build(); // seals into Arc<BRep>
    let brep_tool = builder_tool.build();

    // BVHs over the *split* faces so the coplanar pre-check tests only the
    // handful of opposite-side faces a given face actually overlaps, instead of
    // scanning all of them (O(n·m) → ≈O(n·log m)).
    let split_faces_obj: Vec<Face> = brep_obj
        .faces
        .iter()
        .map(|(id, d)| Face::from_id(brep_obj.clone(), id, d.orientation))
        .collect();
    let split_faces_tool: Vec<Face> = brep_tool
        .faces
        .iter()
        .map(|(id, d)| Face::from_id(brep_tool.clone(), id, d.orientation))
        .collect();
    let bvh_split_obj = Bvh::build(&split_faces_obj);
    let bvh_split_tool = Bvh::build(&split_faces_tool);

    for (f_id, f_data) in &brep_obj.faces {
        let face = Face::from_id(brep_obj.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar_same = false;
        let mut coplanar_opposite = false;

        for ft_id in bvh_split_tool.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let ft_data = &brep_tool.faces[ft_id];
            let face_t = Face::from_id(brep_tool.clone(), ft_id, ft_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face.surface(), face_t.surface()) {
                if surfaces_are_coplanar(s_obj, s_tool, tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_tool, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_t) {
                        let n_obj = match s_obj {
                            GeomSurface::Plane(p) => p.normal(),
                            _ => openrcad_foundation::Dir::dz(),
                        };
                        let n_tool = match s_tool {
                            GeomSurface::Plane(p) => p.normal(),
                            _ => openrcad_foundation::Dir::dz(),
                        };
                        if n_obj.dot(&n_tool) > 0.0 {
                            coplanar_same = true;
                        } else {
                            coplanar_opposite = true;
                        }
                        break;
                    }
                }
            }
        }

        if coplanar_same {
            match op {
                BooleanOp::Fuse | BooleanOp::Common => {
                    kept_faces.push(face);
                }
                BooleanOp::Cut => {}
            }
        } else if coplanar_opposite {
            // Discard both
        } else {
            let inside = is_point_inside_solid(&pos, tool, &bvh_tool);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => !inside,
                BooleanOp::Common => inside,
            };
            if keep {
                kept_faces.push(face);
            }
        }
    }

    for (f_id, f_data) in &brep_tool.faces {
        let face = Face::from_id(brep_tool.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar = false;
        for fo_id in bvh_split_obj.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let fo_data = &brep_obj.faces[fo_id];
            let face_o = Face::from_id(brep_obj.clone(), fo_id, fo_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face_o.surface(), face.surface()) {
                if surfaces_are_coplanar(s_obj, s_tool, tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_obj, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_o) {
                        coplanar = true;
                        break;
                    }
                }
            }
        }

        if coplanar {
            // Discard tool's coincident face (already handled by object's side)
        } else {
            let inside = is_point_inside_solid(&pos, object, &bvh_obj);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => inside,
                BooleanOp::Common => inside,
            };
            if keep {
                if op == BooleanOp::Cut {
                    let reversed_face =
                        Face::from_id(brep_tool.clone(), f_id, f_data.orientation.reversed());
                    kept_faces.push(reversed_face);
                } else {
                    kept_faces.push(face);
                }
            }
        }
    }

    // 4. Sew kept faces together
    let shell = sew(&kept_faces, tol);
    Solid::new(shell)
}

fn validate_operand(input: BooleanInput, solid: &Solid) -> Result<(), BooleanError> {
    let report = solid.health_report();
    if !report.is_healthy() || !solid.is_watertight() {
        return Err(BooleanError::InvalidInput { input, report });
    }
    Ok(())
}

fn validate_output(solid: Solid) -> Result<Solid, BooleanError> {
    let report = solid.health_report();
    if !report.is_healthy() {
        return Err(BooleanError::InvalidOutput { report });
    }
    if !solid.is_watertight() {
        return Err(BooleanError::NonWatertightOutput { report });
    }
    Ok(solid)
}

/// Split `edge_id` at `pt`, but only when `pt` genuinely lies in the *interior*
/// of the finite edge segment. Skips off-segment crossings (the host curves are
/// infinite lines) and crossings that fall on an existing endpoint — both would
/// otherwise spawn zero-length / duplicate edges that break the sewn topology.
fn try_split_edge(
    builder: &mut BRepBuilder,
    edge_id: EdgeId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    pt: &Pnt,
    tol: f64,
) {
    if !builder.brep().edges.contains_key(edge_id) {
        return;
    }
    let t = project_point_on_curve(pt, curve, first, last);
    if curve.point(t).distance(pt) > 1e-6 {
        return; // off the finite segment
    }
    let ed = &builder.brep().edges[edge_id];
    let s_pt = builder.brep().vertices[ed.start].point;
    let e_pt = builder.brep().vertices[ed.end].point;
    if pt.distance(&s_pt) < tol || pt.distance(&e_pt) < tol {
        return; // coincides with an endpoint — no real split
    }
    let v = builder
        .brep_mut()
        .vertices
        .insert(openrcad_topo::arena::VertexData {
            point: *pt,
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
    builder.split_edge(edge_id, v, t);
}

/// Split every sub-face in `subfaces` along `curve`, replacing the list with the
/// resulting (possibly larger) set of descendant faces. Only these descendants
/// are ever revisited, so repeated calls across overlapping pairs stay bounded.
fn split_tracked(
    builder: &mut BRepBuilder,
    subfaces: &mut Vec<FaceId>,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    splitting_edges_map: &mut std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>>,
    tol: f64,
) {
    let mut result = Vec::with_capacity(subfaces.len());
    for &fid in subfaces.iter() {
        let (next_faces, new_edges) = crate::imprint::imprint_curve_on_face(builder, fid, curve, first, last, tol);
        if !new_edges.is_empty() {
            splitting_edges_map.entry(fid).or_default().extend(new_edges);
        }
        result.extend(next_faces);
    }
    *subfaces = result;
}

pub(crate) fn project_point_on_curve(p: &Pnt, curve: &GeomCurve, t_min: f64, t_max: f64) -> f64 {
    let t_min = if t_min.is_infinite() || t_min.is_nan() {
        -100.0
    } else {
        t_min
    };
    let t_max = if t_max.is_infinite() || t_max.is_nan() {
        100.0
    } else {
        t_max
    };
    let mut best_t = t_min;
    let mut min_dist = p.distance(&curve.point(t_min));
    let steps = 10;
    for i in 0..=steps {
        let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
        let dist = p.distance(&curve.point(t));
        if dist < min_dist {
            min_dist = dist;
            best_t = t;
        }
    }

    let mut t = best_t;
    for _ in 0..5 {
        let (pt, tangent) = curve.d1(t);
        let diff = pt - *p;
        let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
        t = (t - dt).clamp(t_min, t_max);
    }
    t
}

fn point_on_face(face: &Face) -> Pnt {
    // Preferred: the centroid of the outer loop's vertices, projected onto the
    // surface. For a convex face this is a robustly *interior* point, which the
    // ray-parity classifier needs — a near-boundary sample produces ambiguous
    // (boundary-grazing) hit counts. Only used if it actually lands inside the
    // trimming loops (non-convex faces fall through to the edge-offset probe).
    if let (Some(outer), Some(surf)) = (face.outer_wire(), face.surface()) {
        let pts: Vec<Pnt> = outer.edges().iter().map(|e| e.start().point()).collect();
        if !pts.is_empty() {
            let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
            for p in &pts {
                sum += p.coord();
            }
            let centroid = Pnt::from_xyz(sum / pts.len() as f64);
            let (cu, cv) = crate::intersect::search_nearest_parameter(surf, &centroid, (0.0, 0.0));
            let on_surf = surf.point(cu, cv);
            if on_surf.distance(&centroid) < 1e-6
                && crate::intersect::is_inside_trimming_loops(cu, cv, face)
            {
                let mut on_boundary = false;
                for wire in face.wires() {
                    for edge in wire.edges() {
                        if distance_point_to_edge(&on_surf, &edge) < 1e-4 {
                            on_boundary = true;
                            break;
                        }
                    }
                    if on_boundary {
                        break;
                    }
                }
                if !on_boundary {
                    return on_surf;
                }
            }
        }
    }

    if let Some(outer) = face.outer_wire() {
        let edges = outer.edges();
        if !edges.is_empty() {
            let e = &edges[0];
            let t_mid = 0.5 * (e.first() + e.last());
            if let Some(curve) = e.curve() {
                let mid_pt = curve.point(t_mid);
                if let Some(surf) = face.surface() {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(surf, &mid_pt, (0.0, 0.0));
                    let (_, tangent) = curve.d1(t_mid);
                    let normal = match surf {
                        GeomSurface::Plane(plane) => plane.normal(),
                        _ => {
                            let (_, su, sv) = eval_d1(surf, u, v);
                            su.cross(&sv)
                                .normalized()
                                .unwrap_or(openrcad_foundation::Dir::dz())
                        }
                    };

                    let dir1 = GeomVec::from_dir(
                        tangent
                            .cross(&openrcad_foundation::Vec::from_dir(normal))
                            .normalized()
                            .unwrap(),
                    );
                    let dir2 = -dir1;

                    let test_dist = 1e-3;
                    let p1 = mid_pt + dir1 * test_dist;
                    let (u1, v1) = crate::intersect::search_nearest_parameter(surf, &p1, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u1, v1, face) {
                        return p1;
                    }

                    let p2 = mid_pt + dir2 * test_dist;
                    let (u2, v2) = crate::intersect::search_nearest_parameter(surf, &p2, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u2, v2, face) {
                        return p2;
                    }
                }
            }
        }
    }

    let vertices = face
        .wires()
        .iter()
        .flat_map(|w| w.edges())
        .map(|e| e.start().point())
        .collect::<Vec<_>>();
    if !vertices.is_empty() {
        let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
        for v in &vertices {
            sum += v.coord();
        }
        Pnt::from_xyz(sum / vertices.len() as f64)
    } else {
        Pnt::origin()
    }
}

fn is_point_inside_solid(p: &Pnt, solid: &Solid, bvh: &Bvh) -> bool {
    // A "generic" irrational-ish direction: a symmetric diagonal like (1,1,1)
    // skewers the corners/edges of axis-aligned boxes, defeating the parity
    // test. This direction avoids hitting integer-coordinate vertices/edges.
    let mut ray_dir = Dir::new(0.182_321, 0.523_157, 0.832_511);
    let mut rng: u64 = 42;

    // Cap the perturbation retries: a point that stubbornly grazes boundaries
    // should not spin forever. After the cap we accept the last parity reading.
    for _attempt in 0..32 {
        let mut intersection_count = 0;
        let mut hit_boundary = false;

        let ray_dir_vec = GeomVec::from_dir(ray_dir);

        // Only test faces whose AABB the ray actually crosses. The BVH prunes
        // the rest, so each cast is O(log F + hits) instead of O(F), and we
        // reconstruct only the hit faces (via Face::from_id) rather than
        // materializing the whole shell's face list on every call.
        let candidate_face_ids = bvh.ray_cast(p, &ray_dir_vec);

        'faces: for fid in candidate_face_ids {
            let orientation = solid.brep().faces[fid].orientation;
            let face = Face::from_id(solid.brep().clone(), fid, orientation);
            for hit in crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7) {
                let mut on_boundary = false;
                for wire in face.wires() {
                    for edge in wire.edges() {
                        let dist_to_edge = distance_point_to_edge(&hit, &edge);
                        if dist_to_edge < 1e-5 {
                            on_boundary = true;
                            break;
                        }
                    }
                    if on_boundary {
                        break;
                    }
                }

                if on_boundary {
                    hit_boundary = true;
                    break 'faces;
                }

                intersection_count += 1;
            }
        }

        if hit_boundary {
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let rx = ((rng & 0xff) as f64 / 255.0) - 0.5;
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let ry = ((rng & 0xff) as f64 / 255.0) - 0.5;
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let rz = ((rng & 0xff) as f64 / 255.0) - 0.5;

            ray_dir =
                Dir::from_vec(&(ray_dir_vec + GeomVec::new(rx, ry, rz) * 0.1)).unwrap_or(ray_dir);
            continue;
        }

        return (intersection_count % 2) == 1;
    }

    // Retries exhausted: fall back to a final parity reading without the
    // boundary veto so we always return a definite answer.
    let ray_dir_vec = GeomVec::from_dir(ray_dir);
    let mut count = 0;
    for fid in bvh.ray_cast(p, &ray_dir_vec) {
        let orientation = solid.brep().faces[fid].orientation;
        let face = Face::from_id(solid.brep().clone(), fid, orientation);
        count += crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7).len();
    }
    (count % 2) == 1
}

fn distance_point_to_edge(p: &Pnt, edge: &openrcad_topo::Edge) -> f64 {
    if let Some(curve) = edge.curve() {
        let (t_min, t_max) = (edge.first(), edge.last());
        let mut best_t = t_min;
        let mut min_dist = p.distance(&curve.point(t_min));

        let steps = 10;
        for i in 0..=steps {
            let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
            let dist = p.distance(&curve.point(t));
            if dist < min_dist {
                min_dist = dist;
                best_t = t;
            }
        }

        let mut t = best_t;
        for _ in 0..5 {
            let (pt, tangent) = curve.d1(t);
            let diff = pt - *p;
            let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
            t = (t - dt).clamp(t_min, t_max);
        }
        p.distance(&curve.point(t))
    } else {
        p.distance(&edge.start().point())
    }
}

fn surfaces_are_coplanar(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> bool {
    match (s1, s2) {
        (GeomSurface::Plane(p1), GeomSurface::Plane(p2)) => {
            let dot = p1.normal().dot(&p2.normal());
            if (dot.abs() - 1.0).abs() > 1e-5 {
                return false;
            }
            let dist = GeomVec::from_dir(p1.normal()).dot(&(p2.location() - p1.location()));
            dist.abs() <= tol
        }
        _ => false,
    }
}

fn eval_d1(s: &GeomSurface, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec) {
    s.d1(u, v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;

    #[test]
    fn primitives_pass_structural_validation() {
        // A well-formed box: contiguous loops, Euler–Poincaré V−E+F = 8−12+6 = 2.
        let b = make_box(&Pnt::new(-1.0, 2.0, 0.5), 2.0, 3.0, 4.0);
        b.assert_valid();
        assert_eq!(b.euler_characteristic(), 2);

        // Adversarial sliver: an extreme aspect ratio (thickness 1e-4 over a
        // 100×100 footprint) is still a topologically valid closed box. Its near
        // degeneracy is exactly the kind of input that trips fragile float
        // classification, so the structural invariants must still hold.
        let sliver = make_box(&Pnt::origin(), 100.0, 100.0, 1e-4);
        sliver.assert_valid();
        assert_eq!(sliver.euler_characteristic(), 2);
    }

    #[test]
    fn test_boolean_intersection_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Common (Intersection)
        let common = boolean(&cube1, &cube2, BooleanOp::Common);
        // The intersection should be a 5x10x10 box
        // Bounding box of the intersection should be [5, 0, 0] to [10, 10, 10]
        let (lo, hi) = common.bounding_box().corners().unwrap();
        assert!((lo.x() - 5.0).abs() < 1e-5);
        assert!((lo.y() - 0.0).abs() < 1e-5);
        assert!((lo.z() - 0.0).abs() < 1e-5);
        assert!((hi.x() - 10.0).abs() < 1e-5);
        assert!((hi.y() - 10.0).abs() < 1e-5);
        assert!((hi.z() - 10.0).abs() < 1e-5);
        assert_eq!(common.face_count(), 6);
        assert_eq!(common.vertex_count(), 8);
        assert_eq!(common.edge_count(), 12);
        assert!(
            common.is_watertight(),
            "intersection result must be watertight"
        );
    }

    #[test]
    fn test_boolean_difference_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Cut (Difference): cube1 - cube2
        let cut = boolean(&cube1, &cube2, BooleanOp::Cut);
        // The result should be a 5x10x10 box from [0, 0, 0] to [5, 10, 10]
        let (lo, hi) = cut.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 5.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert_eq!(cut.face_count(), 6);
        assert_eq!(cut.vertex_count(), 8);
        assert_eq!(cut.edge_count(), 12);
        assert!(cut.is_watertight(), "difference result must be watertight");
    }

    /// Box minus a through-cylinder. This exercises the curved-boolean path:
    /// the planar caps are bored (closed-circle hole cutting) and the cylinder
    /// lateral wall is trimmed where it crosses the caps.
    ///
    /// This used to be a known failure: the cylinder's three lateral faces share
    /// seam edges, and when topology stored a single orientation per edge, cutting
    /// one shared seam corrupted the adjacent walls' loops, leaving them untrimmed
    /// so the result spilled past z∈[0,10]. That is fixed: orientation is now a
    /// per-use property of each co-edge in a loop ([`OrientedEdge`]), not of the
    /// shared edge, so splitting a shared seam keeps every loop tracing cleanly.
    /// The `hi.z ≈ 10` assertion below (the drill extends to z=11) locks that in.
    ///
    /// [`OrientedEdge`]: openrcad_topo::arena::OrientedEdge
    #[test]
    fn test_boolean_box_minus_cylinder_drills_hole() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        // A 10×10×10 box with a radius-2 cylinder running through it along Z,
        // centred at (5,5), extending below and above the box so it pierces both
        // the top and bottom caps.
        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // The outer envelope is unchanged (the drill only removes interior
        // material), so the bounding box is still the original cube.
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((lo.z() - 0.0).abs() < 1e-4, "lo.z={}", lo.z());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());
        assert!((hi.z() - 10.0).abs() < 1e-4, "hi.z={}", hi.z());

        // A through-hole adds the cylindrical wall faces plus bores the two caps,
        // so the result must have strictly more faces than the original 6.
        assert!(
            result.face_count() > 6,
            "expected a drilled solid with extra faces, got {}",
            result.face_count()
        );
        if !result.is_watertight() {
            println!("All faces in the result solid:");
            for (idx, face) in result.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!("    Edge: {:?} -> {:?}", edge.start().point(), edge.end().point());
                    }
                }
            }
        }
        assert!(result.is_watertight(), "drilled solid must be watertight");
    }

    /// The curved-boolean engine completes without hanging (the historical
    /// blocker) and bores the box's caps. A lighter-weight companion to
    /// `test_boolean_box_minus_cylinder_drills_hole`, which additionally asserts
    /// the lateral walls are trimmed to z∈[0,10].
    #[test]
    fn test_boolean_box_minus_cylinder_runs_and_drills_caps() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // Drilling removes no outer XY material: the footprint stays [0,10]².
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());

        // The caps are bored and wall faces added, so the face count grows.
        assert!(
            result.face_count() > 6,
            "expected drilling to add faces, got {}",
            result.face_count()
        );
    }

    #[test]
    fn test_boolean_union_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Fuse (Union)
        let fuse = boolean(&cube1, &cube2, BooleanOp::Fuse);
        // The result bounding box should be [0, 0, 0] to [15, 10, 10]
        let (lo, hi) = fuse.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 15.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        if !fuse.is_watertight() {
            println!("All faces in the union solid:");
            for (idx, face) in fuse.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!("    Edge: {:?} -> {:?}", edge.start().point(), edge.end().point());
                    }
                }
            }
        }
        assert!(fuse.is_watertight(), "union result must be watertight");
    }
}
