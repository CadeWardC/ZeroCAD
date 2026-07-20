//! Datum resolution: turn [`FeatureType::DatumPlane`]/[`DatumAxis`]/[`DatumPoint`]
//! nodes into concrete frames/axes/points.
//!
//! Datums resolve in a pure pre-pass before the body loop — every input is a
//! base plane, another datum, an explicit point, or an expression over the
//! document variables, never live body geometry. That keeps the eval prefix
//! cache sound with one coarse rule: every datum feature is folded into the
//! cache's hash *seed* (like the variables), so any datum edit invalidates all
//! checkpoints instead of tracking per-consumer dependencies.

use super::*;
use crate::geometry::{CoordinateSystem, Vec3};

fn v3(p: [f32; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Rotate `v` around unit axis `k` by `angle` radians (Rodrigues).
fn rotate(v: Vec3, k: Vec3, angle: f32) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v.mul(c)
        .add(k.cross(v).mul(s))
        .add(k.mul(k.dot(v) * (1.0 - c)))
}

fn resolve_plane_base(
    base: &PlaneBase,
    resolved: &HashMap<String, DatumValue>,
) -> Option<CoordinateSystem> {
    match base {
        PlaneBase::XY => Some(CoordinateSystem::XY),
        PlaneBase::XZ => Some(CoordinateSystem::XZ),
        PlaneBase::YZ => Some(CoordinateSystem::YZ),
        PlaneBase::Datum(id) => match resolved.get(id) {
            Some(DatumValue::Plane(cs)) => Some(*cs),
            _ => None,
        },
    }
}

fn resolve_axis_base(
    axis: &AxisBase,
    resolved: &HashMap<String, DatumValue>,
) -> Option<(Vec3, Vec3)> {
    match axis {
        AxisBase::X => Some((Vec3::ZERO, Vec3::X)),
        AxisBase::Y => Some((Vec3::ZERO, Vec3::Y)),
        AxisBase::Z => Some((Vec3::ZERO, Vec3::Z)),
        AxisBase::Datum(id) => match resolved.get(id) {
            Some(DatumValue::Axis { origin, dir }) => Some((*origin, *dir)),
            _ => None,
        },
        AxisBase::TwoPoints { a, b } => {
            let dir = v3(*b).sub(v3(*a)).normalize();
            (dir != Vec3::ZERO).then_some((v3(*a), dir))
        }
    }
}

/// A dimension that may be expression-driven: resolve `expr` against `vars`,
/// falling back to the stored literal (and pushing a warning) when it no
/// longer evaluates — the same fail-loud rule as extrude depth.
fn resolve_dim(
    literal: f32,
    expr: Option<&String>,
    vars: &HashMap<String, f64>,
    node_id: &str,
    what: &str,
    warnings: &mut Vec<String>,
) -> f32 {
    match expr {
        Some(e) => match crate::expr::eval(e, vars) {
            Ok(v) => v as f32,
            Err(_) => {
                warnings.push(format!(
                    "Datum '{node_id}': {what} expression \"{e}\" no longer evaluates; \
                     using last value {literal:.3}."
                ));
                literal
            }
        },
        None => literal,
    }
}

impl ParametricGraph {
    pub(crate) fn resolve_datum_node(
        &self,
        node: &FeatureRecord,
        resolved: &HashMap<String, DatumValue>,
        vars: &HashMap<String, f64>,
        live: &[LiveBody],
        warnings: &mut Vec<String>,
    ) -> Option<DatumValue> {
        match &node.feature {
            FeatureType::DatumPlane { def } => match def {
                DatumPlaneDef::PlanarFace { face } => {
                    resolve_planar_face(face, live).map(DatumValue::Plane)
                }
                _ => self
                    .resolve_plane_def(def, resolved, vars, &node.id, warnings)
                    .map(DatumValue::Plane),
            },
            FeatureType::DatumAxis { def } => match def {
                DatumAxisDef::TwoPoints { a, b } => axis_through(v3(*a), v3(*b))
                    .map(|(origin, dir)| DatumValue::Axis { origin, dir }),
                DatumAxisDef::PlaneIntersection { a, b } => {
                    let ca = resolve_plane_base(a, resolved);
                    let cb = resolve_plane_base(b, resolved);
                    match (ca, cb) {
                        (Some(ca), Some(cb)) => plane_intersection_axis(&ca, &cb)
                            .map(|(origin, dir)| DatumValue::Axis { origin, dir }),
                        _ => None,
                    }
                }
                DatumAxisDef::Edge { edge } => {
                    let (_, edge) = resolve_edge_at_history(edge, live)?;
                    axis_from_edge(&edge).map(|(origin, dir)| DatumValue::Axis { origin, dir })
                }
                DatumAxisDef::CylindricalOrConicalFace { face } => {
                    let (body, resolved_face) = resolve_face_at_history(face, live)?;
                    axial_face_axis(body, &resolved_face)
                        .map(|(origin, dir)| DatumValue::Axis { origin, dir })
                }
                DatumAxisDef::TwoVertices { a, b } => {
                    let a = resolve_vertex_at_history(a, live)?;
                    let b = resolve_vertex_at_history(b, live)?;
                    axis_through(a, b).map(|(origin, dir)| DatumValue::Axis { origin, dir })
                }
            },
            FeatureType::DatumPoint { def } => match def {
                DatumPointDef::Coords { p } => Some(DatumValue::Point(v3(*p))),
                DatumPointDef::Vertex { vertex } => {
                    resolve_vertex_at_history(vertex, live).map(DatumValue::Point)
                }
                DatumPointDef::Midpoint { a, b } => {
                    let a = resolve_vertex_at_history(a, live)?;
                    let b = resolve_vertex_at_history(b, live)?;
                    Some(DatumValue::Point(a.add(b).mul(0.5)))
                }
                DatumPointDef::EdgeMidpoint { edge } => {
                    let (_, edge) = resolve_edge_at_history(edge, live)?;
                    edge_midpoint(&edge).map(DatumValue::Point)
                }
                DatumPointDef::CircleCenter { edge } => {
                    let (_, edge) = resolve_edge_at_history(edge, live)?;
                    let crate::mock_kernel::EdgeCurveHint::Circle { center, .. } = edge.curve?
                    else {
                        return None;
                    };
                    Some(DatumValue::Point(v3(center)))
                }
            },
            _ => None,
        }
    }

    /// Resolve every datum node into a [`DatumValue`], in creation order (so a
    /// datum may reference any earlier datum). A datum whose inputs are missing
    /// or degenerate resolves to nothing and pushes a warning — consumers then
    /// report Unresolved rather than silently landing on a wrong plane.
    pub fn resolve_datums(
        &self,
        vars: &HashMap<String, f64>,
        warnings: &mut Vec<String>,
    ) -> HashMap<String, DatumValue> {
        let mut nodes: Vec<_> = self
            .graph
            .node_indices()
            .filter(|&i| {
                if self.is_feature_suppressed(&self.graph[i].id) {
                    return false;
                }
                matches!(
                    self.graph[i].feature,
                    FeatureType::DatumPlane { .. }
                        | FeatureType::DatumAxis { .. }
                        | FeatureType::DatumPoint { .. }
                )
            })
            .collect();
        nodes.sort_by_key(|&i| {
            self.feature_sequence(&self.graph[i].id)
                .map(|key| key.0)
                .unwrap_or_else(|| creation_key(&self.graph[i].id))
        });

        let mut resolved: HashMap<String, DatumValue> = HashMap::new();
        for idx in nodes {
            let node = &self.graph[idx];
            let value = self.resolve_datum_node(node, &resolved, vars, &[], warnings);
            match value {
                Some(v) => {
                    resolved.insert(node.id.clone(), v);
                }
                None => warnings.push(format!(
                    "Datum '{}' ({}) could not be resolved from its inputs.",
                    node.id, node.name
                )),
            }
        }
        resolved
    }

    fn resolve_plane_def(
        &self,
        def: &DatumPlaneDef,
        resolved: &HashMap<String, DatumValue>,
        vars: &HashMap<String, f64>,
        node_id: &str,
        warnings: &mut Vec<String>,
    ) -> Option<CoordinateSystem> {
        match def {
            DatumPlaneDef::Offset {
                base,
                distance,
                distance_expr,
            } => {
                let cs = resolve_plane_base(base, resolved)?;
                let d = resolve_dim(
                    *distance,
                    distance_expr.as_ref(),
                    vars,
                    node_id,
                    "distance",
                    warnings,
                );
                // Offset along the STORED normal (with_origin keeps the axes):
                // the ground plane XZ is left-handed, and following a recomputed
                // u×v there would offset "up" downward.
                Some(cs.with_origin(cs.origin.add(cs.n.mul(d))))
            }
            DatumPlaneDef::Angle {
                base,
                axis,
                angle_deg,
                angle_expr,
            } => {
                let cs = resolve_plane_base(base, resolved)?;
                let (axis_origin, axis_dir) = resolve_axis_base(axis, resolved)?;
                let deg = resolve_dim(
                    *angle_deg,
                    angle_expr.as_ref(),
                    vars,
                    node_id,
                    "angle",
                    warnings,
                );
                let rad = deg.to_radians();
                // Rotate the frame's axes, and orbit its origin around the axis
                // line. Handedness is preserved: all three stored axes rotate
                // rigidly, none is recomputed.
                let rel = cs.origin.sub(axis_origin);
                Some(CoordinateSystem {
                    origin: axis_origin.add(rotate(rel, axis_dir, rad)),
                    u: rotate(cs.u, axis_dir, rad),
                    v: rotate(cs.v, axis_dir, rad),
                    n: rotate(cs.n, axis_dir, rad),
                })
            }
            DatumPlaneDef::ThreePoints { a, b, c } => {
                let (a, b, c) = (v3(*a), v3(*b), v3(*c));
                let u = b.sub(a).normalize();
                if u == Vec3::ZERO {
                    return None;
                }
                let w = c.sub(a);
                let v = w.sub(u.mul(w.dot(u))).normalize();
                if v == Vec3::ZERO {
                    return None;
                }
                Some(CoordinateSystem::new(a, u, v))
            }
            DatumPlaneDef::MidPlane { a, b } => {
                let ca = resolve_plane_base(a, resolved)?;
                let cb = resolve_plane_base(b, resolved)?;
                // Midway along a's normal between the two origins; a's axes.
                let mid = ca.origin.add(cb.origin).mul(0.5);
                Some(ca.with_origin(mid))
            }
            DatumPlaneDef::PlanarFace { .. } => None,
        }
    }
}

fn axis_through(a: Vec3, b: Vec3) -> Option<(Vec3, Vec3)> {
    let dir = b.sub(a).normalize();
    (dir != Vec3::ZERO).then_some((a, dir))
}

fn resolve_face_at_history<'a>(
    face: &FaceRef,
    live: &'a [LiveBody],
) -> Option<(&'a LiveBody, FaceRef)> {
    if let Some(topology) = face.topology.as_ref() {
        if let Some(body_id) = topology.body_id.as_deref() {
            let body = live.iter().find(|body| body.id == body_id)?;
            let resolved = if topology.face_id.is_some() {
                resolve_face_ref_by_topology(body, face)?
            } else {
                resolve_legacy_face_ref_unique(body, face)?
            };
            return Some((body, resolved));
        }
        if topology.face_id.is_some() {
            return None;
        }
    }
    let mut matches = live.iter().filter_map(|body| {
        resolve_legacy_face_ref_unique(body, face).map(|resolved| (body, resolved))
    });
    let result = matches.next()?;
    matches.next().is_none().then_some(result)
}

fn resolve_edge_at_history<'a>(
    edge: &EdgeRef,
    live: &'a [LiveBody],
) -> Option<(&'a LiveBody, EdgeRef)> {
    if let Some(topology) = edge.topology.as_ref() {
        let has_durable_name = topology
            .edge_id
            .as_deref()
            .is_some_and(|id| crate::parametric::topo_name::TopoName::parse(id).is_durable());
        if let Some(body_id) = topology.body_id.as_deref() {
            let body = live.iter().find(|body| body.id == body_id)?;
            let resolved = if has_durable_name {
                resolve_edge_ref_by_topology(body, edge)
                    .or_else(|| resolve_named_edge_group(body, edge))?
            } else {
                resolve_legacy_edge_ref_unique(body, edge)?
            };
            return Some((body, resolved));
        }
        if has_durable_name {
            return None;
        }
    }
    let mut matches = live.iter().filter_map(|body| {
        resolve_legacy_edge_ref_unique(body, edge).map(|resolved| (body, resolved))
    });
    let result = matches.next()?;
    matches.next().is_none().then_some(result)
}

fn resolve_named_edge_group(body: &LiveBody, requested: &EdgeRef) -> Option<EdgeRef> {
    let requested_id = requested.topology.as_ref()?.edge_id.as_deref()?;
    let rebuilt;
    let mesh = if let Some(pristine) = body.pristine.as_deref() {
        pristine
    } else {
        rebuilt = edge_mod_reference_mesh(body);
        &rebuilt
    };
    let circle_score = |candidate: &crate::mock_kernel::MeshEdgeRef| match (
        requested.curve.as_ref(),
        candidate.curve.as_ref(),
    ) {
        (
            Some(crate::mock_kernel::EdgeCurveHint::Circle {
                center: requested_center,
                radius: requested_radius,
                ..
            }),
            Some(crate::mock_kernel::EdgeCurveHint::Circle { center, radius, .. }),
        ) => point_distance(*requested_center, *center) + (requested_radius - radius).abs(),
        _ => {
            let forward = point_distance(requested.p0, candidate.p0)
                + point_distance(requested.p1, candidate.p1);
            let reverse = point_distance(requested.p0, candidate.p1)
                + point_distance(requested.p1, candidate.p0);
            forward.min(reverse)
        }
    };
    let mut candidates: Vec<_> = mesh
        .edge_refs
        .iter()
        .filter(|candidate| {
            candidate
                .topology
                .as_ref()
                .and_then(|topology| topology.edge_id.as_deref())
                == Some(requested_id)
        })
        .collect();
    candidates.sort_by(|left, right| circle_score(left).total_cmp(&circle_score(right)));
    let best = *candidates.first()?;
    if let Some(second) = candidates.get(1) {
        let equivalent_circle = matches!(
            (best.curve.as_ref(), second.curve.as_ref()),
            (
                Some(crate::mock_kernel::EdgeCurveHint::Circle {
                    center: first_center,
                    radius: first_radius,
                    ..
                }),
                Some(crate::mock_kernel::EdgeCurveHint::Circle {
                    center: second_center,
                    radius: second_radius,
                    ..
                })
            ) if point_distance(*first_center, *second_center) <= 1.0e-6
                && (first_radius - second_radius).abs() <= 1.0e-6
        );
        if !equivalent_circle && (circle_score(second) - circle_score(best)).abs() <= 1.0e-6 {
            return None;
        }
    }
    Some(EdgeRef {
        p0: best.p0,
        p1: best.p1,
        n1: best.n1,
        n2: best.n2,
        curve: best.curve.clone(),
        topology: best.topology.as_ref().map(|topology| TopologyEdgeRef {
            body_id: topology
                .body_id
                .clone()
                .or_else(|| Some(body.id.to_string())),
            topology_version: topology.topology_version,
            edge_id: topology.edge_id.clone(),
            adjacent_face_ids: topology.adjacent_face_ids.clone(),
            curve_kind: topology.curve_kind.clone(),
            adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    })
}

fn resolve_planar_face(face: &FaceRef, live: &[LiveBody]) -> Option<CoordinateSystem> {
    let (body, resolved) = resolve_face_at_history(face, live)?;
    let planar = body.parts.iter().any(|part| {
        !crate::mock_kernel::kernel_faces_matching(part, resolved.centroid, resolved.normal)
            .is_empty()
    });
    planar.then(|| coordinate_system_from_face(resolved.centroid, resolved.normal))
}

fn coordinate_system_from_face(centroid: [f32; 3], normal: [f32; 3]) -> CoordinateSystem {
    let n = v3(normal).normalize();
    let mut u = Vec3::Y.cross(n).normalize();
    if u == Vec3::ZERO {
        u = Vec3::X.cross(n).normalize();
    }
    let v = n.cross(u).normalize();
    CoordinateSystem::new(v3(centroid), u, v)
}

fn axis_from_edge(edge: &EdgeRef) -> Option<(Vec3, Vec3)> {
    match edge.curve.as_ref() {
        Some(crate::mock_kernel::EdgeCurveHint::Circle { center, axis, .. }) => {
            let dir = v3(*axis).normalize();
            (dir != Vec3::ZERO).then_some((v3(*center), dir))
        }
        None | Some(crate::mock_kernel::EdgeCurveHint::Line) => {
            axis_through(v3(edge.p0), v3(edge.p1))
        }
    }
}

fn edge_midpoint(edge: &EdgeRef) -> Option<Vec3> {
    match edge.curve.as_ref() {
        Some(crate::mock_kernel::EdgeCurveHint::Circle {
            center,
            axis,
            x_dir,
            radius,
            start,
            end,
            closed,
        }) => {
            if *closed {
                return None;
            }
            let angle = 0.5 * (*start + *end);
            let (sin, cos) = angle.sin_cos();
            let x_dir = v3(*x_dir).normalize();
            let y_dir = v3(*axis).normalize().cross(x_dir);
            Some(
                v3(*center)
                    .add(x_dir.mul(*radius * cos))
                    .add(y_dir.mul(*radius * sin)),
            )
        }
        None | Some(crate::mock_kernel::EdgeCurveHint::Line) => {
            Some(v3(edge.p0).add(v3(edge.p1)).mul(0.5))
        }
    }
}

fn axial_face_axis(body: &LiveBody, face: &FaceRef) -> Option<(Vec3, Vec3)> {
    let point = face.centroid;
    let radial_error = |origin: [f32; 3], dir: [f32; 3], radius: f32| {
        let relative = v3(point).sub(v3(origin));
        let direction = v3(dir).normalize();
        (relative
            .sub(direction.mul(relative.dot(direction)))
            .length()
            - radius.abs())
        .abs()
    };
    let mut candidates = Vec::new();
    for part in &body.parts {
        if let Some(info) = crate::mock_kernel::cylinder_face_near(part, point) {
            candidates.push((
                radial_error(info.origin, info.dir, info.radius),
                v3(info.origin),
                v3(info.dir).normalize(),
            ));
        }
        if let Some(info) = crate::mock_kernel::cone_face_near(part, point) {
            let relative = v3(point).sub(v3(info.origin));
            let direction = v3(info.dir).normalize();
            let axial = relative.dot(direction);
            candidates.push((
                radial_error(info.origin, info.dir, info.radius_at(axial)),
                v3(info.origin),
                direction,
            ));
        }
    }
    candidates
        .into_iter()
        .min_by(|left, right| left.0.total_cmp(&right.0))
        .and_then(|(_, origin, dir)| (dir != Vec3::ZERO).then_some((origin, dir)))
}

fn resolve_vertex_at_history(vertex: &VertexRef, live: &[LiveBody]) -> Option<Vec3> {
    if let Some(topology) = vertex.topology.as_ref() {
        if !topology.incident_face_ids.is_empty() || !topology.incident_edge_ids.is_empty() {
            let body_id = topology.body_id.as_deref()?;
            let body = live.iter().find(|body| body.id == body_id)?;
            let rebuilt;
            let mesh = if let Some(pristine) = body.pristine.as_deref() {
                pristine
            } else {
                rebuilt = edge_mod_reference_mesh(body);
                &rebuilt
            };
            if !topology.incident_face_ids.is_empty() {
                let mut requested = topology.incident_face_ids.clone();
                requested.sort();
                requested.dedup();
                let scale = mesh
                    .edge_refs
                    .iter()
                    .flat_map(|edge| [edge.p0, edge.p1])
                    .flatten()
                    .map(f32::abs)
                    .fold(1.0_f32, f32::max);
                let tolerance = scale * f32::EPSILON * 512.0 + 1.0e-5;
                let mut candidates = Vec::new();
                for point in mesh.edge_refs.iter().flat_map(|edge| [edge.p0, edge.p1]) {
                    let mut incident_faces: Vec<_> = mesh
                        .edge_refs
                        .iter()
                        .filter(|edge| {
                            point_distance(point, edge.p0) <= tolerance
                                || point_distance(point, edge.p1) <= tolerance
                        })
                        .flat_map(|edge| {
                            edge.topology
                                .as_ref()
                                .into_iter()
                                .flat_map(|topology| topology.adjacent_face_ids.iter().cloned())
                        })
                        .collect();
                    incident_faces.sort();
                    incident_faces.dedup();
                    if requested.iter().all(|face| incident_faces.contains(face))
                        && !candidates
                            .iter()
                            .any(|candidate| point_distance(*candidate, point) <= tolerance)
                    {
                        candidates.push(point);
                    }
                }
                candidates.sort_by(|left, right| {
                    point_distance(*left, vertex.point)
                        .total_cmp(&point_distance(*right, vertex.point))
                });
                let point = *candidates.first()?;
                if candidates.get(1).is_some_and(|second| {
                    (point_distance(*second, vertex.point) - point_distance(point, vertex.point))
                        .abs()
                        <= tolerance
                }) {
                    return None;
                }
                return Some(v3(point));
            }
            let mut ids = topology.incident_edge_ids.clone();
            ids.sort();
            ids.dedup();
            if ids.len() < 2 {
                return None;
            }
            let named_edge_groups: Vec<Vec<_>> = ids
                .iter()
                .map(|id| {
                    let matches: Vec<_> = mesh
                        .edge_refs
                        .iter()
                        .filter(|edge| {
                            edge.topology
                                .as_ref()
                                .and_then(|topology| topology.edge_id.as_deref())
                                == Some(id.as_str())
                        })
                        .collect();
                    (!matches.is_empty()).then_some(matches)
                })
                .collect::<Option<Vec<_>>>()?;
            let scale = named_edge_groups
                .iter()
                .flatten()
                .flat_map(|edge| [edge.p0, edge.p1])
                .flatten()
                .map(f32::abs)
                .fold(1.0_f32, f32::max);
            let tolerance = scale * f32::EPSILON * 512.0 + 1.0e-5;
            let mut candidates = Vec::new();
            for point in named_edge_groups[0]
                .iter()
                .flat_map(|edge| [edge.p0, edge.p1])
            {
                if named_edge_groups[1..].iter().all(|group| {
                    group.iter().any(|edge| {
                        point_distance(point, edge.p0) <= tolerance
                            || point_distance(point, edge.p1) <= tolerance
                    })
                }) && !candidates
                    .iter()
                    .any(|candidate| point_distance(*candidate, point) <= tolerance)
                {
                    candidates.push(point);
                }
            }
            candidates.sort_by(|left, right| {
                point_distance(*left, vertex.point).total_cmp(&point_distance(*right, vertex.point))
            });
            let point = *candidates.first()?;
            if candidates.get(1).is_some_and(|second| {
                (point_distance(*second, vertex.point) - point_distance(point, vertex.point)).abs()
                    <= tolerance
            }) {
                return None;
            }
            return Some(v3(point));
        }
    }
    let scale = vertex
        .point
        .iter()
        .copied()
        .map(f32::abs)
        .fold(1.0_f32, f32::max);
    let tolerance = scale * f32::EPSILON * 512.0 + 1.0e-5;
    let mut points = Vec::new();
    for body in live {
        let mesh = edge_mod_reference_mesh(body);
        for point in mesh.edge_refs.iter().flat_map(|edge| [edge.p0, edge.p1]) {
            if point_distance(point, vertex.point) <= tolerance
                && !points
                    .iter()
                    .any(|candidate| point_distance(*candidate, point) <= tolerance)
            {
                points.push(point);
            }
        }
    }
    (points.len() == 1).then(|| v3(points[0]))
}

fn point_distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    v3(a).sub(v3(b)).length()
}

/// Resolve an [`AxisBase`] against already-resolved datums to a world-space
/// `(origin, unit direction)`. Shared by revolve axes and pattern directions.
pub(crate) fn resolve_axis_base_world(
    axis: &AxisBase,
    datums: &HashMap<String, DatumValue>,
) -> Option<(Vec3, Vec3)> {
    resolve_axis_base(axis, datums)
}

/// Resolve a [`PlaneBase`] against already-resolved datums to a world frame.
pub(crate) fn resolve_plane_base_world(
    plane: &PlaneBase,
    datums: &HashMap<String, DatumValue>,
) -> Option<CoordinateSystem> {
    resolve_plane_base(plane, datums)
}

/// The intersection line of two planes, or `None` when (near-)parallel.
fn plane_intersection_axis(a: &CoordinateSystem, b: &CoordinateSystem) -> Option<(Vec3, Vec3)> {
    let dir = a.n.cross(b.n).normalize();
    if dir == Vec3::ZERO {
        return None;
    }
    // A point on both planes: solve within the span of the two normals.
    // p = c1*n1 + c2*n2 with n1·p = d1, n2·p = d2.
    let d1 = a.n.dot(a.origin);
    let d2 = b.n.dot(b.origin);
    let n1n2 = a.n.dot(b.n);
    let det = 1.0 - n1n2 * n1n2;
    if det.abs() < 1e-12 {
        return None;
    }
    let c1 = (d1 - d2 * n1n2) / det;
    let c2 = (d2 - d1 * n1n2) / det;
    Some((a.n.mul(c1).add(b.n.mul(c2)), dir))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane_of(g: &ParametricGraph, id: &str) -> CoordinateSystem {
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        match &datums[id] {
            DatumValue::Plane(cs) => *cs,
            other => panic!("expected plane, got {other:?}"),
        }
    }

    fn add_plane(g: &mut ParametricGraph, id: &str, def: DatumPlaneDef) {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::DatumPlane { def },
        });
    }

    #[test]
    fn offset_follows_stored_normal_even_left_handed() {
        // XZ (ground) is LEFT-handed: stored n = +Y, u×v = −Y. Offsetting +5
        // must move UP (+Y), not follow a recomputed u×v.
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::XZ,
                distance: 5.0,
                distance_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_1");
        assert_eq!(cs.origin, Vec3::new(0.0, 5.0, 0.0));
        assert_eq!(cs.n, Vec3::Y);
        assert_eq!(cs.u, CoordinateSystem::XZ.u);
        assert_eq!(cs.v, CoordinateSystem::XZ.v);
    }

    #[test]
    fn datum_chain_resolves_in_creation_order() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 2.0,
                distance_expr: None,
            },
        );
        add_plane(
            &mut g,
            "datum_2",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_1".to_string()),
                distance: 3.0,
                distance_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_2");
        assert_eq!(cs.origin, Vec3::new(0.0, 0.0, 5.0));
    }

    #[test]
    fn angle_plane_rotates_rigidly() {
        // XY rotated 90° about the X axis: n goes +Z → −Y (right-hand rule),
        // u stays +X.
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Angle {
                base: PlaneBase::XY,
                axis: AxisBase::X,
                angle_deg: 90.0,
                angle_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_1");
        let close = |a: Vec3, b: Vec3| a.sub(b).length() < 1e-5;
        assert!(close(cs.u, Vec3::X), "u = {:?}", cs.u);
        assert!(close(cs.v, Vec3::Z), "v = {:?}", cs.v);
        assert!(close(cs.n, Vec3::new(0.0, -1.0, 0.0)), "n = {:?}", cs.n);
    }

    #[test]
    fn three_point_plane_and_midplane() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::ThreePoints {
                a: [0.0, 0.0, 1.0],
                b: [1.0, 0.0, 1.0],
                c: [0.0, 1.0, 1.0],
            },
        );
        add_plane(
            &mut g,
            "datum_2",
            DatumPlaneDef::MidPlane {
                a: PlaneBase::XY,
                b: PlaneBase::Datum("datum_1".to_string()),
            },
        );
        let p1 = plane_of(&g, "datum_1");
        assert_eq!(p1.n, Vec3::Z);
        let p2 = plane_of(&g, "datum_2");
        assert_eq!(p2.origin, Vec3::new(0.0, 0.0, 0.5));
    }

    #[test]
    fn axis_from_plane_intersection() {
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "datumaxis_1".to_string(),
            name: "axis".to_string(),
            feature: FeatureType::DatumAxis {
                def: DatumAxisDef::PlaneIntersection {
                    a: PlaneBase::XY,
                    b: PlaneBase::XZ,
                },
            },
        });
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        let DatumValue::Axis { origin, dir } = &datums["datumaxis_1"] else {
            panic!("expected axis");
        };
        // XY (n=+Z) ∩ XZ (n=+Y) = the X axis.
        assert!(dir.y.abs() < 1e-6 && dir.z.abs() < 1e-6 && dir.x.abs() > 0.99);
        assert!(origin.y.abs() < 1e-6 && origin.z.abs() < 1e-6);
    }

    #[test]
    fn unresolvable_datum_warns() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("nonexistent".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        );
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        assert!(datums.is_empty());
        assert_eq!(warnings.len(), 1);
    }

    fn box_graph() -> ParametricGraph {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 12.0,
                h: 8.0,
                d: 6.0,
            },
        });
        graph
    }

    fn captured_face(face: &crate::mock_kernel::MeshFaceRef) -> FaceRef {
        FaceRef {
            centroid: face.centroid,
            normal: face.normal,
            topology: face.topology.as_ref().map(|topology| TopologyFaceRef {
                body_id: topology.body_id.clone(),
                component_id: topology.component_id.clone(),
                topology_version: topology.topology_version,
                face_id: topology.face_id.clone(),
                surface_kind: topology.surface_kind.clone(),
                producer_feature_id: topology.producer_feature_id.clone(),
                source_entity_id: topology.source_entity_id.clone(),
            }),
        }
    }

    fn captured_edge(body_id: &str, edge: &crate::mock_kernel::MeshEdgeRef) -> EdgeRef {
        EdgeRef {
            p0: edge.p0,
            p1: edge.p1,
            n1: edge.n1,
            n2: edge.n2,
            curve: edge.curve.clone(),
            topology: edge.topology.as_ref().map(|topology| TopologyEdgeRef {
                body_id: topology
                    .body_id
                    .clone()
                    .or_else(|| Some(body_id.to_string())),
                topology_version: topology.topology_version,
                edge_id: topology.edge_id.clone(),
                adjacent_face_ids: topology.adjacent_face_ids.clone(),
                curve_kind: topology.curve_kind.clone(),
                adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
                producer_feature_id: topology.producer_feature_id.clone(),
                source_entity_id: topology.source_entity_id.clone(),
            }),
        }
    }

    fn evaluated_mesh(graph: &ParametricGraph) -> MockMesh {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("evaluate geometry datum fixture");
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        bodies.into_iter().next().expect("fixture body").1
    }

    fn cached_datum(graph: &ParametricGraph, id: &str) -> Option<DatumValue> {
        graph
            .eval_cache
            .borrow()
            .checkpoints
            .iter()
            .rev()
            .flatten()
            .find_map(|checkpoint| checkpoint.datums.get(id).cloned())
    }

    #[test]
    fn planar_face_datum_resolves_at_its_history_checkpoint_and_reuses_warm() {
        let mut graph = box_graph();
        let mesh = evaluated_mesh(&graph);
        let top = mesh
            .face_refs
            .iter()
            .max_by(|left, right| left.centroid[2].total_cmp(&right.centroid[2]))
            .expect("top face");
        graph.add_feature(FeatureNode {
            id: "datum_plane_1".to_string(),
            name: "Top plane".to_string(),
            feature: FeatureType::DatumPlane {
                def: DatumPlaneDef::PlanarFace {
                    face: captured_face(top),
                },
            },
        });
        graph.add_dependency("box_1", "datum_plane_1");

        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("cold geometry datum evaluation");
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        let Some(DatumValue::Plane(cold)) = cached_datum(&graph, "datum_plane_1") else {
            panic!("missing planar datum checkpoint");
        };
        assert!((cold.origin.z - 6.0).abs() < 1.0e-5);

        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("warm geometry datum evaluation");
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        assert_eq!(
            cached_datum(&graph, "datum_plane_1"),
            Some(DatumValue::Plane(cold))
        );
        let current = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
        let output = graph
            .evaluate_request(
                &std::collections::HashSet::new(),
                EvaluationQuality::Final,
                &EvaluationCancellation::new(1, current),
            )
            .expect("datum-bearing evaluator output");
        assert_eq!(
            output.datums.get("datum_plane_1"),
            Some(&DatumValue::Plane(cold))
        );
    }

    #[test]
    fn edge_axis_and_incident_edge_vertex_resolve_durably() {
        let mut graph = box_graph();
        let mesh = evaluated_mesh(&graph);
        let named: Vec<_> = mesh
            .edge_refs
            .iter()
            .filter(|edge| {
                edge.topology
                    .as_ref()
                    .and_then(|topology| topology.edge_id.as_ref())
                    .is_some()
            })
            .collect();
        let (first, second, shared_point) = named
            .iter()
            .enumerate()
            .find_map(|(index, first)| {
                named[index + 1..].iter().find_map(|second| {
                    let mut shared = [first.p0, first.p1].into_iter().filter(|point| {
                        point_distance(*point, second.p0) < 1.0e-5
                            || point_distance(*point, second.p1) < 1.0e-5
                    });
                    let point = shared.next()?;
                    shared.next().is_none().then_some((*first, *second, point))
                })
            })
            .expect("two uniquely incident named edges");
        let incident_edge_ids = [first, second]
            .into_iter()
            .map(|edge| {
                edge.topology
                    .as_ref()
                    .and_then(|topology| topology.edge_id.clone())
                    .expect("durable edge id")
            })
            .collect();
        let mut incident_face_ids: Vec<_> = [first, second]
            .into_iter()
            .flat_map(|edge| {
                edge.topology
                    .as_ref()
                    .into_iter()
                    .flat_map(|topology| topology.adjacent_face_ids.iter().cloned())
            })
            .collect();
        incident_face_ids.sort();
        incident_face_ids.dedup();
        let vertex = VertexRef {
            point: shared_point,
            topology: Some(TopologyVertexRef {
                body_id: Some("box_1".to_string()),
                incident_edge_ids,
                incident_face_ids,
                producer_feature_id: Some("box_1".to_string()),
                ..TopologyVertexRef::default()
            }),
        };
        graph.add_feature(FeatureNode {
            id: "datum_axis_1".to_string(),
            name: "Edge axis".to_string(),
            feature: FeatureType::DatumAxis {
                def: DatumAxisDef::Edge {
                    edge: captured_edge("box_1", first),
                },
            },
        });
        graph.add_dependency("box_1", "datum_axis_1");
        graph.add_feature(FeatureNode {
            id: "datum_point_1".to_string(),
            name: "Corner point".to_string(),
            feature: FeatureType::DatumPoint {
                def: DatumPointDef::Vertex { vertex },
            },
        });
        graph.add_dependency("box_1", "datum_point_1");

        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("edge and vertex datums");
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        assert!(matches!(
            cached_datum(&graph, "datum_axis_1"),
            Some(DatumValue::Axis { .. })
        ));
        assert_eq!(
            cached_datum(&graph, "datum_point_1"),
            Some(DatumValue::Point(v3(shared_point)))
        );
    }

    #[test]
    fn missing_durable_face_name_suspends_geometry_datum() {
        let mut graph = box_graph();
        let mesh = evaluated_mesh(&graph);
        let mut face = captured_face(mesh.face_refs.first().expect("box face"));
        face.topology.as_mut().expect("named face").face_id =
            Some("missing:durable:face".to_string());
        graph.add_feature(FeatureNode {
            id: "datum_plane_1".to_string(),
            name: "Lost plane".to_string(),
            feature: FeatureType::DatumPlane {
                def: DatumPlaneDef::PlanarFace { face },
            },
        });
        graph.add_dependency("box_1", "datum_plane_1");

        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("topology loss remains non-fatal");
        assert!(!warnings.is_empty());
        assert_eq!(cached_datum(&graph, "datum_plane_1"), None);
    }

    #[test]
    fn cylinder_face_axis_and_circle_center_keep_analytic_geometry() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "cylinder_1".to_string(),
            name: "Cylinder".to_string(),
            feature: FeatureType::Cylinder { r: 5.0, h: 14.0 },
        });
        let mesh = evaluated_mesh(&graph);
        let wall = mesh
            .face_refs
            .iter()
            .find(|face| face.normal[2].abs() < 0.5)
            .expect("cylindrical wall");
        let rim = mesh
            .edge_refs
            .iter()
            .find(|edge| {
                matches!(
                    edge.curve.as_ref(),
                    Some(crate::mock_kernel::EdgeCurveHint::Circle { .. })
                )
            })
            .expect("analytic circular rim");
        let rim_center = match rim.curve.as_ref().expect("circle hint") {
            crate::mock_kernel::EdgeCurveHint::Circle { center, .. } => *center,
            _ => unreachable!(),
        };
        graph.add_feature(FeatureNode {
            id: "datum_axis_1".to_string(),
            name: "Cylinder axis".to_string(),
            feature: FeatureType::DatumAxis {
                def: DatumAxisDef::CylindricalOrConicalFace {
                    face: captured_face(wall),
                },
            },
        });
        graph.add_dependency("cylinder_1", "datum_axis_1");
        graph.add_feature(FeatureNode {
            id: "datum_point_1".to_string(),
            name: "Rim center".to_string(),
            feature: FeatureType::DatumPoint {
                def: DatumPointDef::CircleCenter {
                    edge: captured_edge("cylinder_1", rim),
                },
            },
        });
        graph.add_dependency("cylinder_1", "datum_point_1");

        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("analytic geometry datums");
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        let Some(DatumValue::Axis { origin, dir }) = cached_datum(&graph, "datum_axis_1") else {
            panic!("missing cylindrical datum axis");
        };
        assert!(origin.x.abs() < 1.0e-5 && origin.z.abs() < 1.0e-5);
        assert!(dir.y.abs() > 0.999);
        let Some(DatumValue::Point(center)) = cached_datum(&graph, "datum_point_1") else {
            panic!("missing circular-rim datum point");
        };
        assert!(
            center.sub(v3(rim_center)).length() <= 1.0e-4,
            "circle-center datum {center:?} drifted from captured analytic center {rim_center:?}"
        );
    }

    #[test]
    fn datum_dependency_cycle_is_rejected_before_geometry_resolution() {
        let mut graph = ParametricGraph::new();
        add_plane(
            &mut graph,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_2".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        );
        add_plane(
            &mut graph,
            "datum_2",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_1".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        );
        graph.add_dependency("datum_1", "datum_2");
        graph.add_dependency("datum_2", "datum_1");
        let error = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect_err("datum cycle must fail graph validation");
        assert!(error.contains("Circular dependency"), "error: {error}");
    }
}
