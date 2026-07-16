use super::*;

/// Atomic Combine/Common service. It shares the same OpenRCAD operation
/// boundary, strict representation gate, and face-naming adapter as Cut.
pub(crate) fn apply_body_intersect(
    node_id: &str,
    target: &str,
    tool: &str,
    keep_tool: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if target == tool {
        warnings.push(format!(
            "Body intersect '{node_id}': target and tool must be different."
        ));
        return;
    }
    let Some(target_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "Body intersect '{node_id}': target body '{target}' no longer exists."
        ));
        return;
    };
    let Some(tool_index) = live.iter().position(|body| body.id == tool) else {
        warnings.push(format!(
            "Body intersect '{node_id}': tool body '{tool}' no longer exists."
        ));
        return;
    };
    let target_body = live[target_index].clone();
    let tool_body = live[tool_index].clone();
    if target_body.parts.is_empty() || tool_body.parts.is_empty() {
        warnings.push(format!(
            "Body intersect '{node_id}': both selected bodies need solid geometry."
        ));
        return;
    }

    let input_names = match (&target_body.pristine, &target_body.parts[..]) {
        (Some(mesh), [part]) => Some(crate::mock_kernel::input_shell_face_names(mesh, part)),
        _ => None,
    };
    let owner_classes = input_names
        .as_ref()
        .map(|names| crate::mock_kernel::owner_classes_from_names(names));
    let exact_history = target_body.parts.len() == 1 && tool_body.parts.len() == 1;
    let mut result_parts = Vec::new();
    let mut histories = Vec::new();
    for target_part in &target_body.parts {
        for tool_part in &tool_body.parts {
            if !aabbs_have_positive_overlap(target_part, tool_part) {
                continue;
            }
            match crate::mock_kernel::common_bodies_with_history(
                target_part,
                tool_part,
                exact_history.then_some(owner_classes.as_deref()).flatten(),
            ) {
                Ok(outcome) => {
                    result_parts.extend(outcome.bodies);
                    histories.extend(outcome.face_history);
                }
                Err(crate::mock_kernel::CommonBodiesError::Empty) => {}
                Err(crate::mock_kernel::CommonBodiesError::Failed(reason)) => {
                    warnings.push(format!(
                        "Body intersect '{node_id}': {reason}; both original bodies were left unchanged."
                    ));
                    return;
                }
            }
        }
    }
    if result_parts.is_empty() {
        warnings.push(format!(
            "Body intersect '{node_id}': the bodies have no positive-volume overlap; \
             both original bodies were left unchanged."
        ));
        return;
    }
    if !exact_history {
        result_parts.sort_by_key(crate::mock_kernel::part_key);
    }

    let pristine = match (
        exact_history,
        target_body.pristine.as_deref(),
        input_names.as_deref(),
    ) {
        (true, Some(input_mesh), Some(names)) => {
            crate::mock_kernel::propagate_face_names_via_body_histories(
                input_mesh,
                names,
                &result_parts,
                &histories,
                node_id,
                &format!("intersect:{node_id}"),
            )
            .map(std::sync::Arc::new)
        }
        _ => None,
    };

    let mut remove = vec![target_index];
    if !keep_tool {
        remove.push(tool_index);
    }
    remove.sort_unstable();
    remove.dedup();
    for index in remove.into_iter().rev() {
        live.remove(index);
    }
    apply_new(
        live,
        LiveBody {
            id: node_id.to_string(),
            parts: result_parts,
            pristine,
            sketch_source: None,
        },
    );
}

/// Transactional planar split. Parts already wholly on one side are retained;
/// crossing parts use the canonical two-sided kernel operation.
pub(crate) fn apply_body_split(
    node_id: &str,
    target: &str,
    plane: &PlaneBase,
    face: Option<&FaceRef>,
    datums: &HashMap<String, DatumValue>,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(target_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "Split body '{node_id}': target body '{target}' no longer exists."
        ));
        return;
    };
    let target_body = live[target_index].clone();
    if target_body.parts.is_empty() {
        warnings.push(format!(
            "Split body '{node_id}': target body '{target}' has no solid geometry."
        ));
        return;
    }
    let split_plane = match face {
        Some(reference) => match resolved_planar_face(live, reference) {
            Ok(plane) => plane,
            Err(reason) => {
                warnings.push(format!("Split body '{node_id}': {reason}."));
                return;
            }
        },
        None => {
            let Some(cs) = super::datum::resolve_plane_base_world(plane, datums) else {
                warnings.push(format!(
                    "Split body '{node_id}': its origin/datum plane could not be resolved."
                ));
                return;
            };
            plane_from_coordinate_system(cs)
        }
    };

    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let normal = openrcad::foundation::Vec::from_dir(split_plane.normal());
    let signed = |point: openrcad::foundation::Pnt| (point - split_plane.location()).dot(&normal);
    let tolerance = policy.classification * 8.0;
    let mut negative = Vec::new();
    let mut positive = Vec::new();
    let mut negative_histories = Vec::new();
    let mut positive_histories = Vec::new();
    let mut used_kernel_split = false;
    for part in &target_body.parts {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for vertex in part.vertices() {
            let distance = signed(vertex.point());
            min = min.min(distance);
            max = max.max(distance);
        }
        if max <= tolerance {
            negative.push(part.clone());
        } else if min >= -tolerance {
            positive.push(part.clone());
        } else {
            let outcome = match crate::mock_kernel::consume_plane_split_operation(
                "split body",
                openrcad::algo::split_solid_by_plane_operation_with_policy(
                    part,
                    &split_plane,
                    &policy,
                ),
            ) {
                Ok(outcome) => outcome,
                Err(reason) => {
                    warnings.push(format!(
                        "Split body '{node_id}': {reason}; the original body was left unchanged."
                    ));
                    return;
                }
            };
            used_kernel_split = true;
            negative.extend(outcome.negative);
            positive.extend(outcome.positive);
            negative_histories.extend(outcome.negative_face_history);
            positive_histories.extend(outcome.positive_face_history);
        }
    }
    if negative.is_empty() || positive.is_empty() {
        warnings.push(format!(
            "Split body '{node_id}': the plane does not divide the body into two positive volumes."
        ));
        return;
    }

    let exact_history = target_body.parts.len() == 1 && used_kernel_split;
    if !exact_history {
        negative.sort_by_key(crate::mock_kernel::part_key);
        positive.sort_by_key(crate::mock_kernel::part_key);
    }
    let input_names = match (&target_body.pristine, &target_body.parts[..]) {
        (Some(mesh), [part]) => Some(crate::mock_kernel::input_shell_face_names(mesh, part)),
        _ => None,
    };
    let named_side = |parts: &[KernelSolid],
                      histories: &[crate::mock_kernel::BooleanFaceHistory],
                      side: &str|
     -> Option<std::sync::Arc<MockMesh>> {
        if !exact_history {
            return None;
        }
        crate::mock_kernel::propagate_face_names_via_body_histories(
            target_body.pristine.as_deref()?,
            input_names.as_deref()?,
            parts,
            histories,
            // Positive-side metadata is restamped to its actual second output below.
            node_id,
            &format!("split:{node_id}:{side}"),
        )
        .map(std::sync::Arc::new)
    };
    let negative_mesh = named_side(&negative, &negative_histories, "negative");
    let mut positive_mesh = named_side(&positive, &positive_histories, "positive");
    let positive_id = body_output_id(node_id, 1);
    if let Some(mesh) = positive_mesh.as_mut() {
        restamp_mesh_body(std::sync::Arc::make_mut(mesh), &positive_id, node_id);
    }

    live.remove(target_index);
    apply_new(
        live,
        LiveBody {
            id: node_id.to_string(),
            parts: negative,
            pristine: negative_mesh,
            sketch_source: None,
        },
    );
    apply_new(
        live,
        LiveBody {
            id: positive_id,
            parts: positive,
            pristine: positive_mesh,
            sketch_source: None,
        },
    );
}

pub(crate) fn apply_body_scale(
    node_id: &str,
    source: &str,
    factor: f32,
    center: [f32; 3],
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if !factor.is_finite() || factor <= 1.0e-6 {
        warnings.push(format!(
            "Scale body '{node_id}': factor must be finite and greater than zero."
        ));
        return;
    }
    if center.iter().any(|value| !value.is_finite()) {
        warnings.push(format!(
            "Scale body '{node_id}': pivot coordinates must be finite."
        ));
        return;
    }
    let Some(source_index) = live.iter().position(|body| body.id == source) else {
        warnings.push(format!(
            "Scale body '{node_id}': source body '{source}' no longer exists."
        ));
        return;
    };
    let source_body = live[source_index].clone();
    if source_body.parts.is_empty() {
        warnings.push(format!(
            "Scale body '{node_id}': source body '{source}' has no solid geometry."
        ));
        return;
    }
    let pivot =
        openrcad::foundation::Pnt::new(center[0] as f64, center[1] as f64, center[2] as f64);
    let transform = openrcad::foundation::Trsf::scale(&pivot, factor as f64);
    let mut parts = Vec::with_capacity(source_body.parts.len());
    for part in &source_body.parts {
        match crate::mock_kernel::transformed_solid_diagnostic(part, &transform, false) {
            Ok(part) => parts.push(part),
            Err(reason) => {
                warnings.push(format!(
                    "Scale body '{node_id}': {reason}; the source body was left unchanged."
                ));
                return;
            }
        }
    }
    let pristine = source_body.pristine.as_deref().and_then(|input_mesh| {
        let mut mesh = MockMesh::empty();
        for part in &parts {
            mesh.append(crate::mock_kernel::propagate_face_names(
                input_mesh, part, node_id,
            ));
        }
        (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh))
    });
    live.remove(source_index);
    apply_new(
        live,
        LiveBody {
            id: node_id.to_string(),
            parts,
            pristine,
            sketch_source: None,
        },
    );
}

fn aabbs_have_positive_overlap(a: &KernelSolid, b: &KernelSolid) -> bool {
    let (Some((alo, ahi)), Some((blo, bhi))) = (
        crate::mock_kernel::solid_aabb(a),
        crate::mock_kernel::solid_aabb(b),
    ) else {
        return false;
    };
    (0..3).all(|axis| ahi[axis].min(bhi[axis]) - alo[axis].max(blo[axis]) > 1.0e-5)
}

fn plane_from_coordinate_system(cs: CoordinateSystem) -> openrcad::geom::Plane {
    use openrcad::foundation::{Ax3, Dir, Pnt};
    openrcad::geom::Plane::new(Ax3::new_axes(
        Pnt::new(cs.origin.x as f64, cs.origin.y as f64, cs.origin.z as f64),
        Dir::new(cs.n.x as f64, cs.n.y as f64, cs.n.z as f64),
        Dir::new(cs.u.x as f64, cs.u.y as f64, cs.u.z as f64),
    ))
}

fn resolved_planar_face(
    live: &[LiveBody],
    reference: &FaceRef,
) -> Result<openrcad::geom::Plane, String> {
    let body = if let Some(body_id) = reference
        .topology
        .as_ref()
        .and_then(|topology| topology.body_id.as_deref())
    {
        live.iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("the selected face body '{body_id}' no longer exists"))?
    } else {
        live.iter()
            .find(|body| resolve_face_on_body(body, reference).is_some())
            .ok_or_else(|| "the selected face could not be resolved on a live body".to_string())?
    };
    let resolved = resolve_face_on_body(body, reference)
        .ok_or_else(|| "the selected face could not be resolved".to_string())?;
    let part = body
        .parts
        .get(resolved.component_index)
        .ok_or_else(|| "the selected face's solid component no longer exists".to_string())?;
    let point = openrcad::foundation::Pnt::new(
        resolved.face.centroid[0] as f64,
        resolved.face.centroid[1] as f64,
        resolved.face.centroid[2] as f64,
    );
    let captured_normal = Vec3::new(
        resolved.face.normal[0],
        resolved.face.normal[1],
        resolved.face.normal[2],
    )
    .normalize();
    part.faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(openrcad::geom::GeomSurface::Plane(plane)) => {
                let normal = Vec3::new(
                    plane.normal().x() as f32,
                    plane.normal().y() as f32,
                    plane.normal().z() as f32,
                );
                if normal.dot(captured_normal).abs() < 0.7 {
                    return None;
                }
                let distance = (point - plane.location())
                    .dot(&openrcad::foundation::Vec::from_dir(plane.normal()))
                    .abs();
                Some((distance, *plane))
            }
            _ => None,
        })
        .min_by(|(left, _), (right, _)| left.total_cmp(right))
        .filter(|(distance, _)| *distance <= 1.0e-3)
        .map(|(_, plane)| plane)
        .ok_or_else(|| "the selected face is not planar".to_string())
}

fn restamp_mesh_body(mesh: &mut MockMesh, body_id: &str, producer: &str) {
    for face in &mut mesh.face_refs {
        if let Some(topology) = face.topology.as_mut() {
            topology.body_id = Some(body_id.to_string());
            topology.producer_feature_id = Some(producer.to_string());
        }
    }
    for edge in &mut mesh.edge_refs {
        if let Some(topology) = edge.topology.as_mut() {
            topology.body_id = Some(body_id.to_string());
            topology.producer_feature_id = Some(producer.to_string());
        }
    }
}

/// Current world-space bounding-box center, used only to initialize the Scale
/// dialog. The stored feature pivot remains explicit and deterministic.
pub fn body_bounds_center(mesh: &MockMesh) -> Option<[f32; 3]> {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    min[0]
        .is_finite()
        .then(|| std::array::from_fn(|axis| 0.5 * (min[axis] + max[axis])))
}
