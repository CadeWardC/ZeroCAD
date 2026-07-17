//! Transactional direct edits for imported and history-free B-Rep bodies.
//!
//! Phase 5 deliberately supports a small exact set: normal translation and
//! offset of planar faces, planar or cylindrical thickening, analytic
//! cylindrical offsets, and deletion/healing of an internal cylindrical wall.
//! Every edit builds canonical pcurve-complete tools, passes through the
//! structured boolean boundary when material is changed, and commits only after
//! the complete replacement body is valid.

use super::*;

const DIRECT_EDIT_OVERSHOOT_RELATIVE: f64 = 0.05;
const DIRECT_EDIT_TOLERANCE_MULTIPLIER: f64 = 8.0;

/// Extend boolean tools beyond the selected face by a model-sized margin.
/// The policy term keeps very small models numerically separated while the
/// relative term prevents large models from depending on a millimetre literal.
fn direct_edit_overshoot(
    solid: &KernelSolid,
    policy: &openrcad::foundation::TolerancePolicy,
) -> f32 {
    let model_scale = crate::mock_kernel::solid_aabb(solid)
        .map(|(min, max)| {
            (0..3)
                .map(|axis| f64::from(max[axis] - min[axis]).abs())
                .fold(0.0_f64, f64::max)
        })
        .unwrap_or(0.0);
    let policy_margin = policy
        .classification
        .max(policy.intersection)
        .max(policy.sewing)
        * DIRECT_EDIT_TOLERANCE_MULTIPLIER;
    (model_scale * DIRECT_EDIT_OVERSHOOT_RELATIVE + policy_margin) as f32
}

pub(crate) fn apply_face_offset(
    node_id: &str,
    target: &str,
    reference: &FaceRef,
    distance: f32,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if !distance.is_finite() || distance.abs() <= 1.0e-5 {
        warnings.push(format!(
            "Press/Pull '{node_id}': distance must be finite and non-zero."
        ));
        return;
    }
    if let Some((body_index, component_index, cylinder)) =
        selected_cylindrical_face(target, reference, live)
    {
        apply_cylindrical_face_offset(
            node_id,
            body_index,
            component_index,
            cylinder,
            distance,
            live,
            warnings,
        );
        return;
    }
    let Some((body_index, component_index, face, normal)) =
        resolve_planar_edit_face(target, reference, live, node_id, "Press/Pull", warnings)
    else {
        return;
    };
    let source = live[body_index].clone();
    let component = &source.parts[component_index];
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let overshoot = direct_edit_overshoot(component, &policy);
    let (start_offset, sweep) = if distance > 0.0 {
        (-overshoot, distance + overshoot)
    } else {
        (overshoot, distance - 2.0 * overshoot)
    };
    let start = face.transformed(&openrcad::foundation::Trsf::translation(
        openrcad::foundation::Vec::new(
            normal[0] as f64 * start_offset as f64,
            normal[1] as f64 * start_offset as f64,
            normal[2] as f64 * start_offset as f64,
        ),
    ));
    let vector = openrcad::foundation::Vec::new(
        normal[0] as f64 * sweep as f64,
        normal[1] as f64 * sweep as f64,
        normal[2] as f64 * sweep as f64,
    );
    let tool = match crate::mock_kernel::consume_operation(
        "direct-edit face prism",
        openrcad::algo::prism_operation_with_policy(&start, vector, &policy),
    ) {
        Ok(outcome) => outcome.solid,
        Err(reason) => {
            warnings.push(format!(
                "Press/Pull '{node_id}': could not construct a valid face-local tool ({reason})."
            ));
            return;
        }
    };

    let (edited, histories) = if distance > 0.0 {
        match crate::mock_kernel::union_with_history_diagnostic(component, &tool, None) {
            Ok((solid, history)) => (vec![solid], vec![history]),
            Err(reason) => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': outward face offset failed ({reason}); the source was left unchanged."
                ));
                return;
            }
        }
    } else {
        match crate::mock_kernel::difference_bodies_with_history(component, &tool, None) {
            Some(outcome) if !outcome.bodies.is_empty() => (outcome.bodies, outcome.face_history),
            _ => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': inward face offset removed or invalidated the body; the source was left unchanged."
                ));
                return;
            }
        }
    };
    commit_component_replacement(
        node_id,
        body_index,
        component_index,
        source,
        edited,
        Some(histories),
        live,
        warnings,
    );
}

pub(crate) fn apply_face_move(
    node_id: &str,
    target: &str,
    reference: &FaceRef,
    translation: [f32; 3],
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if translation.iter().any(|value| !value.is_finite()) {
        warnings.push(format!(
            "Move face '{node_id}': translation coordinates must be finite."
        ));
        return;
    }
    let normal = Vec3::new(
        reference.normal[0],
        reference.normal[1],
        reference.normal[2],
    )
    .normalize();
    if normal == Vec3::ZERO {
        warnings.push(format!(
            "Move face '{node_id}': the selected face has no usable normal."
        ));
        return;
    }
    let translation = Vec3::new(translation[0], translation[1], translation[2]);
    let distance = translation.dot(normal);
    let tangential = translation.sub(normal.mul(distance));
    if tangential.length() > 1.0e-4 {
        warnings.push(format!(
            "Move face '{node_id}': Phase 5 supports planar translation along the face normal; tangential moves are not approximated."
        ));
        return;
    }
    apply_face_offset(node_id, target, reference, distance, live, warnings);
}

pub(crate) fn apply_face_thicken(
    node_id: &str,
    target: &str,
    reference: &FaceRef,
    thickness: f32,
    reverse: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if !thickness.is_finite() || thickness <= 1.0e-5 {
        warnings.push(format!(
            "Thicken face '{node_id}': thickness must be finite and positive."
        ));
        return;
    }
    if let Some((_, _, cylinder)) = selected_cylindrical_face(target, reference, live) {
        apply_cylindrical_face_thicken(node_id, cylinder, thickness, reverse, live, warnings);
        return;
    }
    let Some((_, _, face, mut normal)) =
        resolve_planar_edit_face(target, reference, live, node_id, "Thicken face", warnings)
    else {
        return;
    };
    if reverse {
        normal = normal.map(|component| -component);
    }
    let vector = openrcad::foundation::Vec::new(
        normal[0] as f64 * thickness as f64,
        normal[1] as f64 * thickness as f64,
        normal[2] as f64 * thickness as f64,
    );
    let solid = match crate::mock_kernel::consume_operation(
        "direct-edit thicken",
        openrcad::algo::prism_operation_with_policy(
            &face,
            vector,
            &openrcad::foundation::TolerancePolicy::STANDARD,
        ),
    ) {
        Ok(outcome) => outcome.solid,
        Err(reason) => {
            warnings.push(format!(
                "Thicken face '{node_id}': selected surface is not supported ({reason})."
            ));
            return;
        }
    };
    let mut mesh = match MockMesh::try_from_solid(&solid) {
        Ok(mesh) => mesh,
        Err(reason) => {
            warnings.push(format!(
                "Thicken face '{node_id}': valid result could not be displayed ({reason})."
            ));
            return;
        }
    };
    stamp_generated_face_refs(&mut mesh, node_id, "thicken");
    crate::mock_kernel::populate_edge_adjacent_face_names(&mut mesh);
    apply_new(
        live,
        LiveBody {
            id: node_id.to_string(),
            parts: vec![solid],
            pristine: Some(std::sync::Arc::new(mesh)),
            sketch_source: None,
        },
    );
}

fn selected_cylindrical_face(
    target: &str,
    reference: &FaceRef,
    live: &[LiveBody],
) -> Option<(usize, usize, crate::mock_kernel::CylinderFaceInfo)> {
    let body_index = live.iter().position(|body| body.id == target)?;
    let body = &live[body_index];
    let resolved = resolve_face_on_body(body, reference)?;
    let part = body.parts.get(resolved.component_index)?;
    if !crate::mock_kernel::kernel_faces_matching(
        part,
        resolved.face.centroid,
        resolved.face.normal,
    )
    .is_empty()
    {
        return None;
    }
    let cylinder = crate::mock_kernel::cylinder_face_near(part, resolved.face.centroid)?;
    Some((body_index, resolved.component_index, cylinder))
}

#[allow(clippy::too_many_arguments)]
fn apply_cylindrical_face_offset(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    cylinder: crate::mock_kernel::CylinderFaceInfo,
    distance: f32,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let source = live[body_index].clone();
    let component = &source.parts[component_index];
    let new_radius = if cylinder.internal {
        cylinder.radius - distance
    } else {
        cylinder.radius + distance
    };
    if !new_radius.is_finite() || new_radius <= 1.0e-5 {
        warnings.push(format!(
            "Press/Pull '{node_id}': cylindrical offset collapses or reverses the radius."
        ));
        return;
    }
    // A standalone analytic cylinder can be rebuilt exactly without creating
    // a coincident annular boolean seam. This is both more precise and more
    // stable than fusing a ring onto the existing wall.
    let surfaces = component.shell().faces();
    let plane_count = surfaces
        .iter()
        .filter(|face| matches!(face.surface(), Some(openrcad::geom::GeomSurface::Plane(_))))
        .count();
    let cylinder_count = surfaces
        .iter()
        .filter(|face| {
            matches!(
                face.surface(),
                Some(openrcad::geom::GeomSurface::Cylinder(_))
            )
        })
        .count();
    let plain_cylinder =
        plane_count == 2 && cylinder_count > 0 && plane_count + cylinder_count == surfaces.len();
    if !cylinder.internal && plain_cylinder {
        let axis = Vec3::new(cylinder.dir[0], cylinder.dir[1], cylinder.dir[2]).normalize();
        let origin = Vec3::new(cylinder.origin[0], cylinder.origin[1], cylinder.origin[2])
            .add(axis.mul(cylinder.axial_min));
        let length = cylinder.axial_max - cylinder.axial_min;
        let Some(solid) =
            crate::mock_kernel::cylinder_tool_at(origin, axis, new_radius as f64, length as f64)
        else {
            warnings.push(format!(
                "Press/Pull '{node_id}': could not rebuild the analytic cylinder."
            ));
            return;
        };
        commit_component_replacement(
            node_id,
            body_index,
            component_index,
            source,
            vec![solid],
            None,
            live,
            warnings,
        );
        return;
    }
    let adds_material = if cylinder.internal {
        new_radius < cylinder.radius
    } else {
        new_radius > cylinder.radius
    };
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let overshoot = if adds_material {
        0.0
    } else {
        direct_edit_overshoot(component, &policy)
    };
    let inner = new_radius.min(cylinder.radius);
    let outer = new_radius.max(cylinder.radius);
    let Some(tool) = cylindrical_annulus(&cylinder, inner, outer, overshoot) else {
        warnings.push(format!(
            "Press/Pull '{node_id}': could not construct a strict analytic cylindrical offset tool."
        ));
        return;
    };
    let (edited, histories) = if adds_material {
        match crate::mock_kernel::union_with_history_diagnostic(component, &tool, None) {
            Ok((solid, history)) => (vec![solid], vec![history]),
            Err(reason) => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': analytic cylindrical addition failed ({reason}); the source was left unchanged."
                ));
                return;
            }
        }
    } else {
        match crate::mock_kernel::difference_bodies_with_history(component, &tool, None) {
            Some(outcome) if !outcome.bodies.is_empty() => (outcome.bodies, outcome.face_history),
            _ => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': analytic cylindrical removal failed; the source was left unchanged."
                ));
                return;
            }
        }
    };
    commit_component_replacement(
        node_id,
        body_index,
        component_index,
        source,
        edited,
        Some(histories),
        live,
        warnings,
    );
}

fn apply_cylindrical_face_thicken(
    node_id: &str,
    cylinder: crate::mock_kernel::CylinderFaceInfo,
    thickness: f32,
    reverse: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let outward_increases_radius = !cylinder.internal;
    let increase = outward_increases_radius ^ reverse;
    let (inner, outer) = if increase {
        (cylinder.radius, cylinder.radius + thickness)
    } else {
        (cylinder.radius - thickness, cylinder.radius)
    };
    if inner <= 1.0e-5 {
        warnings.push(format!(
            "Thicken face '{node_id}': thickness collapses the cylindrical inner radius."
        ));
        return;
    }
    let Some(solid) = cylindrical_annulus(&cylinder, inner, outer, 0.0) else {
        warnings.push(format!(
            "Thicken face '{node_id}': could not build a strict analytic cylindrical shell."
        ));
        return;
    };
    let mut mesh = match MockMesh::try_from_solid(&solid) {
        Ok(mesh) => mesh,
        Err(reason) => {
            warnings.push(format!(
                "Thicken face '{node_id}': analytic result could not be displayed ({reason})."
            ));
            return;
        }
    };
    stamp_generated_face_refs(&mut mesh, node_id, "thicken");
    crate::mock_kernel::populate_edge_adjacent_face_names(&mut mesh);
    apply_new(
        live,
        LiveBody {
            id: node_id.to_string(),
            parts: vec![solid],
            pristine: Some(std::sync::Arc::new(mesh)),
            sketch_source: None,
        },
    );
}

fn cylindrical_annulus(
    cylinder: &crate::mock_kernel::CylinderFaceInfo,
    inner_radius: f32,
    outer_radius: f32,
    axial_overshoot: f32,
) -> Option<KernelSolid> {
    let axis = Vec3::new(cylinder.dir[0], cylinder.dir[1], cylinder.dir[2]).normalize();
    let origin = Vec3::new(cylinder.origin[0], cylinder.origin[1], cylinder.origin[2])
        .add(axis.mul(cylinder.axial_min - axial_overshoot));
    let length = cylinder.axial_max - cylinder.axial_min + 2.0 * axial_overshoot;
    let seed = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let radial = axis.cross(seed).normalize();
    let frame = CoordinateSystem::new(origin, radial, axis);
    let profile = [
        (inner_radius, 0.0),
        (outer_radius, 0.0),
        (outer_radius, length),
        (inner_radius, length),
    ];
    crate::mock_kernel::revolved_region_solid(
        &profile,
        &[],
        &frame,
        origin,
        axis,
        std::f64::consts::TAU,
        &[],
    )
}

pub(crate) fn apply_face_delete(
    node_id: &str,
    target: &str,
    reference: &FaceRef,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(body_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "Delete face '{node_id}': target body '{target}' no longer exists."
        ));
        return;
    };
    let source = live[body_index].clone();
    if source.parts.is_empty() {
        warnings.push(format!(
            "Delete face '{node_id}': mesh bodies do not support B-Rep face deletion."
        ));
        return;
    }
    let Some(resolved) = resolve_face_on_body(&source, reference) else {
        warnings.push(format!(
            "Delete face '{node_id}': the selected face could not be reattached."
        ));
        return;
    };
    let Some(component) = source.parts.get(resolved.component_index) else {
        warnings.push(format!(
            "Delete face '{node_id}': the selected component no longer exists."
        ));
        return;
    };
    let Some(cylinder) = crate::mock_kernel::cylinder_face_near(component, resolved.face.centroid)
    else {
        warnings.push(format!(
            "Delete face '{node_id}': Phase 5 supports deletion/healing of internal cylindrical faces; this surface is unsupported."
        ));
        return;
    };
    let axis = Vec3::new(cylinder.dir[0], cylinder.dir[1], cylinder.dir[2]).normalize();
    let axis_origin = Vec3::new(cylinder.origin[0], cylinder.origin[1], cylinder.origin[2]);
    if !cylinder.internal {
        warnings.push(format!(
            "Delete face '{node_id}': deleting an external cylindrical wall would require general surface healing and is not supported."
        ));
        return;
    }
    // A delete-face heal fills exactly the trimmed wall span. Overshooting here
    // would create small bosses beyond the adjacent cap faces rather than
    // merely replacing the removed volume.
    let start = axis_origin.add(axis.mul(cylinder.axial_min));
    let length = cylinder.axial_max - cylinder.axial_min;
    let Some(tool) =
        crate::mock_kernel::cylinder_tool_at(start, axis, cylinder.radius as f64, length as f64)
    else {
        warnings.push(format!(
            "Delete face '{node_id}': could not build the analytic healing patch."
        ));
        return;
    };
    let (edited, histories) = match crate::mock_kernel::union_with_history_diagnostic(
        component, &tool, None,
    ) {
        Ok((solid, history)) => (vec![solid], vec![history]),
        Err(reason) => {
            warnings.push(format!(
                "Delete face '{node_id}': cylindrical hole healing failed ({reason}); the source was left unchanged."
            ));
            return;
        }
    };
    let component_index = resolved.component_index;
    commit_component_replacement(
        node_id,
        body_index,
        component_index,
        source,
        edited,
        Some(histories),
        live,
        warnings,
    );
}

fn resolve_planar_edit_face(
    target: &str,
    reference: &FaceRef,
    live: &[LiveBody],
    node_id: &str,
    operation: &str,
    warnings: &mut Vec<String>,
) -> Option<(usize, usize, openrcad::topo::Face, [f32; 3])> {
    let Some(body_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "{operation} '{node_id}': target body '{target}' no longer exists."
        ));
        return None;
    };
    let body = &live[body_index];
    if body.parts.is_empty() {
        warnings.push(format!(
            "{operation} '{node_id}': mesh bodies cannot enter B-Rep direct edits."
        ));
        return None;
    }
    let Some(resolved) = resolve_face_on_body(body, reference) else {
        warnings.push(format!(
            "{operation} '{node_id}': the selected face could not be reattached."
        ));
        return None;
    };
    let component_index = resolved.component_index;
    let component = &body.parts[component_index];
    let Some(face) = crate::mock_kernel::kernel_faces_matching(
        component,
        resolved.face.centroid,
        resolved.face.normal,
    )
    .into_iter()
    .next() else {
        warnings.push(format!(
            "{operation} '{node_id}': Phase 5 supports planar faces; this selected surface is unsupported."
        ));
        return None;
    };
    let normal = Vec3::new(
        resolved.face.normal[0],
        resolved.face.normal[1],
        resolved.face.normal[2],
    )
    .normalize();
    Some((
        body_index,
        component_index,
        face,
        [normal.x, normal.y, normal.z],
    ))
}

#[allow(clippy::too_many_arguments)]
fn commit_component_replacement(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    source: LiveBody,
    edited: Vec<KernelSolid>,
    histories: Option<Vec<crate::mock_kernel::BooleanFaceHistory>>,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if edited.iter().any(|solid| {
        !solid.is_watertight()
            || !solid.health_report().is_healthy()
            || solid
                .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
                .is_err()
    }) {
        warnings.push(format!(
            "Direct edit '{node_id}': strict topology validation rejected the replacement; the source was left unchanged."
        ));
        return;
    }
    let exact_history = source.parts.len() == 1 && component_index == 0;
    let mut parts = source.parts.clone();
    parts.remove(component_index);
    parts.extend(edited);
    parts.sort_by_key(crate::mock_kernel::part_key);
    let pristine = source.pristine.as_deref().and_then(|input_mesh| {
        if exact_history {
            if let Some(histories) = histories.as_deref() {
                let names =
                    crate::mock_kernel::input_shell_face_names(input_mesh, &source.parts[0]);
                if let Some(mesh) = crate::mock_kernel::propagate_face_names_via_body_histories(
                    input_mesh,
                    &names,
                    &parts,
                    histories,
                    node_id,
                    &format!("direct:{node_id}"),
                ) {
                    return Some(std::sync::Arc::new(mesh));
                }
            }
        }
        let mut mesh = MockMesh::empty();
        for part in &parts {
            mesh.append(crate::mock_kernel::propagate_face_names(
                input_mesh, part, node_id,
            ));
        }
        (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh))
    });
    live.remove(body_index);
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

#[cfg(test)]
mod overshoot_tests {
    use super::*;

    #[test]
    fn boolean_tool_overshoot_tracks_model_scale_and_policy() {
        let small = openrcad::primitives::make_box_operation(
            &openrcad::foundation::Pnt::origin(),
            0.01,
            0.01,
            0.01,
        )
        .expect("small box")
        .value;
        let large = openrcad::primitives::make_box_operation(
            &openrcad::foundation::Pnt::origin(),
            1_000.0,
            1_000.0,
            1_000.0,
        )
        .expect("large box")
        .value;
        let policy = openrcad::foundation::TolerancePolicy::STANDARD;
        let small_margin = direct_edit_overshoot(&small, &policy);
        let large_margin = direct_edit_overshoot(&large, &policy);

        assert!(small_margin > policy.classification as f32);
        assert!(large_margin > small_margin * 1_000.0);
        assert!((large_margin - 50.00008).abs() < 1.0e-3);
    }
}
