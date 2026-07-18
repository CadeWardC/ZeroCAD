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
    if let Some((body_index, component_index, cone)) =
        selected_conical_face(target, reference, live)
    {
        apply_conical_face_offset(
            node_id,
            body_index,
            component_index,
            cone,
            distance,
            live,
            warnings,
        );
        return;
    }
    if let Some((body_index, component_index, sphere)) =
        selected_spherical_face(target, reference, live)
    {
        apply_spherical_face_offset(
            node_id,
            body_index,
            component_index,
            sphere,
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
        if try_move_parallelepiped_face(node_id, target, reference, translation, live, warnings) {
            return;
        }
        warnings.push(format!(
            "Move face '{node_id}': tangential neighborhood surgery is supported for planar six-face blocks; this face is not in that exact set."
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
    if let Some((body_index, component_index, cylinder)) =
        selected_cylindrical_face(target, reference, live)
    {
        let source = live[body_index].clone();
        apply_cylindrical_face_thicken(
            node_id,
            body_index,
            component_index,
            source,
            cylinder,
            thickness,
            reverse,
            live,
            warnings,
        );
        return;
    }
    if let Some((body_index, component_index, cone)) =
        selected_conical_face(target, reference, live)
    {
        let source = live[body_index].clone();
        apply_conical_face_thicken(
            node_id,
            body_index,
            component_index,
            source,
            cone,
            thickness,
            reverse,
            live,
            warnings,
        );
        return;
    }
    if let Some((body_index, component_index, sphere)) =
        selected_spherical_face(target, reference, live)
    {
        let source = live[body_index].clone();
        apply_spherical_face_thicken(
            node_id,
            body_index,
            component_index,
            source,
            sphere,
            thickness,
            reverse,
            live,
            warnings,
        );
        return;
    }
    let Some((body_index, component_index, face, mut normal)) =
        resolve_planar_edit_face(target, reference, live, node_id, "Thicken face", warnings)
    else {
        return;
    };
    let source = live[body_index].clone();
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

fn selected_conical_face(
    target: &str,
    reference: &FaceRef,
    live: &[LiveBody],
) -> Option<(usize, usize, crate::mock_kernel::ConeFaceInfo)> {
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
    crate::mock_kernel::cone_face_near(part, resolved.face.centroid)
        .map(|info| (body_index, resolved.component_index, info))
}

fn selected_spherical_face(
    target: &str,
    reference: &FaceRef,
    live: &[LiveBody],
) -> Option<(usize, usize, crate::mock_kernel::SphereFaceInfo)> {
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
    crate::mock_kernel::sphere_face_near(part, resolved.face.centroid)
        .map(|info| (body_index, resolved.component_index, info))
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

#[allow(clippy::too_many_arguments)]
fn apply_cylindrical_face_thicken(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    source: LiveBody,
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

#[allow(clippy::too_many_arguments)]
fn apply_conical_face_offset(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    cone: crate::mock_kernel::ConeFaceInfo,
    distance: f32,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let source = live[body_index].clone();
    let component = &source.parts[component_index];
    let normal_radius_delta = distance / cone.semi_angle.cos().abs().max(1.0e-4);
    let signed_delta = if cone.internal {
        -normal_radius_delta
    } else {
        normal_radius_delta
    };
    let first = cone.radius_at(cone.axial_min);
    let last = cone.radius_at(cone.axial_max);
    let new_first = first + signed_delta;
    let new_last = last + signed_delta;
    if new_first <= 1.0e-5 || new_last < -1.0e-5 {
        warnings.push(format!(
            "Press/Pull '{node_id}': conical offset collapses or crosses the apex."
        ));
        return;
    }
    let axis = Vec3::new(cone.dir[0], cone.dir[1], cone.dir[2]).normalize();
    let origin =
        Vec3::new(cone.origin[0], cone.origin[1], cone.origin[2]).add(axis.mul(cone.axial_min));
    let length = cone.axial_max - cone.axial_min;
    let plane_count = component
        .faces()
        .iter()
        .filter(|face| matches!(face.surface(), Some(openrcad::geom::GeomSurface::Plane(_))))
        .count();
    let cone_count = component
        .faces()
        .iter()
        .filter(|face| matches!(face.surface(), Some(openrcad::geom::GeomSurface::Cone(_))))
        .count();
    let plain_cone = plane_count >= 1 && plane_count + cone_count == component.face_count();
    if !cone.internal && plain_cone {
        let Some(rebuilt) = crate::mock_kernel::cone_tool_at(
            origin,
            axis,
            f64::from(new_first),
            f64::from(new_last.max(0.0)),
            f64::from(length),
        ) else {
            warnings.push(format!(
                "Press/Pull '{node_id}': could not rebuild the analytic cone."
            ));
            return;
        };
        commit_component_replacement(
            node_id,
            body_index,
            component_index,
            source,
            vec![rebuilt],
            None,
            live,
            warnings,
        );
        return;
    }
    let (inner_first, outer_first) = if signed_delta > 0.0 {
        (first, new_first)
    } else {
        (new_first, first)
    };
    let (inner_last, outer_last) = if signed_delta > 0.0 {
        (last, new_last)
    } else {
        (new_last, last)
    };
    let Some(tool) = conical_annulus(
        origin,
        axis,
        inner_first,
        inner_last.max(0.0),
        outer_first,
        outer_last.max(0.0),
        length,
    ) else {
        warnings.push(format!(
            "Press/Pull '{node_id}': could not build the analytic conical offset tool."
        ));
        return;
    };
    let adds_material = if cone.internal {
        signed_delta < 0.0
    } else {
        signed_delta > 0.0
    };
    let (edited, histories) = if adds_material {
        match crate::mock_kernel::union_with_history_diagnostic(component, &tool, None) {
            Ok((solid, history)) => (vec![solid], vec![history]),
            Err(reason) => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': conical addition failed ({reason})."
                ));
                return;
            }
        }
    } else {
        match crate::mock_kernel::difference_bodies_with_history(component, &tool, None) {
            Some(outcome) if !outcome.bodies.is_empty() => (outcome.bodies, outcome.face_history),
            _ => {
                warnings.push(format!("Press/Pull '{node_id}': conical removal failed."));
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

fn apply_spherical_face_offset(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    sphere: crate::mock_kernel::SphereFaceInfo,
    distance: f32,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let source = live[body_index].clone();
    let component = &source.parts[component_index];
    let delta = if sphere.internal { -distance } else { distance };
    let new_radius = sphere.radius + delta;
    if new_radius <= 1.0e-5 {
        warnings.push(format!(
            "Press/Pull '{node_id}': spherical offset collapses the radius."
        ));
        return;
    }
    let plain_sphere = component
        .faces()
        .iter()
        .all(|face| matches!(face.surface(), Some(openrcad::geom::GeomSurface::Sphere(_))));
    if !sphere.internal && plain_sphere {
        let Some(rebuilt) = sphere_solid(sphere.center, new_radius) else {
            warnings.push(format!(
                "Press/Pull '{node_id}': could not rebuild the analytic sphere."
            ));
            return;
        };
        commit_component_replacement(
            node_id,
            body_index,
            component_index,
            source,
            vec![rebuilt],
            None,
            live,
            warnings,
        );
        return;
    }
    let (inner, outer) = if delta > 0.0 {
        (sphere.radius, new_radius)
    } else {
        (new_radius, sphere.radius)
    };
    let Some(tool) = spherical_annulus(sphere.center, inner, outer) else {
        warnings.push(format!(
            "Press/Pull '{node_id}': could not build the analytic spherical offset tool."
        ));
        return;
    };
    let adds_material = if sphere.internal {
        delta < 0.0
    } else {
        delta > 0.0
    };
    let (edited, histories) = if adds_material {
        match crate::mock_kernel::union_with_history_diagnostic(component, &tool, None) {
            Ok((solid, history)) => (vec![solid], vec![history]),
            Err(reason) => {
                warnings.push(format!(
                    "Press/Pull '{node_id}': spherical addition failed ({reason})."
                ));
                return;
            }
        }
    } else {
        match crate::mock_kernel::difference_bodies_with_history(component, &tool, None) {
            Some(outcome) if !outcome.bodies.is_empty() => (outcome.bodies, outcome.face_history),
            _ => {
                warnings.push(format!("Press/Pull '{node_id}': spherical removal failed."));
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

#[allow(clippy::too_many_arguments)]
fn apply_conical_face_thicken(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    source: LiveBody,
    cone: crate::mock_kernel::ConeFaceInfo,
    thickness: f32,
    reverse: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let delta = thickness / cone.semi_angle.cos().abs().max(1.0e-4);
    let increase = (!cone.internal) ^ reverse;
    let signed = if increase { delta } else { -delta };
    let first = cone.radius_at(cone.axial_min);
    let last = cone.radius_at(cone.axial_max);
    let shifted_first = first + signed;
    let shifted_last = last + signed;
    if shifted_first <= 1.0e-5 || shifted_last < -1.0e-5 {
        warnings.push(format!(
            "Thicken face '{node_id}': thickness crosses the conical apex."
        ));
        return;
    }
    let axis = Vec3::new(cone.dir[0], cone.dir[1], cone.dir[2]).normalize();
    let origin =
        Vec3::new(cone.origin[0], cone.origin[1], cone.origin[2]).add(axis.mul(cone.axial_min));
    let Some(solid) = conical_annulus(
        origin,
        axis,
        first.min(shifted_first),
        last.min(shifted_last).max(0.0),
        first.max(shifted_first),
        last.max(shifted_last).max(0.0),
        cone.axial_max - cone.axial_min,
    ) else {
        warnings.push(format!(
            "Thicken face '{node_id}': conical shell construction failed."
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
}

#[allow(clippy::too_many_arguments)]
fn apply_spherical_face_thicken(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    source: LiveBody,
    sphere: crate::mock_kernel::SphereFaceInfo,
    thickness: f32,
    reverse: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let increase = (!sphere.internal) ^ reverse;
    let shifted = if increase {
        sphere.radius + thickness
    } else {
        sphere.radius - thickness
    };
    if shifted <= 1.0e-5 {
        warnings.push(format!(
            "Thicken face '{node_id}': thickness collapses the spherical inner radius."
        ));
        return;
    }
    let Some(solid) = spherical_annulus(
        sphere.center,
        sphere.radius.min(shifted),
        sphere.radius.max(shifted),
    ) else {
        warnings.push(format!(
            "Thicken face '{node_id}': spherical shell construction failed."
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
}

#[allow(clippy::too_many_arguments)]
fn conical_annulus(
    origin: Vec3,
    axis: Vec3,
    inner_first: f32,
    inner_last: f32,
    outer_first: f32,
    outer_last: f32,
    length: f32,
) -> Option<KernelSolid> {
    let outer = crate::mock_kernel::cone_tool_at(
        origin,
        axis,
        f64::from(outer_first),
        f64::from(outer_last),
        f64::from(length),
    )?;
    let inner = crate::mock_kernel::cone_tool_at(
        origin,
        axis,
        f64::from(inner_first),
        f64::from(inner_last),
        f64::from(length),
    )?;
    nested_void_solid(outer, inner)
}

fn sphere_solid(center: [f32; 3], radius: f32) -> Option<KernelSolid> {
    crate::mock_kernel::consume_operation(
        "sphere primitive",
        openrcad::primitives::make_sphere_operation(
            &openrcad::foundation::Pnt::new(
                f64::from(center[0]),
                f64::from(center[1]),
                f64::from(center[2]),
            ),
            f64::from(radius),
        ),
    )
    .ok()
    .map(|outcome| outcome.solid)
}

fn spherical_annulus(center: [f32; 3], inner: f32, outer: f32) -> Option<KernelSolid> {
    let outer = sphere_solid(center, outer)?;
    let inner = sphere_solid(center, inner)?;
    nested_void_solid(outer, inner)
}

/// Construct a material shell from two exact, nested analytic boundaries.
/// Boolean subtraction has no intersection curve when the bodies are strictly
/// nested, so representing the reversed inner boundary directly is both more
/// exact and more deterministic.
fn nested_void_solid(outer: KernelSolid, inner: KernelSolid) -> Option<KernelSolid> {
    let inner_shell =
        openrcad::topo::Shell::from_faces(inner.faces().into_iter().map(|face| face.reversed()));
    KernelSolid::from_shells([outer.shell(), inner_shell])
}

fn try_move_parallelepiped_face(
    node_id: &str,
    target: &str,
    reference: &FaceRef,
    translation: Vec3,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) -> bool {
    let Some((body_index, component_index, selected, _)) =
        resolve_planar_edit_face(target, reference, live, node_id, "Move face", warnings)
    else {
        return false;
    };
    let source = live[body_index].clone();
    let component = &source.parts[component_index];
    if component.face_count() != 6
        || component.vertex_count() != 8
        || component
            .faces()
            .iter()
            .any(|face| !matches!(face.surface(), Some(openrcad::geom::GeomSurface::Plane(_))))
    {
        return false;
    }
    let selected_plane = match selected.surface() {
        Some(openrcad::geom::GeomSurface::Plane(plane)) => *plane,
        _ => return false,
    };
    let Some(selected_anchor) = selected
        .outer_wire()
        .and_then(|wire| wire.edges().first().map(|edge| edge.source().point()))
    else {
        return false;
    };
    let selected_normal = openrcad::foundation::Vec::from_dir(selected_plane.normal());
    let opposite = component
        .faces()
        .into_iter()
        .filter(|face| face.id() != selected.id())
        .filter_map(|face| match face.surface() {
            Some(openrcad::geom::GeomSurface::Plane(plane))
                if plane.normal().dot(&selected_plane.normal()).abs() > 1.0 - 1.0e-8 =>
            {
                let anchor = face
                    .outer_wire()
                    .and_then(|wire| wire.edges().first().map(|edge| edge.source().point()))?;
                let separation = (selected_anchor - anchor).dot(&selected_normal).abs();
                Some((separation, face, anchor))
            }
            _ => None,
        })
        .max_by(|(left, _, _), (right, _, _)| left.total_cmp(right));
    let Some((separation, opposite, opposite_anchor)) = opposite else {
        return false;
    };
    if separation <= 1.0e-5 {
        return false;
    }
    let current = selected_anchor - opposite_anchor;
    let sweep = openrcad::foundation::Vec::new(
        current.x() + f64::from(translation.x),
        current.y() + f64::from(translation.y),
        current.z() + f64::from(translation.z),
    );
    let rebuilt = match crate::mock_kernel::consume_operation(
        "tangential move face",
        openrcad::algo::prism_operation_with_policy(
            &opposite,
            sweep,
            &openrcad::foundation::TolerancePolicy::STANDARD,
        ),
    ) {
        Ok(outcome) => outcome.solid,
        Err(reason) => {
            warnings.push(format!(
                "Move face '{node_id}': exact tangential rebuild failed ({reason})."
            ));
            return true;
        }
    };
    commit_component_replacement(
        node_id,
        body_index,
        component_index,
        source,
        vec![rebuilt],
        None,
        live,
        warnings,
    );
    true
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
    let (edited, histories) = if cylinder.internal {
        match crate::mock_kernel::union_with_history_diagnostic(component, &tool, None) {
            Ok((solid, history)) => (vec![solid], vec![history]),
            Err(reason) => {
                warnings.push(format!(
                    "Delete face '{node_id}': cylindrical hole healing failed ({reason}); the source was left unchanged."
                ));
                return;
            }
        }
    } else {
        match crate::mock_kernel::difference_bodies_with_history(component, &tool, None) {
            Some(outcome) if !outcome.bodies.is_empty() => (outcome.bodies, outcome.face_history),
            _ => {
                warnings.push(format!(
                    "Delete face '{node_id}': removing this external cylindrical wall would erase the complete component; the source was left unchanged."
                ));
                return;
            }
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
pub(crate) fn commit_component_replacement(
    node_id: &str,
    body_index: usize,
    component_index: usize,
    source: LiveBody,
    edited: Vec<KernelSolid>,
    histories: Option<Vec<crate::mock_kernel::BooleanFaceHistory>>,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) -> bool {
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
    commit_body_outputs(
        node_id,
        body_index,
        vec![LiveBody {
            id: node_id.to_string(),
            parts,
            pristine,
            sketch_source: None,
        }],
        live,
        warnings,
    )
}

/// Atomic replacement boundary shared by direct edits that yield one or more
/// bodies. Every output is accepted by the same representation and
/// tessellation gate before the source collection is mutated.
pub(crate) fn commit_body_outputs(
    node_id: &str,
    body_index: usize,
    outputs: Vec<LiveBody>,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) -> bool {
    if outputs.is_empty() || outputs.iter().any(|body| body.parts.is_empty()) {
        warnings.push(format!(
            "Direct edit '{node_id}': replacement produced an empty body; the source was left unchanged."
        ));
        return false;
    }
    for output in &outputs {
        for (part_index, solid) in output.parts.iter().enumerate() {
            if let Err(reason) = replacement_solid_commit_check(solid) {
                warnings.push(format!(
                    "Direct edit '{node_id}': output '{}' part {} failed the runtime commit gate ({reason}); the source was left unchanged.",
                    output.id,
                    part_index + 1
                ));
                return false;
            }
        }
    }

    live.remove(body_index);
    for output in outputs {
        apply_new(live, output);
    }
    true
}

fn replacement_solid_commit_check(solid: &KernelSolid) -> Result<(), String> {
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    if !solid.is_watertight() {
        return Err("the solid is not watertight".to_string());
    }
    if !solid.health_report().is_healthy() {
        return Err("the topology health report is not clean".to_string());
    }
    if !solid.has_complete_pcurves() {
        return Err("surface-backed coedges do not have complete pcurve coverage".to_string());
    }
    solid
        .validate_strict_with_policy(&policy)
        .map_err(|error| format!("strict topology validation failed: {error}"))?;
    openrcad::mesh::tessellate_checked_with_policy(solid, 0.05, 0.25, &policy)
        .map_err(|error| format!("checked tessellation failed: {error}"))?;
    Ok(())
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
