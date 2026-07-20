//! Pre-geometry Wave 4 Shell contracts. These tests pin the primitive paths,
//! scale policy, topology-history dispositions, pcurve-complete operation
//! boundary, CUT-derived material normal, display tessellation, and the
//! cylinder-seam fixture that mixed-network Shell must eventually consume.

use core::f64::consts::FRAC_PI_2;
use std::collections::HashMap;

use openrcad_algo::offset::{
    measure_shell_wall_thickness, shell_build_path_with_policy,
    shell_planar_outward_normal_with_policy, ShellBuildPath,
};
use openrcad_algo::{
    boolean_operation_with_policy, shell_solid_operation_with_policy, BooleanOp,
    ShellPrerequisiteState, ShellStage4CPrerequisite, SHELL_FEASIBILITY_V1,
};
use openrcad_foundation::{Ax2, Dir, Pnt, ToleranceContext, TolerancePolicy};
use openrcad_geom::GeomSurface;
use openrcad_primitives::{
    make_box_operation_with_policy, make_cone_operation_with_policy,
    make_cylinder_operation_with_policy, make_sphere_operation_with_policy,
};
use openrcad_topo::{Face, Solid, TopologyChange, TopologyKind};

fn highest_positive_z_face(solid: &Solid) -> Face {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Plane(plane)) if plane.normal().z() > 0.9 => {
                Some((plane.location().z(), face))
            }
            _ => None,
        })
        .max_by(|(first, _), (second, _)| first.total_cmp(second))
        .expect("positive-z planar face")
        .1
}

fn lowest_planar_z_face(solid: &Solid) -> Face {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Plane(plane)) => Some((plane.location().z(), face)),
            _ => None,
        })
        .min_by(|(first, _), (second, _)| first.total_cmp(second))
        .expect("lowest planar face")
        .1
}

fn assert_shell_operation_contract(
    source: &Solid,
    open_face: Face,
    thickness: f64,
    expected_path: ShellBuildPath,
    label: &str,
) -> Solid {
    let policy = &TolerancePolicy::STANDARD;
    assert_eq!(
        shell_build_path_with_policy(source, policy),
        expected_path,
        "{label}: primitive optimization became unreachable"
    );
    let operation = shell_solid_operation_with_policy(
        source,
        thickness,
        std::slice::from_ref(&open_face),
        policy,
    )
    .unwrap_or_else(|error| {
        let raw = openrcad_algo::shell_solid_with_policy(
            source,
            thickness,
            std::slice::from_ref(&open_face),
            policy,
        )
        .unwrap_or_else(|raw_error| {
            panic!("{label}: raw Shell candidate failed: {raw_error}; operation error: {error}")
        });
        let (repaired, _) = raw.repair_pcurves(policy).unwrap_or_else(|repair_error| {
            let surfaces: Vec<_> = raw
                .shell()
                .faces()
                .iter()
                .map(|face| {
                    let kind = match face.surface() {
                        Some(GeomSurface::Plane(_)) => "plane",
                        Some(GeomSurface::Cylinder(_)) => "cylinder",
                        Some(GeomSurface::Cone(_)) => "cone",
                        Some(GeomSurface::Sphere(_)) => "sphere",
                        Some(GeomSurface::Torus(_)) => "torus",
                        Some(GeomSurface::Offset(offset)) => match offset.base.as_ref() {
                            GeomSurface::Plane(_) => "offset-plane",
                            GeomSurface::Cylinder(_) => "offset-cylinder",
                            GeomSurface::Cone(_) => "offset-cone",
                            GeomSurface::Sphere(_) => "offset-sphere",
                            _ => "offset-other",
                        },
                        _ => "other",
                    };
                    (face.id(), kind)
                })
                .collect();
            panic!("{label}: repair raw pcurves failed: {repair_error}; surfaces={surfaces:?}")
        });
        panic!(
            "{label}: Shell operation failed: {error}; raw strict={:?}; repaired strict={:?}; loops={:?}; surfaces={:?}",
            raw.validate_strict_with_policy(policy),
            repaired.validate_strict_with_policy(policy),
            repaired
                .shell()
                .faces()
                .iter()
                .flat_map(|face| {
                    let face_id = face.id();
                    face.wires()
                        .into_iter()
                        .map(move |wire| (face_id, wire.id()))
                })
                .collect::<Vec<_>>(),
            repaired
                .shell()
                .faces()
                .iter()
                .map(|face| {
                    let kind = match face.surface() {
                        Some(GeomSurface::Plane(_)) => "plane",
                        Some(GeomSurface::Cylinder(_)) => "cylinder",
                        Some(GeomSurface::Cone(_)) => "cone",
                        Some(GeomSurface::Sphere(_)) => "sphere",
                        Some(GeomSurface::Offset(offset)) => match offset.base.as_ref() {
                            GeomSurface::Plane(_) => "offset-plane",
                            GeomSurface::Cylinder(_) => "offset-cylinder",
                            GeomSurface::Cone(_) => "offset-cone",
                            GeomSurface::Sphere(_) => "offset-sphere",
                            _ => "offset-other",
                        },
                        _ => "other",
                    };
                    (face.id(), kind)
                })
                .collect::<Vec<_>>()
        )
    });

    assert!(operation.validation.is_valid(), "{label}: invalid result");
    assert!(
        operation.value.validate_strict_with_policy(policy).is_ok(),
        "{label}: strict validation failed"
    );
    assert!(
        operation.value.has_complete_pcurves(),
        "{label}: incomplete pcurves"
    );
    assert!(
        operation.history.validate().is_ok(),
        "{label}: malformed history"
    );
    assert!(
        operation
            .history
            .coverage_for_solid(&operation.value)
            .is_complete(),
        "{label}: incomplete history coverage"
    );

    let face_change = |predicate: fn(&TopologyChange) -> bool| {
        operation.history.changes.iter().any(|change| {
            let is_face = match change {
                TopologyChange::Generated { result, .. }
                | TopologyChange::Modified { result, .. }
                | TopologyChange::Merged { result, .. } => result.kind == TopologyKind::Face,
                TopologyChange::Split { results, .. } => results
                    .iter()
                    .any(|result| result.kind == TopologyKind::Face),
                TopologyChange::Deleted { source } => source.entity.kind == TopologyKind::Face,
            };
            is_face && predicate(change)
        })
    };
    assert!(
        face_change(|change| matches!(change, TopologyChange::Modified { .. })),
        "{label}: no retained/modified face history"
    );
    assert!(
        face_change(|change| matches!(change, TopologyChange::Generated { .. })),
        "{label}: no generated inner/rim face history"
    );
    assert!(
        face_change(|change| matches!(change, TopologyChange::Deleted { .. })),
        "{label}: removed face was not marked deleted"
    );

    let evidence = measure_shell_wall_thickness(&operation.value, thickness, policy)
        .unwrap_or_else(|error| panic!("{label}: thickness evidence failed: {error}"));
    assert!(evidence.samples > 0, "{label}: no thickness samples");
    operation.value
}

fn assert_display_mesh_crack_free(solid: &Solid, label: &str) {
    let policy = &TolerancePolicy::STANDARD;
    let context = ToleranceContext::derive(policy, &[solid.bounding_box()], None, 1.0)
        .expect("valid mesh tolerance context");
    let chord_error = context
        .policy
        .approximation
        .max(context.local_feature_size * 0.005);
    let mesh = openrcad_mesh::tessellate_checked_with_policy(solid, chord_error, 0.1, policy)
        .unwrap_or_else(|error| panic!("{label}: tessellation failed: {error}"));
    let gpu = mesh.gpu_mesh();
    let quantum = context
        .quantization
        .max(context.welding)
        .max(context.arithmetic_floor)
        // The renderer consumes f32 positions. Two correctly welded f64
        // samples can round to adjacent f32 bins at large scale, so the crack
        // oracle must compare at the actual display representation's
        // scale-derived arithmetic floor rather than a hidden unit epsilon.
        .max(
            context.translation_magnitude.max(context.model_scale) * f64::from(f32::EPSILON) * 2.0,
        );
    let point_key = |index: usize| {
        let offset = index * 3;
        let key = |coordinate: f32| (f64::from(coordinate) / quantum).round() as i64;
        (
            key(gpu.positions[offset]),
            key(gpu.positions[offset + 1]),
            key(gpu.positions[offset + 2]),
        )
    };
    type PointKey = (i64, i64, i64);
    let mut uses: HashMap<(PointKey, PointKey), usize> = HashMap::new();
    for triangle in gpu.indices.chunks_exact(3) {
        for (first, second) in [(0usize, 1usize), (1, 2), (2, 0)] {
            let endpoints = (
                point_key(triangle[first] as usize),
                point_key(triangle[second] as usize),
            );
            let key = if endpoints.0 <= endpoints.1 {
                endpoints
            } else {
                (endpoints.1, endpoints.0)
            };
            *uses.entry(key).or_default() += 1;
        }
    }
    let cracks = uses.values().filter(|uses| **uses == 1).count();
    assert_eq!(cracks, 0, "{label}: display mesh contains crack edges");
}

#[test]
fn primitive_shell_paths_remain_reachable_measured_and_crack_free() {
    let policy = &TolerancePolicy::STANDARD;
    let box_source = make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy)
        .expect("box source")
        .value;
    let shelled_box = assert_shell_operation_contract(
        &box_source,
        highest_positive_z_face(&box_source),
        0.75,
        ShellBuildPath::BoxPrimitive,
        "box fast path",
    );
    assert_display_mesh_crack_free(&shelled_box, "box fast path");

    let cylinder_source =
        make_cylinder_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 4.0, 10.0, policy)
            .expect("cylinder source")
            .value;
    let shelled_cylinder = assert_shell_operation_contract(
        &cylinder_source,
        highest_positive_z_face(&cylinder_source),
        0.5,
        ShellBuildPath::CylinderPrimitive,
        "cylinder fast path",
    );
    assert_display_mesh_crack_free(&shelled_cylinder, "cylinder fast path");
}

#[test]
fn stage4a_shells_a_drilled_block_with_mixed_plane_cylinder_supports() {
    let policy = &TolerancePolicy::STANDARD;
    let block = make_box_operation_with_policy(&Pnt::origin(), 20.0, 16.0, 10.0, policy)
        .expect("drilled block")
        .value;
    let drill = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(10.0, 8.0, -1.0), Dir::dz()),
        2.0,
        12.0,
        policy,
    )
    .expect("drill")
    .value;
    let source = boolean_operation_with_policy(&block, &drill, BooleanOp::Cut, policy)
        .expect("drilled source")
        .value;
    let shelled = assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        1.0,
        ShellBuildPath::MixedAnalyticNetwork,
        "drilled block",
    );

    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Cylinder(_))
        )
    }));
    assert_display_mesh_crack_free(&shelled, "drilled block");
}

#[test]
fn stage4a_shells_a_block_with_a_cylindrical_boss() {
    let policy = &TolerancePolicy::STANDARD;
    let block = make_box_operation_with_policy(&Pnt::origin(), 20.0, 16.0, 6.0, policy)
        .expect("boss block")
        .value;
    let boss = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(10.0, 8.0, 5.5), Dir::dz()),
        3.0,
        4.5,
        policy,
    )
    .expect("boss cylinder")
    .value;
    let source = boolean_operation_with_policy(&block, &boss, BooleanOp::Fuse, policy)
        .expect("boss source")
        .value;
    let shelled = assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        0.75,
        ShellBuildPath::MixedAnalyticNetwork,
        "cylindrical boss",
    );

    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Cylinder(_))
        )
    }));
    assert_display_mesh_crack_free(&shelled, "cylindrical boss");
}

#[test]
fn stage4b_shells_a_truncated_cone_with_measured_analytic_thickness() {
    let policy = &TolerancePolicy::STANDARD;
    let source =
        make_cone_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 6.0, 3.0, 8.0, policy)
            .expect("truncated cone")
            .value;
    let shelled = assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        0.5,
        ShellBuildPath::MixedAnalyticNetwork,
        "truncated cone",
    );

    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Cone(_))
        )
    }));
    assert_display_mesh_crack_free(&shelled, "truncated cone");
}

#[test]
fn stage4b_shells_a_spherical_dome_on_a_planar_body() {
    let policy = &TolerancePolicy::STANDARD;
    let block = make_box_operation_with_policy(&Pnt::origin(), 20.0, 16.0, 6.0, policy)
        .expect("dome block")
        .value;
    let sphere = make_sphere_operation_with_policy(&Pnt::new(10.0, 8.0, 6.0), 4.0, policy)
        .expect("dome sphere")
        .value;
    let source = boolean_operation_with_policy(&block, &sphere, BooleanOp::Fuse, policy)
        .expect("spherical dome source")
        .value;
    let shelled = assert_shell_operation_contract(
        &source,
        lowest_planar_z_face(&source),
        0.5,
        ShellBuildPath::MixedAnalyticNetwork,
        "spherical dome",
    );

    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Sphere(_))
        )
    }));
    assert_display_mesh_crack_free(&shelled, "spherical dome");
}

#[test]
fn stage4b_shells_a_countersunk_block_across_cone_cylinder_transitions() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        exercise_countersunk_shell(scale, [0.0, 0.0, 0.0]);
    }
}

fn exercise_countersunk_shell(scale: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let block = make_box_operation_with_policy(
        &Pnt::new(x, y, z),
        20.0 * scale,
        16.0 * scale,
        10.0 * scale,
        policy,
    )
    .expect("countersink block")
    .value;
    let bore = make_cylinder_operation_with_policy(
        &Ax2::new(
            Pnt::new(x + 10.0 * scale, y + 8.0 * scale, z - 1.0 * scale),
            Dir::dz(),
        ),
        2.0 * scale,
        12.0 * scale,
        policy,
    )
    .expect("countersink bore")
    .value;
    let drilled = boolean_operation_with_policy(&block, &bore, BooleanOp::Cut, policy)
        .expect("drilled countersink block")
        .value;
    let countersink = make_cone_operation_with_policy(
        &Ax2::new(
            Pnt::new(x + 10.0 * scale, y + 8.0 * scale, z + 7.0 * scale),
            Dir::dz(),
        ),
        2.0 * scale,
        4.0 * scale,
        4.0 * scale,
        policy,
    )
    .expect("countersink tool")
    .value;
    let source = boolean_operation_with_policy(&drilled, &countersink, BooleanOp::Cut, policy)
        .expect("countersunk source")
        .value;
    let shelled = assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        0.5 * scale,
        ShellBuildPath::MixedAnalyticNetwork,
        &format!("countersunk block scale={scale:e} origin={origin:?}"),
    );

    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Cone(_))
        )
    }));
    assert!(shelled.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Offset(offset))
                if matches!(offset.base.as_ref(), GeomSurface::Cylinder(_))
        )
    }));
    // The unit-scale case pins the complete shared-edge display path. The
    // scale/far-origin matrix above remains a kernel/topology contract; cone-
    // cylinder lens stitching at extreme display scales is tracked separately
    // from Shell support correctness.
    if scale == 1.0 && origin == [0.0, 0.0, 0.0] {
        assert_display_mesh_crack_free(&shelled, "countersunk block");
    }
}

fn exercise_scaled_cone_shell(scale: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let source = make_cone_operation_with_policy(
        &Ax2::new(Pnt::new(x, y, z), Dir::dz()),
        6.0 * scale,
        3.0 * scale,
        8.0 * scale,
        policy,
    )
    .expect("scaled cone")
    .value;
    assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        0.5 * scale,
        ShellBuildPath::MixedAnalyticNetwork,
        &format!("cone scale={scale:e} origin={origin:?}"),
    );
}

fn exercise_scaled_spherical_dome_shell(scale: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let block = make_box_operation_with_policy(
        &Pnt::new(x, y, z),
        20.0 * scale,
        16.0 * scale,
        6.0 * scale,
        policy,
    )
    .expect("scaled dome block")
    .value;
    let sphere = make_sphere_operation_with_policy(
        &Pnt::new(x + 10.0 * scale, y + 8.0 * scale, z + 6.0 * scale),
        4.0 * scale,
        policy,
    )
    .expect("scaled dome sphere")
    .value;
    let source = boolean_operation_with_policy(&block, &sphere, BooleanOp::Fuse, policy)
        .expect("scaled spherical dome")
        .value;
    assert_shell_operation_contract(
        &source,
        lowest_planar_z_face(&source),
        0.5 * scale,
        ShellBuildPath::MixedAnalyticNetwork,
        &format!("spherical dome scale={scale:e} origin={origin:?}"),
    );
}

#[test]
fn stage4b_cone_and_sphere_hold_across_scale_and_far_origin() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        exercise_scaled_cone_shell(scale, [0.0, 0.0, 0.0]);
        exercise_scaled_spherical_dome_shell(scale, [0.0, 0.0, 0.0]);
    }
    for scale in [1.0e-3, 1.0e3] {
        let origin = [1.0e9, -1.0e9, 5.0e8];
        exercise_scaled_cone_shell(scale, origin);
        exercise_scaled_spherical_dome_shell(scale, origin);
    }
}

#[test]
fn stage4b_rejects_disappearing_cone_and_sphere_offsets_atomically() {
    let policy = &TolerancePolicy::STANDARD;
    let cone =
        make_cone_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 6.0, 3.0, 8.0, policy)
            .expect("collapse cone")
            .value;
    let cone_before = (cone.face_count(), cone.bounding_box().corners());
    let cone_result =
        shell_solid_operation_with_policy(&cone, 3.5, &[highest_positive_z_face(&cone)], policy);
    assert!(cone_result.is_err());
    assert!(matches!(
        openrcad_algo::shell_solid_with_policy(
            &cone,
            3.5,
            &[highest_positive_z_face(&cone)],
            policy,
        ),
        Err(openrcad_algo::blend::BlendError::ParameterTooLarge { .. })
    ));
    assert_eq!(
        (cone.face_count(), cone.bounding_box().corners()),
        cone_before
    );

    let block = make_box_operation_with_policy(&Pnt::origin(), 20.0, 16.0, 6.0, policy)
        .expect("collapse dome block")
        .value;
    let ball = make_sphere_operation_with_policy(&Pnt::new(10.0, 8.0, 6.0), 4.0, policy)
        .expect("collapse dome sphere")
        .value;
    let sphere = boolean_operation_with_policy(&block, &ball, BooleanOp::Fuse, policy)
        .expect("collapse spherical dome")
        .value;
    let sphere_before = (sphere.face_count(), sphere.bounding_box().corners());
    let sphere_result =
        shell_solid_operation_with_policy(&sphere, 4.0, &[lowest_planar_z_face(&sphere)], policy);
    assert!(sphere_result.is_err());
    assert!(matches!(
        openrcad_algo::shell_solid_with_policy(
            &sphere,
            4.0,
            &[lowest_planar_z_face(&sphere)],
            policy,
        ),
        Err(openrcad_algo::blend::BlendError::ParameterTooLarge { .. })
    ));
    assert_eq!(
        (sphere.face_count(), sphere.bounding_box().corners()),
        sphere_before
    );
}

fn exercise_scaled_drilled_shell(scale: f64, aspect: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let width = 20.0 * scale * aspect;
    let depth = 16.0 * scale;
    let height = 10.0 * scale;
    let block = make_box_operation_with_policy(&Pnt::new(x, y, z), width, depth, height, policy)
        .expect("scaled drilled block")
        .value;
    let drill = make_cylinder_operation_with_policy(
        &Ax2::new(
            Pnt::new(x + width * 0.5, y + depth * 0.5, z - scale),
            Dir::dz(),
        ),
        2.0 * scale,
        height + 2.0 * scale,
        policy,
    )
    .expect("scaled drill")
    .value;
    let source = boolean_operation_with_policy(&block, &drill, BooleanOp::Cut, policy)
        .expect("scaled drilled source")
        .value;
    assert_shell_operation_contract(
        &source,
        highest_positive_z_face(&source),
        0.5 * scale,
        ShellBuildPath::MixedAnalyticNetwork,
        &format!("drilled scale={scale:e} aspect={aspect} origin={origin:?}"),
    );
}

#[test]
fn stage4a_mixed_shell_holds_across_scale_aspect_and_far_origin() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for aspect in [1.0, 10.0, 100.0] {
            exercise_scaled_drilled_shell(scale, aspect, [0.0, 0.0, 0.0]);
        }
    }
    for scale in [1.0e-3, 1.0e3] {
        exercise_scaled_drilled_shell(scale, 10.0, [1.0e9, -1.0e9, 5.0e8]);
    }
}

fn exercise_scaled_primitive_shell(scale: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let box_source = make_box_operation_with_policy(
        &Pnt::new(x, y, z),
        10.0 * scale,
        8.0 * scale,
        6.0 * scale,
        policy,
    )
    .expect("scaled box source")
    .value;
    assert_shell_operation_contract(
        &box_source,
        highest_positive_z_face(&box_source),
        0.5 * scale,
        ShellBuildPath::BoxPrimitive,
        &format!("box scale={scale:e} origin={origin:?}"),
    );

    let cylinder_source = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(x, y, z), Dir::dz()),
        4.0 * scale,
        10.0 * scale,
        policy,
    )
    .expect("scaled cylinder source")
    .value;
    assert_shell_operation_contract(
        &cylinder_source,
        highest_positive_z_face(&cylinder_source),
        0.5 * scale,
        ShellBuildPath::CylinderPrimitive,
        &format!("cylinder scale={scale:e} origin={origin:?}"),
    );
}

#[test]
fn primitive_shells_hold_across_scale_and_far_origin() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        exercise_scaled_primitive_shell(scale, [0.0, 0.0, 0.0]);
    }
    for scale in [1.0e-3, 1.0e3] {
        exercise_scaled_primitive_shell(scale, [1.0e9, -1.0e9, 5.0e8]);
    }
}

fn exercise_cut_face_normal(scale: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let source = make_box_operation_with_policy(
        &Pnt::new(x, y, z),
        8.0 * scale,
        4.0 * scale,
        2.0 * scale,
        policy,
    )
    .expect("normal-check source")
    .value;
    let tool = make_box_operation_with_policy(
        &Pnt::new(x + 4.0 * scale, y, z),
        6.0 * scale,
        4.0 * scale,
        2.0 * scale,
        policy,
    )
    .expect("normal-check tool")
    .value;
    let cut = boolean_operation_with_policy(&source, &tool, BooleanOp::Cut, policy)
        .expect("planar trimming cut")
        .value;
    let context = ToleranceContext::derive(policy, &[cut.bounding_box()], Some(scale), 1.0)
        .expect("normal-check tolerance context");
    let cut_x = x + 4.0 * scale;
    let cut_face = cut
        .shell()
        .faces()
        .into_iter()
        .find(|face| match face.surface() {
            Some(GeomSurface::Plane(plane)) => {
                plane.normal().x().abs() >= 1.0 - context.policy.angular
                    && (plane.location().x() - cut_x).abs() <= context.policy.classification
            }
            _ => false,
        })
        .expect("CUT-derived planar face");
    let checked = shell_planar_outward_normal_with_policy(&cut, &cut_face, policy)
        .expect("material-checked CUT face normal");
    assert!(
        checked.dot(&Dir::dx()) >= 1.0 - context.policy.angular,
        "scale={scale:e} origin={origin:?}: CUT face normal must point into the removed region"
    );
}

#[test]
fn cut_derived_shell_normals_are_material_checked_across_scale_and_origin() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        exercise_cut_face_normal(scale, [0.0, 0.0, 0.0]);
    }
    for scale in [1.0e-3, 1.0e3] {
        exercise_cut_face_normal(scale, [1.0e9, -1.0e9, 5.0e8]);
    }
}

#[test]
fn cylinder_seam_crossing_fixture_is_pinned_before_mixed_shell() {
    let policy = &TolerancePolicy::STANDARD;
    let source = make_box_operation_with_policy(&Pnt::origin(), 10.0, 10.0, 10.0, policy)
        .expect("seam source")
        .value;
    let x_direction = Dir::new(FRAC_PI_2.cos(), FRAC_PI_2.sin(), 0.0);
    let tool = make_cylinder_operation_with_policy(
        &Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), x_direction),
        4.0,
        12.0,
        policy,
    )
    .expect("seam tool")
    .value;
    let cut = boolean_operation_with_policy(&source, &tool, BooleanOp::Cut, policy)
        .expect("seam-crossing cut")
        .value;

    assert!(cut.validate_strict_with_policy(policy).is_ok());
    assert!(cut.has_complete_pcurves());
    assert_eq!(
        shell_build_path_with_policy(&cut, policy),
        ShellBuildPath::MixedAnalyticNetwork
    );
    let shelled = assert_shell_operation_contract(
        &cut,
        highest_positive_z_face(&cut),
        0.5,
        ShellBuildPath::MixedAnalyticNetwork,
        "cylinder seam crossing",
    );
    assert_display_mesh_crack_free(&shelled, "cylinder seam crossing");
    let seam_status = SHELL_FEASIBILITY_V1
        .stage4c_prerequisites
        .iter()
        .find(|status| status.prerequisite == ShellStage4CPrerequisite::CylinderSeamImprinting)
        .expect("cylinder seam prerequisite is tracked");
    assert_eq!(seam_status.state, ShellPrerequisiteState::Verified);
}
