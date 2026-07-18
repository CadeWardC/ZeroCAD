use openrcad::algo::{
    blend_contour_operation_with_policy, boolean_operation_with_policy,
    chamfer_operation_with_policy, fillet_operation_with_policy, prism_operation_with_policy,
    revolve_operation_with_policy, shell_solid_operation_with_policy,
    skin_polygon_rings_operation_with_policy, transform_operation_with_policy, BlendContour,
    BlendKind, BooleanOp,
};
use openrcad::foundation::{Ax2, Dir, Pnt, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad::geom::{GeomSurface, Plane};
use openrcad::primitives::{
    make_box_operation_with_policy, make_cone_operation_with_policy,
    make_cylinder_operation_with_policy, make_sphere_operation_with_policy,
    make_wedge_operation_with_policy,
};
use openrcad::topo::{Edge, Face, OperationResult, Solid, Wire};

fn assert_phase3_gate(label: &str, result: &OperationResult<Solid>) {
    let policy = &TolerancePolicy::STANDARD;
    assert!(result.validation.is_valid(), "{label}: operation report");
    assert!(
        result.value.health_report_with_policy(policy).is_healthy(),
        "{label}: health"
    );
    assert!(
        result.value.is_watertight_with_policy(policy),
        "{label}: watertight"
    );
    assert!(
        result.value.has_complete_pcurves(),
        "{label}: pcurve coverage"
    );
    result
        .value
        .validate_strict_with_policy(policy)
        .unwrap_or_else(|error| panic!("{label}: strict validation: {error}"));
    result
        .history
        .validate()
        .unwrap_or_else(|error| panic!("{label}: history: {error}"));
    assert!(
        result
            .history
            .coverage_for_solid(&result.value)
            .is_complete(),
        "{label}: incomplete topology history"
    );
    let mesh = openrcad::mesh::tessellate_checked_with_policy(&result.value, 0.08, 0.35, policy)
        .unwrap_or_else(|error| panic!("{label}: strict tessellation: {error}"));
    assert!(!mesh.triangles.is_empty(), "{label}: empty mesh");
}

fn rectangle_face(x0: f64, x1: f64, y0: f64, y1: f64) -> Face {
    let points = [
        Pnt::new(x0, y0, 0.0),
        Pnt::new(x1, y0, 0.0),
        Pnt::new(x1, y1, 0.0),
        Pnt::new(x0, y1, 0.0),
    ];
    let wire = Wire::from_edges(
        (0..4).map(|index| Edge::between_points(points[index], points[(index + 1) % points.len()])),
    );
    Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        wire,
    )
}

#[test]
fn phase3_native_import_boolean_and_tessellation_gate() {
    let policy = &TolerancePolicy::STANDARD;
    let box_result =
        make_box_operation_with_policy(&Pnt::origin(), 12.0, 10.0, 8.0, policy).expect("box");
    let cylinder_result = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(6.0, 5.0, -1.0), Dir::dz()),
        2.0,
        10.0,
        policy,
    )
    .expect("cylinder");
    let cone_result =
        make_cone_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 4.0, 1.5, 7.0, policy)
            .expect("cone");
    let sphere_result =
        make_sphere_operation_with_policy(&Pnt::origin(), 4.0, policy).expect("sphere");
    let wedge_result = make_wedge_operation_with_policy(8.0, 6.0, 5.0, 2.0, policy).expect("wedge");
    for (label, result) in [
        ("box", &box_result),
        ("cylinder", &cylinder_result),
        ("cone", &cone_result),
        ("sphere", &sphere_result),
        ("wedge", &wedge_result),
    ] {
        assert_phase3_gate(label, result);
    }

    for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Common] {
        let tool = make_box_operation_with_policy(&Pnt::new(6.0, 2.0, 1.0), 8.0, 6.0, 6.0, policy)
            .expect("boolean tool");
        let result = boolean_operation_with_policy(&box_result.value, &tool.value, op, policy)
            .unwrap_or_else(|error| panic!("{op:?}: {error}"));
        assert_phase3_gate(&format!("boolean {op:?}"), &result);
    }

    let imported = openrcad::exchange::read_step_str_operation_with_policy_and_options(
        include_str!("fixtures/step/nist-bracket1-part.stp"),
        policy,
        openrcad::exchange::StepImportOptions::LEGACY_PCURVE_RECONSTRUCTION,
    )
    .expect("validated frozen STEP import");
    assert!(
        imported.recovery.was_modified(),
        "the legacy NIST fixture must report its pcurve reconstruction"
    );
    let imported_euler = imported.value.euler_characteristic();
    assert!(
        imported_euler.rem_euclid(2) == 0,
        "a closed orientable STEP shell must have an even Euler characteristic, got {imported_euler} (V={}, E={}, F={})",
        imported.value.vertex_count(),
        imported.value.edge_count(),
        imported.value.face_count(),
    );
    assert_phase3_gate("STEP import", &imported);
}

#[test]
fn phase3_general_operation_gate() {
    let policy = &TolerancePolicy::STANDARD;
    let source =
        make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy).expect("source box");

    let fillet = fillet_operation_with_policy(&source.value, 0.75, policy).expect("fillet");
    assert_phase3_gate("fillet", &fillet);
    let chamfer = chamfer_operation_with_policy(&source.value, 0.75, policy).expect("chamfer");
    assert_phase3_gate("chamfer", &chamfer);
    let shell = shell_solid_operation_with_policy(
        &source.value,
        0.5,
        &[source.value.faces()[0].clone()],
        policy,
    )
    .expect("shell");
    assert_phase3_gate("shell", &shell);

    let profile = rectangle_face(2.0, 4.0, -2.0, 2.0);
    let prism =
        prism_operation_with_policy(&profile, GeomVec::new(0.0, 0.0, 5.0), policy).expect("prism");
    assert_phase3_gate("prism", &prism);
    let full_revolve = revolve_operation_with_policy(
        &profile,
        Pnt::origin(),
        Dir::dy(),
        std::f64::consts::TAU,
        policy,
    )
    .expect("full revolve");
    assert_phase3_gate("full revolve", &full_revolve);
    let partial_revolve = revolve_operation_with_policy(
        &profile,
        Pnt::origin(),
        Dir::dy(),
        std::f64::consts::PI,
        policy,
    )
    .expect("partial revolve");
    assert_phase3_gate("partial revolve", &partial_revolve);

    let rings = vec![
        vec![
            Pnt::new(-2.0, -2.0, 0.0),
            Pnt::new(2.0, -2.0, 0.0),
            Pnt::new(2.0, 2.0, 0.0),
            Pnt::new(-2.0, 2.0, 0.0),
        ],
        vec![
            Pnt::new(-1.0, -1.0, 5.0),
            Pnt::new(1.0, -1.0, 5.0),
            Pnt::new(1.0, 1.0, 5.0),
            Pnt::new(-1.0, 1.0, 5.0),
        ],
    ];
    let skin = skin_polygon_rings_operation_with_policy(&rings, policy).expect("skin/loft");
    assert_phase3_gate("skin/loft", &skin);

    let transformed = transform_operation_with_policy(
        &source.value,
        &Trsf::translation(GeomVec::new(3.0, -2.0, 4.0)),
        false,
        policy,
    )
    .expect("transform");
    assert_phase3_gate("transform", &transformed);

    let blend = blend_contour_operation_with_policy(
        &source.value,
        &BlendContour::constant(
            vec![source.value.edges()[0].clone()],
            BlendKind::Fillet,
            0.5,
            None,
        ),
        policy,
    );
    if let Ok(blend) = blend {
        assert_phase3_gate("selected blend contour", &blend);
    }
}
