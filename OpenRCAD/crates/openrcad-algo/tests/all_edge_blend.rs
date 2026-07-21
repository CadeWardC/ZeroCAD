//! Wave 5D strict whole-solid Fillet/Chamfer contracts.

use openrcad_algo::{
    chamfer_all_edges_strict_operation_with_policy, chamfer_all_edges_strict_with_policy,
    fillet_all_edges_strict_operation_with_policy, fillet_all_edges_strict_with_policy,
    AllEdgeBlendError, AllEdgeBlendFailure,
};
use openrcad_foundation::{
    Ax1, Ax2, Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec,
};
use openrcad_primitives::{make_box_operation, make_cylinder_operation, make_sphere_operation};

fn transformed_box(scale: f64, angle_degrees: f64, translation: GeomVec) -> openrcad_topo::Solid {
    transformed_box_with_aspect(scale, 1.0, angle_degrees, translation)
}

fn transformed_box_with_aspect(
    scale: f64,
    aspect: f64,
    angle_degrees: f64,
    translation: GeomVec,
) -> openrcad_topo::Solid {
    let rotation = Trsf::rotation(
        &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
        angle_degrees.to_radians(),
    );
    make_box_operation(
        &Pnt::origin(),
        4.0 * scale * aspect,
        5.0 * scale,
        6.0 * scale,
    )
    .expect("strict all-edge source box")
    .value
    .transformed(&rotation)
    .transformed(&Trsf::translation(translation))
}

fn assert_strict(solid: &openrcad_topo::Solid, value: f64) {
    let context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[solid.bounding_box()],
        Some(value),
        1.0,
    )
    .expect("strict all-edge tolerance context");
    solid
        .validate_strict_with_policy(&context.policy)
        .expect("strict all-edge result");
    assert!(solid.is_watertight_with_policy(&context.policy));
    assert!(solid
        .health_report_with_policy(&context.policy)
        .is_healthy());
}

#[test]
fn primitive_fast_paths_return_complete_validated_history() {
    let source = transformed_box(1.0, 37.0, GeomVec::new(10.0, -7.0, 3.0));
    let fillet =
        fillet_all_edges_strict_operation_with_policy(&source, 0.4, &TolerancePolicy::STANDARD)
            .expect("strict all-edge box fillet");
    assert!(fillet.validation.is_valid());
    fillet.history.validate().expect("fillet history");
    assert!(fillet
        .history
        .coverage_for_solid(&fillet.value)
        .is_complete());
    assert_strict(&fillet.value, 0.4);

    let chamfer =
        chamfer_all_edges_strict_operation_with_policy(&source, 0.4, &TolerancePolicy::STANDARD)
            .expect("strict all-edge box chamfer");
    assert!(chamfer.validation.is_valid());
    chamfer.history.validate().expect("chamfer history");
    assert!(chamfer
        .history
        .coverage_for_solid(&chamfer.value)
        .is_complete());
    assert_strict(&chamfer.value, 0.4);

    let cylinder =
        make_cylinder_operation(&Ax2::new(Pnt::origin(), Dir::new(0.0, 0.0, 1.0)), 3.0, 7.0)
            .expect("strict all-edge cylinder")
            .value
            .transformed(&Trsf::rotation(
                &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
                37.0_f64.to_radians(),
            ))
            .transformed(&Trsf::translation(GeomVec::new(10.0, -7.0, 3.0)));
    let cylinder_fillet =
        fillet_all_edges_strict_operation_with_policy(&cylinder, 0.4, &TolerancePolicy::STANDARD)
            .expect("strict all-edge cylinder fillet");
    assert!(cylinder_fillet.validation.is_valid());
    assert!(cylinder_fillet
        .history
        .coverage_for_solid(&cylinder_fillet.value)
        .is_complete());
    assert_strict(&cylinder_fillet.value, 0.4);
    let cylinder_chamfer =
        chamfer_all_edges_strict_operation_with_policy(&cylinder, 0.4, &TolerancePolicy::STANDARD)
            .expect("strict all-edge cylinder chamfer");
    assert!(cylinder_chamfer.validation.is_valid());
    assert!(cylinder_chamfer
        .history
        .coverage_for_solid(&cylinder_chamfer.value)
        .is_complete());
    assert_strict(&cylinder_chamfer.value, 0.4);
}

#[test]
fn strict_all_edge_paths_hold_across_scale_rotation_and_far_origin() {
    let cases = [
        (1.0e-3, 0.0, GeomVec::ZERO),
        (1.0e-3, 37.0, GeomVec::new(1.0e6, -1.0e6, 5.0e5)),
        (1.0, 90.0, GeomVec::new(1.0e6, -1.0e6, 5.0e5)),
        (1.0e3, 37.0, GeomVec::new(1.0e9, -1.0e9, 5.0e8)),
    ];
    for (scale, angle, translation) in cases {
        let source = transformed_box(scale, angle, translation);
        let value = scale * 0.2;
        let fillet = fillet_all_edges_strict_with_policy(
            &source,
            value,
            &TolerancePolicy::STANDARD,
        )
        .unwrap_or_else(|error| {
            panic!(
                "strict fillet failed at scale={scale}, angle={angle}, translation={translation:?}: {error:?}"
            )
        });
        assert_strict(&fillet, value);
        let chamfer = chamfer_all_edges_strict_with_policy(
            &source,
            value,
            &TolerancePolicy::STANDARD,
        )
        .unwrap_or_else(|error| {
            panic!(
                "strict chamfer failed at scale={scale}, angle={angle}, translation={translation:?}: {error:?}"
            )
        });
        assert_strict(&chamfer, value);
    }
}

#[test]
fn extreme_aspect_sweep_is_strict_or_deterministically_typed_and_atomic() {
    let cases = [
        (1.0e-3, 1.0e-3, 0.0, GeomVec::ZERO),
        (1.0, 1.0e-3, 37.0, GeomVec::new(1.0e6, -1.0e6, 5.0e5)),
        (1.0, 1.0e3, 90.0, GeomVec::new(1.0e6, -1.0e6, 5.0e5)),
        (1.0e3, 1.0e3, 37.0, GeomVec::new(1.0e9, -1.0e9, 5.0e8)),
    ];
    for (scale, aspect, angle, translation) in cases {
        let source = transformed_box_with_aspect(scale, aspect, angle, translation);
        let original_brep = source.brep().clone();
        let value = scale * aspect.min(1.0) * 0.2;

        let fillet =
            fillet_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD);
        assert_eq!(
            fillet,
            fillet_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD),
            "fillet must be deterministic at scale={scale}, aspect={aspect}"
        );
        match fillet {
            Ok(candidate) => assert_strict(&candidate, value),
            Err(error) => assert!(
                !error.blockers().is_empty(),
                "aspect rejection must identify blockers: {error:?}"
            ),
        }

        let chamfer =
            chamfer_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD);
        assert_eq!(
            chamfer,
            chamfer_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD),
            "chamfer must be deterministic at scale={scale}, aspect={aspect}"
        );
        match chamfer {
            Ok(candidate) => assert_strict(&candidate, value),
            Err(error) => assert!(
                !error.blockers().is_empty(),
                "aspect rejection must identify blockers: {error:?}"
            ),
        }
        assert!(std::sync::Arc::ptr_eq(&original_brep, source.brep()));
    }
}

#[test]
fn global_limit_reports_every_blocking_edge_without_mutating_the_source() {
    let source = transformed_box(1.0, 0.0, GeomVec::ZERO);
    let original_brep = source.brep().clone();
    let expected = source.edges().len();
    let first = fillet_all_edges_strict_with_policy(&source, 2.0, &TolerancePolicy::STANDARD)
        .expect_err("half the minimum box span must reject");
    let second = fillet_all_edges_strict_with_policy(&source, 2.0, &TolerancePolicy::STANDARD)
        .expect_err("the same request must reject deterministically");
    assert_eq!(first, second);
    assert_eq!(first.blockers().len(), expected);
    assert_eq!(
        first
            .blockers()
            .iter()
            .map(|blocker| blocker.ordinal)
            .collect::<Vec<_>>(),
        (0..expected).collect::<Vec<_>>()
    );
    assert!(first.blockers().iter().all(|blocker| matches!(
        blocker.failure,
        AllEdgeBlendFailure::WholeSolid(openrcad_algo::BlendError::ParameterTooLarge { .. })
    )));
    assert!(std::sync::Arc::ptr_eq(&original_brep, source.brep()));
    assert_strict(&source, 2.0);
}

#[test]
fn arbitrary_analytic_solid_returns_all_discovered_blockers_atomically() {
    let source = make_sphere_operation(&Pnt::origin(), 4.0)
        .expect("strict all-edge sphere source")
        .value;
    let original_brep = source.brep().clone();
    let error = chamfer_all_edges_strict_with_policy(&source, 0.25, &TolerancePolicy::STANDARD)
        .expect_err("smooth sphere construction edges are not chamferable");
    let AllEdgeBlendError::BlockingEdges { blockers, .. } = error else {
        panic!("expected typed edge blockers, got {error:?}");
    };
    assert!(!blockers.is_empty());
    assert!(blockers
        .windows(2)
        .all(|pair| pair[0].ordinal < pair[1].ordinal));
    assert!(std::sync::Arc::ptr_eq(&original_brep, source.brep()));
    assert_strict(&source, 0.25);
}

#[test]
fn invalid_values_reject_before_geometry() {
    let source = transformed_box(1.0, 0.0, GeomVec::ZERO);
    for value in [f64::NAN, f64::INFINITY, -1.0] {
        assert!(matches!(
            fillet_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD),
            Err(AllEdgeBlendError::InvalidValue { .. })
        ));
        assert!(matches!(
            chamfer_all_edges_strict_with_policy(&source, value, &TolerancePolicy::STANDARD),
            Err(AllEdgeBlendError::InvalidValue { .. })
        ));
    }
}
