use openrcad_algo::{
    boolean_operation_with_policy, fillet_operation_with_policy,
    split_solid_by_plane_operation_with_policy, transform_operation_with_policy, BooleanOp,
};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy, Trsf};
use openrcad_geom::Plane;
use openrcad_primitives::{
    make_box_operation_with_policy, make_cone_operation_with_policy,
    make_cylinder_operation_with_policy, make_sphere_operation_with_policy,
};
use openrcad_topo::{OperationResult, RecoveryAction, Solid, TopologyHistory};

fn assert_strict_solids(label: &str, solids: &[Solid], history: &TopologyHistory) {
    let policy = &TolerancePolicy::STANDARD;
    assert!(!solids.is_empty(), "{label}: no output solids");
    for solid in solids {
        assert!(
            solid.is_watertight_with_policy(policy),
            "{label}: open solid"
        );
        assert!(solid.has_complete_pcurves(), "{label}: missing pcurve");
        solid
            .validate_strict_with_policy(policy)
            .unwrap_or_else(|error| panic!("{label}: strict validation: {error}"));
        let mesh = openrcad_mesh::tessellate_checked_with_policy(solid, 0.04, 0.25, policy)
            .unwrap_or_else(|error| panic!("{label}: checked tessellation: {error}"));
        assert!(!mesh.triangles.is_empty(), "{label}: empty mesh");
        assert!(
            openrcad_mesh::mass_properties(&mesh)
                .is_some_and(|properties| properties.volume > policy.resolution),
            "{label}: no positive volume"
        );
    }
    history
        .validate()
        .unwrap_or_else(|error| panic!("{label}: invalid history: {error}"));
    assert!(
        history.coverage_for_solids(solids).is_complete(),
        "{label}: incomplete history coverage"
    );
}

#[test]
fn common_split_and_scale_share_the_strict_operation_gate() {
    let policy = &TolerancePolicy::STANDARD;
    let source =
        make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy).expect("source box");
    let tool = make_box_operation_with_policy(&Pnt::new(5.0, 2.0, -1.0), 8.0, 5.0, 8.0, policy)
        .expect("common tool");

    let common =
        boolean_operation_with_policy(&source.value, &tool.value, BooleanOp::Common, policy)
            .expect("positive-volume Common");
    assert_strict_solids(
        "intersect",
        core::slice::from_ref(&common.value),
        &common.history,
    );

    let plane = Plane::from_point_normal(Pnt::new(4.0, 0.0, 0.0), Dir::dx());
    let split = split_solid_by_plane_operation_with_policy(&source.value, &plane, policy)
        .expect("planar split");
    assert!(split.validation.is_valid(), "split operation report");
    let split_solids = split.value.solids().cloned().collect::<Vec<_>>();
    assert_strict_solids("split", &split_solids, &split.history);

    let cylinder =
        make_cylinder_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 6.0, policy)
            .expect("cylinder");
    let cone =
        make_cone_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 1.0, 6.0, policy)
            .expect("cone");
    let sphere = make_sphere_operation_with_policy(&Pnt::origin(), 3.0, policy).expect("sphere");
    let blend = fillet_operation_with_policy(&source.value, 0.5, policy).expect("fillet");
    for (source_label, scale_source) in [
        ("box", &source.value),
        ("cylinder", &cylinder.value),
        ("cone", &cone.value),
        ("sphere", &sphere.value),
        ("boolean", &common.value),
        ("blend", &blend.value),
    ] {
        let source_mesh =
            openrcad_mesh::tessellate_checked_with_policy(scale_source, 0.04, 0.25, policy)
                .expect("source measurement mesh");
        let source_properties =
            openrcad_mesh::mass_properties(&source_mesh).expect("source mass properties");
        for factor in [0.1_f64, 0.5, 2.0, 100.0] {
            let label = format!("scale {source_label} by {factor}");
            let scaled: OperationResult<Solid> = transform_operation_with_policy(
                scale_source,
                &Trsf::scale(&Pnt::origin(), factor),
                false,
                policy,
            )
            .unwrap_or_else(|error| panic!("{label}: {error}"));
            assert!(scaled.validation.is_valid(), "{label}: operation report");
            assert!(
                scaled.recovery.actions.iter().all(|action| !matches!(
                    action,
                    RecoveryAction::ReconstructPcurve { .. }
                        | RecoveryAction::ReconstructPcurves { .. }
                )),
                "{label}: native scale unexpectedly rebuilt pcurves: {:?}",
                scaled.recovery.actions
            );
            assert_strict_solids(
                &label,
                core::slice::from_ref(&scaled.value),
                &scaled.history,
            );
            let mesh = openrcad_mesh::tessellate_checked_with_policy(
                &scaled.value,
                (0.04 * factor).max(policy.approximation * 4.0),
                0.25,
                policy,
            )
            .expect("scaled measurement mesh");
            let properties = openrcad_mesh::mass_properties(&mesh).expect("scaled mass properties");
            let area_error =
                (properties.surface_area - source_properties.surface_area * factor.powi(2)).abs()
                    / properties.surface_area.max(policy.resolution);
            let volume_error = (properties.volume - source_properties.volume * factor.powi(3))
                .abs()
                / properties.volume.max(policy.resolution);
            // Curved faces use chordal mass properties. Scaling the chord with
            // the body keeps the same sampling regime, with small cap/side
            // triangulation round-off at extreme factors.
            assert!(area_error < 5.0e-3, "{label}: area error {area_error}");
            assert!(
                volume_error < 5.0e-3,
                "{label}: volume error {volume_error}"
            );
        }
    }
}

#[test]
fn transform_orientation_flag_must_match_the_matrix() {
    let policy = &TolerancePolicy::STANDARD;
    let source =
        make_box_operation_with_policy(&Pnt::origin(), 3.0, 4.0, 5.0, policy).expect("box");
    let mirror = Trsf::mirror_plane(&Ax2::new(Pnt::origin(), Dir::dx()));

    let mismatch = transform_operation_with_policy(&source.value, &mirror, false, policy)
        .expect_err("reflection cannot be disguised as an orientation-preserving transform");
    assert!(mismatch.to_string().contains("linear determinant"));

    let mirrored = transform_operation_with_policy(&source.value, &mirror, true, policy)
        .expect("determinant-matched reflection");
    assert_strict_solids(
        "reflected transform",
        core::slice::from_ref(&mirrored.value),
        &mirrored.history,
    );
}
