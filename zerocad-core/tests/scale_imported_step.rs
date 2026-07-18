use openrcad::algo::{boolean_operation_with_policy, transform_operation_with_policy, BooleanOp};
use openrcad::exchange::{read_step_operation_with_policy, write_step};
use openrcad::foundation::{Ax2, Dir, Pnt, TolerancePolicy, Trsf};
use openrcad::primitives::{make_box_operation_with_policy, make_cylinder_operation_with_policy};

#[test]
fn imported_step_scales_strictly_across_the_phase35_factor_matrix() {
    let policy = &TolerancePolicy::STANDARD;
    let native =
        make_cylinder_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 8.0, policy)
            .expect("native cylinder");
    let path = std::env::temp_dir().join(format!(
        "zerocad-phase35-scale-import-{}.stp",
        std::process::id()
    ));
    write_step(&native.value, path.to_str().unwrap()).expect("strict STEP write");
    let imported = read_step_operation_with_policy(path.to_str().unwrap(), policy)
        .expect("strict STEP round-trip import");
    let _ = std::fs::remove_file(path);
    let base_mesh =
        openrcad::mesh::tessellate_checked_with_policy(&imported.value, 0.15, 0.3, policy)
            .expect("import measurement mesh");
    let base = openrcad::mesh::mass_properties(&base_mesh).expect("import mass properties");

    for factor in [0.1_f64, 0.5, 2.0, 100.0] {
        let result = transform_operation_with_policy(
            &imported.value,
            &Trsf::scale(&Pnt::origin(), factor),
            false,
            policy,
        )
        .unwrap_or_else(|error| panic!("scale imported STEP by {factor}: {error}"));
        assert!(result.validation.is_valid());
        result.value.validate_strict_with_policy(policy).unwrap();
        assert!(result.value.is_watertight_with_policy(policy));
        assert!(result.value.has_complete_pcurves());
        assert!(result
            .history
            .coverage_for_solid(&result.value)
            .is_complete());

        let mesh = openrcad::mesh::tessellate_checked_with_policy(
            &result.value,
            (0.15 * factor).max(policy.approximation * 4.0),
            0.3,
            policy,
        )
        .expect("scaled import measurement mesh");
        let properties =
            openrcad::mesh::mass_properties(&mesh).expect("scaled import mass properties");
        let area_error = (properties.surface_area - base.surface_area * factor.powi(2)).abs()
            / properties.surface_area.max(policy.resolution);
        let volume_error = (properties.volume - base.volume * factor.powi(3)).abs()
            / properties.volume.max(policy.resolution);
        assert!(
            area_error < 1.0e-2,
            "factor {factor}: area error {area_error}"
        );
        assert!(
            volume_error < 1.0e-2,
            "factor {factor}: volume error {volume_error}"
        );
    }
}

#[test]
fn native_body_intersects_a_strictly_imported_step_body() {
    let policy = &TolerancePolicy::STANDARD;
    let native_tool = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()),
        2.0,
        12.0,
        policy,
    )
    .expect("native STEP source");
    let path = std::env::temp_dir().join(format!(
        "zerocad-phase35-common-import-{}.stp",
        std::process::id()
    ));
    write_step(&native_tool.value, path.to_str().unwrap()).expect("strict STEP write");
    let imported = read_step_operation_with_policy(path.to_str().unwrap(), policy)
        .expect("strict STEP import");
    let _ = std::fs::remove_file(path);
    let native_target = make_box_operation_with_policy(&Pnt::origin(), 10.0, 10.0, 10.0, policy)
        .expect("native target");

    let common = boolean_operation_with_policy(
        &native_target.value,
        &imported.value,
        BooleanOp::Common,
        policy,
    )
    .expect("native/imported Common");
    assert!(common.validation.is_valid());
    assert!(common.value.is_watertight_with_policy(policy));
    assert!(common.value.has_complete_pcurves());
    common.value.validate_strict_with_policy(policy).unwrap();
    assert!(common
        .history
        .coverage_for_solid(&common.value)
        .is_complete());
    openrcad::mesh::tessellate_checked_with_policy(&common.value, 0.04, 0.25, policy)
        .expect("checked native/imported Common mesh");
}
