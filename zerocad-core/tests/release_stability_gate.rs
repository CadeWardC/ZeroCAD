use openrcad::algo::{boolean_operation_with_policy, transform_operation_with_policy, BooleanOp};
use openrcad::exchange::read_step_str_operation_with_policy;
use openrcad::foundation::{Ax2, Dir, Pnt, TolerancePolicy, Trsf};
use openrcad::primitives::{make_box_operation_with_policy, make_cylinder_operation_with_policy};
use openrcad::topo::Solid;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use zerocad_core::{
    read_document_from_slice, validate_phase7_exception_ledger, write_document_to_vec, Document,
    FeatureNode, FeatureType, HydrationBundle, LoadOptions, SaveOptions,
    PHASE7_COMPATIBILITY_EXCEPTIONS,
};

#[derive(Debug, Deserialize)]
struct Oracle {
    schema: u32,
    generator: String,
    occt_version: String,
    step_fixture_sha256: String,
    cases: Vec<OracleCase>,
}

#[derive(Debug, Deserialize)]
struct OracleCase {
    id: String,
    category: String,
    classification: String,
    volume: f64,
    surface_area: f64,
    centroid: [f64; 3],
}

fn assert_strict(solid: &Solid) {
    let policy = &TolerancePolicy::STANDARD;
    assert!(solid.is_watertight_with_policy(policy));
    assert!(solid.has_complete_pcurves());
    solid.validate_strict_with_policy(policy).unwrap();
}

fn measured(solid: &Solid) -> openrcad::mesh::MassProperties {
    assert_strict(solid);
    let mesh = openrcad::mesh::tessellate_checked_with_policy(
        solid,
        0.02,
        0.15,
        &TolerancePolicy::STANDARD,
    )
    .expect("strict differential tessellation");
    openrcad::mesh::mass_properties(&mesh).expect("closed differential solid")
}

fn relative_error(actual: f64, expected: f64) -> f64 {
    (actual - expected).abs() / expected.abs().max(1.0e-9)
}

fn assert_oracle(case: &OracleCase, solid: &Solid) {
    assert_eq!(case.classification, "solid", "{} classification", case.id);
    let actual = measured(solid);
    assert!(
        relative_error(actual.volume, case.volume) < 0.01,
        "{} volume: actual {}, OCCT {}",
        case.id,
        actual.volume,
        case.volume
    );
    assert!(
        relative_error(actual.surface_area, case.surface_area) < 0.01,
        "{} area: actual {}, OCCT {}",
        case.id,
        actual.surface_area,
        case.surface_area
    );
    for axis in 0..3 {
        assert!(
            (actual.centroid[axis] - case.centroid[axis]).abs() < 0.03,
            "{} centroid axis {axis}: actual {}, OCCT {}",
            case.id,
            actual.centroid[axis],
            case.centroid[axis]
        );
    }
}

#[test]
fn curated_occt_differential_oracle() {
    let oracle: Oracle = serde_json::from_str(include_str!("fixtures/occt/phase7-oracle-v1.json"))
        .expect("frozen OCCT oracle JSON");
    assert_eq!(oracle.schema, 1);
    assert_eq!(oracle.generator, "tools/phase7_occt_oracle.py");
    assert_eq!(oracle.occt_version, "7.8.1.1");
    assert_eq!(
        oracle.step_fixture_sha256,
        "e927cffb0dddb9415f541d57132ae5f3f8013a7c4d8baf01669e052030dfdf0a"
    );
    let cases: HashMap<_, _> = oracle
        .cases
        .iter()
        .map(|case| (case.id.as_str(), case))
        .collect();
    assert_eq!(cases.len(), 9);
    assert_eq!(
        oracle
            .cases
            .iter()
            .map(|case| case.category.as_str())
            .collect::<HashSet<_>>(),
        HashSet::from([
            "primitive",
            "boolean",
            "import",
            "direct_edit",
            "failure_classification",
        ])
    );
    let policy = &TolerancePolicy::STANDARD;

    let primitive_box =
        make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy).unwrap();
    assert_oracle(cases["primitive_box"], &primitive_box.value);
    let primitive_cylinder =
        make_cylinder_operation_with_policy(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 8.0, policy)
            .unwrap();
    assert_oracle(cases["primitive_cylinder"], &primitive_cylinder.value);

    let fuse_a = make_box_operation_with_policy(&Pnt::origin(), 10.0, 10.0, 10.0, policy).unwrap();
    let fuse_b =
        make_box_operation_with_policy(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0, policy).unwrap();
    let fused =
        boolean_operation_with_policy(&fuse_a.value, &fuse_b.value, BooleanOp::Fuse, policy)
            .unwrap();
    assert_oracle(cases["boolean_fuse_boxes"], &fused.value);
    let common =
        boolean_operation_with_policy(&fuse_a.value, &fuse_b.value, BooleanOp::Common, policy)
            .unwrap();
    assert_oracle(cases["boolean_common_boxes"], &common.value);

    let hole_box = make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy).unwrap();
    let hole_tool = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(5.0, 4.0, -1.0), Dir::dz()),
        2.0,
        8.0,
        policy,
    )
    .unwrap();
    let cut =
        boolean_operation_with_policy(&hole_box.value, &hole_tool.value, BooleanOp::Cut, policy)
            .unwrap();
    assert_oracle(cases["boolean_cut_through_hole"], &cut.value);

    let imported = read_step_str_operation_with_policy(
        include_str!("fixtures/step/mayo-freecad-cube.step"),
        policy,
    )
    .expect("strict Mayo cube import");
    assert_oracle(cases["import_mayo_cube"], &imported.value);

    let scaled = transform_operation_with_policy(
        &primitive_box.value,
        &Trsf::scale(&Pnt::origin(), 2.0),
        false,
        policy,
    )
    .unwrap();
    assert_oracle(cases["direct_uniform_scale"], &scaled.value);

    for (id, x) in [
        ("failure_disjoint_common", 20.0),
        ("failure_tangent_common", 10.0),
    ] {
        assert_eq!(cases[id].classification, "empty_or_contact");
        let tool =
            make_box_operation_with_policy(&Pnt::new(x, 0.0, 0.0), 2.0, 2.0, 2.0, policy).unwrap();
        let result =
            boolean_operation_with_policy(&fuse_a.value, &tool.value, BooleanOp::Common, policy);
        assert!(
            result.is_err(),
            "{id}: contact-only/disjoint Common must be a structured failure"
        );
    }
}

#[test]
fn seeded_scale_boolean_and_persistence_stress_smoke() {
    let policy = &TolerancePolicy::STANDARD;
    let iterations = std::env::var("ZEROCAD_LONG_STRESS_ITERATIONS")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(12)
        .clamp(12, 1_000);
    let mut state = 0x7A17_CAD5_D15C_A11Eu64;
    for index in 0..iterations {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let unit = ((state >> 16) & 0xffff) as f64 / 65535.0;
        let scale = 10_f64.powf(-1.0 + 2.0 * unit);
        let dx = (4.0 + index as f64 % 5.0) * scale;
        let dy = (5.0 + index as f64 % 3.0) * scale;
        let dz = (6.0 + index as f64 % 4.0) * scale;
        let body = make_box_operation_with_policy(&Pnt::origin(), dx, dy, dz, policy)
            .unwrap_or_else(|error| panic!("seeded case {index}: box: {error}"));
        assert_strict(&body.value);
        let factor = [0.1, 0.5, 2.0, 10.0][index % 4];
        let scaled = transform_operation_with_policy(
            &body.value,
            &Trsf::scale(&Pnt::origin(), factor),
            false,
            policy,
        )
        .unwrap_or_else(|error| panic!("seeded case {index}: scale: {error}"));
        assert_strict(&scaled.value);

        if index < 4 {
            let tool =
                make_box_operation_with_policy(&Pnt::new(dx * 0.5, 0.0, 0.0), dx, dy, dz, policy)
                    .unwrap();
            let common =
                boolean_operation_with_policy(&body.value, &tool.value, BooleanOp::Common, policy)
                    .unwrap_or_else(|error| panic!("seeded case {index}: common: {error}"));
            assert_strict(&common.value);
        }
    }

    let mut document = Document::new();
    for index in 0..40 {
        document.add_feature(FeatureNode {
            id: format!("stress_box_{index}"),
            name: format!("Stress Box {index}"),
            feature: FeatureType::Box {
                w: 1.0 + index as f32 * 0.01,
                h: 2.0,
                d: 3.0,
            },
        });
    }
    for round in 0..6 {
        let id = format!("stress_box_{}", (round * 7) % 40);
        let suppressed = round % 2 == 0;
        document.set_feature_suppressed(&id, suppressed);
        assert_eq!(document.is_feature_suppressed(&id), suppressed);
        let mut sequence: Vec<_> = document
            .graph
            .node_weights()
            .filter(|node| !matches!(node.feature, FeatureType::Origin))
            .map(|node| node.id.clone())
            .collect();
        sequence.reverse();
        for (position, feature_id) in sequence.iter().enumerate() {
            assert!(document
                .set_feature_sequence(feature_id, zerocad_core::SequenceKey(position as u64 + 1),));
        }
        let bytes = write_document_to_vec(
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("stress save");
        let loaded =
            read_document_from_slice(&bytes, &LoadOptions::default()).expect("stress load");
        document = loaded.document;
        document
            .validate_semantic_contracts()
            .expect("stress semantics");
    }
}

#[test]
fn retained_exceptions_are_owned_and_triggered() {
    validate_phase7_exception_ledger().unwrap();
    assert_eq!(PHASE7_COMPATIBILITY_EXCEPTIONS.len(), 5);
    assert!(PHASE7_COMPATIBILITY_EXCEPTIONS.iter().all(|exception| {
        exception.id.starts_with("P7-E")
            && (exception.removal_trigger.contains("pass")
                || exception.removal_trigger.contains("ship"))
            && !exception.user_impact.contains("silent")
    }));
}
