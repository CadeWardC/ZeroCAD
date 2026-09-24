use zerocad_core::boolean_case::v2::{asset_sha256, BooleanCaseV2, GeometryObservation, Operand};
use zerocad_core::*;

fn drill(center: f64, height: f64) -> Operand {
    Operand::Primitive {
        operand: BooleanOperandV1 {
            primitive: AnalyticPrimitiveV1::Cylinder {
                radius: 2.,
                height: height + 2.,
            },
            transform: RigidTransformV1 {
                translation: [center, center, -1.],
                ..Default::default()
            },
        },
    }
}

fn drilled_cube_properties(side: f64, height: f64) -> GeometryObservation {
    let pi = std::f64::consts::PI;
    GeometryObservation {
        body_count: 1,
        volume: side * side * height - 4. * pi * height,
        surface_area: 2. * side * side + 4. * side * height - 8. * pi + 4. * pi * height,
        centroid: [side / 2., side / 2., height / 2.],
    }
}

#[test]
fn imported_operand_reads_the_foreign_asset_and_verifies_material_removal() {
    let data = include_str!("fixtures/step/mayo-freecad-cube.step").to_owned();
    let case = BooleanCaseV2 {
        operation: BooleanVerificationOp::Cut,
        object: Operand::Step {
            sha256: asset_sha256(data.as_bytes()),
            data,
        },
        tool: drill(5., 10.),
    };
    let expected = drilled_cube_properties(10., 10.);
    let observed = case.replay().unwrap();
    observed.compare(&expected, 0.003, 0.01).unwrap();
    let mut incorrect = expected.clone();
    incorrect.volume = 1000.; // A healthy unchanged cube must fail the oracle.
    assert!(observed.compare(&incorrect, 0.003, 0.01).is_err());
    let id = case.identity().unwrap();
    let mut tampered = case.clone();
    if let Operand::Step { data, .. } = &mut tampered.object {
        data.push(' ');
    }
    assert!(tampered.identity().is_err());
    assert!(tampered.replay().is_err());
    let decoded: BooleanCaseV2 =
        serde_json::from_str(&serde_json::to_string_pretty(&case).unwrap()).unwrap();
    assert_eq!(decoded.identity().unwrap(), id);
}

#[test]
fn history_operand_rebuilds_a_saved_sketch_extrusion() {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0., 0.), (12., 12.));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Profile".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Plate".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 8.,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");
    let data = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    let case = BooleanCaseV2 {
        operation: BooleanVerificationOp::Cut,
        object: Operand::Document {
            sha256: asset_sha256(&data),
            data,
            body_id: "extrude_2".into(),
        },
        tool: drill(6., 8.),
    };
    case.replay()
        .unwrap()
        .compare(&drilled_cube_properties(12., 8.), 0.003, 0.01)
        .unwrap();
    let mut missing = case.clone();
    if let Operand::Document { body_id, .. } = &mut missing.object {
        *body_id = "missing".into();
    }
    assert_ne!(missing.identity().unwrap(), case.identity().unwrap());
    assert!(missing.replay().is_err());
}

#[test]
fn all_signed_byte_scale_values_are_validated_without_panicking() {
    let mut case = BooleanCaseV1::from_fuzz_bytes(&[]);
    for scale in i8::MIN..=i8::MAX {
        case.logarithmic_scale = scale;
        assert_eq!(case.validate().is_ok(), (-9..=9).contains(&scale));
    }
}
