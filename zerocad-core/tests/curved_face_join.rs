use openrcad::foundation::Pnt;
use zerocad_core::*;

fn document(depth: f32) -> ParametricGraph {
    // Autosave immediately before the failed action. The rejected feature was
    // never committed by the GUI, so recreate its two-region selection here.
    let mut graph = read_document_from_slice(
        include_bytes!("fixtures/curved-face-join.zcad"),
        &Default::default(),
    )
    .unwrap()
    .document
    .into_evaluator_graph();
    graph.add_feature(FeatureNode {
        id: "extrude_10".into(),
        name: "Join".into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![0, 1],
            mode: ExtrudeMode::Join,
            target: Some("extrude_4".into()),
            depth_expr: None,
            draft_angle_deg: 0.,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_9", "extrude_10");
    graph
}

fn check(graph: &ParametricGraph) {
    let mut separate = graph.clone();
    for node in separate.graph.node_weights_mut() {
        if node.id == "extrude_10" {
            if let FeatureType::Extrude { mode, .. } = &mut node.feature {
                *mode = ExtrudeMode::NewBody;
            }
        }
    }
    separate.commit_feature_edit("extrude_10").unwrap();
    let (inputs, warnings) = separate
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "reference: {warnings:?}");
    assert_eq!(inputs.len(), 2);
    // The new prism starts on the opposite cap and sweeps away from the body.
    let expected: f64 = inputs
        .iter()
        .map(|(_, m)| m.mass_properties().unwrap().volume)
        .sum();
    let reference = separate
        .evaluated_kernel_bodies(&Default::default())
        .unwrap();
    for _ in 0..2 {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(bodies.len(), 1);
        let volume = bodies[0].1.mass_properties().unwrap().volume;
        assert!((volume - expected).abs() < 0.02, "{volume} != {expected}");
        let solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
        assert_eq!(solids.len(), 1);
        assert_eq!(solids[0].1.len(), 1);
        let solid = &solids[0].1[0];
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
        solid
            .validate_strict_with_policy(&Default::default())
            .unwrap();
        assert_eq!(solid.split_disconnected().len(), 1);
        for x in [8., 10., 30., 59., 90., 108., 110.] {
            for y in [-3., 5., 20., 25., 33.] {
                for z in [-29., -25., -21., -18., -11., -9.] {
                    let p = Pnt::new(x, y, z);
                    let inside = reference
                        .iter()
                        .flat_map(|(_, parts)| parts)
                        .any(|part| openrcad::algo::boolean::point_in_solid(&p, part));
                    assert_eq!(
                        openrcad::algo::boolean::point_in_solid(&p, solid),
                        inside,
                        "material at {p:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn successive_curved_face_join_preserves_material_and_reopens() {
    let graph = document(10.);
    check(&graph);
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &Default::default(),
        &Default::default(),
    )
    .unwrap();
    let reopened = read_document_from_slice(&bytes, &Default::default()).unwrap();
    check(reopened.document.evaluator_graph());
}

#[test]
fn successive_curved_face_join_supports_depth_edits() {
    check(&document(4.));
}

#[test]
fn successive_join_depth_edit_rebuilds_missing_hydrated_sources() {
    let mut graph = document(10.);
    graph.evaluate_bodies(&Default::default()).unwrap();
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(&graph.evaluation_cache_snapshot(), &mut bytes).unwrap();
    graph.install_evaluation_cache(ciborium::de::from_reader(bytes.as_slice()).unwrap());
    for node in graph.graph.node_weights_mut() {
        if node.id == "extrude_10" {
            if let FeatureType::Extrude { depth, .. } = &mut node.feature {
                *depth = 6.;
            }
        }
    }
    graph.commit_feature_edit("extrude_10").unwrap();
    check(&graph);
}
