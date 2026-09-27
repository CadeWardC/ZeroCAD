use openrcad::foundation::Pnt;
use zerocad_core::*;

fn document(bytes: &[u8]) -> ParametricGraph {
    read_document_from_slice(bytes, &Default::default())
        .unwrap()
        .document
        .into_evaluator_graph()
        .clone_document()
}

fn check(graph: &ParametricGraph) {
    // The outward cap and old body have disjoint interiors. Their separately
    // evaluated volumes and membership provide an independent union reference.
    let mut separate = graph.clone_document();
    for node in separate.graph.node_weights_mut() {
        if node.id == "extrude_45" {
            if let FeatureType::Extrude { mode, .. } = &mut node.feature {
                *mode = ExtrudeMode::NewBody;
            }
        }
    }
    separate.commit_feature_edit("extrude_45").unwrap();
    let (inputs, warnings) = separate
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "reference: {warnings:?}");
    let expected: f64 = inputs
        .iter()
        .map(|(_, mesh)| mesh.mass_properties().unwrap().volume)
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
        assert!((volume - expected).abs() < 0.05, "{volume} != {expected}");
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
        for x in [-21., -15., -5.2, -5.1, 0., 5.1, 5.2, 9., 11.] {
            for y in [-1., 1., 7.3, 12.5, 17.7, 24.9, 25.01, 27., 29., 32.7, 33.] {
                for z in [-30., -28., -23., -20., 0., 19.3, 19.6, 23.3, 23.6, 28., 30.] {
                    let point = Pnt::new(x, y, z);
                    let expected = reference
                        .iter()
                        .flat_map(|(_, parts)| parts)
                        .any(|part| openrcad::algo::boolean::point_in_solid(&point, part));
                    assert_eq!(
                        openrcad::algo::boolean::point_in_solid(&point, solid),
                        expected,
                        "material at {point:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn saved_join_with_near_tangent_cuts_is_valid_and_reopens() {
    for depth in [3.0, 7.776] {
        let mut graph = document(include_bytes!("fixtures/join-failed.zcad"));
        for node in graph.graph.node_weights_mut() {
            if node.id == "extrude_45" {
                if let FeatureType::Extrude { depth: value, .. } = &mut node.feature {
                    *value = depth;
                }
            }
        }
        graph.commit_feature_edit("extrude_45").unwrap();
        check(&graph);
        let bytes = write_document_to_vec(
            &Document::from_graph(graph, Unit::Millimeter),
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        check(&document(&bytes));
    }
}
