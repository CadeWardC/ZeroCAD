use openrcad::foundation::Pnt;
use zerocad_core::*;

fn document() -> ParametricGraph {
    read_document_from_slice(
        include_bytes!("fixtures/overhanging-face-join.zcad"),
        &Default::default(),
    )
    .unwrap()
    .document
    .into_evaluator_graph()
}

fn check(graph: &ParametricGraph) {
    let cs = graph
        .graph
        .node_weights()
        .find_map(|node| match &node.feature {
            FeatureType::Sketch { cs, .. } if node.id == "sketch_1" => Some(*cs),
            _ => None,
        })
        .unwrap();
    // Build the same selected regions separately as a material oracle. These
    // prisms meet at the support plane, so their volumes must add exactly.
    let mut separate = graph.clone();
    for node in separate.graph.node_weights_mut() {
        if node.id == "extrude_5" {
            if let FeatureType::Extrude { mode, .. } = &mut node.feature {
                *mode = ExtrudeMode::NewBody;
            }
        }
    }
    separate.commit_feature_edit("extrude_5").unwrap();
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
        assert!((volume - expected).abs() < 0.01, "{volume} != {expected}");
        let solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
        assert_eq!(solids.len(), 1);
        assert_eq!(solids[0].1.len(), 1);
        let solid = &solids[0].1[0];
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
        assert!(solid
            .validate_strict_with_policy(&Default::default())
            .is_ok());
        assert_eq!(solid.split_disconnected().len(), 1);
        // Check both retained material and void on either side of the join,
        // including the overhang and the concave/polyline footprint.
        for x in [-48., -32., 0., 31., 48., 55.] {
            for y in [-30., -23., -10., 10., 23., 30.] {
                for z in [-11., -7., -2., 2., 5.] {
                    let world = cs.origin.add(cs.u.mul(x)).add(cs.n.mul(y)).add(cs.v.mul(z));
                    let point = Pnt::new(world.x as f64, world.y as f64, world.z as f64);
                    let inside = reference
                        .iter()
                        .flat_map(|(_, parts)| parts)
                        .any(|part| openrcad::algo::boolean::point_in_solid(&point, part));
                    assert_eq!(
                        openrcad::algo::boolean::point_in_solid(&point, solid),
                        inside,
                        "material at {point:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn saved_overhanging_multiregion_join_is_exact_and_reopens() {
    let graph = document();
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
fn overhanging_join_preserves_depth_edits_and_negative_sweeps() {
    for sign in [1., -1.] {
        let mut graph = document();
        for node in graph.graph.node_weights_mut() {
            match &mut node.feature {
                FeatureType::Extrude { depth, .. } => {
                    *depth = sign * if node.id == "extrude_5" { 12. } else { 21.5 };
                }
                FeatureType::Sketch { cs, .. } if node.id == "sketch_3" => {
                    cs.origin.y *= sign;
                }
                _ => {}
            }
        }
        check(&graph);
    }
}

#[test]
fn overhanging_join_is_independent_of_world_frame() {
    let mut graph = document();
    let rotate = |v: Vec3| Vec3::new(v.z, v.x, v.y);
    for node in graph.graph.node_weights_mut() {
        if let FeatureType::Sketch { cs, .. } = &mut node.feature {
            cs.origin = rotate(cs.origin).add(Vec3::new(123., -47., 9.));
            cs.u = rotate(cs.u);
            cs.v = rotate(cs.v);
            cs.n = rotate(cs.n);
        }
    }
    check(&graph);
}
