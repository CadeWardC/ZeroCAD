use openrcad::foundation::Pnt;
use zerocad_core::*;

fn document() -> ParametricGraph {
    read_document_from_slice(
        include_bytes!("fixtures/fit-gpu.zcad"),
        &LoadOptions::default(),
    )
    .unwrap()
    .document
    .into_evaluator_graph()
}

fn check(graph: &ParametricGraph) {
    let cs = graph
        .graph
        .node_weights()
        .find_map(|node| match node.feature {
            FeatureType::Sketch { cs, .. } => Some(cs),
            _ => None,
        })
        .unwrap();
    // The selected arrangement regions have disjoint interiors. Build them
    // independently to check both total volume and the union's material/void.
    let mut separate = graph.clone();
    for node in separate.graph.node_weights_mut() {
        if node.id == "extrude_7" {
            if let FeatureType::Extrude { mode, .. } = &mut node.feature {
                *mode = ExtrudeMode::NewBody;
            }
        }
    }
    separate.commit_feature_edit("extrude_7").unwrap();
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
        for x in [-9., -6., -2., 8., 19., 26., 35., 39.] {
            for y in [-21., -17., -13., 0., 12., 25., 65., 80., 84., 89.] {
                for z in [-6., -3., -1., 1., 3., 6., 9.] {
                    let world = cs.origin.add(cs.u.mul(x)).add(cs.v.mul(y)).add(cs.n.mul(z));
                    let point = Pnt::new(world.x as f64, world.y as f64, world.z as f64);
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
fn fit_gpu_join_is_valid_and_reopens() {
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
fn merged_polygon_join_preserves_depth_edits_direction_and_frame() {
    for sign in [1., -1.] {
        let mut graph = document();
        let rotate = |v: Vec3| Vec3::new(v.z, v.x, v.y);
        for node in graph.graph.node_weights_mut() {
            match &mut node.feature {
                FeatureType::Extrude { depth, .. } => {
                    *depth = sign * if node.id == "extrude_7" { 7.5 } else { 2.5 };
                }
                FeatureType::Sketch { cs, .. } => {
                    cs.origin = rotate(cs.origin).add(Vec3::new(17., -31., 43.));
                    cs.u = rotate(cs.u);
                    cs.v = rotate(cs.v);
                    cs.n = rotate(cs.n);
                }
                _ => {}
            }
        }
        check(&graph);
    }
}
