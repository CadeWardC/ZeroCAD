use std::collections::HashSet;
use zerocad_core::{ExtrudeMode, FeatureNode, FeatureType, ParametricGraph};

fn model(mode: ExtrudeMode) -> ParametricGraph {
    let loaded = zerocad_core::read_document_from_slice(
        include_bytes!("fixtures/broken-extrusion.zcad"),
        &Default::default(),
    )
    .unwrap();
    let mut graph = loaded.document.evaluator_graph().clone();
    // Model the GUI's warm-preview path: the saved prefix has already evaluated.
    graph.evaluate_bodies(&HashSet::new()).unwrap();
    graph.add_feature(FeatureNode {
        id: "extrude_11".into(),
        name: "Selected strip".into(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![0],
            mode,
            target: (mode == ExtrudeMode::Join).then(|| "extrude_5".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_9", "extrude_11");
    graph
}

#[test]
fn reused_face_sketch_keeps_selected_strip_after_its_first_join() {
    let mut graph = model(ExtrudeMode::NewBody);
    for pass in 0..3 {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&HashSet::new())
            .unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        let mesh = &bodies.iter().find(|(id, _)| id == "extrude_11").unwrap().1;
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for v in mesh.vertices.chunks_exact(6) {
            for axis in 0..3 {
                min[axis] = min[axis].min(v[axis]);
                max[axis] = max[axis].max(v[axis]);
            }
        }
        for (actual, expected) in min
            .into_iter()
            .zip([-8.652453, -50.0, 4.0999985])
            .chain(max.into_iter().zip([-6.4000006, -20.0, 63.1]))
        {
            assert!(
                (actual - expected).abs() < 0.001,
                "wrong strip bounds: {min:?} .. {max:?}"
            );
        }
        let expected_volume = (8.652453_f64 - 6.4000006) * 59.0 * 30.0;
        assert!((mesh.mass_properties().unwrap().volume - expected_volume).abs() < 0.05);
        graph.apply_face_reattach();
        if pass == 1 {
            let document =
                zerocad_core::Document::from_graph(graph, zerocad_core::Unit::Millimeter);
            let bytes = zerocad_core::zcad_format::write_document_to_vec(
                &document,
                &Default::default(),
                &Default::default(),
            )
            .unwrap();
            graph = zerocad_core::read_document_from_slice(&bytes, &Default::default())
                .unwrap()
                .document
                .into_evaluator_graph();
        }
    }
}

#[test]
fn reused_face_sketch_join_does_not_add_shifted_rectangle() {
    let graph = model(ExtrudeMode::Join);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    assert!(
        !warnings.is_empty(),
        "an edge-only contact must not be accepted"
    );
    let mut baseline = graph.clone();
    baseline.remove_feature("extrude_11");
    let original = baseline.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), original.len());
    for ((id, mesh), (old_id, old_mesh)) in bodies.iter().zip(&original) {
        assert_eq!(id, old_id);
        assert_eq!(
            mesh.vertices, old_mesh.vertices,
            "failed Join changed the body"
        );
        assert_eq!(mesh.indices, old_mesh.indices);
    }
    let mesh = &bodies.iter().find(|(id, _)| id == "extrude_5").unwrap().1;
    assert!(
        mesh.vertices.chunks_exact(6).all(|v| v[0] >= -11.653),
        "join added the shifted slab"
    );
}

#[test]
fn adjoining_strips_join_successfully_in_the_original_sketch_frame() {
    let mut graph = model(ExtrudeMode::Join);
    for node in graph.graph.node_weights_mut() {
        if node.id == "extrude_11" {
            if let FeatureType::Extrude { region_indices, .. } = &mut node.feature {
                *region_indices = vec![0, 2];
            }
        }
    }
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    let mesh = &bodies.iter().find(|(id, _)| id == "extrude_5").unwrap().1;
    assert!(mesh.vertices.chunks_exact(6).all(|v| v[0] >= -11.653));
    let volume = mesh.mass_properties().unwrap().volume;
    let mut baseline = graph.clone();
    baseline.remove_feature("extrude_11");
    let original = baseline.evaluate_bodies(&HashSet::new()).unwrap();
    let original_volume = original
        .iter()
        .find(|(id, _)| id == "extrude_5")
        .unwrap()
        .1
        .mass_properties()
        .unwrap()
        .volume;
    let expected_added = (8.652453_f64 - 3.2000003) * 59.0 * 30.0;
    assert!((volume - original_volume - expected_added).abs() < 0.1);
    let solids = graph.evaluated_kernel_bodies(&HashSet::new()).unwrap();
    for (_, parts) in solids {
        for part in parts {
            assert!(part
                .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
                .is_ok());
        }
    }
}

#[test]
fn reused_sketch_still_follows_an_upstream_depth_edit() {
    let mut graph = model(ExtrudeMode::NewBody);
    for node in graph.graph.node_weights_mut() {
        if node.id == "extrude_8" {
            if let FeatureType::Extrude { depth, .. } = &mut node.feature {
                *depth = 25.0;
            }
        }
    }
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    let mesh = &bodies.iter().find(|(id, _)| id == "extrude_11").unwrap().1;
    let min_y = mesh
        .vertices
        .chunks_exact(6)
        .map(|v| v[1])
        .fold(f32::INFINITY, f32::min);
    let max_y = mesh
        .vertices
        .chunks_exact(6)
        .map(|v| v[1])
        .fold(f32::NEG_INFINITY, f32::max);
    assert!((min_y + 55.0).abs() < 0.001 && (max_y + 25.0).abs() < 0.001);
    assert!(mesh
        .vertices
        .chunks_exact(6)
        .all(|v| v[0] >= -8.653 && v[0] <= -6.399));
}
