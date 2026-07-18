use super::*;

fn bounds(mesh: &MockMesh) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    (min, max)
}

fn graph_with_transform(copy: bool) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 2.0,
            h: 3.0,
            d: 4.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "transform_2".to_string(),
        name: "Transform".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [7.0, -2.0, 5.0],
            copy,
        },
    });
    graph.add_dependency("box_1", "transform_2");
    graph
}

#[test]
fn body_copy_leaves_source_and_places_translated_body() {
    let graph = graph_with_transform(true);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    let source = bodies.iter().find(|(id, _)| id == "box_1").unwrap();
    let copied = bodies.iter().find(|(id, _)| id == "transform_2").unwrap();
    let (source_min, _) = bounds(&source.1);
    let (copy_min, _) = bounds(&copied.1);
    for axis in 0..3 {
        assert!((copy_min[axis] - source_min[axis] - [7.0, -2.0, 5.0][axis]).abs() < 1e-3);
    }
}

#[test]
fn body_move_consumes_source_and_keeps_volume() {
    let graph = graph_with_transform(false);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "transform_2");
    let (min, max) = bounds(&bodies[0].1);
    assert!((min[0] - 7.0).abs() < 1e-3);
    assert!((min[1] + 2.0).abs() < 1e-3);
    assert!((min[2] - 5.0).abs() < 1e-3);
    let volume = bodies[0].1.mass_properties().unwrap().volume;
    assert!((volume - 24.0).abs() < 1e-3, "volume={volume}");
    assert!((max[0] - min[0] - 2.0).abs() < 1e-3);
}

#[test]
fn second_disconnected_extrude_body_can_be_transformed_independently() {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (5.0, 5.0));
    curves.add_rectangle((20.0, 0.0), (25.0, 5.0));

    let mut graph = ParametricGraph::new();
    add_sketch(&mut graph, "sketch_1", curves);
    add_extrude(
        &mut graph,
        "extrude_2",
        "sketch_1",
        4.0,
        ExtrudeMode::NewBody,
    );
    graph.add_feature(FeatureNode {
        id: "transform_3".to_string(),
        name: "Move second body".to_string(),
        feature: FeatureType::BodyTransform {
            source: body_output_id("extrude_2", 1),
            translation: [100.0, 0.0, 0.0],
            copy: true,
        },
    });
    graph.add_dependency("extrude_2::body:2", "transform_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 3);
    assert!(bodies.iter().any(|(id, _)| id == "extrude_2"));
    let second = bodies
        .iter()
        .find(|(id, _)| id == "extrude_2::body:2")
        .expect("second extrusion body");
    let moved = bodies
        .iter()
        .find(|(id, _)| id == "transform_3")
        .expect("independent transformed copy");
    let (second_min, _) = bounds(&second.1);
    let (moved_min, _) = bounds(&moved.1);
    assert!((moved_min[0] - second_min[0] - 100.0).abs() < 1e-3);
}

#[test]
fn missing_transform_source_warns_without_panicking() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "move_1".to_string(),
        name: "Move".to_string(),
        feature: FeatureType::BodyTransform {
            source: "gone".to_string(),
            translation: [1.0, 0.0, 0.0],
            copy: false,
        },
    });
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(output.diagnostics.iter().any(|diagnostic| {
        diagnostic.feature_id == "move_1"
            && diagnostic.code.as_str() == DiagnosticCode::REFERENCE_MISSING
    }));
}

#[test]
fn body_transform_candidate_contract_cold_warm_cancel_and_restore() {
    let graph = graph_with_transform(true);
    let hidden = std::collections::HashSet::new();
    let cold = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let warm = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty() && warm.1.is_empty());
    assert_eq!(cold.0.len(), warm.0.len());
    for ((cold_id, cold_mesh), (warm_id, warm_mesh)) in cold.0.iter().zip(&warm.0) {
        assert_eq!(cold_id, warm_id);
        assert_eq!(cold_mesh.indices, warm_mesh.indices);
    }
    let cancelled = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        graph.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled),
        ),
        Err(EvaluationError::Cancelled)
    ));
    let restored: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&graph.clone_document()).unwrap()).unwrap();
    assert!(restored
        .evaluate_bodies_with_warnings(&hidden)
        .unwrap()
        .1
        .is_empty());
}
