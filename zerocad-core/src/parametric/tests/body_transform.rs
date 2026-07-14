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
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(bodies.is_empty());
    assert!(warnings.iter().any(|warning| warning.contains("gone")));
}
