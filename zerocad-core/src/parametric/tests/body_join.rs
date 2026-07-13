use super::*;

fn joined_boxes_graph(offset: f32) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "copy_2".to_string(),
        name: "Copy".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [offset, 0.0, 0.0],
            copy: true,
        },
    });
    graph.add_dependency("box_1", "copy_2");
    graph.add_feature(FeatureNode {
        id: "join_bodies_3".to_string(),
        name: "Join".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["box_1".to_string(), "copy_2".to_string()],
        },
    });
    graph.add_dependency("box_1", "join_bodies_3");
    graph.add_dependency("copy_2", "join_bodies_3");
    graph
}

#[test]
fn body_join_consumes_two_overlapping_bodies() {
    let graph = joined_boxes_graph(5.0);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "join_bodies_3");
    let volume = bodies[0].1.mass_properties().unwrap().volume;
    assert!((volume - 1500.0).abs() < 1.0, "volume={volume}");
}

#[test]
fn body_join_fuses_bodies_that_touch_on_a_face() {
    let graph = joined_boxes_graph(10.0);
    let solids = graph
        .debug_kernel_solids(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].0, "join_bodies_3");
    assert_eq!(solids[0].1.len(), 1, "touching boxes must be fused");
}

#[test]
fn disconnected_join_fails_and_keeps_both_original_bodies() {
    let graph = joined_boxes_graph(20.0);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 2);
    assert!(bodies.iter().any(|(id, _)| id == "box_1"));
    assert!(bodies.iter().any(|(id, _)| id == "copy_2"));
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("must touch or overlap")));
}
