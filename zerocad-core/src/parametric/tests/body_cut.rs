use super::*;

fn body_cut_graph(offset: f32, keep_tool: bool) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Target".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "copy_2".to_string(),
        name: "Cutting body".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [offset, 0.0, 0.0],
            copy: true,
        },
    });
    graph.add_dependency("box_1", "copy_2");
    graph.add_feature(FeatureNode {
        id: "cut_bodies_3".to_string(),
        name: "Cut".to_string(),
        feature: FeatureType::BodyCut {
            target: "box_1".to_string(),
            tool: "copy_2".to_string(),
            keep_tool,
        },
    });
    graph.add_dependency("box_1", "cut_bodies_3");
    graph.add_dependency("copy_2", "cut_bodies_3");
    graph
}

#[test]
fn body_cut_consumes_tool_and_keeps_target_remainder() {
    let graph = body_cut_graph(5.0, false);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "cut_bodies_3");
    let volume = bodies[0].1.mass_properties().unwrap().volume;
    assert!((volume - 500.0).abs() < 1.0, "volume={volume}");
}

#[test]
fn body_cut_can_keep_the_cutting_body() {
    let graph = body_cut_graph(5.0, true);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    assert!(bodies.iter().any(|(id, _)| id == "cut_bodies_3"));
    assert!(bodies.iter().any(|(id, _)| id == "copy_2"));
}

#[test]
fn body_cut_requires_positive_overlap_and_keeps_inputs_on_failure() {
    for offset in [10.0, 20.0] {
        let graph = body_cut_graph(offset, false);
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 2, "offset={offset}");
        assert!(bodies.iter().any(|(id, _)| id == "box_1"));
        assert!(bodies.iter().any(|(id, _)| id == "copy_2"));
        assert!(warnings
            .iter()
            .any(|warning| warning.contains("must overlap")));
    }
}
