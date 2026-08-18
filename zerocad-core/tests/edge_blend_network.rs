use std::collections::HashSet;

use zerocad_core::{
    CornerKind, EdgeCornerMode, EdgeRef, FeatureNode, FeatureType, ParametricGraph,
};

fn bottom_edges() -> Vec<EdgeRef> {
    vec![
        EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [10.0, 0.0, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [0.0, 10.0, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [-1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
    ]
}

fn graph(kind: CornerKind, edges: Vec<EdgeRef>, dist: f32) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "edgeblend_2".into(),
        name: "Edge Blend".into(),
        feature: FeatureType::EdgeBlend {
            target: "box_1".into(),
            edges: zerocad_core::canonicalize_edge_refs(edges),
            dist,
            dist_expr: None,
            kind,
            corner_mode: EdgeCornerMode::Miter,
        },
    });
    graph.add_dependency("box_1", "edgeblend_2");
    graph
}

#[test]
fn two_selected_edges_commit_as_one_atomic_candidate() {
    for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
        let graph = graph(kind, bottom_edges(), 1.0);
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&HashSet::new())
            .expect("edge network evaluation");
        assert!(warnings.is_empty(), "{kind:?}: {warnings:?}");
        assert_eq!(bodies.len(), 1);
        assert!(bodies[0].1.mass_properties().is_some());
    }
}

#[test]
fn rejected_network_leaves_the_original_body_unchanged() {
    let original = {
        let mut plain = ParametricGraph::new();
        plain.add_feature(FeatureNode {
            id: "box_1".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        plain
            .evaluate_bodies_with_warnings(&HashSet::new())
            .unwrap()
            .0[0]
            .1
            .mass_properties()
            .unwrap()
            .volume
    };
    let rejected = graph(CornerKind::Fillet, bottom_edges(), 100.0);
    let (bodies, warnings) = rejected
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("failed feature remains a valid model");
    assert!(!warnings.is_empty());
    let volume = bodies[0].1.mass_properties().unwrap().volume;
    assert!((volume - original).abs() <= 1.0e-3);
}
