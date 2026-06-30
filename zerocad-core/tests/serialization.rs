//! Document persistence guarantees: a `ParametricGraph` survives the JSON
//! round-trip the Save/Load path uses, and a corrupt document fails gracefully
//! (an `Err`, never a panic) so the GUI can surface it.

use zerocad_core::{
    CornerKind, EdgeModReplayIntent, EdgeModReplayMode, EdgeModScope, EdgeRef, FeatureNode,
    FeatureType, ParametricGraph,
};

fn box_graph() -> ParametricGraph {
    let mut pg = ParametricGraph::new();
    pg.add_feature(FeatureNode {
        id: "box1".to_string(),
        name: "Block".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 20.0,
            d: 30.0,
        },
    });
    pg
}

#[test]
fn graph_survives_json_round_trip() {
    let pg = box_graph();
    let json = serde_json::to_string(&pg).expect("serialize");
    let restored: ParametricGraph = serde_json::from_str(&json).expect("deserialize");

    // The restored graph must evaluate to the same geometry as the original.
    let before = pg.evaluate().expect("eval original");
    let after = restored.evaluate().expect("eval restored");
    assert_eq!(before.indices.len(), after.indices.len());
    assert_eq!(before.vertices.len(), after.vertices.len());
}

#[test]
fn pretty_json_round_trip_matches() {
    // The Save Design path uses `to_string_pretty`; ensure that form round-trips.
    let pg = box_graph();
    let json = serde_json::to_string_pretty(&pg).expect("serialize pretty");
    let restored: ParametricGraph = serde_json::from_str(&json).expect("deserialize pretty");
    assert_eq!(
        restored.evaluate().unwrap().indices.len(),
        pg.evaluate().unwrap().indices.len()
    );
}

#[test]
fn malformed_document_is_an_error_not_a_panic() {
    // Loading a corrupt `.zcad` must fail gracefully — never panic.
    for bad in ["", "not json at all", "[1,2,3]", "{\"graph\": 42}"] {
        let res = serde_json::from_str::<ParametricGraph>(bad);
        assert!(res.is_err(), "expected a load error for input {bad:?}");
    }
}

#[test]
fn edge_mod_replay_metadata_survives_json_round_trip() {
    let edge = EdgeRef {
        p0: [0.0, 0.0, 10.0],
        p1: [8.5, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    };
    let replay = EdgeModReplayIntent {
        mode: EdgeModReplayMode::Auto,
        pre_cut_target: Some("box1".to_string()),
        replay_cut_nodes: vec!["cut1".to_string(), "cut2".to_string()],
        selected_span: Some(edge.clone()),
    };
    let mut pg = box_graph();
    pg.add_feature(FeatureNode {
        id: "fillet1".to_string(),
        name: "Replay Fillet".to_string(),
        feature: FeatureType::EdgeMod {
            target: "box1".to_string(),
            edge,
            dist: 3.0,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            replay: replay.clone(),
            kind: CornerKind::Fillet,
        },
    });
    pg.add_dependency("box1", "fillet1");

    let json = serde_json::to_string(&pg).expect("serialize");
    let restored: ParametricGraph = serde_json::from_str(&json).expect("deserialize");
    let idx = restored
        .graph
        .node_indices()
        .find(|&idx| restored.graph[idx].id == "fillet1")
        .expect("restored replay fillet node");
    match &restored.graph[idx].feature {
        FeatureType::EdgeMod {
            replay: restored, ..
        } => assert_eq!(restored, &replay),
        other => panic!("expected restored EdgeMod, got {other:?}"),
    }
}
