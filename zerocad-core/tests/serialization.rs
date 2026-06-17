//! Document persistence guarantees: a `ParametricGraph` survives the JSON
//! round-trip the Save/Load path uses, and a corrupt document fails gracefully
//! (an `Err`, never a panic) so the GUI can surface it.

use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

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
