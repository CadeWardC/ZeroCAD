//! node_map desync regressions: `node_map` is `#[serde(skip)]` and petgraph
//! `remove_node` swap-moves indices, so a graph restored from an undo/redo
//! snapshot (or a node deleted outside `remove_feature`) used to leave
//! `add_dependency` silently no-opping — a re-extruded sketch produced an
//! extrude node with no parent edge, which builds no body at all.

use super::*;
use std::collections::HashSet;

fn body_count(g: &ParametricGraph) -> usize {
    let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    bodies.iter().filter(|(_, m)| !m.indices.is_empty()).count()
}

/// GUI undo path: the snapshot is a serde round-trip, which drops `node_map`.
/// Re-extruding the restored sketch must still wire the dependency edge.
#[test]
fn re_extrude_after_serde_round_trip_builds_a_body() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));

    // Simulate undo-of-extrude: snapshot with just the sketch, restore it.
    let snap = serde_json::to_string(&g).unwrap();
    let mut g: ParametricGraph = serde_json::from_str(&snap).unwrap();

    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    assert_eq!(
        body_count(&g),
        1,
        "extrude added after a serde round-trip must find its sketch parent"
    );
}

#[test]
fn serde_restore_rederives_node_map_before_the_first_evaluation() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);

    let restored: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&g).unwrap()).unwrap();
    assert_eq!(restored.node_map.len(), restored.graph.node_count());
    for index in restored.graph.node_indices() {
        assert_eq!(
            restored.node_map.get(&restored.graph[index].id),
            Some(&index)
        );
    }
    assert_eq!(body_count(&restored), 1);
}

/// GUI delete path: removing a middle node swap-moves the last node's index.
/// `remove_feature` must leave the map consistent so a later extrude attaches
/// to the sketch, not to whatever slid into the stale slot.
#[test]
fn re_extrude_after_remove_feature_builds_a_body() {
    let mut g = ParametricGraph::new();
    // Order matters: sketch_1 sits AFTER extrude-victim candidates so its
    // index shifts when an earlier node is removed.
    add_sketch(&mut g, "sketch_0", rect_sketch((20.0, 20.0), (25.0, 25.0)));
    add_extrude(&mut g, "extrude_1", "sketch_0", 5.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));

    assert!(g.remove_feature("extrude_1"));
    assert!(!g.remove_feature("extrude_1"), "second delete is a no-op");

    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    assert_eq!(
        body_count(&g),
        1,
        "extrude added after remove_feature must attach to the surviving sketch"
    );
}

/// Full lifecycle regression for the historical serde-skip trap: capture an
/// undo snapshot, restore it, serialize/load once more, delete the original
/// body feature (which swap-moves the arena), and re-extrude the surviving
/// sketch. Every restore/delete boundary must rederive or self-heal `node_map`.
#[test]
fn undo_load_delete_and_re_extrude_preserves_dependency_wiring() {
    let mut original = ParametricGraph::new();
    add_sketch(
        &mut original,
        "sketch_1",
        rect_sketch((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(
        &mut original,
        "extrude_2",
        "sketch_1",
        5.0,
        ExtrudeMode::NewBody,
    );

    let undo_snapshot = serde_json::to_string(&original.clone_document()).unwrap();
    original.add_feature(FeatureNode {
        id: "box_3".into(),
        name: "Transient edit".into(),
        feature: FeatureType::Box {
            w: 2.0,
            h: 2.0,
            d: 2.0,
        },
    });
    let restored: ParametricGraph = serde_json::from_str(&undo_snapshot).unwrap();

    // A second round trip represents canonical document loading after undo.
    let loaded_bytes = serde_json::to_vec(&restored).unwrap();
    let mut loaded: ParametricGraph = serde_json::from_slice(&loaded_bytes).unwrap();
    assert!(loaded.remove_feature("extrude_2"));
    add_extrude(
        &mut loaded,
        "extrude_4",
        "sketch_1",
        7.0,
        ExtrudeMode::NewBody,
    );

    assert_eq!(body_count(&loaded), 1);
    let sketch = loaded.node_map["sketch_1"];
    let extrude = loaded.node_map["extrude_4"];
    assert!(loaded.graph.find_edge(sketch, extrude).is_some());
}

#[test]
fn evaluator_family_dependency_matrix_is_explicit() {
    use crate::document::FeatureEvaluatorKind as Family;

    // Infrastructure/sketch/datum are pre-pass families. Every body family is
    // listed here so typed-ID migration cannot silently omit a newly registered
    // evaluator from lifecycle coverage.
    let body_families = [
        Family::Box,
        Family::Cylinder,
        Family::Extrude,
        Family::EdgeMod,
        Family::Import,
        Family::Revolve,
        Family::Loft,
        Family::Sweep,
        Family::Shell,
        Family::Hole,
        Family::Pattern,
        Family::BodyTransform,
        Family::Thread,
        Family::BodyJoin,
        Family::BodyCut,
        Family::BodyIntersect,
        Family::BodySplit,
        Family::BodyScale,
        Family::FaceOffset,
        Family::FaceMove,
        Family::FaceDelete,
        Family::FaceThicken,
        Family::ImportStl,
    ];
    let registered: Vec<_> = crate::document::FeatureRegistry::BUILTINS
        .iter()
        .map(|registration| registration.evaluator)
        .filter(|family| {
            !matches!(
                family,
                Family::Infrastructure | Family::Sketch | Family::Datum
            )
        })
        .collect();
    assert_eq!(registered.len(), body_families.len());
    for family in body_families {
        assert!(registered.contains(&family), "missing family {family:?}");
    }
}

/// Even a raw `graph.remove_node` (bypassing `remove_feature`) must not poison
/// later `add_dependency` calls: the lookup self-heals on a stale entry.
#[test]
fn add_dependency_self_heals_after_raw_remove_node() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_0", rect_sketch((20.0, 20.0), (25.0, 25.0)));
    add_extrude(&mut g, "extrude_1", "sketch_0", 5.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));

    // Raw removal: swap-moves sketch_1 into extrude_1's slot, node_map stale.
    let victim = g.node_map["extrude_1"];
    g.graph.remove_node(victim);

    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    assert_eq!(
        body_count(&g),
        1,
        "add_dependency must re-resolve ids when the cached index is stale"
    );
}

/// Deleting a body reveals its consumed sketch: `sole_sketch_parents` reports
/// the sketch only while the extrude is its sole consumer.
#[test]
fn sole_sketch_parents_reports_consumed_sketch() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    assert_eq!(g.sole_sketch_parents("extrude_2"), vec!["sketch_1"]);

    // A second consumer of the sketch means deleting one extrude must NOT
    // reveal it.
    add_extrude(&mut g, "extrude_3", "sketch_1", 2.0, ExtrudeMode::NewBody);
    assert!(g.sole_sketch_parents("extrude_2").is_empty());
    assert!(g.sole_sketch_parents("missing").is_empty());
}

#[test]
fn generated_body_output_dependency_resolves_to_owning_feature() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    g.add_feature(FeatureNode {
        id: "move_3".to_string(),
        name: "Move".to_string(),
        feature: FeatureType::BodyTransform {
            source: body_output_id("extrude_2", 1),
            translation: [1.0, 0.0, 0.0],
            copy: true,
        },
    });
    // Secondary outputs are document semantics, not an identifier convention.
    // A producer must register them before another feature may depend on them.
    g.semantics.body_outputs.insert(
        body_output_id("extrude_2", 1),
        crate::document::FeatureId::from("extrude_2"),
    );
    g.add_dependency("extrude_2::body:2", "move_3");

    let owner = g.node_map["extrude_2"];
    let child = g.node_map["move_3"];
    assert!(g.graph.find_edge(owner, child).is_some());
    assert_eq!(
        g.body_producer_feature_id("extrude_2::body:2"),
        Some("extrude_2")
    );
    assert_eq!(g.body_producer_feature_id("missing::body:2"), None);
}
