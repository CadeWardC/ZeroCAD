use super::*;

#[test]
fn region_cache_returns_consistent_regions() {
    // The cached path (second call) must return the same regions as the
    // first, uncached call — the cache is a transparent accelerator.
    let g = ParametricGraph::new();
    let curves = rect_sketch((0.0, 0.0), (10.0, 10.0));
    let first = g.cached_regions(&curves);
    let second = g.cached_regions(&curves);
    assert_eq!(first, second, "cache must not change the result");
    assert_eq!(first.len(), 1, "a rectangle is exactly one region");
}

#[test]
fn eval_cache_matches_cold_eval_after_radius_drag() {
    // The prefix cache is a pure accelerator: re-evaluating after changing only
    // a trailing edge-mod (a radius drag) must yield byte-identical geometry to
    // a freshly built graph at the same final state — never a stale prefix.
    let empty = std::collections::HashSet::new();

    let mut warm = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    // Warm the cache at radius 2.0, then "drag" to 3.0 and re-evaluate (the box
    // prefix is reused from the checkpoint; only the fillet re-runs).
    let _ = warm.evaluate_bodies_with_warnings(&empty).unwrap();
    set_edge_mod_dist(&mut warm, "edgemod_2", 3.0);
    let (warm_bodies, warm_warn) = warm.evaluate_bodies_with_warnings(&empty).unwrap();

    // Cold: an identical graph at radius 3.0 with an empty cache.
    let cold = box_with_edge_mod(3.0, crate::sketch::CornerKind::Fillet);
    let (cold_bodies, cold_warn) = cold.evaluate_bodies_with_warnings(&empty).unwrap();

    assert_eq!(
        mesh_digest(&warm_bodies),
        mesh_digest(&cold_bodies),
        "cached re-eval after a radius drag must match a cold rebuild exactly"
    );
    assert_eq!(
        warm_warn, cold_warn,
        "warnings must match the cold rebuild too"
    );
}

#[test]
fn eval_cache_is_invalidated_when_an_upstream_node_changes() {
    // Changing an *upstream* dimension (the box size) must not serve a stale
    // cached body — the prefix key changes, forcing a rebuild that matches cold.
    let empty = std::collections::HashSet::new();
    let mut g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let _ = g.evaluate_bodies_with_warnings(&empty).unwrap();

    // Grow the box: this is the very first node, so nothing downstream can be
    // reused.
    let box_idx = g.node_map["box_1"];
    if let FeatureType::Box { w, h, d } = &mut g.graph[box_idx].feature {
        *w = 20.0;
        *h = 20.0;
        *d = 20.0;
    }
    let (warm_bodies, _) = g.evaluate_bodies_with_warnings(&empty).unwrap();

    let mut cold = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let cold_idx = cold.node_map["box_1"];
    if let FeatureType::Box { w, h, d } = &mut cold.graph[cold_idx].feature {
        *w = 20.0;
        *h = 20.0;
        *d = 20.0;
    }
    let (cold_bodies, _) = cold.evaluate_bodies_with_warnings(&empty).unwrap();

    assert_eq!(
        mesh_digest(&warm_bodies),
        mesh_digest(&cold_bodies),
        "an upstream change must invalidate the cache, not serve stale geometry"
    );
}

#[test]
fn eval_cache_key_changes_when_a_sketch_constraint_changes() {
    // The solver model lives INSIDE the serde-hashed Sketch feature, so any
    // constraint add/edit must change the prefix key — a `#[serde(skip)]`
    // regression here would rebuild downstream bodies from a stale mesh.
    let mut baseline = ParametricGraph::new();
    add_sketch(
        &mut baseline,
        "sketch_1",
        rect_sketch((0.0, 0.0), (20.0, 12.0)),
    );
    add_extrude(
        &mut baseline,
        "extrude_2",
        "sketch_1",
        8.0,
        ExtrudeMode::NewBody,
    );

    let mut with_constraint = baseline.clone();
    let idx = with_constraint.node_map["sketch_1"];
    if let FeatureType::Sketch { solver, .. } = &mut with_constraint.graph[idx].feature {
        use crate::sketch::{Constraint, EntityId, SketchPoint, SketchSolverModel};
        *solver = Some(SketchSolverModel {
            points: vec![SketchPoint {
                id: EntityId(0),
                pos: (0.0, 0.0),
            }],
            entities: vec![],
            constraints: vec![Constraint::Fixed {
                id: EntityId(1),
                p: EntityId(0),
            }],
            construction: vec![],
            driven_dimensions: vec![],
            projected_edges: vec![],
        });
    } else {
        panic!("test fixture should contain a Sketch");
    }

    let hidden = std::collections::HashSet::new();
    let vars = std::collections::HashMap::new();
    let baseline_nodes = baseline.body_nodes_in_creation_order();
    let constraint_nodes = with_constraint.body_nodes_in_creation_order();
    assert_ne!(
        baseline.eval_prefix_keys(&baseline_nodes, &hidden, &vars),
        with_constraint.eval_prefix_keys(&constraint_nodes, &hidden, &vars),
        "the solver model must participate in the mesh-cache prefix hash"
    );
}

#[test]
fn eval_cache_key_changes_when_edge_mod_selection_changes() {
    let baseline = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let mut changed = baseline.clone();
    let idx = changed.node_map["edgemod_2"];
    if let FeatureType::EdgeMod { edge, .. } = &mut changed.graph[idx].feature {
        edge.p1[0] -= 0.25;
    } else {
        panic!("test fixture should contain an EdgeMod");
    }

    let hidden = std::collections::HashSet::new();
    let vars = std::collections::HashMap::new();
    let baseline_nodes = baseline.body_nodes_in_creation_order();
    let changed_nodes = changed.body_nodes_in_creation_order();
    assert_ne!(
        baseline.eval_prefix_keys(&baseline_nodes, &hidden, &vars),
        changed.eval_prefix_keys(&changed_nodes, &hidden, &vars),
        "the semantic edge selection must participate in the mesh-cache prefix hash"
    );
}

#[test]
fn checkpoints_share_unchanged_pristine_meshes() {
    let mut graph = ParametricGraph::new();
    for i in 1..=8 {
        graph.add_feature(FeatureNode {
            id: format!("box_{i}"),
            name: format!("Box {i}"),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
    }
    graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    let cache = graph.eval_cache.borrow();
    let first = cache.checkpoints[0].as_ref().unwrap().live[0]
        .pristine
        .as_ref()
        .expect("box checkpoint has a pristine mesh");
    for checkpoint in cache.checkpoints.iter().skip(1).flatten() {
        let reused = checkpoint.live[0]
            .pristine
            .as_ref()
            .expect("unchanged box remains pristine");
        assert!(
            std::sync::Arc::ptr_eq(first, reused),
            "checkpoint clones must share unchanged mesh allocation"
        );
    }
}

#[test]
fn complete_checkpoint_hit_keeps_the_immutable_cache_allocation() {
    let graph = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let hidden = std::collections::HashSet::new();
    graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let before = graph.eval_cache.borrow().clone();

    graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let after = graph.eval_cache.borrow().clone();

    assert!(
        std::sync::Arc::ptr_eq(&before, &after),
        "a complete warm hit must not rebuild the checkpoint vector"
    );
}

#[test]
fn superseded_evaluation_exits_without_geometry() {
    let graph = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    let cancellation = EvaluationCancellation::new(1, latest);
    let result = graph.evaluate_request(
        &std::collections::HashSet::new(),
        EvaluationQuality::Interactive,
        &cancellation,
    );
    assert!(matches!(result, Err(EvaluationError::Cancelled)));
}

#[test]
fn cache_snapshot_survives_background_graph_handoff() {
    let graph = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let expected = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap()
        .0;
    let snapshot = graph.evaluation_cache_snapshot();

    let restored = graph.clone_document();
    assert!(restored.eval_cache.borrow().checkpoints.is_empty());
    restored.install_evaluation_cache(snapshot);
    assert!(restored
        .eval_cache
        .borrow()
        .checkpoints
        .iter()
        .any(Option::is_some));
    let actual = restored
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap()
        .0;
    assert_eq!(actual.len(), expected.len());
    for ((actual_id, actual_mesh), (expected_id, expected_mesh)) in actual.iter().zip(&expected) {
        assert_eq!(actual_id, expected_id);
        assert_eq!(actual_mesh.vertices, expected_mesh.vertices);
        assert_eq!(actual_mesh.indices, expected_mesh.indices);
    }
}

#[test]
fn graph_clone_shares_warm_cache_until_worker_rebuilds_it() {
    let graph = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    let worker_graph = graph.clone();
    assert!(std::sync::Arc::ptr_eq(
        &graph.eval_cache.borrow(),
        &worker_graph.eval_cache.borrow()
    ));
    let snapshot = graph.evaluation_cache_snapshot();
    assert!(std::sync::Arc::ptr_eq(
        &graph.eval_cache.borrow(),
        &snapshot.cache
    ));
}
