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
fn eval_cache_key_changes_when_edge_mod_replay_metadata_changes() {
    let baseline = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let mut with_replay = baseline.clone();
    let idx = with_replay.node_map["edgemod_2"];
    if let FeatureType::EdgeMod { edge, replay, .. } = &mut with_replay.graph[idx].feature {
        *replay = EdgeModReplayIntent {
            mode: EdgeModReplayMode::Auto,
            pre_cut_target: Some("box_1".to_string()),
            replay_cut_nodes: vec!["cut_1".to_string()],
            selected_span: Some(edge.clone()),
        };
    } else {
        panic!("test fixture should contain an EdgeMod");
    }

    let hidden = std::collections::HashSet::new();
    let vars = std::collections::HashMap::new();
    let baseline_nodes = baseline.body_nodes_in_creation_order();
    let replay_nodes = with_replay.body_nodes_in_creation_order();
    assert_ne!(
        baseline.eval_prefix_keys(&baseline_nodes, &hidden, &vars),
        with_replay.eval_prefix_keys(&replay_nodes, &hidden, &vars),
        "replay metadata must participate in the mesh-cache prefix hash"
    );
}
