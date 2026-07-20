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
            offsets: vec![],
            patterns: vec![],
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

fn live_cancellation() -> EvaluationCancellation {
    EvaluationCancellation::new(1, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1)))
}

#[test]
fn evaluation_trace_classifies_cold_and_warm_work_without_wall_clock_time() {
    let mut graph = ParametricGraph::new();
    for i in 1..=3 {
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
    let hidden = std::collections::HashSet::new();
    let cold = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .unwrap();
    assert_eq!(cold.trace.evaluated_features.len(), 3);
    assert!(cold.trace.reused_checkpoints.is_empty());
    assert_eq!(cold.trace.pristine_mesh_reuse.len(), 3);
    assert!(cold.trace.tessellated_bodies.is_empty());

    let warm = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .unwrap();
    assert!(warm.trace.evaluated_features.is_empty());
    assert_eq!(warm.trace.reused_checkpoints.len(), 3);
    assert_eq!(warm.trace.pristine_mesh_reuse.len(), 3);
    assert_eq!(
        cold.trace.pristine_mesh_reuse, warm.trace.pristine_mesh_reuse,
        "warm evaluation must reuse the exact pristine mesh allocations"
    );
}

#[test]
fn typed_diagnostics_refresh_while_geometry_checkpoint_is_reused() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "import_1".into(),
        name: "Original import".into(),
        feature: FeatureType::Import {
            step_data: "garbage, not a STEP file".into(),
            label: "broken".into(),
        },
    });
    let hidden = std::collections::HashSet::new();
    let cold = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .unwrap();
    assert_eq!(
        cold.trace.evaluated_features,
        [crate::document::FeatureId::from("import_1")]
    );
    let cold_diagnostic = cold
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "import_1")
        .expect("failed import must emit a typed diagnostic");
    assert_eq!(
        cold_diagnostic.code.as_str(),
        DiagnosticCode::OPERATION_FAILED
    );
    assert_eq!(
        cold_diagnostic.parameters.get("feature_name"),
        Some(&DiagnosticParameterValue::Text("Original import".into()))
    );

    graph.graph[graph.node_map["import_1"]].name = "Renamed import".into();

    let warm = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .unwrap();
    assert!(warm.trace.evaluated_features.is_empty());
    assert_eq!(
        warm.trace.reused_checkpoints,
        [crate::document::FeatureId::from("import_1")]
    );
    let warm_diagnostic = warm
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "import_1")
        .expect("reused failure checkpoint must refresh its typed diagnostic");
    assert_eq!(
        warm_diagnostic.code.as_str(),
        DiagnosticCode::OPERATION_FAILED
    );
    assert_eq!(
        warm_diagnostic.parameters.get("feature_name"),
        Some(&DiagnosticParameterValue::Text("Renamed import".into()))
    );
    assert_eq!(warm.statuses[0].feature_name, "Renamed import");
}

#[test]
fn document_revision_advances_for_authoritative_modeling_mutators() {
    let mut document = crate::Document::new();
    let mut expected = document.revision();

    document.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    expected = expected.next();
    assert_eq!(document.revision(), expected, "add_feature");

    assert!(document.set_feature_suppressed("box_1", true));
    expected = expected.next();
    assert_eq!(document.revision(), expected, "set_feature_suppressed");

    assert!(document.set_feature_sequence("box_1", crate::document::SequenceKey(42)));
    expected = expected.next();
    assert_eq!(document.revision(), expected, "set_feature_sequence");

    let box_index = document.node_map["box_1"];
    document.graph[box_index].name = "Renamed box".into();
    expected = expected.next();
    assert_eq!(document.revision(), expected, "authoritative field edit");

    document.commit_feature_edit("box_1").unwrap();
    expected = expected.next();
    assert_eq!(document.revision(), expected, "commit_feature_edit");

    document.set_visible("box_1", false);
    expected = expected.next();
    assert_eq!(document.revision(), expected, "set_visible");
    document.set_visible("box_1", false);
    assert_eq!(
        document.revision(),
        expected,
        "a visibility no-op is not an edit"
    );

    assert!(document.remove_feature("box_1"));
    expected = expected.next();
    assert_eq!(document.revision(), expected, "remove_feature");

    document.clear();
    expected = expected.next();
    assert_eq!(document.revision(), expected, "clear");

    let mut ordered = crate::Document::from_graph(
        box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet),
        crate::Unit::Millimeter,
    );
    let before_reorder = ordered.revision();
    assert!(ordered.move_feature_in_timeline("edgemod_2", -1));
    assert_eq!(
        ordered.revision(),
        before_reorder.next(),
        "move_feature_in_timeline"
    );
}

#[test]
fn normal_document_evaluation_stamps_all_revision_bound_writebacks() {
    let mut document = crate::Document::new();
    document.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });

    let output = document
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .unwrap();
    assert_eq!(output.revision, document.revision());
    assert_eq!(
        output.face_reattach.producing_revision(),
        document.revision()
    );
    assert_eq!(
        output.legacy_reference_backfills.producing_revision(),
        document.revision()
    );
}

#[test]
fn stale_revision_bound_writebacks_are_rejected() {
    let mut document = crate::Document::new();
    let producing_revision = document.revision();
    let mut stale = FaceReattach::for_revision(producing_revision);
    stale
        .boundaries
        .insert("sketch_1".into(), SketchCurves::default());
    document.mark_edited();
    assert!(!document.apply_face_reattach_updates(stale));
    assert!(!document.sketch_face_boundaries.contains_key("sketch_1"));

    let mut current = FaceReattach::for_revision(document.revision());
    current
        .boundaries
        .insert("sketch_1".into(), SketchCurves::default());
    assert!(document.apply_face_reattach_updates(current));
    assert!(document.sketch_face_boundaries.contains_key("sketch_1"));
}

#[test]
fn authoritative_document_clone_drops_warm_evaluator_checkpoints() {
    let graph = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let document = crate::Document::from_graph(graph, crate::Unit::Millimeter);
    let hidden = std::collections::HashSet::new();
    document
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .unwrap();
    assert!(document
        .evaluator_graph()
        .eval_cache
        .borrow()
        .checkpoints
        .iter()
        .any(Option::is_some));

    let snapshot = document.clone_authoritative();
    assert!(snapshot
        .evaluator_graph()
        .eval_cache
        .borrow()
        .checkpoints
        .is_empty());
    assert_eq!(snapshot.revision(), document.revision());
    assert_eq!(snapshot.state.units, document.state.units);
}

#[derive(Debug, Clone)]
enum JsonStep {
    Key(String),
    Index(usize),
}

fn feature_mutation_corpus() -> Vec<FeatureType> {
    use crate::mock_kernel::EdgeCurveHint;
    use crate::parametric::{
        DraftNeutral, FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind,
    };
    use crate::{
        AxisBase, CoordinateSystem, DatumAxisDef, DatumPlaneDef, DatumPointDef, HoleKind,
        PatternKind, PlaneBase, Variable,
    };

    let face = FaceRef {
        centroid: [1.0, 2.0, 3.0],
        normal: [0.0, 0.0, 1.0],
        topology: None,
    };
    let direct_face = FaceRef {
        topology: Some(TopologyFaceRef {
            body_id: Some("missing_a".into()),
            face_id: Some("foreign:face:1".into()),
            producer_feature_id: Some("missing_a".into()),
            ..TopologyFaceRef::default()
        }),
        ..face.clone()
    };
    let edge = EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [1.0, 0.0, 0.0],
        n1: [0.0, 1.0, 0.0],
        n2: [0.0, 0.0, 1.0],
        curve: Some(EdgeCurveHint::Line),
        topology: None,
    };
    vec![
        FeatureType::Origin,
        FeatureType::Box {
            w: 2.0,
            h: 3.0,
            d: 4.0,
        },
        FeatureType::Cylinder { r: 2.0, h: 5.0 },
        FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 1,
            solver: None,
        },
        FeatureType::Extrude {
            depth: 6.0,
            region_indices: vec![0],
            mode: ExtrudeMode::NewBody,
            target: Some("missing_body".into()),
            depth_expr: Some("3*2".into()),
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
        FeatureType::EdgeMod {
            target: "missing_body".into(),
            edge,
            dist: 0.2,
            dist_expr: Some("0.1*2".into()),
            kind: crate::sketch::CornerKind::Chamfer,
        },
        FeatureType::VariableSet {
            variables: vec![Variable::new("width", crate::Unit::Millimeter)],
        },
        FeatureType::Import {
            step_data: "ISO-10303-21;END-ISO-10303-21;".into(),
            label: "fixture".into(),
        },
        FeatureType::Revolve {
            axis: AxisBase::X,
            angle_deg: 180.0,
            angle_expr: Some("90*2".into()),
            region_indices: vec![0],
            mode: ExtrudeMode::NewBody,
            target: Some("missing_body".into()),
        },
        FeatureType::Loft {
            sections: vec![("section_a".into(), 0), ("section_b".into(), 1)],
            mode: ExtrudeMode::NewBody,
            target: Some("missing_body".into()),
        },
        FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: Some("missing_body".into()),
            total_twist_deg: 30.0,
            total_twist_expr: Some("twist".into()),
        },
        FeatureType::Shell {
            target: "missing_body".into(),
            thickness: 0.5,
            thickness_expr: Some("wall".into()),
            open_faces: vec![face.clone()],
        },
        FeatureType::Hole {
            target: "missing_body".into(),
            position: [1.0, 1.0, 1.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: Some("bore".into()),
            depth: Some(4.0),
            kind: HoleKind::Counterbore {
                diameter: 4.0,
                depth: 1.0,
            },
            standard: None,
            manufacturing: None,
        },
        FeatureType::Pattern {
            source: "missing_body".into(),
            kind: PatternKind::Linear {
                dir: AxisBase::Y,
                spacing: 3.0,
                spacing_expr: Some("step".into()),
                count: 3,
            },
        },
        FeatureType::FeaturePattern {
            target: "missing_body".into(),
            source_feature: "missing_source".into(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::Y,
                spacing: 3.0,
                spacing_expr: Some("feature_step".into()),
                count: 3,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::SourceExtent,
        },
        FeatureType::BodyTransform {
            source: "missing_body".into(),
            translation: [1.0, 2.0, 3.0],
            copy: true,
        },
        FeatureType::Thread {
            target: "missing_body".into(),
            face: face.clone(),
            internal: true,
            pitch: 1.0,
            depth: 0.3,
            angle_deg: 60.0,
            right_handed: true,
            starts: 2,
            length: Some(8.0),
            flip: true,
            designation: "M6x1".into(),
            standard: None,
        },
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: Some("datum_height".into()),
            },
        },
        FeatureType::DatumAxis {
            def: DatumAxisDef::TwoPoints {
                a: [0.0, 0.0, 0.0],
                b: [0.0, 0.0, 1.0],
            },
        },
        FeatureType::DatumPoint {
            def: DatumPointDef::Coords { p: [1.0, 2.0, 3.0] },
        },
        FeatureType::BodyJoin {
            sources: vec!["missing_a".into(), "missing_b".into()],
        },
        FeatureType::BodyCut {
            target: "missing_a".into(),
            tool: "missing_b".into(),
            keep_tool: true,
        },
        FeatureType::BodyIntersect {
            target: "missing_a".into(),
            tool: "missing_b".into(),
            keep_tool: true,
        },
        FeatureType::BodySplit {
            target: "missing_a".into(),
            plane: PlaneBase::Datum("missing_plane".into()),
            face: None,
        },
        FeatureType::BodyScale {
            source: "missing_a".into(),
            factor: 2.0,
            factor_expr: Some("scale_factor".into()),
            center: [1.0, 2.0, 3.0],
        },
        FeatureType::FaceOffset {
            target: "missing_a".into(),
            face: direct_face.clone(),
            distance: -2.0,
            distance_expr: Some("-wall".into()),
        },
        FeatureType::FaceMove {
            target: "missing_a".into(),
            face: direct_face.clone(),
            translation: [0.0, 0.0, 2.0],
        },
        FeatureType::FaceDelete {
            target: "missing_a".into(),
            face: direct_face.clone(),
        },
        FeatureType::FaceThicken {
            target: "missing_a".into(),
            face: direct_face.clone(),
            thickness: 1.5,
            thickness_expr: Some("sheet".into()),
            reverse: true,
        },
        FeatureType::Draft {
            target: "missing_a".into(),
            faces: vec![direct_face.clone()],
            neutral: DraftNeutral::Face(direct_face),
            angle_deg: 3.0,
            angle_expr: Some("draft_angle".into()),
            flip_pull: true,
        },
        FeatureType::ImportStl {
            stl_data: b"solid empty\nendsolid empty\n".to_vec(),
            label: "empty.stl".into(),
        },
    ]
}

fn json_leaf_paths(
    value: &serde_json::Value,
    path: &mut Vec<JsonStep>,
    out: &mut Vec<Vec<JsonStep>>,
) {
    match value {
        serde_json::Value::Object(fields) => {
            for (key, value) in fields {
                path.push(JsonStep::Key(key.clone()));
                json_leaf_paths(value, path, out);
                path.pop();
            }
        }
        serde_json::Value::Array(values) if !values.is_empty() => {
            for (index, value) in values.iter().enumerate() {
                path.push(JsonStep::Index(index));
                json_leaf_paths(value, path, out);
                path.pop();
            }
        }
        _ => out.push(path.clone()),
    }
}

fn value_at_mut<'a>(
    mut value: &'a mut serde_json::Value,
    path: &[JsonStep],
) -> &'a mut serde_json::Value {
    for step in path {
        value = match step {
            JsonStep::Key(key) => value.get_mut(key).unwrap(),
            JsonStep::Index(index) => value.get_mut(*index).unwrap(),
        };
    }
    value
}

fn mutation_candidates(value: &serde_json::Value) -> Vec<serde_json::Value> {
    use serde_json::{Number, Value};
    match value {
        Value::Bool(value) => vec![Value::Bool(!value)],
        Value::Number(value) => vec![Value::Number(
            Number::from_f64(value.as_f64().unwrap() + 0.375).unwrap(),
        )],
        Value::String(value) => {
            let mut candidates = match value.as_str() {
                "SourceExtent" => vec![Value::String("ThroughAllLocalTarget".into())],
                "ThroughAllLocalTarget" => vec![Value::String("SourceExtent".into())],
                "Identical" => vec![Value::String("UnsupportedForTest".into())],
                _ => Vec::new(),
            };
            candidates.push(Value::String(format!("{value}_mutated")));
            candidates
        }
        Value::Null => vec![
            Value::String("mutated".into()),
            Value::Number(Number::from_f64(1.375).unwrap()),
            Value::Bool(true),
            Value::Array(Vec::new()),
            Value::Object(Default::default()),
        ],
        Value::Array(values) if values.is_empty() => vec![
            Value::Array(vec![Value::String("mutated".into())]),
            Value::Array(vec![Value::Number(Number::from(1))]),
            Value::Array(vec![Value::Bool(true)]),
        ],
        Value::Array(_) | Value::Object(_) => unreachable!("only leaves are mutated"),
    }
}

#[test]
fn every_mutable_persisted_feature_leaf_matches_a_cache_free_rebuild() {
    let hidden = std::collections::HashSet::new();
    let mut exercised = 0usize;
    let mut variants = 0usize;

    for (variant_index, original) in feature_mutation_corpus().into_iter().enumerate() {
        let encoded = serde_json::to_value(&original).unwrap();
        let variant_name = encoded
            .as_object()
            .and_then(|object| object.keys().next())
            .cloned()
            .unwrap_or_else(|| "Origin".into());
        let mut paths = Vec::new();
        json_leaf_paths(&encoded, &mut Vec::new(), &mut paths);
        let mut variant_exercised = 0usize;

        for (leaf_index, path) in paths.into_iter().enumerate() {
            let original_leaf = path.iter().fold(&encoded, |value, step| match step {
                JsonStep::Key(key) => &value[key],
                JsonStep::Index(index) => &value[*index],
            });
            let changed = mutation_candidates(original_leaf)
                .into_iter()
                .find_map(|candidate| {
                    let mut encoded = encoded.clone();
                    *value_at_mut(&mut encoded, &path) = candidate;
                    serde_json::from_value::<FeatureType>(encoded).ok()
                });
            let Some(changed) = changed else { continue };

            let id = format!("mutation_{variant_index}_{leaf_index}");
            let mut warm = ParametricGraph::new();
            warm.add_feature(FeatureNode {
                id: id.clone(),
                name: variant_name.clone(),
                feature: original.clone(),
            });
            warm.evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
                .unwrap();
            let index = warm.node_map[id.as_str()];
            warm.graph[index].feature = changed;
            warm.refresh_feature_contract_over_warm_cache_for_test(&id)
                .unwrap();
            let cold = warm.clone_document();

            let warm_output = warm
                .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
                .unwrap();
            let cold_output = cold
                .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
                .unwrap();
            assert_eq!(
                mesh_digest(&warm_output.bodies),
                mesh_digest(&cold_output.bodies),
                "warm/cold geometry drift for {variant_name} at {path:?}"
            );
            assert_eq!(
                warm_output.warnings, cold_output.warnings,
                "warm/cold warnings drift for {variant_name} at {path:?}"
            );
            assert_eq!(
                warm_output.diagnostics, cold_output.diagnostics,
                "warm/cold diagnostics drift for {variant_name} at {path:?}"
            );
            assert_eq!(
                warm_output.statuses, cold_output.statuses,
                "warm/cold statuses drift for {variant_name} at {path:?}"
            );
            variant_exercised += 1;
            exercised += 1;
        }

        if !matches!(original, FeatureType::Origin) {
            assert!(
                variant_exercised > 0,
                "the mutation corpus must exercise at least one persisted field of {variant_name}"
            );
            variants += 1;
        }
    }

    assert_eq!(
        variants, 30,
        "all persisted feature variants must be present"
    );
    assert!(
        exercised >= 150,
        "mutation coverage unexpectedly fell to {exercised} leaves"
    );
}
