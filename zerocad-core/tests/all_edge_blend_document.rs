//! Wave 5D product-level all-edge persistence and durable-selection gate.

use std::collections::HashSet;
use std::sync::atomic::AtomicU64;
use std::sync::Arc;

use zerocad_core::parametric::TopologyEdgeRef;
use zerocad_core::{
    read_document_file, write_document_file, AllEdgeSelector, CoordinateSystem, CornerKind,
    DiagnosticCode, Document, EdgeRef, EvaluationCancellation, EvaluationError, EvaluationQuality,
    ExtrudeMode, FeatureNode, FeatureType, HydrationBundle, LoadOptions, ParametricGraph,
    SaveOptions, SketchCurves, Unit,
};

fn captured_edge(body_id: &str, edge: &zerocad_core::mock_kernel::MeshEdgeRef) -> EdgeRef {
    EdgeRef {
        p0: edge.p0,
        p1: edge.p1,
        n1: edge.n1,
        n2: edge.n2,
        curve: edge.curve.clone(),
        topology: edge.topology.as_ref().map(|topology| TopologyEdgeRef {
            body_id: topology
                .body_id
                .clone()
                .or_else(|| Some(body_id.to_string())),
            topology_version: topology.topology_version,
            edge_id: topology.edge_id.clone(),
            adjacent_face_ids: topology.adjacent_face_ids.clone(),
            curve_kind: topology.curve_kind.clone(),
            adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    }
}

fn all_edge_box_graph(kind: CornerKind) -> (ParametricGraph, usize) {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (4.0, 5.0));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch".into(),
        name: "All-edge box profile".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude".into(),
        name: "All-edge box".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 6.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "extrude");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate all-edge source box");
    assert!(warnings.is_empty(), "source warnings: {warnings:?}");
    let source = bodies.first().expect("one all-edge source body");
    let mut edges = source
        .1
        .edge_refs
        .iter()
        .map(|edge| {
            let captured = captured_edge(&source.0, edge);
            let name = captured
                .topology
                .as_ref()
                .and_then(|topology| topology.edge_id.clone())
                .expect("every GUI-selectable all-edge candidate must have a durable name");
            (name, captured)
        })
        .collect::<Vec<_>>();
    edges.sort_by(|left, right| left.0.cmp(&right.0));
    assert!(!edges.is_empty());
    assert!(edges.windows(2).all(|pair| pair[0].0 != pair[1].0));

    let edge_count = edges.len();
    let selector = AllEdgeSelector::new(edges.iter().map(|(name, _)| name.clone()))
        .expect("unique durable all-edge selector")
        .encode()
        .expect("encode all-edge selector");
    let mut representative = edges[0].1.clone();
    representative
        .topology
        .as_mut()
        .expect("representative topology")
        .edge_id = Some(selector);
    graph.add_feature(FeatureNode {
        id: "fillet_all".into(),
        name: format!("{kind:?} all edges"),
        feature: FeatureType::EdgeMod {
            target: "extrude".into(),
            edge: representative,
            dist: 0.4,
            dist_expr: None,
            kind,
        },
    });
    graph.add_dependency("extrude", "fillet_all");
    (graph, edge_count)
}

fn all_edge_primitive_box_graph(kind: CornerKind) -> (ParametricGraph, usize) {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box".into(),
        name: "Primitive box".into(),
        feature: FeatureType::Box {
            w: 4.0,
            h: 5.0,
            d: 6.0,
        },
    });
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate primitive all-edge source");
    assert!(warnings.is_empty());
    let source = &bodies[0];
    let mut edges = source
        .1
        .edge_refs
        .iter()
        .map(|edge| {
            let topology = edge.topology.as_ref().expect("primitive edge topology");
            let key = AllEdgeSelector::durable_edge_key(
                topology.edge_id.as_deref(),
                &topology.adjacent_face_ids,
            )
            .expect("primitive edge has a durable face-pair identity");
            (key, captured_edge(&source.0, edge))
        })
        .collect::<Vec<_>>();
    edges.sort_by(|left, right| left.0.cmp(&right.0));
    assert_eq!(edges.len(), 12);
    assert!(edges.windows(2).all(|pair| pair[0].0 != pair[1].0));
    let selector = AllEdgeSelector::new(edges.iter().map(|(key, _)| key.clone()))
        .expect("primitive selector")
        .encode()
        .expect("encode primitive selector");
    let mut representative = edges[0].1.clone();
    representative
        .topology
        .as_mut()
        .expect("primitive representative topology")
        .edge_id = Some(selector);
    graph.add_feature(FeatureNode {
        id: "fillet_all".into(),
        name: format!("{kind:?} primitive edges"),
        feature: FeatureType::EdgeMod {
            target: "box".into(),
            edge: representative,
            dist: 0.4,
            dist_expr: None,
            kind,
        },
    });
    graph.add_dependency("box", "fillet_all");
    (graph, edges.len())
}

fn evaluate_clean(graph: &ParametricGraph) -> (Vec<u32>, usize) {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate all-edge document");
    assert!(warnings.is_empty(), "all-edge warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;
    assert!(!mesh.vertices.is_empty());
    (mesh.face_ids.clone(), mesh.indices.len())
}

fn sharp_source_mesh(graph: &ParametricGraph) -> zerocad_core::MockMesh {
    let mut source = graph.clone();
    assert!(source.remove_feature("fillet_all"));
    let (bodies, warnings) = source
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate unmodified source");
    assert!(warnings.is_empty());
    bodies.into_iter().next().expect("source body").1
}

fn active_cancellation() -> EvaluationCancellation {
    EvaluationCancellation::new(1, Arc::new(AtomicU64::new(1)))
}

fn cancelled_request() -> EvaluationCancellation {
    EvaluationCancellation::new(1, Arc::new(AtomicU64::new(2)))
}

#[test]
fn durable_all_edge_features_survive_warm_eval_and_real_disk_reload() {
    for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
        let (graph, edge_count) = all_edge_box_graph(kind);
        assert_eq!(edge_count, 12, "a box must materialize all twelve edges");
        let cold = evaluate_clean(&graph);
        let warm = evaluate_clean(&graph);
        assert_eq!(cold, warm);

        let document = Document::from_graph(graph, Unit::Millimeter);
        let directory = tempfile::tempdir().expect("all-edge temporary directory");
        let path = directory.path().join(format!("all-edge-{kind:?}.zcad"));
        write_document_file(
            &path,
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("save all-edge document");
        let loaded = read_document_file(&path, &LoadOptions::default())
            .expect("reload all-edge document")
            .document;
        let reloaded = evaluate_clean(loaded.evaluator_graph());
        assert_eq!(cold, reloaded);

        let persisted = loaded
            .evaluator_graph()
            .graph
            .node_indices()
            .filter_map(
                |index| match &loaded.evaluator_graph().graph[index].feature {
                    FeatureType::EdgeMod {
                        edge,
                        kind: persisted_kind,
                        ..
                    } => {
                        assert_eq!(*persisted_kind, kind);
                        Some(edge)
                    }
                    _ => None,
                },
            )
            .collect::<Vec<_>>();
        assert_eq!(persisted.len(), 1, "all edges are one atomic feature");
        let selector_id = persisted[0]
            .topology
            .as_ref()
            .and_then(|topology| topology.edge_id.as_deref())
            .expect("persisted selector id");
        let selector = AllEdgeSelector::decode(selector_id)
            .expect("reserved selector prefix")
            .expect("valid persisted selector");
        assert_eq!(selector.edge_names.len(), edge_count);
    }
}

#[test]
fn primitive_face_pair_identities_keep_both_fast_paths_reachable() {
    for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
        let (graph, edge_count) = all_edge_primitive_box_graph(kind);
        assert_eq!(edge_count, 12);
        let cold = evaluate_clean(&graph);
        let warm = evaluate_clean(&graph);
        assert_eq!(cold, warm);
    }
}

#[test]
fn changed_name_set_rejects_atomically_with_typed_reference_diagnostic() {
    let (mut graph, _) = all_edge_box_graph(CornerKind::Fillet);
    let source = sharp_source_mesh(&graph);
    let cold = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &active_cancellation(),
        )
        .expect("evaluate exact all-edge selector");
    assert!(cold.warnings.is_empty());
    let warm = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &active_cancellation(),
        )
        .expect("reuse exact all-edge selector checkpoint");
    assert!(warm.trace.evaluated_features.is_empty());
    assert!(warm
        .trace
        .reused_checkpoints
        .iter()
        .any(|feature| feature.as_str() == "fillet_all"));
    let edge_index = graph
        .graph
        .node_indices()
        .find(|index| matches!(graph.graph[*index].feature, FeatureType::EdgeMod { .. }))
        .expect("all-edge feature");
    let FeatureType::EdgeMod { edge, .. } = &mut graph.graph[edge_index].feature else {
        unreachable!("located an EdgeMod feature")
    };
    let edge_id = edge
        .topology
        .as_ref()
        .and_then(|topology| topology.edge_id.as_deref())
        .expect("selector id");
    let mut selector = AllEdgeSelector::decode(edge_id)
        .expect("selector prefix")
        .expect("valid selector");
    selector.edge_names.pop();
    edge.topology.as_mut().expect("topology").edge_id =
        Some(selector.encode().expect("encode incomplete selector"));

    let output = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &active_cancellation(),
        )
        .expect("atomic rejection remains an evaluation result");
    assert_eq!(output.bodies.len(), 1);
    assert_eq!(output.bodies[0].1.vertices, source.vertices);
    assert_eq!(output.bodies[0].1.indices, source.indices);
    assert!(output
        .trace
        .evaluated_features
        .iter()
        .any(|feature| feature.as_str() == "fillet_all"));
    assert!(output.diagnostics.iter().any(|diagnostic| {
        diagnostic.feature_id == "fillet_all"
            && diagnostic.code.as_str() == DiagnosticCode::REFERENCE_MISSING
    }));
}

#[test]
fn blocked_all_edge_kernel_candidate_reports_complete_set_and_preserves_source() {
    let (mut graph, edge_count) = all_edge_box_graph(CornerKind::Fillet);
    let source = sharp_source_mesh(&graph);
    for index in graph.graph.node_indices().collect::<Vec<_>>() {
        if let FeatureType::EdgeMod { dist, .. } = &mut graph.graph[index].feature {
            *dist = 20.0;
        }
    }

    let output = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &active_cancellation(),
        )
        .expect("blocked candidate is nonfatal");
    assert_eq!(output.bodies[0].1.vertices, source.vertices);
    assert_eq!(output.bodies[0].1.indices, source.indices);
    let warning = output
        .warnings
        .iter()
        .find(|warning| warning.contains("strict all-edge operation failed atomically"))
        .expect("atomic all-edge warning");
    assert!(warning.contains(&format!("rejected by {edge_count} edge(s)")));
    for ordinal in 0..edge_count {
        assert!(
            warning.contains(&format!("#{ordinal} (")),
            "missing blocker #{ordinal}: {warning}"
        );
    }
    assert!(output.diagnostics.iter().any(|diagnostic| {
        diagnostic.feature_id == "fillet_all"
            && diagnostic.code.as_str() == DiagnosticCode::OPERATION_FAILED
    }));
}

#[test]
fn cancelled_all_edge_request_returns_no_partial_result_and_remains_rebuildable() {
    let (graph, _) = all_edge_box_graph(CornerKind::Fillet);
    assert!(matches!(
        graph.evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &cancelled_request(),
        ),
        Err(EvaluationError::Cancelled)
    ));
    evaluate_clean(&graph);
}
