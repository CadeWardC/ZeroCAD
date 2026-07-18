use super::*;
use crate::document::{
    FeatureEditorGroup, FeatureEvaluatorKind, FeatureId, FeatureInputTarget, FeaturePayloadDecoder,
    FeatureRegistry, FeatureState, SelectorResolutionTier, SequenceKey,
};
use std::collections::HashSet;

fn add_box(graph: &mut ParametricGraph, id: &str) {
    graph.add_feature(FeatureNode {
        id: id.to_owned(),
        name: "Box".to_owned(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 8.0,
            d: 6.0,
        },
    });
}

fn add_hole(graph: &mut ParametricGraph, id: &str, target: &str) {
    graph.add_feature(FeatureNode {
        id: id.to_owned(),
        name: "Hole".to_owned(),
        feature: FeatureType::Hole {
            target: target.to_owned(),
            position: [5.0, 4.0, 6.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency(target, id);
}

#[test]
fn stable_kind_registry_covers_every_builtin_without_duplicate_ids() {
    let mut ids: Vec<_> = FeatureRegistry::BUILTINS
        .iter()
        .map(|registration| registration.kind_id)
        .collect();
    let original_len = ids.len();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), original_len);
    assert!(FeatureRegistry::get("part.extrude").is_some());
    assert!(FeatureRegistry::get("not.a.feature").is_none());
    let extrude = FeatureRegistry::get("part.extrude").unwrap();
    assert_eq!(extrude.evaluator, FeatureEvaluatorKind::Extrude);
    assert_eq!(extrude.editor_group, FeatureEditorGroup::Solid);
    for registration in FeatureRegistry::BUILTINS {
        assert!(
            registration
                .payload_decoder(registration.payload_version)
                .is_some(),
            "{} must decode its current payload schema",
            registration.kind_id
        );
        let mut schemas: Vec<_> = registration
            .payload_decoders
            .iter()
            .map(|decoder| decoder.schema)
            .collect();
        let original_len = schemas.len();
        schemas.sort_unstable();
        schemas.dedup();
        assert_eq!(
            schemas.len(),
            original_len,
            "{} has duplicate payload decoders",
            registration.kind_id
        );
    }
    assert_eq!(
        FeatureRegistry::get("exchange.step_import")
            .unwrap()
            .payload_decoder(1),
        Some(FeaturePayloadDecoder::StepAssetV1)
    );
    assert_eq!(
        FeatureRegistry::get("exchange.stl_import")
            .unwrap()
            .payload_decoder(1),
        Some(FeaturePayloadDecoder::StlAssetV1)
    );
}

#[test]
fn shared_selector_suspends_named_references_instead_of_geometric_retargeting() {
    let named = FaceRef {
        centroid: [1.0, 2.0, 3.0],
        normal: [0.0, 0.0, 1.0],
        topology: Some(TopologyFaceRef {
            body_id: Some("box_1".into()),
            component_id: Some("component".into()),
            topology_version: Some(7),
            face_id: Some("opaque-topology-name-v7".into()),
            surface_kind: Some("plane".into()),
            producer_feature_id: Some("box_1".into()),
            source_entity_id: None,
        }),
    };
    let selector = crate::document::SemanticSelector::from_face(&named);
    assert_eq!(
        selector.provenance.feature,
        Some(FeatureId::from("box_1")),
        "provenance comes from recorded metadata, not entity-id parsing"
    );
    assert!(selector.applies_to_body("box_1"));
    assert!(!selector.applies_to_body("box_2"));
    assert_eq!(
        selector.resolution_order(),
        [
            SelectorResolutionTier::DurableTopology,
            SelectorResolutionTier::OperationProvenance,
            SelectorResolutionTier::SemanticRole,
        ]
    );

    let unnamed = crate::document::SemanticSelector::from_face(&FaceRef {
        centroid: [1.0, 2.0, 3.0],
        normal: [0.0, 0.0, 1.0],
        topology: None,
    });
    assert_eq!(
        unnamed.resolution_order(),
        [SelectorResolutionTier::GeometricIntent]
    );
}

#[test]
fn explicit_sequence_replaces_numeric_id_suffix_ordering() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_999");
    graph.add_feature(FeatureNode {
        id: "cylinder_1".to_owned(),
        name: "Cylinder".to_owned(),
        feature: FeatureType::Cylinder { r: 2.0, h: 5.0 },
    });

    assert_eq!(graph.feature_sequence("box_999"), Some(SequenceKey(1)));
    assert_eq!(graph.feature_sequence("cylinder_1"), Some(SequenceKey(2)));
    let ids: Vec<_> = graph
        .evaluate_bodies(&HashSet::new())
        .unwrap()
        .into_iter()
        .map(|(id, _)| id)
        .collect();
    assert_eq!(ids, ["box_999", "cylinder_1"]);
}

#[test]
fn suppression_changes_geometry_while_visibility_only_filters_display() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    let nodes = graph.body_nodes_in_creation_order();
    let vars = graph.variable_map();
    let visible = HashSet::new();
    let hidden = HashSet::from(["box_1".to_owned()]);

    assert_eq!(
        graph.eval_prefix_keys(&nodes, &visible, &vars),
        graph.eval_prefix_keys(&nodes, &hidden, &vars),
        "visibility must not invalidate geometry checkpoints"
    );
    assert_eq!(graph.evaluate_bodies(&visible).unwrap().len(), 1);
    assert!(graph.evaluate_bodies(&hidden).unwrap().is_empty());

    assert!(graph.set_feature_suppressed("box_1", true));
    assert_eq!(graph.feature_state("box_1"), Some(FeatureState::Suppressed));
    assert!(graph.evaluate_bodies(&visible).unwrap().is_empty());
    let (_, _, statuses) = graph.evaluate_bodies_with_status(&visible).unwrap();
    assert_eq!(statuses.len(), 1);
    assert_eq!(statuses[0].state, ResolutionState::Suppressed);

    assert!(graph.set_feature_suppressed("box_1", false));
    assert_eq!(graph.evaluate_bodies(&visible).unwrap().len(), 1);
}

#[test]
fn body_timeline_and_input_roles_are_explicit() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "hole_2".to_owned(),
        name: "Hole".to_owned(),
        feature: FeatureType::Hole {
            target: "box_1".to_owned(),
            position: [5.0, 4.0, 6.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency("box_1", "hole_2");

    let body = graph
        .semantics
        .bodies
        .get(&crate::document::BodyId::from("box_1"))
        .unwrap();
    assert_eq!(
        body.timeline,
        [FeatureId::from("box_1"), FeatureId::from("hole_2")]
    );
    let hole = &graph.graph[graph.node_map["hole_2"]];
    assert!(hole.inputs.iter().any(|input| {
        input.role == "target"
            && matches!(
                &input.target,
                FeatureInputTarget::Selection(selector)
                    if selector.topology.body.as_ref().map(|body| body.as_str()) == Some("box_1")
            )
    }));
    assert!(hole.inputs.iter().any(|input| {
        input.role == "dependency"
            && input.target == FeatureInputTarget::Feature(FeatureId::from("box_1"))
    }));
}

#[test]
fn timeline_reorder_is_independent_but_never_precedes_a_dependency() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "hole_2".to_owned(),
        name: "Hole".to_owned(),
        feature: FeatureType::Hole {
            target: "box_1".to_owned(),
            position: [5.0, 4.0, 6.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency("box_1", "hole_2");

    assert!(graph.move_feature_in_timeline("hole_2", -1));
    let body = &graph.semantics.bodies[&crate::document::BodyId::from("box_1")];
    assert_eq!(
        body.timeline,
        [FeatureId::from("hole_2"), FeatureId::from("box_1")],
        "the user timeline is editable independently of the dependency DAG"
    );
    let evaluation_ids: Vec<_> = graph
        .body_nodes_in_creation_order()
        .into_iter()
        .map(|index| graph.graph[index].id.as_str())
        .collect();
    assert_eq!(evaluation_ids, ["box_1", "hole_2"]);
    graph.evaluate_bodies(&HashSet::new()).unwrap();
}

#[test]
fn semantic_validation_rejects_dependency_input_drift() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_hole(&mut graph, "hole_2", "box_1");

    let hole = graph.node_map["hole_2"];
    graph.graph[hole]
        .inputs
        .retain(|input| input.role != "dependency");

    let error = graph.validate_semantic_contracts().unwrap_err();
    assert!(error.contains("semantic inputs disagree"), "{error}");
    let evaluation_error = graph.evaluate_bodies(&HashSet::new()).unwrap_err();
    assert!(evaluation_error.contains("Invalid semantic document"));
}

#[test]
fn semantic_validation_rejects_missing_body_timeline_membership() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_hole(&mut graph, "hole_2", "box_1");

    graph
        .semantics
        .bodies
        .get_mut(&crate::document::BodyId::from("box_1"))
        .unwrap()
        .timeline
        .retain(|member| member.as_str() != "hole_2");

    let error = graph.validate_semantic_contracts().unwrap_err();
    assert!(error.contains("must appear exactly once"), "{error}");
}

#[test]
fn explicit_refresh_repairs_a_relationship_bearing_payload_edit() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_box(&mut graph, "box_2");
    add_hole(&mut graph, "hole_3", "box_1");

    let box_1 = graph.node_map["box_1"];
    let hole = graph.node_map["hole_3"];
    let old_edge = graph.graph.find_edge(box_1, hole).unwrap();
    graph.graph.remove_edge(old_edge);
    if let FeatureType::Hole { target, .. } = &mut graph.graph[hole].feature {
        *target = "box_2".to_owned();
    }
    let box_2 = graph.node_map["box_2"];
    graph.graph.add_edge(box_2, hole, ());

    assert!(graph.validate_semantic_contracts().is_err());
    graph.commit_feature_edit("hole_3").unwrap();
    graph.validate_semantic_contracts().unwrap();
    let body = &graph.semantics.bodies[&crate::document::BodyId::from("box_2")];
    assert!(body
        .timeline
        .iter()
        .any(|member| member.as_str() == "hole_3"));
}

#[test]
fn legacy_body_index_backfill_preserves_authoritative_inputs() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_hole(&mut graph, "hole_2", "box_1");

    graph.semantics = crate::document::DocumentSemantics::default();
    graph.rebuild_node_map();
    graph.validate_semantic_contracts().unwrap();

    let hole = &graph.graph[graph.node_map["hole_2"]];
    assert!(hole.inputs.iter().any(|input| {
        input.role == "dependency"
            && input.target == FeatureInputTarget::Feature(FeatureId::from("box_1"))
    }));
}
