use super::*;
use crate::document::{
    FeatureEditorGroup, FeatureEvaluatorKind, FeatureId, FeatureInputTarget, FeatureRegistry,
    FeatureState, SelectorResolutionTier, SequenceKey,
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
    assert_eq!(extrude.evaluator, FeatureEvaluatorKind::BodyOperation);
    assert_eq!(extrude.editor_group, FeatureEditorGroup::Solid);
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
    let hole = graph
        .semantics
        .features
        .get(&FeatureId::from("hole_2"))
        .unwrap();
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
