//! Shell as a parametric feature: hollow a body with selected faces removed.

use super::*;
use crate::parametric::{FaceRef, TopologyFaceRef};

fn face_ref(centroid: [f32; 3], normal: [f32; 3]) -> FaceRef {
    FaceRef {
        centroid,
        normal,
        topology: None,
    }
}

#[test]
fn shell_box_open_top_matches_cup_volume() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    g.add_feature(FeatureNode {
        id: "shell_2".to_string(),
        name: "Shell".to_string(),
        feature: FeatureType::Shell {
            target: "box_1".to_string(),
            thickness: 1.0,
            thickness_expr: None,
            open_faces: vec![face_ref([5.0, 5.0, 10.0], [0.0, 0.0, 1.0])],
        },
    });
    g.add_dependency("box_1", "shell_2");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mp = bodies[0].1.mass_properties().expect("closed cup");
    // 10³ cup, 1mm walls, open top: void = 8×8×9.
    let expected = 1000.0 - 8.0 * 8.0 * 9.0;
    assert!(
        (mp.volume - expected).abs() / expected < 0.01,
        "cup volume {} vs {expected}",
        mp.volume
    );
}

#[test]
fn shell_without_open_faces_creates_closed_box_cavity() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    g.add_feature(FeatureNode {
        id: "shell_2".to_string(),
        name: "Shell".to_string(),
        feature: FeatureType::Shell {
            target: "box_1".to_string(),
            thickness: 1.0,
            thickness_expr: None,
            open_faces: vec![],
        },
    });
    g.add_dependency("box_1", "shell_2");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert_eq!(output.bodies.len(), 1);
    assert!(
        output.diagnostics.is_empty(),
        "diagnostics: {:?}",
        output.diagnostics
    );
    let mp = output.bodies[0].1.mass_properties().unwrap();
    // Closed 10Â³ box with a 1 mm wall: subtract the enclosed 8Â³ cavity.
    let expected = 1000.0 - 8.0 * 8.0 * 8.0;
    assert!(
        (mp.volume - expected).abs() / expected < 0.01,
        "closed shell volume {} vs {expected}",
        mp.volume
    );
}

fn captured_face(face: &crate::mock_kernel::MeshFaceRef) -> FaceRef {
    FaceRef {
        centroid: face.centroid,
        normal: face.normal,
        topology: face.topology.as_ref().map(|topology| TopologyFaceRef {
            body_id: topology.body_id.clone(),
            component_id: topology.component_id.clone(),
            topology_version: topology.topology_version,
            face_id: topology.face_id.clone(),
            surface_kind: topology.surface_kind.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    }
}

#[test]
fn shell_candidate_contract_cold_warm_cancel_and_restore() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_contract".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 8.0,
            h: 8.0,
            d: 8.0,
        },
    });
    g.add_feature(FeatureNode {
        id: "shell_contract".into(),
        name: "Shell".into(),
        feature: FeatureType::Shell {
            target: "box_contract".into(),
            thickness: 1.0,
            thickness_expr: None,
            open_faces: vec![face_ref([4.0, 4.0, 8.0], [0.0, 0.0, 1.0])],
        },
    });
    g.add_dependency("box_contract", "shell_contract");
    let hidden = std::collections::HashSet::new();
    let cold = g.evaluate_bodies_with_warnings(&hidden).unwrap();
    let warm = g.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty() && warm.1.is_empty());
    assert_eq!(cold.0[0].1.indices, warm.0[0].1.indices);
    assert_eq!(cold.0[0].1.face_ids, warm.0[0].1.face_ids);
    let cancelled = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        g.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled),
        ),
        Err(EvaluationError::Cancelled)
    ));
    let restored: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&g.clone_document()).unwrap()).unwrap();
    assert!(restored
        .evaluate_bodies_with_warnings(&hidden)
        .unwrap()
        .1
        .is_empty());
}

#[test]
fn shell_preserves_retained_face_names_and_names_generated_faces() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_named".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 8.0,
            d: 6.0,
        },
    });
    let initial = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("initial named box");
    let top = initial[0]
        .1
        .face_refs
        .iter()
        .filter(|face| face.normal[2] > 0.9)
        .max_by(|first, second| first.centroid[2].total_cmp(&second.centroid[2]))
        .map(captured_face)
        .expect("named top face");
    let bottom = initial[0]
        .1
        .face_refs
        .iter()
        .filter(|face| face.normal[2] < -0.9)
        .min_by(|first, second| first.centroid[2].total_cmp(&second.centroid[2]))
        .map(captured_face)
        .expect("named bottom face");
    let bottom_name = bottom
        .topology
        .as_ref()
        .and_then(|topology| topology.face_id.clone())
        .expect("bottom face has a durable name");

    graph.add_feature(FeatureNode {
        id: "shell_named".into(),
        name: "Shell".into(),
        feature: FeatureType::Shell {
            target: "box_named".into(),
            thickness: 0.75,
            thickness_expr: None,
            open_faces: vec![top],
        },
    });
    graph.add_dependency("box_named", "shell_named");

    let (live, warnings) = graph
        .build_live(&std::collections::HashSet::new(), false)
        .expect("named Shell live state");
    assert!(warnings.is_empty(), "Shell warnings: {warnings:?}");
    let resolved = resolve_face_ref_by_topology(&live[0], &bottom)
        .expect("retained bottom face reattaches through Shell");
    assert_eq!(
        resolved
            .topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref()),
        Some(bottom_name.as_str())
    );

    let mesh = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("named Shell mesh")
        .remove(0)
        .1;
    assert!(mesh.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| super::super::topo_name::TopoName::parse(name).is_durable())
    }));
    assert!(mesh.face_refs.iter().any(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| name.starts_with("shell:shell_named:face:"))
    }));

    let restored: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&graph.clone_document()).unwrap()).unwrap();
    let restored_mesh = restored
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("restored named Shell")
        .remove(0)
        .1;
    assert!(restored_mesh.face_refs.iter().any(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            == Some(bottom_name.as_str())
    }));
}

#[test]
fn shell_does_not_geometrically_substitute_a_missing_durable_face() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_missing_name".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "shell_missing_name".into(),
        name: "Shell".into(),
        feature: FeatureType::Shell {
            target: "box_missing_name".into(),
            thickness: 1.0,
            thickness_expr: None,
            open_faces: vec![FaceRef {
                centroid: [5.0, 5.0, 10.0],
                normal: [0.0, 0.0, 1.0],
                topology: Some(TopologyFaceRef {
                    body_id: Some("box_missing_name".into()),
                    face_id: Some("box:box_missing_name:face:missing".into()),
                    ..TopologyFaceRef::default()
                }),
            }],
        },
    });
    graph.add_dependency("box_missing_name", "shell_missing_name");

    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(
                1,
                std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1)),
            ),
        )
        .expect("missing-name evaluation remains atomic");
    assert!(output.diagnostics.iter().any(|diagnostic| {
        diagnostic.feature_id == "shell_missing_name"
            && diagnostic.code.as_str() == DiagnosticCode::REFERENCE_MISSING
    }));
    let mass = output.bodies[0].1.mass_properties().expect("original box");
    assert!((mass.volume - 1000.0).abs() < 1.0e-3);
}
