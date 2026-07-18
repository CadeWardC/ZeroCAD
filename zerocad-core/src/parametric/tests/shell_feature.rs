//! Shell as a parametric feature: hollow a body with selected faces removed.

use super::*;
use crate::parametric::FaceRef;

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
fn shell_without_open_faces_warns() {
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
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "shell_2"
                && diagnostic.code.as_str() == DiagnosticCode::FEATURE_UNRESOLVED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
    // Box untouched.
    let mp = output.bodies[0].1.mass_properties().unwrap();
    assert!((mp.volume - 1000.0).abs() < 1e-3);
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
