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
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1);
    assert!(
        warnings.iter().any(|w| w.contains("shell_2")),
        "warnings: {warnings:?}"
    );
    // Box untouched.
    let mp = bodies[0].1.mass_properties().unwrap();
    assert!((mp.volume - 1000.0).abs() < 1e-3);
}
