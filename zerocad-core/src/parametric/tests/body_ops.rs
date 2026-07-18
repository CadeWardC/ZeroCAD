use super::*;

fn add_box(graph: &mut ParametricGraph, id: &str) {
    graph.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
}

fn add_shifted_copy(graph: &mut ParametricGraph, offset: f32) {
    graph.add_feature(FeatureNode {
        id: "copy_2".to_string(),
        name: "Tool".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [offset, 0.0, 0.0],
            copy: true,
        },
    });
    graph.add_dependency("box_1", "copy_2");
}

#[test]
fn intersect_keeps_only_common_volume_and_consumes_inputs() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_shifted_copy(&mut graph, 5.0);
    graph.add_feature(FeatureNode {
        id: "intersect_3".to_string(),
        name: "Intersect".to_string(),
        feature: FeatureType::BodyIntersect {
            target: "box_1".to_string(),
            tool: "copy_2".to_string(),
            keep_tool: false,
        },
    });
    graph.add_dependency("box_1", "intersect_3");
    graph.add_dependency("copy_2", "intersect_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(
        bodies.len(),
        1,
        "bodies={:?}",
        bodies
            .iter()
            .map(|body| body.0.as_str())
            .collect::<Vec<_>>()
    );
    assert_eq!(bodies[0].0, "intersect_3");
    let volume = bodies[0].1.mass_properties().unwrap().volume;
    assert!((volume - 500.0).abs() < 1.0, "volume={volume}");
}

#[test]
fn empty_intersect_is_atomic() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_shifted_copy(&mut graph, 10.0);
    graph.add_feature(FeatureNode {
        id: "intersect_3".to_string(),
        name: "Intersect".to_string(),
        feature: FeatureType::BodyIntersect {
            target: "box_1".to_string(),
            tool: "copy_2".to_string(),
            keep_tool: false,
        },
    });
    graph.add_dependency("box_1", "intersect_3");
    graph.add_dependency("copy_2", "intersect_3");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 2);
    assert!(bodies.iter().any(|(id, _)| id == "box_1"));
    assert!(bodies.iter().any(|(id, _)| id == "copy_2"));
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("positive-volume")));
}

#[test]
fn intersect_handles_box_cylinder_containment_and_keep_tool() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "cylinder_2".into(),
        name: "Cylinder".into(),
        feature: FeatureType::Cylinder { r: 2.0, h: 10.0 },
    });
    graph.add_feature(FeatureNode {
        id: "move_3".into(),
        name: "Center cylinder".into(),
        feature: FeatureType::BodyTransform {
            source: "cylinder_2".into(),
            translation: [5.0, 0.0, 5.0],
            copy: false,
        },
    });
    graph.add_dependency("cylinder_2", "move_3");
    graph.add_feature(FeatureNode {
        id: "intersect_4".into(),
        name: "Intersect".into(),
        feature: FeatureType::BodyIntersect {
            target: "box_1".into(),
            tool: "move_3".into(),
            keep_tool: true,
        },
    });
    graph.add_dependency("box_1", "intersect_4");
    graph.add_dependency("move_3", "intersect_4");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    assert!(bodies.iter().any(|body| body.0 == "move_3"));
    let common = &bodies
        .iter()
        .find(|body| body.0 == "intersect_4")
        .expect("common result")
        .1;
    let volume = common.mass_properties().unwrap().volume;
    assert!(
        (volume - std::f64::consts::PI * 40.0).abs() < 1.0,
        "volume={volume}"
    );
}

#[test]
fn intersect_commits_all_overlapping_multipart_pairs_atomically() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "pattern_2".into(),
        name: "Two boxes".into(),
        feature: FeatureType::Pattern {
            source: "box_1".into(),
            kind: PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 20.0,
                spacing_expr: None,
                count: 3,
            },
        },
    });
    graph.add_dependency("box_1", "pattern_2");
    graph.add_feature(FeatureNode {
        id: "tool_3".into(),
        name: "Covering tool".into(),
        feature: FeatureType::Box {
            w: 50.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "intersect_4".into(),
        name: "Multipart intersect".into(),
        feature: FeatureType::BodyIntersect {
            target: "pattern_2".into(),
            tool: "tool_3".into(),
            keep_tool: false,
        },
    });
    graph.add_dependency("pattern_2", "intersect_4");
    graph.add_dependency("tool_3", "intersect_4");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(
        bodies.len(),
        2,
        "bodies={:?}",
        bodies
            .iter()
            .map(|body| body.0.as_str())
            .collect::<Vec<_>>()
    );
    assert!(bodies.iter().any(|body| body.0 == "box_1"));
    let volume = bodies
        .iter()
        .find(|body| body.0 == "intersect_4")
        .expect("multipart Common")
        .1
        .mass_properties()
        .unwrap()
        .volume;
    assert!((volume - 2_000.0).abs() < 2.0, "volume={volume}");
}

#[test]
fn split_emits_two_registered_stable_outputs() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "datum_2".to_string(),
        name: "Mid plane".to_string(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "split_3".to_string(),
        name: "Split".to_string(),
        feature: FeatureType::BodySplit {
            target: "box_1".to_string(),
            plane: PlaneBase::Datum("datum_2".to_string()),
            face: None,
        },
    });
    graph.add_dependency("box_1", "split_3");
    graph.add_dependency("datum_2", "split_3");
    assert_eq!(
        graph.body_producer_feature_id("split_3::body:2"),
        Some("split_3")
    );

    let (mut bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    bodies.sort_by(|left, right| left.0.cmp(&right.0));
    assert_eq!(
        bodies
            .iter()
            .map(|body| body.0.as_str())
            .collect::<Vec<_>>(),
        vec!["split_3", "split_3::body:2"]
    );
    for (_, mesh) in bodies {
        let volume = mesh.mass_properties().unwrap().volume;
        assert!((volume - 500.0).abs() < 1.0, "volume={volume}");
    }
}

#[test]
fn split_accepts_a_planar_face_from_an_explicit_reference_body() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    add_shifted_copy(&mut graph, 5.0);
    graph.add_feature(FeatureNode {
        id: "split_3".into(),
        name: "Split by face".into(),
        feature: FeatureType::BodySplit {
            target: "box_1".into(),
            plane: PlaneBase::XY,
            face: Some(FaceRef {
                centroid: [5.0, 5.0, 5.0],
                normal: [-1.0, 0.0, 0.0],
                topology: Some(TopologyFaceRef {
                    body_id: Some("copy_2".into()),
                    producer_feature_id: Some("copy_2".into()),
                    ..TopologyFaceRef::default()
                }),
            }),
        },
    });
    graph.add_dependency("box_1", "split_3");
    graph.add_dependency("copy_2", "split_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 3, "two split sides plus the reference body");
    for id in ["split_3", "split_3::body:2"] {
        let mesh = &bodies.iter().find(|body| body.0 == id).unwrap().1;
        assert!((mesh.mass_properties().unwrap().volume - 500.0).abs() < 1.0);
    }
}

#[test]
fn split_supports_a_rotated_datum_plane() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "datum_2".into(),
        name: "Diagonal plane".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::ThreePoints {
                a: [0.0, 0.0, 0.0],
                b: [10.0, 10.0, 0.0],
                c: [0.0, 0.0, 10.0],
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "split_3".into(),
        name: "Diagonal split".into(),
        feature: FeatureType::BodySplit {
            target: "box_1".into(),
            plane: PlaneBase::Datum("datum_2".into()),
            face: None,
        },
    });
    graph.add_dependency("box_1", "split_3");
    graph.add_dependency("datum_2", "split_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    let total = bodies
        .iter()
        .map(|body| body.1.mass_properties().unwrap().volume)
        .sum::<f64>();
    assert!((total - 1_000.0).abs() < 2.0, "total volume={total}");
}

#[test]
fn split_outputs_accept_independent_downstream_features() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "datum_2".into(),
        name: "Mid plane".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "split_3".into(),
        name: "Split".into(),
        feature: FeatureType::BodySplit {
            target: "box_1".into(),
            plane: PlaneBase::Datum("datum_2".into()),
            face: None,
        },
    });
    graph.add_dependency("box_1", "split_3");
    graph.add_dependency("datum_2", "split_3");
    graph.add_feature(FeatureNode {
        id: "scale_4".into(),
        name: "Scale positive side".into(),
        feature: FeatureType::BodyScale {
            source: "split_3::body:2".into(),
            factor: 0.5,
            factor_expr: None,
            center: [5.0, 5.0, 7.5],
        },
    });
    graph.add_dependency("split_3::body:2", "scale_4");

    let (mut bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    bodies.sort_by(|left, right| left.0.cmp(&right.0));
    assert_eq!(
        bodies
            .iter()
            .map(|body| body.0.as_str())
            .collect::<Vec<_>>(),
        ["scale_4", "split_3"]
    );
    let scaled_volume = bodies
        .iter()
        .find(|body| body.0 == "scale_4")
        .unwrap()
        .1
        .mass_properties()
        .unwrap()
        .volume;
    assert!((scaled_volume - 62.5).abs() < 0.5, "volume={scaled_volume}");
}

#[test]
fn uniform_scale_uses_persisted_pivot_and_expression() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1");
    graph.add_feature(FeatureNode {
        id: "variables_2".to_string(),
        name: "Variables".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "s".to_string(),
                value: 2.0,
                unit: crate::Unit::Millimeter,
                expression: None,
            }],
        },
    });
    graph.add_feature(FeatureNode {
        id: "scale_3".to_string(),
        name: "Scale".to_string(),
        feature: FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: 1.5,
            factor_expr: Some("s".to_string()),
            center: [5.0, 5.0, 5.0],
        },
    });
    graph.add_dependency("box_1", "scale_3");
    graph.add_dependency("variables_2", "scale_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;
    let volume = mesh.mass_properties().unwrap().volume;
    assert!((volume - 8_000.0).abs() < 2.0, "volume={volume}");
    assert_eq!(body_bounds_center(mesh).unwrap(), [5.0, 5.0, 5.0]);
}

#[test]
fn invalid_scale_factor_leaves_source_body_visible() {
    for factor in [0.0, -1.0, 1.0e-9, f32::NAN, f32::INFINITY] {
        let mut graph = ParametricGraph::new();
        add_box(&mut graph, "box_1");
        graph.add_feature(FeatureNode {
            id: "scale_2".to_string(),
            name: "Scale".to_string(),
            feature: FeatureType::BodyScale {
                source: "box_1".to_string(),
                factor,
                factor_expr: None,
                center: [0.0; 3],
            },
        });
        graph.add_dependency("box_1", "scale_2");
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "factor={factor:?}");
        assert_eq!(bodies[0].0, "box_1", "factor={factor:?}");
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains("greater than zero")),
            "factor={factor:?}, warnings={warnings:?}"
        );
    }
}

#[test]
fn suppression_restores_inputs_for_every_phase35_feature() {
    let hidden = std::collections::HashSet::new();

    let mut intersect = ParametricGraph::new();
    add_box(&mut intersect, "box_1");
    add_shifted_copy(&mut intersect, 5.0);
    intersect.add_feature(FeatureNode {
        id: "intersect_3".into(),
        name: "Intersect".into(),
        feature: FeatureType::BodyIntersect {
            target: "box_1".into(),
            tool: "copy_2".into(),
            keep_tool: false,
        },
    });
    intersect.add_dependency("box_1", "intersect_3");
    intersect.add_dependency("copy_2", "intersect_3");
    assert!(intersect.set_feature_suppressed("intersect_3", true));
    let mut ids = intersect
        .evaluate_bodies(&hidden)
        .unwrap()
        .into_iter()
        .map(|body| body.0)
        .collect::<Vec<_>>();
    ids.sort();
    assert_eq!(ids, ["box_1", "copy_2"]);

    let mut split = ParametricGraph::new();
    add_box(&mut split, "box_1");
    split.add_feature(FeatureNode {
        id: "split_2".into(),
        name: "Split".into(),
        feature: FeatureType::BodySplit {
            target: "box_1".into(),
            plane: PlaneBase::XY,
            face: None,
        },
    });
    split.add_dependency("box_1", "split_2");
    assert!(split.set_feature_suppressed("split_2", true));
    let ids = split
        .evaluate_bodies(&hidden)
        .unwrap()
        .into_iter()
        .map(|body| body.0)
        .collect::<Vec<_>>();
    assert_eq!(ids, ["box_1"]);
    assert!(!ids.iter().any(|id| id == "split_2::body:2"));

    let mut scale = ParametricGraph::new();
    add_box(&mut scale, "box_1");
    scale.add_feature(FeatureNode {
        id: "scale_2".into(),
        name: "Scale".into(),
        feature: FeatureType::BodyScale {
            source: "box_1".into(),
            factor: 2.0,
            factor_expr: None,
            center: [5.0; 3],
        },
    });
    scale.add_dependency("box_1", "scale_2");
    assert!(scale.set_feature_suppressed("scale_2", true));
    let ids = scale
        .evaluate_bodies(&hidden)
        .unwrap()
        .into_iter()
        .map(|body| body.0)
        .collect::<Vec<_>>();
    assert_eq!(ids, ["box_1"]);
}
