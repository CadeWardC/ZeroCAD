use super::*;
use crate::parametric::{
    AxisBase, FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind, HoleKind,
    PatternKind,
};

fn add_box(graph: &mut ParametricGraph) {
    add_box_named(graph, "plate", 30.0, 20.0, 8.0);
}

fn add_box_named(graph: &mut ParametricGraph, id: &str, w: f32, h: f32, d: f32) {
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Box { w, h, d },
    });
}

fn add_source_hole(graph: &mut ParametricGraph, depth: Option<f32>) {
    graph.add_feature(FeatureNode {
        id: "hole_source".into(),
        name: "Source Hole".into(),
        feature: FeatureType::Hole {
            target: "plate".into(),
            position: [5.0, 10.0, 8.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency("plate", "hole_source");
}

fn add_linear_pattern(
    graph: &mut ParametricGraph,
    count: u32,
    spacing: f32,
    extent_policy: FeaturePatternExtentPolicy,
) {
    add_linear_pattern_for(
        graph,
        "feature_pattern",
        "plate",
        "hole_source",
        count,
        spacing,
        FeaturePatternComputeMode::Identical,
        extent_policy,
        true,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_linear_pattern_for(
    graph: &mut ParametricGraph,
    id: &str,
    target: &str,
    source_feature: &str,
    count: u32,
    spacing: f32,
    compute_mode: FeaturePatternComputeMode,
    extent_policy: FeaturePatternExtentPolicy,
    connect_source: bool,
) {
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::FeaturePattern {
            target: target.into(),
            source_feature: source_feature.into(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::X,
                spacing,
                spacing_expr: None,
                count,
            },
            compute_mode,
            extent_policy,
        },
    });
    if connect_source {
        graph.add_dependency(source_feature, id);
    }
}

fn plate_volume(bodies: &[(String, MockMesh)]) -> f64 {
    body_volume(bodies, "plate")
}

fn body_volume(bodies: &[(String, MockMesh)], id: &str) -> f64 {
    bodies
        .iter()
        .find(|(body_id, _)| body_id == id)
        .and_then(|(_, mesh)| mesh.mass_properties())
        .unwrap_or_else(|| panic!("{id} must remain one closed body"))
        .volume
}

fn live_cancellation() -> EvaluationCancellation {
    EvaluationCancellation::new(1, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1)))
}

fn set_pattern_count(graph: &mut ParametricGraph, id: &str, new_count: u32) {
    let index = graph.node_map[id];
    let FeatureType::FeaturePattern { kind, .. } = &mut graph.graph[index].feature else {
        panic!("{id} must be a feature pattern");
    };
    match kind {
        FeaturePatternKind::Linear { count, .. } | FeaturePatternKind::Circular { count, .. } => {
            *count = new_count
        }
    }
}

fn add_new_body_extrude_source(graph: &mut ParametricGraph, id: &str) {
    let sketch_id = format!("{id}_sketch");
    add_sketch(graph, &sketch_id, rect_sketch((40.0, 0.0), (44.0, 4.0)));
    add_extrude(graph, id, &sketch_id, 4.0, ExtrudeMode::NewBody);
}

fn add_new_body_revolve_source(graph: &mut ParametricGraph, id: &str) {
    let sketch_id = format!("{id}_sketch");
    add_sketch(graph, &sketch_id, rect_sketch((1.0, 0.0), (2.0, 3.0)));
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Revolve {
            axis: AxisBase::Y,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency(&sketch_id, id);
}

fn add_cut_extrude_source(graph: &mut ParametricGraph, id: &str) {
    let sketch_id = format!("{id}_sketch");
    let mut curves = SketchCurves::new();
    curves.add_circle((5.0, 10.0), 2.0);
    add_sketch_cs(
        graph,
        &sketch_id,
        CoordinateSystem::new(
            crate::geometry::Vec3::new(0.0, 0.0, 0.0),
            crate::geometry::Vec3::X,
            crate::geometry::Vec3::Y,
        ),
        curves,
    );
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude {
            target: Some("plate".into()),
            depth: 3.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency(&sketch_id, id);
    graph.add_dependency("plate", id);
}

fn joined_source_pattern_graph(with_witness: bool) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_cut_extrude_source(&mut graph, "join_source");
    if with_witness {
        add_box_named(&mut graph, "witness", 2.0, 2.0, 2.0);
    }
    add_linear_pattern_for(
        &mut graph,
        "feature_pattern",
        "plate",
        "join_source",
        2,
        10.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        true,
    );
    graph
}

fn stepped_target_pattern_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    let upper_cs = CoordinateSystem::new(
        crate::geometry::Vec3::new(0.0, 0.0, 4.0),
        crate::geometry::Vec3::X,
        crate::geometry::Vec3::Y,
    );
    add_sketch_cs(
        &mut graph,
        "upper_slab_sketch",
        upper_cs,
        rect_sketch((0.0, 0.0), (30.0, 20.0)),
    );
    add_extrude(
        &mut graph,
        "plate",
        "upper_slab_sketch",
        4.0,
        ExtrudeMode::NewBody,
    );

    add_sketch(
        &mut graph,
        "lower_step_sketch",
        rect_sketch((10.0, 0.0), (30.0, 20.0)),
    );
    graph.add_feature(FeatureNode {
        id: "lower_step".into(),
        name: "Lower step".into(),
        feature: FeatureType::Extrude {
            target: Some("plate".into()),
            depth: 4.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("lower_step_sketch", "lower_step");
    graph.add_dependency("plate", "lower_step");
    add_source_hole(&mut graph, None);
    add_linear_pattern(
        &mut graph,
        3,
        10.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    graph
}

fn body_checkpoint_position_after(graph: &ParametricGraph, feature_id: &str) -> usize {
    graph
        .body_nodes_in_creation_order()
        .iter()
        .position(|index| graph.graph[*index].id == feature_id)
        .unwrap_or_else(|| panic!("missing checkpoint after {feature_id}"))
}

fn remove_body_from_checkpoint(graph: &ParametricGraph, checkpoint_index: usize, body_id: &str) {
    let mut cache = graph.eval_cache.borrow_mut();
    let cache = std::sync::Arc::make_mut(&mut *cache);
    let checkpoint = cache.checkpoints[checkpoint_index]
        .as_mut()
        .expect("checkpoint must be populated");
    checkpoint.live.retain(|body| body.id != body_id);
}

#[test]
fn through_all_hole_pattern_recomputes_each_local_cut_and_reuses_warm_checkpoint() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_source_hole(&mut graph, None);
    add_linear_pattern(
        &mut graph,
        3,
        8.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );

    let hidden = std::collections::HashSet::new();
    let (cold, cold_warnings) = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let (warm, warm_warnings) = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold_warnings.is_empty(), "cold warnings: {cold_warnings:?}");
    assert!(warm_warnings.is_empty(), "warm warnings: {warm_warnings:?}");
    let expected = 30.0 * 20.0 * 8.0 - 3.0 * std::f64::consts::PI * 8.0;
    let cold_volume = plate_volume(&cold);
    assert!(
        (cold_volume - expected).abs() / expected < 0.01,
        "three-hole volume {cold_volume} vs {expected}"
    );
    assert!((plate_volume(&warm) - cold_volume).abs() < 1.0e-6);

    let mesh = &cold.iter().find(|(id, _)| id == "plate").unwrap().1;
    let names: Vec<_> = mesh
        .face_refs
        .iter()
        .filter_map(|face| face.topology.as_ref())
        .filter_map(|topology| topology.face_id.clone())
        .collect();
    let faces: Vec<_> = mesh
        .face_refs
        .iter()
        .map(|face| {
            (
                face.centroid,
                face.normal,
                face.topology
                    .as_ref()
                    .and_then(|topology| topology.surface_kind.clone()),
                face.topology
                    .as_ref()
                    .and_then(|topology| topology.face_id.clone()),
            )
        })
        .collect();
    for instance in [1, 2] {
        let expected =
            format!("pattern:feature_pattern:instance:{instance}:source:cut:hole_source:tool-face");
        assert!(
            names.iter().any(|name| name.contains(&expected)),
            "missing {expected}; durable face names: {names:?}; faces: {faces:?}"
        );
    }
}

#[test]
fn count_one_is_the_source_only_and_count_zero_is_unresolved() {
    let mut one = ParametricGraph::new();
    add_box(&mut one);
    add_source_hole(&mut one, Some(4.0));
    add_linear_pattern(&mut one, 1, 8.0, FeaturePatternExtentPolicy::SourceExtent);
    let (one_body, one_warnings) = one
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(one_warnings.is_empty(), "warnings: {one_warnings:?}");

    let mut source_only = ParametricGraph::new();
    add_box(&mut source_only);
    add_source_hole(&mut source_only, Some(4.0));
    let source_body = source_only
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!((plate_volume(&one_body) - plate_volume(&source_body)).abs() < 1.0e-6);

    let mut zero = ParametricGraph::new();
    add_box(&mut zero);
    add_source_hole(&mut zero, Some(4.0));
    add_linear_pattern(&mut zero, 0, 8.0, FeaturePatternExtentPolicy::SourceExtent);
    let (zero_body, zero_warnings) = zero
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(zero_warnings
        .iter()
        .any(|warning| warning.contains("must be at least 1")));
    assert!((plate_volume(&zero_body) - plate_volume(&source_body)).abs() < 1.0e-6);
}

#[test]
fn a_missed_later_instance_rolls_back_the_entire_pattern() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_source_hole(&mut graph, None);
    // x=15 succeeds, x=25 also succeeds actually; use x=35 for the third
    // additional occurrence so the final one misses the 30 mm plate.
    add_linear_pattern(
        &mut graph,
        4,
        10.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    let (body, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("entire pattern was rolled back")));

    let mut source_only = ParametricGraph::new();
    add_box(&mut source_only);
    add_source_hole(&mut source_only, None);
    let expected = source_only
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!((plate_volume(&body) - plate_volume(&expected)).abs() < 1.0e-6);
}

#[test]
fn suppressed_source_suspends_the_pattern_without_guessing() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_source_hole(&mut graph, None);
    add_linear_pattern(
        &mut graph,
        3,
        8.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    assert!(graph.set_feature_suppressed("hole_source", true));
    let (body, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("suppressed or unresolved")));
    assert!((plate_volume(&body) - 30.0 * 20.0 * 8.0).abs() < 1.0e-3);
}

#[test]
fn cancelled_feature_pattern_request_publishes_no_candidate() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_source_hole(&mut graph, None);
    add_linear_pattern(
        &mut graph,
        3,
        8.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    let superseded = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        graph.evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, superseded),
        ),
        Err(EvaluationError::Cancelled)
    ));
}

#[test]
fn invalid_source_feature_families_are_rejected_without_touching_the_target() {
    let mut cases = Vec::new();

    let mut box_source = ParametricGraph::new();
    add_box(&mut box_source);
    add_box_named(&mut box_source, "source_box", 4.0, 4.0, 4.0);
    add_linear_pattern_for(
        &mut box_source,
        "feature_pattern",
        "plate",
        "source_box",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        true,
    );
    cases.push(("Box", box_source, "v1 sources must be"));

    let mut body_pattern_source = ParametricGraph::new();
    add_box(&mut body_pattern_source);
    add_box_named(&mut body_pattern_source, "pattern_seed", 2.0, 2.0, 2.0);
    body_pattern_source.add_feature(FeatureNode {
        id: "body_pattern_source".into(),
        name: "Body pattern source".into(),
        feature: FeatureType::Pattern {
            source: "pattern_seed".into(),
            kind: PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 6.0,
                spacing_expr: None,
                count: 2,
            },
        },
    });
    body_pattern_source.add_dependency("pattern_seed", "body_pattern_source");
    add_linear_pattern_for(
        &mut body_pattern_source,
        "feature_pattern",
        "plate",
        "body_pattern_source",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        true,
    );
    cases.push(("body Pattern", body_pattern_source, "v1 sources must be"));

    let mut sketch_source = ParametricGraph::new();
    add_box(&mut sketch_source);
    add_sketch(
        &mut sketch_source,
        "source_sketch",
        rect_sketch((0.0, 0.0), (2.0, 2.0)),
    );
    add_linear_pattern_for(
        &mut sketch_source,
        "feature_pattern",
        "plate",
        "source_sketch",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        true,
    );
    cases.push(("Sketch", sketch_source, "not a material feature"));

    for (label, graph, expected_warning) in cases {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap_or_else(|error| panic!("{label} source evaluation failed: {error}"));
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains(expected_warning)),
            "{label} source warnings: {warnings:?}"
        );
        assert!(
            (plate_volume(&bodies) - 30.0 * 20.0 * 8.0).abs() < 1.0e-3,
            "{label} source changed the pattern target"
        );
    }
}

#[test]
fn extent_policy_mismatches_are_rejected_atomically() {
    for (label, depth, extent_policy, expected_warning) in [
        (
            "through-all",
            None,
            FeaturePatternExtentPolicy::SourceExtent,
            "through-all Hole requires ThroughAllLocalTarget",
        ),
        (
            "blind",
            Some(4.0),
            FeaturePatternExtentPolicy::ThroughAllLocalTarget,
            "blind Hole requires SourceExtent",
        ),
    ] {
        let mut graph = ParametricGraph::new();
        add_box(&mut graph);
        add_source_hole(&mut graph, depth);
        add_linear_pattern(&mut graph, 2, 8.0, extent_policy);
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains(expected_warning)),
            "{label} warnings: {warnings:?}"
        );

        let mut source_only = ParametricGraph::new();
        add_box(&mut source_only);
        add_source_hole(&mut source_only, depth);
        let expected = source_only
            .evaluate_bodies(&Default::default())
            .expect("source-only reference");
        assert!((plate_volume(&bodies) - plate_volume(&expected)).abs() < 1.0e-6);
    }
}

#[test]
fn feature_pattern_self_and_indirect_cycles_fail_graph_validation() {
    let mut direct = ParametricGraph::new();
    add_box(&mut direct);
    add_linear_pattern_for(
        &mut direct,
        "self_pattern",
        "plate",
        "self_pattern",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        true,
    );
    let direct_error = direct
        .evaluate_bodies_with_warnings(&Default::default())
        .expect_err("self-reference must be cyclic");
    assert!(
        direct_error.contains("Circular dependency"),
        "{direct_error}"
    );

    let mut indirect = ParametricGraph::new();
    add_box(&mut indirect);
    add_linear_pattern_for(
        &mut indirect,
        "pattern_a",
        "plate",
        "pattern_b",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        false,
    );
    add_linear_pattern_for(
        &mut indirect,
        "pattern_b",
        "plate",
        "pattern_a",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::SourceExtent,
        false,
    );
    indirect.add_dependency("pattern_a", "pattern_b");
    indirect.add_dependency("pattern_b", "pattern_a");
    let indirect_error = indirect
        .evaluate_bodies_with_warnings(&Default::default())
        .expect_err("indirect feature-pattern cycle must fail");
    assert!(
        indirect_error.contains("Circular dependency"),
        "{indirect_error}"
    );
}

#[test]
fn new_body_sources_and_cross_body_holes_are_rejected() {
    for (label, mut graph, source_id) in {
        let mut extrude = ParametricGraph::new();
        add_box(&mut extrude);
        add_new_body_extrude_source(&mut extrude, "new_body_extrude");

        let mut revolve = ParametricGraph::new();
        add_box(&mut revolve);
        add_new_body_revolve_source(&mut revolve, "new_body_revolve");

        vec![
            ("NewBody Extrude", extrude, "new_body_extrude"),
            ("NewBody Revolve", revolve, "new_body_revolve"),
        ]
    } {
        add_linear_pattern_for(
            &mut graph,
            "feature_pattern",
            "plate",
            source_id,
            2,
            8.0,
            FeaturePatternComputeMode::Identical,
            FeaturePatternExtentPolicy::SourceExtent,
            true,
        );
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains("v1 sources must be")),
            "{label} warnings: {warnings:?}"
        );
        assert!((plate_volume(&bodies) - 30.0 * 20.0 * 8.0).abs() < 1.0e-3);
    }

    let mut cross_body = ParametricGraph::new();
    add_box(&mut cross_body);
    add_box_named(&mut cross_body, "other_body", 30.0, 20.0, 8.0);
    cross_body.add_feature(FeatureNode {
        id: "other_hole".into(),
        name: "Other-body hole".into(),
        feature: FeatureType::Hole {
            target: "other_body".into(),
            position: [5.0, 10.0, 8.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    cross_body.add_dependency("other_body", "other_hole");
    add_linear_pattern_for(
        &mut cross_body,
        "feature_pattern",
        "plate",
        "other_hole",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        true,
    );
    let (bodies, warnings) = cross_body
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("v1 sources must be")),
        "cross-body warnings: {warnings:?}"
    );
    assert!((plate_volume(&bodies) - 30.0 * 20.0 * 8.0).abs() < 1.0e-3);
}

#[test]
fn through_all_recomputes_depth_for_each_local_target_section() {
    // The target has one flat top at z=8, but its underside steps from z=4
    // under the source hole to z=0 under both patterned instances. Reusing the
    // source's 4 mm material depth would leave the two instances blind; the
    // correct local through-all result removes 4 + 8 + 8 mm of unit-radius bore.
    let graph = stepped_target_pattern_graph();
    let output = graph
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .expect("evaluate stepped through-all pattern");
    assert!(
        output.warnings.is_empty(),
        "warnings: {:?}",
        output.warnings
    );

    let initial_volume = 30.0 * 20.0 * 4.0 + 20.0 * 20.0 * 4.0;
    let removed = initial_volume - plate_volume(&output.bodies);
    let expected_removed = std::f64::consts::PI * (4.0 + 8.0 + 8.0);
    assert!(
        (removed - expected_removed).abs() / expected_removed < 0.05,
        "local through-all removal {removed} vs {expected_removed}"
    );
}

#[test]
fn a_source_later_in_history_fails_loud_without_reordering_or_guessing() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_linear_pattern_for(
        &mut graph,
        "feature_pattern",
        "plate",
        "late_hole",
        2,
        8.0,
        FeaturePatternComputeMode::Identical,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        false,
    );
    graph.add_feature(FeatureNode {
        id: "late_hole".into(),
        name: "Late source".into(),
        feature: FeatureType::Hole {
            target: "plate".into(),
            position: [5.0, 10.0, 8.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    // The target dependency intentionally follows the body timeline, making
    // the late source depend on the pattern. There is no source -> pattern
    // scheduling edge, so the evaluator must reject the forward reference.
    graph.add_dependency("plate", "late_hole");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .expect("late source must fail as an unresolved feature, not abort evaluation");
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("not earlier in the evaluated history")),
        "warnings: {warnings:?}"
    );

    let mut source_only = ParametricGraph::new();
    add_box(&mut source_only);
    add_source_hole(&mut source_only, None);
    let expected = source_only
        .evaluate_bodies(&Default::default())
        .expect("source-only reference");
    assert!((plate_volume(&bodies) - plate_volume(&expected)).abs() < 1.0e-6);
}

#[test]
fn missing_historical_target_before_and_after_source_fail_loud() {
    let mut missing_before = joined_source_pattern_graph(false);
    let cold = missing_before
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .expect("prime joined-source checkpoints");
    assert!(cold.warnings.is_empty(), "warnings: {:?}", cold.warnings);
    let source_checkpoint = body_checkpoint_position_after(&missing_before, "join_source");
    assert!(source_checkpoint > 0);
    remove_body_from_checkpoint(&missing_before, source_checkpoint - 1, "plate");
    set_pattern_count(&mut missing_before, "feature_pattern", 3);
    let before = missing_before
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .expect("missing historical-before state is a feature rejection");
    assert!(
        before
            .warnings
            .iter()
            .any(|warning| warning.contains("missing immediately before source")),
        "warnings: {:?}",
        before.warnings
    );
    assert!(before
        .trace
        .evaluated_features
        .iter()
        .any(|id| id.as_str() == "feature_pattern"));

    // An independent body checkpoint after the source restores a valid current
    // target while the deliberately damaged source checkpoint lacks its
    // historical-after state. That reaches the separate defensive branch.
    let mut missing_after = joined_source_pattern_graph(true);
    let cold = missing_after
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .expect("prime joined-source checkpoints with witness");
    assert!(cold.warnings.is_empty(), "warnings: {:?}", cold.warnings);
    let source_checkpoint = body_checkpoint_position_after(&missing_after, "join_source");
    remove_body_from_checkpoint(&missing_after, source_checkpoint, "plate");
    set_pattern_count(&mut missing_after, "feature_pattern", 3);
    let after = missing_after
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &live_cancellation(),
        )
        .expect("missing historical-after state is a feature rejection");
    assert!(
        after
            .warnings
            .iter()
            .any(|warning| warning.contains("missing immediately after source")),
        "warnings: {:?}",
        after.warnings
    );
    assert!(after
        .trace
        .reused_checkpoints
        .iter()
        .any(|id| id.as_str() == "witness"));
}

#[test]
fn extent_and_compute_mode_changes_invalidate_the_pattern_checkpoint() {
    let hidden = std::collections::HashSet::new();

    let mut extent_graph = ParametricGraph::new();
    add_box(&mut extent_graph);
    add_source_hole(&mut extent_graph, None);
    add_linear_pattern(
        &mut extent_graph,
        3,
        8.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    extent_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("prime extent-policy cache");
    let pattern_index = extent_graph.node_map["feature_pattern"];
    let FeatureType::FeaturePattern { extent_policy, .. } =
        &mut extent_graph.graph[pattern_index].feature
    else {
        unreachable!()
    };
    *extent_policy = FeaturePatternExtentPolicy::SourceExtent;
    let warm_extent = extent_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("warm extent-policy mutation");
    let cold_extent_graph = extent_graph.clone_document();
    let cold_extent = cold_extent_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("cold extent-policy rebuild");
    assert_eq!(
        mesh_digest(&warm_extent.bodies),
        mesh_digest(&cold_extent.bodies)
    );
    assert_eq!(warm_extent.warnings, cold_extent.warnings);
    assert!(warm_extent
        .trace
        .evaluated_features
        .iter()
        .any(|id| id.as_str() == "feature_pattern"));
    assert!(warm_extent
        .trace
        .reused_checkpoints
        .iter()
        .any(|id| id.as_str() == "hole_source"));

    let mut compute_graph = ParametricGraph::new();
    add_box(&mut compute_graph);
    add_source_hole(&mut compute_graph, None);
    add_linear_pattern(
        &mut compute_graph,
        3,
        8.0,
        FeaturePatternExtentPolicy::ThroughAllLocalTarget,
    );
    compute_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("prime compute-mode cache");
    let pattern_index = compute_graph.node_map["feature_pattern"];
    let FeatureType::FeaturePattern { compute_mode, .. } =
        &mut compute_graph.graph[pattern_index].feature
    else {
        unreachable!()
    };
    *compute_mode = FeaturePatternComputeMode::UnsupportedForTest;
    let warm_compute = compute_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("warm compute-mode mutation");
    let cold_compute_graph = compute_graph.clone_document();
    let cold_compute = cold_compute_graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &live_cancellation())
        .expect("cold compute-mode rebuild");
    assert_eq!(
        mesh_digest(&warm_compute.bodies),
        mesh_digest(&cold_compute.bodies)
    );
    assert_eq!(warm_compute.warnings, cold_compute.warnings);
    assert!(warm_compute
        .warnings
        .iter()
        .any(|warning| warning.contains("only Identical computation is supported")));
    assert!(warm_compute
        .trace
        .evaluated_features
        .iter()
        .any(|id| id.as_str() == "feature_pattern"));
    assert!(warm_compute
        .trace
        .reused_checkpoints
        .iter()
        .any(|id| id.as_str() == "hole_source"));
}
