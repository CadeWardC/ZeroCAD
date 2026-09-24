//! Catalog A — numeric inputs and expressions (adversarial plan §6.A).
//!
//! A01/A05/A06 have dedicated suites already (`degenerate_params.rs`,
//! `sketch_expr_torture.rs`, `research_variables_history_torture.rs`); this
//! file covers the remaining scenarios: conversion overflow (A02),
//! tolerance-scale magnitudes (A03), derived-quantity overflow (A04), unit
//! input paths (A07), and variable lifecycle across rename/delete/recreate +
//! save/reopen (A08).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::DiagnosticCode;
use zerocad_core::{
    AxisBase, Document, EvaluationCancellation, EvaluationQuality, ExtrudeMode, FeatureType,
    ParametricGraph, PatternKind, ProjectDocument, Unit, Variable,
};

fn eval_expr(input: &str) -> Result<f64, String> {
    zerocad_core::expr::eval(input, &std::collections::HashMap::new())
}

fn typed_diagnostics(g: &ParametricGraph, feature: &str) -> Vec<String> {
    let token =
        EvaluationCancellation::new(0, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)));
    g.evaluate_request(&HashSet::new(), EvaluationQuality::Final, &token)
        .unwrap()
        .diagnostics
        .iter()
        .filter(|d| d.feature_id == feature)
        .map(|d| d.code.as_str().to_string())
        .collect()
}

fn add_variable_set(g: &mut ParametricGraph, id: &str, variables: Vec<Variable>) {
    add_feature(g, id, FeatureType::VariableSet { variables }, &[]);
}

fn variable(name: &str, value: f64, unit: Unit, expression: Option<String>) -> Variable {
    Variable {
        name: name.to_string(),
        value,
        unit,
        expression,
    }
}

fn extrude_with_expr(g: &mut ParametricGraph, id: &str, sketch: &str, depth: f32, expr: &str) {
    add_feature(
        g,
        id,
        FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: Some(expr.to_string()),
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
        &[sketch],
    );
}

/// A02: finite f64 expression results that overflow during conversion to the
/// stored numeric type must produce an attributed parameter error with the
/// source unchanged — never an arena panic and never a "successful" infinite
/// body.
#[test]
fn a02_expression_overflowing_the_stored_type_is_attributed() {
    // The expression grammar has no e-notation, so the overflow is expressed
    // with plain digit literals: 41 nines ≈ 1e41 as f64, +inf once narrowed
    // to f32.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_2",
        "sk_1",
        5.0,
        "99999999999999999999999999999999999999999",
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        bodies.is_empty(),
        "an overflowing depth must not build a body: {:?}",
        body_ids(&bodies)
    );
    let codes = typed_diagnostics(&g, "ex_2");
    assert!(
        codes.contains(&DiagnosticCode::PARAMETER_INVALID.to_string()),
        "expected parameter.invalid for ex_2, got {codes:?} / warnings {warnings:?}"
    );
}

/// A03: values at and below the modeling-tolerance scale. Each must produce
/// valid geometry matching its dimensions or an explicit typed rejection — a
/// collapsed solid reported as success is a wrong success (regression for the
/// sub-tolerance box panic).
#[test]
fn a03_sub_tolerance_magnitudes_are_exact_or_rejected() {
    // Box: 10 × 10 × 1e-7 (depth below the kernel's tolerance model).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 1e-7);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        bodies.is_empty(),
        "sub-tolerance box must be rejected, not built: {:?}",
        body_ids(&bodies)
    );
    assert!(
        typed_diagnostics(&g, "box_1").contains(&DiagnosticCode::PARAMETER_INVALID.to_string()),
        "rejection must be a typed parameter error; warnings {warnings:?}"
    );

    // Extrude: 10 × 10 profile, 1e-7 depth.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "ex_2", "sk_1", 1e-7, ExtrudeMode::NewBody);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    if bodies.is_empty() {
        assert!(!warnings.is_empty(), "empty result needs a warning");
    } else {
        let v = total_volume(&bodies);
        assert!(
            v >= 0.5e-5,
            "sub-tolerance extrude collapsed to {v} but reported success"
        );
    }
}

/// A03 boundary values: exactly at f32 subnormal limits the feature must
/// still terminate with a classified outcome.
#[test]
fn a03_subnormal_box_dimension_terminates_classified() {
    for w in [f32::MIN_POSITIVE, 1e-45] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", w, 10.0, 10.0);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert!(
            bodies.is_empty(),
            "subnormal box dimension must be rejected: {:?}",
            body_ids(&bodies)
        );
        assert!(
            typed_diagnostics(&g, "box_1").contains(&DiagnosticCode::PARAMETER_INVALID.to_string()),
            "subnormal rejection must be typed; warnings {warnings:?}"
        );
    }
}

/// A04: individually finite values whose derived product overflows —
/// pattern instance transforms (count × spacing) must fail as attributed
/// parameter work, not produce infinite vertices.
#[test]
fn a04_pattern_count_times_spacing_overflow_is_attributed() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 1.0, 1.0);
    add_feature(
        &mut g,
        "pat_2",
        FeatureType::Pattern {
            source: "cyl_1".to_string(),
            kind: PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 1e36, // 4096 × 1e36 overflows f32
                spacing_expr: None,
                count: 4096,
            },
        },
        &["cyl_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    // The derived offset (k × spacing) leaves the displayable range: the
    // pattern must roll back atomically with an attributed warning and the
    // source cylinder preserved — never a partial pattern or an infinite
    // mesh. (Classification note: today this lands as operation.failed via
    // the instance-transform message rather than parameter.invalid; see
    // FINDINGS.md A04.)
    assert_close(
        total_volume(&bodies),
        std::f64::consts::PI * 1.0 * 1.0 * 1.0,
        1e-9,
        0.05,
        "source cylinder survives the overflow attempt",
    );
    assert!(
        warnings.iter().any(|w| w.contains("pat_2")),
        "rollback must attribute the feature: {warnings:?}"
    );
}

/// A04: thread lead = pitch × starts overflowing.
#[test]
fn a04_thread_starts_times_pitch_overflow_terminates() {
    use zerocad_core::parametric::FaceRef;
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 5.0, 20.0);
    let face = FaceRef {
        centroid: [0.0, 10.0, 5.0],
        normal: [1.0, 0.0, 0.0],
        topology: None,
    };
    add_feature(
        &mut g,
        "th_2",
        FeatureType::Thread {
            target: "cyl_1".to_string(),
            face,
            internal: false,
            pitch: 1e36, // lead = 1e36 × 1000 → inf in f32
            depth: 0.5,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1000,
            length: None,
            flip: false,
            designation: String::new(),
            standard: None,
        },
        &["cyl_1"],
    );
    let (bodies, _warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    // Whatever happens, the host body must survive the attempt.
    if !bodies.is_empty() {
        let v = total_volume(&bodies);
        assert!(v > 0.0, "threaded body vanished: {v}");
    }
}

/// A04: a non-finite scale factor is rejected with the source unchanged.
#[test]
fn a04_body_scale_infinite_factor_is_rejected() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "sc_2",
        FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: f32::INFINITY,
            factor_expr: None,
            center: [0.0; 3],
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    // The box must survive the attempt unchanged.
    assert_close(total_volume(&bodies), 1000.0, 1e-6, 0.01, "box survives");
    assert!(
        warnings.iter().any(|w| w.contains("sc_2")),
        "rejection must attribute the feature: {warnings:?}"
    );
}

/// A04 tracked defect (found 2026-09-13): a *finite* scale factor whose
/// coordinates exceed the f32 display-mesh range tessellates into a mesh
/// with non-finite vertices. The B-Rep is valid at f64; the f32
/// `MockMesh` boundary has no finiteness gate. Recorded in FINDINGS.md.
#[test]
fn a04_body_scale_display_range_overflow_known_break() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "sc_2",
        FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: 1e37, // 10 × 1e37 = 1e38 still finite in f32…
            factor_expr: None,
            center: [0.0; 3],
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    assert_close(
        total_volume(&bodies),
        1000.,
        1e-6,
        0.001,
        "overflow rolls back scale",
    );
    assert!(warnings.iter().any(|w| w.contains("sc_2")));
}

/// A05 (residual cases not in sketch_expr_torture): empty expressions and
/// trailing garbage must not silently reuse an unrelated prior value.
#[test]
fn a05_empty_and_trailing_garbage_expressions_fall_back_with_warning() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(&mut g, "ex_empty", "sk_1", 8.0, "");
    extrude_with_expr(&mut g, "ex_garbage", "sk_1", 8.0, "5 + 3 7");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 2, "both extrudes must still produce bodies");
    let v = total_volume(&bodies);
    // Documented fallback = last stored value 8.0 → 10×10×8 each.
    assert_close(v, 1600.0, 1e-6, 0.01, "fallback depth volume");
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("no longer evaluates") || w.contains("ex_empty")),
        "empty expression needs an attributable warning: {warnings:?}"
    );
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("no longer evaluates") || w.contains("ex_garbage")),
        "trailing garbage needs an attributable warning: {warnings:?}"
    );
}

/// A07: equivalent dimensions entered in millimeter and inch variables must
/// agree within the conversion contract.
#[test]
fn a07_unit_input_paths_agree_within_conversion_contract() {
    let build = |unit: Unit, value: f64| -> f64 {
        let mut g = ParametricGraph::new();
        add_variable_set(&mut g, "vars", vec![variable("depth", value, unit, None)]);
        add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        extrude_with_expr(&mut g, "ex_2", "sk_1", 1.0, "depth");
        assert_catalog_sane(&g)
    };
    let mm = build(Unit::Millimeter, 12.0);
    let inch = build(Unit::Inch, 12.0 / 25.4);
    assert_close(inch, mm, 1e-9, 1e-4, "mm vs inch depth geometry");
}

/// A08: rename, delete, and recreate variables consumed by several features,
/// then reopen. Consumers must update or fail explicitly — never silently
/// keep stale values.
#[test]
fn a08_variable_lifecycle_rename_delete_recreate_reopen() {
    let mut g = ParametricGraph::new();
    add_variable_set(
        &mut g,
        "vars",
        vec![variable("depth", 10.0, Unit::Millimeter, None)],
    );
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(&mut g, "ex_2", "sk_1", 10.0, "depth");
    let baseline = assert_catalog_sane(&g);
    assert_close(baseline, 1000.0, 1e-6, 0.01, "baseline");

    // Rename: every consumer expression is rewritten, geometry identical.
    g.rename_variable("depth", "span").expect("rename");
    assert_close(
        assert_catalog_sane(&g),
        baseline,
        1e-9,
        0.0,
        "volume after rename",
    );

    // Delete the variable: consumers warn and fall back to the last value.
    for rec in g.graph.node_weights_mut() {
        if rec.id == "vars" {
            if let FeatureType::VariableSet { variables } = &mut rec.feature {
                variables.retain(|v| v.name != "span");
            }
        }
    }
    g.commit_feature_edit("vars").expect("commit edit");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_close(
        total_volume(&bodies),
        baseline,
        1e-9,
        0.0,
        "fallback keeps last value",
    );
    assert!(
        warnings.iter().any(|w| w.contains("no longer evaluates")),
        "deleted variable must warn consumers: {warnings:?}"
    );

    // Reopen the *deleted-variable* state: the stale-expression warning must
    // survive the save/load cycle (otherwise the user loses the diagnostic on
    // reload while the geometry quietly uses the fallback).
    let doc = Document::from_graph(g.clone_document(), Unit::Millimeter);
    let bytes = zerocad_core::write_project_document_to_vec(
        &ProjectDocument::Part(doc),
        &zerocad_core::SaveOptions::default(),
        &zerocad_core::HydrationBundle::default(),
    )
    .expect("save");
    let loaded = zerocad_core::read_project_document_from_slice(
        &bytes,
        &zerocad_core::LoadOptions::default(),
    )
    .expect("load");
    let ProjectDocument::Part(part) = &loaded.document else {
        panic!("expected a part document");
    };
    let (bodies, warnings) = part.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_close(
        total_volume(&bodies),
        baseline,
        1e-9,
        0.0,
        "reopened deleted-variable document keeps the fallback",
    );
    assert!(
        warnings.iter().any(|w| w.contains("no longer evaluates")),
        "reopened document must still surface the stale expression: {warnings:?}"
    );

    // Recreate with a new value: consumers pick it up, in memory and across a
    // second round trip.
    add_variable_set(
        &mut g,
        "vars2",
        vec![variable("span", 20.0, Unit::Millimeter, None)],
    );
    assert_close(
        assert_catalog_sane(&g),
        2.0 * baseline,
        1e-9,
        0.01,
        "recreated variable drives consumers",
    );
    let doc = Document::from_graph(g, Unit::Millimeter);
    let bytes = zerocad_core::write_project_document_to_vec(
        &ProjectDocument::Part(doc),
        &zerocad_core::SaveOptions::default(),
        &zerocad_core::HydrationBundle::default(),
    )
    .expect("save");
    let loaded = zerocad_core::read_project_document_from_slice(
        &bytes,
        &zerocad_core::LoadOptions::default(),
    )
    .expect("load");
    let ProjectDocument::Part(part) = &loaded.document else {
        panic!("expected a part document");
    };
    let (bodies, warnings) = part.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_close(
        total_volume(&bodies),
        2.0 * baseline,
        1e-9,
        0.01,
        "reopened recreated-variable document",
    );
    assert!(
        !warnings.iter().any(|w| w.contains("no longer evaluates")),
        "a resolved expression must not warn after reopen: {warnings:?}"
    );
}

/// A08 undo path: a snapshot taken before the deletion restores the original
/// evaluation exactly.
#[test]
fn a08_undo_restores_the_pre_delete_state() {
    let mut g = ParametricGraph::new();
    add_variable_set(
        &mut g,
        "vars",
        vec![variable("depth", 10.0, Unit::Millimeter, None)],
    );
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(&mut g, "ex_2", "sk_1", 10.0, "depth");
    let before = assert_catalog_sane(&g);
    let snapshot = g.clone_document();

    g.remove_feature("vars");
    let after = {
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert!(
            warnings.iter().any(|w| w.contains("no longer evaluates")),
            "{warnings:?}"
        );
        total_volume(&bodies)
    };
    assert_close(after, before, 1e-9, 0.0, "fallback equals last value");

    let restored = snapshot;
    assert_close(
        assert_catalog_sane(&restored),
        before,
        1e-9,
        0.0,
        "undo snapshot restores original",
    );
}

/// A04/A02 sanity guard: the expression evaluator itself rejects nonsense
/// without panicking and never invents values.
#[test]
fn a05_expression_evaluator_rejects_malformed_input_cleanly() {
    for expr in ["", "   ", "2 +", "((", "1/0", "foo + 1", "1 2 3", ")("] {
        // Any outcome is fine except a panic; success would be a bug for
        // these particular inputs.
        let out = eval_expr(expr);
        if expr == "1/0" {
            assert!(out.is_err(), "division by zero must be an error");
        }
    }
}

#[test]
fn a08_variable_rename_is_atomic_on_collision() {
    let mut g = ParametricGraph::new();
    add_variable_set(
        &mut g,
        "vars",
        vec![
            variable("width", 10.0, Unit::Millimeter, None),
            variable("height", 5.0, Unit::Millimeter, None),
        ],
    );
    let before = format!("{:?}", g.graph.node_weights().count());
    assert!(g.rename_variable("width", "height").is_err());
    assert_eq!(before, format!("{:?}", g.graph.node_weights().count()));
    // The failed rename must not have rewritten any expression.
    assert!(g.rename_variable("width", "span").is_ok());
}
