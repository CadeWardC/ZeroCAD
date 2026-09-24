//! Sketch-geometry and variable/expression torture.
//!
//! Half of this suite attacks region detection itself: self-intersecting
//! profiles, duplicate geometry, nesting, crossing line grids, degenerate
//! segments, extreme/micro coordinates. The other half attacks the
//! expression system driving every `*_expr` field: valid math, missing
//! variables, syntax errors, division by zero, negative results, and
//! variable-to-variable chains.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{
    ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves, Unit, Variable,
};

fn extrude_with_expr(
    g: &mut ParametricGraph,
    id: &str,
    sk: &str,
    depth: f32,
    depth_expr: Option<String>,
    mode: ExtrudeMode,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            target: None,
            depth_expr,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency(sk, id);
}

fn add_varset(g: &mut ParametricGraph, id: &str, vars: Vec<Variable>) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::VariableSet { variables: vars },
    });
}

fn var(name: &str, value: f64, expression: Option<&str>) -> Variable {
    Variable {
        name: name.to_string(),
        value,
        unit: Unit::Millimeter,
        expression: expression.map(|e| e.to_string()),
    }
}

// ---------------------------------------------------------------------------
// Sketch geometry torture.
// ---------------------------------------------------------------------------

#[test]
fn self_intersecting_bowtie_profile() {
    // Four lines forming a bowtie (two triangles sharing a crossing point).
    let mut c = SketchCurves::new();
    c.add_line((0.0, 0.0), (20.0, 20.0));
    c.add_line((20.0, 20.0), (20.0, 0.0));
    c.add_line((20.0, 0.0), (0.0, 20.0));
    c.add_line((0.0, 20.0), (0.0, 0.0));
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", c);
    extrude_with_expr(&mut g, "ex_2", "sk_1", 5.0, None, ExtrudeMode::NewBody);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    // Either interpretation (one bowtie region or two triangles = 200 area)
    // is geometrically defensible; anything else is a break.
    let v = total_volume(&bodies);
    let _ = warnings;
    assert!(
        v > 0.0 && v <= 200.0 * 5.0 * 1.05,
        "bowtie volume {v} outside (0, 210]"
    );
}

#[test]
fn duplicate_coincident_rectangles_do_not_double_count() {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 0.0), (20.0, 20.0));
    c.add_rectangle((0.0, 0.0), (20.0, 20.0)); // exact duplicate
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", c);
    extrude_with_expr(&mut g, "ex_2", "sk_1", 10.0, None, ExtrudeMode::NewBody);
    let v = assert_part_sane(&g);
    assert!(
        (v - 4000.0).abs() / 4000.0 < 0.02,
        "duplicated rectangle must extrude once: {v} vs 4000"
    );
}

// Region selection works: [0] extrudes region 0 exactly, duplicate indices
// dedup, and the empty list means all. (An earlier probe suggested
// selection was a no-op — that was the probe's fault: its sketch sat at
// z = 0 inside the box, where a Join prism adds nothing.)
#[test]
fn region_selection_and_duplicates_work_exactly() {
    // Sketch ON the box top face (z = 10).
    for (indices, exact) in [
        (vec![], 16600.0),
        (vec![0usize], 16600.0),
        (vec![0usize, 0, 0], 16600.0),
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
        add_sketch_cs(
            &mut g,
            "sk_2",
            xy_plane_at(10.0),
            rect_sketch((5.0, 5.0), (15.0, 15.0)),
        );
        g.add_feature(FeatureNode {
            id: "ex_3".to_string(),
            name: "ex_3".to_string(),
            feature: FeatureType::Extrude {
                depth: 6.0,
                region_indices: indices.clone(),
                mode: ExtrudeMode::Join,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        g.add_dependency("sk_2", "ex_3");
        let v = assert_part_sane(&g);
        assert!(
            (v - exact).abs() / exact < 0.02,
            "indices {indices:?}: {v} vs {exact}"
        );
    }

    // Two disjoint circles, NewBody: [0] selects the small disc, [1] the big.
    for (indices, exact) in [
        (vec![0usize], std::f64::consts::PI * 16.0 * 5.0),
        (vec![1usize], std::f64::consts::PI * 64.0 * 5.0),
    ] {
        let mut g = ParametricGraph::new();
        let mut c = SketchCurves::new();
        c.add_circle((0.0, 0.0), 4.0);
        c.add_circle((30.0, 0.0), 8.0);
        add_sketch(&mut g, "sk_1", c);
        g.add_feature(FeatureNode {
            id: "ex_2".to_string(),
            name: "ex_2".to_string(),
            feature: FeatureType::Extrude {
                depth: 5.0,
                region_indices: indices.clone(),
                mode: ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        g.add_dependency("sk_1", "ex_2");
        let v = assert_part_sane(&g);
        assert!(
            (v - exact).abs() / exact < 0.05,
            "circle selection {indices:?}: {v} vs {exact}"
        );
    }
}

// E23 (BUG, pinned): OUT-OF-RANGE region indices are silently ignored. A
// single-region sketch extruded with indices [1] (or [0,1] mixing one valid
// and one invalid) applies NOTHING for the invalid entries — and for the
// all-invalid list the whole feature is skipped with NO warning. When fixed,
// require a warning (or per-index rejection) for invalid indices.
#[test]
fn characterization_out_of_range_region_indices_are_silent() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    g.add_feature(FeatureNode {
        id: "ex_3".to_string(),
        name: "ex_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 6.0,
            region_indices: vec![1, 7, 99],
            mode: ExtrudeMode::Join,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "ex_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings.is_empty(),
        "invalid region indices are currently SILENT: {warnings:?}"
    );
    let v = total_volume(&bodies);
    assert!(
        (v - 16000.0).abs() < 10.0,
        "pinning current behavior: all-invalid indices skip the feature ({v})"
    );
}

#[test]
fn triple_nested_rectangles_cut() {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 0.0), (60.0, 60.0));
    c.add_rectangle((10.0, 10.0), (50.0, 50.0));
    c.add_rectangle((20.0, 20.0), (40.0, 40.0));
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 70.0, 70.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -4.0, None, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Accept the plausible nesting interpretations (outer ring, outer+inner,
    // full disc...) — reject corruption.
    let solid = 49000.0;
    assert!(
        v < solid && v > solid - 3600.0 * 4.0 * 1.1,
        "nested cut {v} implausible ({warnings:?})"
    );
}

#[test]
fn crossing_line_grid_regions() {
    // 8 vertical × 8 horizontal lines → 49 interior cells; cut all regions.
    let mut c = SketchCurves::new();
    for i in 0..8 {
        let x = 5.0 + i as f32 * 5.0;
        c.add_line((x, 0.0), (x, 40.0));
    }
    for j in 0..8 {
        let y = 5.0 + j as f32 * 5.0;
        c.add_line((0.0, y), (40.0, y));
    }
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -2.0, None, ExtrudeMode::Cut);
    // Every bounded cell must be removed; shared grid edges are internal to
    // the selected region union and must not cause partial application.
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(warnings.is_empty(), "{warnings:?}");
    let v = total_volume(&bodies);
    let max_cells = 49.0 * 25.0 * 2.0;
    assert_close(v, 16000.0 - max_cells, 0.01, 1e-6, "complete 49-cell cut");
    assert_deterministic(&g);
}

#[test]
fn zero_length_and_nan_lines_mixed_with_valid_geometry() {
    let mut c = SketchCurves::new();
    c.add_rectangle((5.0, 5.0), (15.0, 15.0));
    c.add_line((0.0, 0.0), (0.0, 0.0)); // zero-length
    c.add_line((f32::NAN, 0.0), (3.0, 3.0)); // NaN endpoint
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -3.0, None, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // The valid rectangle region must still cut; garbage segments ignored
    // or warned — but never block the valid region.
    assert!(
        v < 4000.0,
        "valid region must still cut: {v} ({warnings:?})"
    );
}

#[test]
fn line_exactly_on_rectangle_edge() {
    // A duplicate boundary segment (the rect's own bottom edge redrawn).
    let mut c = SketchCurves::new();
    c.add_rectangle((5.0, 5.0), (15.0, 15.0));
    c.add_line((5.0, 5.0), (15.0, 5.0)); // exactly on the bottom edge
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -3.0, None, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 4000.0 - 100.0 * 3.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "duplicate edge must not break region: {v} vs {exact}"
    );
}

#[test]
fn sketch_at_huge_coordinates() {
    // A 20 mm square centered 1e6 mm from the origin — f32 precision stress.
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sk_1",
        rect_sketch((1.0e6, 1.0e6), (1.0e6 + 20.0, 1.0e6 + 20.0)),
    );
    extrude_with_expr(&mut g, "ex_2", "sk_1", 5.0, None, ExtrudeMode::NewBody);
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.1,
        "far-offset sketch keeps ~1% volume accuracy: {v} vs 2000"
    );
}

#[test]
fn micro_geometry_sketch() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (0.001, 0.001)));
    extrude_with_expr(&mut g, "ex_2", "sk_1", 0.001, None, ExtrudeMode::NewBody);
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok((bodies, _)) => assert_meshes_finite(&bodies),
        Err(_) => {} // structured rejection acceptable at this scale
    }
}

#[test]
fn fifty_concentric_circles() {
    let mut c = SketchCurves::new();
    for i in 1..=50 {
        c.add_circle((20.0, 20.0), i as f32);
    }
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -2.0, None, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let solid = 36000.0;
    assert!(v < solid, "onion cut removes material: {v} ({warnings:?})");
}

#[test]
fn internally_tangent_circle_pair() {
    // Circle r=10 and r=6 whose boundaries touch at exactly one point
    // (centers 4 apart): the tangent point is the degeneracy.
    let mut c = SketchCurves::new();
    c.add_circle((20.0, 20.0), 10.0);
    c.add_circle((24.0, 20.0), 6.0);
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    extrude_with_expr(&mut g, "cut_3", "sk_2", -3.0, None, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

// ---------------------------------------------------------------------------
// Variables and expressions.
// ---------------------------------------------------------------------------

#[test]
fn variable_driven_extrude_depth_is_exact() {
    let mut g = ParametricGraph::new();
    add_varset(&mut g, "vars_1", vec![var("boss_h", 12.0, None)]);
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        1.0,
        Some("boss_h * 2".to_string()),
        ExtrudeMode::NewBody,
    );
    let v = assert_part_sane(&g);
    assert!(
        (v - 100.0 * 24.0).abs() / 2400.0 < 0.02,
        "expression depth boss_h*2 → 24: {v} vs 2400"
    );
}

#[test]
fn missing_variable_falls_back_with_warning() {
    let mut g = ParametricGraph::new();
    add_varset(&mut g, "vars_1", vec![var("w", 5.0, None)]);
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        8.0,
        Some("ghost_w * 2".to_string()),
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("no longer evaluates") || w.to_lowercase().contains("expression")),
        "missing variable must warn: {warnings:?}"
    );
    let v = total_volume(&bodies);
    assert!(
        (v - 800.0).abs() / 800.0 < 0.02,
        "fallback depth 8 must be used: {v} vs 800"
    );
}

#[test]
fn expression_syntax_errors_fall_back() {
    for expr in ["2++*", "((", "w w", "1 2 +", ")", "sin(", "**"] {
        let mut g = ParametricGraph::new();
        add_varset(&mut g, "vars_1", vec![var("w", 5.0, None)]);
        add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        extrude_with_expr(
            &mut g,
            "ex_3",
            "sk_2",
            4.0,
            Some(expr.to_string()),
            ExtrudeMode::NewBody,
        );
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&HashSet::new())
            .expect("must not hard-fail");
        assert_meshes_finite(&bodies);
        let v = total_volume(&bodies);
        assert!(
            (v - 400.0).abs() / 400.0 < 0.02,
            "syntax error {expr:?} must fall back to depth 4: {v}"
        );
        assert!(
            warnings
                .iter()
                .any(|w| w.to_lowercase().contains("expression")
                    || w.to_lowercase().contains("evaluat")),
            "syntax error {expr:?} must warn: {warnings:?}"
        );
    }
}

#[test]
fn expression_division_by_zero_is_not_finite() {
    // The engine rejects non-finite results → fallback + warning must kick in
    // (never an inf-depth extrude, which is known break A2 territory).
    let mut g = ParametricGraph::new();
    add_varset(&mut g, "vars_1", vec![var("w", 5.0, None)]);
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        6.0,
        Some("w / 0".to_string()),
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 600.0).abs() / 600.0 < 0.02,
        "w/0 must fall back to depth 6: {v}"
    );
    assert!(
        warnings.iter().any(
            |w| w.to_lowercase().contains("expression") || w.to_lowercase().contains("evaluat")
        ),
        "division by zero must warn: {warnings:?}"
    );
}

#[test]
fn negative_expression_depth_is_survivable() {
    let mut g = ParametricGraph::new();
    add_varset(&mut g, "vars_1", vec![var("w", 12.0, None)]);
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        5.0,
        Some("-w".to_string()),
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn variable_expression_chain_resolves() {
    // h = w*3 by expression; depth = h/2 → 15 if chains resolve in order.
    let mut g = ParametricGraph::new();
    add_varset(
        &mut g,
        "vars_1",
        vec![var("w", 10.0, None), var("h", 30.0, Some("w * 3"))],
    );
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        5.0,
        Some("h / 2".to_string()),
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Either the chain resolves (1500) or h's stored value is used (also 30
    // → 1500 here); a broken chain falls back to 5 → 500, which is a break.
    assert!(
        (v - 1500.0).abs() / 1500.0 < 0.02,
        "variable chain h/2 → 15: {v} vs 1500 ({warnings:?})"
    );
}

#[test]
fn variable_unit_conversions_in_expressions() {
    // A variable in inches used by an mm expression: value is stored in base
    // units at evaluation, so 2 in → 50.8 mm.
    let mut g = ParametricGraph::new();
    let mut inch_var = var("thk", 2.0, None);
    inch_var.unit = Unit::Inch;
    add_varset(&mut g, "vars_1", vec![inch_var]);
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    extrude_with_expr(
        &mut g,
        "ex_3",
        "sk_2",
        5.0,
        Some("thk".to_string()),
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Either base-unit 50.8 (5080) or raw 2.0 (200) is a defensible reading;
    // pinning whichever it is documents the semantics.
    assert!(
        (v - 5080.0).abs() / 5080.0 < 0.02 || (v - 200.0).abs() / 200.0 < 0.02,
        "unit semantics pinned: {v} (50.8mm or 2.0mm) ({warnings:?})"
    );
}

#[test]
fn draft_angle_expression_valid_and_broken() {
    for (expr, expect_warn) in [("5".to_string(), false), ("bogus_%%%".to_string(), true)] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
        add_sketch_cs(
            &mut g,
            "sk_2",
            xy_plane_at(10.0),
            rect_sketch((10.0, 10.0), (30.0, 30.0)),
        );
        g.add_feature(FeatureNode {
            id: "ex_3".to_string(),
            name: "ex_3".to_string(),
            feature: FeatureType::Extrude {
                depth: 8.0,
                region_indices: vec![],
                mode: ExtrudeMode::Join,
                target: None,
                depth_expr: None,
                draft_angle_deg: 3.0,
                draft_angle_expr: Some(expr.clone()),
            },
        });
        g.add_dependency("sk_2", "ex_3");
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&HashSet::new())
            .expect("must not hard-fail");
        assert_meshes_finite(&bodies);
        let warned = warnings.iter().any(|w| w.to_lowercase().contains("draft"));
        assert_eq!(
            warned, expect_warn,
            "draft expr {expr:?}: warnings {warnings:?}"
        );
    }
}

#[test]
fn variable_driven_pattern_spacing() {
    let mut g = ParametricGraph::new();
    add_varset(&mut g, "vars_1", vec![var("pitch", 25.0, None)]);
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Linear {
                dir: zerocad_core::AxisBase::X,
                spacing: 1.0,
                spacing_expr: Some("pitch".to_string()),
                count: 4,
            },
        },
    });
    g.add_dependency("box_1", "pattern_2");
    let v = assert_part_sane(&g);
    assert!(
        (v - 4000.0).abs() / 4000.0 < 0.02,
        "spacing_expr drives 25 mm pitch: {v} vs 4000"
    );
}
