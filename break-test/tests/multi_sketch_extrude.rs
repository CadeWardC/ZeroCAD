//! Single-feature / multi-sketch capability pins.
//!
//! E21: ONE Extrude driving TWO sketches. A Loft already legally consumes
//! several sketch sections in one feature, but an Extrude with two sketch
//! parents is rejected at the document semantic gate today. This suite pins
//! the current error and the surrounding variants (zero sketches, sketch +
//! non-sketch parents) so the capability gap is recorded, not folklore.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{ExtrudeMode, FeatureNode, FeatureType, ParametricGraph};

fn add_extrude_node(
    g: &mut ParametricGraph,
    id: &str,
    depth: f32,
    mode: ExtrudeMode,
    parents: &[&str],
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    for p in parents {
        g.add_dependency(p, id);
    }
}

// E21 regression (fixed 2026-09-11): one Extrude with TWO sketch parents used
// to SILENTLY extrude only the first matching parent — evaluation succeeded,
// the body grew by exactly one boss (16800 instead of 17600), and NO warning
// was emitted. The evaluator now counts eligible sketch parents and requires
// exactly one: zero or multiple produce an attributed unresolved outcome that
// names every attached sketch and leaves every body unchanged. (Driving
// several sketches from one extrude remains a separately-scoped capability;
// until then, split them across separate Extrude features — see the control
// below — the way the Loft contrast treats its multi-sketch sections.)
#[test]
fn one_extrude_with_two_sketch_parents_is_rejected_loudly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    // Two independent face sketches at the same height, disjoint footprints.
    add_sketch_cs(
        &mut g,
        "sk_a",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_sketch_cs(
        &mut g,
        "sk_b",
        xy_plane_at(10.0),
        rect_sketch((25.0, 25.0), (35.0, 35.0)),
    );
    add_extrude_node(&mut g, "boss_4", 8.0, ExtrudeMode::Join, &["sk_a", "sk_b"]);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("the guard must degrade to a warning, not an error");
    assert_meshes_finite(&bodies);
    let warning = warnings
        .iter()
        .find(|w| w.contains("boss_4") && w.contains("2 sketch inputs"))
        .unwrap_or_else(|| panic!("the dropped sketch must fail loudly: {warnings:?}"));
    assert!(
        warning.contains("sk_a") && warning.contains("sk_b"),
        "the warning must name every attached sketch: {warnings:?}"
    );
    let v = total_volume(&bodies);
    assert!(
        (v - 16000.0).abs() < 5.0,
        "nothing may be extruded while the feature is unresolved: {v} (base box is 16000)"
    );
}

// Control: the same two sketches extruded by two separate features works
// exactly (and is the current required structure).
#[test]
fn two_sketches_two_extrudes_works_exactly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_a",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_sketch_cs(
        &mut g,
        "sk_b",
        xy_plane_at(10.0),
        rect_sketch((25.0, 25.0), (35.0, 35.0)),
    );
    add_extrude_node(&mut g, "boss_3", 8.0, ExtrudeMode::Join, &["sk_a"]);
    add_extrude_node(&mut g, "boss_4", 8.0, ExtrudeMode::Join, &["sk_b"]);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 + 2.0 * 100.0 * 8.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "two extrudes, exact: {v} vs {exact}"
    );
}

// The Loft contrast: ONE feature legally consuming TWO sketch sections —
// the pattern the two-sketch extrude should follow.
#[test]
fn one_loft_two_sketch_sections_is_legal() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_a", rect_sketch((0.0, 0.0), (20.0, 20.0)));
    add_sketch_cs(
        &mut g,
        "sk_b",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "loft_3".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sk_a".to_string(), 0), ("sk_b".to_string(), 0)],
            surface_mode: zerocad_core::LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sk_a", "loft_3");
    g.add_dependency("sk_b", "loft_3");
    let v = assert_part_sane(&g);
    assert!(v > 0.0, "two-sketch loft must build: {v}");
}

// Degenerate variants around the same rule: an Extrude with NO sketch
// parent, and an Extrude whose extra parent is a box (not a sketch).
#[test]
fn extrude_with_zero_or_non_sketch_parents_is_graceful() {
    // Zero sketch parents.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_extrude_node(&mut g, "orphan_2", 5.0, ExtrudeMode::Join, &[]);
    let (bodies, warnings, statuses) = g.evaluate_bodies_with_status(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "box_1");
    assert!(!warnings.is_empty());
    assert!(statuses
        .iter()
        .any(|s| s.feature_id.as_str() == "orphan_2" && s.is_unresolved()));

    // Sketch + non-sketch (box) parents.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch(&mut g, "sk_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
    add_extrude_node(
        &mut g,
        "weird_3",
        5.0,
        ExtrudeMode::Join,
        &["sk_2", "box_1"],
    );
    let (bodies, warnings, statuses) = g.evaluate_bodies_with_status(&HashSet::new()).unwrap();
    assert!(!bodies.is_empty());
    assert!(warnings.is_empty(), "{warnings:?}");
    assert!(statuses
        .iter()
        .any(|s| s.feature_id.as_str() == "weird_3" && !s.is_unresolved()));
}
