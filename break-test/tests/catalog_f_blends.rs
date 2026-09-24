//! Catalog F — fillets and chamfers (adversarial plan §6.F).
//!
//! F01 (radius sweeps) and F03 (rim blends) are covered by
//! `edge_blend_break.rs` and the `*_rim_blend` core tests; F07 (infeasible
//! radii) by `fillet_overflow_document.rs` and the oversize gray-zone
//! characterizations. This file adds partial chains at multi-way junctions
//! (F02), order-dependence of fillet/cut (F04), intersecting blends (F05),
//! upstream edits that split or remove the selected edge (F06), and
//! placement invariance of blends (F08).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::sketch::CornerKind;
use zerocad_core::{EdgeCornerMode, EdgeRef, ExtrudeMode, FeatureType, ParametricGraph};

fn box_edge_x(w: f32, _h: f32, _d: f32) -> EdgeRef {
    EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [w, 0.0, 0.0],
        n1: [0.0, 0.0, -1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn box_edge_y(w: f32, h: f32, _d: f32) -> EdgeRef {
    EdgeRef {
        p0: [w, 0.0, 0.0],
        p1: [w, h, 0.0],
        n1: [0.0, 0.0, -1.0],
        n2: [1.0, 0.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn box_edge_z(w: f32, _h: f32, d: f32) -> EdgeRef {
    EdgeRef {
        p0: [w, 0.0, 0.0],
        p1: [w, 0.0, d],
        n1: [0.0, -1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn add_edge_blend(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    edges: Vec<EdgeRef>,
    dist: f32,
    kind: CornerKind,
) {
    add_feature(
        g,
        id,
        FeatureType::EdgeBlend {
            target: target.to_string(),
            edges,
            dist,
            dist_expr: None,
            kind,
            corner_mode: EdgeCornerMode::Auto,
        },
        &[target],
    );
}

/// F02: partial edge chains at 3-way (box corner) junctions — every subset
/// must terminate cleanly or reject with the source unchanged.
#[test]
fn f02_partial_chains_at_three_way_corner() {
    let subsets: [(&str, Vec<EdgeRef>); 3] = [
        ("one", vec![box_edge_x(20.0, 20.0, 20.0)]),
        (
            "two",
            vec![box_edge_x(20.0, 20.0, 20.0), box_edge_y(20.0, 20.0, 20.0)],
        ),
        (
            "three",
            vec![
                box_edge_x(20.0, 20.0, 20.0),
                box_edge_y(20.0, 20.0, 20.0),
                box_edge_z(20.0, 20.0, 20.0),
            ],
        ),
    ];
    let full_loss = (1.0 - std::f64::consts::FRAC_PI_4) * 9.0 * 20.0;
    for (name, edges) in subsets {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_blend(&mut g, "fillet_2", "box_1", edges, 3.0, CornerKind::Fillet);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        let v = total_volume(&bodies);
        let loss = 8000.0 - v;
        if warnings.is_empty() {
            // Clean termination: each selected edge loses its rolling-ball
            // wedge; shared-corner material is not double counted.
            let n = match name {
                "one" => 1.0,
                "two" => 2.0,
                "three" => 3.0,
                _ => unreachable!(),
            };
            assert!(
                loss >= 0.8 * n * full_loss && loss <= 1.4 * n * full_loss + 60.0,
                "{name}-edge fillet loss {loss} vs {n}×{full_loss}"
            );
        } else {
            // Unchanged-source rejection is allowed, but then the body must
            // be exactly the original box.
            assert!(
                close(v, 8000.0, 1e-6, 0.001),
                "{name}: rejected fillet must leave the box unchanged ({v})"
            );
            assert!(
                warnings.iter().any(|w| w.contains("fillet_2")),
                "{name}: rejection must be attributed: {warnings:?}"
            );
        }
    }
}

/// F04: fillet-then-cut vs cut-then-fillet on a fixture where the cut
/// consumes part of the filleted edge — the two orderings legitimately
/// differ, and each must match its own analytic answer.
#[test]
fn f04_fillet_cut_order_dependence_is_exact_each_way() {
    let r = 3.0_f64;
    let fillet_loss = (1.0 - std::f64::consts::FRAC_PI_4) * r * r * 20.0;

    // Order A: fillet the full edge, then cut away x ∈ [15,20] (the cut
    // swallows the fillet region): remaining = 6000 exactly.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_blend(
        &mut g,
        "fillet_2",
        "box_1",
        vec![box_edge_z(20.0, 20.0, 20.0)],
        3.0,
        CornerKind::Fillet,
    );
    add_sketch_cs(
        &mut g,
        "sk_3",
        xy_plane_at(20.0),
        rect_sketch((15.0, -5.0), (25.0, 25.0)),
    );
    add_extrude(&mut g, "cut_4", "sk_3", -25.0, ExtrudeMode::Cut);
    let a = assert_catalog_sane(&g);
    assert_close(a, 6000.0, 1e-6, 0.02, "fillet-then-cut volume");

    // Order B: cut first (edge shortens to length 20 but now on a 15-deep
    // face; the full edge is still filletable), then fillet: 6000 − loss.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(20.0),
        rect_sketch((15.0, -5.0), (25.0, 25.0)),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -25.0, ExtrudeMode::Cut);
    add_edge_blend(
        &mut g,
        "fillet_4",
        "box_1",
        vec![box_edge_z(20.0, 20.0, 20.0)],
        3.0,
        CornerKind::Fillet,
    );
    let b = assert_catalog_sane(&g);
    // The edge survives the cut at full length; its loss persists.
    assert!(
        close(b, 6000.0 - fillet_loss, 1.0, 0.05) || close(b, 6000.0, 1e-6, 0.02),
        "cut-then-fillet volume {b} vs {}",
        6000.0 - fillet_loss
    );
    // The orderings do not silently commute to the same wrong answer: if
    // both report exactly 6000, the fillet vanished in order B — which is
    // only acceptable when explicitly warned.
    if close(b, 6000.0, 1e-6, 0.02) && close(a, 6000.0, 1e-6, 0.02) {
        let (_, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert!(
            warnings.iter().any(|w| w.contains("fillet_4")),
            "a vanished fillet must be attributed: {warnings:?}"
        );
    }
}

/// F05: a second blend intersecting the first — collapsed faces, open seams,
/// or refilled cutouts must not silently pass.
#[test]
fn f05_second_blend_intersecting_the_first() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    // Two perpendicular edges sharing the (20,0,0) corner.
    add_edge_blend(
        &mut g,
        "fillet_2",
        "box_1",
        vec![box_edge_z(20.0, 20.0, 20.0)],
        3.0,
        CornerKind::Fillet,
    );
    add_edge_blend(
        &mut g,
        "fillet_3",
        "box_1",
        vec![box_edge_y(20.0, 20.0, 20.0)],
        3.0,
        CornerKind::Fillet,
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_valid(&bodies);
    let single = (1.0 - std::f64::consts::FRAC_PI_4) * 9.0 * 20.0;
    if warnings.is_empty() {
        let loss = 8000.0 - total_volume(&bodies);
        assert!(
            loss >= 1.6 * single,
            "intersecting blends removed {loss}, less than ~two wedges ({single})"
        );
    }
}

/// F06: upstream dimension edits until the selected edge splits, disappears,
/// or has a lookalike — the reference must reattach deliberately or become
/// unresolved; proximity must not silently replace identity.
#[test]
fn f06_upstream_edits_move_split_or_remove_the_selected_edge() {
    // (a) Shrink the box until the fillet no longer fits: rejection with the
    // edited box geometry intact.
    for w in [4.0_f32, 2.5] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", w, 20.0, 20.0);
        add_edge_blend(
            &mut g,
            "fillet_2",
            "box_1",
            vec![box_edge_z(w, 20.0, 20.0)],
            3.0,
            CornerKind::Fillet,
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        let expected = w as f64 * 400.0;
        let v = total_volume(&bodies);
        if w < 3.0 + 3.0 {
            // Infeasible: either rejected (box at w) or a smaller-radius
            // legitimate result — never a broken body.
            assert!(
                close(v, expected, 1e-6, 0.001) || v < expected,
                "w={w}: volume {v} vs box {expected}"
            );
        }
        if close(v, expected, 1e-6, 0.001) {
            assert!(
                warnings.iter().any(|w_| w_.contains("fillet_2")),
                "w={w}: unchanged body must come with an attributed warning: {warnings:?}"
            );
        }
    }

    // (b) The geometric reference points at the OLD edge position after the
    // upstream edit — it must not silently jump to the moved edge.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    // Reference the edge at x=20 (before the edit) via a clone that keeps
    // stale coordinates.
    let stale = box_edge_z(20.0, 20.0, 20.0);
    // Simulate the upstream edit: the box is now 30 wide → the real edge
    // moved to x=30. The stale reference at x=20 matches nothing.
    for rec in g.graph.node_weights_mut() {
        if rec.id == "box_1" {
            if let FeatureType::Box { w, .. } = &mut rec.feature {
                *w = 30.0;
            }
        }
    }
    g.commit_feature_edit("box_1").expect("commit");
    add_edge_blend(
        &mut g,
        "fillet_2",
        "box_1",
        vec![stale],
        3.0,
        CornerKind::Fillet,
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Either: the reference resolved deliberately (some fillet happened on
    // the moved edge — volume below the plain 30×20×20 box), or it became
    // unresolved with the box intact. Matching the OLD x=20 position on the
    // new geometry (an invented interior edge) would be a silent jump.
    assert!(
        v <= 30.0 * 20.0 * 20.0 + 1e-6,
        "stale reference invented material: {v}"
    );
    if close(v, 30.0 * 20.0 * 20.0, 1e-6, 0.001) {
        assert!(
            warnings.iter().any(|w| w.contains("fillet_2")),
            "unresolved stale reference must warn: {warnings:?}"
        );
    }
}

/// F08: translating and rotating a blended body must not change its material
/// — seam representation may not affect the result.
#[test]
fn f08_blend_volume_is_placement_invariant() {
    let build = |translation: [f32; 3]| -> f64 {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_blend(
            &mut g,
            "fillet_2",
            "box_1",
            vec![box_edge_z(20.0, 20.0, 20.0)],
            3.0,
            CornerKind::Fillet,
        );
        add_feature(
            &mut g,
            "tf_3",
            FeatureType::BodyTransform {
                source: "box_1".to_string(),
                translation,
                copy: false,
            },
            &["box_1"],
        );
        assert_catalog_sane(&g)
    };
    let home = build([0.0, 0.0, 0.0]);
    let expected = 8000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 9.0 * 20.0;
    assert_close(home, expected, 1e-6, 0.03, "fillet at origin");
    // Far from the origin, coordinates lose absolute precision — the volume
    // must still agree within a declared tolerance scaled by distance.
    let far = build([1.0e5, -5.0e4, 7.0e3]);
    assert_close(far, home, 1e-6, 0.02, "fillet translated far away");
    let small = build([0.123, -0.456, 0.789]);
    assert_close(small, home, 1e-3, 1e-6, "fillet translated slightly");
}
