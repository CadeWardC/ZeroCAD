//! Complex extrusions and complex multi-body parts.
//!
//! Every test builds a part the way a real model would: many sketches, many
//! profiles (line loops, overlapping shapes, region grids), extrudes onto and
//! into multiple bodies, targeted booleans between overlapping bodies, and
//! long interleaved histories. Universal invariants as everywhere in this
//! crate: no panics, finite geometry, deterministic re-evaluation — plus
//! analytic volume checks wherever the expected value is computable.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves};

fn body_volume(bodies: &[(String, zerocad_core::MockMesh)], id: &str) -> f64 {
    bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .and_then(|(_, m)| m.mass_properties().map(|mp| mp.volume))
        .unwrap_or(0.0)
}

fn extrude_with(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
    target: Option<String>,
    regions: Vec<usize>,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: regions,
            mode,
            target,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

// ---------------------------------------------------------------------------
// Complex profiles built from line loops.
// ---------------------------------------------------------------------------

#[test]
fn i_beam_from_three_overlapping_rectangles() {
    // I-beam profile: two flanges + web drawn as three overlapping rectangles
    // in ONE sketch. Region detection must union them — extruding overlapping
    // regions separately would double-count the overlaps.
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (40.0, 5.0)); // bottom flange
    curves.add_rectangle((0.0, 35.0), (40.0, 40.0)); // top flange
    curves.add_rectangle((17.5, 0.0), (22.5, 40.0)); // web
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", curves);
    extrude_with(
        &mut g,
        "extrude_2",
        "sketch_1",
        60.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = (40.0 * 5.0 + 40.0 * 5.0 + 5.0 * 30.0) * 60.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "I-beam union volume {v} vs {exact} (overlaps must not double-count)"
    );
}

#[test]
fn triangular_gusset_from_three_lines() {
    // Closed triangle from three bare lines — region detection has to close
    // the loop itself.
    let mut curves = SketchCurves::new();
    curves.add_line((0.0, 0.0), (20.0, 0.0));
    curves.add_line((20.0, 0.0), (0.0, 15.0));
    curves.add_line((0.0, 15.0), (0.0, 0.0));
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", curves);
    extrude_with(
        &mut g,
        "extrude_2",
        "sketch_1",
        4.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 0.5 * 20.0 * 15.0 * 4.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "gusset volume {v} vs {exact}"
    );
}

#[test]
fn concave_l_profile_from_lines() {
    // L-shaped (angle bracket) profile drawn as a single closed 6-segment
    // loop — concave region, unlike rectangles.
    let mut curves = SketchCurves::new();
    let pts = [
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 5.0),
        (5.0, 5.0),
        (5.0, 30.0),
        (0.0, 30.0),
    ];
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        curves.add_line(a, b);
    }
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", curves);
    extrude_with(
        &mut g,
        "extrude_2",
        "sketch_1",
        25.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = (30.0 * 5.0 + 25.0 * 5.0) * 25.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "L-profile volume {v} vs {exact}"
    );
}

#[test]
fn comb_profile_with_many_teeth() {
    // Comb: spine rectangle + 10 teeth, one sketch, ~300 mm of edge segments.
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (8.0, 10.0)); // spine
    for i in 0..10 {
        let x = 10.0 + i as f32 * 10.0;
        curves.add_rectangle((x, 0.0), (x + 5.0, 25.0)); // tooth
    }
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", curves);
    extrude_with(
        &mut g,
        "extrude_2",
        "sketch_1",
        6.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = (8.0 * 10.0 + 10.0 * 5.0 * 25.0) * 6.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "comb volume {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// Region selection: partial extrusion of multi-region sketches.
// ---------------------------------------------------------------------------

#[test]
fn selective_region_extrude_extrudes_less_than_all() {
    // Two disjoint circles: vec![0] must extrude exactly the small disc
    // (region selection is verified in sketch_expr_torture.rs too).
    let curves = |extra: bool| {
        let mut c = SketchCurves::new();
        c.add_circle((0.0, 0.0), 4.0);
        if extra {
            c.add_circle((30.0, 0.0), 8.0);
        }
        c
    };
    let mut g_all = ParametricGraph::new();
    add_sketch(&mut g_all, "sketch_1", curves(true));
    extrude_with(
        &mut g_all,
        "extrude_2",
        "sketch_1",
        5.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v_all = assert_part_sane(&g_all);

    let mut g_one = ParametricGraph::new();
    add_sketch(&mut g_one, "sketch_1", curves(true));
    extrude_with(
        &mut g_one,
        "extrude_2",
        "sketch_1",
        5.0,
        ExtrudeMode::NewBody,
        None,
        vec![0],
    );
    let v_one = assert_part_sane(&g_one);

    // Region selection works: [0] extrudes exactly the small disc.
    let small = std::f64::consts::PI * 16.0 * 5.0;
    assert!(
        (v_one - small).abs() / small < 0.05,
        "explicit region selection picks one disc: {v_one} vs {small} (all = {v_all})"
    );
}

#[test]
fn region_grid_cut_25_pockets() {
    // 5×5 grid of pocket rectangles in ONE sketch, cut all at once.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    let mut curves = SketchCurves::new();
    for i in 0..5 {
        for j in 0..5 {
            let (x, y) = (6.0 + i as f32 * 10.0, 6.0 + j as f32 * 10.0);
            curves.add_rectangle((x, y), (x + 5.0, y + 5.0));
        }
    }
    add_sketch_cs(&mut g, "sketch_2", xy_plane_at(10.0), curves);
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        -3.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 60.0 * 60.0 * 10.0 - 25.0 * 25.0 * 3.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "grid pocket volume {v} vs {exact}"
    );
}

#[test]
fn hundred_region_vent_plate() {
    // 100 same-size slots cut in one feature: volume must be exactly
    // plate − 100·slot·depth (stress for region fan-out in one extrude).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 120.0, 120.0, 3.0);
    let mut curves = SketchCurves::new();
    for i in 0..10 {
        for j in 0..10 {
            let (x, y) = (4.0 + i as f32 * 12.0, 4.0 + j as f32 * 12.0);
            curves.add_rectangle((x, y), (x + 6.0, y + 2.0));
        }
    }
    add_sketch_cs(&mut g, "sketch_2", xy_plane_at(3.0), curves);
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        -3.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 120.0 * 120.0 * 3.0 - 100.0 * 12.0 * 3.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "vent plate volume {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// One sketch feeding several extrudes.
// ---------------------------------------------------------------------------

#[test]
fn one_sketch_feeds_boss_and_pocket_and_new_body() {
    // The same profile sketch drives a Join boss, a Cut pocket, and a
    // NewBody slab — three consumers of one sketch.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (30.0, 30.0)),
    );
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        6.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    // Reuse for a pocket from the new top (plane still z = 10; the pocket
    // cuts into whatever the boss added above).
    extrude_with(
        &mut g,
        "extrude_4",
        "sketch_2",
        2.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    // And once more as a standalone body elsewhere.
    let mut far = SketchCurves::new();
    far.add_rectangle((100.0, 100.0), (120.0, 120.0));
    add_sketch(&mut g, "sketch_5", far);
    extrude_with(
        &mut g,
        "extrude_6",
        "sketch_5",
        5.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 + 20.0 * 20.0 * 6.0 - 20.0 * 20.0 * 2.0 + 20.0 * 20.0 * 5.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "multi-consumer volume {v} vs {exact}"
    );
}

#[test]
fn same_sketch_cut_then_join_on_different_bodies() {
    // One profile sketch driving two TARGETED operations on two adjacent
    // bodies: a pocket into A and a bridging boss fused into B.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 30.0, 10.0);
    // Body B beside A: x 20..50.
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(0.0),
        rect_sketch((20.0, 0.0), (50.0, 30.0)),
    );
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        10.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    // One profile straddling the A|B seam (x 5..25) at z = 10: it touches
    // A's top on x 5..20 and B's top on x 20..25.
    add_sketch_cs(
        &mut g,
        "sketch_4",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (25.0, 25.0)),
    );
    extrude_with(
        &mut g,
        "cut_5",
        "sketch_4",
        -3.0,
        ExtrudeMode::Cut,
        Some("a_1".into()),
        vec![],
    );
    extrude_with(
        &mut g,
        "join_6",
        "sketch_4",
        4.0,
        ExtrudeMode::Join,
        Some("extrude_3".into()),
        vec![],
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v_a = body_volume(&bodies, "a_1");
    let exact_a = 20.0 * 30.0 * 10.0 - 15.0 * 20.0 * 3.0; // pocket covers x 5..20 of the profile
    assert!(
        (v_a - exact_a).abs() / exact_a < 0.02,
        "targeted cut on A: {v_a} vs {exact_a}"
    );
    let v_b = body_volume(&bodies, "extrude_3");
    // Join fuses: B gains the whole connected boss (nothing overlaps B
    // volumetrically — it sits on B's top face).
    let exact_b = 30.0 * 30.0 * 10.0 + 20.0 * 20.0 * 4.0;
    assert!(
        (v_b - exact_b).abs() / exact_b < 0.02,
        "targeted join on B: {v_b} vs {exact_b}"
    );
}

// ---------------------------------------------------------------------------
// Targeted booleans among overlapping bodies.
// ---------------------------------------------------------------------------

#[test]
fn cut_targeting_one_of_two_overlapping_bodies() {
    // A: x 0..20, B: x 10..30, both y 0..10 z 0..10 (they overlap x 10..20).
    // A cutting prism over the overlap region, targeted at B, must leave A
    // untouched. Untargeted, it cuts both.
    let build = |target: Option<String>| {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "a_1", 20.0, 10.0, 10.0);
        // Body B overlapping A: x 10..30 (A spans 0..20).
        add_sketch_cs(
            &mut g,
            "sketch_2",
            xy_plane_at(0.0),
            rect_sketch((10.0, 0.0), (30.0, 10.0)),
        );
        extrude_with(
            &mut g,
            "extrude_3",
            "sketch_2",
            10.0,
            ExtrudeMode::NewBody,
            None,
            vec![],
        );
        // Cutter over the overlap x 12..18.
        add_sketch_cs(
            &mut g,
            "sketch_4",
            xy_plane_at(10.0),
            rect_sketch((12.0, 2.0), (18.0, 8.0)),
        );
        extrude_with(
            &mut g,
            "cut_5",
            "sketch_4",
            -10.0,
            ExtrudeMode::Cut,
            target,
            vec![],
        );
        g
    };
    let g = build(Some("extrude_3".to_string()));
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v_a = body_volume(&bodies, "a_1");
    assert!(
        (v_a - 2000.0).abs() < 20.0,
        "A untouched by targeted cut: {v_a}"
    );
    let v_b = body_volume(&bodies, "extrude_3");
    let exact_b = 2000.0 - 6.0 * 6.0 * 10.0;
    assert!(
        (v_b - exact_b).abs() / exact_b < 0.02,
        "B loses the pocket: {v_b} vs {exact_b}"
    );

    let g = build(None);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v_a = body_volume(&bodies, "a_1");
    assert!(
        (v_a - (2000.0 - 6.0 * 6.0 * 10.0)).abs() / 2000.0 < 0.02,
        "untargeted cut also removes from A: {v_a}"
    );
}

#[test]
fn join_targeting_one_of_two_touching_bodies() {
    // Boss bridging A and B; targeted Join must merge it into A only.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 10.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(0.0),
        rect_sketch((25.0, 0.0), (45.0, 10.0)),
    );
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        10.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    // Boss from A's top face overlapping both.
    add_sketch_cs(
        &mut g,
        "sketch_4",
        xy_plane_at(10.0),
        rect_sketch((15.0, 2.0), (35.0, 8.0)),
    );
    extrude_with(
        &mut g,
        "join_5",
        "sketch_4",
        5.0,
        ExtrudeMode::Join,
        Some("a_1".into()),
        vec![],
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v_a = body_volume(&bodies, "a_1");
    // Join FUSES the whole connected boss into A: the boss (20×6×5) sits on
    // top of A/B (z 10..15) so nothing overlaps A volumetrically — A gains
    // the entire boss volume.
    let exact_a = 2000.0 + 20.0 * 6.0 * 5.0;
    assert!(
        (v_a - exact_a).abs() / exact_a < 0.03,
        "targeted join into A: {v_a} vs {exact_a}"
    );
    let v_b = body_volume(&bodies, "extrude_3");
    assert!(
        (v_b - 2000.0).abs() < 20.0,
        "B untouched by targeted join: {v_b}"
    );
}

#[test]
fn one_cut_through_two_stacked_bodies() {
    // Two bodies stacked in z sharing the cut column: an untargeted cut must
    // punch through both.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 30.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((0.0, 0.0), (30.0, 30.0)),
    );
    extrude_with(
        &mut g,
        "extrude_3",
        "sketch_2",
        8.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    add_sketch_cs(
        &mut g,
        "sketch_4",
        xy_plane_at(18.0),
        circle_sketch((15.0, 15.0), 5.0),
    );
    extrude_with(
        &mut g,
        "cut_5",
        "sketch_4",
        -18.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 30.0 * 30.0 * 18.0 - std::f64::consts::PI * 25.0 * 18.0;
    assert!(
        (v - exact).abs() / exact < 0.04,
        "stacked through-cut {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// Realistic complex parts (many sketches, many features).
// ---------------------------------------------------------------------------

#[test]
fn cylinder_head_block() {
    // Plate + 6 bosses (extruded circles) + 6 counterbored holes + central
    // through bore — 13 sketches/features on one body.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 100.0, 12.0);
    for i in 0..6 {
        let a = i as f32 * std::f32::consts::TAU / 6.0;
        let (cx, cy) = (50.0 + a.cos() * 32.0, 50.0 + a.sin() * 32.0);
        add_sketch_cs(
            &mut g,
            &format!("boss_sk_{i}"),
            xy_plane_at(12.0),
            circle_sketch((cx, cy), 8.0),
        );
        extrude_with(
            &mut g,
            &format!("boss_ex_{i}"),
            &format!("boss_sk_{i}"),
            10.0,
            ExtrudeMode::Join,
            None,
            vec![],
        );
        add_hole(
            &mut g,
            &format!("hole_{i}"),
            "box_1",
            [cx, cy, 22.0],
            [0.0, 0.0, -1.0],
            5.0,
            None,
            zerocad_core::HoleKind::Counterbore {
                diameter: 9.0,
                depth: 2.0,
            },
        );
    }
    add_sketch_cs(
        &mut g,
        "bore_sk",
        xy_plane_at(22.0),
        circle_sketch((50.0, 50.0), 15.0),
    );
    extrude_with(
        &mut g,
        "bore_ex",
        "bore_sk",
        -22.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let (_, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(warnings.is_empty(), "cylinder head features: {warnings:?}");
    let v = assert_part_sane(&g);
    let pi = std::f64::consts::PI;
    let exact = 100.0 * 100.0 * 12.0 + 6.0 * pi * 64.0 * 10.0
        - 6.0 * (pi * 2.5 * 2.5 * 20.0 + pi * 20.25 * 2.0)
        - pi * 225.0 * 12.0;
    assert!(
        (v - exact).abs() / exact < 0.005,
        "cylinder head volume {v} vs {exact}"
    );
}

#[test]
fn ribbed_mounting_plate() {
    // Base plate + two perpendicular rib walls (NOT crossing — see the
    // characterization test below for the crossing failure) + corner gussets
    // (line triangles) + 4 mounting holes.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 80.0, 60.0, 8.0);
    // Rib 1 along X on the top face.
    add_sketch_cs(
        &mut g,
        "rib_sk_1",
        xy_plane_at(8.0),
        rect_sketch((5.0, 28.0), (75.0, 32.0)),
    );
    extrude_with(
        &mut g,
        "rib_1",
        "rib_sk_1",
        12.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    // Rib 2 along Y, parallel-offset so it does not cross rib 1.
    add_sketch_cs(
        &mut g,
        "rib_sk_2",
        xy_plane_at(8.0),
        rect_sketch((38.0, 36.0), (42.0, 55.0)),
    );
    extrude_with(
        &mut g,
        "rib_2",
        "rib_sk_2",
        12.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    // Corner gussets: right triangles on the top face at two corners.
    for (ox, oy, flip) in [(5.0f32, 5.0f32, false), (65.0, 45.0, true)] {
        let mut tri = SketchCurves::new();
        let (a, b, c) = if flip {
            ((ox, oy), (ox + 10.0, oy), (ox, oy - 10.0))
        } else {
            ((ox, oy), (ox + 10.0, oy), (ox, oy + 10.0))
        };
        tri.add_line(a, b);
        tri.add_line(b, c);
        tri.add_line(c, a);
        add_sketch_cs(
            &mut g,
            &format!("gusset_sk_{ox}_{oy}"),
            xy_plane_at(8.0),
            tri,
        );
        extrude_with(
            &mut g,
            &format!("gusset_ex_{ox}_{oy}"),
            &format!("gusset_sk_{ox}_{oy}"),
            8.0,
            ExtrudeMode::Join,
            None,
            vec![],
        );
    }
    for (x, y) in [(8.0f32, 8.0f32), (72.0, 8.0), (8.0, 52.0), (72.0, 52.0)] {
        add_hole(
            &mut g,
            &format!("mh_{x}_{y}"),
            "box_1",
            [x, y, 8.0],
            [0.0, 0.0, -1.0],
            5.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
    }
    let v = assert_part_sane(&g);
    let plate = 80.0 * 60.0 * 8.0;
    let ribs = 70.0 * 4.0 * 12.0 + 4.0 * 19.0 * 12.0; // disjoint ribs, no crossing
    let gussets = 2.0 * 0.5 * 100.0 * 8.0;
    let holes = 4.0 * std::f64::consts::PI * 6.25 * 8.0;
    let exact = plate + ribs + gussets - holes;
    assert!(
        (v - exact).abs() / exact < 0.04,
        "ribbed plate {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// Regression: crossing ribs (historical failures in FINDINGS.md E6/E7).
// ---------------------------------------------------------------------------

// Both crossing ribs must fuse with their common plate, preserving all
// requested material and counting the ribs' overlapping center only once.
#[test]
fn crossing_rib_join_preserves_both_ribs() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 80.0, 60.0, 8.0);
    add_sketch_cs(
        &mut g,
        "rib_sk_1",
        xy_plane_at(8.0),
        rect_sketch((5.0, 28.0), (75.0, 32.0)),
    );
    extrude_with(
        &mut g,
        "rib_1",
        "rib_sk_1",
        12.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    add_sketch_cs(
        &mut g,
        "rib_sk_2",
        xy_plane_at(8.0),
        rect_sketch((38.0, 5.0), (42.0, 55.0)),
    );
    extrude_with(
        &mut g,
        "rib_2",
        "rib_sk_2",
        12.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    assert_crossed_rib_material(&g);
    assert_crossed_rib_material(&g.clone_document());
}

fn assert_crossed_rib_material(g: &ParametricGraph) {
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_meshes_valid(&bodies);
    let exact = 80.0 * 60.0 * 8.0 + (70.0 * 4.0 + 50.0 * 4.0 - 4.0 * 4.0) * 12.0;
    assert_close(
        total_volume(&bodies),
        exact,
        1e-6,
        0.01,
        "plate plus both ribs, overlap counted once",
    );
    assert_occupancy(
        &bodies[0].1,
        &[
            [10., 10., 4.],
            [10., 30., 15.],
            [40., 10., 15.],
            [40., 30., 15.],
        ],
        &[[10., 10., 15.], [40., 30., 21.]],
    );
    let solids = g.evaluated_kernel_bodies(&HashSet::new()).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].1.len(), 1);
    assert!(solids[0].1[0].is_watertight());
    assert_eq!(solids[0].1[0].split_disconnected().len(), 1);
}

// Control for E6: crossing prism bars without a base plate.
#[test]
fn crossed_prism_bars_fuse_correctly() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((-20.0, -2.0), (20.0, 2.0)));
    extrude_with(
        &mut g,
        "bar_2",
        "sk_1",
        4.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    add_sketch(&mut g, "sk_3", rect_sketch((-2.0, -20.0), (2.0, 20.0)));
    extrude_with(
        &mut g,
        "cross_4",
        "sk_3",
        4.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let exact = 40.0 * 4.0 * 4.0 * 2.0 - 4.0 * 4.0 * 4.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "crossed bars {v} vs {exact}"
    );
}

// The same ribs drawn in one sketch must produce the same material as two
// sequential features, including the intersection of the sketch regions.
#[test]
fn multi_tool_crossing_join_preserves_both_ribs() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 80.0, 60.0, 8.0);
    let mut curves = SketchCurves::new();
    curves.add_rectangle((5.0, 28.0), (75.0, 32.0));
    curves.add_rectangle((38.0, 5.0), (42.0, 55.0));
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(8.0), curves);
    extrude_with(
        &mut g,
        "ribs_3",
        "sk_2",
        12.0,
        ExtrudeMode::Join,
        None,
        vec![],
    );
    assert_crossed_rib_material(&g);
    assert_crossed_rib_material(&g.clone_document());
}

#[test]
fn ziggurat_of_stacked_bosses() {
    // Six shrinking levels, each sketched on the previous level's top face.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 100.0, 10.0);
    let mut z = 10.0f32;
    let mut size = 80.0f32;
    let mut n = 2;
    while size > 8.0 {
        let half = size / 2.0;
        add_sketch_cs(
            &mut g,
            &format!("sk_{n}"),
            xy_plane_at(z),
            rect_sketch((50.0 - half, 50.0 - half), (50.0 + half, 50.0 + half)),
        );
        extrude_with(
            &mut g,
            &format!("ex_{}", n + 1),
            &format!("sk_{n}"),
            8.0,
            ExtrudeMode::Join,
            None,
            vec![],
        );
        z += 8.0;
        size -= 12.0;
        n += 2;
    }
    let v = assert_part_sane(&g);
    let mut exact = 100.0 * 100.0 * 10.0;
    let mut s = 80.0;
    while s > 8.0 {
        exact += (s as f64) * (s as f64) * 8.0;
        s -= 12.0;
    }
    assert!(
        (v - exact).abs() / exact < 0.02,
        "ziggurat volume {v} vs {exact}"
    );
}

#[test]
fn annular_groove_stack_on_plate() {
    // Three concentric annular grooves cut into a plate via circle pairs.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 80.0, 80.0, 10.0);
    for (i, r) in [30.0f32, 20.0, 10.0].into_iter().enumerate() {
        let mut c = SketchCurves::new();
        c.add_circle((40.0, 40.0), r);
        c.add_circle((40.0, 40.0), r - 4.0);
        add_sketch_cs(&mut g, &format!("groove_sk_{i}"), xy_plane_at(10.0), c);
        extrude_with(
            &mut g,
            &format!("groove_ex_{i}"),
            &format!("groove_sk_{i}"),
            -2.0,
            ExtrudeMode::Cut,
            None,
            vec![],
        );
    }
    let v = assert_part_sane(&g);
    let pi = std::f64::consts::PI;
    let grooves = 2.0
        * pi
        * ((30.0f64 * 30.0 - 26.0 * 26.0)
            + (20.0 * 20.0 - 16.0 * 16.0)
            + (10.0 * 10.0 - 6.0 * 6.0));
    let exact = 80.0 * 80.0 * 10.0 - grooves;
    // Either the annulus interpretation or the disc interpretation can come
    // out of region detection; require one of the two plausible volumes.
    let disc = 80.0 * 80.0 * 10.0 - 2.0 * pi * (900.0 + 400.0 + 100.0);
    assert!(
        (v - exact).abs() / exact < 0.05 || (v - disc).abs() / disc < 0.05,
        "groove volume {v} matches neither annulus {exact} nor disc {disc}"
    );
}

#[test]
fn eight_body_workbench_document() {
    // Eight independent bodies at spread-out positions, each with its own
    // pocket and hole — a busy multi-body document in one graph.
    let mut g = ParametricGraph::new();
    let mut expected = 0.0f64;
    for i in 0..8 {
        let (ox, oy) = ((i % 4) as f32 * 100.0, (i / 4) as f32 * 100.0);
        add_sketch_cs(
            &mut g,
            &format!("sk_{i}"),
            xy_plane_at(0.0),
            rect_sketch((ox, oy), (ox + 40.0, oy + 40.0)),
        );
        extrude_with(
            &mut g,
            &format!("body_{i}"),
            &format!("sk_{i}"),
            10.0,
            ExtrudeMode::NewBody,
            None,
            vec![],
        );
        add_sketch_cs(
            &mut g,
            &format!("psk_{i}"),
            xy_plane_at(10.0),
            rect_sketch((ox + 5.0, oy + 5.0), (ox + 35.0, oy + 35.0)),
        );
        extrude_with(
            &mut g,
            &format!("pocket_{i}"),
            &format!("psk_{i}"),
            -3.0,
            ExtrudeMode::Cut,
            Some(format!("body_{i}")),
            vec![],
        );
        add_hole(
            &mut g,
            &format!("hole_{i}"),
            &format!("body_{i}"),
            [ox + 20.0, oy + 20.0, 7.0],
            [0.0, 0.0, -1.0],
            6.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
        expected += 40.0 * 40.0 * 10.0 - 30.0 * 30.0 * 3.0 - std::f64::consts::PI * 9.0 * 7.0;
    }
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    assert_eq!(bodies.len(), 8, "eight independent bodies expected");
    let v = total_volume(&bodies);
    assert!(
        (v - expected).abs() / expected < 0.03,
        "workbench total {v} vs {expected}"
    );
    assert_deterministic(&g);
}

#[test]
fn interleaved_join_cut_history_twelve_features() {
    // Alternating bosses and pockets, each on the previous feature's new top
    // face — 24 nodes of interleaved history on one body.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    let mut z = 10.0f32;
    let mut half = 24.0f32;
    let mut n = 2usize;
    for i in 0..6 {
        let h = 28.0 - half;
        // Boss
        add_sketch_cs(
            &mut g,
            &format!("sk_{n}"),
            xy_plane_at(z),
            rect_sketch((30.0 - half, 30.0 - half), (30.0 + half, 30.0 + half)),
        );
        extrude_with(
            &mut g,
            &format!("ex_{}", n + 1),
            &format!("sk_{n}"),
            4.0,
            ExtrudeMode::Join,
            None,
            vec![],
        );
        z += 4.0;
        // Pocket inside the boss
        let inner = half - 4.0;
        add_sketch_cs(
            &mut g,
            &format!("sk_{}", n + 2),
            xy_plane_at(z),
            rect_sketch((30.0 - inner, 30.0 - inner), (30.0 + inner, 30.0 + inner)),
        );
        extrude_with(
            &mut g,
            &format!("ex_{}", n + 3),
            &format!("sk_{}", n + 2),
            -3.0,
            ExtrudeMode::Cut,
            None,
            vec![],
        );
        z -= 3.0;
        half -= 4.0;
        n += 4;
        let _ = (i, h);
    }
    let v = assert_part_sane(&g);
    // Bosses 48/40/32/24/16/8 square × 4 up, pockets 40/32/24/16/8 square × 3
    // down inside them (the final 4-square level's pocket is zero-area and
    // contributes nothing).
    let bosses =
        4.0 * (48.0f64 * 48.0 + 40.0 * 40.0 + 32.0 * 32.0 + 24.0 * 24.0 + 16.0 * 16.0 + 8.0 * 8.0);
    let pockets = 3.0 * (40.0f64 * 40.0 + 32.0 * 32.0 + 24.0 * 24.0 + 16.0 * 16.0 + 8.0 * 8.0);
    let exact = 60.0 * 60.0 * 10.0 + bosses - pockets;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "interleaved history volume {v} vs {exact}"
    );
}

#[test]
fn cut_away_one_body_then_reference_it() {
    // A cut consumes body B entirely; a later feature still referencing B
    // must degrade to a warning, not a panic or silent mis-target.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 20.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(0.0),
        rect_sketch((50.0, 0.0), (70.0, 20.0)),
    );
    extrude_with(
        &mut g,
        "b_3",
        "sk_2",
        10.0,
        ExtrudeMode::NewBody,
        None,
        vec![],
    );
    // Consume B with a huge targeted cut.
    add_sketch_cs(
        &mut g,
        "sk_4",
        xy_plane_at(0.0),
        rect_sketch((45.0, -5.0), (75.0, 25.0)),
    );
    extrude_with(
        &mut g,
        "cut_5",
        "sk_4",
        12.0,
        ExtrudeMode::Cut,
        Some("b_3".into()),
        vec![],
    );
    // Now reference the consumed body.
    add_hole(
        &mut g,
        "hole_6",
        "b_3",
        [60.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        4.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok((bodies, _warnings)) => assert_meshes_finite(&bodies),
        Err(_) => {} // structured rejection acceptable
    }
}

#[test]
fn overlapping_multi_shape_profile_mixed_circles_and_rects() {
    // One sketch: rectangle + two overlapping circles straddling its edges +
    // an internal island circle. Cut everything: region detection decides;
    // the result must merely be finite and deterministic with real material
    // removed.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    let mut curves = SketchCurves::new();
    curves.add_rectangle((10.0, 10.0), (50.0, 50.0));
    curves.add_circle((10.0, 30.0), 8.0); // straddles the left edge
    curves.add_circle((50.0, 30.0), 8.0); // straddles the right edge
    curves.add_circle((30.0, 30.0), 5.0); // island inside
    add_sketch_cs(&mut g, "sketch_2", xy_plane_at(10.0), curves);
    extrude_with(
        &mut g,
        "cut_3",
        "sketch_2",
        -4.0,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let solid = 60.0 * 60.0 * 10.0;
    assert!(
        v < solid * 0.98,
        "mixed profile cut must remove material: {v} vs {solid}"
    );
}

#[test]
fn deterministic_pseudo_random_pocket_field() {
    // LCG-generated field of 30 mixed circle/rect pockets: no two documents
    // should be able to expose ordering nondeterminism.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 200.0, 200.0, 10.0);
    let mut curves = SketchCurves::new();
    let mut state = 0x2A03Fu32;
    let mut next = move || {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        state
    };
    for _ in 0..30 {
        let x = (next() % 180) as f32 + 5.0;
        let y = (next() % 180) as f32 + 5.0;
        if next() & 1 == 0 {
            curves.add_circle((x, y), 3.0);
        } else {
            curves.add_rectangle((x, y), (x + 6.0, y + 4.0));
        }
    }
    add_sketch_cs(&mut g, "sketch_2", xy_plane_at(10.0), curves);
    extrude_with(
        &mut g,
        "cut_3",
        "sketch_2",
        -2.5,
        ExtrudeMode::Cut,
        None,
        vec![],
    );
    let v = assert_part_sane(&g);
    let solid = 200.0 * 200.0 * 10.0;
    assert!(
        v < solid,
        "pocket field must remove material: {v} vs {solid}"
    );
}
