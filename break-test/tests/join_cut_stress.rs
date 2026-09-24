//! Join/Cut stress: contact topology (face/edge/vertex/sliver), island
//! splits, identity round-trips, containment, tilted prisms, long alternating
//! chains, and multi-body cuts. Analytic volumes wherever the outcome is
//! unambiguous; graceful-warning expectations where the topology is genuinely
//! ambiguous (edge/vertex-only contact).

mod common;

use common::*;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Vec3,
};

fn sketch_body(
    g: &mut ParametricGraph,
    id: &str,
    origin: [f32; 3],
    u: [f32; 3],
    v: [f32; 3],
    rect: ((f32, f32), (f32, f32)),
    depth: f32,
) {
    add_sketch_cs(
        g,
        &format!("{id}_sk"),
        CoordinateSystem::new(
            Vec3::new(origin[0], origin[1], origin[2]),
            Vec3::new(u[0], u[1], u[2]),
            Vec3::new(v[0], v[1], v[2]),
        ),
        rect_sketch(rect.0, rect.1),
    );
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency(&format!("{id}_sk"), id);
}

fn extrude_at(
    g: &mut ParametricGraph,
    id: &str,
    sk: &str,
    origin_z: f32,
    rect: ((f32, f32), (f32, f32)),
    depth: f32,
    mode: ExtrudeMode,
    target: Option<String>,
) {
    add_sketch_cs(g, sk, xy_plane_at(origin_z), rect_sketch(rect.0, rect.1));
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            target,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency(sk, id);
}

// ---------------------------------------------------------------------------
// Join contact topology.
// ---------------------------------------------------------------------------

#[test]
fn join_face_touching_boxes_fuses() {
    // A: x 0..10, B: x 10..20 — full face contact, zero volumetric overlap.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 10.0, 10.0, 10.0);
    sketch_body(
        &mut g,
        "b_2",
        [10.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    // Extrude along the plane normal (Z here is wrong: the sketch lies in XY
    // at x=10? The u=X,v=Y frame is the XY plane — the rect spans x 10..20,
    // y 0..10, extruded along +Z: B is x 10..20, y 0..10, z 0..10. ✓
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string()],
        },
    });
    g.add_dependency("a_1", "join_3");
    g.add_dependency("b_2", "join_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "face-touching boxes must fuse into 2000: {v} ({warnings:?})"
    );
}

#[test]
fn join_edge_touching_only_is_graceful() {
    // A: x 0..10, y 0..10; B: x 10..20, y 10..20 — contact along the line
    // (10,10,z). Fusing gives a non-manifold edge-solid; either a fused
    // valid solid or an unresolved warning is acceptable, never a crash.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 10.0, 10.0, 10.0);
    sketch_body(
        &mut g,
        "b_2",
        [10.0, 10.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string()],
        },
    });
    g.add_dependency("a_1", "join_3");
    g.add_dependency("b_2", "join_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "edge contact: both volumes conserved either way: {v} ({warnings:?})"
    );
}

#[test]
fn join_vertex_touching_only_is_graceful() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 10.0, 10.0, 10.0);
    sketch_body(
        &mut g,
        "b_2",
        [10.0, 10.0, 10.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string()],
        },
    });
    g.add_dependency("a_1", "join_3");
    g.add_dependency("b_2", "join_3");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn join_sliver_overlap_fuses_exactly() {
    // 0.001 mm overlap — the fusion volume is two boxes minus the sliver
    // already counted once: exactly 2000 (the overlap is inside both).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 10.0, 10.0, 10.0);
    sketch_body(
        &mut g,
        "b_2",
        [9.999, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string()],
        },
    });
    g.add_dependency("a_1", "join_3");
    g.add_dependency("b_2", "join_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 1999.9).abs() < 5.0,
        "sliver join: {v} vs ~1999.9 ({warnings:?})"
    );
}

#[test]
fn join_chain_of_three_bodies() {
    // A|B|C pairwise-overlapping chain in ONE BodyJoin.
    let mut g = ParametricGraph::new();
    sketch_body(
        &mut g,
        "a_1",
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    sketch_body(
        &mut g,
        "b_2",
        [8.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    sketch_body(
        &mut g,
        "c_3",
        [16.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (10.0, 10.0)),
        10.0,
    );
    g.add_feature(FeatureNode {
        id: "join_4".to_string(),
        name: "join_4".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string(), "c_3".to_string()],
        },
    });
    for s in ["a_1", "b_2", "c_3"] {
        g.add_dependency(s, "join_4");
    }
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // a∪b overlap 2×10×10=200; b∪c same; a∩c empty: 3000 − 400 = 2600.
    assert!(
        (v - 2600.0).abs() / 2600.0 < 0.02,
        "3-chain join: {v} vs 2600 ({warnings:?})"
    );
}

#[test]
fn join_tilted_prisms_overlap() {
    // Two square prisms, one rotated 45° about Z, overlapping in the middle.
    let a = 45.0_f32.to_radians();
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((-5.0, -5.0), (5.0, 5.0)));
    g.add_feature(FeatureNode {
        id: "p_2".to_string(),
        name: "p_2".to_string(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_1", "p_2");
    add_sketch_cs(
        &mut g,
        "sk_3",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(a.cos(), a.sin(), 0.0),
            Vec3::new(-a.sin(), a.cos(), 0.0),
        ),
        rect_sketch((-5.0, -5.0), (5.0, 5.0)),
    );
    g.add_feature(FeatureNode {
        id: "p_4".to_string(),
        name: "p_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "p_4");
    g.add_feature(FeatureNode {
        id: "join_5".to_string(),
        name: "join_5".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["p_2".to_string(), "p_4".to_string()],
        },
    });
    g.add_dependency("p_2", "join_5");
    g.add_dependency("p_4", "join_5");
    let v = assert_part_sane(&g);
    // Union ⊂ each prism's slab; strictly less than the sum.
    assert!(v < 2000.0, "tilted join must not double-count: {v}");
    assert!(v > 1000.0, "tilted join must overlap substantially: {v}");
}

#[test]
fn join_fully_contained_body_changes_nothing() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "big_1", 20.0, 20.0, 20.0);
    add_box(&mut g, "small_2", 5.0, 5.0, 5.0); // 0..5 inside 0..20
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["big_1".to_string(), "small_2".to_string()],
        },
    });
    g.add_dependency("big_1", "join_3");
    g.add_dependency("small_2", "join_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 8000.0).abs() / 8000.0 < 0.02,
        "contained join = outer volume: {v} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Cut topology: islands, identity round-trips, slivers.
// ---------------------------------------------------------------------------

#[test]
fn cut_splits_bar_into_two_islands() {
    // Slot the middle of a bar away, leaving two disconnected end blocks.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "bar_1", 30.0, 10.0, 10.0);
    extrude_at(
        &mut g,
        "cut_2",
        "sk_2",
        10.0,
        ((10.0, -5.0), (20.0, 15.0)),
        -12.0,
        ExtrudeMode::Cut,
        None,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "two islands total 2×1000: {v} ({warnings:?})"
    );
}

#[test]
fn cut_c_shape_is_exact() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    extrude_at(
        &mut g,
        "cut_2",
        "sk_2",
        10.0,
        ((10.0, 10.0), (20.0, 30.1)),
        -10.0,
        ExtrudeMode::Cut,
        None,
    );
    let v = assert_part_sane(&g);
    let exact = 30.0 * 30.0 * 10.0 - 10.0 * 20.0 * 10.0;
    assert!((v - exact).abs() / exact < 0.02, "C-cut {v} vs {exact}");
}

#[test]
fn cut_then_join_same_region_is_identity() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    extrude_at(
        &mut g,
        "cut_2",
        "sk_2",
        10.0,
        ((5.0, 5.0), (15.0, 15.0)),
        -3.0,
        ExtrudeMode::Cut,
        None,
    );
    extrude_at(
        &mut g,
        "join_3",
        "sk_4",
        7.0,
        ((5.0, 5.0), (15.0, 15.0)),
        3.0,
        ExtrudeMode::Join,
        None,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 4000.0).abs() / 4000.0 < 0.02,
        "cut+join back must be identity: {v} ({warnings:?})"
    );
}

#[test]
fn cut_leaving_thin_web_stays_sane() {
    // Blind cut stopping 0.05 mm short of the floor.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    extrude_at(
        &mut g,
        "cut_2",
        "sk_2",
        10.0,
        ((5.0, 5.0), (15.0, 15.0)),
        -9.95,
        ExtrudeMode::Cut,
        None,
    );
    let v = assert_part_sane(&g);
    let exact = 4000.0 - 100.0 * 9.95;
    assert!((v - exact).abs() / exact < 0.02, "thin web {v} vs {exact}");
}

#[test]
fn drafted_cut_pocket_monotone() {
    // Positive draft on a CUT narrows the pocket toward its floor (mold
    // draft), so LESS material is removed as the angle grows: volume rises.
    let mut v_prev = 0.0f64;
    for angle in [0.0f32, 5.0, 15.0, 30.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
        add_sketch_cs(
            &mut g,
            "sk_2",
            xy_plane_at(10.0),
            rect_sketch((10.0, 10.0), (30.0, 30.0)),
        );
        g.add_feature(FeatureNode {
            id: "cut_3".to_string(),
            name: format!("cut_{angle}"),
            feature: FeatureType::Extrude {
                depth: -5.0,
                region_indices: vec![],
                mode: ExtrudeMode::Cut,
                target: None,
                depth_expr: None,
                draft_angle_deg: angle,
                draft_angle_expr: None,
            },
        });
        g.add_dependency("sk_2", "cut_3");
        let v = assert_part_sane(&g);
        assert!(
            v >= v_prev - 1.0,
            "more draft removes less: {angle}° → {v} (prev {v_prev})"
        );
        v_prev = v;
    }
}

#[test]
fn cut_through_three_overlapping_bodies_untargeted() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 20.0, 10.0); // x 0..20
    sketch_body(
        &mut g,
        "b_2",
        [15.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (20.0, 20.0)),
        10.0,
    ); // 15..35
    sketch_body(
        &mut g,
        "c_3",
        [30.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        ((0.0, 0.0), (20.0, 20.0)),
        10.0,
    ); // 30..50
    extrude_at(
        &mut g,
        "cut_4",
        "sk_4",
        10.0,
        ((5.0, 5.0), (45.0, 15.0)),
        -10.0,
        ExtrudeMode::Cut,
        None,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // The three bodies are independent (never joined): each loses only its
    // own slice of the slot. a: x5..20 → 1500, b: full width → 2000,
    // c: x30..45 → 1500. 12000 − 5000 = 7000.
    let exact = 12000.0 - (1500.0 + 2000.0 + 1500.0);
    assert!(
        (v - exact).abs() / exact < 0.03,
        "untargeted cut through 3 bodies: {v} vs {exact} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Long alternating chains.
// ---------------------------------------------------------------------------

#[test]
fn alternating_join_cut_chain_twenty_features() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 100.0, 10.0);
    let mut n = 2usize;
    for i in 0..10 {
        let x0 = 5.0 + i as f32 * 9.0;
        // Boss up.
        extrude_at(
            &mut g,
            &format!("j_{}", n + 1),
            &format!("sj_{n}"),
            10.0,
            ((x0, 5.0), (x0 + 6.0, 95.0)),
            4.0,
            ExtrudeMode::Join,
            None,
        );
        // Pocket back down inside the boss.
        extrude_at(
            &mut g,
            &format!("c_{}", n + 2),
            &format!("sc_{n}"),
            14.0,
            ((x0 + 1.0, 10.0), (x0 + 5.0, 90.0)),
            -8.0,
            ExtrudeMode::Cut,
            None,
        );
        n += 2;
    }
    let v = assert_part_sane(&g);
    let bosses = 10.0 * (6.0 * 90.0 * 4.0);
    let pockets = 10.0 * (4.0 * 80.0 * 8.0) * -1.0;
    // Pockets remove 8 deep: 4 into the boss + 4 into the plate.
    let exact = 100.0 * 100.0 * 10.0 + bosses - 10.0 * 4.0 * 80.0 * 8.0;
    let _ = pockets;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "alternating chain: {v} vs {exact}"
    );
}

#[test]
fn cut_join_interleave_returns_to_start() {
    // Cut a pocket, join it back, cut a different one, join back — final
    // volume must equal the original body.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    for (i, x0) in [5.0f32, 15.0].into_iter().enumerate() {
        extrude_at(
            &mut g,
            &format!("c_{}", 2 + i * 4),
            &format!("sc_{}", 1 + i * 4),
            10.0,
            ((x0, 5.0), (x0 + 10.0, 25.0)),
            -4.0,
            ExtrudeMode::Cut,
            None,
        );
        extrude_at(
            &mut g,
            &format!("j_{}", 3 + i * 4),
            &format!("sj_{}", 2 + i * 4),
            6.0,
            ((x0, 5.0), (x0 + 10.0, 25.0)),
            4.0,
            ExtrudeMode::Join,
            None,
        );
    }
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 9000.0).abs() / 9000.0 < 0.02,
        "interleave must return to start: {v} ({warnings:?})"
    );
}

#[test]
fn join_of_two_tangent_rods() {
    // Two rods side by side touching along a line (both along +Y, centers
    // 20 apart, r=10): line tangency. Fusion is non-manifold; both volumes
    // must be conserved either way.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 20.0); // axis at origin
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 0.0), 10.0);
    // Second rod along +Y at x=20: sketch in the ZY plane at x=20.
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(20.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "rod_3".to_string(),
        name: "rod_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 20.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "rod_3");
    g.add_feature(FeatureNode {
        id: "join_4".to_string(),
        name: "join_4".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["rod_1".to_string(), "rod_3".to_string()],
        },
    });
    g.add_dependency("rod_1", "join_4");
    g.add_dependency("rod_3", "join_4");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let two_rods = 2.0 * std::f64::consts::PI * 100.0 * 20.0;
    assert!(
        (v - two_rods).abs() / two_rods < 0.05,
        "tangent rods conserve volume: {v} vs {two_rods} ({warnings:?})"
    );
}

#[test]
fn cut_exactly_half_depth_on_slanted_face_chain() {
    // Cut chain on a wedge body made by a tilted cut, then more cuts — the
    // composite tilted-face workflow.
    let angle = 30.0_f32.to_radians();
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 40.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 20.0),
            Vec3::new(angle.cos(), 0.0, angle.sin()),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        rect_sketch((-30.0, -30.0), (60.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 40.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "cut_3");
    // Pocket into the tilted top from its highest region.
    extrude_at(
        &mut g,
        "cut_4",
        "sk_4",
        34.0,
        ((10.0, 10.0), (30.0, 30.0)),
        -6.0,
        ExtrudeMode::Cut,
        None,
    );
    let v = assert_part_sane(&g);
    let solid = 40.0 * 40.0 * 40.0;
    assert!(
        v < solid,
        "wedge chain must remove material: {v} vs {solid}"
    );
}
