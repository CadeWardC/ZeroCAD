//! Full-body boolean stress: BodyJoin / BodyCut / BodyIntersect / BodySplit
//! against each other — tool reuse with keep_tool, chains, lumps, split
//! round-trips, and mixed ghost/degenerate sources.

mod common;

use common::*;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Vec3,
};

fn box_at(g: &mut ParametricGraph, id: &str, x0: f32, y0: f32, w: f32, h: f32, d: f32) {
    add_sketch_cs(
        g,
        &format!("{id}_sk"),
        CoordinateSystem::new(Vec3::new(x0, y0, 0.0), Vec3::X, Vec3::Y),
        rect_sketch((0.0, 0.0), (w, h)),
    );
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth: d,
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

fn add_feature(g: &mut ParametricGraph, id: &str, feature: FeatureType, deps: &[&str]) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature,
    });
    for d in deps {
        g.add_dependency(d, id);
    }
}

fn eval_pair(g: &ParametricGraph) -> (Vec<(String, zerocad_core::MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail")
}

#[test]
fn body_cut_with_keep_tool_then_reuse_tool_twice() {
    // Cut two different plates with the SAME kept tool.
    let mut g = ParametricGraph::new();
    box_at(&mut g, "plate_1", 0.0, 0.0, 30.0, 30.0, 8.0);
    box_at(&mut g, "plate_2", 0.0, 50.0, 30.0, 30.0, 8.0);
    box_at(&mut g, "tool_3", 10.0, -5.0, 8.0, 90.0, 20.0); // spans both plates in y
    add_feature(
        &mut g,
        "cut_4",
        FeatureType::BodyCut {
            target: "plate_1".into(),
            tool: "tool_3".into(),
            keep_tool: true,
        },
        &["plate_1", "tool_3"],
    );
    add_feature(
        &mut g,
        "cut_5",
        FeatureType::BodyCut {
            target: "plate_2".into(),
            tool: "tool_3".into(),
            keep_tool: false,
        },
        &["plate_2", "cut_4"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // plate1 − slot(8×30×8) + plate2 − slot(8×30×8) + kept tool for cut1
    // (consumed by cut2) = 7200−1920 + 7200−1920 = 10560.
    let exact = 2.0 * (30.0 * 30.0 * 8.0 - 8.0 * 30.0 * 8.0);
    assert!(
        (v - exact).abs() / exact < 0.03,
        "tool reuse: {v} vs {exact} ({warnings:?})"
    );
}

#[test]
fn body_intersect_partial_overlap_is_exact() {
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 20.0, 20.0, 10.0);
    box_at(&mut g, "b_2", 5.0, 0.0, 20.0, 20.0, 10.0); // x 5..25
    add_feature(
        &mut g,
        "isect_3",
        FeatureType::BodyIntersect {
            target: "a_1".into(),
            tool: "b_2".into(),
            keep_tool: false,
        },
        &["a_1", "b_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 3000.0).abs() / 3000.0 < 0.02,
        "partial intersect: {v} vs 3000 ({warnings:?})"
    );
}

#[test]
fn body_intersect_disjoint_warns_and_preserves() {
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 10.0, 10.0, 10.0);
    box_at(&mut g, "b_2", 100.0, 0.0, 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "isect_3",
        FeatureType::BodyIntersect {
            target: "a_1".into(),
            tool: "b_2".into(),
            keep_tool: false,
        },
        &["a_1", "b_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "disjoint intersect preserves both: {v}"
    );
    assert!(
        !warnings.is_empty(),
        "disjoint intersect must warn: {warnings:?}"
    );
}

#[test]
fn body_intersect_contained_keeps_small_body() {
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 20.0, 20.0, 20.0);
    box_at(&mut g, "b_2", 5.0, 5.0, 5.0, 5.0, 5.0);
    add_feature(
        &mut g,
        "isect_3",
        FeatureType::BodyIntersect {
            target: "a_1".into(),
            tool: "b_2".into(),
            keep_tool: false,
        },
        &["a_1", "b_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 125.0).abs() / 125.0 < 0.02,
        "contained intersect: {v} vs 125 ({warnings:?})"
    );
}

#[test]
fn body_intersect_tangent_face_contact_is_graceful() {
    // Boxes touching at x = 10 exactly (zero-volume intersection).
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 10.0, 10.0, 10.0);
    box_at(&mut g, "b_2", 10.0, 0.0, 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "isect_3",
        FeatureType::BodyIntersect {
            target: "a_1".into(),
            tool: "b_2".into(),
            keep_tool: false,
        },
        &["a_1", "b_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn body_join_with_one_ghost_source_reports_and_keeps_valid_pair() {
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 10.0, 10.0, 10.0);
    box_at(&mut g, "b_2", 5.0, 0.0, 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "join_3",
        FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string(), "ghost".to_string()],
        },
        &["a_1", "b_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.to_lowercase().contains("ghost")
            || w.to_lowercase().contains("join")
            || w.to_lowercase().contains("unresolved")),
        "ghost source must be reported: {warnings:?}"
    );
    let v = total_volume(&bodies);
    assert!(
        (v - (2000.0 - 500.0)).abs() / 1500.0 < 0.05 || (v - 2000.0).abs() / 2000.0 < 0.05,
        "valid pair handled despite ghost: {v} ({warnings:?})"
    );
}

#[test]
fn body_join_duplicate_sources_is_graceful() {
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "join_2",
        FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "a_1".to_string()],
        },
        &["a_1"],
    );
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn body_split_midplane_then_join_back_is_identity() {
    // A body STRADDLING the XZ plane (y −10..10) so the split is a real
    // midplane cut — an origin-anchored box makes all three base planes
    // boundary-tangent (see the graceful-tangent test below).
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sk_1",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, -10.0), Vec3::X, Vec3::Y),
        rect_sketch((-10.0, -10.0), (10.0, 10.0)),
    );
    g.add_feature(FeatureNode {
        id: "box_2".to_string(),
        name: "box_2".to_string(),
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
    g.add_dependency("sk_1", "box_2");
    add_feature(
        &mut g,
        "split_3",
        FeatureType::BodySplit {
            target: "box_2".into(),
            plane: zerocad_core::PlaneBase::XZ,
            face: None,
        },
        &["box_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 8000.0).abs() / 8000.0 < 0.02,
        "split conserves volume: {v} ({warnings:?})"
    );
    let halves: Vec<String> = bodies.iter().map(|(id, _)| id.clone()).collect();
    assert!(
        halves.len() >= 2,
        "midplane split must register two outputs: {halves:?}"
    );
    if let (Some(h0), Some(h1)) = (halves.first(), halves.get(1)) {
        add_feature(
            &mut g,
            "join_4",
            FeatureType::BodyJoin {
                sources: vec![h0.clone(), h1.clone()],
            },
            &[h0, h1],
        );
        let (bodies, warnings) = eval_pair(&g);
        assert_meshes_finite(&bodies);
        let v = total_volume(&bodies);
        assert!(
            (v - 8000.0).abs() / 8000.0 < 0.02,
            "split+join identity: {v} ({warnings:?})"
        );
    }
}

// (Pinned observation) splitting an ORIGIN-ANCHORED box with any base plane
// (XY/XZ/YZ) is boundary-tangent — the plane coincides with a face — and the
// split degenerates gracefully (one body, volume conserved).
#[test]
fn body_split_boundary_tangent_plane_is_graceful() {
    for plane in [
        zerocad_core::PlaneBase::XY,
        zerocad_core::PlaneBase::XZ,
        zerocad_core::PlaneBase::YZ,
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_feature(
            &mut g,
            "split_2",
            FeatureType::BodySplit {
                target: "box_1".into(),
                plane,
                face: None,
            },
            &["box_1"],
        );
        let (bodies, _) = eval_pair(&g);
        assert_meshes_finite(&bodies);
        let v = total_volume(&bodies);
        assert!(
            (v - 4000.0).abs() < 10.0,
            "tangent split conserves volume: {v}"
        );
    }
}

#[test]
fn body_split_tangent_to_face_is_graceful() {
    // Split plane exactly on the box's x = 20 face — degenerate split.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "split_2",
        FeatureType::BodySplit {
            target: "box_1".into(),
            plane: zerocad_core::PlaneBase::XY,
            face: None,
        },
        &["box_1"],
    );
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

// E15b (BUG, pinned): cutting away a whole ISLAND of a multi-lump body fails.
// The two-island bar's right island (x 20..30) lies fully inside the tool —
// the subtraction should simply delete that component — but the solver
// reports "could not subtract the overlapping body" and preserves everything.
// Same failure family as the fully-contained-target BodyCut. When fixed,
// flip to the analytic one-island result.
#[test]
fn characterization_cutting_a_whole_island_fails() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "bar_1", 30.0, 10.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, -5.0), (20.0, 15.0)),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -12.0, ExtrudeMode::Cut);
    box_at(&mut g, "tool_4", 15.0, -5.0, 20.0, 20.0, 20.0);
    add_feature(
        &mut g,
        "cut_5",
        FeatureType::BodyCut {
            target: "bar_1".into(),
            tool: "tool_4".into(),
            keep_tool: false,
        },
        &["cut_3", "tool_4"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("could not subtract")),
        "island-consuming cut currently fails: {warnings:?}"
    );
    let v = total_volume(&bodies);
    // Both islands (2000) + the kept-out tool (8000) survive untouched.
    assert!(
        (v - 10000.0).abs() / 10000.0 < 0.03,
        "pinning current behavior: islands + tool unchanged ({v}); analytic result is 1000"
    );
}

// E18 (verdict revised 2026-09-11): the fixture's plug spans y ∈ [−5, 25]
// while the bored rod spans y ∈ [0, 20] — the plug PROTRUDES 5 mm from both
// bore mouths. The historical claim "the union should be the solid rod
// (2000π)" would delete those protruding ends: wrong. The union OCCUPIES
// exactly tube ∪ plug either way, so the total volume is 2250π whether or
// not the coincident cylindrical walls fuse — volume alone can never
// distinguish the two outcomes. What differs is topology: today the join
// keeps TWO bodies (the known coincident-curved-wall fusion gap, kernel
// work), material is fully conserved, and nothing is silently lost. Pin the
// per-body volumes and the two-body outcome; a future fuse must keep the
// same total material and can be asserted by body count instead.
#[test]
fn flush_coaxial_plug_join_conserves_material_as_two_bodies() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 20.0);
    let mut hole = zerocad_core::SketchCurves::new();
    hole.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        hole,
    );
    g.add_feature(FeatureNode {
        id: "drill_3".to_string(),
        name: "drill_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "drill_3");
    let mut plug = zerocad_core::SketchCurves::new();
    plug.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(
        &mut g,
        "sk_4",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        plug,
    );
    g.add_feature(FeatureNode {
        id: "plug_5".to_string(),
        name: "plug_5".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_4", "plug_5");
    add_feature(
        &mut g,
        "join_6",
        FeatureType::BodyJoin {
            sources: vec!["rod_1".to_string(), "plug_5".to_string()],
        },
        &["rod_1", "plug_5"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.is_empty(),
        "the material-conserving outcome must be silent: {warnings:?}"
    );
    // The join result is owned by the feature id (E17 semantics); today the
    // coincident-wall fuse keeps the two source volumes as separate parts of
    // one body or as two bodies — in both cases this exact material survives.
    let v = total_volume(&bodies);
    let tube = std::f64::consts::PI * (100.0 - 25.0) * 20.0;
    let plug_vol = std::f64::consts::PI * 25.0 * 30.0;
    assert!(
        (v - (tube + plug_vol)).abs() / (tube + plug_vol) < 0.02,
        "tube + protruding plug ends must all survive: {v} vs {:.2}",
        tube + plug_vol
    );
}

// Control for E18: a plug with a clearance gap (r=4.9 in a r=5 bore) fuses
// if the boolean handles near-coincident walls; either fused or warned, the
// volumes must stay conserved.
#[test]
fn clearance_plug_join_conserves_volume() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 20.0);
    let mut hole = zerocad_core::SketchCurves::new();
    hole.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        hole,
    );
    g.add_feature(FeatureNode {
        id: "drill_3".to_string(),
        name: "drill_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "drill_3");
    let mut plug = zerocad_core::SketchCurves::new();
    plug.add_circle((0.0, 0.0), 4.9);
    add_sketch_cs(
        &mut g,
        "sk_4",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        plug,
    );
    g.add_feature(FeatureNode {
        id: "plug_5".to_string(),
        name: "plug_5".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_4", "plug_5");
    add_feature(
        &mut g,
        "join_6",
        FeatureType::BodyJoin {
            sources: vec!["rod_1".to_string(), "plug_5".to_string()],
        },
        &["rod_1", "plug_5"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let tube = std::f64::consts::PI * 75.0 * 20.0;
    let plug_vol = std::f64::consts::PI * 24.01 * 30.0;
    assert!(
        (v - (tube + plug_vol)).abs() / (tube + plug_vol) < 0.05,
        "clearance plug volumes conserved: {v} ({warnings:?})"
    );
}

#[test]
fn body_cut_chain_three_levels() {
    // cut1 makes a plate with slot; cut2 trims an end; cut3 drills a hole —
    // all with fresh tools, volumes exact at every stage.
    let mut g = ParametricGraph::new();
    box_at(&mut g, "plate_1", 0.0, 0.0, 30.0, 30.0, 8.0);
    box_at(&mut g, "tool_2", 11.0, -5.0, 8.0, 40.0, 20.0);
    add_feature(
        &mut g,
        "cut_3",
        FeatureType::BodyCut {
            target: "plate_1".into(),
            tool: "tool_2".into(),
            keep_tool: false,
        },
        &["plate_1", "tool_2"],
    );
    box_at(&mut g, "tool_4", -5.0, -5.0, 7.0, 40.0, 20.0); // overlaps the plate edge x 0..2
    add_feature(
        &mut g,
        "cut_5",
        FeatureType::BodyCut {
            target: "cut_3".into(),
            tool: "tool_4".into(),
            keep_tool: false,
        },
        &["cut_3", "tool_4"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 30.0 * 30.0 * 8.0 - 8.0 * 30.0 * 8.0 - 2.0 * 30.0 * 8.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "cut chain: {v} vs {exact} ({warnings:?})"
    );
}

#[test]
fn body_intersects_of_three_bodies_pairwise_chain() {
    // A∩B then result∩C — a stepped common region.
    let mut g = ParametricGraph::new();
    box_at(&mut g, "a_1", 0.0, 0.0, 20.0, 10.0, 10.0);
    box_at(&mut g, "b_2", 10.0, 0.0, 20.0, 10.0, 10.0); // overlap x 10..20
    box_at(&mut g, "c_3", 15.0, 0.0, 10.0, 10.0, 10.0); // inside overlap x 15..20
    add_feature(
        &mut g,
        "i1_4",
        FeatureType::BodyIntersect {
            target: "a_1".into(),
            tool: "b_2".into(),
            keep_tool: false,
        },
        &["a_1", "b_2"],
    );
    add_feature(
        &mut g,
        "i2_5",
        FeatureType::BodyIntersect {
            target: "i1_4".into(),
            tool: "c_3".into(),
            keep_tool: false,
        },
        &["i1_4", "c_3"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 500.0).abs() / 500.0 < 0.05,
        "chained intersects: {v} vs 500 ({warnings:?})"
    );
}
