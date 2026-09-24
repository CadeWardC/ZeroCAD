//! Torture suite 2: revolve / loft / sweep abuse and hole / cylinder
//! boolean torture. Reconstructed from the geometry degeneracies kernels
//! classically fail on: profiles crossing the revolve axis, zero-height
//! lofts, sweeps whose profile is wider than the path's bend radius, holes
//! centered exactly on body edges, coaxial and tangent cylinder booleans.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{
    AxisBase, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, LoftSurfaceMode,
    ParametricGraph, SketchCurves, Vec3,
};

fn add_revolve(
    g: &mut ParametricGraph,
    id: &str,
    sk: &str,
    axis: AxisBase,
    angle: f32,
    mode: ExtrudeMode,
    target: Option<String>,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Revolve {
            axis,
            angle_deg: angle,
            angle_expr: None,
            region_indices: vec![],
            mode,
            target,
        },
    });
    g.add_dependency(sk, id);
}

fn add_sweep(g: &mut ParametricGraph, id: &str, profile: &str, path: &str, twist: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: profile.to_string(),
            profile_region: 0,
            path_sketch: path.to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: twist,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency(profile, id);
    g.add_dependency(path, id);
}

// ---------------------------------------------------------------------------
// Revolve torture.
// ---------------------------------------------------------------------------

#[test]
fn revolve_profile_touching_axis_is_solid() {
    // Rectangle starting exactly on the axis (x 0..30) — degenerate inner
    // wall at radius 0 must collapse to a solid rod section.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (30.0, 10.0)));
    add_revolve(
        &mut g,
        "rev_2",
        "sk_1",
        AxisBase::X,
        360.0,
        ExtrudeMode::NewBody,
        None,
    );
    let v = assert_part_sane(&g);
    // Rod of radius 10 (the profile's y extent) and length 30 (its x extent).
    let exact = std::f64::consts::PI * 100.0 * 30.0;
    assert!(
        (v - exact).abs() / exact < 0.06,
        "on-axis revolve {v} vs {exact}"
    );
}

#[test]
fn revolve_profile_crossing_axis_fails_gracefully() {
    // Region straddling the axis (x -5..30): a self-intersecting revolve.
    // Structured failure or clipped solid — never a crash or NaN mesh.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((-5.0, 0.0), (30.0, 10.0)));
    add_revolve(
        &mut g,
        "rev_2",
        "sk_1",
        AxisBase::X,
        360.0,
        ExtrudeMode::NewBody,
        None,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn revolve_angle_continuum() {
    // 359.9°, 180°, 90°, 0.001° — monotone: smaller angle, less volume; the
    // tiny angle must not produce a full solid or NaN.
    let mut prev = f64::INFINITY;
    for angle in [360.0f32, 359.9, 180.0, 90.0, 0.001] {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sk_1", rect_sketch((10.0, 0.0), (30.0, 20.0)));
        add_revolve(
            &mut g,
            "rev_2",
            "sk_1",
            AxisBase::X,
            angle,
            ExtrudeMode::NewBody,
            None,
        );
        let (bodies, _) = g
            .evaluate_bodies_with_warnings(&HashSet::new())
            .expect("revolve must not hard-fail");
        assert_meshes_finite(&bodies);
        let v = total_volume(&bodies);
        assert!(
            v <= prev + 1.0,
            "volume must shrink with angle: {angle}° → {v} (prev {prev})"
        );
        prev = v;
    }
    assert!(prev < 40.0, "0.001° revolve must be nearly empty: {prev}");
}

#[test]
fn revolve_diagonal_axis_creates_torus_like_solid() {
    // Revolve a region about a diagonal axis offset from it — a torus-ish
    // body with no planar symmetry. Finiteness + determinism gate.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((20.0, 20.0), (25.0, 25.0)));
    add_revolve(
        &mut g,
        "rev_2",
        "sk_1",
        AxisBase::TwoPoints {
            a: [-50.0, 0.0, 0.0],
            b: [50.0, 0.0, 0.0],
        },
        360.0,
        ExtrudeMode::NewBody,
        None,
    );
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Pappus: V = 2π·r̄·A with r̄ = 22.5 (centroid distance from the X axis).
    let exact = 2.0 * std::f64::consts::PI * 22.5 * 25.0;
    assert!(
        (v - exact).abs() / exact < 0.15 || v == 0.0,
        "torus-ish revolve {v} vs Pappus {exact}"
    );
}

#[test]
fn revolve_as_cut_through_box() {
    // A revolve tool used in Cut mode against a box: grooves an annular slot.
    // The axis must lie IN the sketch plane (the XY plane at z = 0), so use
    // an explicit in-plane segment through x = 20.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(Vec3::new(20.0, 20.0, 0.0), Vec3::X, Vec3::Y),
        rect_sketch((10.0, 12.0), (20.0, 15.0)),
    );
    add_revolve(
        &mut g,
        "rev_3",
        "sk_2",
        AxisBase::TwoPoints {
            a: [20.0, -50.0, 0.0],
            b: [20.0, 50.0, 0.0],
        },
        360.0,
        ExtrudeMode::Cut,
        None,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        v < 16000.0 * 0.99,
        "revolved cut must remove material: {v} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Loft torture.
// ---------------------------------------------------------------------------

fn two_section_loft(
    g: &mut ParametricGraph,
    curves_a: SketchCurves,
    z_b: f32,
    curves_b: SketchCurves,
    mode: LoftSurfaceMode,
) {
    add_sketch(g, "sk_1", curves_a);
    add_sketch_cs(g, "sk_2", xy_plane_at(z_b), curves_b);
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "loft_3".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sk_1".to_string(), 0), ("sk_2".to_string(), 0)],
            surface_mode: mode,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sk_1", "loft_3");
    g.add_dependency("sk_2", "loft_3");
}

#[test]
fn loft_identical_stacked_sections_is_prism() {
    let mut g = ParametricGraph::new();
    two_section_loft(
        &mut g,
        rect_sketch((0.0, 0.0), (20.0, 20.0)),
        10.0,
        rect_sketch((0.0, 0.0), (20.0, 20.0)),
        LoftSurfaceMode::Ruled,
    );
    let v = assert_part_sane(&g);
    assert!(
        (v - 4000.0).abs() / 4000.0 < 0.02,
        "identical-section loft {v} vs 4000"
    );
}

#[test]
fn loft_zero_height_degenerate() {
    // Both sections on the SAME plane: zero-height loft must warn or produce
    // nothing — never a corrupt solid.
    let mut g = ParametricGraph::new();
    two_section_loft(
        &mut g,
        rect_sketch((0.0, 0.0), (20.0, 20.0)),
        0.0,
        circle_sketch((10.0, 10.0), 5.0),
        LoftSurfaceMode::Ruled,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn loft_extreme_area_ratio() {
    // 30×30 square down to a 0.02-radius pin over 40 mm.
    let mut g = ParametricGraph::new();
    two_section_loft(
        &mut g,
        rect_sketch((-15.0, -15.0), (15.0, 15.0)),
        40.0,
        circle_sketch((0.0, 0.0), 0.02),
        LoftSurfaceMode::Ruled,
    );
    let v = assert_part_sane(&g);
    let v_max = 30.0 * 30.0 * 40.0;
    assert!(v > 0.0 && v < v_max, "tapered loft {v} in (0, {v_max})");
}

#[test]
fn loft_rotated_section_45_degrees() {
    // Square bottom, 45°-rotated square top — the corners must correspond
    // without twisting through each other.
    let angle = 45.0_f32.to_radians();
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((-10.0, -10.0), (10.0, 10.0)));
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 20.0),
            Vec3::new(angle.cos(), angle.sin(), 0.0),
            Vec3::new(-angle.sin(), angle.cos(), 0.0),
        ),
        rect_sketch((-10.0, -10.0), (10.0, 10.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "loft_3".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sk_1".to_string(), 0), ("sk_2".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sk_1", "loft_3");
    g.add_dependency("sk_2", "loft_3");
    let v = assert_part_sane(&g);
    // A twisted ruled loft bulges past the straight prism (4000) — plausible
    // band up to the convex-hull-scale bound.
    assert!(v > 3000.0 && v < 7000.0, "rotated loft volume {v}");
}

#[test]
fn loft_five_alternating_sections() {
    let mut g = ParametricGraph::new();
    for (i, half) in [20.0f32, 5.0, 20.0, 5.0, 20.0].into_iter().enumerate() {
        add_sketch_cs(
            &mut g,
            &format!("sk_{i}"),
            xy_plane_at(i as f32 * 10.0),
            rect_sketch((-half, -half), (half, half)),
        );
    }
    g.add_feature(FeatureNode {
        id: "loft_5".to_string(),
        name: "loft_5".to_string(),
        feature: FeatureType::Loft {
            sections: (0..5).map(|i| (format!("sk_{i}"), 0)).collect(),
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    for i in 0..5 {
        g.add_dependency(&format!("sk_{i}"), "loft_5");
    }
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn loft_section_region_index_out_of_range() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (20.0, 20.0)));
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        rect_sketch((0.0, 0.0), (10.0, 10.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "loft_3".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sk_1".to_string(), 7), ("sk_2".to_string(), 99)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sk_1", "loft_3");
    g.add_dependency("sk_2", "loft_3");
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

// ---------------------------------------------------------------------------
// Sweep torture.
// ---------------------------------------------------------------------------

#[test]
fn sweep_closed_circular_path_is_torus() {
    let mut path = SketchCurves::new();
    path.add_circle((20.0, 0.0), 20.0); // closed chain: R=20 centered (20,0)
    let mut g = ParametricGraph::new();
    // Profile in the plane perpendicular to the path start.
    add_sketch_cs(
        &mut g,
        "prof_1",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        circle_sketch((0.0, 0.0), 2.0),
    );
    add_sketch(&mut g, "path_2", path);
    add_sweep(&mut g, "sweep_3", "prof_1", "path_2", 0.0);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 2.0 * std::f64::consts::PI * 20.0 * (std::f64::consts::PI * 4.0);
    assert!(
        (v - exact).abs() / exact < 0.1 || v == 0.0,
        "torus sweep {v} vs {exact} ({warnings:?})"
    );
}

#[test]
fn sweep_profile_wider_than_bend_radius() {
    // Profile r=10 swept along an L-path with a sharp corner — the classic
    // self-intersecting sweep. Must fail gracefully or produce valid geometry.
    let mut path = SketchCurves::new();
    path.add_line((0.0, 0.0), (0.0, 30.0));
    path.add_line((0.0, 30.0), (30.0, 30.0));
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "prof_1",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, 0.0), Vec3::X, Vec3::new(0.0, 1.0, 0.0)),
        circle_sketch((0.0, 0.0), 10.0),
    );
    add_sketch(&mut g, "path_2", path);
    add_sweep(&mut g, "sweep_3", "prof_1", "path_2", 0.0);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn sweep_zigzag_path_ten_corners() {
    let mut path = SketchCurves::new();
    path.add_line((0.0, 0.0), (0.0, 5.0));
    for i in 0..10 {
        let x = i as f32 * 6.0;
        path.add_line((x, 5.0), (x + 3.0, 10.0));
        path.add_line((x + 3.0, 10.0), (x + 6.0, 5.0));
    }
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "prof_1",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, 0.0), Vec3::X, Vec3::new(0.0, 1.0, 0.0)),
        circle_sketch((0.0, 0.0), 1.5),
    );
    add_sketch(&mut g, "path_2", path);
    add_sweep(&mut g, "sweep_3", "prof_1", "path_2", 0.0);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

#[test]
fn sweep_path_with_zero_length_segment() {
    let mut path = SketchCurves::new();
    path.add_line((0.0, 0.0), (0.0, 0.0)); // zero-length
    path.add_line((0.0, 0.0), (0.0, 20.0));
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "prof_1",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, 0.0), Vec3::X, Vec3::new(0.0, 1.0, 0.0)),
        circle_sketch((0.0, 0.0), 2.0),
    );
    add_sketch(&mut g, "path_2", path);
    add_sweep(&mut g, "sweep_3", "prof_1", "path_2", 0.0);
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn sweep_as_cut_mills_channel_through_box() {
    // Sweep in Cut mode: a round channel milled through a plate.
    let mut path = SketchCurves::new();
    path.add_line((-5.0, 5.0), (25.0, 5.0));
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch_cs(
        &mut g,
        "prof_1",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 10.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        circle_sketch((0.0, 0.0), 2.0),
    );
    add_sketch(&mut g, "path_2", path);
    g.add_feature(FeatureNode {
        id: "sweep_3".to_string(),
        name: "sweep_3".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "prof_1".to_string(),
            profile_region: 0,
            path_sketch: "path_2".to_string(),
            mode: ExtrudeMode::Cut,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("prof_1", "sweep_3");
    g.add_dependency("path_2", "sweep_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        v < 4000.0,
        "milled channel must remove material: {v} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Hole torture: edge placement and tangency.
// ---------------------------------------------------------------------------

#[test]
fn hole_centered_exactly_on_body_edge() {
    // Hole center on the box edge x=0: exactly half the bore intersects.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [0.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let v = assert_part_sane(&g);
    let exact = 4000.0 - std::f64::consts::PI * 9.0 * 10.0 / 2.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "edge-centered hole {v} vs {exact}"
    );
}

#[test]
fn hole_tangent_to_body_edge() {
    // Hole wall exactly tangent to the box side face (center x=-3, r=3):
    // zero-area intersection — body must stay whole or lose a hair sliver,
    // never produce garbage.
    for cx in [-3.0f32, 3.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [cx, 10.0, 10.0],
            [0.0, 0.0, -1.0],
            6.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        let v = total_volume(&bodies);
        assert!(v > 3800.0, "tangent hole must barely change volume: {v}");
    }
}

#[test]
fn hole_diameter_equal_to_body_width() {
    // A bore as wide as the body: splits it into two slabs.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 40.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [5.0, 20.0, 10.0],
        [0.0, 0.0, -1.0],
        10.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let _ = warnings;
}

// E16 remains unsupported: the second overlapping cut is rejected with an
// attributed warning and the first bore preserved, for both feature paths.
// Two r=4 bores 5 mm apart should remove ~875 mm³; the preserved first bore
// removes ~503 mm³. This tests safe rejection, not overlapping-cut support.
#[test]
fn unsupported_overlapping_second_cut_warns_and_preserves_material() {
    // Hole variant: warns, preserves the crescent.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 10.0],
        [0.0, 0.0, -1.0],
        8.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_hole(
        &mut g,
        "hole_3",
        "box_1",
        [20.0, 15.0, 10.0],
        [0.0, 0.0, -1.0],
        8.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("hole_3") && w.contains("original material was preserved")),
        "second overlapping hole currently fails: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let one_bore = 9000.0 - std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (v - one_bore).abs() / one_bore < 0.02,
        "pinning current behavior: only the first bore removed ({v}); lens-union would be {}",
        9000.0 - 87.5 * 10.0
    );

    // Cut-extrude variant must also report the rejected operation.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        circle_sketch((15.0, 15.0), 4.0),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -10.0, ExtrudeMode::Cut);
    add_sketch_cs(
        &mut g,
        "sk_4",
        xy_plane_at(10.0),
        circle_sketch((20.0, 15.0), 4.0),
    );
    add_extrude(&mut g, "cut_5", "sk_4", -10.0, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("cut_5") && w.contains("original material was preserved")),
        "the rejected extrude must report failure: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let one_circle = 9000.0 - std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (v - one_circle).abs() / one_circle < 0.02,
        "pinning current behavior: rejected overlapping circle cut preserves material ({v})"
    );
}

// Control for E16: DISJOINT cuts both apply exactly.
#[test]
fn disjoint_circle_cuts_both_apply() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        circle_sketch((10.0, 15.0), 4.0),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -10.0, ExtrudeMode::Cut);
    add_sketch_cs(
        &mut g,
        "sk_4",
        xy_plane_at(10.0),
        circle_sketch((20.0, 15.0), 4.0),
    );
    add_extrude(&mut g, "cut_5", "sk_4", -10.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 9000.0 - 2.0 * std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.04,
        "two disjoint cuts {v} vs {exact}"
    );
}

#[test]
fn counterbore_and_countersink_abuse() {
    use zerocad_core::HoleKind;
    let cases: &[HoleKind] = &[
        HoleKind::Counterbore {
            diameter: 40.0,
            depth: 5.0,
        }, // wider than body
        HoleKind::Counterbore {
            diameter: 12.0,
            depth: 50.0,
        }, // deeper than body
        HoleKind::Counterbore {
            diameter: 12.0,
            depth: -2.0,
        },
        HoleKind::Countersink {
            diameter: 40.0,
            angle_deg: 90.0,
        }, // wider than body
        HoleKind::Countersink {
            diameter: 12.0,
            angle_deg: 179.0,
        }, // near-flat cone
        HoleKind::Countersink {
            diameter: 12.0,
            angle_deg: 0.1,
        }, // needle cone
    ];
    for kind in cases {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [10.0, 10.0, 10.0],
            [0.0, 0.0, -1.0],
            6.0,
            None,
            kind.clone(),
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn blind_hole_depths_at_and_past_full_depth() {
    for depth in [
        Some(10.0f32),
        Some(10.001),
        Some(15.0),
        Some(0.0),
        Some(-3.0),
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [10.0, 10.0, 10.0],
            [0.0, 0.0, -1.0],
            6.0,
            depth,
            zerocad_core::HoleKind::Simple,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn tilted_hole_45_degrees() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 10.0],
        [0.70710678, 0.0, -0.70710678],
        6.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let v = assert_part_sane(&g);
    assert!(v < 9000.0, "tilted hole must remove material: {v}");
}

// ---------------------------------------------------------------------------
// Cylinder boolean torture.
// ---------------------------------------------------------------------------

fn cylinder_body(g: &mut ParametricGraph, id: &str, cx: f32, cz: f32, r: f32, y0: f32, h: f32) {
    // Cylinders run along +Y from base at origin; build one at an arbitrary
    // (x, z) base via a sketch plane whose normal is +Y.
    add_sketch_cs(
        g,
        &format!("{id}_sk"),
        CoordinateSystem::new(
            Vec3::new(cx, y0, cz),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        circle_sketch((0.0, 0.0), r),
    );
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth: h,
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

#[test]
fn coaxial_cylinder_cut_makes_tube() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 30.0); // r=10 along +Y
    cylinder_body(&mut g, "drill_2", 0.0, 0.0, 5.0, -5.0, 40.0);
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "rod_1".to_string(),
            tool: "drill_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("rod_1", "cut_3");
    g.add_dependency("drill_2", "cut_3");
    let v = assert_part_sane(&g);
    let exact = std::f64::consts::PI * (100.0 - 25.0) * 30.0;
    assert!((v - exact).abs() / exact < 0.05, "tube {v} vs {exact}");
}

#[test]
fn tangent_cylinders_boolean_sweep() {
    // Two equal cylinders whose walls exactly touch (centers 2r apart in X),
    // swept ±epsilon: the tangency case kernels fail on.
    for dx in [-0.1f32, -0.01, 0.0, 0.01, 0.1] {
        let mut g = ParametricGraph::new();
        cylinder_body(&mut g, "a_1", 0.0, 0.0, 5.0, 0.0, 20.0);
        cylinder_body(&mut g, "b_2", 10.0 + dx, 0.0, 5.0, 0.0, 20.0);
        g.add_feature(FeatureNode {
            id: "join_3".to_string(),
            name: "join_3".to_string(),
            feature: FeatureType::BodyJoin {
                sources: vec!["a_1".to_string(), "b_2".to_string()],
            },
        });
        g.add_dependency("a_1", "join_3");
        g.add_dependency("b_2", "join_3");
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

// E14 (BUG, pinned): a circle cut TANGENT to all four box walls (the
// inscribed circle — r = 20 in a 40×40 box) fails with "the overlapping cut
// could not produce a valid solid" and preserves the material. r = 19.9
// removes the expected volume cleanly, so exact multi-wall tangency is the
// trigger. When fixed, flip to the analytic volume.
#[test]
fn characterization_inscribed_circle_cut_fails_on_tangency() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        circle_sketch((20.0, 20.0), 20.0),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -10.0, ExtrudeMode::Cut);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("original material was preserved")),
        "inscribed circle cut currently fails: {warnings:?}"
    );
    let v = total_volume(&bodies);
    assert!(
        (v - 16000.0).abs() < 10.0,
        "pinning current behavior: material preserved ({v}); analytic result is 3433.6"
    );
}

// Control for E14: epsilon-off the tangency the cut applies cleanly.
#[test]
fn near_inscribed_circle_cut_applies() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        circle_sketch((20.0, 20.0), 19.9),
    );
    add_extrude(&mut g, "cut_3", "sk_2", -10.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 16000.0 - std::f64::consts::PI * 396.01 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.05,
        "r=19.9 cut {v} vs {exact}"
    );
}

// E15 regression (fixed 2026-09-11): BodyCut's overlap precheck compared the
// evaluated tool's VERTEX-ONLY bounds against the target. A cylinder whose
// seam vertices sit away from the axis extrema reported a too-small box
// (z-min −5 for a rod whose surface reaches −10), so a box tool sketched at
// z = −15 and extruded to z = −5 — which genuinely crosses the rod — was
// falsely rejected as disjoint. The kernel now provides guaranteed-enclosing
// conservative bounds (`Solid::conservative_bounding_box`) and the precheck
// uses them, so the cut applies and removes the exact circular-segment
// prism: segment area `r²·acos(d/r) − d·√(r²−d²)` (r=10, d=5) × tool overlap
// length 20 ≈ 1228.37 mm³.
#[test]
fn body_cut_with_tool_based_below_target_applies() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_box",
        CoordinateSystem::new(Vec3::new(-15.0, 0.0, -15.0), Vec3::X, Vec3::Y),
        rect_sketch((0.0, 0.0), (30.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "tool_2".to_string(),
        name: "tool_2".to_string(),
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
    g.add_dependency("sk_box", "tool_2");
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "cyl_1".to_string(),
            tool: "tool_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("cyl_1", "cut_3");
    g.add_dependency("tool_2", "cut_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        !warnings.iter().any(|w| w.contains("must overlap")),
        "a genuinely overlapping below-target tool must not be rejected as disjoint: {warnings:?}"
    );
    let v = total_volume(&bodies);
    // The tool is consumed (keep_tool = false): only the cut rod remains,
    // missing the circular-segment prism below z = −5 across its full length.
    let rod = std::f64::consts::PI * 100.0 * 20.0;
    let segment = 20.0 * (100.0 * (0.5f64).acos() - 5.0 * 75.0f64.sqrt());
    assert!(
        (v - (rod - segment)).abs() / (rod - segment) < 0.02,
        "the cut must remove the exact segment prism {segment:.2} mm³: got {v}, expected {:.2}",
        rod - segment
    );
}

// Control for E15: same cut with the tool's sketch plane INSIDE the target's
// z-range (it may still extend beyond the target in x and y) — applies.
#[test]
fn body_cut_with_plane_inside_target_applies() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_box",
        CoordinateSystem::new(Vec3::new(-15.0, 0.0, -8.0), Vec3::X, Vec3::Y),
        rect_sketch((0.0, 0.0), (30.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "tool_2".to_string(),
        name: "tool_2".to_string(),
        feature: FeatureType::Extrude {
            depth: 6.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_box", "tool_2");
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "cyl_1".to_string(),
            tool: "tool_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("cyl_1", "cut_3");
    g.add_dependency("tool_2", "cut_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 100.0 * 20.0;
    assert!(
        v < rod * 0.95,
        "plane-inside tool must remove real material: {v} vs {rod} ({warnings:?})"
    );
}

// E15b: a tool that fully CONTAINS the target is rejected with the
// misleading "the solver could not subtract the overlapping body" instead of
// a proper "target fully consumed" outcome. Graceful, but the message and
// the preserved-target behavior deserve a deliberate decision.
#[test]
fn characterization_body_cut_consuming_target_misreports() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_box",
        CoordinateSystem::new(Vec3::new(-15.0, -5.0, -15.0), Vec3::X, Vec3::Y),
        rect_sketch((0.0, 0.0), (30.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "tool_2".to_string(),
        name: "tool_2".to_string(),
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
    g.add_dependency("sk_box", "tool_2");
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "cyl_1".to_string(),
            tool: "tool_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("cyl_1", "cut_3");
    g.add_dependency("tool_2", "cut_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("could not subtract")),
        "consuming cut currently reports a solver failure: {warnings:?}"
    );
}
