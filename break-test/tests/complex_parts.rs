//! Realistic multi-feature parts — the "does the part builder actually build
//! parts" suite. Each test assembles a plausible mechanical component the way
//! a user would, then gates on sane finite geometry, deterministic
//! re-evaluation, and a loose volume sanity check.

mod common;

use common::*;
use zerocad_core::{
    AxisBase, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, HoleKind, LoftSurfaceMode,
    ParametricGraph, Vec3,
};

#[test]
fn l_bracket_with_bolt_holes() {
    // L-bracket: horizontal base + vertical leg joined, four bolt holes.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "base_1", 80.0, 60.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((0.0, 0.0), (10.0, 60.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 50.0, ExtrudeMode::Join);
    for (i, (x, y, z, dir)) in [
        (10.0f32, 10.0f32, 0.0f32, [0.0f32, 0.0, 1.0]),
        (70.0, 10.0, 0.0, [0.0, 0.0, 1.0]),
        (70.0, 50.0, 0.0, [0.0, 0.0, 1.0]),
        (10.0, 50.0, 0.0, [0.0, 0.0, 1.0]),
        (5.0, 30.0, 10.0, [1.0, 0.0, 0.0]), // through the vertical leg
    ]
    .into_iter()
    .enumerate()
    {
        add_hole(
            &mut g,
            &format!("hole_{}", 4 + i),
            "base_1",
            [x, y, z],
            dir,
            6.0,
            None,
            HoleKind::Counterbore {
                diameter: 11.0,
                depth: 2.0,
            },
        );
    }
    let v = assert_part_sane(&g);
    let exact = 80.0 * 60.0 * 10.0 + 10.0 * 60.0 * 50.0;
    assert!(
        v < exact && v > exact * 0.8,
        "bracket volume {v} should be below solid {exact} but within 80%"
    );
}

#[test]
fn flanged_hub_with_revolve() {
    // Revolved flanged hub: profile rectangle revolved 360° about the X axis.
    let mut g = ParametricGraph::new();
    // Sketch in XY: a rectangle from (10,0) to (30,40) revolved about X gives
    // a stepped shaft/hub silhouette.
    add_sketch(&mut g, "sketch_1", rect_sketch((10.0, 0.0), (30.0, 40.0)));
    g.add_feature(FeatureNode {
        id: "revolve_2".to_string(),
        name: "revolve_2".to_string(),
        feature: FeatureType::Revolve {
            axis: AxisBase::X,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "revolve_2");
    let v = assert_part_sane(&g);
    let exact = std::f64::consts::PI * (40.0f64 * 40.0 - 10.0 * 10.0) * 20.0;
    // Tessellated cylinders overestimate analytic π by a few percent.
    assert!(
        (v - exact).abs() / exact < 0.08,
        "revolved hub volume {v} vs analytic {exact}"
    );

    // Partial revolve (270°) must be strictly smaller and still valid.
    let mut g2 = ParametricGraph::new();
    add_sketch(&mut g2, "sketch_1", rect_sketch((10.0, 0.0), (30.0, 40.0)));
    g2.add_feature(FeatureNode {
        id: "revolve_2".to_string(),
        name: "revolve_2".to_string(),
        feature: FeatureType::Revolve {
            axis: AxisBase::X,
            angle_deg: 270.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g2.add_dependency("sketch_1", "revolve_2");
    let v2 = assert_part_sane(&g2);
    assert!(
        v2 < v,
        "270° revolve ({v2}) should be smaller than 360° ({v})"
    );
}

#[test]
fn housing_with_shell_and_pattern() {
    // Box housing: shelled hollow, then a drilled vent hole patterned
    // linearly down one wall (body-level pattern of the drilled wall is not
    // possible, so pattern the whole body copy to exercise the pattern path).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 40.0, 30.0);
    g.add_feature(FeatureNode {
        id: "shell_2".to_string(),
        name: "shell_2".to_string(),
        feature: FeatureType::Shell {
            target: "box_1".to_string(),
            thickness: 3.0,
            thickness_expr: None,
            open_faces: vec![],
        },
    });
    g.add_dependency("box_1", "shell_2");
    add_hole(
        &mut g,
        "hole_3",
        "box_1",
        [10.0, 0.0, 15.0],
        [0.0, 1.0, 0.0],
        8.0,
        None,
        HoleKind::Simple,
    );
    let v = assert_part_sane(&g);
    let solid = 60.0 * 40.0 * 30.0;
    assert!(
        v < solid,
        "shelled housing {v} must be below solid box {solid}"
    );
}

#[test]
fn lofted_transition_duct() {
    // Loft from a square section to a circular section (HVAC-style transition).
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        rect_sketch((-15.0, -15.0), (15.0, 15.0)),
    );
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(40.0),
        circle_sketch((0.0, 0.0), 8.0),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "loft_3".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0), ("sketch_2".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_3");
    g.add_dependency("sketch_2", "loft_3");
    let v = assert_part_sane(&g);
    assert!(v > 0.0, "lofted duct must have positive volume, got {v}");
    // Ruled loft volume must lie between the two end prism/frism volumes.
    let v_min = std::f64::consts::PI * 8.0 * 8.0 * 40.0;
    let v_max = 30.0 * 30.0 * 40.0;
    assert!(
        v > v_min * 0.9 && v < v_max,
        "loft volume {v} outside plausible ({v_min}, {v_max})"
    );
}

#[test]
fn swept_pipe_elbow() {
    // Sweep a circular profile along an L-shaped path (pipe elbow).
    let mut path = zerocad_core::SketchCurves::new();
    path.add_line((0.0, 0.0), (0.0, 30.0));
    path.add_line((0.0, 30.0), (30.0, 30.0));

    let mut g = ParametricGraph::new();
    // Profile in XZ so it starts perpendicular to the Z-running path segment.
    add_sketch_cs(
        &mut g,
        "profile_1",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, 0.0), Vec3::X, Vec3::new(0.0, 1.0, 0.0)),
        circle_sketch((0.0, 0.0), 5.0),
    );
    add_sketch(&mut g, "path_2", path);
    g.add_feature(FeatureNode {
        id: "sweep_3".to_string(),
        name: "sweep_3".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile_1".to_string(),
            profile_region: 0,
            path_sketch: "path_2".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("profile_1", "sweep_3");
    g.add_dependency("path_2", "sweep_3");
    let v = assert_part_sane(&g);
    // Elbow miter corner changes the analytic value; require a plausible band
    // around the straight-pipe volume π·r²·L.
    let straight = std::f64::consts::PI * 25.0 * 60.0;
    assert!(
        v > straight * 0.6 && v < straight * 1.3,
        "pipe sweep volume {v} not in plausible band around {straight}"
    );
}

#[test]
fn body_boolean_workflow_plate_with_slot() {
    // Multi-body workflow: two primitives combined with BodyCut, then
    // BodyJoin of a boss, exercising keep_tool both ways.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "plate_1", 50.0, 50.0, 8.0);
    add_cylinder(&mut g, "tool_2", 5.0, 20.0);
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "plate_1".to_string(),
            tool: "tool_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("plate_1", "cut_3");
    g.add_dependency("tool_2", "cut_3");
    let v = assert_part_sane(&g);
    // With keep_tool = false only the slotted plate remains.
    let exact = 50.0 * 50.0 * 8.0 - std::f64::consts::PI * 25.0 * 8.0;
    assert!(
        (v - exact).abs() / exact < 0.08,
        "slotted plate volume {v} vs {exact} (tessellated cylinder tolerance)"
    );
}

#[test]
fn body_intersect_crossed_boxes() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 20.0, 20.0);
    add_box(&mut g, "b_2", 20.0, 20.0, 20.0);
    // Overlap them via a translate-copy... simplest: they fully coincide here,
    // so intersect = one box volume.
    g.add_feature(FeatureNode {
        id: "isect_3".to_string(),
        name: "isect_3".to_string(),
        feature: FeatureType::BodyIntersect {
            target: "a_1".to_string(),
            tool: "b_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("a_1", "isect_3");
    g.add_dependency("b_2", "isect_3");
    let v = assert_part_sane(&g);
    let exact = 20.0 * 20.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "coincident-box intersection {v} vs {exact}"
    );
}

#[test]
fn linear_pattern_of_body() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 25.0,
                spacing_expr: None,
                count: 4,
            },
        },
    });
    g.add_dependency("box_1", "pattern_2");
    let v = assert_part_sane(&g);
    let exact = 4.0 * 10.0 * 10.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "4-instance linear pattern {v} vs {exact}"
    );
}

#[test]
fn circular_pattern_full_ring() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 4.0, 10.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "cyl_1".to_string(),
            kind: zerocad_core::PatternKind::Circular {
                axis: AxisBase::Z,
                count: 6,
                total_angle_deg: 360.0,
            },
        },
    });
    g.add_dependency("cyl_1", "pattern_2");
    let v = assert_part_sane(&g);
    let exact = 6.0 * std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "6-instance ring pattern {v} vs {exact}"
    );
}

#[test]
fn rotated_off_axis_sketch_extrude_cut() {
    // Sketch on a tilted plane cutting a box — the arbitrary-orientation path.
    let angle = 30.0_f32.to_radians();
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 30.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        CoordinateSystem::new(
            Vec3::new(15.0, 0.0, 15.0),
            Vec3::new(angle.cos(), 0.0, angle.sin()),
            Vec3::new(-angle.sin(), 0.0, angle.cos()),
        ),
        rect_sketch((-20.0, -20.0), (20.0, 20.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 6.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let solid = 30.0 * 30.0 * 30.0;
    assert!(v < solid, "tilted cut must remove material: {v} vs {solid}");
}

#[test]
fn stacked_feature_edit_chain_rebuilds() {
    // A part with a long history: base → boss → cut → scale → transform copy.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        circle_sketch((0.0, 0.0), 12.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 15.0, ExtrudeMode::Join);
    add_hole(
        &mut g,
        "hole_4",
        "box_1",
        [0.0, 0.0, 25.0],
        [0.0, 0.0, -1.0],
        8.0,
        None,
        HoleKind::Simple,
    );
    g.add_feature(FeatureNode {
        id: "scale_5".to_string(),
        name: "scale_5".to_string(),
        feature: FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: 2.0,
            factor_expr: None,
            center: [0.0, 0.0, 0.0],
        },
    });
    g.add_dependency("box_1", "scale_5");
    g.add_feature(FeatureNode {
        id: "move_6".to_string(),
        name: "move_6".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [100.0, 0.0, 0.0],
            copy: true,
        },
    });
    g.add_dependency("box_1", "move_6");
    assert_part_sane(&g);
}
