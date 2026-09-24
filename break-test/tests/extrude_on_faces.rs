//! Extrude-on-face and sketch-on-face usage: the bread-and-butter parametric
//! workflow (draw on a body face, boss/pocket it) plus the weird variants —
//! sketches overhanging the face, planes floating mid-body, face-on-face
//! chains, and sliver extrusions. Reconstructed from common CAD tutorial
//! parts and the geometry degeneracies real users hit.

mod common;

use common::*;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Vec3,
};

fn face_sketch(
    g: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    curves: zerocad_core::SketchCurves,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: true,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
}

// ---------------------------------------------------------------------------
// General usage: draw on a face, boss or pocket.
// ---------------------------------------------------------------------------

#[test]
fn boss_on_top_face_of_box() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    // Sketch on the top face (z = 10).
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (30.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 15.0, ExtrudeMode::Join);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 + 20.0 * 20.0 * 15.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "boss volume {v} vs {exact}"
    );
}

#[test]
fn pocket_on_top_face_of_box() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (30.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -4.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 - 20.0 * 20.0 * 4.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "pocket volume {v} vs {exact}"
    );
}

#[test]
fn boss_on_side_face_extruded_sideways() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    // Side face at x = 40 (boxes are anchored at the origin): a YZ-plane
    // sketch whose normal (+X) points away from the box.
    face_sketch(
        &mut g,
        "sketch_2",
        CoordinateSystem::new(
            Vec3::new(40.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        rect_sketch((0.0, 0.0), (20.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 8.0, ExtrudeMode::Join);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 + 20.0 * 10.0 * 8.0;
    assert!((v - exact).abs() / exact < 0.02, "side boss {v} vs {exact}");
}

#[test]
fn through_cut_on_side_face() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        CoordinateSystem::new(
            Vec3::new(20.0, 0.0, 5.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        circle_sketch((0.0, 0.0), 4.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 40.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 - std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "side through-hole {v} vs {exact}"
    );
}

#[test]
fn sketch_on_cylinder_top_face_pocket() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 15.0, 20.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(20.0),
        circle_sketch((0.0, 0.0), 8.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -5.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let solid = std::f64::consts::PI * 225.0 * 20.0;
    assert!(
        v < solid,
        "cylinder pocket must remove material: {v} vs {solid}"
    );
}

// ---------------------------------------------------------------------------
// Weird placements: overhangs, floating planes, mid-body cuts.
// ---------------------------------------------------------------------------

#[test]
fn sketch_overhanging_the_face() {
    // Rectangle larger than the face it sits on — the extrusion overhangs in
    // Join mode (material grows past the wall) and in Cut mode (removes a
    // notch wider than the body). Both must produce sane solids.
    for mode in [ExtrudeMode::Join, ExtrudeMode::Cut] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        face_sketch(
            &mut g,
            "sketch_2",
            xy_plane_at(10.0),
            rect_sketch((-10.0, 5.0), (30.0, 15.0)),
        );
        add_extrude(
            &mut g,
            "extrude_3",
            "sketch_2",
            if mode == ExtrudeMode::Join { 5.0 } else { -3.0 },
            mode,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        // Independently seeded hash tables must not change merged loop starts.
        for _ in 0..8 {
            assert_deterministic(&g);
        }
    }
}

#[test]
fn sketch_floating_above_the_face() {
    // The sketch plane is 2 mm above the actual face — the join leaves an
    // internal void / dangling boss. Whatever comes back must be finite.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(12.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn midplane_sketch_cut_through_body_interior() {
    // Sketch plane strictly inside the body (z = 5 of a 10-tall box spanning
    // x,y in 0..20), cut upward: removes the entire upper half exactly.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(5.0),
        rect_sketch((-5.0, -5.0), (25.0, 25.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 20.0 * 20.0 * 5.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "half-box cut {v} vs {exact}"
    );
}

#[test]
fn sliver_extrusion_off_face() {
    // 1e-3 deep pocket: the guard against zero/thin cuts must keep geometry.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -0.001, ExtrudeMode::Cut);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn extrude_both_directions_with_two_features() {
    // One sketch used by two extrudes: a boss up and a boss down from the
    // same top-face plane (the plane cuts the box at z = 10, the downward
    // extrude joins material already inside the box).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
    add_extrude(&mut g, "extrude_4", "sketch_2", -3.0, ExtrudeMode::Cut);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn draft_on_face_boss() {
    // Drafted boss on a face: positive draft contracts the far section.
    for angle in [2.0f32, 10.0, 30.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
        face_sketch(
            &mut g,
            "sketch_2",
            xy_plane_at(10.0),
            rect_sketch((10.0, 10.0), (30.0, 30.0)),
        );
        add_extrude_full(
            &mut g,
            "extrude_3",
            "sketch_2",
            15.0,
            ExtrudeMode::Join,
            None,
            angle,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

// ---------------------------------------------------------------------------
// Face-on-face chains: pocket in a pocket, sketch on a boss top.
// ---------------------------------------------------------------------------

#[test]
fn pocket_in_pocket_chain() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 20.0);
    // First pocket: 4 deep from z = 20.
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(20.0),
        rect_sketch((5.0, 5.0), (35.0, 35.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -4.0, ExtrudeMode::Cut);
    // Second pocket on the new floor (z = 16), inside the first.
    face_sketch(
        &mut g,
        "sketch_4",
        xy_plane_at(16.0),
        rect_sketch((12.0, 12.0), (28.0, 28.0)),
    );
    add_extrude(&mut g, "extrude_5", "sketch_4", -6.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 20.0 - 30.0 * 30.0 * 4.0 - 16.0 * 16.0 * 6.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "stepped pocket {v} vs {exact}"
    );
}

#[test]
fn sketch_on_top_of_boss_then_hole() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (30.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 10.0, ExtrudeMode::Join);
    // New top face at z = 20: drill through boss + base.
    face_sketch(
        &mut g,
        "sketch_4",
        xy_plane_at(20.0),
        circle_sketch((20.0, 20.0), 5.0),
    );
    add_extrude(&mut g, "extrude_5", "sketch_4", -20.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let exact = 40.0 * 40.0 * 10.0 + 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 25.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "boss with through-hole {v} vs {exact}"
    );
}

#[test]
fn cut_exactly_to_the_floor() {
    // Pocket depth exactly reaching the opposite face (leaves zero-thickness
    // floor): must resolve to a through-cut or warn, never corrupt.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -10.0, ExtrudeMode::Cut);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 20.0 * 20.0 * 10.0 - 10.0 * 10.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "zero-floor pocket should become a through cut: {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// New-body extrusions from face-anchored sketches.
// ---------------------------------------------------------------------------

#[test]
fn new_body_from_face_sketch_not_touching() {
    // on_face sketch but extruded as a new body away from everything: a
    // floating body must exist independently and not disturb the box.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    face_sketch(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 8.0, ExtrudeMode::NewBody);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 20.0 * 20.0 * 10.0 + 10.0 * 10.0 * 8.0;
    assert!(
        (v - exact).abs() / exact < 0.05,
        "separate bodies total {v} vs {exact}"
    );
}

#[test]
fn sketch_on_tilted_face_of_rotated_cut() {
    // Cut a wedge out of a box with a tilted plane, then sketch on the new
    // tilted face and cut again — arbitrary face orientation chain.
    let angle = 45.0_f32.to_radians();
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 30.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 15.0),
            Vec3::new(angle.cos(), 0.0, angle.sin()),
            Vec3::new(-angle.sin(), 0.0, angle.cos()),
        ),
        rect_sketch((-30.0, -30.0), (30.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 10.0, ExtrudeMode::Cut);
    // A second cut on a parallel plane 5 mm up the wedge.
    add_sketch_cs(
        &mut g,
        "sketch_4",
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 20.0),
            Vec3::new(angle.cos(), 0.0, angle.sin()),
            Vec3::new(-angle.sin(), 0.0, angle.cos()),
        ),
        rect_sketch((-30.0, -30.0), (30.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_5", "sketch_4", 10.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let solid = 30.0 * 30.0 * 30.0;
    assert!(
        v < solid,
        "tilted cuts must remove material: {v} vs {solid}"
    );
}

// ---------------------------------------------------------------------------
// Region-selection misuse.
// ---------------------------------------------------------------------------

#[test]
fn extrude_with_out_of_range_region_indices_does_not_panic() {
    for indices in [vec![99usize], vec![0, 1, 2], vec![usize::MAX]] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
        g.add_feature(FeatureNode {
            id: "extrude_3".to_string(),
            name: "extrude_3".to_string(),
            feature: FeatureType::Extrude {
                depth: 5.0,
                region_indices: indices,
                mode: ExtrudeMode::Join,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        g.add_dependency("sketch_2", "extrude_3");
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

#[test]
fn extrude_open_profile_of_lines_only() {
    // A sketch of disconnected lines (no closed region): the extruder must
    // report "no regions" gracefully, not crash or invent geometry.
    let mut curves = zerocad_core::SketchCurves::new();
    curves.add_line((0.0, 0.0), (10.0, 0.0));
    curves.add_line((0.0, 5.0), (10.0, 5.0));
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch(&mut g, "sketch_2", curves);
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Cut);
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn deep_face_edit_chain_ten_pockets() {
    // Ten successively smaller/nested pockets each sketched on the previous
    // floor: the long face-on-face history the GUI produces in real models.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 30.0);
    let mut z = 30.0f32;
    let mut half = 28.0f32;
    let mut n = 2usize;
    for _ in 0..10 {
        face_sketch(
            &mut g,
            &format!("sketch_{n}"),
            xy_plane_at(z),
            rect_sketch((30.0 - half, 30.0 - half), (30.0 + half, 30.0 + half)),
        );
        add_extrude(
            &mut g,
            &format!("extrude_{}", n + 1),
            &format!("sketch_{n}"),
            -2.5,
            ExtrudeMode::Cut,
        );
        z -= 2.5;
        half -= 2.5;
        n += 2;
    }
    let v = assert_part_sane(&g);
    let solid = 60.0 * 60.0 * 30.0;
    assert!(
        v < solid,
        "nested pockets must remove material: {v} vs {solid}"
    );
}
