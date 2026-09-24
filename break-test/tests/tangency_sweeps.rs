//! Tangency and coincident-geometry sweeps.
//!
//! Coplanar faces, shared edges, and exact tangency are the historically
//! fragile zone for the boolean kernel. Each test sweeps a transform through
//! exact contact (offset 0.0 plus epsilons on both sides) and requires every
//! step to stay finite and deterministic.

mod common;

use common::*;
use zerocad_core::{ExtrudeMode, ParametricGraph};

/// Offsets straddling exact tangency at increasing magnitudes.
const MICRO: [f32; 7] = [-1e-2, -1e-3, -1e-4, 0.0, 1e-4, 1e-3, 1e-2];
const COARSE: [f32; 7] = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0];

fn box_cut_by_box_at_offset(off: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    // Cutter starts exactly at the box's top face (z = 10) plus the offset:
    // off = 0 means the cutter's bottom face is coplanar with the target's top.
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0 + off),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -8.0, ExtrudeMode::Cut);
    g
}

fn box_joined_to_box_at_offset(off: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0 + off),
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 6.0, ExtrudeMode::Join);
    g
}

fn edge_aligned_cut_at_offset(off: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    // Cutter whose side wall is exactly flush with the target's side wall
    // (x = 20) plus the offset: off = 0 gives a shared planar face.
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(5.0),
        rect_sketch((20.0 + off, 0.0), (30.0, 20.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 10.0, ExtrudeMode::Cut);
    g
}

#[test]
fn box_cut_through_coplanar_top_face() {
    tangency_sweep(&MICRO, box_cut_by_box_at_offset);
    tangency_sweep(&COARSE, box_cut_by_box_at_offset);
}

#[test]
fn box_join_through_coplanar_top_face() {
    tangency_sweep(&MICRO, box_joined_to_box_at_offset);
    tangency_sweep(&COARSE, box_joined_to_box_at_offset);
}

#[test]
fn box_cut_with_flush_side_walls() {
    tangency_sweep(&MICRO, edge_aligned_cut_at_offset);
    tangency_sweep(&COARSE, edge_aligned_cut_at_offset);
}

#[test]
fn cylinder_cut_tangent_to_flat_face() {
    // A cylindrical cutter whose silhouette is exactly tangent to a box's
    // side face sweeps through tangency.
    let offsets: [f32; 7] = [-1.0, -0.1, -0.01, 0.0, 0.01, 0.1, 1.0];
    tangency_sweep(&offsets, |off| {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        // Circle of radius 6 centered so its leftmost point touches x = 0
        // (the box's -x face lies at x = -10 in sketch space? No: the sketch
        // is the XY plane, so local (x, y) map to world (x, y). The box spans
        // x in [-10, 10]; center the circle so its edge is tangent to x = -10.
        add_sketch_cs(
            &mut g,
            "sketch_2",
            xy_plane_at(5.0),
            circle_sketch((-4.0 + off, 0.0), 6.0),
        );
        add_extrude(&mut g, "extrude_3", "sketch_2", 10.0, ExtrudeMode::Cut);
        g
    });
}

#[test]
fn cylinder_join_cylinder_coaxial_and_tangent() {
    let offsets: [f32; 5] = [-0.5, -0.05, 0.0, 0.05, 0.5];
    tangency_sweep(&offsets, |off| {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 10.0, 10.0);
        // A boss on top of the cylinder, tangent internally: radius 10 - off
        // sits exactly on the wall when off = 0.
        add_sketch_cs(
            &mut g,
            "sketch_2",
            xy_plane_at(10.0),
            circle_sketch((0.0, 0.0), 10.0 - off),
        );
        add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
        g
    });
}

#[test]
fn identical_coincident_bodies_boolean() {
    // The degenerate extreme: a boolean where object and tool occupy exactly
    // the same space.
    for mode in [ExtrudeMode::Join, ExtrudeMode::Cut] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_sketch(
            &mut g,
            "sketch_2",
            rect_sketch((-10.0, -10.0), (10.0, 10.0)),
        );
        add_extrude(&mut g, "extrude_3", "sketch_2", 10.0, mode);
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn cut_that_consumes_entire_body() {
    // A cutter strictly larger than the target on all sides: the result is
    // either empty (documented) or unresolved — never a corrupt mesh.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_sketch(
        &mut g,
        "sketch_2",
        rect_sketch((-20.0, -20.0), (20.0, 20.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 30.0, ExtrudeMode::Cut);
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    // And the epsilon version: cutter barely overlapping the target corner.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_sketch(
        &mut g,
        "sketch_2",
        rect_sketch((-20.0, -20.0), (-9.999, -9.999)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 30.0, ExtrudeMode::Cut);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn thin_ribbon_residue_after_cut() {
    // Cut leaving a nearly-zero-thickness ribbon of material — the classic
    // source of sliver faces and degenerate triangles.
    let thicknesses: [f32; 5] = [1e-1, 1e-2, 1e-3, 1e-4, 1e-5];
    for &t in &thicknesses {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        // Cut everything except a ribbon of thickness t at the top.
        add_sketch_cs(
            &mut g,
            "sketch_2",
            xy_plane_at(0.0),
            rect_sketch((-15.0, -15.0), (15.0, 15.0)),
        );
        let mut g2 = g;
        add_extrude(&mut g2, "extrude_3", "sketch_2", 10.0 - t, ExtrudeMode::Cut);
        let bodies = eval(&g2);
        assert_meshes_finite(&bodies);
    }
}
