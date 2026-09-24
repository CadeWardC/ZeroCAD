//! Fillet / chamfer (EdgeMod + EdgeBlend) break tests.
//!
//! Cases reconstructed from the documented CAD-kernel failure taxonomy (see
//! break-test/FINDINGS.md sources): radius too large for the adjacent faces,
//! radius exactly consuming a face, blended corner networks, fillet on
//! cylinder/box junctions, thin-plate fillets, acute-angle edges, chained
//! fillets, stale edge references, and degenerate radii.
//!
//! Every case must produce valid geometry OR an unresolved-feature warning —
//! never a panic, NaN mesh, or silent no-op that reports success.

mod common;

use common::*;
use zerocad_core::sketch::CornerKind;
use zerocad_core::{
    AxisBase, EdgeCornerMode, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph,
};

/// Bottom-front edge of a box centered at origin with size (w,h,d):
/// along +X at y = -h/2, z = -d/2.
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

fn add_edge_mod(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    edge: EdgeRef,
    dist: f32,
    kind: CornerKind,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::EdgeMod {
            target: target.to_string(),
            edge,
            dist,
            dist_expr: None,
            kind,
        },
    });
    g.add_dependency(target, id);
}

fn add_edge_blend(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    edges: Vec<EdgeRef>,
    dist: f32,
    kind: CornerKind,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::EdgeBlend {
            target: target.to_string(),
            edges,
            dist,
            dist_expr: None,
            kind,
            corner_mode: EdgeCornerMode::Auto,
        },
    });
    g.add_dependency(target, id);
}

// ---------------------------------------------------------------------------
// Baseline general usage (must keep passing forever).
// ---------------------------------------------------------------------------

#[test]
fn simple_fillet_on_box_edge() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Fillet,
    );
    let v = assert_part_sane(&g);
    let exact = 20.0 * 20.0 * 20.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 9.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "fillet volume {v} vs {exact}"
    );
}

#[test]
fn simple_chamfer_on_box_edge() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "chamfer_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Chamfer,
    );
    let v = assert_part_sane(&g);
    let exact = 20.0 * 20.0 * 20.0 - 0.5 * 9.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "chamfer volume {v} vs {exact}"
    );
}

// ---------------------------------------------------------------------------
// "Radius too large" — the #1 documented fillet failure everywhere.
// ---------------------------------------------------------------------------

#[test]
fn fillet_radius_sweep_up_to_overflow() {
    // On a 20-box the max possible edge fillet is 10 (half the smallest face
    // extent). Sweep toward and past it.
    for r in [4.0f32, 8.0, 9.9, 9.999, 10.0, 10.001, 11.0, 20.0, 100.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_mod(
            &mut g,
            "fillet_2",
            "box_1",
            box_edge_x(20.0, 20.0, 20.0),
            r,
            CornerKind::Fillet,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn chamfer_setback_larger_than_face() {
    // Chamfer setback 25 on a 20-box: the cutters overlap past the opposite
    // corners — must resolve to a warning or a valid (heavily cut) body.
    for s in [9.9f32, 10.0, 14.14, 15.0, 20.0, 40.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_mod(
            &mut g,
            "chamfer_2",
            "box_1",
            box_edge_x(20.0, 20.0, 20.0),
            s,
            CornerKind::Chamfer,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn fillet_radius_larger_than_thin_plate_thickness() {
    // Thin plate (2 mm) with a 5 mm fillet on a rim edge: the fillet cylinder
    // is bigger than the plate is thick.
    for r in [0.5f32, 1.0, 1.9, 2.0, 2.1, 5.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 2.0);
        add_edge_mod(
            &mut g,
            "fillet_2",
            "box_1",
            box_edge_x(30.0, 30.0, 2.0),
            r,
            CornerKind::Fillet,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn fillet_radius_equal_to_edge_length() {
    // 20-long edge, radius 20: the fillet spans the entire edge length.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        box_edge_x(40.0, 20.0, 20.0),
        20.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

// ---------------------------------------------------------------------------
// Degenerate radii.
// ---------------------------------------------------------------------------

#[test]
fn degenerate_fillet_distances_do_not_panic() {
    for r in [0.0f32, -1.0, -100.0, 1e-7, 1e7, f32::NAN] {
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            let mut g = ParametricGraph::new();
            add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
            add_edge_mod(
                &mut g,
                "mod_2",
                "box_1",
                box_edge_x(20.0, 20.0, 20.0),
                r,
                kind,
            );
            let bodies = eval(&g);
            assert_meshes_finite(&bodies);
        }
    }
}

// ---------------------------------------------------------------------------
// Corner networks — three fillets meeting at one vertex (EdgeBlend).
// ---------------------------------------------------------------------------

#[test]
fn blend_three_edges_meeting_at_a_corner() {
    // The classic hard case: three mutually perpendicular edges sharing a
    // vertex — the corner needs a rolling-ball/setback patch.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_blend(
        &mut g,
        "blend_2",
        "box_1",
        vec![
            box_edge_x(20.0, 20.0, 20.0),
            box_edge_y(20.0, 20.0, 20.0),
            box_edge_z(20.0, 20.0, 20.0),
        ],
        3.0,
        CornerKind::Fillet,
    );
    let v = assert_part_sane(&g);
    let solid = 20.0 * 20.0 * 20.0;
    assert!(
        v < solid,
        "corner blend must remove material: {v} vs {solid}"
    );
}

#[test]
fn blend_network_with_too_large_radius() {
    for r in [9.0f32, 9.9, 10.0, 10.1, 15.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_blend(
            &mut g,
            "blend_2",
            "box_1",
            vec![
                box_edge_x(20.0, 20.0, 20.0),
                box_edge_y(20.0, 20.0, 20.0),
                box_edge_z(20.0, 20.0, 20.0),
            ],
            r,
            CornerKind::Fillet,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn blend_mixed_fillet_and_chamfer_corner_modes() {
    // Chamfer network on the same corner (miter planes must meet).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_blend(
        &mut g,
        "blend_2",
        "box_1",
        vec![
            box_edge_x(20.0, 20.0, 20.0),
            box_edge_y(20.0, 20.0, 20.0),
            box_edge_z(20.0, 20.0, 20.0),
        ],
        4.0,
        CornerKind::Chamfer,
    );
    let v = assert_part_sane(&g);
    assert!(v < 20.0 * 20.0 * 20.0);
}

#[test]
fn blend_empty_and_duplicate_edge_selections() {
    // Degenerate selections a buggy caller could produce.
    for edges in [
        vec![],
        vec![box_edge_x(20.0, 20.0, 20.0), box_edge_x(20.0, 20.0, 20.0)],
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_blend(&mut g, "blend_2", "box_1", edges, 3.0, CornerKind::Fillet);
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

// ---------------------------------------------------------------------------
// Fillets on curved / mixed surfaces.
// ---------------------------------------------------------------------------

#[test]
fn fillet_box_to_cylinder_junction() {
    // Boss (cylinder) on a plate, fillet the rim where they meet — the
    // planar/cylindrical junction from the Fusion failure article.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    // Boss centered on the plate (box spans 0..40).
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        circle_sketch((20.0, 20.0), 10.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 15.0, ExtrudeMode::Join);
    // Rim edge: a circular edge approximated by its extreme point + radial normals.
    add_edge_mod(
        &mut g,
        "fillet_4",
        "box_1",
        EdgeRef {
            p0: [30.0, 20.0, 10.0],
            p1: [10.0, 20.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
        2.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn fillet_cylinder_rim_edge() {
    // Fillet the top rim of a standalone cylinder: radius vs cylinder radius.
    for r in [1.0f32, 5.0, 9.9, 10.0, 10.1, 15.0] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
        add_edge_mod(
            &mut g,
            "fillet_2",
            "cyl_1",
            EdgeRef {
                // Cylinders run along +Y with base at origin: top rim at y=20.
                p0: [10.0, 20.0, 0.0],
                p1: [-10.0, 20.0, 0.0],
                n1: [0.0, 1.0, 0.0],
                n2: [1.0, 0.0, 0.0],
                curve: None,
                topology: None,
            },
            r,
            CornerKind::Fillet,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn fillet_on_acute_angle_edge() {
    // Cut a wedge to create a non-90° dihedral angle, then fillet it — the
    // "interior edge at a sharp angle" FreeCAD case. Plane tilted 60° about
    // the X axis through the box center (15,15): its dihedral edge with the
    // bottom face (z=0) runs along X at y = 15 - 15·cot60° ≈ 6.34.
    let angle = 60.0_f32.to_radians();
    let (s, c) = (angle.sin(), angle.cos());
    let y_edge = 15.0 - 15.0 * (c / s);
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 30.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        zerocad_core::CoordinateSystem::new(
            zerocad_core::Vec3::new(15.0, 15.0, 15.0),
            zerocad_core::Vec3::new(1.0, 0.0, 0.0),
            zerocad_core::Vec3::new(0.0, s, c),
        ),
        rect_sketch((-15.0, -30.0), (15.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 12.0, ExtrudeMode::Cut);
    add_edge_mod(
        &mut g,
        "fillet_4",
        "box_1",
        EdgeRef {
            p0: [0.0, y_edge, 0.0],
            p1: [30.0, y_edge, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -s, c],
            curve: None,
            topology: None,
        },
        1.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

// ---------------------------------------------------------------------------
// Chained / sequential fillets and stale references.
// ---------------------------------------------------------------------------

#[test]
fn two_sequential_fillets_on_adjacent_edges() {
    // Fillet one edge, then a neighbor: the second fillet's reference edge
    // endpoints were captured before the first fillet reshaped the corner.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Fillet,
    );
    add_edge_mod(
        &mut g,
        "fillet_3",
        "box_1",
        box_edge_y(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn fillet_then_chamfer_same_edge() {
    // Second mod targets the same (already-consumed) edge.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "mod_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Fillet,
    );
    add_edge_mod(
        &mut g,
        "mod_3",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        2.0,
        CornerKind::Chamfer,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn edge_mod_referencing_edge_far_away_from_body() {
    // Stale reference after the body was replaced/moved: the captured edge
    // no longer lies on the target. Must warn (reference.missing), not cut
    // some random spot or panic.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        EdgeRef {
            p0: [500.0, 500.0, 500.0],
            p1: [510.0, 500.0, 500.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        3.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 20.0 * 20.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "stale edge ref must leave the body untouched: {v} vs {exact}"
    );
}

#[test]
fn edge_mod_on_missing_target_body() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "ghost_body",
        box_edge_x(20.0, 20.0, 20.0),
        3.0,
        CornerKind::Fillet,
    );
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn edge_mod_with_garbage_edge_geometry() {
    // NaN endpoints / zero-length edge / non-unit normals.
    let bad_edges: &[EdgeRef] = &[
        EdgeRef {
            p0: [f32::NAN, 0.0, 0.0],
            p1: [10.0, 0.0, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [0.0, 0.0, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [-10.0, -10.0, -10.0],
            p1: [10.0, -10.0, -10.0],
            n1: [0.0, 0.0, 0.0],
            n2: [0.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
    ];
    for edge in bad_edges {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_mod(
            &mut g,
            "mod_2",
            "box_1",
            edge.clone(),
            3.0,
            CornerKind::Fillet,
        );
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

// ---------------------------------------------------------------------------
// Fillets after history edits (the classic rebuild failure).
// ---------------------------------------------------------------------------

#[test]
fn fillet_survives_preceding_pocket_edit() {
    // Pocket first, then fillet a far edge — then the fillet must reattach
    // even though the body's face list changed since capture.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        rect_sketch((5.0, 5.0), (25.0, 25.0)),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -3.0, ExtrudeMode::Cut);
    add_edge_mod(
        &mut g,
        "fillet_4",
        "box_1",
        box_edge_x(30.0, 30.0, 10.0),
        2.0,
        CornerKind::Fillet,
    );
    let v = assert_part_sane(&g);
    let solid = 30.0 * 30.0 * 10.0;
    assert!(
        v < solid,
        "pocket+fillet must remove material: {v} vs {solid}"
    );
}

#[test]
fn fillet_after_hole_through_same_face() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [0.0, 0.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_edge_mod(
        &mut g,
        "fillet_3",
        "box_1",
        box_edge_x(30.0, 30.0, 10.0),
        2.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn fillet_radius_grows_until_it_hits_the_hole() {
    // A fillet on the rim edge whose radius grows until it collides with a
    // central through-hole — the "insufficient clearance" failure.
    for r in [1.0f32, 3.0, 5.0, 6.9, 7.0, 7.1, 10.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [0.0, 0.0, 10.0],
            [0.0, 0.0, -1.0],
            8.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
        add_edge_mod(
            &mut g,
            "fillet_3",
            "box_1",
            box_edge_x(30.0, 30.0, 10.0),
            r,
            CornerKind::Fillet,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn fillet_on_revolved_body_edge() {
    let mut g = ParametricGraph::new();
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
    // Outer rim edge of the flange (circle at radius 40 in the YZ plane at x=30).
    add_edge_mod(
        &mut g,
        "fillet_3",
        "revolve_2",
        EdgeRef {
            p0: [30.0, 0.0, 40.0],
            p1: [30.0, 0.0, -40.0],
            n1: [1.0, 0.0, 0.0],
            n2: [0.0, 0.0, 1.0],
            curve: None,
            topology: None,
        },
        2.0,
        CornerKind::Fillet,
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

// ---------------------------------------------------------------------------
// Characterization tests: gaps and gray zones found by probing (2026-09-10).
// These pin CURRENT behavior so a fix changes the assertion deliberately.
// ---------------------------------------------------------------------------

fn warnings_for(g: &ParametricGraph) -> (f64, Vec<String>) {
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("evaluation must not hard-fail");
    (total_volume(&bodies), warnings)
}

// E1 (verdict revised 2026-09-11): this fixture pins the LEGACY
// incomplete-reference path only — the EdgeRef supplies the rim as a
// straight chord with `curve: None` and no durable topology, which no
// matcher may guess from. It is NOT evidence about cylinder-rim fillets in
// general: references captured from real picks carry topology names and
// circular curve hints, and that path is covered by the core regression
// suites (`circular_rim_blend.rs`, 19 tests; `annular_rim_blend.rs`, 2).
// The unsupported outcome here must stay explicit (warn + unchanged body),
// never a wrong-radius guess from ambiguous endpoints.
#[test]
fn legacy_chord_reference_rim_fillet_is_rejected_not_guessed() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "cyl_1",
        EdgeRef {
            p0: [10.0, 20.0, 0.0],
            p1: [-10.0, 20.0, 0.0],
            n1: [0.0, 1.0, 0.0],
            n2: [1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
        2.0,
        CornerKind::Fillet,
    );
    let (v, warnings) = warnings_for(&g);
    let rod = std::f64::consts::PI * 100.0 * 20.0;
    assert!(
        (v - rod).abs() / rod < 0.05,
        "unmatched rim fillet leaves rod unchanged: {v} vs {rod}"
    );
    assert!(
        warnings.iter().any(|w| w.contains("left unchanged")),
        "an unmatched legacy reference must warn, got {warnings:?}"
    );
}

// E2 (verdict revised 2026-09-11): same incomplete-reference situation as
// E1 — the boss-root circular edge is supplied as a straight chord with no
// curve hint and no topology. The legacy rejection must stay explicit. A
// genuinely captured concave boss-root selection (GUI pick with topology +
// circular hint) is the remaining unproven case; these assertions must not
// be read as a general boss-root capability claim.
#[test]
fn legacy_chord_reference_boss_root_fillet_is_rejected_not_guessed() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sketch_2",
        xy_plane_at(10.0),
        circle_sketch((20.0, 20.0), 10.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 15.0, ExtrudeMode::Join);
    add_edge_mod(
        &mut g,
        "fillet_4",
        "box_1",
        EdgeRef {
            p0: [30.0, 20.0, 10.0],
            p1: [10.0, 20.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
        2.0,
        CornerKind::Fillet,
    );
    let (v, warnings) = warnings_for(&g);
    let unfilleted = 40.0 * 40.0 * 10.0 + std::f64::consts::PI * 100.0 * 15.0;
    assert!(
        (v - unfilleted).abs() / unfilleted < 0.03,
        "junction fillet leaves body unchanged: {v} vs {unfilleted}"
    );
    assert!(
        warnings.iter().any(|w| w.contains("left unchanged")),
        "unsupported junction fillet must warn, got {warnings:?}"
    );
}

// E3 corrected (single isolated edge, review 2026-09-11): for ONE convex
// 90° edge of a 20 mm cube the fillet consumes `r` along each adjacent face,
// so the fit limit is NOT half the face width — radii up to 20 mm fit this
// isolated edge. The analytic oracle is `8000 − 20·r²·(1 − π/4)`
// (the removed wedge cross-section `r² − πr²/4` swept 20 mm). The boolean
// cutter is faceted, so the observed removal runs slightly ABOVE the exact
// rolling-ball value (r=3: 7960.45 vs analytic 7961.2) — bounded here by a
// relative band.
#[test]
fn single_edge_large_radius_fillet_matches_analytic_volume() {
    for radius in [3.0f32, 10.0, 10.001, 15.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
        add_edge_mod(
            &mut g,
            "fillet_2",
            "box_1",
            box_edge_x(20.0, 20.0, 20.0),
            radius,
            CornerKind::Fillet,
        );
        let (v, warnings) = warnings_for(&g);
        assert!(
            warnings.is_empty(),
            "a fitting isolated-edge radius must not warn (r={radius}): {warnings:?}"
        );
        let analytic = 8000.0
            - 20.0 * f64::from(radius) * f64::from(radius) * (1.0 - std::f64::consts::FRAC_PI_4);
        assert!(
            (v - analytic).abs() / analytic < 0.03,
            "r={radius}: rolling-ball oracle {analytic:.2}, got {v}"
        );
    }
}

// E3 infeasible companion (kept separate from the fitting band): at exactly
// r = 20 the fillet consumes BOTH adjacent faces completely — the
// construction fails and must roll back loudly, never commit a broken solid.
// Interacting-edge networks need their own local fit analysis and are
// covered by `blend_network_with_too_large_radius`.
#[test]
fn single_edge_fillet_at_full_face_width_rejects_cleanly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        20.0,
        CornerKind::Fillet,
    );
    let (v, warnings) = warnings_for(&g);
    assert!(
        (v - 8000.0).abs() < 1.0,
        "an infeasible radius must leave the box unchanged: {v}"
    );
    assert!(
        !warnings.is_empty(),
        "an infeasible radius must warn: {warnings:?}"
    );
}

// E4 (scope decision, review 2026-09-11): a sub-tolerance positive radius
// (1e-7 mm, below the kernel's modeling resolution) honestly reports an
// unresolved feature instead of claiming a successful fillet of unchanged
// geometry — display quantization must not authorize "success". The body
// stays exactly unchanged and the warning names the failure.
#[test]
fn sub_tolerance_fillet_radius_leaves_body_unchanged_with_warning() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        box_edge_x(20.0, 20.0, 20.0),
        1e-7,
        CornerKind::Fillet,
    );
    let (v, warnings) = warnings_for(&g);
    assert!(
        (v - 8000.0).abs() < 1.0,
        "sub-tolerance fillet leaves box unchanged: {v}"
    );
    assert!(
        !warnings.is_empty(),
        "a sub-tolerance radius must report an explicit unresolved result: {warnings:?}"
    );
}

// EXPECTED (documented) graceful failure: fillet radius larger than a thin
// plate's thickness fails the rolling-ball construction and warns, leaving
// the body intact. This is correct behavior, pinned here as a regression.
#[test]
fn characterization_fillet_larger_than_thickness_warns_cleanly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 2.0);
    add_edge_mod(
        &mut g,
        "fillet_2",
        "box_1",
        EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [30.0, 0.0, 0.0],
            n1: [0.0, 0.0, -1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        5.0,
        CornerKind::Fillet,
    );
    let (v, warnings) = warnings_for(&g);
    assert!(
        (v - 1800.0).abs() < 1.0,
        "oversize thin-plate fillet leaves body unchanged: {v}"
    );
    assert!(
        warnings.iter().any(|w| w.contains("left unchanged")),
        "expected a clean fallback warning, got {warnings:?}"
    );
}
