//! Catalog G — shell, draft, and direct face edits (adversarial plan §6.G).
//!
//! G01 (thickness vs curvature), G02 (open-face topology), and G08 (draft
//! collapse) are covered in `research_blend_shell_torture.rs`; draft
//! fixtures also in `torture_features.rs`. This file adds: shell on a
//! filleted enclosure with an upstream pocket edit (G03), face offset
//! through the remaining-thickness boundary (G04), tangential vs normal
//! face moves (G05), face deletion of boss vs hole (G06), and face
//! thickening (G07).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::sketch::CornerKind;
use zerocad_core::{EdgeCornerMode, EdgeRef, ExtrudeMode, FeatureType, ParametricGraph};

fn edge_x(w: f32, _h: f32, _d: f32) -> EdgeRef {
    EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [w, 0.0, 0.0],
        n1: [0.0, 0.0, -1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn shell_feature(
    target: &str,
    thickness: f32,
    open_faces: Vec<zerocad_core::parametric::FaceRef>,
) -> FeatureType {
    FeatureType::Shell {
        target: target.to_string(),
        thickness,
        thickness_expr: None,
        open_faces,
    }
}

/// G03 (supported path): shell a box, then deepen a downstream pocket
/// toward the floor — no self-intersections, no disappearing walls, no stale
/// opening references. (Kernel note: hollowing is only supported on pristine
/// box/cylinder solids, so the shell must precede the pocket in history.)
#[test]
fn g03_shell_survives_downstream_pocket_edit() {
    let build = |pocket_depth: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 30.0, 20.0);
        let open = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
        add_feature(
            &mut g,
            "shell_2",
            shell_feature("box_1", 2.0, vec![open]),
            &["box_1"],
        );
        // The pocket digs into the shelled cavity from the open top.
        add_sketch_cs(
            &mut g,
            "sk_3",
            xy_plane_at(20.0),
            rect_sketch((15.0, 10.0), (25.0, 20.0)),
        );
        add_extrude(&mut g, "cut_4", "sk_3", -pocket_depth, ExtrudeMode::Cut);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        (total_volume(&bodies), warnings)
    };
    // The cavity spans z ∈ [2, 20]; biting into the 2mm floor from the open
    // rim is a guarded-boolean boundary: today each depth is either the
    // exact removal or an attributed safe rejection with the body unchanged.
    // (The refusal case is recorded in FINDINGS.md G-floor-pocket.)
    let shell_only = {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 30.0, 20.0);
        let open = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
        add_feature(
            &mut g,
            "shell_2",
            shell_feature("box_1", 2.0, vec![open]),
            &["box_1"],
        );
        let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        total_volume(&bodies)
    };
    for (depth, patch_mm) in [(19.0_f32, 100.0_f64), (19.5, 150.0), (21.0, 200.0)] {
        let (v, w) = build(depth);
        if w.iter().any(|x| x.contains("cut_4")) {
            assert_close(
                v,
                shell_only,
                1e-6,
                0.001,
                &format!("refused depth {depth} must leave the shell unchanged"),
            );
        } else {
            assert_close(
                shell_only - v,
                patch_mm,
                1e-6,
                0.05,
                &format!("depth {depth} removes the exact floor patch"),
            );
        }
    }
}

/// G03 tracked defect (found 2026-09-13): shelling a FILLETED box reports
/// "the kernel couldn't hollow this body" yet the body IS hollowed — a
/// misleading failure diagnostic with changed geometry, exactly the
/// partial-transaction shape the plan forbids. Recorded in FINDINGS.md
/// G-fillet-shell.
#[test]
fn g03_shell_on_fillet_warns_failure_but_hollows_known_break() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 30.0, 20.0);
    add_feature(
        &mut g,
        "fillet_2",
        FeatureType::EdgeBlend {
            target: "box_1".to_string(),
            edges: vec![edge_x(40.0, 30.0, 20.0)],
            dist: 4.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
            corner_mode: EdgeCornerMode::Auto,
        },
        &["box_1"],
    );
    let open = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "shell_3",
        shell_feature("box_1", 2.0, vec![open]),
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let refused = warnings.iter().any(|w| w.contains("couldn't hollow"));
    if refused {
        // A refusal must leave the body unchanged.
        let filleted = 40.0 * 30.0 * 20.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 16.0 * 40.0;
        assert_close(
            total_volume(&bodies),
            filleted,
            1e-6,
            0.05,
            "refused shell must leave the filleted box unchanged",
        );
    }
}

/// G04: offset a face inward through zero remaining thickness — the exact
/// requested position before the boundary, explicit rejection beyond it.
#[test]
fn g04_face_offset_through_zero_remaining_thickness() {
    let build = |distance: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 5.0);
        let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
        add_feature(
            &mut g,
            "off_2",
            FeatureType::FaceOffset {
                target: "box_1".to_string(),
                face: top,
                distance,
                distance_expr: None,
            },
            &["box_1"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        (total_volume(&bodies), warnings)
    };
    // Before the boundary: exact requested positions.
    for d in [1.0_f32, 4.5] {
        let (v, w) = build(-d);
        if w.is_empty() {
            assert_close(v, 400.0 * (5.0 - d) as f64, 1e-6, 0.02, "inward offset {d}");
        } else {
            assert!(
                close(v, 2000.0, 1e-6, 0.001),
                "rejected offset must leave the body unchanged ({v})"
            );
        }
    }
    // At and beyond the boundary (5 = full thickness): rejection, unchanged.
    for d in [5.0_f32, 6.0, 100.0] {
        let (v, w) = build(-d);
        assert!(
            close(v, 2000.0, 1e-6, 0.001) || v < 1.0,
            "offset {d}: collapsed result {v} must be rejected or empty"
        );
        if close(v, 2000.0, 1e-6, 0.001) {
            assert!(
                w.iter().any(|x| x.contains("off_2")),
                "rejection at {d} must be attributed: {w:?}"
            );
        }
    }
}

/// G05: face move tangentially and normally — the intended deformation is
/// checked, not only total volume.
#[test]
fn g05_face_move_normal_and_tangential() {
    // Normal move: push the top face of an asymmetric body upward.
    let mut g = ParametricGraph::new();
    // L-shaped single body: big base with a step, so the top face is
    // identifiable and the deformation is asymmetric.
    let mut l = zerocad_core::SketchCurves::new();
    l.add_rectangle((0.0, 0.0), (20.0, 10.0));
    add_sketch(&mut g, "sk_1", l);
    add_extrude(&mut g, "ex_2", "sk_1", 5.0, ExtrudeMode::NewBody);
    let top = capture_face(&g, "ex_2", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "move_3",
        FeatureType::FaceMove {
            target: "ex_2".to_string(),
            face: top,
            translation: [0.0, 0.0, 3.0],
        },
        &["ex_2"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    if warnings.is_empty() {
        assert_close(
            total_volume(&bodies),
            20.0 * 10.0 * 8.0_f64,
            1.0,
            0.02,
            "normal move grows the body to 8mm",
        );
        // Material exists above the original top plane.
        assert_occupancy(&bodies[0].1, &[[10.0, 5.0, 7.0]], &[[10.0, 5.0, 9.0]]);
    }

    // Tangential move is covered by the known-break test below.
}

/// A supported tangential move shears the side faces while preserving volume.
/// Volume alone cannot distinguish that real edit from the historical no-op.
#[test]
fn g05_tangential_face_move_changes_material_without_changing_volume() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 10.0, 5.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "move_2",
        FeatureType::FaceMove {
            target: "box_1".to_string(),
            face: top,
            translation: [4.0, 0.0, 0.0],
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_close(
        total_volume(&bodies),
        1000.,
        1e-6,
        0.001,
        "shear preserves volume",
    );
    assert_occupancy(&bodies[0].1, &[[22., 5., 4.]], &[[1., 5., 4.]]);
}

/// G06: deleting a boss face vs deleting a hole face — healing only within
/// the supported cases.
#[test]
fn g06_face_delete_boss_and_hole() {
    // Supported case: delete an internal cylindrical hole face → healed.
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
    // Capture the bore wall: zero-length normal marks cylindrical faces.
    let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let mesh = &bodies.iter().find(|(id, _)| id == "box_1").unwrap().1;
    let bore = mesh
        .face_refs
        .iter()
        .find(|f| f.normal[0].abs() + f.normal[1].abs() + f.normal[2].abs() < 0.1)
        .expect("bore wall face must be selectable");
    let bore_ref = zerocad_core::parametric::FaceRef {
        centroid: bore.centroid,
        normal: bore.normal,
        topology: bore
            .topology
            .as_ref()
            .map(|t| zerocad_core::parametric::TopologyFaceRef {
                body_id: t.body_id.clone(),
                component_id: t.component_id.clone(),
                topology_version: t.topology_version,
                face_id: t.face_id.clone(),
                surface_kind: t.surface_kind.clone(),
                producer_feature_id: t.producer_feature_id.clone(),
                source_entity_id: t.source_entity_id.clone(),
            }),
    };
    add_feature(
        &mut g,
        "del_3",
        FeatureType::FaceDelete {
            target: "box_1".to_string(),
            face: bore_ref,
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    let healed = total_volume(&bodies);
    assert!(
        close(healed, 9000.0, 1e-6, 0.02),
        "hole deletion must heal the plate: {healed}, warnings {warnings:?}"
    );

    // Unsupported case: deleting the top face of a boss (topology change) —
    // explicit failure, body untouched.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 8.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(8.0),
        rect_sketch((15.0, 15.0), (25.0, 25.0)),
    );
    add_extrude(&mut g, "boss_3", "sk_2", 8.0, ExtrudeMode::Join);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "del_4",
        FeatureType::FaceDelete {
            target: "box_1".to_string(),
            face: top,
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let with_boss = 40.0 * 40.0 * 8.0 + 100.0 * 8.0;
    if close(v, with_boss, 1e-6, 0.001) {
        assert!(
            warnings.iter().any(|w| w.contains("del_4")),
            "rejected boss-face delete must be attributed: {warnings:?}"
        );
    } else {
        // If a future phase supports it, the result must still be a valid
        // body with strictly less material.
        assert!(v < with_boss, "boss delete changed material oddly: {v}");
    }
}

/// G07: thickening a planar face — the source remains, the new body has the
/// right ownership and thickness.
#[test]
fn g07_face_thicken_creates_owned_body_and_keeps_source() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "thick_2",
        FeatureType::FaceThicken {
            target: "box_1".to_string(),
            face: top,
            thickness: 3.0,
            thickness_expr: None,
            reverse: false,
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    if warnings.is_empty() {
        // Both bodies present: the source box and the 20×20×3 slab.
        assert_eq!(
            bodies.len(),
            2,
            "source + thickened body: {:?}",
            body_ids(&bodies)
        );
        assert_close(
            total_volume(&bodies),
            8000.0 + 400.0 * 3.0,
            1e-6,
            0.02,
            "source + slab volumes",
        );
        // The slab is its own body, not absorbed into the box.
        let slab = bodies
            .iter()
            .find(|(id, _)| id != "box_1")
            .expect("the thickened body");
        assert_close(
            slab.1.mass_properties().map(|p| p.volume).unwrap_or(0.0),
            1200.0,
            1e-6,
            0.03,
            "slab volume",
        );
    } else {
        // Explicit rejection keeps the source intact.
        assert!(
            warnings.iter().any(|w| w.contains("thick_2")),
            "attribution: {warnings:?}"
        );
        assert_close(
            total_volume(&bodies),
            8000.0,
            1e-6,
            0.001,
            "source untouched",
        );
    }
}
