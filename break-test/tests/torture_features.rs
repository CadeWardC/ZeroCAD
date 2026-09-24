//! Torture suite 1: the features never exercised by the earlier suites —
//! Draft, BodySplit, direct face modeling (FaceOffset/FaceMove/FaceDelete/
//! FaceThicken), FeaturePattern, Mirror patterns, and datum-anchored
//! construction. Face selections are captured from evaluated meshes exactly
//! the way the GUI does (with durable topology names); baselines must hold
//! analytically and degenerate variants must degrade to warnings.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::{
    DraftNeutral, FaceRef, FeaturePatternComputeMode, FeaturePatternExtentPolicy,
    FeaturePatternKind, TopologyFaceRef,
};
use zerocad_core::{
    AxisBase, DatumPlaneDef, ExtrudeMode, FeatureNode, FeatureType, MockMesh, ParametricGraph,
    PlaneBase,
};

fn face(centroid: [f32; 3], normal: [f32; 3]) -> FaceRef {
    FaceRef {
        centroid,
        normal,
        topology: None,
    }
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

fn eval_ok(g: &ParametricGraph) -> (Vec<(String, MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail")
}

/// Capture a face reference from an evaluated mesh the way the GUI does:
/// durable topology names attached, centroid/normal kept for fallback.
fn capture_face(g: &ParametricGraph, body_id: &str, want_normal: [f32; 3]) -> FaceRef {
    let (bodies, _) = eval_ok(g);
    let mesh = &bodies
        .iter()
        .find(|(id, _)| id == body_id)
        .unwrap_or_else(|| panic!("body {body_id} missing"))
        .1;
    let f = mesh
        .face_refs
        .iter()
        .find(|f| {
            let dot = f.normal[0] * want_normal[0]
                + f.normal[1] * want_normal[1]
                + f.normal[2] * want_normal[2];
            dot > 0.99
        })
        .unwrap_or_else(|| {
            panic!(
                "no face with normal {want_normal:?} among {} faces",
                mesh.face_refs.len()
            )
        });
    FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    }
}

// ---------------------------------------------------------------------------
// Draft.
// ---------------------------------------------------------------------------

#[test]
fn draft_four_side_faces_of_box() {
    // Baseline: draft all four side walls of a 20×20×30 box 5° inward,
    // neutral = bottom face. A frustum results (or a documented rejection).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 30.0);
    let sides = vec![
        capture_face(&g, "box_1", [-1.0, 0.0, 0.0]),
        capture_face(&g, "box_1", [1.0, 0.0, 0.0]),
        capture_face(&g, "box_1", [0.0, -1.0, 0.0]),
        capture_face(&g, "box_1", [0.0, 1.0, 0.0]),
    ];
    let neutral = capture_face(&g, "box_1", [0.0, 0.0, -1.0]);
    add_feature(
        &mut g,
        "draft_2",
        FeatureType::Draft {
            target: "box_1".to_string(),
            faces: sides,
            neutral: DraftNeutral::Face(neutral),
            angle_deg: 5.0,
            angle_expr: None,
            flip_pull: false,
        },
        &["box_1"],
    );
    let v = assert_part_sane(&g);
    let t = 30.0 * (5.0f64.to_radians().tan());
    let a = 20.0 - 2.0 * t;
    let frustum = 30.0 / 3.0 * (400.0 + a * a + (400.0f64 * a * a).sqrt());
    assert!(
        (v - frustum).abs() / frustum < 0.03 || (v - 12000.0).abs() / 12000.0 < 0.02,
        "drafted box volume {v} vs frustum {frustum} (unchanged 12000 if rejected)"
    );
}

#[test]
fn draft_degenerate_angles_do_not_panic() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 30.0);
    let sides = vec![
        capture_face(&g, "box_1", [-1.0, 0.0, 0.0]),
        capture_face(&g, "box_1", [1.0, 0.0, 0.0]),
    ];
    let neutral = capture_face(&g, "box_1", [0.0, 0.0, -1.0]);
    for angle in [0.0f32, -5.0, 89.0, 90.0, 179.0, 1e6, f32::NAN] {
        let mut g = g.clone();
        add_feature(
            &mut g,
            &format!("draft_{angle}"),
            FeatureType::Draft {
                target: "box_1".to_string(),
                faces: sides.clone(),
                neutral: DraftNeutral::Face(neutral.clone()),
                angle_deg: angle,
                angle_expr: None,
                flip_pull: false,
            },
            &["box_1"],
        );
        let (bodies, _) = eval_ok(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn draft_with_stale_and_ghost_references_does_not_panic() {
    // Face captured on a body, then a pocket changed the walls before the
    // draft; ghost neutral datum; empty face list.
    for faces in [vec![face([999.0, 10.0, 15.0], [-1.0, 0.0, 0.0])], vec![]] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 30.0);
        add_sketch_cs(
            &mut g,
            "sk_2",
            xy_plane_at(30.0),
            rect_sketch((5.0, 5.0), (15.0, 15.0)),
        );
        add_extrude(&mut g, "cut_3", "sk_2", -5.0, ExtrudeMode::Cut);
        add_feature(
            &mut g,
            "draft_4",
            FeatureType::Draft {
                target: "box_1".to_string(),
                faces,
                neutral: DraftNeutral::Datum("ghost_datum".to_string()),
                angle_deg: 5.0,
                angle_expr: None,
                flip_pull: false,
            },
            &["box_1"],
        );
        let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
    }
}

// E8 (pinned design behavior): a face selection with NO durable topology
// name and NO producer feature (a bare centroid+normal ref, as a hand-edited
// or very old file would carry) is rejected by the document-level semantic
// gate — and that rejection fails the WHOLE document evaluation, not just
// the one feature. See FINDINGS.md E8.
#[test]
fn characterization_bare_face_ref_rejects_whole_document() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "offset_2",
        FeatureType::FaceOffset {
            target: "box_1".to_string(),
            face: face([10.0, 10.0, 10.0], [0.0, 0.0, 1.0]),
            distance: 3.0,
            distance_expr: None,
        },
        &["box_1"],
    );
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok(_) => panic!("bare face ref is currently rejected at the semantic gate"),
        Err(e) => assert!(
            e.contains("invalid semantic input"),
            "expected semantic-gate rejection, got: {e}"
        ),
    }
}

// ---------------------------------------------------------------------------
// BodySplit.
// ---------------------------------------------------------------------------

#[test]
fn body_split_box_in_half() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "split_2",
        FeatureType::BodySplit {
            target: "box_1".to_string(),
            plane: PlaneBase::XY,
            face: None,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 4000.0).abs() < 10.0,
        "split must conserve volume: {v} vs 4000 ({warnings:?})"
    );
}

#[test]
fn body_split_on_ghost_plane_fails_gracefully() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "split_2",
        FeatureType::BodySplit {
            target: "box_1".to_string(),
            plane: PlaneBase::Datum("ghost".to_string()),
            face: None,
        },
        &["box_1"],
    );
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn body_split_cylinder_mid_height() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_feature(
        &mut g,
        "split_2",
        FeatureType::BodySplit {
            target: "cyl_1".to_string(),
            plane: PlaneBase::XZ,
            face: None,
        },
        &["cyl_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 100.0 * 20.0;
    assert!(
        (v - rod).abs() / rod < 0.08,
        "split cylinder must conserve volume: {v} vs {rod} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Direct face modeling (faces captured from the evaluated mesh, GUI-style).
// ---------------------------------------------------------------------------

#[test]
fn face_offset_positive_grows_box_exactly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "offset_2",
        FeatureType::FaceOffset {
            target: "box_1".to_string(),
            face: top,
            distance: 3.0,
            distance_expr: None,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 20.0 * 20.0 * 13.0).abs() / 2600.0 < 0.02,
        "positive face offset: {v} ({warnings:?})"
    );
}

// E12 regression (fixed 2026-09-11): NEGATIVE FaceOffset used to remove one
// model-scale overshoot MORE than requested, silently, at every magnitude
// (−1 → removed 2, −4 → removed 5, −6 → removed 7): the inward cutter swept
// `distance − 2·overshoot` from `+overshoot`, landing one overshoot past the
// requested plane. The sweep now stops exactly at `distance`, so the moved
// face lands on the requested plane — asserted on the mesh's top-plane z,
// not just the volume.
#[test]
fn negative_face_offset_lands_exactly_on_the_requested_plane() {
    for (distance, expected_height) in [(-1.0f32, 9.0f32), (-4.0, 6.0), (-6.0, 4.0), (-9.5, 0.5)] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
        add_feature(
            &mut g,
            "offset_2",
            FeatureType::FaceOffset {
                target: "box_1".to_string(),
                face: top,
                distance,
                distance_expr: None,
            },
            &["box_1"],
        );
        let (bodies, warnings) = eval_ok(&g);
        assert_meshes_finite(&bodies);
        assert!(
            warnings.is_empty(),
            "offset {distance} must be silent: {warnings:?}"
        );
        let expected_volume = 20.0 * 20.0 * f64::from(expected_height);
        let v = total_volume(&bodies);
        assert!(
            (v - expected_volume).abs() / expected_volume < 0.01,
            "offset {distance}: expected volume {expected_volume}, got {v}"
        );
        // The moved top face must sit exactly on the requested plane. (The
        // offset commit gives the body the feature's id.)
        assert_eq!(bodies.len(), 1, "one body after the offset: {bodies:?}");
        let (_, mesh) = &bodies[0];
        let top_z = mesh
            .vertices
            .chunks_exact(6)
            .map(|vertex| vertex[2])
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            (top_z - expected_height).abs() < 0.02,
            "offset {distance}: the moved face plane must be z = {expected_height}, got {top_z}"
        );
    }
}

#[test]
fn face_offset_degenerate_distances_do_not_panic() {
    for distance in [0.0f32, -10.0, -10.001, -1e9, 1e9, f32::NAN, f32::INFINITY] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
        add_feature(
            &mut g,
            "offset_2",
            FeatureType::FaceOffset {
                target: "box_1".to_string(),
                face: top,
                distance,
                distance_expr: None,
            },
            &["box_1"],
        );
        let (bodies, _) = eval_ok(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn face_move_top_face_along_normal() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "move_2",
        FeatureType::FaceMove {
            target: "box_1".to_string(),
            face: top,
            translation: [0.0, 0.0, 5.0],
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 20.0 * 20.0 * 15.0).abs() / 3000.0 < 0.02,
        "face move along normal: {v} ({warnings:?})"
    );
}

// E9 (verdict revised 2026-09-11): FaceMove with a tangential component is
// SUPPORTED for an exact six-face block — the evaluator rebuilds the box as
// a sheared parallelepiped, moving the selected top face by the FULL
// translation vector while the opposite face stays put (volume is unchanged
// by a shear, which is why the old volume-only probe misread this as "the
// tangential part was dropped"). Assertions pin the moved top vertices and
// the untouched opposite face, not volume alone.
#[test]
fn face_move_applies_full_tangential_translation_on_six_face_blocks() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "move_2",
        FeatureType::FaceMove {
            target: "box_1".to_string(),
            face: top,
            translation: [5.0, 0.0, 5.0],
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.is_empty(),
        "the supported tangential block move must be silent: {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "one sheared body: {bodies:?}");
    let (_, mesh) = &bodies[0];
    let vertices: Vec<[f32; 3]> = mesh
        .vertices
        .chunks_exact(6)
        .map(|v| [v[0], v[1], v[2]])
        .collect();
    // Shear preserves volume: 20×20×10 block, top plane lifted to z = 15.
    let v = total_volume(&bodies);
    assert!(
        (v - 6000.0).abs() / 6000.0 < 0.02,
        "a shear keeps the volume: {v}"
    );
    // The moved top face sits at z = 15, shifted +5 in x.
    let top_z_max = vertices
        .iter()
        .map(|p| p[2])
        .fold(f32::NEG_INFINITY, f32::max);
    assert!((top_z_max - 15.0).abs() < 0.02, "top plane z: {top_z_max}");
    let top_vertices: Vec<_> = vertices.iter().filter(|p| p[2] > 14.9).collect();
    assert!(!top_vertices.is_empty(), "moved top vertices exist");
    assert!(
        top_vertices.iter().all(|p| p[0] >= 4.9 && p[0] <= 25.1),
        "the top face shifts +5 in x: {top_vertices:?}"
    );
    // The opposite face is untouched: z = 0, x spans the original [0, 20].
    let bottom_vertices: Vec<_> = vertices.iter().filter(|p| p[2] < 0.1).collect();
    assert!(
        bottom_vertices
            .iter()
            .all(|p| (p[0] - 0.0).abs() < 0.1 || (p[0] - 20.0).abs() < 0.1),
        "the bottom face keeps its x extent: {bottom_vertices:?}"
    );
}

#[test]
fn face_delete_removes_hole() {
    // Drill a hole, then delete the hole's cylindrical wall — Phase 5's
    // supported heal case. Either heal or documented rejection is graceful.
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
        zerocad_core::HoleKind::Simple,
    );
    // Capture the cylindrical wall face: curved faces are listed with the
    // hole-axis centroid and a zero normal (undefined at the centroid).
    let (bodies, _) = eval_ok(&g);
    let mesh = &bodies.iter().find(|(id, _)| id == "box_1").unwrap().1;
    let wall = mesh
        .face_refs
        .iter()
        .find(|f| {
            f.normal == [0.0, 0.0, 0.0]
                && (f.centroid[0] - 10.0).abs() < 0.5
                && (f.centroid[1] - 10.0).abs() < 0.5
                && (f.centroid[2] - 5.0).abs() < 0.5
        })
        .expect("hole wall face should be selectable");
    let wall_ref = FaceRef {
        centroid: wall.centroid,
        normal: wall.normal,
        topology: wall.topology.as_ref().map(|t| TopologyFaceRef {
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
            face: wall_ref,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let healed = 4000.0;
    let rejected = 4000.0 - std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (v - healed).abs() < 10.0 || (v - rejected).abs() / rejected < 0.02,
        "face delete must heal or reject: {v} ({warnings:?})"
    );
}

// E10 regression (fixed 2026-09-11): FaceThicken is documented to leave the
// source body in the document ("the source remains in the document and the
// new feature owns the thickened result") — but the planar path REPLACED the
// source component, silently deleting the 4000 mm³ box and leaving only the
// 1600 mm³ plate. All four surface families (planar/cylindrical/conical/
// spherical) now commit through the shared create-new-body route: the source
// body is immutable and the thickened solid becomes a new body owned by the
// feature node.
#[test]
fn face_thicken_keeps_source_and_creates_new_body() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "thicken_2",
        FeatureType::FaceThicken {
            target: "box_1".to_string(),
            face: top,
            thickness: 4.0,
            thickness_expr: None,
            reverse: false,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.is_empty(),
        "the documented create-new-body behavior must be silent: {warnings:?}"
    );
    assert_eq!(
        bodies.len(),
        2,
        "source box plus thickened plate: {bodies:?}"
    );
    let volume_of = |id: &str| -> f64 {
        let (_, mesh) = bodies
            .iter()
            .find(|(body_id, _)| body_id == id)
            .unwrap_or_else(|| panic!("body '{id}' missing from {bodies:?}"));
        mesh.mass_properties()
            .map(|mp| mp.volume)
            .unwrap_or(f64::NAN)
    };
    assert!(
        (volume_of("box_1") - 4000.0).abs() < 40.0,
        "the source box must survive unchanged: {}",
        volume_of("box_1")
    );
    assert!(
        (volume_of("thicken_2") - 1600.0).abs() < 16.0,
        "the feature owns the 20×20×4 plate: {}",
        volume_of("thicken_2")
    );
}

// ---------------------------------------------------------------------------
// FeaturePattern (replicating a hole / cut extrude).
// ---------------------------------------------------------------------------

#[test]
fn feature_pattern_replicates_hole_linear() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::X,
                spacing: 20.0,
                spacing_expr: None,
                count: 5,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            // Through-all holes must use the through-all extent policy.
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 100.0 * 30.0 * 10.0 - 5.0 * std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "5-hole pattern: {v} vs {exact} ({warnings:?})"
    );
}

#[test]
fn feature_pattern_blind_hole_with_source_extent() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        Some(6.0),
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::X,
                spacing: 20.0,
                spacing_expr: None,
                count: 5,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::SourceExtent,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 100.0 * 30.0 * 10.0 - 5.0 * std::f64::consts::PI * 9.0 * 6.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "5 blind-hole pattern: {v} vs {exact} ({warnings:?})"
    );
}

// E11 corrected control (expected miss): rotating the hole at (30, 40)
// around the WORLD Z axis lands the first instance at (-40, 30) — outside
// the 60×60 box that spans x, y ∈ [0, 60]. The pattern therefore correctly
// rolls back with "instance 1 failed (the instance missed the target or made
// no material change)": the no-material-change guard is doing its job on a
// genuinely-missing target. The historical characterization framed this as a
// bug; the review traced it to the wrong axis choice in the fixture.
#[test]
fn circular_pattern_instances_off_the_target_roll_back() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [30.0, 40.0, 10.0],
        [0.0, 0.0, -1.0],
        4.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::Z,
                total_angle_deg: 360.0,
                total_angle_expr: None,
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("instance 1 failed")),
        "an instance that misses the target must roll the pattern back: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let only_source = 36000.0 - std::f64::consts::PI * 4.0 * 10.0;
    assert!(
        (v - only_source).abs() / only_source < 0.02,
        "rollback keeps exactly the source hole: {v} (expected {only_source})"
    );
}

// E11 regression (fixed 2026-09-11, box fixture): the same bolt circle with
// the axis through the BOX CENTER (30, 30) — every instance lands in
// material, so the full ring must apply. The historical failure here was the
// shared conservative-bounds defect (see the cylinder variant): the rotated
// hole instance's vertex-only AABB missed the curved cut region, so the
// material-delta recovery judged the instance as "no material change".
#[test]
fn circular_feature_pattern_box_center_bolt_circle_applies() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 60.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [40.0, 30.0, 10.0],
        [0.0, 0.0, -1.0],
        4.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::TwoPoints {
                    a: [30.0, 30.0, 0.0],
                    b: [30.0, 30.0, 10.0],
                },
                total_angle_deg: 360.0,
                total_angle_expr: None,
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        !warnings.iter().any(|w| w.contains("instance 1 failed")),
        "a bolt circle whose every instance hits material must not roll back: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let full_ring = 36000.0 - 4.0 * std::f64::consts::PI * 4.0 * 10.0;
    assert!(
        (v - full_ring).abs() / full_ring < 0.02,
        "the full ring of four r=2 holes must apply: {v} (expected {full_ring})"
    );
}

// E11 regression (fixed 2026-09-11, cylinder fixture): a 6-hole bolt circle
// (r=13 bores, r=20 rod) on the cylinder cap. This fixture reproduced the
// real defect — the instance recovery used vertex-only bounds, and the
// rotated cylindrical cut region was judged not to overlap the target. With
// conservative bounds the ring applies exactly.
#[test]
fn circular_feature_pattern_on_cylinder_cap_applies() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "cyl_1",
        [13.0, 10.0, 0.0],
        [0.0, -1.0, 0.0],
        3.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "cyl_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::Y,
                total_angle_deg: 360.0,
                total_angle_expr: None,
                count: 6,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        },
        &["cyl_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        !warnings.iter().any(|w| w.contains("instance 1 failed")),
        "a bolt circle on the cylinder cap must not roll back: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let full_ring = std::f64::consts::PI * 400.0 * 10.0 - 6.0 * std::f64::consts::PI * 2.25 * 10.0;
    assert!(
        (v - full_ring).abs() / full_ring < 0.03,
        "the full ring of six r=1.5 bores must apply: {v} (expected {full_ring})"
    );
}

#[test]
fn feature_pattern_degenerate_sources_do_not_panic() {
    for (source, count, extent) in [
        (
            "hole_2".to_string(),
            0u32,
            FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        ),
        (
            "hole_2".to_string(),
            1,
            FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        ),
        (
            "ghost".to_string(),
            4,
            FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        ),
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 60.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [15.0, 15.0, 10.0],
            [0.0, 0.0, -1.0],
            6.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
        add_feature(
            &mut g,
            "pattern_3",
            FeatureType::FeaturePattern {
                target: "box_1".to_string(),
                source_feature: source,
                kind: FeaturePatternKind::Linear {
                    direction: AxisBase::X,
                    spacing: 10.0,
                    spacing_expr: None,
                    count,
                },
                compute_mode: FeaturePatternComputeMode::Identical,
                extent_policy: extent,
            },
            &["box_1"],
        );
        let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
    }
}

// ---------------------------------------------------------------------------
// Mirror patterns.
// ---------------------------------------------------------------------------

#[test]
fn mirror_pattern_creates_reflected_copy() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0); // spans 0..10
    add_feature(
        &mut g,
        "mirror_2",
        FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Mirror {
                plane: PlaneBase::XY,
                face: None,
                offset: 0.0,
                offset_expr: None,
                join: false,
            },
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "mirror copy doubles volume: {v} ({warnings:?})"
    );
}

#[test]
fn mirror_pattern_join_touching_bodies_fuses() {
    // Source 0..10 mirrored across YZ → copy −10..0 touching at x = 0.
    // join=true must keep both halves as one fused body.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "mirror_2",
        FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Mirror {
                plane: PlaneBase::YZ,
                face: None,
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 2000.0).abs() / 2000.0 < 0.02,
        "joined mirror keeps both halves: {v} ({warnings:?})"
    );
}

#[test]
fn mirror_pattern_extreme_finite_offsets_do_not_panic() {
    for offset in [-5.0f32, 1e6] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
        add_feature(
            &mut g,
            "mirror_2",
            FeatureType::Pattern {
                source: "box_1".to_string(),
                kind: zerocad_core::PatternKind::Mirror {
                    plane: PlaneBase::XY,
                    face: None,
                    offset,
                    offset_expr: None,
                    join: true,
                },
            },
            &["box_1"],
        );
        let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
    }
}

// A6 regression (fixed 2026-09-11): a NaN mirror offset built a NaN
// translation and panicked inside the kernel's transform validation
// ("a rigid transform of a valid solid must remain valid", NonFiniteVertex in
// the report) — same root cause family as the NaN linear-pattern spacing
// (FINDINGS.md A4). The mirror offset — including the offset-expression
// result — is validated before the transform; the feature resolves Unresolved
// with a PARAMETER_INVALID diagnostic and the source body survives.
#[test]
fn mirror_pattern_nan_offset_resolves_to_parameter_diagnostic() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "mirror_2",
        FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Mirror {
                plane: PlaneBase::XY,
                face: None,
                offset: f32::NAN,
                offset_expr: None,
                join: true,
            },
        },
        &["box_1"],
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("NaN mirror offset must resolve to a warning, not an error");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|m| m.contains("offset must be a finite number")),
        "NaN mirror offset must produce the parameter warning, got {warnings:?}"
    );
    assert_eq!(
        bodies.len(),
        1,
        "only the source body may remain: {bodies:?}"
    );
    assert!((total_volume(&bodies) - 1000.0).abs() < 0.5);
}

// ---------------------------------------------------------------------------
// Datum-anchored construction.
// ---------------------------------------------------------------------------

#[test]
fn sketch_on_offset_datum_plane_extrudes() {
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "datum_1",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
        &[],
    );
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "ex_3", "sk_2", 8.0, ExtrudeMode::NewBody);
    g.add_dependency("datum_1", "sk_2");
    let (bodies, warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);
    assert!(
        !warnings.iter().any(|w| w.contains("datum")),
        "datum must resolve: {warnings:?}"
    );
}

#[test]
fn datum_three_collinear_points_is_degenerate_but_survivable() {
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "datum_1",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::ThreePoints {
                a: [0.0, 0.0, 0.0],
                b: [10.0, 0.0, 0.0],
                c: [20.0, 0.0, 0.0],
            },
        },
        &[],
    );
    add_sketch(&mut g, "sk_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "ex_3", "sk_2", 5.0, ExtrudeMode::NewBody);
    g.add_dependency("datum_1", "sk_2");
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn datum_chain_and_cycle() {
    // Legal chain: datum_2 offset from datum_1; a sketch depends on datum_2.
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "datum_1",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 2.0,
                distance_expr: None,
            },
        },
        &[],
    );
    add_feature(
        &mut g,
        "datum_2",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_1".to_string()),
                distance: 3.0,
                distance_expr: None,
            },
        },
        &["datum_1"],
    );
    add_sketch(&mut g, "sk_3", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "ex_4", "sk_3", 5.0, ExtrudeMode::NewBody);
    g.add_dependency("datum_2", "sk_3");
    let (bodies, _warnings) = eval_ok(&g);
    assert_meshes_finite(&bodies);

    // Cycle: datum_1 offset from datum_2 while datum_2 offsets from datum_1 —
    // the toposort must catch this without hanging or panicking.
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "datum_1",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_2".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        },
        &[],
    );
    add_feature(
        &mut g,
        "datum_2",
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_1".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        },
        &["datum_1"],
    );
    add_sketch(&mut g, "sk_3", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "ex_4", "sk_3", 5.0, ExtrudeMode::NewBody);
    g.add_dependency("datum_1", "sk_3");
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}
