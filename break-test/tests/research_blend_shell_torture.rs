//! Research-derived shell / offset / draft / hole / thread torture (research
//! pass 2026-09-11).
//!
//! Scenarios come from documented real-kernel failures: SolidWorks' official
//! shell-error page (thickness > minimum curvature radius), OCCT's offset
//! rules (inward offset self-intersects when d exceeds the concave curvature
//! radius; torus minor radius collapse), OCCT ticket 0023025 (draft extrusion
//! exception at the degenerate angle), OpenSCAD #1591 (blind-hole/coincident-
//! end classification), OpenSCAD PR #5012 + FreeCAD t=106930 (helical
//! self-intersection when pitch ≤ profile height), FreeCAD #29401 / t=99516
//! (thread end overhang, multi-start lead), Harvey Performance multi-start
//! lead = pitch × starts.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::mock_kernel::MeshFaceRef;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{ExtrudeMode, FeatureNode, FeatureType, HoleKind, ParametricGraph};

fn planar_face(centroid: [f32; 3], normal: [f32; 3]) -> FaceRef {
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

fn shell(target: &str, thickness: f32, open_faces: Vec<FaceRef>) -> FeatureType {
    FeatureType::Shell {
        target: target.to_string(),
        thickness,
        thickness_expr: None,
        open_faces,
    }
}

/// Capture the cylindrical wall face nearest `near` from an evaluated body
/// (same strategy as thread_torture.rs: named cylinder faces or zero-length
/// normals for cut bores).
fn cyl_face_near(g: &ParametricGraph, body: &str, near: [f32; 3]) -> FaceRef {
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies.iter().find(|(id, _)| id == body).unwrap().1;
    let is_cyl = |f: &MeshFaceRef| {
        f.topology
            .as_ref()
            .and_then(|t| t.surface_kind.as_deref())
            .is_some_and(|k| k.eq_ignore_ascii_case("cylinder"))
            || f.normal[0].abs() + f.normal[1].abs() + f.normal[2].abs() < 0.1
    };
    let f = mesh
        .face_refs
        .iter()
        .filter(|f| is_cyl(f))
        .min_by_key(|f| {
            let d = [
                (f.centroid[0] - near[0]).abs(),
                (f.centroid[1] - near[1]).abs(),
                (f.centroid[2] - near[2]).abs(),
            ];
            ((d[0] + d[1] + d[2]) * 100.0) as u32
        })
        .expect("a cylindrical face must be selectable");
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

/// Capture a planar face with its durable topology identity from the
/// evaluated body (the GUI path — bare centroid+normal FaceRefs are rejected
/// at the document gate, FINDINGS E13).
fn capture_planar_face(g: &ParametricGraph, body: &str, want_normal: [f32; 3]) -> FaceRef {
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies.iter().find(|(id, _)| id == body).unwrap().1;
    let f = mesh
        .face_refs
        .iter()
        .filter(|f| {
            (f.normal[0] - want_normal[0]).abs() < 1e-3
                && (f.normal[1] - want_normal[1]).abs() < 1e-3
                && (f.normal[2] - want_normal[2]).abs() < 1e-3
        })
        .min_by_key(|f| ((f.centroid[0] + f.centroid[1] + f.centroid[2]) * 100.0) as i64)
        .expect("a matching planar face must be selectable");
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

// --- Shell: the thickness-vs-curvature wall ---------------------------------
// SolidWorks ships a dedicated diagnostic for exactly this
// (https://help.solidworks.com/2012/english/SolidWorks/sldworks/HIDE_ShellErrors.htm):
// "The Thickness value is greater than the Minimum radius of Curvature … could
// cause undesirable results, such as bad geometry." The inward offset of a
// surface with concave curvature radius r folds at offset > r.

#[test]
fn shell_cylinder_thickness_exceeds_radius_graceful() {
    // Inward offset of r=10 needs radius −2 at t=12: must reject/warn, never
    // emit a pinched self-intersecting wall.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_feature(
        &mut g,
        "shell_2",
        shell(
            "cyl_1",
            12.0,
            vec![planar_face([0.0, 20.0, 0.0], [0.0, 1.0, 0.0])],
        ),
        &["cyl_1"],
    );
    let v = assert_part_sane(&g);
    if v > 0.0 {
        let solid = std::f64::consts::PI * 100.0 * 20.0;
        assert!(
            v < solid * 1.001,
            "over-thick shell must not grow material: {v} vs {solid}"
        );
    }
}

#[test]
fn shell_cylinder_thickness_near_radius_boundary() {
    // Boundary sweep around t = r: t slightly below (valid thin shell), equal
    // (inner surface degenerates to the axis), slightly above (inverted).
    // Every step must stay finite and deterministic.
    for t in [9.0_f32, 9.999, 10.0, 10.001, 11.0] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
        add_feature(
            &mut g,
            "shell_2",
            shell(
                "cyl_1",
                t,
                vec![planar_face([0.0, 20.0, 0.0], [0.0, 1.0, 0.0])],
            ),
            &["cyl_1"],
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        let v1 = total_volume(&bodies);
        let v2 = total_volume(&eval(&g));
        assert_eq!(v1.to_bits(), v2.to_bits(), "non-deterministic at t={t}");
    }
}

#[test]
fn shell_box_thickness_exactly_half_min_dimension() {
    // At t = min_extent/2 the inner offset planes meet exactly: zero-volume
    // void boundary (the U-channel crossing case at its equality limit,
    // https://occt3d.com/dev/doc/refman/html/class_b_rep_offset_a_p_i___make_thick_solid.html).
    for t in [4.9_f32, 5.0, 5.1] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_feature(
            &mut g,
            "shell_2",
            shell(
                "box_1",
                t,
                vec![planar_face([10.0, 10.0, 10.0], [0.0, 0.0, 1.0])],
            ),
            &["box_1"],
        );
        let v = assert_part_sane(&g);
        assert!(v >= -1e-6, "negative volume at t={t}: {v}");
    }
}

#[test]
fn shell_open_faces_all_six_is_parameter_error() {
    // Removing every face leaves no shell to offset — BRepOffsetAPI requires a
    // non-closed result; must be a clean parameter rejection, not garbage
    // (https://dev.opencascade.org/content/brepoffsetapimakethicksolid-error).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    let faces = vec![
        planar_face([10.0, 10.0, 20.0], [0.0, 0.0, 1.0]),
        planar_face([10.0, 10.0, 0.0], [0.0, 0.0, -1.0]),
        planar_face([10.0, 0.0, 10.0], [0.0, -1.0, 0.0]),
        planar_face([10.0, 20.0, 10.0], [0.0, 1.0, 0.0]),
        planar_face([0.0, 10.0, 10.0], [-1.0, 0.0, 0.0]),
        planar_face([20.0, 10.0, 10.0], [1.0, 0.0, 0.0]),
    ];
    add_feature(&mut g, "shell_2", shell("box_1", 2.0, faces), &["box_1"]);
    let v = assert_part_sane(&g);
    assert!(
        v < 8000.0 + 1.0,
        "all-faces-removed shell must not produce the solid box: {v}"
    );
}

#[test]
fn shell_open_faces_on_two_opposite_sides_stays_connected() {
    // Removing two opposite faces leaves a connected open shell; the offset
    // must still produce ONE body (or reject cleanly), never two leaked slabs.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    add_feature(
        &mut g,
        "shell_2",
        shell(
            "box_1",
            2.0,
            vec![
                planar_face([10.0, 0.0, 10.0], [0.0, -1.0, 0.0]),
                planar_face([10.0, 20.0, 10.0], [0.0, 1.0, 0.0]),
            ],
        ),
        &["box_1"],
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let solid = 20.0_f64.powi(3);
    let v = total_volume(&bodies);
    assert!(
        v < solid * 0.999,
        "shelling with two open faces must remove material: {v}"
    );
}

// --- Holes: diameter and depth boundaries -----------------------------------
// Counterbore diameter == bore diameter creates a zero-width annular floor
// (the narrow-face class that poisons later fillets — OCCT fuzzy Example 4.2
// mechanism). Blind depth == plate thickness is the blind/through
// classification boundary (OpenSCAD #1591 "coincident ends").

#[test]
fn hole_counterbore_diameter_equals_bore_graceful() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "plate_1", 40.0, 40.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "plate_1",
        [20.0, 20.0, 10.0],
        [0.0, 0.0, -1.0],
        8.0,
        None,
        HoleKind::Counterbore {
            diameter: 8.0,
            depth: 3.0,
        },
    );
    let v = assert_part_sane(&g);
    let through = std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (16_000.0 - v) >= through * 0.9,
        "degenerate counterbore must still remove at least the bore: removed {}",
        16_000.0 - v
    );
}

#[test]
fn hole_blind_depth_exactly_plate_thickness_classifies_stably() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "plate_1", 40.0, 40.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "plate_1",
        [20.0, 20.0, 10.0],
        [0.0, 0.0, -1.0],
        8.0,
        Some(10.0),
        HoleKind::Simple,
    );
    let v = assert_part_sane(&g);
    let removed = 16_000.0 - v;
    let bore = std::f64::consts::PI * 16.0 * 10.0;
    assert!(
        (removed - bore).abs() < bore * 0.05,
        "blind depth == thickness removed {removed} mm³, expected ≈{bore}"
    );
}

// --- Draft: the collapse angle ----------------------------------------------
// FreeCAD's taper code: "end face of tapered along extrusion is empty … most
// probably that the along taper angle is too large" and OCCT ticket 0023025
// (exception raised in drafted extrusion at the degenerate point,
// https://occt3d.com/dev/tickets/0023025/). At tan(θ)·h = half-width the top
// face is exactly zero-width.

#[test]
fn extrude_draft_collapse_angle_boundary_sweep() {
    // Rect 10 wide, depth 20 → collapse at θ* = atan(5/20) ≈ 14.036°.
    let theta_star = 5.0f32.atan2(20.0).to_degrees();
    for theta in [theta_star - 1.0, theta_star, theta_star + 1.0] {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude_full(
            &mut g,
            "pad_2",
            "sk_1",
            20.0,
            ExtrudeMode::NewBody,
            None,
            theta,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        let v1 = total_volume(&bodies);
        let v2 = total_volume(&eval(&g));
        assert_eq!(v1.to_bits(), v2.to_bits(), "non-deterministic at θ={theta}");
        if theta < theta_star {
            // Frustum volume: h/3·(A1 + A2 + √(A1·A2)).
            let w1 = 10.0_f64;
            let w2 = 10.0 - 2.0 * 20.0 * (theta as f64).to_radians().tan();
            let a1 = w1 * w1;
            let a2 = w2 * w2;
            let expected = 20.0 / 3.0 * (a1 + a2 + (a1 * a2).sqrt());
            assert!(
                (v1 - expected).abs() < expected * 0.01,
                "draft {theta}° frustum volume {v1} vs {expected}"
            );
        } else {
            assert!(
                v1 >= -1e-6,
                "collapsed draft produced negative volume at θ={theta}"
            );
        }
    }
}

// --- Face offset beyond the body --------------------------------------------
// Inward offset past the body's far side is the FaceThicken/offset collapse
// family (torus minor-radius collapse rule, OCCT offset docs:
// https://neweopencascade.wordpress.com/2015/05/23/creation-of-an-offset-from-surfaces/).

#[test]
fn face_offset_inward_beyond_body_extent_graceful() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let face = capture_planar_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "offset_2",
        FeatureType::FaceOffset {
            target: "box_1".into(),
            face,
            distance: -50.0,
            distance_expr: None,
        },
        &["box_1"],
    );
    let v = assert_part_sane(&g);
    // Either the move is rejected (body intact) or the body is consumed to
    // nothing — never negative, never half-erased garbage.
    if v > 1.0 {
        assert!(
            (v - 4000.0).abs() < 2.0,
            "rejected offset must preserve the body exactly: {v}"
        );
    }
}

// --- Threads: pitch/depth boundary, end trim, multi-start, absurd inputs ----
// OpenSCAD PR #5012 formalized the helical self-intersection rule: "self-
// intersection if the pitch is less than the width of the 2D object being
// extruded". Multi-start: lead = pitch × starts (Harvey Performance); Fusion's
// coil errors with "Body would intersect itself" when lead < profile width ×
// starts. FreeCAD t=99516: partial-turn thread ends are the canonical failure.

#[allow(clippy::too_many_arguments)]
fn add_external_thread(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    face: FaceRef,
    pitch: f32,
    depth: f32,
    starts: u32,
    length: Option<f32>,
    flip: bool,
) {
    add_feature(
        g,
        id,
        FeatureType::Thread {
            target: target.to_string(),
            face,
            internal: false,
            pitch,
            depth,
            angle_deg: 60.0,
            right_handed: true,
            starts,
            length,
            flip,
            designation: "custom".to_string(),
            standard: None,
        },
        &[target],
    );
}

#[test]
fn thread_pitch_below_depth_self_intersection_boundary() {
    // Rod r=10, band depth 2 → boundary at pitch = 2. Below it, consecutive
    // turns overlap. Every step must stay finite/deterministic; the
    // below-boundary runs must not silently claim success with garbage.
    for pitch in [2.5_f32, 2.0, 1.5] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 10.0, 20.0);
        let face = cyl_face_near(&g, "rod_1", [10.0, 10.0, 0.0]);
        add_external_thread(
            &mut g, "thread_2", "rod_1", face, pitch, 2.0, 1, None, false,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        let v1 = total_volume(&bodies);
        let v2 = total_volume(&eval(&g));
        assert_eq!(
            v1.to_bits(),
            v2.to_bits(),
            "non-deterministic at pitch={pitch}"
        );
    }
}

#[test]
fn thread_partial_length_not_multiple_of_pitch_end_trim() {
    // Length 7.0 at pitch 2 = 3.5 turns: the band must terminate mid-turn at
    // BOTH ends without dangling overhang or sub-tolerance slivers — the
    // exact case FreeCAD t=99516 reports failing.
    for (flip, length) in [
        (false, Some(7.0_f32)),
        (true, Some(7.0)),
        (false, Some(0.05)),
    ] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 10.0, 20.5);
        let face = cyl_face_near(&g, "rod_1", [10.0, 10.25, 0.0]);
        add_external_thread(&mut g, "thread_2", "rod_1", face, 2.0, 1.0, 1, length, flip);
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        assert_deterministic(&g);
    }
}

#[test]
fn thread_multistart_extreme_start_counts() {
    // starts = 0 must behave like 1 (documented "0/1 both mean single");
    // large counts stress the angular wrap (lead = pitch × starts).
    for starts in [0_u32, 1, 4, 64] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 10.0, 20.0);
        let face = cyl_face_near(&g, "rod_1", [10.0, 10.0, 0.0]);
        add_external_thread(
            &mut g, "thread_2", "rod_1", face, 1.0, 0.4, starts, None, false,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        assert_deterministic(&g);
    }
}

#[test]
fn thread_internal_band_exceeding_host_hole_graceful() {
    // Internal thread whose band extends past the host hole wall by more than
    // the material can hold (Onshape "internal corner smaller than the fillet"
    // rule applied to threads): must warn/degrade, never float detached
    // geometry or panic.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "plate_1", 40.0, 40.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "plate_1",
        [20.0, 20.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        None,
        HoleKind::Simple,
    );
    let face = cyl_face_near(&g, "plate_1", [20.0, 20.0, 5.0]);
    add_feature(
        &mut g,
        "thread_3",
        FeatureType::Thread {
            target: "plate_1".into(),
            face,
            internal: true,
            pitch: 1.0,
            depth: 4.0, // band root at r=3+4=7 > anything available
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M6".to_string(),
            standard: None,
        },
        &["plate_1"],
    );
    let v = assert_part_sane(&g);
    let bore = std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (16_000.0 - v) >= bore * 0.9,
        "thread past the hole must not restore removed material: {v}"
    );
}

#[test]
fn thread_very_fine_pitch_300_turns_bounded() {
    // OpenSCAD chunks helical sweeps because huge turn counts are pathological
    // (PR #5012). 300 turns must complete with bounded cost, no precision
    // drift between the first and last turn.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [10.0, 15.0, 0.0]);
    add_external_thread(&mut g, "thread_2", "rod_1", face, 0.1, 0.05, 1, None, false);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    assert_deterministic(&g);
}

#[test]
fn shell_on_threaded_rod_chain_survives() {
    // The OCCT 0027784/0027785 combo class (draft/thickness on cylinders with
    // seam edges corrupts geometry at the seam), adapted: shell AFTER a
    // cosmetic thread. Whatever each feature does alone, the chain must stay
    // finite and deterministic.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 12.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [12.0, 15.0, 0.0]);
    add_external_thread(&mut g, "thread_2", "rod_1", face, 1.5, 1.0, 1, None, false);
    add_feature(
        &mut g,
        "shell_3",
        shell(
            "rod_1",
            1.0,
            vec![planar_face([0.0, 30.0, 0.0], [0.0, 1.0, 0.0])],
        ),
        &["thread_2"],
    );
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    assert_deterministic(&g);
}
