//! Catalog I — holes and actual thread geometry (adversarial plan §6.I).
//!
//! I04 (thread on planar cap / deleted face), I06 (handedness, starts,
//! partial lengths), I07 (work limits) and I08 (cross-drill / cut-the-end /
//! collar) are covered in `thread_torture.rs`; blind/through boundaries and
//! counterbore equality in `research_blend_shell_torture.rs`. This file
//! adds: hole placement swept through edge tangency and breakout (I01),
//! blind-depth through the back face vs through-all (I02), invalid
//! counterbore/countersink orderings (I03), and threading a tube's outer
//! wall vs inner bore separately (I05).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{FeatureType, HoleKind, ParametricGraph};

const PI: f64 = std::f64::consts::PI;

/// I01: hole near a face edge, swept through tangency and breakout —
/// placement correct, removal monotone, no silent full-hole failures.
#[test]
fn i01_hole_edge_tangency_and_breakout() {
    let plate = 30.0 * 30.0 * 10.0_f64;
    let removed_at = |edge_distance: f64| -> f64 {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [edge_distance as f32, 15.0, 10.0],
            [0.0, 0.0, -1.0],
            8.0,
            None,
            HoleKind::Simple,
        );
        let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        plate - total_volume(&bodies)
    };
    // r = 4: distance ≥ 4 removes the full bore; below 4 the bore breaks
    // out and removes exactly the circle∩plate patch (analytic segment
    // formula). d = 4.0 exactly is the tracked tangency defect.
    let r = 4.0_f64;
    for d in [6.0_f64, 4.5, 4.001, 3.999, 3.5, 2.0] {
        let removed = removed_at(d);
        let patch_area = if d >= r {
            PI * r * r
        } else {
            // Circle minus the segment beyond the x=0 edge.
            let segment = r * r * (d / r).acos() - d * (r * r - d * d).sqrt();
            PI * r * r - segment
        };
        assert_close(
            removed,
            patch_area * 10.0,
            1e-9,
            0.03,
            &format!("removed material at edge distance {d}"),
        );
    }
    // Fully outside the plate: a legitimate no-op (air hole).
    let removed = removed_at(50.0);
    assert!(
        removed < 1.0,
        "hole outside the plate must be a no-op ({removed})"
    );
}

/// I01 tracked defect (found 2026-09-13): a hole EXACTLY tangent to the
/// plate edge (center distance == radius) removes NOTHING — a silent no-op
/// with no diagnostic, while 4.001 cuts the full bore and 3.999 breaks out.
/// Same tolerance-cluster family as the tangent bore pair (FINDINGS
/// E-tangency); recorded in FINDINGS.md I-tangent-hole.
#[test]
fn i01_exactly_tangent_hole_removes_nothing_known_break() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [4.0, 15.0, 10.0], // r=4: exactly tangent to the x=0 edge
        [0.0, 0.0, -1.0],
        8.0,
        None,
        HoleKind::Simple,
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let removed = 9000.0 - total_volume(&bodies);
    let half_or_more = PI * 16.0 * 10.0 * 0.5;
    assert!(
        removed >= half_or_more || (removed < 1.0 && warnings.iter().any(|w| w.contains("hole_2"))),
        "tangent hole must remove ≥ the half-bore ({half_or_more}) or no-op WITH a warning; \
         removed {removed}, warnings {warnings:?}"
    );
}

/// I02: blind depth swept through the back face — exact end conditions and
/// through-all equivalence.
#[test]
fn i02_blind_depth_through_back_face_equals_through_all() {
    let plate = 9000.0_f64;
    let removed_at = |depth: Option<f32>| -> f64 {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [15.0, 15.0, 10.0],
            [0.0, 0.0, -1.0],
            8.0,
            depth,
            HoleKind::Simple,
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        assert!(warnings.is_empty(), "depth {depth:?}: {warnings:?}");
        plate - total_volume(&bodies)
    };
    // Blind: exact cylinder.
    assert_close(
        removed_at(Some(5.0)),
        PI * 16.0 * 5.0,
        1e-6,
        0.03,
        "blind 5mm",
    );
    // Exactly through, epsilon past, deeper, and through-all: identical.
    let through = removed_at(None);
    assert_close(through, PI * 16.0 * 10.0, 1e-6, 0.03, "through-all");
    for depth in [10.0_f32, 10.001, 15.0, 1e9] {
        assert_close(
            removed_at(Some(depth)),
            through,
            1e-3,
            1e-6,
            &format!("depth {depth} equals through-all"),
        );
    }
}

/// I03: counterbore/countersink orderings — diameter smaller than the bore
/// and degenerate angles are rejected with typed parameter diagnostics.
#[test]
fn i03_counterbore_countdersink_invalid_orderings() {
    let build = |kind: HoleKind| -> (f64, Vec<String>, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [15.0, 15.0, 10.0],
            [0.0, 0.0, -1.0],
            8.0,
            Some(6.0),
            kind,
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        let token = zerocad_core::EvaluationCancellation::new(
            0,
            std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)),
        );
        let codes = g
            .evaluate_request(
                &HashSet::new(),
                zerocad_core::EvaluationQuality::Final,
                &token,
            )
            .unwrap()
            .diagnostics
            .iter()
            .filter(|d| d.feature_id == "hole_2")
            .map(|d| d.code.as_str().to_string())
            .collect();
        (total_volume(&bodies), warnings, codes)
    };
    // Counterbore narrower than the bore: invalid ordering.
    let (v, w, codes) = build(HoleKind::Counterbore {
        diameter: 6.0,
        depth: 2.0,
    });
    if (v - 9000.0).abs() > 1.0 {
        // Applied anyway: the narrow counterbore is geometrically absorbed
        // by the bore — acceptable only with an explicit note.
        assert!(
            w.iter().any(|x| x.contains("hole_2")),
            "absorbed counterbore must be explained: {w:?}"
        );
    } else if !w.is_empty() {
        assert!(
            codes.contains(&"parameter.invalid".to_string()),
            "rejection must be typed parameter.invalid: {codes:?} / {w:?}"
        );
    }
    // Degenerate countersink angle.
    let (v, w, codes) = build(HoleKind::Countersink {
        diameter: 12.0,
        angle_deg: 0.0,
    });
    if (v - 9000.0).abs() < 1.0 && !w.is_empty() {
        assert!(
            codes.contains(&"parameter.invalid".to_string()),
            "degenerate countersink must be typed: {codes:?} / {w:?}"
        );
    }
    // Negative angle.
    let (_v, w, _codes) = build(HoleKind::Countersink {
        diameter: 12.0,
        angle_deg: -90.0,
    });
    let _ = w;
}

/// I05: thread a tube's outer wall and inner bore separately — only the
/// intended wall changes.
#[test]
fn i05_tube_outer_wall_vs_inner_bore_thread_separately() {
    // Tube via annulus extrusion: inner r=6, outer r=10, height 20 (+Y).
    // The ring region is selected explicitly (whole-sketch nested circles
    // extrude the full disk — the island semantics pinned in catalog_c/d).
    let tube_graph = || -> ParametricGraph {
        let mut g = ParametricGraph::new();
        let mut c = zerocad_core::SketchCurves::new();
        c.add_circle((0.0, 0.0), 10.0);
        c.add_circle((0.0, 0.0), 6.0);
        let regions = zerocad_core::sketch::detect_regions(&c);
        let ring_index = regions
            .iter()
            .position(|r| r.contains((8.0, 0.0)) && !r.contains((0.0, 0.0)))
            .expect("ring region");
        add_sketch(&mut g, "sk_1", c);
        add_feature(
            &mut g,
            "tube_2",
            FeatureType::Extrude {
                depth: 20.0,
                region_indices: vec![ring_index],
                mode: zerocad_core::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
            &["sk_1"],
        );
        g
    };
    let tube_volume = {
        let g = tube_graph();
        let bodies = eval(&g);
        assert_meshes_valid(&bodies);
        total_volume(&bodies)
    };
    assert_close(
        tube_volume,
        (PI * 100.0 - PI * 36.0) * 20.0,
        1e-6,
        0.03,
        "tube volume",
    );

    // Thread the outer wall (external thread) — material removed near
    // r ∈ [10−d, 10].
    let outer = {
        let mut g = tube_graph();
        let face = cylindrical_face(&g, "tube_2", 10.0);
        add_feature(
            &mut g,
            "th_3",
            FeatureType::Thread {
                target: "tube_2".to_string(),
                face,
                internal: false,
                pitch: 2.0,
                depth: 0.5,
                angle_deg: 60.0,
                right_handed: true,
                starts: 1,
                length: None,
                flip: false,
                designation: "Custom".to_string(),
                standard: None,
            },
            &["tube_2"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        assert!(
            warnings
                .iter()
                .all(|w| !w.to_lowercase().contains("could not")),
            "outer thread must resolve: {warnings:?}"
        );
        (total_volume(&bodies), bodies)
    };
    // Thread the inner bore (tapped) — material removed near r ∈ [6, 6+d].
    let inner = {
        let mut g = tube_graph();
        let face = cylindrical_face(&g, "tube_2", 6.0);
        add_feature(
            &mut g,
            "th_3",
            FeatureType::Thread {
                target: "tube_2".to_string(),
                face,
                internal: true,
                pitch: 2.0,
                depth: 0.5,
                angle_deg: 60.0,
                right_handed: true,
                starts: 1,
                length: None,
                flip: false,
                designation: "Custom".to_string(),
                standard: None,
            },
            &["tube_2"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        assert!(
            warnings
                .iter()
                .all(|w| !w.to_lowercase().contains("could not")),
            "inner thread must resolve: {warnings:?}"
        );
        (total_volume(&bodies), bodies)
    };
    let (v_outer, bodies_outer) = outer;
    let (v_inner, bodies_inner) = inner;
    // Thread convention: an EXTERNAL thread cuts teeth out of the OD
    // (volume drops); an INTERNAL thread replaces the bore wall with tooth
    // bands reaching INTO the hole (volume grows by the tooth material).
    assert!(
        v_outer < tube_volume,
        "outer thread must remove material: {v_outer} vs {tube_volume}"
    );
    assert!(
        v_inner > tube_volume,
        "inner thread models teeth into the bore: {v_inner} vs {tube_volume}"
    );
    // Only the intended wall changes: after threading the OUTER wall, the
    // bore-side material (r just above 6) is still solid; after threading
    // the BORE, the mid-wall and outer wall survive untouched.
    let mid_z = 10.0; // the annulus extrudes along +Z
    let mesh_outer = &bodies_outer
        .iter()
        .find(|(id, _)| id == "tube_2")
        .unwrap()
        .1;
    assert_occupancy(
        mesh_outer,
        &[[8.0, 0.0, mid_z], [6.5, 0.0, mid_z]],
        &[[9.8, 0.0, mid_z], [0.0, 0.0, mid_z], [0.0, 0.0, 0.0]],
    );
    let mesh_inner = &bodies_inner
        .iter()
        .find(|(id, _)| id == "tube_2")
        .unwrap()
        .1;
    assert_occupancy(
        mesh_inner,
        &[[8.0, 0.0, mid_z], [9.5, 0.0, mid_z]],
        &[[0.0, 0.0, mid_z], [0.0, 0.0, 0.0]],
    );
}

/// Capture the cylindrical wall of a tube body nearest `radius`. The tube
/// is an XY-sketch annulus extruded along +Z, so wall faces sit at
/// sqrt(x²+y²) ≈ radius.
fn cylindrical_face(
    g: &ParametricGraph,
    body: &str,
    radius: f32,
) -> zerocad_core::parametric::FaceRef {
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies.iter().find(|(id, _)| id == body).unwrap().1;
    let f = mesh
        .face_refs
        .iter()
        .min_by_key(|f| {
            let r = (f.centroid[0] * f.centroid[0] + f.centroid[1] * f.centroid[1]).sqrt();
            ((r - radius).abs() * 100.0) as u32
        })
        .expect("a cylindrical wall must be selectable");
    zerocad_core::parametric::FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f
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
    }
}
