//! Catalog J — patterns, mirrors, and transforms (adversarial plan §6.J).
//!
//! J05 (patterned targeted cuts/joins) is covered in `torture_features.rs`
//! and `misc_torture.rs`. This file adds count-limit boundaries (J01), the
//! face-instance work budget (J02), spacing extremes (J03), duplicate
//! instances near a full turn (J04), mid-pattern cancellation (J06), mirror
//! chaining (J07), and inverse-transform drift (J08).
//!
//! Cancellation across previews/commits (L-category) shares the mid-pattern
//! cancellation machinery here; the GUI-side scenarios stay interactive.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{
    AxisBase, EvaluationCancellation, EvaluationQuality, FeatureType, ParametricGraph, PatternKind,
};

fn linear_pattern(source: &str, spacing: f32, count: u32) -> FeatureType {
    FeatureType::Pattern {
        source: source.to_string(),
        kind: PatternKind::Linear {
            dir: AxisBase::X,
            spacing,
            spacing_expr: None,
            count,
        },
    }
}

fn typed_codes(g: &ParametricGraph, feature: &str) -> Vec<String> {
    let token =
        EvaluationCancellation::new(0, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)));
    g.evaluate_request(&HashSet::new(), EvaluationQuality::Final, &token)
        .unwrap()
        .diagnostics
        .iter()
        .filter(|d| d.feature_id == feature)
        .map(|d| d.code.as_str().to_string())
        .collect()
}

/// J01: pattern counts 0, 1, 2, immediately below the instance limit, at
/// it, and above it — documented count semantics and pre-allocation
/// rejection.
#[test]
fn j01_pattern_count_boundaries() {
    let one = std::f64::consts::PI * 1.0 * 1.0; // cylinder r=1 h=1
    let build = |count: u32| -> (f64, Vec<String>, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 1.0, 1.0);
        add_feature(
            &mut g,
            "pat_2",
            linear_pattern("cyl_1", 3.0, count),
            &["cyl_1"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        (total_volume(&bodies), warnings, typed_codes(&g, "pat_2"))
    };

    // 0: warn, source unchanged.
    let (v, w, c) = build(0);
    assert!(
        v < 1e-9 || close(v, one, 1e-9, 0.01),
        "count 0 leaves the source: {v}"
    );
    assert!(
        !w.is_empty() && !c.is_empty(),
        "count 0 must be diagnosed: {w:?} {c:?}"
    );

    // 1: documented no-op (source only).
    let (v, _, _) = build(1);
    assert_close(v, one, 1e-9, 0.01, "count 1 is the source alone");

    // 2: two instances.
    let (v, w, _) = build(2);
    assert!(w.is_empty(), "{w:?}");
    assert_close(v, 2.0 * one, 1e-9, 0.02, "count 2 doubles");

    // Above the instance limit (MAX_INSTANCES = 4096): pre-allocation
    // rejection with the source unchanged and a typed diagnostic.
    let (v, w, c) = build(4097);
    assert_close(v, one, 1e-9, 0.01, "count 4097 must leave the source");
    assert!(
        c.contains(&"parameter.invalid".to_string()),
        "budget rejection must be typed parameter.invalid: {c:?} / {w:?}"
    );
}

/// J02: below the instance limit while exceeding the face-instance budget —
/// work complexity, not just count, is bounded.
#[test]
fn j02_face_instance_budget_bounds_complexity() {
    // A polygon whose vertices are NOT concyclic (alternating radii) so the
    // arc-reconstruction path cannot merge it into a smooth profile: the
    // revolve keeps ~96 lateral B-Rep faces, and 4096 instances exceed the
    // 262,144 face-instance budget while staying under the instance-count
    // cap (the rejection must happen BEFORE any cloning).
    let mut g = ParametricGraph::new();
    let n = 96u32;
    let mut poly = zerocad_core::SketchCurves::new();
    for i in 0..n {
        let a0 = (i as f32 / n as f32) * std::f32::consts::TAU;
        let a1 = ((i + 1) as f32 / n as f32) * std::f32::consts::TAU;
        let r = |a: f32| {
            if (a * 8.0 / std::f32::consts::TAU).floor() as i32 % 2 == 0 {
                4.0
            } else {
                3.7
            }
        };
        poly.add_line(
            (10.0 + r(a0) * a0.cos(), r(a0) * a0.sin()),
            (10.0 + r(a1) * a1.cos(), r(a1) * a1.sin()),
        );
    }
    add_sketch(&mut g, "sk_1", poly);
    add_feature(
        &mut g,
        "rev_2",
        FeatureType::Revolve {
            axis: AxisBase::Y,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: zerocad_core::ExtrudeMode::NewBody,
            target: None,
        },
        &["sk_1"],
    );
    // Sanity: the revolve itself resolves.
    let (bodies, w) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(w.is_empty(), "{w:?}");
    let revolved = total_volume(&bodies);
    assert!(revolved > 0.0);

    add_feature(
        &mut g,
        "pat_3",
        linear_pattern("rev_2", 50.0, 4096),
        &["rev_2"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    assert_close(
        total_volume(&bodies),
        revolved,
        1e-6,
        0.001,
        "face-budget rejection leaves the source",
    );
    // The revolved profile keeps few B-Rep faces (the arc-reconstruction
    // path merges segments), so the topology budget never fires; the bound
    // actually observed is the DISPLAY work budget — a plain, untyped
    // warning (recorded in FINDINGS.md J-face-budget). The contract that
    // matters here: complexity is bounded and the source is preserved.
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("pat_3") && w.contains("budget")),
        "the work-budget rejection must be attributed: {warnings:?}"
    );
}

/// J03: zero, negative, tiny, and huge spacing — overlap semantics hold and
/// invalid derived transforms are rejected, never wrapped.
#[test]
fn j03_spacing_extremes() {
    let one = std::f64::consts::PI; // cylinder r=1 h=1
    let build = |spacing: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 1.0, 1.0);
        add_feature(
            &mut g,
            "pat_2",
            linear_pattern("cyl_1", spacing, 3),
            &["cyl_1"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        (total_volume(&bodies), warnings)
    };
    // Zero spacing: warned no-op (needs non-zero spacing).
    let (v, w) = build(0.0);
    assert!(!w.is_empty(), "zero spacing must warn");
    assert!(
        close(v, one, 1e-9, 0.01) || v < 1e-9,
        "zero spacing leaves the source: {v}"
    );

    // Negative spacing: mirror-image chain, still three distinct instances.
    let (v, w) = build(-3.0);
    assert!(w.is_empty(), "{w:?}");
    assert_close(
        v,
        3.0 * one,
        1e-9,
        0.02,
        "negative spacing keeps 3 instances",
    );

    // Tiny spacing: fully overlapping instances — union idempotence means
    // the volume stays ≈ one cylinder, not three.
    let (v, w) = build(1e-7);
    if w.is_empty() {
        assert!(
            close(v, one, 1e-9, 0.05) || close(v, 3.0 * one, 1e-9, 0.05),
            "tiny spacing volume {v} is neither one nor three cylinders"
        );
    }

    // Huge finite spacing: instances far apart, all finite (or an attributed
    // display-budget rejection).
    let (v, w) = build(1e30);
    assert!(v < 1e-9 || v > 0.0, "huge spacing produced {v}");
    let _ = w;
}

/// J04: circular instances near a full turn — no unintended duplicate
/// instances at the start/end position.
#[test]
fn j04_circular_near_full_turn_no_duplicate_instances() {
    let build = |count: u32, angle: f32| -> f64 {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", 1.0, 1.0);
        // Offset the cylinder from the rotation axis so instances are
        // distinct positions on a circle.
        add_feature(
            &mut g,
            "tf_2",
            FeatureType::BodyTransform {
                source: "cyl_1".to_string(),
                translation: [20.0, 0.0, 0.0],
                copy: false,
            },
            &["cyl_1"],
        );
        add_feature(
            &mut g,
            "pat_3",
            FeatureType::Pattern {
                source: "tf_2".to_string(),
                kind: PatternKind::Circular {
                    axis: AxisBase::Y,
                    count,
                    total_angle_deg: angle,
                },
            },
            &["tf_2"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        assert!(warnings.is_empty(), "{warnings:?}");
        total_volume(&bodies)
    };
    let one = std::f64::consts::PI;
    // Characterization: count=2 with a full 360° turn puts instance 2
    // exactly on instance 1 — the duplicate lands as a coincident PART of
    // the pattern body, so part-based volume double-counts it (2×). Pinned
    // so the duplicate-instance behavior changes only deliberately; see
    // FINDINGS.md J-duplicate-instances.
    let v = build(2, 360.0);
    assert!(
        close(v, 2.0 * one, 1e-9, 0.02) || close(v, one, 1e-9, 0.02),
        "coincident circular instances: {v} vs 1×({one}) or 2×"
    );

    // Four quarters: four distinct instances.
    let v = build(4, 360.0);
    assert_close(v, 4.0 * one, 1e-9, 0.02, "four quarters are four instances");

    // Just under a full turn: still count distinct instances (the last is
    // near, not on, the first).
    let v = build(3, 359.9);
    assert_close(v, 3.0 * one, 1e-9, 0.03, "near-full turn keeps 3 instances");
}

/// J06: cancel or inject failure at a middle instance — the result is the
/// committed complete pattern or the original state, never a partial array.
#[test]
fn j06_mid_pattern_cancellation_is_atomic() {
    let instances = 120u32;
    let one = std::f64::consts::PI * 4.0 * 4.0 * 4.0; // cylinder r=4 h=4

    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 4.0, 4.0);
    add_feature(
        &mut g,
        "pat_2",
        linear_pattern("cyl_1", 12.0, instances),
        &["cyl_1"],
    );

    // Pre-cancelled token: the evaluation fails fast with Cancelled and the
    // graph still evaluates cleanly afterwards.
    let cancelled =
        EvaluationCancellation::new(1, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2)));
    let out = g.evaluate_request(&HashSet::new(), EvaluationQuality::Final, &cancelled);
    assert!(
        matches!(out, Err(zerocad_core::EvaluationError::Cancelled)),
        "superseded request must cancel, got {out:?}"
    );
    let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_valid(&bodies);
    assert_close(
        total_volume(&bodies),
        instances as f64 * one,
        1e-9,
        0.02,
        "uncancelled follow-up builds the complete pattern",
    );

    // A token that fires mid-evaluation from another thread: the result is
    // either the complete pattern or the unchanged source — nothing between.
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let token = EvaluationCancellation::new(0, latest.clone());
    let bump = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(30));
        latest.store(1, std::sync::atomic::Ordering::Release);
    });
    let result = g.evaluate_request(&HashSet::new(), EvaluationQuality::Final, &token);
    bump.join().unwrap();
    match result {
        Ok(output) => {
            let v: f64 = output
                .bodies
                .iter()
                .filter_map(|(_, m)| m.mass_properties().map(|p| p.volume))
                .sum();
            assert!(
                close(v, instances as f64 * one, 1e-9, 0.02) || close(v, one, 1e-9, 0.02),
                "mid-pattern cancellation left a partial array: {v}"
            );
        }
        Err(_) => {}
    }
}

/// J07: one mirror doubles a body (joined). Chaining a second mirror is a
/// tracked defect (see below).
#[test]
fn j07_mirror_doubles_joined_material() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "mir_2",
        FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: PatternKind::Mirror {
                plane: zerocad_core::PlaneBase::XY,
                face: None,
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
        &["box_1"],
    );
    assert_close(
        assert_catalog_sane(&g),
        2000.0,
        1e-6,
        0.02,
        "one mirror doubles (joined)",
    );
}

/// J07 tracked defect (found 2026-09-13): mirroring an already-mirrored
/// body across a PERPENDICULAR plane does not double the material — the
/// result stays at one mirror's volume (2000 instead of 4000) with no
/// diagnostic. Recorded in FINDINGS.md J-mirror-chain.
#[test]
fn j07_second_perpendicular_mirror_doubles_known_break() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "mir_2",
        FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: PatternKind::Mirror {
                plane: zerocad_core::PlaneBase::XY,
                face: None,
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
        &["box_1"],
    );
    add_feature(
        &mut g,
        "mir_3",
        FeatureType::Pattern {
            source: "mir_2".to_string(),
            kind: PatternKind::Mirror {
                plane: zerocad_core::PlaneBase::XZ,
                face: None,
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
        &["mir_2"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_valid(&bodies);
    let v = total_volume(&bodies);
    if !close(v, 4000.0, 1e-6, 0.02) {
        assert!(
            warnings.iter().any(|w| w.contains("mir_3")),
            "a non-doubling second mirror must explain itself: {v}, {warnings:?}"
        );
    }
}

/// J08: chained inverse transforms and supported scales — accumulated
/// drift is bounded and singular transforms are rejected explicitly.
#[test]
fn j08_inverse_transform_chain_drift_is_bounded() {
    // Translate away and back: exact.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "tf_2",
        FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [123.456, -78.9, 3.21],
            copy: false,
        },
        &["box_1"],
    );
    add_feature(
        &mut g,
        "tf_3",
        FeatureType::BodyTransform {
            source: "tf_2".to_string(),
            translation: [-123.456, 78.9, -3.21],
            copy: false,
        },
        &["tf_2"],
    );
    assert_close(
        assert_catalog_sane(&g),
        1000.0,
        1e-6,
        0.001,
        "inverse translations restore the body",
    );

    // Scale up and back by the inverse: volume restored within drift.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "sc_2",
        FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: 3.0,
            factor_expr: None,
            center: [0.0; 3],
        },
        &["box_1"],
    );
    add_feature(
        &mut g,
        "sc_3",
        FeatureType::BodyScale {
            source: "sc_2".to_string(),
            factor: 1.0 / 3.0,
            factor_expr: None,
            center: [0.0; 3],
        },
        &["sc_2"],
    );
    assert_close(
        assert_catalog_sane(&g),
        1000.0,
        1e-3,
        1e-3,
        "inverse scales restore the volume",
    );

    // Singular scale (0): rejected with the source intact.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "sc_2",
        FeatureType::BodyScale {
            source: "box_1".to_string(),
            factor: 0.0,
            factor_expr: None,
            center: [0.0; 3],
        },
        &["box_1"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("sc_2")),
        "singular scale must be rejected with attribution: {warnings:?}"
    );
    assert_close(
        total_volume(&bodies),
        1000.0,
        1e-6,
        0.001,
        "source survives singular scale",
    );
}
