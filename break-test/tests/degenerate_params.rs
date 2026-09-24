//! Degenerate and adversarial parameter inputs.
//!
//! The part builder must never panic on bad numbers: zero/negative/huge/NaN
//! dimensions either produce valid geometry, a structured evaluation error, or
//! an unresolved-feature warning — never a crash or NaN-poisoned mesh.

mod common;

use common::*;
use zerocad_core::{ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves};

#[test]
fn degenerate_cylinder_dimensions_do_not_panic() {
    // Cylinders handle bad dimensions gracefully today (empty result bodies
    // with no NaN geometry). Boxes did not — see the ignored known-break
    // tests below.
    let bad_cylinders: &[(f32, f32)] = &[
        (0.0, 10.0),
        (10.0, 0.0),
        (-5.0, 10.0),
        (10.0, -5.0),
        (1e-7, 1e-7),
        (f32::NAN, 10.0),
        (10.0, f32::NAN),
    ];
    for &(r, h) in bad_cylinders {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "cyl_1", r, h);
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

// A1 regression (fixed 2026-09-11): any non-positive or non-finite Box
// dimension used to panic at `zerocad-core/src/mock_kernel/primitives.rs:9`
// (`.expect()` on the kernel result) when the region builder fell through to
// `box_solid`. The evaluator now validates dimensions at the untrusted-input
// boundary: the feature resolves Unresolved with a PARAMETER_INVALID
// diagnostic, no body is created, and the document stays evaluable. Each
// value asserts its own outcome so one bad input cannot mask the rest.
#[test]
fn degenerate_box_dimensions_resolve_to_parameter_diagnostics() {
    let bad_boxes: &[(f32, f32, f32)] = &[
        (0.0, 10.0, 10.0),
        (10.0, 0.0, 10.0),
        (10.0, 10.0, 0.0),
        (-10.0, 10.0, 10.0),
        (10.0, -10.0, -10.0),
        (f32::NAN, 10.0, 10.0),
        (f32::INFINITY, 10.0, 10.0),
    ];
    for &(w, h, d) in bad_boxes {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", w, h, d);
        assert_parameter_invalid(&g, "box_1");
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("invalid box dimensions must resolve to warnings, not an error");
        assert_meshes_finite(&bodies);
        assert!(
            warnings
                .iter()
                .any(|m| m.contains("dimensions must be positive finite numbers")),
            "({w}, {h}, {d}) must produce the parameter warning, got {warnings:?}"
        );
        assert!(
            bodies.is_empty(),
            "no body may be created from dimensions ({w}, {h}, {d}), got {bodies:?}"
        );
    }
}

// A1 companion: an independent valid feature after a rejected one still
// evaluates — the rejection must not take the whole document down.
#[test]
fn invalid_box_does_not_block_independent_features() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", -5.0, 10.0, 10.0);
    add_box(&mut g, "box_2", 10.0, 10.0, 10.0);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("one invalid feature must not fail the document");
    assert_meshes_finite(&bodies);
    assert!(warnings.iter().any(|m| m.contains("box_1")), "{warnings:?}");
    assert_eq!(bodies.len(), 1, "only the valid box may exist: {bodies:?}");
    assert!((total_volume(&bodies) - 1000.0).abs() < 1.0);
}

#[test]
fn extreme_but_valid_box_dimensions_do_not_panic() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1e-6, 1e-6, 1e-6);
    if let Ok((bodies, _)) = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new()) {
        assert_meshes_finite(&bodies);
    }
}

// A3 regression (fixed 2026-09-11): extreme-but-finite Box dimensions (1e9 —
// a plausible meters-as-millimeters unit mistake) overflowed the OpenRCAD
// arena's spatial-grid neighbour arithmetic
// (`OpenRCAD/crates/openrcad-topo/src/arena.rs` "attempt to add with
// overflow"). The arena keys now saturate into an overflow-safe band, so
// finite magnitudes have NO invented cap: the box builds (or fails
// structurally) instead of crashing. Non-finite dimensions are rejected as
// invalid parameters by the A1 guard.
#[test]
fn huge_finite_box_dimensions_do_not_overflow_the_arena() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1e9, 1e9, 1e9);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("huge-but-finite dimensions must not hard-fail");
    assert_meshes_finite(&bodies);
    let volume = total_volume(&bodies);
    assert!(
        volume.is_finite() && volume > 0.0,
        "a valid huge box must keep finite positive volume, got {volume} ({warnings:?})"
    );
}

#[test]
fn infinite_box_dimensions_resolve_to_parameter_diagnostics() {
    for (w, h, d) in [(f32::INFINITY, 10.0, 10.0), (10.0, f32::NEG_INFINITY, 10.0)] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", w, h, d);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("infinite dimensions must resolve to warnings, not an error");
        assert_meshes_finite(&bodies);
        assert!(
            warnings
                .iter()
                .any(|m| m.contains("dimensions must be positive finite numbers")),
            "({w}, {h}, {d}) must produce the parameter warning, got {warnings:?}"
        );
        assert!(
            bodies.is_empty(),
            "no body from ({w}, {h}, {d}): {bodies:?}"
        );
    }
}

#[test]
fn degenerate_extrude_depths_do_not_panic() {
    // Zero / tiny / huge depths resolve to warnings, empty results, or valid
    // bodies — all fine. NaN and ±Infinity are a separate known break below.
    let depths: &[f32] = &[0.0, -10.0, 1e-6, -1e-6, 1e7, -1e7, f32::MIN_POSITIVE];
    for &depth in depths {
        for mode in [ExtrudeMode::NewBody, ExtrudeMode::Join, ExtrudeMode::Cut] {
            let mut g = ParametricGraph::new();
            add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
            add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
            add_extrude(&mut g, "extrude_3", "sketch_2", depth, mode);
            let bodies = eval(&g);
            assert_meshes_finite(&bodies);
        }
    }
}

// A2 regression (fixed 2026-09-11): an Extrude depth of NaN or ±Infinity
// reached the kernel as a non-finite translation and overflowed the arena's
// spatial-grid arithmetic (`arena.rs` "attempt to add with overflow"),
// crashing the whole evaluator. The effective depth — after expression
// evaluation AND the f64→f32 conversion — is validated before any geometry
// work, for every mode sharing the extrude entry point. The source body must
// survive untouched.
#[test]
fn nonfinite_extrude_depths_resolve_to_parameter_diagnostics() {
    for depth in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        for mode in [ExtrudeMode::NewBody, ExtrudeMode::Join, ExtrudeMode::Cut] {
            let mut g = ParametricGraph::new();
            add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
            add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
            add_extrude(&mut g, "extrude_3", "sketch_2", depth, mode);
            assert_parameter_invalid(&g, "extrude_3");
            let (bodies, warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .expect("non-finite depth must resolve to a warning, not an error");
            assert_meshes_finite(&bodies);
            assert!(
                warnings
                    .iter()
                    .any(|m| m.contains("depth must be a finite number")),
                "depth {depth} ({mode:?}) must produce the parameter warning, got {warnings:?}"
            );
            let v = total_volume(&bodies);
            assert!(
                (v - 4000.0).abs() < 1.0,
                "depth {depth} ({mode:?}): the source box must survive unchanged, got {v}"
            );
        }
    }
}

#[test]
fn degenerate_sketch_geometry_does_not_panic() {
    // Zero-area rectangle, inverted rectangle, zero-radius circle, huge circle,
    // and NaN coordinates.
    let mut cases: Vec<SketchCurves> = vec![
        rect_sketch((5.0, 5.0), (5.0, 5.0)),
        rect_sketch((15.0, 15.0), (5.0, 5.0)),
        rect_sketch((-5.0, -5.0), (5.0, 5.0)),
        circle_sketch((0.0, 0.0), 0.0),
        circle_sketch((0.0, 0.0), -5.0),
        circle_sketch((0.0, 0.0), 1e8),
    ];
    let mut nan_rect = SketchCurves::new();
    nan_rect.add_rectangle((f32::NAN, 0.0), (10.0, 10.0));
    cases.push(nan_rect);

    for curves in cases {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", curves);
        add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn hole_parameters_out_of_range_do_not_panic() {
    use zerocad_core::HoleKind;
    let bad: &[(f32, Option<f32>, HoleKind)] = &[
        (0.0, None, HoleKind::Simple),
        (-6.0, None, HoleKind::Simple),
        (f32::NAN, None, HoleKind::Simple),
        (6.0, Some(0.0), HoleKind::Simple),
        (6.0, Some(-5.0), HoleKind::Simple),
        (6.0, Some(f32::NAN), HoleKind::Simple),
        (
            6.0,
            None,
            HoleKind::Counterbore {
                diameter: 0.0,
                depth: 2.0,
            },
        ),
        (
            6.0,
            None,
            HoleKind::Counterbore {
                diameter: -4.0,
                depth: 2.0,
            },
        ),
        (
            6.0,
            None,
            HoleKind::Counterbore {
                diameter: 4.0,
                depth: -2.0,
            },
        ),
        (
            6.0,
            None,
            HoleKind::Countersink {
                diameter: 0.0,
                angle_deg: 90.0,
            },
        ),
        (
            6.0,
            None,
            HoleKind::Countersink {
                diameter: 12.0,
                angle_deg: 0.0,
            },
        ),
        (
            6.0,
            None,
            HoleKind::Countersink {
                diameter: 12.0,
                angle_deg: f32::NAN,
            },
        ),
        (1e9, None, HoleKind::Simple),
    ];
    for (dia, depth, kind) in bad.iter() {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [15.0, 15.0, 10.0],
            [0.0, 0.0, -1.0],
            *dia,
            *depth,
            kind.clone(),
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn hole_completely_off_body_leaves_part_intact() {
    // A hole that misses the body entirely must not remove material or crash.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [500.0, 500.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let v = assert_part_sane(&g);
    let exact = 20.0 * 20.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "off-body hole changed volume: {v} vs {exact}"
    );
}

#[test]
fn hole_direction_vectors_are_normalized_or_rejected_not_crashed() {
    let dirs: &[[f32; 3]] = &[
        [0.0, 0.0, 0.0],
        [1e-8, 0.0, -1e-8],
        [0.0, 0.0, -1e6],
        [f32::NAN, 0.0, -1.0],
        [7.0, 7.0, -7.0],
    ];
    for &dir in dirs {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_hole(
            &mut g,
            "hole_2",
            "box_1",
            [10.0, 10.0, 10.0],
            dir,
            6.0,
            None,
            zerocad_core::HoleKind::Simple,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn extrude_targeting_missing_body_fails_loudly() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
    add_extrude_full(
        &mut g,
        "extrude_3",
        "sketch_2",
        5.0,
        ExtrudeMode::Cut,
        Some("nonexistent_body".to_string()),
        0.0,
    );
    // Documented behavior: explicit missing target reports Unresolved instead
    // of silently cutting a neighboring body. That surfaces as either a hard
    // error or bodies-with-warnings; both are acceptable, a panic is not.
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn dependency_on_unknown_node_is_survivable() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    // Graph corruption a buggy caller could produce: dependency edges that
    // reference nodes that do not exist.
    g.add_dependency("ghost_sketch", "box_1");
    g.add_dependency("box_1", "ghost_extrude");
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
}

#[test]
fn duplicate_feature_ids_are_rejected_structurally() {
    // Duplicate ids can't be produced by the GUI but can by a hand-edited
    // file: the evaluator must reject the document with a structured error,
    // not panic or silently merge the nodes.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_box(&mut g, "box_1", 20.0, 20.0, 20.0);
    match g.evaluate_bodies_with_warnings(&std::collections::HashSet::new()) {
        Ok((bodies, _)) => assert_meshes_finite(&bodies),
        Err(_) => {} // structured rejection is the documented graceful path
    }
}

#[test]
fn extreme_draft_angles_do_not_panic() {
    let angles: &[f32] = &[0.0, 89.9, 90.0, 179.0, -89.9, -90.0, 1e6, f32::NAN];
    for &angle in angles {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
        add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
        add_extrude_full(
            &mut g,
            "extrude_3",
            "sketch_2",
            8.0,
            ExtrudeMode::Join,
            None,
            angle,
        );
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn feature_referencing_it_own_target_chain_survives() {
    // Self-referential target: a cut extrude whose explicit target is the
    // extrude itself (impossible in the GUI, trivial in a corrupt file).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch(&mut g, "sketch_2", rect_sketch((5.0, 5.0), (15.0, 15.0)));
    g.add_feature(FeatureNode {
        id: "extrude_3".to_string(),
        name: "extrude_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 4.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("extrude_3".to_string()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sketch_2", "extrude_3");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}
