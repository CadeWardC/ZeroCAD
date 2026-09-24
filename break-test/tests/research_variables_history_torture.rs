//! Research-derived variable / expression / history / document-format torture
//! (research pass 2026-09-11).
//!
//! Sources: SolidWorks circular-reference diagnostics
//! (https://help.solidworks.com/2026/english/SolidWorks/Sldworks/c_circular_reference_warnings.htm),
//! the FreeCAD solver's QR pivot-threshold redundancy diagnosis
//! (https://wiki.freecad.org/Sketcher_Dialog), large-coordinate precision
//! (https://www.autodesk.com/blogs/autocad/working-large-coordinates-in-autocad/),
//! bincode/serde length-prefix and duplicate-key pitfalls
//! (https://docs.rs/bincode/2.0.1/bincode/spec/index.html), and the diamond-
//! dependency failure mode from the FreeCAD topological-naming-problem
//! literature (https://wiki.freecad.org/Topological_naming_problem).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::document::Document;
use zerocad_core::units::Unit;
use zerocad_core::zcad_format::{
    read_document_from_slice, write_document_to_vec, HydrationBundle, LoadOptions, SaveOptions,
};
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Variable,
};

fn add_variable_set(g: &mut ParametricGraph, id: &str, variables: Vec<Variable>) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::VariableSet { variables },
    });
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

fn variable(name: &str, value: f64, expression: Option<String>) -> Variable {
    Variable {
        name: name.to_string(),
        value,
        unit: Unit::Millimeter,
        expression,
    }
}

/// A 10×10 pad with its depth driven by `expr` (base units are mm).
fn pad_with_depth_expr(expr: &str) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    g.add_feature(FeatureNode {
        id: "pad_2".to_string(),
        name: "pad_2".to_string(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: Some(expr.to_string()),
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_1", "pad_2");
    g
}

fn warnings_of(g: &ParametricGraph) -> Vec<String> {
    g.evaluate_bodies_with_warnings(&HashSet::new())
        .map(|(_, w)| w)
        .unwrap_or_default()
}

// --- Expression cycles and non-finite values --------------------------------
// SolidWorks ships dedicated circular-reference and 0/0 diagnostics; the
// danger is an evaluator that hangs, stack-overflows, or lets NaN reach the
// kernel (where it becomes the A2 arena overflow).

#[test]
fn circular_variable_references_terminate_cleanly() {
    let mut g = pad_with_depth_expr("a");
    add_variable_set(
        &mut g,
        "vars_0",
        vec![
            variable("a", 10.0, Some("b + 1".to_string())),
            variable("b", 10.0, Some("a + 1".to_string())),
        ],
    );
    // Must terminate (no hang), never panic; a hard error or an
    // unresolved/warning outcome are both acceptable.
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn self_referential_variable_terminates_cleanly() {
    let mut g = pad_with_depth_expr("x");
    add_variable_set(
        &mut g,
        "vars_0",
        vec![variable("x", 10.0, Some("x + 1".to_string()))],
    );
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn division_by_zero_expression_is_structurally_rejected() {
    // h = 10 → 10/(h-10) = +inf. The expression result must be rejected at
    // the variable layer: an infinite depth reaching the kernel is the
    // A2 known break (arena overflow).
    let mut g = pad_with_depth_expr("10 / (h - 10)");
    add_variable_set(&mut g, "vars_0", vec![variable("h", 10.0, None)]);
    let outcome = g.evaluate_bodies_with_warnings(&HashSet::new());
    match outcome {
        Err(_) => {} // structured rejection: fine
        Ok((bodies, warnings)) => {
            assert_meshes_finite(&bodies);
            assert!(
                !warnings.is_empty(),
                "div-by-zero depth must warn, not silently evaluate"
            );
        }
    }
}

#[test]
fn deep_variable_chain_200_evaluates_exactly() {
    // Recursive/stack-based resolution dies around here; SolidWorks users hit
    // O(n²) rebuilds with long equation chains
    // (https://www.eng-tips.com/threads/problems-with-equations.100517/).
    let mut vars = vec![variable("v001", 1.0, None)];
    for i in 2..=200 {
        vars.push(variable(
            &format!("v{i:03}"),
            1.0,
            Some(format!("v{:03} + 1", i - 1)),
        ));
    }
    let mut g = pad_with_depth_expr("v200");
    add_variable_set(&mut g, "vars_0", vars);
    let v = assert_part_sane(&g);
    let expected = 10.0 * 10.0 * 200.0;
    assert!(
        (v - expected).abs() < 1e-6,
        "chain of 200 resolved to depth volume {v}, expected {expected}"
    );
}

#[test]
fn dangling_expression_reference_is_graceful() {
    // The rename/deletion dangling-reference state (SolidWorks "What's
    // Wrong"): missing identifier must resolve to an unresolved feature or a
    // warning — never a panic or a silent fallback to a stale number.
    let g = pad_with_depth_expr("width_that_no_longer_exists");
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let warnings = warnings_of(&g);
    assert!(
        !warnings.is_empty(),
        "dangling expression reference must surface a warning"
    );
}

// --- Large-coordinate precision ----------------------------------------------
// At 1e6 mm the f64 ULP is ~1.16e-10 mm; fixed absolute tolerances sit in the
// noise (Autodesk large-coordinates guidance; FreeCAD t=71183 sketches break
// far from origin). The far-from-origin build must equal the local one.

#[test]
fn sketch_far_from_origin_1e6_matches_local_build() {
    let build = |dx: f64, dy: f64| -> ParametricGraph {
        let (dx, dy) = (dx as f32, dy as f32);
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_1",
            CoordinateSystem::new(
                zerocad_core::Vec3::new(dx, dy, 0.0),
                zerocad_core::Vec3::X,
                zerocad_core::Vec3::Y,
            ),
            rect_sketch((0.0, 0.0), (20.0, 20.0)),
        );
        add_extrude(&mut g, "pad_2", "sk_1", 10.0, ExtrudeMode::NewBody);
        add_sketch_cs(
            &mut g,
            "sk_3",
            CoordinateSystem::new(
                zerocad_core::Vec3::new(dx, dy, 10.0),
                zerocad_core::Vec3::X,
                zerocad_core::Vec3::Y,
            ),
            rect_sketch((5.0, 5.0), (15.0, 15.0)),
        );
        add_extrude_full(
            &mut g,
            "boss_4",
            "sk_3",
            5.0,
            ExtrudeMode::Join,
            Some("pad_2".into()),
            0.0,
        );
        g
    };
    let local = assert_part_sane(&build(0.0, 0.0));
    let far = assert_part_sane(&build(1.0e6, -1.0e6));
    assert!(
        (local - far).abs() < local * 1e-6,
        "far-from-origin build drifted: {local} vs {far}"
    );
}

// --- History: independent consumers of one body ------------------------------
// Diamond dependency: two consumers of the same intermediate must be diagnosed
// independently and one branch's failure must not corrupt the other
// (FreeCAD TNP wiki; SolidWorks parent/child reorder rules).

#[test]
fn two_consumers_of_one_body_fail_and_succeed_independently() {
    // BodyCut with a DISJOINT tool (documented graceful rejection) and
    // BodyIntersect with an overlapping tool both consume box_1. The failed
    // cut must leave the target untouched for the intersect to still apply.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    // Disjoint tool: far away in +Y.
    add_sketch_cs(
        &mut g,
        "sk_cut",
        xy_plane_at(-100.0),
        rect_sketch((0.0, 100.0), (10.0, 110.0)),
    );
    add_extrude(&mut g, "tool_far", "sk_cut", 10.0, ExtrudeMode::NewBody);
    add_feature(
        &mut g,
        "cut_op",
        FeatureType::BodyCut {
            target: "box_1".into(),
            tool: "tool_far".into(),
            keep_tool: false,
        },
        &["box_1", "tool_far"],
    );
    // Overlapping tool: half-space slab through the middle of box_1.
    add_sketch_cs(
        &mut g,
        "sk_isect",
        xy_plane_at(0.0),
        rect_sketch((-50.0, -50.0), (10.0, 50.0)),
    );
    add_extrude(&mut g, "tool_isect", "sk_isect", 10.0, ExtrudeMode::NewBody);
    add_feature(
        &mut g,
        "isect_op",
        FeatureType::BodyIntersect {
            target: "box_1".into(),
            tool: "tool_isect".into(),
            keep_tool: false,
        },
        &["box_1", "tool_isect"],
    );
    let v = assert_part_sane(&g);
    // CHARACTERIZATION (probed 2026-09-11): the failed cut leaves BOTH of its
    // bodies unchanged (the disjoint `tool_far` survives too), and the
    // successful intersect renames the result to the feature id (FINDINGS
    // E17). Total = 2000 (kept half) + 1000 (surviving tool) = 3000, and the
    // branches clearly did not corrupt each other.
    assert!(
        (v - 3000.0).abs() < 1.0,
        "diamond branches interacted: got {v}, expected 3000"
    );
}

// --- Document format robustness ----------------------------------------------
// bincode-style binary containers fail on corrupted length prefixes and lose
// id boundaries at varint tier lengths (251 / 65536)
// (https://docs.rs/bincode/2.0.1/bincode/spec/index.html). Round-trip must be
// bit-exact and corruption must surface as errors, never panics.

fn sample_graph() -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_sketch(&mut g, "sk_2", circle_sketch((10.0, 10.0), 3.0));
    add_extrude_full(
        &mut g,
        "hole_3",
        "sk_2",
        10.0,
        ExtrudeMode::Cut,
        Some("box_1".into()),
        0.0,
    );
    g
}

fn roundtrip(g: &ParametricGraph) -> ParametricGraph {
    let document = Document::from_graph(g.clone(), Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write must succeed");
    let loaded =
        read_document_from_slice(&bytes, &LoadOptions::default()).expect("read must succeed");
    loaded.document.into_evaluator_graph()
}

#[test]
fn zcad_roundtrip_unicode_and_boundary_length_ids() {
    let mut g = sample_graph();
    // Unicode id, a 250-char id (below the first varint tier boundary), and a
    // 251-char id (exactly at it).
    for id in [
        "سمت_🎉-body",
        "x".repeat(250).as_str(),
        "y".repeat(251).as_str(),
    ] {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::Box {
                w: 5.0,
                h: 5.0,
                d: 5.0,
            },
        });
    }
    let before = assert_part_sane(&g);
    let loaded = roundtrip(&g);
    let after = assert_part_sane(&loaded);
    assert_eq!(
        before.to_bits(),
        after.to_bits(),
        "round-trip changed evaluation: {before} vs {after}"
    );
}

#[test]
fn zcad_truncation_and_bitflip_never_panic() {
    let sample = sample_graph();
    let document = Document::from_graph(sample, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write must succeed");

    // Deterministic LCG so failures are reproducible.
    let mut state: u64 = 0x9E3779B97F4A7C15;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };

    // Truncations: 64 evenly spaced cut points.
    for i in 0..64 {
        let cut = bytes.len() * i / 64;
        let result = read_document_from_slice(&bytes[..cut], &LoadOptions::default());
        if let Ok(loaded) = result {
            // A prefix that still parses must still evaluate sanely.
            let _ = loaded
                .document
                .into_evaluator_graph()
                .evaluate_bodies_with_warnings(&HashSet::new());
        }
    }

    // Single-byte flips across the file.
    for i in 0..200u32 {
        let _ = i;
        let mut corrupted = bytes.clone();
        let pos = (next() as usize) % corrupted.len();
        corrupted[pos] ^= (next() % 255 + 1) as u8;
        let result = read_document_from_slice(&corrupted, &LoadOptions::default());
        if let Ok(loaded) = result {
            // Corrupt-but-parseable payload (e.g. a flipped float) must not
            // crash the evaluator — it lands in the same parameter guards as
            // the direct-input torture suites.
            let _ = loaded
                .document
                .into_evaluator_graph()
                .evaluate_bodies_with_warnings(&HashSet::new());
        }
    }
}

#[test]
fn zcad_empty_document_roundtrip() {
    let g = ParametricGraph::new();
    let document = Document::from_graph(g, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("empty write must succeed");
    let loaded =
        read_document_from_slice(&bytes, &LoadOptions::default()).expect("empty read must succeed");
    let bodies = eval(&loaded.document.into_evaluator_graph());
    assert!(
        bodies.is_empty(),
        "empty document produced bodies: {bodies:?}"
    );
}
