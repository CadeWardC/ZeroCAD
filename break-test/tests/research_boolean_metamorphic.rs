//! Research-derived boolean + metamorphic torture (research pass 2026-09-11).
//!
//! Every scenario here comes from a documented real-kernel failure that is
//! NOT covered by the earlier suites (coplanar flush booleans, simple
//! external tangency, crossing ribs, inscribed-circle cuts are all already
//! pinned elsewhere). Sources per test.
//!
//! Universal invariants under test (see `common`): no panics, finite meshes,
//! deterministic volumes — plus metamorphic laws (volume algebra, frame
//! invariance, scale invariance) that shipped kernels are known to violate
//! silently.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::FaceRef;
use zerocad_core::{
    AxisBase, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Vec3,
};

fn top_face_of(x: f32, y: f32, z: f32) -> FaceRef {
    FaceRef {
        centroid: [x, y, z],
        normal: [0.0, 0.0, 1.0],
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

/// A box body spanning [0, w] × [0, h] × [0, d], built from a sketch so it can
/// sit at an arbitrary plane offset (primitives are pinned to the origin).
fn box_body(g: &mut ParametricGraph, id: &str, min: [f32; 3], max: [f32; 3]) {
    add_sketch_cs(
        g,
        &format!("{id}_sk"),
        xy_plane_at(min[2]),
        rect_sketch((min[0], min[1]), (max[0], max[1])),
    );
    add_extrude(
        g,
        id,
        &format!("{id}_sk"),
        max[2] - min[2],
        ExtrudeMode::NewBody,
    );
}

// --- Metamorphic volume algebra -------------------------------------------
// Hoffmann, "Robustness in Geometric Computations" (2001):
// https://ftp.cs.wisc.edu/pub/users/prem/for-prem/surveys/Hoffmann-robust4.pdf —
// tolerance-driven kernels violate vol(A∪B) = vol(A)+vol(B)−vol(A∩B) near
// degeneracies; OpenSCAD #6116 is the canonical shipped bug (nested
// differences flip parity and phantom volume appears).

#[test]
fn metamorphic_volume_laws_union_intersect_cut() {
    // A = [0,20]²×[0,10], B = [5,25]²×[5,15]. Overlap = 15·15·5 = 1125.
    let (va, vb, overlap) = (4000.0, 4000.0, 1125.0_f64);

    let build = |op: &str| -> ParametricGraph {
        let mut g = ParametricGraph::new();
        box_body(&mut g, "body_a", [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
        box_body(&mut g, "body_b", [5.0, 5.0, 5.0], [25.0, 25.0, 15.0]);
        let feature = match op {
            "join" => FeatureType::BodyJoin {
                sources: vec!["body_a".into(), "body_b".into()],
            },
            "cut" => FeatureType::BodyCut {
                target: "body_a".into(),
                tool: "body_b".into(),
                keep_tool: false,
            },
            "isect" => FeatureType::BodyIntersect {
                target: "body_a".into(),
                tool: "body_b".into(),
                keep_tool: false,
            },
            _ => unreachable!(),
        };
        add_feature(&mut g, "bool_op", feature, &["body_a", "body_b"]);
        g
    };

    let vj = assert_part_sane(&build("join"));
    let vc = assert_part_sane(&build("cut"));
    let vi = assert_part_sane(&build("isect"));

    assert!(
        (vj - (va + vb - overlap)).abs() < 0.5,
        "union volume law broken: {vj} vs {}",
        va + vb - overlap
    );
    assert!(
        (vi - overlap).abs() < 0.5,
        "intersect volume wrong: {vi} vs {overlap}"
    );
    assert!(
        (vc - (va - overlap)).abs() < 0.5,
        "cut volume law broken: {vc} vs {}",
        va - overlap
    );
}

#[test]
fn metamorphic_union_intersect_cut_curved_tool() {
    // Same laws with a cylindrical tool — curved-face intersection curves are
    // where parity classification actually breaks (OCCT ticket 0029034: Cut,
    // Common and Fuse ALL wrong for a sphere/cylinder pair,
    // https://occt3d.com/dev/tickets/0029034/).
    // Cylinder r=5, h=20 along +Y at origin; box A = [0,20]²×[0,10].
    let mut g = ParametricGraph::new();
    add_box(&mut g, "body_a", 20.0, 20.0, 10.0);
    let va = assert_part_sane(&g);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "body_b", 5.0, 20.0);
    let vb = assert_part_sane(&g);

    let build = |op: &str| -> ParametricGraph {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "body_a", 20.0, 20.0, 10.0);
        add_cylinder(&mut g, "body_b", 5.0, 20.0);
        let feature = match op {
            "join" => FeatureType::BodyJoin {
                sources: vec!["body_a".into(), "body_b".into()],
            },
            "cut" => FeatureType::BodyCut {
                target: "body_a".into(),
                tool: "body_b".into(),
                keep_tool: false,
            },
            "isect" => FeatureType::BodyIntersect {
                target: "body_a".into(),
                tool: "body_b".into(),
                keep_tool: false,
            },
            _ => unreachable!(),
        };
        add_feature(&mut g, "bool_op", feature, &["body_a", "body_b"]);
        g
    };

    let vj = assert_part_sane(&build("join"));
    let vc = assert_part_sane(&build("cut"));
    let vi = assert_part_sane(&build("isect"));

    // cut ∪ intersect partition A exactly (held to 1e-2 mm³ when probed).
    assert!(
        (vc + vi - va).abs() < 0.5,
        "cut+intersect must partition the target: {vc} + {vi} vs {va}"
    );
    // The union law holds only to curved-wall chord re-segmentation noise:
    // the fused cylinder wall is re-tessellated, so allow ~0.5% of vB.
    let tol = vb * 0.005;
    assert!(
        (vj - (va + vb - vi)).abs() < tol,
        "union law broken with curved tool: {vj} vs {} (±{tol})",
        va + vb - vi
    );
}

#[test]
fn union_order_independence_three_bodies() {
    // Nearness/tolerance stitching is order-dependent in tolerance-based
    // kernels (Hoffmann 2001, "nearness is not transitive"; CGAL exactness
    // notes, https://www.cgal.org/exact.html). All six orders must agree.
    let mut volumes = Vec::new();
    for order in [["a", "b", "c"], ["c", "b", "a"], ["b", "a", "c"]] {
        let mut g = ParametricGraph::new();
        box_body(&mut g, "a", [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
        box_body(&mut g, "b", [10.0, 0.0, 0.0], [30.0, 20.0, 10.0]);
        box_body(&mut g, "c", [20.0, 0.0, 0.0], [40.0, 20.0, 10.0]);
        add_feature(
            &mut g,
            "join_op",
            FeatureType::BodyJoin {
                sources: order.iter().map(|s| s.to_string()).collect(),
            },
            &["a", "b", "c"],
        );
        volumes.push(assert_part_sane(&g));
    }
    let expected = 20.0 * 20.0 * 10.0 + 20.0 * 20.0 * 10.0 + 20.0 * 20.0 * 10.0
        - 10.0 * 20.0 * 10.0
        - 10.0 * 20.0 * 10.0; // 12000 − 2·2000 = 8000
    for v in &volumes {
        assert!(
            (v - expected).abs() < 0.5,
            "union order changes the volume: {volumes:?} vs {expected}"
        );
    }
}

// --- Cutting nothing / cutting everything ----------------------------------
// OCCT distinguishes empty-because-disjoint from failed; kernels that don't
// return "null shapes" that destroy user models
// (https://forum.freecad.org/viewtopic.php?style=10&t=98543).

#[test]
fn cut_tool_fully_inside_existing_void_is_noop() {
    // OpenSCAD #1591 family inverted: a tool entirely inside an existing
    // cavity must remove NOTHING (the OpenSCAD "coincident ends" bug removed
    // the wrong thing; here the tool is in air). Shelled box: 20×20×10 with
    // 3 mm walls, open top. Cavity interior = [3,17]²×[3,10].
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "shell_2",
        FeatureType::Shell {
            target: "box_1".into(),
            thickness: 3.0,
            thickness_expr: None,
            open_faces: vec![top_face_of(10.0, 10.0, 10.0)],
        },
        &["box_1"],
    );
    let bodies = eval(&g);
    let shell_volume = total_volume(&bodies);
    assert!(shell_volume > 0.0, "shell must produce a hollow body first");

    add_sketch_cs(
        &mut g,
        "sk_3",
        xy_plane_at(4.0),
        rect_sketch((7.0, 7.0), (13.0, 13.0)),
    );
    add_extrude_full(
        &mut g,
        "cut_4",
        "sk_3",
        3.0,
        ExtrudeMode::Cut,
        Some("shell_2".into()),
        0.0,
    );
    let after = assert_part_sane(&g);
    assert!(
        (after - shell_volume).abs() < shell_volume * 1e-6 + 1e-6,
        "cutting air inside a void changed the body: {shell_volume} -> {after}"
    );
}

#[test]
fn body_cut_consuming_entire_target_stays_graceful() {
    // "Cut consumes the target" must be an explicit outcome, never a wrong
    // solid or a destroyed document. CHARACTERIZATION (probed 2026-09-11):
    // containment is misreported as a failed subtraction (FINDINGS E15c) and
    // the rollback preserves BOTH bodies — tool included despite
    // `keep_tool: false` — so the total is 1000 + 8000.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    box_body(&mut g, "big_tool", [-5.0, -5.0, -5.0], [15.0, 15.0, 15.0]);
    add_feature(
        &mut g,
        "cut_op",
        FeatureType::BodyCut {
            target: "box_1".into(),
            tool: "big_tool".into(),
            keep_tool: false,
        },
        &["box_1", "big_tool"],
    );
    let v = assert_part_sane(&g);
    assert!(
        (v - 9000.0).abs() < 2.0 || v.abs() < 5.0 || (v - 1000.0).abs() < 2.0,
        "consumed-target cut must be rollback-both, empty, or target-only; got {v}"
    );
}

#[test]
fn near_miss_cut_with_1e5_gap_removes_nothing() {
    // OCCT fuzzy-boolean Example 4.3: a near-miss edge 1e-5 above a face
    // produces a "pin-like protrusion" in shipped kernels
    // (https://occt3d.com/dev/content/fuzzy-boolean_operations/). The cut
    // must cleanly remove nothing (or warn), never grow a needle.
    for center_x in [45.0_f32, 45.000_01] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
        add_sketch(&mut g, "sk_2", circle_sketch((center_x, 20.0), 5.0));
        add_extrude(&mut g, "cut_3", "sk_2", 10.0, ExtrudeMode::Cut);
        let v = assert_part_sane(&g);
        assert!(
            (v - 16_000.0).abs() < 1.0,
            "cut {center_x} from the face must remove nothing: {v}"
        );
    }
}

#[test]
fn through_cut_fails_only_in_the_5e5_breakthrough_band() {
    // OCCT fuzzy Example 4.1: a cut ending a hair short of the far face is
    // the near-coincident-back-face regime. Probed 2026-09-11: depths
    // 9.9–9.9999 (of 10) cut correctly; exactly 9.99995 FAILS with a warning
    // and removes NOTHING, while the full through-cut works. The valid band
    // must stay correct, and the failure band must stay LOUD (warn + full
    // preservation), never silent, never garbage.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch(&mut g, "sk_2", circle_sketch((20.0, 20.0), 5.0));
    add_extrude(&mut g, "cut_3", "sk_2", 9.9999, ExtrudeMode::Cut);
    let removed = 16_000.0 - assert_part_sane(&g);
    assert!(
        (700.0..850.0).contains(&removed),
        "partial-depth cut removed {removed} mm³, expected ≈785"
    );

    // The 5e-5 band: whatever the outcome, it must be one of the two clean
    // ones (cut like its neighbors, or a full warned rollback).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch(&mut g, "sk_2", circle_sketch((20.0, 20.0), 5.0));
    add_extrude(&mut g, "cut_3", "sk_2", 9.999_95, ExtrudeMode::Cut);
    let outcome = g.evaluate_bodies_with_warnings(&HashSet::new());
    let (bodies, warnings) = outcome.expect("boundary cut must not hard-fail the document");
    assert_meshes_finite(&bodies);
    let removed = 16_000.0 - total_volume(&bodies);
    if removed < 700.0 {
        assert!(
            warnings
                .iter()
                .any(|w| w.contains("cut_3") || w.to_lowercase().contains("cut")),
            "boundary-band cut must warn when it fails to apply: {warnings:?}"
        );
    }
}

// --- Coincident-face fuses and near-coincident slivers ----------------------
// OCCT fuzzy Example 4.2: fusing near-identical boxes leaves an "extremely
// narrow face"; FreeCAD t=1067: cylinder cut by a sphere centered ON its wall
// made the wall VANISH (https://forum.freecad.org/viewtopic.php?style=10&t=1067).

#[test]
fn flush_stack_fuse_with_1e6_size_mismatch_conserves_volume() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    box_body(
        &mut g,
        "box_2",
        [0.0, 0.0, 10.0],
        [20.000_01, 20.000_01, 20.0],
    );
    add_feature(
        &mut g,
        "join_op",
        FeatureType::BodyJoin {
            sources: vec!["box_1".into(), "box_2".into()],
        },
        &["box_1", "box_2"],
    );
    let v = assert_part_sane(&g);
    let expected = 4000.0 + 20.000_01 * 20.000_01 * 10.0;
    assert!(
        (v - expected).abs() < 0.1,
        "flush stack fuse must conserve volume: {v} vs {expected}"
    );
}

#[test]
fn vertex_touching_chain_union_stays_graceful() {
    // Two cubes sharing exactly one vertex form a figure-eight neighborhood —
    // not a 2-manifold (OpenSCAD docs/issues #2034/#4039). A diagonal chain of
    // three must not panic, poison the mesh, or silently lose material.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    box_body(&mut g, "box_2", [10.0, 10.0, 10.0], [20.0, 20.0, 20.0]);
    box_body(&mut g, "box_3", [20.0, 20.0, 20.0], [30.0, 30.0, 30.0]);
    add_feature(
        &mut g,
        "join_op",
        FeatureType::BodyJoin {
            sources: vec!["box_1".into(), "box_2".into(), "box_3".into()],
        },
        &["box_1", "box_2", "box_3"],
    );
    let v = assert_part_sane(&g);
    assert!(
        (2990.0..=3001.0).contains(&v),
        "vertex-contact chain must conserve all material: {v}"
    );
}

#[test]
fn deep_union_chain_with_micro_jitter_stays_exact() {
    // Hoffmann's "precision proliferation": each boolean's tolerance noise
    // feeds the next operation; OCCT users measured tolerance inflation to
    // 0.75–1.9 mm after chains (tolerance-issues thread,
    // https://occt3d.com/dev/content/tolerance-issues/index.html). 20 joins
    // with 1e-7 offsets must stay analytic to 1e-4 relative. NOTE: a Join
    // extrude keeps the TARGET body's id (ownership renames are a
    // BodyCut/BodyIntersect behavior), so every boss must target `base`.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "base", 40.0, 40.0, 5.0);
    for i in 0..20u32 {
        let jitter = i as f32 * 1e-7;
        let x = 4.0 + (i % 5) as f32 * 8.0 + jitter;
        let y = 4.0 + (i / 5) as f32 * 8.0 + jitter;
        let id = format!("boss_{i}");
        add_sketch_cs(
            &mut g,
            &format!("{id}_sk"),
            xy_plane_at(5.0),
            rect_sketch((x - 2.0, y - 2.0), (x + 2.0, y + 2.0)),
        );
        add_extrude_full(
            &mut g,
            &id,
            &format!("{id}_sk"),
            2.0,
            ExtrudeMode::Join,
            Some("base".to_string()),
            0.0,
        );
    }
    let v = assert_part_sane(&g);
    // Each 4x4 footprint at depth 2 contributes 32 mm³.
    let expected = 40.0 * 40.0 * 5.0 + 20.0 * 32.0;
    assert!(
        (v - expected).abs() < expected * 1e-4,
        "jittered union chain drifted: {v} vs {expected}"
    );
}

// --- Metamorphic frame and scale invariance ---------------------------------
// Kernels are not scale-invariant (absolute tolerances); OCCT measured a
// vertex 1.9 mm off at build scale that DID NOT improve when the model was
// scaled 1000× (tolerance-issues thread). Rotated frames produce irrational
// coordinates — Hoffmann shows rounding them makes valid inputs
// self-intersect.

#[test]
fn scale_metamorphic_volume_cubes() {
    for k in [0.25_f32, 1.0, 4.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 20.0 * k, 20.0 * k, 10.0 * k);
        add_sketch_cs(
            &mut g,
            "sk_2",
            xy_plane_at(10.0 * k),
            rect_sketch((2.0 * k, 2.0 * k), (6.0 * k, 6.0 * k)),
        );
        add_extrude(&mut g, "boss_3", "sk_2", 2.0 * k, ExtrudeMode::Join);
        let v = assert_part_sane(&g);
        // Boss footprint is (6k−2k)² · 2k = 32k³ on top of the 4000k³ plate.
        let expected = 4032.0_f64 * (k as f64).powi(3);
        assert!(
            (v - expected).abs() < expected * 1e-3,
            "volume does not cube with scale {k}: {v} vs {expected}"
        );
    }
}

#[test]
fn rotated_frame_45deg_volume_invariance() {
    let build = |rotated: bool| -> ParametricGraph {
        let mut g = ParametricGraph::new();
        let cs = if rotated {
            let (s, c) = (
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
            );
            CoordinateSystem::new(Vec3::ZERO, Vec3::new(c, s, 0.0), Vec3::new(-s, c, 0.0))
        } else {
            CoordinateSystem::XY
        };
        add_sketch_cs(&mut g, "sk_1", cs, rect_sketch((0.0, 0.0), (20.0, 20.0)));
        add_extrude(&mut g, "plate_2", "sk_1", 10.0, ExtrudeMode::NewBody);
        add_sketch_cs(&mut g, "sk_3", cs, circle_sketch((10.0, 10.0), 4.0));
        add_extrude_full(
            &mut g,
            "hole_4",
            "sk_3",
            10.0,
            ExtrudeMode::Cut,
            Some("plate_2".into()),
            0.0,
        );
        g
    };
    let v0 = assert_part_sane(&build(false));
    let v45 = assert_part_sane(&build(true));
    assert!(
        (v0 - v45).abs() < v0 * 1e-3,
        "45° frame rotation changed the volume: {v0} vs {v45}"
    );
}

#[test]
fn boolean_result_invariant_under_far_translation() {
    // Blender T67744 documents the epsilon-wall: solvers mixing doubles with
    // epsilons break on "nearby but not exactly coinciding geometry" far from
    // the origin. The same cut at 1e5 mm must give the same volume.
    let build = |far: bool| -> ParametricGraph {
        let (dx, dy, dz) = if far {
            (1.0e5, 2.0e5, 3.0e5)
        } else {
            (0.0, 0.0, 0.0)
        };
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_1",
            CoordinateSystem::new(Vec3::new(dx, dy, dz), Vec3::X, Vec3::Y),
            rect_sketch((0.0, 0.0), (40.0, 40.0)),
        );
        add_extrude(&mut g, "plate_2", "sk_1", 10.0, ExtrudeMode::NewBody);
        add_sketch_cs(
            &mut g,
            "sk_3",
            CoordinateSystem::new(Vec3::new(dx, dy, dz + 10.0), Vec3::X, Vec3::Y),
            circle_sketch((20.0, 20.0), 5.0),
        );
        add_extrude_full(
            &mut g,
            "hole_4",
            "sk_3",
            10.0,
            ExtrudeMode::Cut,
            Some("plate_2".into()),
            0.0,
        );
        g
    };
    let v0 = assert_part_sane(&build(false));
    let vf = assert_part_sane(&build(true));
    assert!(
        (v0 - vf).abs() < v0 * 1e-4,
        "cut volume changed 1e5 mm from the origin: {v0} vs {vf}"
    );
}

// --- Curved tangency beyond the already-pinned cases ------------------------
// FreeCAD t=21969: a torus tangent to a planar face broke OCCT in a DIFFERENT
// way in each of two OCC versions — assert validity, not exact topology.

#[test]
fn torus_tube_tangent_to_planar_face_fuse_stays_valid() {
    let mut g = ParametricGraph::new();
    // Box centered on the Z axis: [−20,20]²×[0,10], top face z = 10.
    add_sketch(&mut g, "sk_1", rect_sketch((-20.0, -20.0), (20.0, 20.0)));
    add_extrude(&mut g, "plate_2", "sk_1", 10.0, ExtrudeMode::NewBody);
    // Torus: circle at radial 10, height 13 (tube r=3), revolved about Z —
    // the tube's lowest circle touches z = 10 exactly.
    add_sketch_cs(
        &mut g,
        "sk_3",
        CoordinateSystem::XZ,
        circle_sketch((10.0, 13.0), 3.0),
    );
    add_feature(
        &mut g,
        "torus_4",
        FeatureType::Revolve {
            axis: AxisBase::Z,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        &["sk_3"],
    );
    add_feature(
        &mut g,
        "join_5",
        FeatureType::BodyJoin {
            sources: vec!["plate_2".into(), "torus_4".into()],
        },
        &["plate_2", "torus_4"],
    );
    let v = assert_part_sane(&g);
    let expected = 16_000.0 + 2.0 * std::f64::consts::PI.powi(2) * 10.0 * 9.0;
    assert!(
        (v - expected).abs() < expected * 0.02,
        "tangent torus fuse lost or gained material: {v} vs {expected}"
    );
}

#[test]
fn revolve_axis_crossing_profile_stays_graceful() {
    // FreeCAD rejects revolves whose profile crosses the axis ("Revolve axis
    // intersects the sketch"); SolidWorks accepts them. Either behavior is
    // fine — a silent self-intersecting solid is not
    // (https://forum.freecad.org/viewtopic.php?t=3861).
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sk_1",
        CoordinateSystem::XZ,
        circle_sketch((2.0, 5.0), 3.0), // spans x ∈ [−1, 5]: crosses the axis
    );
    add_feature(
        &mut g,
        "spindle_2",
        FeatureType::Revolve {
            axis: AxisBase::Z,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        &["sk_1"],
    );
    assert_part_sane(&g);
}
