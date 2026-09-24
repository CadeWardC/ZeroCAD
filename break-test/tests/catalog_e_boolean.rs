//! Catalog E — boolean material and contact (adversarial plan §6.E).
//!
//! Face/edge/vertex tangency sweeps live in `tangency_sweeps.rs` and
//! `join_cut_stress.rs`; tool consumption/rollback in `body_booleans_stress.rs`;
//! air-cut-in-void and metamorphic volume algebra in
//! `research_boolean_metamorphic.rs`. This file adds the exact analytic
//! answers with occupancy checks (E01), overlapping-bore removal measured
//! against the union of the bores (E03), full-consumption and island removal
//! (E05), and direct removed-material checks for sliver cuts (E08).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{ExtrudeMode, FeatureType, ParametricGraph};

fn transformed_box(
    g: &mut ParametricGraph,
    id: &str,
    w: f32,
    h: f32,
    d: f32,
    translation: [f32; 3],
) {
    add_box(g, id, w, h, d);
    add_feature(
        g,
        &format!("tf_{id}"),
        FeatureType::BodyTransform {
            source: id.to_string(),
            translation,
            copy: false,
        },
        &[id],
    );
}

/// E01: union, intersect, subtract of overlapping boxes with exact analytic
/// answers, per-body checks, and occupancy probes away from boundaries.
#[test]
fn e01_exact_analytic_box_booleans() {
    // Two 10³ boxes offset by 5 along X: overlap 5×10×10 = 500 →
    // union 1500, intersection 500, difference 500.
    let build = |kind: &str| -> ParametricGraph {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
        transformed_box(&mut g, "box_2", 10.0, 10.0, 10.0, [5.0, 0.0, 0.0]);
        let feature = match kind {
            "union" => FeatureType::BodyJoin {
                sources: vec!["box_1".into(), "tf_box_2".into()],
            },
            "cut" => FeatureType::BodyCut {
                target: "box_1".into(),
                tool: "tf_box_2".into(),
                keep_tool: false,
            },
            "intersect" => FeatureType::BodyIntersect {
                target: "box_1".into(),
                tool: "tf_box_2".into(),
                keep_tool: false,
            },
            _ => unreachable!(),
        };
        add_feature(&mut g, "op_3", feature, &["box_1", "tf_box_2"]);
        g
    };

    // Union: one body, exact volume, material at both far ends.
    let g = build("union");
    let bodies = eval(&g);
    assert_eq!(bodies.len(), 1, "union must produce one body");
    assert_meshes_valid(&bodies);
    assert_close(total_volume(&bodies), 1500.0, 1e-6, 0.01, "union volume");
    assert_occupancy(
        &bodies[0].1,
        &[[2.5, 5.0, 5.0], [12.5, 5.0, 5.0]],
        &[[-1.0, 5.0, 5.0], [16.0, 5.0, 5.0]],
    );

    // Difference: box_1 keeps only x ∈ [0,5].
    let g = build("cut");
    let bodies = eval(&g);
    assert_close(total_volume(&bodies), 500.0, 1e-6, 0.01, "cut volume");
    assert_occupancy(
        &bodies[0].1,
        &[[2.5, 5.0, 5.0]],
        &[[7.5, 5.0, 5.0], [12.5, 5.0, 5.0]],
    );

    // Intersection: the common x ∈ [5,10] slab.
    let g = build("intersect");
    let bodies = eval(&g);
    assert_close(total_volume(&bodies), 500.0, 1e-6, 0.01, "intersect volume");
    assert_occupancy(
        &bodies[0].1,
        &[[7.5, 5.0, 5.0]],
        &[[2.5, 5.0, 5.0], [12.5, 5.0, 5.0]],
    );
}

/// E03: two overlapping bores — the removed material equals the union of the
/// bore regions and removal is monotone as the bores approach.
#[test]
fn e03_overlapping_bores_remove_the_union_once() {
    let pi = std::f64::consts::PI;
    let r = 4.0_f64;
    let plate = 30.0 * 30.0 * 10.0_f64;
    // Area of the union of two radius-r circles whose centers are s apart.
    let union_area = |s: f64| -> f64 {
        if s >= 2.0 * r {
            2.0 * pi * r * r
        } else if s <= 0.0 {
            pi * r * r
        } else {
            let a = 2.0 * r * r * (s / (2.0 * r)).acos();
            let l = 0.5 * s * (4.0 * r * r - s * s).sqrt();
            2.0 * pi * r * r - (a - l)
        }
    };
    let removed_at = |s: f64, select_all_regions: bool| -> f64 {
        let mut curves = zerocad_core::SketchCurves::new();
        curves.add_circle(((15.0 - s / 2.0) as f32, 15.0), r as f32);
        curves.add_circle(((15.0 + s / 2.0) as f32, 15.0), r as f32);
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), curves.clone());
        // The whole-sketch default applies overlap-cluster tool-lens
        // semantics (one base circle) — the union contract needs the explicit
        // "all regions" selection, exactly what a user clicking each bore
        // produces in the UI.
        let region_indices = if select_all_regions {
            (0..zerocad_core::sketch::detect_regions(&curves).len()).collect::<Vec<usize>>()
        } else {
            vec![]
        };
        add_feature(
            &mut g,
            "cut_3",
            FeatureType::Extrude {
                depth: -12.0,
                region_indices,
                mode: ExtrudeMode::Cut,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
            &["sk_2"],
        );
        let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        plate - total_volume(&bodies)
    };
    let mut previous_removed = f64::INFINITY;
    for s in [12.0_f64, 9.0, 8.001, 0.0] {
        let removed = removed_at(s, true);
        let expected = union_area(s) * 10.0;
        assert_close(
            removed,
            expected,
            1e-9,
            0.02,
            &format!("removed material at separation {s}"),
        );
        assert!(
            removed <= previous_removed + 1e-3,
            "removal must be monotone as bores approach: s={s} removed {removed} > {previous_removed}"
        );
        previous_removed = removed;
    }
    // Concentric bores remove the circle once, not twice.
    assert_close(
        removed_at(0.0, true),
        pi * r * r * 10.0,
        1e-9,
        0.02,
        "concentric bore removal",
    );
}

/// Whole-sketch overlapping circles remove the complete union once.
#[test]
fn e03_whole_sketch_overlapping_bores_remove_the_union() {
    let pi = std::f64::consts::PI;
    let r = 4.0_f64;
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((11.5, 15.0), r as f32);
    c.add_circle((18.5, 15.0), r as f32); // overlap of 2 mm
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    add_extrude(&mut g, "cut_3", "sk_2", -12.0, ExtrudeMode::Cut);
    let (bodies, _) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let removed = 9000.0 - total_volume(&bodies);
    let separation = 7.0_f64;
    let lens = 2.0 * r * r * (separation / (2.0 * r)).acos()
        - 0.5 * separation * (4.0 * r * r - separation * separation).sqrt();
    let expected = (2.0 * pi * r * r - lens) * 10.0;
    assert!(
        close(removed, expected, 1e-9, 0.02),
        "whole-sketch overlap cut union expects {expected}, got {removed}"
    );
}

/// E03 tracked defect (found 2026-09-13): for two PARTIALLY OVERLAPPING
/// bores (0 < separation < 2r), only the base circle is removed — even when
/// every detected region is explicitly selected. The overlap-cluster
/// tool-lens rules in `boolean_region_plan`/`prepare_extrude_regions` drop
/// the second bore's exclusive region, so ≈502 mm³ is removed where the
/// union of the bores requires up to ≈1005 mm³ — a silent half cut with no
/// diagnostic. Disjoint and exactly-concentric pairs behave correctly.
/// Recorded in FINDINGS.md E-overlap.
#[test]
fn e03_overlapping_bores_drop_the_second_bore_known_break() {
    let pi = std::f64::consts::PI;
    let r = 4.0_f64;
    for s in [7.9_f64, 6.0, 4.0, 2.0] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
        let mut c = zerocad_core::SketchCurves::new();
        c.add_circle(((15.0 - s / 2.0) as f32, 15.0), r as f32);
        c.add_circle(((15.0 + s / 2.0) as f32, 15.0), r as f32);
        add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c.clone());
        let regions = zerocad_core::sketch::detect_regions(&c);
        add_feature(
            &mut g,
            "cut_3",
            FeatureType::Extrude {
                depth: -12.0,
                region_indices: (0..regions.len()).collect(),
                mode: ExtrudeMode::Cut,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
            &["sk_2"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        let removed = 9000.0 - total_volume(&bodies);
        // Union of two circles s apart.
        let a = 2.0 * r * r * (s / (2.0 * r)).acos();
        let l = 0.5 * s * (4.0 * r * r - s * s).sqrt();
        let expected = (2.0 * pi * r * r - (a - l)) * 10.0;
        assert!(
            close(removed, expected, 1e-9, 0.02),
            "s={s}: removed {removed}, union expects {expected}, warnings {warnings:?}"
        );
    }
}

/// E03 tracked defect (found 2026-09-13): two bores at EXACT tangency
/// (separation == 2r) drop one bore of the pair — the sketch region planner
/// classifies the point-touching circles as an overlap cluster and discards
/// the inner lens, so only ~one bore is removed (≈502 mm³ instead of
/// ≈1005 mm³) with no diagnostic. Exactly the "silent missing cut" the plan
/// forbids; boundary s = 8.001/7.999 behave correctly. Recorded in
/// FINDINGS.md E-tangency.
#[test]
fn e03_exactly_tangent_bore_pair_rejects_nonmanifold_wall_atomically() {
    let r = 4.0_f64;
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 10.0);
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((11.0, 15.0), r as f32);
    c.add_circle((19.0, 15.0), r as f32); // exactly tangent at (15, 15)
    add_sketch_cs(&mut g, "sk_2", xy_plane_at(10.0), c);
    add_extrude(&mut g, "cut_3", "sk_2", -12.0, ExtrudeMode::Cut);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let plate = 30.0 * 30.0 * 10.0_f64;
    let removed = plate - total_volume(&bodies);
    // Exactly tangent through-bores leave a zero-thickness line of material.
    // That is not a manifold solid: reject the entire feature, never one bore.
    assert_close(
        removed,
        0.0,
        1e-9,
        0.0,
        "nonmanifold cut preserves the entire plate",
    );
    assert!(
        warnings.iter().any(|w| w.contains("cut_3")),
        "rejected cut must be attributed: {warnings:?}"
    );
}

/// E05: fully consuming the target, and removing one island from a
/// multicomponent body — both follow an explicit contract, never partials.
#[test]
fn e05_full_consumption_and_island_removal() {
    // Full consumption: an identical-size cutter removes everything.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_box(&mut g, "box_2", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "cut_3",
        FeatureType::BodyCut {
            target: "box_1".into(),
            tool: "box_2".into(),
            keep_tool: false,
        },
        &["box_1", "box_2"],
    );
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    // The explicit empty-result contract: either the body is gone entirely,
    // or an empty result is reported with a warning — never a phantom solid.
    if !bodies.is_empty() {
        let vols: Vec<(String, f64)> = bodies
            .iter()
            .map(|(id, m)| {
                (
                    id.clone(),
                    m.mass_properties().map(|p| p.volume).unwrap_or(0.0),
                )
            })
            .collect();
        if total_volume(&bodies) >= 1.0 {
            // The cut was refused: that is a correct safe rejection only with
            // an attributed warning; a silent no-op with both bodies intact
            // would be a wrong success.
            assert!(
                warnings.iter().any(|w| w.contains("cut_3")),
                "full-consumption cut refused without attribution: {warnings:?} / {vols:?}"
            );
        } else {
            assert!(
                !warnings.is_empty(),
                "an emptied body needs an explanatory warning: {warnings:?} / {vols:?}"
            );
        }
    }

    // Island removal: cut a full-height slot through a plate, splitting it
    // into a single body with two islands, then remove one island entirely.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 60.0, 20.0, 5.0); // x ∈ [0,60]
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(5.0),
        rect_sketch((25.0, -5.0), (35.0, 25.0)),
    );
    add_extrude(&mut g, "slot", "sk_2", -10.0, ExtrudeMode::Cut);
    {
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_close(
            total_volume(&bodies),
            5000.0,
            1e-6,
            0.01,
            "slotted plate keeps both islands",
        );
    }

    // Remove the right island (x ∈ [35,60]) with a full-covering cutter.
    add_box(&mut g, "cutter", 40.0, 40.0, 40.0);
    add_feature(
        &mut g,
        "tf_c",
        FeatureType::BodyTransform {
            source: "cutter".into(),
            translation: [30.0, -15.0, -15.0],
            copy: false,
        },
        &["cutter"],
    );
    let cut_id = "cut_x";
    g.add_feature(zerocad_core::FeatureNode {
        id: cut_id.to_string(),
        name: cut_id.to_string(),
        feature: FeatureType::BodyCut {
            target: "box_1".into(),
            tool: "tf_c".into(),
            keep_tool: false,
        },
    });
    g.add_dependency("tf_c", cut_id);
    g.add_dependency("slot", cut_id);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // Island removal through BodyCut is currently an attributed safe
    // rejection: the solver refuses to subtract from the multicomponent
    // body and leaves BOTH inputs unchanged. Per the plan's taxonomy that is
    // a correct safe rejection (capability gap, FINDINGS.md E-island) — the
    // corruption to guard against is a partial change.
    if close(v, 2500.0, 1e-6, 0.03) {
        let mesh = &bodies[0].1;
        assert_occupancy(
            mesh,
            &[[10.0, 10.0, 2.5]],
            &[[45.0, 10.0, 2.5], [30.0, 10.0, 2.5]],
        );
    } else {
        assert!(
            close(v, 5000.0 + 64000.0, 1e-6, 0.001),
            "refused island cut must leave both bodies unchanged, got {v}, bodies {:?}",
            body_ids(&bodies)
        );
        assert!(
            warnings.iter().any(|w| w.contains(cut_id)),
            "the refusal must be attributed to the cut: {warnings:?}"
        );
    }
}

/// E08: a very thin sliver cut from a large body — the removed material is
/// checked directly so whole-body tolerances cannot hide a no-op.
#[test]
fn e08_sliver_cut_removed_material_is_measured_directly() {
    let _whole = 100.0_f64 * 100.0 * 100.0;
    let sliver_at = |t: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 100.0, 100.0, 100.0);
        add_box(&mut g, "tool", 100.0, 100.0, t);
        add_feature(
            &mut g,
            "tf_3",
            FeatureType::BodyTransform {
                source: "tool".into(),
                translation: [0.0, 0.0, 50.0],
                copy: false,
            },
            &["tool"],
        );
        add_feature(
            &mut g,
            "cut_4",
            FeatureType::BodyCut {
                target: "box_1".into(),
                tool: "tf_3".into(),
                keep_tool: false,
            },
            &["box_1", "tf_3"],
        );
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        (100.0 * 100.0 * 100.0 - total_volume(&bodies), warnings)
    };
    // A 1e-3 slab: 10 mm³ must actually leave the body.
    let (removed, warnings) = sliver_at(1e-3);
    let expected = 100.0 * 100.0 * 1e-3_f64;
    assert!(
        close(removed, expected, 1e-9, 0.05) || removed < 1e-9,
        "sliver removal {removed} vs expected {expected}: silent partial removal is corruption"
    );
    assert!(
        close(removed, expected, 1e-9, 0.05),
        "the 1e-3 sliver must be removed (found {removed} of {expected}); a no-op needs an explicit warning: {warnings:?}"
    );
    // A 1e-5 slab (0.1 mm³) is below the f32-tessellation volume noise floor
    // (~±0.1 mm³ on a 10⁶ mm³ body), so the check is: the body stays within
    // noise of the original — never a gross change in either direction.
    let (removed, _) = sliver_at(1e-5);
    assert!(
        removed.abs() < 1.0,
        "sub-tolerance sliver changed the body by {removed} — far beyond tessellation noise"
    );
}
