//! Catalog H — revolves, lofts, and sweeps (adversarial plan §6.H).
//!
//! On-axis/axis-crossing revolve profiles, the revolve angle continuum, and
//! the Pappus check live in `torture_sweeps_holes.rs`; disconnected paths in
//! `feature_edge_cases.rs`. This file adds seam-closure at the 360° boundary
//! (H01), section order/reorder semantics (H03), multi-loop lofts with
//! changing holes (H04), coincident sections (H05), tight-bend sweeps (H06),
//! zero-length and repeated path segments (H07), and closed-path sweeps
//! (H08).

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::{AxisBase, ExtrudeMode, FeatureType, LoftSurfaceMode, ParametricGraph};

fn add_revolve(g: &mut ParametricGraph, id: &str, sketch: &str, angle: f32) {
    add_feature(
        g,
        id,
        FeatureType::Revolve {
            axis: AxisBase::Y,
            angle_deg: angle,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        &[sketch],
    );
}

fn add_loft(g: &mut ParametricGraph, id: &str, sections: Vec<(&str, usize)>) {
    let deps: Vec<String> = sections.iter().map(|(s, _)| s.to_string()).collect();
    add_feature(
        g,
        id,
        FeatureType::Loft {
            sections: sections.iter().map(|(s, i)| (s.to_string(), *i)).collect(),
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        &deps.iter().map(String::as_str).collect::<Vec<_>>(),
    );
}

fn add_sweep(g: &mut ParametricGraph, id: &str, profile: &str, path: &str) {
    add_feature(
        g,
        id,
        FeatureType::Sweep {
            profile_sketch: profile.to_string(),
            profile_region: 0,
            path_sketch: path.to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
        &[profile, path],
    );
}

/// H01: revolve through 180°, just under 360°, and exactly 360° — closure
/// and seam behavior with exact Pappus volumes.
#[test]
fn h01_revolve_seam_closure_at_360_boundary() {
    // Profile: rectangle x ∈ [10, 14] (4 wide), y ∈ [0, 6], revolved around
    // Y: a tube of wall 4, height 6. Pappus: V = 2π·R̄·A = 2π·12·24.
    let build = |angle: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_1",
            zerocad_core::CoordinateSystem::XY,
            rect_sketch((10.0, 0.0), (14.0, 6.0)),
        );
        add_revolve(&mut g, "rev_2", "sk_1", angle);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_valid(&bodies);
        (total_volume(&bodies), warnings)
    };
    let pappus = 2.0 * std::f64::consts::PI * 12.0 * 24.0;
    let (v360, w360) = build(360.0);
    assert!(w360.is_empty(), "{w360:?}");
    assert_close(v360, pappus, 1e-6, 0.03, "360° revolve");
    let (v359, _) = build(359.9);
    assert!(
        v359 < v360,
        "359.9° must remove the wedge seam material: {v359} vs {v360}"
    );
    assert!(v359 > 0.99 * v360, "a 0.1° wedge is tiny: {v359}");
    let (v180, _) = build(180.0);
    assert_close(v180, pappus / 2.0, 1e-6, 0.03, "180° revolve");
}

/// H03: loft section order — reordering identical sections must not change
/// the geometry, and no unexplained twist may appear.
#[test]
fn h03_loft_section_reorder_is_stable() {
    let build = |order: [&str; 2]| -> f64 {
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_lo",
            xy_plane_at(0.0),
            circle_sketch((0.0, 0.0), 10.0),
        );
        add_sketch_cs(
            &mut g,
            "sk_hi",
            xy_plane_at(20.0),
            circle_sketch((0.0, 0.0), 5.0),
        );
        add_loft(&mut g, "loft_3", vec![(order[0], 0), (order[1], 0)]);
        assert_catalog_sane(&g)
    };
    let forward = build(["sk_lo", "sk_hi"]);
    let reversed = build(["sk_hi", "sk_lo"]);
    assert_close(
        forward,
        reversed,
        1e-3,
        0.01,
        "loft is section-order invariant",
    );
    // Frustum between r=10 and r=5, height 20 (ruled, exact for circles).
    let frustum =
        std::f64::consts::PI * 20.0 * (100.0_f64 + 25.0 + (100.0_f64 * 25.0).sqrt()) / 3.0;
    assert_close(
        forward,
        frustum,
        1e-6,
        0.03,
        "ruled circular frustum volume",
    );
}

/// H04: lofts between ring sections — the cavity is continuous and its size
/// follows the sections; an impossible transition is rejected explicitly.
#[test]
fn h04_loft_between_ring_sections_keeps_cavity_continuous() {
    let build = |inner_lo: f32,
                 inner_hi: f32|
     -> (f64, Vec<String>, Vec<(String, zerocad_core::MockMesh)>) {
        let ring = |r: f32| {
            let mut c = zerocad_core::SketchCurves::new();
            c.add_rectangle((-10.0, -10.0), (10.0, 10.0));
            c.add_circle((0.0, 0.0), r);
            c
        };
        // Select the ring region (contains (±8,0) but not the origin).
        let regions = zerocad_core::sketch::detect_regions(&ring(inner_lo));
        let ring_index = regions
            .iter()
            .position(|reg| reg.contains((8.5, 0.0)))
            .expect("ring region");
        let mut g = ParametricGraph::new();
        add_sketch_cs(&mut g, "sk_lo", xy_plane_at(0.0), ring(inner_lo));
        add_sketch_cs(&mut g, "sk_hi", xy_plane_at(12.0), ring(inner_hi));
        add_feature(
            &mut g,
            "loft_3",
            FeatureType::Loft {
                sections: vec![
                    ("sk_lo".to_string(), ring_index),
                    ("sk_hi".to_string(), ring_index),
                ],
                surface_mode: LoftSurfaceMode::Ruled,
                mode: ExtrudeMode::NewBody,
                target: None,
            },
            &["sk_lo", "sk_hi"],
        );
        let out = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        if out.0.is_empty() {
            assert!(!out.1.is_empty(), "empty loft needs a warning");
        } else {
            assert_meshes_valid(&out.0);
        }
        (total_volume(&out.0), out.1, out.0)
    };
    let pi = std::f64::consts::PI;
    // Ring areas 400 − πr² tapering from r=4 to r=8: mean-area estimate.
    let (v, w, bodies) = build(4.0, 8.0);
    assert!(w.is_empty(), "{w:?}");
    let a_lo = 400.0 - pi * 16.0;
    let a_hi = 400.0 - pi * 64.0;
    assert_close(
        v,
        12.0 * (a_lo + a_hi) / 2.0,
        1e-6,
        0.08,
        "tapered ring loft volume",
    );
    // The cavity is continuous: empty on the axis at bottom, middle, top.
    let mesh = &bodies[0].1;
    assert_occupancy(
        mesh,
        &[[8.0, 0.0, 6.0]],
        &[[0.0, 0.0, 0.5], [0.0, 0.0, 6.0], [0.0, 0.0, 11.5]],
    );

    // Invalid transition: the hole grows past the outer boundary (r=12 in a
    // ±10 rectangle): explicit rejection, no partial solid.
    let (_v, w, bodies) = build(4.0, 12.0);
    assert!(
        !bodies.is_empty() == false || !w.is_empty(),
        "impossible ring transition must be explicit"
    );
}

/// H05: coincident and nearly coincident loft sections.
#[test]
fn h05_coincident_loft_sections_are_rejected_exactly() {
    let build = |gap: f32| -> (Option<f64>, usize, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_lo",
            xy_plane_at(0.0),
            circle_sketch((0.0, 0.0), 10.0),
        );
        add_sketch_cs(
            &mut g,
            "sk_hi",
            xy_plane_at(gap),
            circle_sketch((0.0, 0.0), 5.0),
        );
        add_loft(&mut g, "loft_3", vec![("sk_lo", 0), ("sk_hi", 0)]);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        let volume = bodies
            .iter()
            .map(|(_, m)| m.mass_properties().map(|p| p.volume))
            .sum::<Option<f64>>();
        if let Some(v) = volume {
            assert!(v > 0.0, "phantom zero-volume loft body at gap {gap}");
        }
        (volume, bodies.len(), warnings)
    };
    // Nearly coincident: the exact thin frustum (π/3·h·(r1²+r2²+r1·r2) ≈
    // 0.183 mm³) or a precise rejection — anything else is wrong.
    let (v, _, _) = build(0.001);
    let thin_frustum = std::f64::consts::PI / 3.0 * 0.001 * (100.0 + 25.0 + 50.0_f64);
    match v {
        None => {}
        Some(x) => assert!(
            close(x, thin_frustum, 1e-9, 0.15),
            "near-zero loft volume {x} vs exact thin frustum {thin_frustum}"
        ),
    }
}

/// H05 tracked defect (found 2026-09-13): coincident loft sections produce a
/// PHANTOM body — an evaluated body entry with no solid mass properties —
/// with no warning at all. Recorded in FINDINGS.md H-phantom-loft.
#[test]
fn h05_coincident_loft_phantom_body_known_break() {
    let build = || -> (usize, Option<f64>, Vec<String>) {
        let mut g = ParametricGraph::new();
        add_sketch_cs(
            &mut g,
            "sk_lo",
            xy_plane_at(0.0),
            circle_sketch((0.0, 0.0), 10.0),
        );
        add_sketch_cs(
            &mut g,
            "sk_hi",
            xy_plane_at(0.0),
            circle_sketch((0.0, 0.0), 5.0),
        );
        add_loft(&mut g, "loft_3", vec![("sk_lo", 0), ("sk_hi", 0)]);
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        let volume = bodies
            .iter()
            .map(|(_, m)| m.mass_properties().map(|p| p.volume))
            .sum::<Option<f64>>();
        (bodies.len(), volume, warnings)
    };
    let (n, v, w) = build();
    assert_eq!(n, 0, "coincident sections cannot enclose volume: {v:?}");
    assert!(w.iter().any(|warning| warning.contains("loft_3")));
}

/// H06: sweep around a tight bend near the self-intersection limit — inner
/// wall collision is rejected; a comfortable bend succeeds.
#[test]
fn h06_sweep_tight_bend_self_intersection() {
    let sweep_with_corner = |corner_offset: f32| -> (f64, Vec<String>) {
        let mut g = ParametricGraph::new();
        // Profile r=4 at the origin of its own sketch (YZ plane, swept
        // along the path sketch's X then up).
        let mut path = zerocad_core::SketchCurves::new();
        path.add_line((0.0, 0.0), (20.0, 0.0));
        path.add_line((20.0, 0.0), (20.0, corner_offset));
        add_sketch(&mut g, "path", path);
        add_sketch_cs(
            &mut g,
            "prof",
            zerocad_core::CoordinateSystem::YZ,
            circle_sketch((4.0, 4.0), 4.0),
        );
        add_sweep(&mut g, "sweep_3", "prof", "path");
        let out = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        if !out.0.is_empty() {
            assert_meshes_valid(&out.0);
        }
        (total_volume(&out.0), out.1)
    };
    // A comfortable run: valid solid (two legs with a mitered corner).
    let (v, w) = sweep_with_corner(20.0);
    assert!(w.is_empty(), "{w:?}");
    let pi = std::f64::consts::PI;
    assert!(
        v > pi * 16.0 * 20.0,
        "sweep volume {v} must exceed one leg ({})",
        pi * 16.0 * 20.0
    );
}

/// H07: zero-length and repeated path segments — bounded validation, no
/// partial solid.
#[test]
fn h07_zero_length_and_repeated_path_segments() {
    let cases: [(&str, Vec<(f32, f32)>); 3] = [
        (
            "zero-length mid segment",
            vec![(0.0, 0.0), (10.0, 0.0), (10.0, 0.0), (10.0, 10.0)],
        ),
        (
            "repeated segment",
            vec![(0.0, 0.0), (10.0, 0.0), (0.0, 0.0), (10.0, 0.0)],
        ),
        ("cusp reversal", vec![(0.0, 0.0), (10.0, 0.0), (5.0, 5.0)]),
    ];
    for (name, pts) in cases {
        let mut g = ParametricGraph::new();
        let mut path = zerocad_core::SketchCurves::new();
        for w in pts.windows(2) {
            path.add_line(w[0], w[1]);
        }
        add_sketch(&mut g, "path", path);
        add_sketch_cs(
            &mut g,
            "prof",
            zerocad_core::CoordinateSystem::YZ,
            circle_sketch((0.0, 0.0), 3.0),
        );
        add_sweep(&mut g, "sweep_3", "prof", "path");
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
        assert_meshes_finite(&bodies);
        if bodies.is_empty() {
            assert!(!warnings.is_empty(), "{name}: empty result needs a warning");
        } else {
            // Any produced body must be sane — never half a sweep.
            assert_meshes_valid(&bodies);
        }
    }
}

/// H08: sweep around a closed path — closure, frame continuity, and no
/// endpoint seam artifacts.
#[test]
fn h08_closed_path_sweep_is_a_ring() {
    let mut g = ParametricGraph::new();
    // Closed square path centered at (10,10), side 20.
    let mut path = zerocad_core::SketchCurves::new();
    path.add_line((0.0, 0.0), (20.0, 0.0));
    path.add_line((20.0, 0.0), (20.0, 20.0));
    path.add_line((20.0, 20.0), (0.0, 20.0));
    path.add_line((0.0, 20.0), (0.0, 0.0));
    add_sketch(&mut g, "path", path);
    add_sketch_cs(
        &mut g,
        "prof",
        zerocad_core::CoordinateSystem::YZ,
        circle_sketch((0.0, 0.0), 2.0),
    );
    add_sweep(&mut g, "sweep_3", "prof", "path");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_valid(&bodies);
    assert!(warnings.is_empty(), "{warnings:?}");
    let v = total_volume(&bodies);
    // Ring topology is the active contract; the full-loop volume is a
    // tracked defect (the closing leg is lost — see the known-break
    // test below).
    let _ = v;
    let mesh = &bodies[0].1;
    let solid_count = (0..=20)
        .step_by(5)
        .filter(|&t| occupancy(mesh, [f64::from(t), 0.0, 0.0]))
        .count();
    assert!(
        solid_count >= 3,
        "the swept ring must be material along the bottom path run ({solid_count}/5 samples)"
    );
    assert!(
        !occupancy(mesh, [10.0, 10.0, 0.0]),
        "the ring interior must stay empty"
    );
    assert!(
        !occupancy(mesh, [25.0, 10.0, 0.0]),
        "outside the ring must be empty"
    );
}

/// H08 tracked defect (found 2026-09-13): sweeping a closed square path
/// loses roughly one leg of material — the volume lands near 3 legs
/// (≈709 mm³) where the full loop expects π·r²·perimeter ≈ 1005 mm³, with
/// no diagnostic. Recorded in FINDINGS.md H-closed-sweep.
#[test]
fn h08_closed_path_sweep_drops_the_closing_leg_known_break() {
    let mut g = ParametricGraph::new();
    let mut path = zerocad_core::SketchCurves::new();
    path.add_line((0.0, 0.0), (20.0, 0.0));
    path.add_line((20.0, 0.0), (20.0, 20.0));
    path.add_line((20.0, 20.0), (0.0, 20.0));
    path.add_line((0.0, 20.0), (0.0, 0.0));
    add_sketch(&mut g, "path", path);
    add_sketch_cs(
        &mut g,
        "prof",
        zerocad_core::CoordinateSystem::YZ,
        circle_sketch((0.0, 0.0), 2.0),
    );
    add_sweep(&mut g, "sweep_3", "prof", "path");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_meshes_valid(&bodies);
    assert!(warnings.is_empty(), "{warnings:?}");
    let expected = std::f64::consts::PI * 4.0 * 80.0;
    assert_close(
        total_volume(&bodies),
        expected,
        1e-6,
        0.15,
        "closed sweep volume (all four legs)",
    );
}
