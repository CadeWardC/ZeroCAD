//! Hole as a parametric feature: simple bore, counterbore, countersink,
//! through-all vs blind, all composed from analytic cutters + the guarded cut.

use super::*;
use crate::parametric::HoleKind;

fn add_box(g: &mut ParametricGraph, id: &str, w: f32, h: f32, d: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Box { w, h, d },
    });
}

/// A hole drilled straight down (−z... the box primitive extrudes its rect on
/// XY by depth `d` along +z? Use the geometry as-built: box_1 spans
/// x∈[0,w], y∈[0,h], z∈[0,d]; drill from the top face z=d downward).
#[allow(clippy::too_many_arguments)]
fn add_hole(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    position: [f32; 3],
    direction: [f32; 3],
    diameter: f32,
    depth: Option<f32>,
    kind: HoleKind,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Hole {
            target: target.to_string(),
            position,
            direction,
            diameter,
            diameter_expr: None,
            depth,
            kind,
            standard: None,
            manufacturing: None,
        },
    });
    g.add_dependency(target, id);
}

fn body_volume(bodies: &[(String, MockMesh)], id: &str) -> f64 {
    bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .and_then(|(_, m)| m.mass_properties())
        .expect("closed body")
        .volume
}

#[test]
fn through_hole_removes_full_cylinder() {
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
        HoleKind::Simple,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = body_volume(&bodies, "box_1");
    let exact = 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "through-hole volume {v} vs {exact}"
    );
}

#[test]
fn blind_hole_stops_at_depth() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [10.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        Some(4.0),
        HoleKind::Simple,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = body_volume(&bodies, "box_1");
    // The overshoot convention makes blind pockets ~CUT_OVERSHOOT deeper than
    // nominal (see the join.rs constant), so allow that band.
    let nominal = 4000.0 - std::f64::consts::PI * 9.0 * 4.0;
    let with_overshoot = 4000.0 - std::f64::consts::PI * 9.0 * 4.1;
    assert!(
        v <= nominal + 1.0 && v >= with_overshoot - 1.0,
        "blind-hole volume {v}, expected in [{with_overshoot}, {nominal}]"
    );
}

#[test]
fn blind_hole_models_the_persisted_drill_point() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [10.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        6.0,
        Some(4.0),
        HoleKind::Simple,
    );
    let hole = g.node_map["hole_2"];
    let FeatureType::Hole { manufacturing, .. } = &mut g.graph[hole].feature else {
        unreachable!()
    };
    *manufacturing = Some(crate::parametric::HoleManufacturingMetadata {
        application: crate::parametric::HoleApplication::Clearance,
        drill_point_angle_deg: Some(118.0),
        cosmetic_thread: false,
        thread_designation: None,
        thread_class: None,
        tap_pitch_mm: None,
    });

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let volume = body_volume(&bodies, "box_1");
    let radius = 3.0_f64;
    let point_height = radius / 59_f64.to_radians().tan();
    let removed = std::f64::consts::PI * radius * radius * 4.0
        + std::f64::consts::PI * radius * radius * point_height / 3.0;
    let expected = 4_000.0 - removed;
    assert!(
        (volume - expected).abs() / expected < 0.01,
        "drill-point volume {volume} vs {expected}"
    );
}

#[test]
fn counterbore_removes_extra_ring() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [10.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        4.0,
        None,
        HoleKind::Counterbore {
            diameter: 8.0,
            depth: 3.0,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = body_volume(&bodies, "box_1");
    // Through bore Ø4 + counterbore ring Ø8 over ~3mm (± the overshoot).
    let pi = std::f64::consts::PI;
    let nominal = 4000.0 - pi * 4.0 * 10.0 - pi * (16.0 - 4.0) * 3.0;
    assert!(
        (v - nominal).abs() < pi * 12.0 * 0.15 + 1.0,
        "counterbore volume {v} vs {nominal}"
    );
}

#[test]
fn countersink_removes_conical_ring() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [10.0, 10.0, 10.0],
        [0.0, 0.0, -1.0],
        4.0,
        None,
        HoleKind::Countersink {
            diameter: 8.0,
            angle_deg: 90.0,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = body_volume(&bodies, "box_1");
    // 90° countersink from Ø8 to Ø4: depth = (8−4)/2 / tan45 = 2. Removed ring
    // = frustum(r 4→2 over 2mm) − bore cylinder over the same 2mm.
    let pi = std::f64::consts::PI;
    let frustum = pi * 2.0 / 3.0 * (16.0 + 8.0 + 4.0);
    let ring = frustum - pi * 4.0 * 2.0;
    let nominal = 4000.0 - pi * 4.0 * 10.0 - ring;
    // Cone tessellation is coarse (see the revolve cone note) — 5% band.
    assert!(
        (v - nominal).abs() / nominal < 0.05,
        "countersink volume {v} vs {nominal}"
    );
}

#[test]
fn missing_target_warns() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "nonexistent",
        [5.0, 5.0, 10.0],
        [0.0, 0.0, -1.0],
        3.0,
        None,
        HoleKind::Simple,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1);
    assert!(
        warnings.iter().any(|w| w.contains("hole_2")),
        "warnings: {warnings:?}"
    );
    // The box is untouched.
    let v = body_volume(&bodies, "box_1");
    assert!((v - 1000.0).abs() < 1e-3);
}
