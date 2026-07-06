//! Revolve as a parametric feature: sketch → Revolve node → live body, with
//! datum-axis support, partial angles, and cut mode.

use super::*;
use crate::parametric::{AxisBase, DatumAxisDef};

fn add_revolve(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    axis: AxisBase,
    angle_deg: f32,
    mode: ExtrudeMode,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Revolve {
            axis,
            angle_deg,
            angle_expr: None,
            region_indices: vec![],
            mode,
            target: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

fn volume(mesh: &MockMesh) -> f64 {
    mesh.mass_properties().expect("closed mesh").volume
}

#[test]
fn full_revolve_makes_cylinder_body() {
    // Rect x∈[0,2], y∈[0,5] on the XY plane, revolved about the sketch-plane
    // Y axis → cylinder r=2 h=5 along Y.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (2.0, 5.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    let exact = std::f64::consts::PI * 4.0 * 5.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "cylinder volume {v} vs {exact}"
    );
}

#[test]
fn partial_revolve_about_datum_axis() {
    // Quarter-turn washer about a datum axis at x=0 (the sketch Y axis).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "datumaxis_1".to_string(),
        name: "axis".to_string(),
        feature: FeatureType::DatumAxis {
            def: DatumAxisDef::TwoPoints {
                a: [0.0, 0.0, 0.0],
                b: [0.0, 1.0, 0.0],
            },
        },
    });
    add_sketch(&mut g, "sketch_2", rect_sketch((1.0, 0.0), (2.0, 3.0)));
    add_revolve(
        &mut g,
        "revolve_3",
        "sketch_2",
        AxisBase::Datum("datumaxis_1".to_string()),
        90.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    let exact = std::f64::consts::PI * (4.0 - 1.0) * 3.0 / 4.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "quarter washer volume {v} vs {exact}"
    );
}

#[test]
fn crossing_profile_warns_and_builds_nothing() {
    // Rect straddling the axis: fails loud, no body.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((-1.0, 0.0), (1.0, 2.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(bodies.is_empty());
    assert!(
        warnings.iter().any(|w| w.contains("revolve_2")),
        "warnings: {warnings:?}"
    );
}

#[test]
fn revolve_cut_bores_groove_into_box() {
    // 10×10×10 box, then cut a full-turn revolved ring (rect r∈[8,11] of the
    // box's local frame? keep simple: revolve about the Y axis at the box
    // corner, cutting a quarter-cylinder groove out of the box edge).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    // Sketch on XY: rect x∈[0,2], y∈[0,10] revolved 360° about the sketch Y
    // axis = cylinder r=2 through the box's z column at the origin corner.
    add_sketch(&mut g, "sketch_2", rect_sketch((0.0, 0.0), (2.0, 10.0)));
    add_revolve(
        &mut g,
        "revolve_3",
        "sketch_2",
        AxisBase::Y,
        360.0,
        ExtrudeMode::Cut,
    );
    g.add_dependency("box_1", "revolve_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    // Box minus the quarter of the cylinder that overlaps it (cylinder axis
    // on the box corner along Y): 1000 − (π·4·10)/4.
    let exact = 1000.0 - std::f64::consts::PI * 4.0 * 10.0 / 4.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "cut volume {v} vs {exact}"
    );
}
