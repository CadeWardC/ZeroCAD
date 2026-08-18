//! Regression for fillet/chamfer on the circular rim of an extruded annulus.
//! This is the GUI workflow of sketching two concentric circles, extruding the
//! material between them, and selecting the outer top rim.

use std::collections::HashSet;

use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{
    detect_regions, CoordinateSystem, CornerKind, EdgeRef, ExtrudeMode, FeatureNode, FeatureType,
    ParametricGraph, SketchCurves,
};

const OUTER_RADIUS: f32 = 10.0;
const INNER_RADIUS: f32 = 5.0;
const HEIGHT: f32 = 10.0;
// Matches the value from the reported failing ring workflow.
const BLEND_SIZE: f32 = 2.77;

fn annular_graph() -> ParametricGraph {
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), OUTER_RADIUS);
    curves.add_circle((0.0, 0.0), INNER_RADIUS);
    let regions = detect_regions(&curves);
    let annulus = regions
        .iter()
        .position(|region| region.contains((7.5, 0.0)))
        .expect("annular material region");

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XZ,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Extrude".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: HEIGHT,
            region_indices: vec![annulus],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");
    graph
}

fn outer_top_rim() -> EdgeRef {
    EdgeRef {
        p0: [OUTER_RADIUS, HEIGHT, 0.0],
        p1: [0.0, HEIGHT, OUTER_RADIUS],
        n1: [0.0, 1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, HEIGHT, 0.0],
            axis: [0.0, 1.0, 0.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: OUTER_RADIUS,
            start: 0.0,
            end: std::f32::consts::TAU,
            closed: true,
        }),
        topology: None,
    }
}

fn assert_annular_rim_blend_commits(kind: CornerKind) {
    let mut graph = annular_graph();
    let (plain, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("plain annulus evaluates");
    assert!(warnings.is_empty(), "plain annulus warnings: {warnings:?}");
    let plain_volume = plain[0]
        .1
        .mass_properties()
        .expect("plain annulus volume")
        .volume;

    graph.add_feature(FeatureNode {
        id: "edgemod_3".into(),
        name: format!("{kind:?}"),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: outer_top_rim(),
            dist: BLEND_SIZE,
            dist_expr: None,
            kind,
        },
    });
    graph.add_dependency("extrude_2", "edgemod_3");

    let (blended, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("annular blend evaluates");
    assert!(
        warnings.is_empty(),
        "{kind:?} on an annular outer rim must commit: {warnings:?}"
    );
    let blended_volume = blended[0]
        .1
        .mass_properties()
        .expect("blended annulus volume")
        .volume;
    assert!(
        blended_volume < plain_volume - 1.0,
        "{kind:?} must remove material: {blended_volume} vs {plain_volume}"
    );
}

#[test]
fn annular_outer_rim_fillet_commits() {
    assert_annular_rim_blend_commits(CornerKind::Fillet);
}

#[test]
fn annular_outer_rim_chamfer_commits() {
    assert_annular_rim_blend_commits(CornerKind::Chamfer);
}
