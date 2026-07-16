//! Cross-cutting Phase 3 construction-equivalence gate.

use std::collections::HashSet;

use zerocad_core::{
    read_document_from_slice, write_document_to_vec, CoordinateSystem, CornerKind, Document,
    EdgeRef, ExtrudeMode, FeatureId, FeatureNode, FeatureState, FeatureType, HoleKind,
    HydrationBundle, LoadOptions, ParametricGraph, SaveOptions, SketchCurves, Unit,
};

fn evaluate(graph: &ParametricGraph) -> Vec<(String, zerocad_core::MockMesh)> {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("equivalence graph must evaluate");
    assert!(warnings.is_empty(), "equivalence warnings: {warnings:?}");
    bodies
}

fn properties(graph: &ParametricGraph) -> openrcad::mesh::MassProperties {
    let bodies = evaluate(graph);
    assert_eq!(bodies.len(), 1, "equivalence fixture must make one body");
    bodies[0]
        .1
        .mass_properties()
        .expect("equivalence fixture must be closed")
}

fn assert_properties_close(
    left: openrcad::mesh::MassProperties,
    right: openrcad::mesh::MassProperties,
) {
    let relative = |a: f64, b: f64| (a - b).abs() / a.abs().max(b.abs()).max(1.0);
    assert!(
        relative(left.volume, right.volume) < 1.0e-4,
        "volume mismatch: {} vs {}",
        left.volume,
        right.volume
    );
    assert!(
        relative(left.surface_area, right.surface_area) < 1.0e-4,
        "area mismatch: {} vs {}",
        left.surface_area,
        right.surface_area
    );
    for axis in 0..3 {
        assert!(
            (left.centroid[axis] - right.centroid[axis]).abs() < 1.0e-4,
            "centroid axis {axis} mismatch: {:?} vs {:?}",
            left.centroid,
            right.centroid
        );
    }
}

fn add_box(graph: &mut ParametricGraph) {
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 16.0,
            d: 10.0,
        },
    });
}

fn add_hole(graph: &mut ParametricGraph, id: &str) {
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: "Hole".into(),
        feature: FeatureType::Hole {
            target: "box_1".into(),
            position: [10.0, 8.0, 10.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 4.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
        },
    });
    graph.add_dependency("box_1", id);
}

fn add_disjoint_fillet(graph: &mut ParametricGraph, id: &str) {
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: "Fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "box_1".into(),
            edge: EdgeRef {
                p0: [0.0, 0.0, 0.0],
                p1: [20.0, 0.0, 0.0],
                n1: [0.0, 0.0, -1.0],
                n2: [0.0, -1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist: 1.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("box_1", id);
}

#[test]
fn primitive_and_sketched_prism_are_equivalent() {
    let mut primitive = ParametricGraph::new();
    primitive.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 16.0,
            d: 10.0,
        },
    });

    let mut sketch = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (20.0, 16.0));
    sketch.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Rectangle".into(),
        feature: FeatureType::Sketch {
            entity_ids: Vec::new(),
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XY,
            curves,
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
        },
    });
    sketch.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Extrude".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: Vec::new(),
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    sketch.add_dependency("sketch_1", "extrude_2");

    assert_properties_close(properties(&primitive), properties(&sketch));
}

#[test]
fn disjoint_cut_and_fillet_orders_are_equivalent() {
    let mut fillet_then_cut = ParametricGraph::new();
    add_box(&mut fillet_then_cut);
    add_disjoint_fillet(&mut fillet_then_cut, "fillet_2");
    add_hole(&mut fillet_then_cut, "hole_3");
    fillet_then_cut.add_dependency("fillet_2", "hole_3");

    let mut cut_then_fillet = ParametricGraph::new();
    add_box(&mut cut_then_fillet);
    add_hole(&mut cut_then_fillet, "hole_2");
    add_disjoint_fillet(&mut cut_then_fillet, "fillet_3");
    cut_then_fillet.add_dependency("hole_2", "fillet_3");

    assert_properties_close(properties(&fillet_then_cut), properties(&cut_then_fillet));
}

#[test]
fn reordered_suppressed_timeline_survives_save_load_and_resume() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph);
    add_hole(&mut graph, "hole_2");
    assert!(graph.move_feature_in_timeline("hole_2", -1));
    assert!(graph.set_feature_suppressed("hole_2", true));
    let suppressed = properties(&graph);

    let document = Document::from_graph(graph, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save reordered suppressed document");
    let mut loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load reordered suppressed document")
        .document;

    let body = loaded
        .semantics
        .bodies
        .get(&zerocad_core::BodyId::from("box_1"))
        .expect("box timeline");
    assert_eq!(
        body.timeline,
        [FeatureId::from("hole_2"), FeatureId::from("box_1")]
    );
    assert_eq!(
        loaded.feature_state("hole_2"),
        Some(FeatureState::Suppressed)
    );
    assert_properties_close(suppressed, properties(&loaded));

    assert!(loaded.set_feature_suppressed("hole_2", false));
    let resumed = properties(&loaded);
    assert!(
        resumed.volume < suppressed.volume,
        "resuming the reordered hole must remove material"
    );
}
