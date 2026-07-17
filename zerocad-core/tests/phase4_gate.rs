//! Cross-cutting Phase 4 gate: curves/import, standards, parameters,
//! persistence, evaluated geometry, and inspection must agree end to end.

use std::collections::HashSet;

use zerocad_core::parametric::{FaceRef, ThreadStandard, METRIC_COARSE};
use zerocad_core::sketch::{Constraint, Dimension, EntityId, SketchSolverModel};
use zerocad_core::{
    read_document_from_slice, read_dxf_str, write_document_to_vec, CoordinateSystem, Document,
    FeatureNode, FeatureType, HoleApplication, HoleFit, HoleKind, HydrationBundle, LoadOptions,
    ParametricGraph, ResolvedStandardGeometry, SaveOptions, SketchCurves, SketchShape, Unit,
    Variable, HOLE_STANDARD_PRESETS,
};

const DXF: &str = "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n\
0\nSECTION\n2\nENTITIES\n\
0\nLWPOLYLINE\n8\nPROFILE\n70\n1\n10\n0\n20\n0\n10\n1\n20\n0\n10\n1\n20\n1\n10\n0\n20\n1\n\
0\nSPLINE\n8\nCURVES\n70\n0\n71\n2\n10\n0\n20\n0\n10\n1\n20\n2\n10\n2\n20\n0\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n\
0\nENDSEC\n0\nEOF\n";

fn evaluate_properties(graph: &ParametricGraph) -> openrcad::mesh::MassProperties {
    let (bodies, diagnostics) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("Phase 4 graph evaluates");
    assert!(diagnostics.is_empty(), "diagnostics: {diagnostics:?}");
    assert_eq!(bodies.len(), 1);
    bodies[0]
        .1
        .mass_properties()
        .expect("closed evaluated body")
}

#[test]
fn phase4_features_survive_rename_save_load_and_rebuild() {
    let imported = read_dxf_str("phase4-inch.dxf", DXF).expect("DXF import");
    assert_eq!(imported.unit.scale_to_mm, 25.4);
    assert_eq!(imported.shapes.len(), 2);
    assert!(imported.shapes.iter().any(|shape| matches!(
        shape,
        SketchShape::Imported { curves, metadata }
            if metadata.layer == "CURVES" && curves.splines.len() == 1
    )));

    let preset = HOLE_STANDARD_PRESETS
        .iter()
        .copied()
        .find(|preset| {
            preset.application == HoleApplication::Clearance
                && preset.fit == HoleFit::Normal
                && preset.designation == "M6"
        })
        .expect("ISO M6 clearance preset");
    let standard = preset.reference_with_resolved(
        preset.bore_diameter_mm,
        preset.head_diameter_mm,
        preset.head_depth_mm,
        preset.head_angle_deg,
    );

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "parameters".to_string(),
        name: "Parameters".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "clearance".to_string(),
                value: f64::from(preset.bore_diameter_mm),
                unit: Unit::Millimeter,
                expression: None,
            }],
        },
    });
    graph.add_feature(FeatureNode {
        id: "dxf_sketch".to_string(),
        name: "Imported profile".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: imported.shapes,
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "box".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 20.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "hole".to_string(),
        name: "ISO clearance hole".to_string(),
        feature: FeatureType::Hole {
            target: "box".to_string(),
            position: [10.0, 10.0, 10.0],
            direction: [0.0, 0.0, -1.0],
            diameter: preset.bore_diameter_mm,
            diameter_expr: Some("clearance".to_string()),
            depth: None,
            kind: HoleKind::Simple,
            standard: Some(standard.clone()),
            manufacturing: Some(preset.manufacturing_metadata(None)),
        },
    });
    graph.add_dependency("box", "hole");

    let before_rename = evaluate_properties(&graph);
    graph
        .rename_variable("clearance", "drill_diameter")
        .expect("safe rename");
    assert_eq!(
        graph.resolve_variables().values["drill_diameter"],
        f64::from(preset.bore_diameter_mm)
    );
    let after_rename = evaluate_properties(&graph);
    assert!((before_rename.volume - after_rename.volume).abs() < 1.0e-6);

    // Simulate a future standards package changing both its identity and its
    // table value. Existing geometry must continue to read the feature's frozen
    // numeric fields, never the live/reference metadata.
    let mut future_library = graph.clone_document();
    let future_hole = future_library
        .graph
        .node_weights_mut()
        .find(|feature| feature.id == "hole")
        .expect("future-library hole");
    if let FeatureType::Hole {
        standard: Some(reference),
        ..
    } = &mut future_hole.feature
    {
        reference.library_version += 1;
        reference.resolved = ResolvedStandardGeometry::Hole {
            bore_diameter_mm: 99.0,
            head_diameter_mm: Some(123.0),
            head_depth_mm: Some(45.0),
            head_angle_deg: Some(17.0),
            tap_pitch_mm: Some(8.0),
        };
    } else {
        panic!("hole must retain its standard reference")
    }
    let after_library_update = evaluate_properties(&future_library);
    assert!((after_library_update.volume - after_rename.volume).abs() < 1.0e-6);
    assert!((after_library_update.surface_area - after_rename.surface_area).abs() < 1.0e-6);
    let measured_before_save = graph.inspect_body("box").expect("strict body inspection");

    let document = Document::from_graph(graph, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save Phase 4 document");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load Phase 4 document")
        .document;
    let after_load = evaluate_properties(&loaded);
    assert!((after_load.volume - after_rename.volume).abs() < 1.0e-6);
    assert!((after_load.surface_area - after_rename.surface_area).abs() < 1.0e-6);

    let hole = loaded
        .graph
        .node_weights()
        .find(|feature| feature.id == "hole")
        .expect("hole feature");
    let FeatureType::Hole {
        diameter_expr,
        standard: loaded_standard,
        manufacturing,
        ..
    } = &hole.feature
    else {
        panic!("hole payload")
    };
    assert_eq!(diameter_expr.as_deref(), Some("drill_diameter"));
    assert_eq!(loaded_standard.as_ref(), Some(&standard));
    assert_eq!(
        manufacturing.as_ref().map(|metadata| metadata.application),
        Some(HoleApplication::Clearance)
    );

    let sketch = loaded
        .graph
        .node_weights()
        .find(|feature| feature.id == "dxf_sketch")
        .expect("DXF sketch");
    let FeatureType::Sketch { shapes, .. } = &sketch.feature else {
        panic!("sketch payload")
    };
    assert!(shapes.iter().any(|shape| matches!(
        shape,
        SketchShape::Imported { curves, metadata }
            if metadata.source_unit_code == 1 && curves.splines.len() == 1
    )));

    let measured = loaded.inspect_body("box").expect("strict body inspection");
    assert_eq!(measured, measured_before_save);
    let measured_volume = measured.volume_mm3.expect("closed volume after load");
    assert!((measured_volume - after_load.volume).abs() / measured_volume < 0.005);
    assert!(
        (measured.surface_area_mm2 - after_load.surface_area).abs() / measured.surface_area_mm2
            < 0.005
    );
}

#[test]
fn thread_standard_reference_round_trips_with_identity_and_class() {
    let preset = METRIC_COARSE
        .iter()
        .find(|preset| preset.designation == "M6×1")
        .expect("M6 thread preset");
    let standard = zerocad_core::parametric::thread_reference_with_class(
        ThreadStandard::MetricCoarse,
        preset,
        false,
        "4g6g",
    )
    .expect("selectable ISO class");
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "cylinder".to_string(),
        name: "Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 3.0, h: 12.0 },
    });
    graph.add_feature(FeatureNode {
        id: "thread".to_string(),
        name: "ISO thread".to_string(),
        feature: FeatureType::Thread {
            target: "cylinder".to_string(),
            face: FaceRef {
                centroid: [3.0, 6.0, 0.0],
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: preset.pitch_mm,
            depth: zerocad_core::parametric::default_depth_mm(preset.pitch_mm),
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: preset.designation.to_string(),
            standard: Some(standard.clone()),
        },
    });
    graph.add_dependency("cylinder", "thread");
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save standardized thread");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load standardized thread")
        .document;
    let loaded_reference = loaded
        .graph
        .node_weights()
        .find_map(|feature| match &feature.feature {
            FeatureType::Thread { standard, .. } => standard.as_ref(),
            _ => None,
        })
        .expect("thread standard reference");
    assert_eq!(loaded_reference, &standard);
    assert_eq!(loaded_reference.class.as_deref(), Some("4g6g"));
}

#[test]
fn section_and_interference_are_part_of_the_phase4_gate() {
    let mesh = zerocad_core::mock_kernel::MockMesh::make_box(10.0, 10.0, 10.0);
    let section = zerocad_core::mock_kernel::clip_mesh_by_plane(
        &mesh,
        [5.0, 5.0, 5.0],
        [1.0, 2.0, 3.0],
        true,
    )
    .expect("arbitrary visual section");
    assert!(!section.mesh.indices.is_empty());
    assert_eq!(section.contours.len(), 1);

    let mut graph = ParametricGraph::new();
    for id in ["overlap_a", "overlap_b"] {
        graph.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
    }
    let reports = graph
        .inspect_interference(&["overlap_a".to_string(), "overlap_b".to_string()], 1.0e-9)
        .expect("exact Common interference");
    assert_eq!(reports.len(), 1);
    assert!((reports[0].volume_mm3 - 1_000.0).abs() < 1.0e-6);
}

#[test]
fn projected_edges_and_reference_dimensions_round_trip_with_dependencies() {
    let mut solver = SketchSolverModel::default();
    let mut next = 1;
    zerocad_core::sketch::append_projected_edge(
        &mut solver,
        "source_box".to_string(),
        zerocad_core::parametric::EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [10.0, 0.0, 0.0],
            n1: [0.0, -1.0, 0.0],
            n2: [0.0, 0.0, -1.0],
            curve: Some(zerocad_core::mock_kernel::EdgeCurveHint::Line),
            topology: None,
        },
        CoordinateSystem::XY,
        &mut next,
    )
    .expect("project source edge");
    let projection = &solver.projected_edges[0];
    let dimension_id = EntityId(next);
    next += 1;
    solver.constraints.push(Constraint::Distance {
        id: dimension_id,
        a: projection.point_ids[0],
        b: projection.point_ids[1],
        d: Dimension::literal(10.0),
    });
    solver.driven_dimensions.push(dimension_id);

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "source_box".to_string(),
        name: "Source".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "reference_sketch".to_string(),
        name: "Reference sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: next,
            solver: Some(solver),
        },
    });
    graph.add_dependency("source_box", "reference_sketch");
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save projected sketch");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load projected sketch")
        .document;
    let model = loaded
        .graph
        .node_weights()
        .find_map(|feature| match &feature.feature {
            FeatureType::Sketch {
                solver: Some(model),
                ..
            } => Some(model),
            _ => None,
        })
        .expect("persisted solver model");
    assert_eq!(model.projected_edges.len(), 1);
    assert!(model.is_driven_dimension(dimension_id));
    assert!(model.is_projected_entity(model.projected_edges[0].entity_ids[0]));
}
