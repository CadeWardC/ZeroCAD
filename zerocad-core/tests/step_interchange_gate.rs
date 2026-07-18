//! Cross-cutting Phase 6 gate: closed migration boundaries, append-only sketch
//! and standards data, exact inspection, and strict STEP interchange.

use openrcad::foundation::{Ax2, Dir, Pnt};
use std::fs;
use zerocad_core::sketch::{
    Constraint, Dimension, EntityId, SketchEntity, SketchPoint, SketchSolverModel,
};
use zerocad_core::{
    body_output_id, read_document_from_slice, write_document_to_vec, CoordinateSystem, Document,
    FeatureNode, FeatureType, HydrationBundle, LoadOptions, ParametricGraph, SaveOptions,
    SketchCurves, SplineContinuity, SplineKind, Unit, STANDARDS_DATA_PACKS,
    STANDARDS_LIBRARY_VERSION,
};

#[test]
fn semantic_body_output_ownership_is_behavioral_and_round_trips() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "midplane".into(),
        name: "Midplane".into(),
        feature: FeatureType::DatumPlane {
            def: zerocad_core::DatumPlaneDef::Offset {
                base: zerocad_core::PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "split".into(),
        name: "Split".into(),
        feature: FeatureType::BodySplit {
            target: "box".into(),
            plane: zerocad_core::PlaneBase::Datum("midplane".into()),
            face: None,
        },
    });
    graph.add_dependency("box", "split");
    graph.add_dependency("midplane", "split");

    let secondary = body_output_id("split", 1);
    assert_eq!(graph.body_producer_feature_id(&secondary), Some("split"));
    assert_eq!(
        graph.body_producer_feature_id("unregistered::body:2"),
        None,
        "identifier spelling must never imply ownership"
    );
    graph.semantics.body_outputs.insert(
        "opaque-secondary-output".into(),
        zerocad_core::document::FeatureId::from("split"),
    );
    assert_eq!(
        graph.body_producer_feature_id("opaque-secondary-output"),
        Some("split"),
        "the semantic table must resolve ids with no naming convention"
    );

    let (mut bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("evaluate split through semantic ownership");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    bodies.sort_by(|left, right| left.0.cmp(&right.0));
    assert_eq!(
        bodies
            .iter()
            .map(|body| body.0.as_str())
            .collect::<Vec<_>>(),
        vec!["split", secondary.as_str()]
    );
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save semantic ownership document");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load semantic ownership document")
        .document;
    assert_eq!(loaded.body_producer_feature_id(&secondary), Some("split"));
    assert_eq!(
        loaded.body_producer_feature_id("unregistered::body:2"),
        None
    );
}

#[test]
fn sketch_and_standards_extensions_survive_the_canonical_document() {
    assert_eq!(
        STANDARDS_DATA_PACKS.last().map(|pack| pack.version),
        Some(STANDARDS_LIBRARY_VERSION)
    );
    assert!(STANDARDS_DATA_PACKS
        .windows(2)
        .all(|pair| pair[0].version < pair[1].version));

    let center = EntityId(1);
    let spline = EntityId(10);
    let line = EntityId(11);
    let model = SketchSolverModel {
        points: vec![
            SketchPoint {
                id: center,
                pos: (0.0, 0.0),
            },
            SketchPoint {
                id: EntityId(2),
                pos: (1.0, 0.0),
            },
            SketchPoint {
                id: EntityId(3),
                pos: (2.0, 1.0),
            },
            SketchPoint {
                id: EntityId(4),
                pos: (2.0, 0.0),
            },
        ],
        entities: vec![
            SketchEntity::Ellipse {
                id: EntityId(9),
                center,
                major_axis: [4.0, 0.0],
                minor_axis: [0.0, 2.0],
                start_parameter: 0.0,
                end_parameter: std::f64::consts::TAU,
                closed: true,
                derived_from: None,
            },
            SketchEntity::Spline {
                id: spline,
                points: vec![center, EntityId(2), EntityId(3)],
                kind: SplineKind::ControlPoint,
                degree: 2,
                knots: Vec::new(),
                weights: Vec::new(),
                closed: false,
                periodic: false,
                continuity: SplineContinuity::Curvature,
                trim: None,
                derived_from: None,
            },
            SketchEntity::Line {
                id: line,
                p0: center,
                p1: EntityId(4),
                derived_from: None,
            },
        ],
        constraints: vec![
            Constraint::SplineTangent {
                id: EntityId(20),
                spline,
                line,
                at_start: true,
            },
            Constraint::SplineCurvature {
                id: EntityId(21),
                spline,
                at_start: true,
                radius: Dimension::literal(1.0),
            },
        ],
        ..Default::default()
    };
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "phase6_sketch".into(),
        name: "Phase 6 sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 22,
            solver: Some(model),
        },
    });
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save Phase 6 sketch");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
        .expect("load Phase 6 sketch")
        .document;
    let loaded_model = loaded
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
    assert!(loaded_model
        .entities
        .iter()
        .any(|entity| matches!(entity, SketchEntity::Ellipse { .. })));
    assert!(loaded_model
        .constraints
        .iter()
        .any(|constraint| matches!(constraint, Constraint::SplineTangent { .. })));
    assert!(loaded_model
        .constraints
        .iter()
        .any(|constraint| matches!(constraint, Constraint::SplineCurvature { .. })));
}

#[test]
fn exact_inspection_and_strict_step_round_trip_cross_the_interchange_gate() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 8.0,
            d: 6.0,
        },
    });
    let edge = zerocad_core::parametric::EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [10.0, 0.0, 0.0],
        n1: [0.0, -1.0, 0.0],
        n2: [0.0, 0.0, -1.0],
        curve: Some(zerocad_core::mock_kernel::EdgeCurveHint::Line),
        topology: None,
    };
    let measured = graph
        .inspect_edge("box", &edge)
        .expect("exact edge inspection");
    assert!((measured.length_mm - 10.0).abs() < 1.0e-8);
    assert_eq!(measured.curve_kind, "line");

    let axis = Ax2::new(Pnt::new(0.0, 0.0, 0.0), Dir::new(0.0, 0.0, 1.0));
    let cone = openrcad::primitives::make_cone_operation(&axis, 4.0, 0.0, 9.0)
        .expect("strict apex cone")
        .value;
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    let path = std::env::temp_dir().join(format!("zerocad-phase6-{nonce}.step"));
    openrcad::exchange::write_step(&cone, path.to_str().expect("UTF-8 temp path"))
        .expect("write strict STEP");
    let parsed = openrcad::exchange::read_step_operation(path.to_str().expect("UTF-8 temp path"))
        .expect("read strict STEP");
    let _ = fs::remove_file(path);
    assert!(parsed.validation.is_valid());
    assert!(parsed.validation.pcurves_complete);
    assert!(parsed.validation.watertight);
    assert!(parsed
        .history
        .coverage_for_solid(&parsed.value)
        .is_complete());
}

#[test]
fn byte_frozen_third_party_freecad_step_fixture_imports_strictly() {
    const FIXTURE: &[u8] = include_bytes!("fixtures/step/mayo-freecad-cube.step");
    assert_eq!(
        blake3::hash(FIXTURE).to_hex().as_str(),
        "6bffcf5f83c9149474cff18801176a6c57d6e31f516927b39630ea31ab3a8e2e",
        "the licensed upstream fixture changed; review and deliberately refreeze it"
    );
    let step = std::str::from_utf8(FIXTURE).expect("ASCII STEP fixture");
    assert!(step.contains("FreeCAD"));
    let parsed = openrcad::exchange::read_step_str_operation(step)
        .expect("strictly import Mayo's FreeCAD AP214 cube");
    assert!(parsed.validation.is_valid());
    assert!(parsed.validation.watertight);
    assert!(parsed.validation.pcurves_complete);
    assert!(parsed
        .history
        .coverage_for_solid(&parsed.value)
        .is_complete());
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    parsed
        .value
        .validate_strict_with_policy(&policy)
        .expect("strict topology");
    openrcad::mesh::tessellate_checked_with_policy(&parsed.value, 0.05, 0.25, &policy)
        .expect("checked tessellation");
}
