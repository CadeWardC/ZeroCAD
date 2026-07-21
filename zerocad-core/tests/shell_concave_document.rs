//! Wave 5E product gate for concave Shell persistence, naming, diagnostics,
//! cache equivalence, and atomic rejection.

use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::atomic::AtomicU64;
use std::sync::Arc;

use sha2::{Digest, Sha256};
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{
    read_document_file, write_document_file, CoordinateSystem, DiagnosticParameterValue, Document,
    EvaluationCancellation, EvaluationQuality, ExtrudeMode, FeatureNode, FeatureType,
    HydrationBundle, LoadOptions, ParametricGraph, SaveOptions, SketchCurves, Unit,
};

fn captured_face(face: &zerocad_core::mock_kernel::MeshFaceRef) -> FaceRef {
    FaceRef {
        centroid: face.centroid,
        normal: face.normal,
        topology: face.topology.as_ref().map(|topology| TopologyFaceRef {
            body_id: topology.body_id.clone(),
            component_id: topology.component_id.clone(),
            topology_version: topology.topology_version,
            face_id: topology.face_id.clone(),
            surface_kind: topology.surface_kind.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    }
}

fn concave_shell_graph(thickness: f32) -> ParametricGraph {
    let points = [
        (0.0, 0.0),
        (8.0, 0.0),
        (8.0, 3.0),
        (3.0, 3.0),
        (3.0, 8.0),
        (0.0, 8.0),
    ];
    let mut curves = SketchCurves::new();
    for index in 0..points.len() {
        curves.add_line(points[index], points[(index + 1) % points.len()]);
    }
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch".into(),
        name: "Concave L profile".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude".into(),
        name: "Concave prism".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "extrude");
    let source = graph
        .evaluate_bodies(&HashSet::new())
        .expect("evaluate concave prism");
    let top = source[0]
        .1
        .face_refs
        .iter()
        .filter(|face| face.normal[2] > 0.9)
        .max_by(|first, second| first.centroid[2].total_cmp(&second.centroid[2]))
        .map(captured_face)
        .expect("durably named top opening");
    graph.add_feature(FeatureNode {
        id: "shell".into(),
        name: "Concave Shell".into(),
        feature: FeatureType::Shell {
            target: "extrude".into(),
            thickness,
            thickness_expr: None,
            open_faces: vec![top],
        },
    });
    graph.add_dependency("extrude", "shell");
    graph
}

fn resolved_face_ids(graph: &ParametricGraph) -> Vec<u32> {
    let cold = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("cold concave Shell evaluation");
    let warm = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("warm concave Shell evaluation");
    assert!(cold.1.is_empty(), "cold warnings: {:?}", cold.1);
    assert!(warm.1.is_empty(), "warm warnings: {:?}", warm.1);
    assert_eq!(cold.0[0].1.indices, warm.0[0].1.indices);
    assert_eq!(cold.0[0].1.face_ids, warm.0[0].1.face_ids);
    assert!(cold.0[0].1.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some()
    }));
    cold.0[0].1.face_ids.clone()
}

#[test]
fn concave_shell_survives_real_disk_reload_with_durable_topology() {
    let graph = concave_shell_graph(0.5);
    let latest = Arc::new(AtomicU64::new(1));
    let evaluated = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .expect("evaluate certified concave Shell");
    let certificate = evaluated
        .diagnostics
        .iter()
        .find(|diagnostic| {
            diagnostic.feature_id == "shell"
                && diagnostic.code.as_str() == "kernel.shell_concave_certificate"
        })
        .unwrap_or_else(|| {
            panic!(
                "normal evaluation must publish the concave-envelope certificate: {:?}",
                evaluated.diagnostics
            )
        });
    assert_eq!(
        certificate.parameters.get("kernel_code"),
        Some(&DiagnosticParameterValue::Text(
            "shell.concave_certificate".into()
        ))
    );
    let before_face_ids = resolved_face_ids(&graph);
    let document = Document::from_graph(graph, Unit::Millimeter);
    let directory = tempfile::tempdir().expect("temporary concave Shell directory");
    let path = directory.path().join("shell-concave-l-prism.zcad");
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write canonical concave Shell .zcad v5 fixture");

    let fixture_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("modeling_fixtures");
    let fixture_path = fixture_directory.join("shell-concave-l-prism.zcad");
    if std::env::var_os("ZEROCAD_UPDATE_MODELING_FIXTURES").is_some() {
        std::fs::create_dir_all(&fixture_directory).expect("create fixture directory");
        write_document_file(
            &fixture_path,
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("promote canonical concave Shell fixture");
        return;
    }

    let fixture_bytes =
        std::fs::read(&fixture_path).expect("the promoted concave Shell fixture must be present");
    let temporary_bytes = std::fs::read(&path).expect("read temporary concave Shell file");
    assert_eq!(fixture_bytes, temporary_bytes);
    let manifest: serde_json::Value =
        serde_json::from_str(include_str!("modeling_fixtures/manifest.json"))
            .expect("valid modeling fixture manifest");
    let fixture = manifest["fixtures"]
        .as_array()
        .expect("fixture entries")
        .iter()
        .find(|fixture| fixture["file"] == "shell-concave-l-prism.zcad")
        .expect("concave Shell manifest entry");
    assert_eq!(
        fixture["sha256"],
        format!("{:x}", Sha256::digest(&fixture_bytes))
    );
    assert_eq!(fixture["expected_resolution"], "resolved");
    assert_eq!(fixture["body_count"], 1);
    assert_eq!(fixture["measurements"]["wall_thickness_mm"], 0.5);
    assert_eq!(fixture["measurements"]["concave_edges"], 1);

    let loaded = read_document_file(&fixture_path, &LoadOptions::default())
        .expect("reload canonical concave Shell fixture")
        .document;
    let after_face_ids = resolved_face_ids(loaded.evaluator_graph());
    assert_eq!(before_face_ids, after_face_ids);
}

#[test]
fn concave_shell_collapse_is_typed_atomic_and_cache_stable() {
    let graph = concave_shell_graph(2.0);
    let latest = Arc::new(AtomicU64::new(1));
    let evaluate = || {
        graph
            .evaluate_request(
                &HashSet::new(),
                EvaluationQuality::Final,
                &EvaluationCancellation::new(1, Arc::clone(&latest)),
            )
            .expect("typed concave Shell rejection remains an evaluation result")
    };
    let cold = evaluate();
    let warm = evaluate();
    assert_eq!(cold.bodies[0].1.indices, warm.bodies[0].1.indices);
    let diagnostic = cold
        .diagnostics
        .iter()
        .find(|diagnostic| {
            diagnostic.feature_id == "shell" && diagnostic.code.as_str() == "shell.offset_collapse"
        })
        .expect("typed Shell collapse diagnostic");
    assert_eq!(
        diagnostic.parameters.get("thickness"),
        Some(&DiagnosticParameterValue::Decimal("2".into()))
    );
    assert!(matches!(
        diagnostic.parameters.get("maximum"),
        Some(DiagnosticParameterValue::Decimal(_))
    ));
    assert!((cold.bodies[0].1.mass_properties().unwrap().volume - 195.0).abs() < 0.1);
    assert_eq!(
        cold.diagnostics
            .iter()
            .map(|diagnostic| (&diagnostic.code, &diagnostic.parameters))
            .collect::<Vec<_>>(),
        warm.diagnostics
            .iter()
            .map(|diagnostic| (&diagnostic.code, &diagnostic.parameters))
            .collect::<Vec<_>>()
    );
}
