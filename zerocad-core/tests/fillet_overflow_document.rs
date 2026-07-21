//! End-to-end Wave 5C acceptance through the persisted parametric document.

use std::collections::HashSet;
use std::path::PathBuf;

use sha2::{Digest, Sha256};
use zerocad_core::parametric::TopologyEdgeRef;
use zerocad_core::{
    read_document_file, write_document_file, CoordinateSystem, CornerKind, Document, EdgeRef,
    ExtrudeMode, FeatureNode, FeatureType, HydrationBundle, LoadOptions, ParametricGraph,
    SaveOptions, SketchCurves, Unit,
};

fn captured_edge(body_id: &str, edge: &zerocad_core::mock_kernel::MeshEdgeRef) -> EdgeRef {
    EdgeRef {
        p0: edge.p0,
        p1: edge.p1,
        n1: edge.n1,
        n2: edge.n2,
        curve: edge.curve.clone(),
        topology: edge.topology.as_ref().map(|topology| TopologyEdgeRef {
            body_id: topology
                .body_id
                .clone()
                .or_else(|| Some(body_id.to_string())),
            topology_version: topology.topology_version,
            edge_id: topology.edge_id.clone(),
            adjacent_face_ids: topology.adjacent_face_ids.clone(),
            curve_kind: topology.curve_kind.clone(),
            adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    }
}

fn overflow_graph() -> ParametricGraph {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (1.0, 4.0));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch".into(),
        name: "Overflow profile".into(),
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
        name: "Straight planar prism".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 1.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "extrude");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate overflow source prism");
    assert!(warnings.is_empty(), "source warnings: {warnings:?}");
    let source = bodies.first().expect("one source body");
    let selected = source
        .1
        .edge_refs
        .iter()
        .find(|edge| {
            let on_selected_line =
                |point: [f32; 3]| point[0].abs() < 1.0e-4 && point[1].abs() < 1.0e-4;
            on_selected_line(edge.p0)
                && on_selected_line(edge.p1)
                && (edge.p0[2] - edge.p1[2]).abs() > 0.9
        })
        .map(|edge| captured_edge(&source.0, edge))
        .expect("durably named vertical overflow edge");
    assert!(
        selected
            .topology
            .as_ref()
            .and_then(|topology| topology.edge_id.as_deref())
            .is_some(),
        "fixture selection must use a durable edge name"
    );

    graph.add_feature(FeatureNode {
        id: "fillet".into(),
        name: "Successive-face overflow fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude".into(),
            edge: selected,
            dist: 1.5,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude", "fillet");
    graph
}

fn assert_resolved_geometry(graph: &ParametricGraph) -> Vec<u32> {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate persisted overflow fillet");
    assert!(warnings.is_empty(), "overflow warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;
    let requested_arcs = mesh
        .edge_refs
        .iter()
        .filter(|edge| {
            matches!(
                edge.curve,
                Some(zerocad_core::mock_kernel::EdgeCurveHint::Circle { radius, .. })
                    if (radius - 1.5).abs() <= 1.0e-4
            )
        })
        .count();
    assert!(
        requested_arcs >= 2,
        "the exact radius-1.5 overflow band must expose both cap arcs"
    );
    assert!(
        mesh.face_refs
            .iter()
            .filter(|face| face
                .topology
                .as_ref()
                .and_then(|topology| topology.face_id.as_deref())
                .is_some())
            .count()
            >= 5,
        "overflow result must retain durable face names"
    );
    mesh.face_ids.clone()
}

#[test]
fn overflow_fillet_survives_real_disk_reload_with_durable_topology() {
    let graph = overflow_graph();
    let before_face_ids = assert_resolved_geometry(&graph);
    let document = Document::from_graph(graph, Unit::Millimeter);
    let directory = tempfile::tempdir().expect("temporary overflow fixture directory");
    let path = directory.path().join("fillet-overflow.zcad");
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write canonical overflow .zcad v5 fixture");

    let fixture_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("modeling_fixtures");
    let fixture_path = fixture_directory.join("fillet-overflow.zcad");
    if std::env::var_os("ZEROCAD_UPDATE_MODELING_FIXTURES").is_some() {
        std::fs::create_dir_all(&fixture_directory).expect("create fixture directory");
        write_document_file(
            &fixture_path,
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("promote canonical overflow .zcad v5 fixture");
        return;
    }

    let fixture_bytes =
        std::fs::read(&fixture_path).expect("the promoted fillet-overflow fixture must be present");
    let temporary_bytes = std::fs::read(&path).expect("read temporary overflow lifecycle file");
    assert_eq!(
        fixture_bytes, temporary_bytes,
        "the public-operation recipe must reproduce the overflow fixture byte-for-byte"
    );
    let manifest: serde_json::Value =
        serde_json::from_str(include_str!("modeling_fixtures/manifest.json"))
            .expect("valid modeling fixture manifest");
    let fixture = manifest["fixtures"]
        .as_array()
        .expect("fixture entries")
        .iter()
        .find(|fixture| fixture["file"] == "fillet-overflow.zcad")
        .expect("fillet-overflow manifest entry");
    let actual_hash = format!("{:x}", Sha256::digest(&fixture_bytes));
    assert_eq!(fixture["sha256"], actual_hash);
    assert_eq!(fixture["expected_resolution"], "resolved");
    assert_eq!(fixture["body_count"], 1);
    assert_eq!(fixture["measurements"]["fillet_radius_mm"], 1.5);
    assert_eq!(fixture["measurements"]["successive_planar_faces"], 1);

    let loaded = read_document_file(&fixture_path, &LoadOptions::default())
        .expect("reload canonical overflow .zcad v5 fixture")
        .document;
    for feature_id in fixture["required_features"]
        .as_array()
        .expect("required feature list")
    {
        let feature_id = feature_id.as_str().expect("feature id string");
        assert!(
            loaded.evaluator_graph().feature_state(feature_id).is_some(),
            "overflow fixture is missing required feature {feature_id}"
        );
    }
    let after_face_ids = assert_resolved_geometry(loaded.evaluator_graph());
    assert_eq!(before_face_ids, after_face_ids);
}
