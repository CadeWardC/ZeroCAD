//! Wave 5F product gate for exact Smooth Loft persistence, naming, caching,
//! and real save-to-disk/reload behavior.

use std::collections::HashSet;
use std::path::PathBuf;

use sha2::{Digest, Sha256};
use zerocad_core::{
    read_document_file, write_document_file, CoordinateSystem, Document, ExtrudeMode, FeatureNode,
    FeatureType, HydrationBundle, LoadOptions, LoftSurfaceMode, ParametricGraph, SaveOptions,
    SketchCurves, Unit, Vec3,
};

fn add_rectangle_section(
    graph: &mut ParametricGraph,
    id: &str,
    z: f32,
    center_x: f32,
    half_width: f32,
    half_height: f32,
) {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((-half_width, -half_height), (half_width, half_height));
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: format!("Smooth section {id}"),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XY.with_origin(Vec3::new(center_x, 0.0, z)),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
}

fn smooth_loft_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add_rectangle_section(&mut graph, "section_0", 0.0, 0.0, 2.0, 1.5);
    add_rectangle_section(&mut graph, "section_1", 3.0, 0.2, 2.4, 1.2);
    add_rectangle_section(&mut graph, "section_2", 7.0, -0.1, 1.8, 1.7);
    add_rectangle_section(&mut graph, "section_3", 12.0, 0.4, 2.8, 1.4);
    let sections = (0..4)
        .map(|index| (format!("section_{index}"), 0))
        .collect::<Vec<_>>();
    graph.add_feature(FeatureNode {
        id: "smooth_loft".into(),
        name: "Exact analytic Smooth Loft".into(),
        feature: FeatureType::Loft {
            sections: sections.clone(),
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    for (section, _) in sections {
        graph.add_dependency(&section, "smooth_loft");
    }
    graph
}

fn assert_resolved_geometry(graph: &ParametricGraph) -> Vec<u32> {
    let cold = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("cold exact Smooth Loft evaluation");
    let warm = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("warm exact Smooth Loft evaluation");
    assert!(cold.1.is_empty(), "cold warnings: {:?}", cold.1);
    assert!(warm.1.is_empty(), "warm warnings: {:?}", warm.1);
    assert_eq!(cold.0.len(), 1);
    assert_eq!(cold.0[0].1.indices, warm.0[0].1.indices);
    assert_eq!(cold.0[0].1.face_ids, warm.0[0].1.face_ids);
    assert!(cold.0[0].1.mass_properties().unwrap().volume > 0.0);
    assert!(cold.0[0].1.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some()
    }));
    cold.0[0].1.face_ids.clone()
}

#[test]
fn exact_smooth_loft_survives_real_disk_reload_with_durable_topology() {
    let graph = smooth_loft_graph();
    let before_face_ids = assert_resolved_geometry(&graph);
    let document = Document::from_graph(graph, Unit::Millimeter);
    let directory = tempfile::tempdir().expect("temporary Smooth Loft fixture directory");
    let path = directory.path().join("smooth-loft-analytic.zcad");
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write canonical Smooth Loft .zcad v5 fixture");

    let fixture_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("modeling_fixtures");
    let fixture_path = fixture_directory.join("smooth-loft-analytic.zcad");
    if std::env::var_os("ZEROCAD_UPDATE_MODELING_FIXTURES").is_some() {
        std::fs::create_dir_all(&fixture_directory).expect("create fixture directory");
        write_document_file(
            &fixture_path,
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("promote canonical Smooth Loft fixture");
        return;
    }

    let fixture_bytes =
        std::fs::read(&fixture_path).expect("the promoted Smooth Loft fixture must be present");
    assert_eq!(
        fixture_bytes,
        std::fs::read(&path).expect("read temporary Smooth Loft lifecycle file")
    );
    let manifest: serde_json::Value =
        serde_json::from_str(include_str!("modeling_fixtures/manifest.json"))
            .expect("valid modeling fixture manifest");
    let fixture = manifest["fixtures"]
        .as_array()
        .expect("fixture entries")
        .iter()
        .find(|fixture| fixture["file"] == "smooth-loft-analytic.zcad")
        .expect("Smooth Loft manifest entry");
    assert_eq!(
        fixture["sha256"],
        format!("{:x}", Sha256::digest(&fixture_bytes))
    );
    assert_eq!(fixture["expected_resolution"], "resolved");
    assert_eq!(fixture["body_count"], 1);
    assert_eq!(fixture["measurements"]["section_count"], 4);
    assert_eq!(fixture["measurements"]["analytic_span_count"], 16);

    let loaded = read_document_file(&fixture_path, &LoadOptions::default())
        .expect("reload canonical Smooth Loft fixture")
        .document;
    for feature_id in fixture["required_features"]
        .as_array()
        .expect("required feature list")
    {
        let feature_id = feature_id.as_str().expect("feature id string");
        assert!(
            loaded.evaluator_graph().feature_state(feature_id).is_some(),
            "Smooth Loft fixture is missing required feature {feature_id}"
        );
    }
    let after_face_ids = assert_resolved_geometry(loaded.evaluator_graph());
    assert_eq!(before_face_ids, after_face_ids);
}
