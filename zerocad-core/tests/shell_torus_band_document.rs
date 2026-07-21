//! End-to-end Wave 4C acceptance through the persisted parametric document.

use std::collections::HashSet;
use std::path::PathBuf;

use sha2::{Digest, Sha256};
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{
    detect_regions, read_document_file, write_document_file, CoordinateSystem, DatumPlaneDef,
    Document, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, HydrationBundle, LoadOptions,
    ParametricGraph, SaveOptions, SketchCurves, Unit, Vec3,
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

fn filleted_bite_graph() -> ParametricGraph {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 5.0), (40.0, 35.0));
    curves.add_circle((20.0, 8.0), 14.0);
    let region = detect_regions(&curves)
        .iter()
        .position(|region| region.contains((5.0, 30.0)) && !region.contains((20.0, 8.0)))
        .expect("material region above circular bite");

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch".into(),
        name: "Bite profile".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude".into(),
        name: "Extrude".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "extrude");

    let half = (14.0_f32 * 14.0 - 3.0 * 3.0).sqrt();
    graph.add_feature(FeatureNode {
        id: "fillet".into(),
        name: "Constant-radius bite fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude".into(),
            edge: EdgeRef {
                p0: [20.0 - half, 5.0, 10.0],
                p1: [20.0 + half, 5.0, 10.0],
                n1: [0.0, 0.0, 1.0],
                n2: [0.0, -1.0, 0.0],
                curve: Some(zerocad_core::mock_kernel::EdgeCurveHint::Circle {
                    center: [20.0, 8.0, 10.0],
                    axis: [0.0, 0.0, 1.0],
                    x_dir: [1.0, 0.0, 0.0],
                    radius: 14.0,
                    start: 6.0,
                    end: std::f32::consts::TAU + 3.4,
                    closed: false,
                }),
                topology: None,
            },
            dist: 1.5,
            dist_expr: None,
            kind: zerocad_core::CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude", "fillet");
    graph
}

fn add_shell_and_downstream_datum(graph: &mut ParametricGraph) {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate constant-radius fillet");
    assert!(warnings.is_empty(), "fillet warnings: {warnings:?}");
    let mesh = &bodies[0].1;
    let top = mesh
        .face_refs
        .iter()
        .filter(|face| face.normal[2] > 0.9 && face.centroid[2] > 9.9)
        .map(captured_face)
        .collect::<Vec<_>>();
    let bottom = mesh
        .face_refs
        .iter()
        .filter(|face| face.normal[2] < -0.9 && face.centroid[2] < 0.1)
        .map(captured_face)
        .collect::<Vec<_>>();
    assert!(!top.is_empty() && !bottom.is_empty());
    let retained_side = mesh
        .face_refs
        .iter()
        .find(|face| {
            face.normal[2].abs() < 0.1
                && face
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.face_id.as_deref())
                    .is_some()
        })
        .map(captured_face)
        .expect("durably named retained planar side face");

    let mut openings = top;
    openings.extend(bottom);
    graph.add_feature(FeatureNode {
        id: "shell".into(),
        name: "Torus-band Shell".into(),
        feature: FeatureType::Shell {
            target: "extrude".into(),
            thickness: 0.5,
            thickness_expr: None,
            open_faces: openings,
        },
    });
    graph.add_dependency("fillet", "shell");
    graph.add_feature(FeatureNode {
        id: "downstream_datum".into(),
        name: "Retained-face datum".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace {
                face: retained_side,
            },
        },
    });
    graph.add_dependency("shell", "downstream_datum");
}

#[test]
fn fillet_shell_multiple_openings_survive_real_disk_reload_and_reattach() {
    let mut graph = filleted_bite_graph();
    add_shell_and_downstream_datum(&mut graph);
    let before = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate torus-band Shell and downstream datum");
    assert!(before.1.is_empty(), "pre-save warnings: {:?}", before.1);
    assert_eq!(before.0.len(), 1);

    let directory = tempfile::tempdir().expect("temporary fixture directory");
    let path = directory.path().join("shell-on-fillet-band.zcad");
    let document = Document::from_graph(graph, Unit::Millimeter);
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("write canonical .zcad v5 fixture");
    assert!(path.is_file());

    let fixture_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("modeling_fixtures");
    let fixture_path = fixture_directory.join("shell-on-fillet-band.zcad");
    if std::env::var_os("ZEROCAD_UPDATE_MODELING_FIXTURES").is_some() {
        std::fs::create_dir_all(&fixture_directory).expect("create fixture directory");
        write_document_file(
            &fixture_path,
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .expect("promote canonical .zcad v5 fixture");
        return;
    }

    let fixture_bytes = std::fs::read(&fixture_path)
        .expect("the promoted torus-band Shell fixture must be present");
    let temporary_bytes = std::fs::read(&path).expect("read temporary lifecycle file");
    assert_eq!(
        fixture_bytes, temporary_bytes,
        "the public-operation recipe must reproduce the promoted fixture byte-for-byte"
    );
    let manifest: serde_json::Value =
        serde_json::from_str(include_str!("modeling_fixtures/manifest.json"))
            .expect("valid modeling fixture manifest");
    let fixture = &manifest["fixtures"][0];
    let actual_hash = format!("{:x}", Sha256::digest(&fixture_bytes));
    assert_eq!(fixture["file"], "shell-on-fillet-band.zcad");
    assert_eq!(fixture["sha256"], actual_hash);
    assert_eq!(fixture["expected_resolution"], "resolved");
    assert_eq!(fixture["body_count"], 1);
    assert_eq!(fixture["measurements"]["opening_count"], 2);
    assert_eq!(fixture["measurements"]["wall_thickness_mm"], 0.5);

    let loaded = read_document_file(&fixture_path, &LoadOptions::default())
        .expect("reload canonical .zcad v5 fixture")
        .document;
    for feature_id in fixture["required_features"]
        .as_array()
        .expect("required feature list")
    {
        let feature_id = feature_id.as_str().expect("feature id string");
        assert!(
            loaded.evaluator_graph().feature_state(feature_id).is_some(),
            "promoted fixture is missing required feature {feature_id}"
        );
    }
    let after = loaded
        .evaluator_graph()
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate reloaded torus-band Shell and downstream datum");
    assert!(after.1.is_empty(), "post-load warnings: {:?}", after.1);
    assert_eq!(after.0.len(), 1);
    assert_eq!(before.0[0].1.face_ids, after.0[0].1.face_ids);
}
