//! `.zcad` container guarantees: a document round-trips through the binary
//! format, corruption is detected (never a panic), incompatible prototype files
//! are rejected, and unknown container sections remain safely skippable.

// This suite intentionally exercises the deprecated `.zcad` compatibility
// adapters alongside the canonical Document APIs. Production use is forbidden
// by the Phase 2/6 architecture gates; retaining adapter coverage keeps the
// downstream source-compatibility promise testable without warning noise.
#![allow(deprecated)]

use std::collections::HashSet;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::zcad_format::{
    read_document, read_document_file, read_document_from_slice, read_zcad, read_zcad_file,
    write_document, write_document_file, write_document_to_vec, write_zcad, write_zcad_file,
    HydrationBundle, LoadDiagnostic, LoadOptions, SaveOptions, SaveProfile, ZcadDocument,
    ZcadError, CURRENT_VERSION, MAGIC,
};
use zerocad_core::{Document, FeatureNode, FeatureState, FeatureType, ParametricGraph, Unit};

fn feature_payload_corpus() -> Vec<FeatureType> {
    use zerocad_core::mock_kernel::EdgeCurveHint;
    use zerocad_core::{
        AxisBase, CoordinateSystem, DatumAxisDef, DatumPlaneDef, DatumPointDef, EdgeCornerMode,
        EdgeRef, ExtrudeMode, HoleKind, PatternKind, PlaneBase, SketchCurves, Variable,
    };

    let face = FaceRef {
        centroid: [1.0, 2.0, 3.0],
        normal: [0.0, 0.0, 1.0],
        topology: None,
    };
    let direct_face = FaceRef {
        topology: Some(TopologyFaceRef {
            body_id: Some("missing_a".into()),
            face_id: Some("foreign:face:1".into()),
            producer_feature_id: Some("missing_a".into()),
            ..TopologyFaceRef::default()
        }),
        ..face.clone()
    };
    vec![
        FeatureType::Origin,
        FeatureType::Box {
            w: 2.0,
            h: 3.0,
            d: 4.0,
        },
        FeatureType::Cylinder { r: 2.0, h: 5.0 },
        FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 1,
            solver: None,
        },
        FeatureType::Extrude {
            depth: 6.0,
            region_indices: vec![0],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: Some("3*2".into()),
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
        FeatureType::EdgeMod {
            target: "missing_body".into(),
            edge: EdgeRef {
                p0: [0.0, 0.0, 0.0],
                p1: [1.0, 0.0, 0.0],
                n1: [0.0, 1.0, 0.0],
                n2: [0.0, 0.0, 1.0],
                curve: Some(EdgeCurveHint::Line),
                topology: None,
            },
            dist: 0.2,
            dist_expr: Some("0.1*2".into()),
            kind: zerocad_core::CornerKind::Chamfer,
        },
        FeatureType::EdgeBlend {
            target: "missing_body".into(),
            edges: vec![EdgeRef {
                p0: [0.0, 0.0, 0.0],
                p1: [1.0, 0.0, 0.0],
                n1: [0.0, 1.0, 0.0],
                n2: [0.0, 0.0, 1.0],
                curve: Some(EdgeCurveHint::Line),
                topology: None,
            }],
            dist: 0.2,
            dist_expr: Some("0.1*2".into()),
            kind: zerocad_core::CornerKind::Fillet,
            corner_mode: EdgeCornerMode::Miter,
        },
        FeatureType::VariableSet {
            variables: vec![Variable::new("width", Unit::Millimeter)],
        },
        FeatureType::Import {
            step_data: "ISO-10303-21;END-ISO-10303-21;".into(),
            label: "fixture".into(),
        },
        FeatureType::Revolve {
            axis: AxisBase::X,
            angle_deg: 180.0,
            angle_expr: Some("90*2".into()),
            region_indices: vec![0],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        FeatureType::Loft {
            sections: vec![("section_a".into(), 0), ("section_b".into(), 1)],
            surface_mode: zerocad_core::LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
        FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: -45.0,
            total_twist_expr: Some("-twist".into()),
            guide: None,
        },
        FeatureType::Shell {
            target: "missing_body".into(),
            thickness: 0.5,
            thickness_expr: None,
            open_faces: vec![face.clone()],
        },
        FeatureType::Hole {
            target: "missing_body".into(),
            position: [1.0, 1.0, 1.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: Some(4.0),
            kind: HoleKind::Counterbore {
                diameter: 4.0,
                depth: 1.0,
            },
            standard: None,
            manufacturing: None,
        },
        FeatureType::Pattern {
            source: "missing_body".into(),
            kind: PatternKind::Linear {
                dir: AxisBase::Y,
                spacing: 3.0,
                spacing_expr: None,
                count: 3,
            },
        },
        FeatureType::BodyTransform {
            source: "missing_body".into(),
            translation: [1.0, 2.0, 3.0],
            copy: true,
        },
        FeatureType::Thread {
            target: "missing_body".into(),
            face,
            internal: true,
            pitch: 1.0,
            depth: 0.3,
            angle_deg: 60.0,
            right_handed: true,
            starts: 2,
            length: Some(8.0),
            flip: true,
            designation: "M6x1".into(),
            standard: None,
        },
        FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
        FeatureType::DatumAxis {
            def: DatumAxisDef::TwoPoints {
                a: [0.0, 0.0, 0.0],
                b: [0.0, 0.0, 1.0],
            },
        },
        FeatureType::DatumPoint {
            def: DatumPointDef::Coords { p: [1.0, 2.0, 3.0] },
        },
        FeatureType::BodyJoin {
            sources: vec!["missing_a".into(), "missing_b".into()],
        },
        FeatureType::BodyCut {
            target: "missing_a".into(),
            tool: "missing_b".into(),
            keep_tool: true,
        },
        FeatureType::BodyIntersect {
            target: "missing_a".into(),
            tool: "missing_b".into(),
            keep_tool: true,
        },
        FeatureType::BodySplit {
            target: "missing_a".into(),
            plane: PlaneBase::Datum("missing_plane".into()),
            face: None,
        },
        FeatureType::BodyScale {
            source: "missing_a".into(),
            factor: 2.0,
            factor_expr: Some("scale_factor".into()),
            center: [1.0, 2.0, 3.0],
        },
        FeatureType::FaceOffset {
            target: "missing_a".into(),
            face: direct_face.clone(),
            distance: -2.0,
            distance_expr: Some("-wall".into()),
        },
        FeatureType::FaceMove {
            target: "missing_a".into(),
            face: direct_face.clone(),
            translation: [0.0, 0.0, 2.0],
        },
        FeatureType::FaceDelete {
            target: "missing_a".into(),
            face: direct_face.clone(),
        },
        FeatureType::FaceThicken {
            target: "missing_a".into(),
            face: direct_face,
            thickness: 1.5,
            thickness_expr: Some("sheet".into()),
            reverse: true,
        },
        FeatureType::ImportStl {
            stl_data: b"solid empty\nendsolid empty\n".to_vec(),
            label: "empty.stl".into(),
        },
    ]
}

/// A non-trivial document: a box plus a cylinder cut into it.
fn sample_graph() -> ParametricGraph {
    let mut pg = ParametricGraph::new();
    pg.add_feature(FeatureNode {
        id: "box1".to_string(),
        name: "Block".to_string(),
        feature: FeatureType::Box {
            w: 40.0,
            h: 20.0,
            d: 30.0,
        },
    });
    pg.add_feature(FeatureNode {
        id: "cyl1".to_string(),
        name: "Post".to_string(),
        feature: FeatureType::Cylinder { r: 5.0, h: 25.0 },
    });
    pg
}

fn doc_for(graph: &ParametricGraph) -> ZcadDocument<'_> {
    ZcadDocument {
        graph,
        thumbnail_png: None,
        mesh_cache: None,
        units: Unit::Millimeter,
        bbox: [0.0; 6],
        created_unix: None,
        hidden_nodes: HashSet::new(),
        evaluation_cache: None,
        hydrated_cache_limit: None,
    }
}

#[test]
fn round_trip_recipe_only() {
    let pg = sample_graph();
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    assert_eq!(&bytes[0..4], MAGIC, "file must start with the magic bytes");
    assert_eq!(
        u16::from_le_bytes([bytes[4], bytes[5]]),
        CURRENT_VERSION,
        "writer must stamp the current geometry-semantics contract"
    );

    let loaded = read_zcad(&bytes).expect("read");
    assert!(loaded.mesh_cache.is_none());
    assert_eq!(loaded.metadata.feature_count, pg.graph.node_count() as u32);

    // The restored graph must evaluate to the same geometry.
    let before = pg.evaluate().expect("eval original");
    let after = loaded.graph.evaluate().expect("eval restored");
    assert_eq!(before.indices.len(), after.indices.len());
    assert_eq!(before.vertices.len(), after.vertices.len());
}

#[test]
fn v5_compact_save_is_deterministic_and_strips_accelerators() {
    let mut graph = sample_graph();
    assert!(graph.set_feature_suppressed("cyl1", true));
    let bodies = graph.evaluate_bodies(&HashSet::new()).expect("bodies");
    let mut document = Document::from_graph(graph, Unit::Millimeter);
    document.set_visible("box1", false);
    let accelerators = HydrationBundle {
        small_preview_png: Some(vec![0x89, b'P', b'N', b'G']),
        large_preview_png: Some(vec![7; 1024]),
        display_meshes: Some(bodies),
        evaluation_cache: Some(document.evaluation_cache_snapshot()),
        world_bbox: None,
    };
    let options = SaveOptions {
        profile: SaveProfile::Compact,
    };

    let first = write_document_to_vec(&document, &options, &accelerators).expect("first save");
    let second = write_document_to_vec(&document, &options, &accelerators).expect("second save");
    assert_eq!(
        first, second,
        "identical document state must be byte-stable"
    );

    let loaded = read_document_from_slice(&first, &LoadOptions::default()).expect("load");
    assert_eq!(loaded.profile, SaveProfile::Compact);
    assert!(loaded.accelerators.display_meshes.is_none());
    assert!(loaded.accelerators.evaluation_cache.is_none());
    assert!(loaded.accelerators.large_preview_png.is_none());
    assert!(!loaded.document.is_visible("box1"));
    assert_eq!(
        loaded.document.feature_state("cyl1"),
        Some(FeatureState::Suppressed)
    );
}

#[test]
fn canonical_save_rejects_semantic_dependency_drift() {
    let mut graph = sample_graph();
    graph.add_dependency("box1", "cyl1");
    let cylinder = graph
        .graph
        .node_indices()
        .find(|&index| graph.graph[index].id == "cyl1")
        .unwrap();
    graph.graph[cylinder]
        .inputs
        .retain(|input| input.role != "dependency");

    let document = Document::from_graph(graph, Unit::Millimeter);
    let error = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap_err();
    assert!(
        matches!(error, ZcadError::Decode(message) if message.contains("semantic inputs disagree"))
    );
}

#[test]
fn canonical_round_trip_preserves_creation_time_and_prunes_stale_visibility() {
    let mut document = Document::from_graph(sample_graph(), Unit::Millimeter);
    document.state.created_unix = Some(1_700_000_123);
    document.set_visible("box1", false);
    document.set_visible("deleted_feature", false);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default()).expect("load");

    assert_eq!(loaded.document.state.created_unix, Some(1_700_000_123));
    assert!(!loaded.document.is_visible("box1"));
    assert!(loaded.document.is_visible("deleted_feature"));
    assert!(loaded.diagnostics.iter().any(|diagnostic| matches!(
        diagnostic,
        LoadDiagnostic::PrunedStaleVisibility { entity_id }
            if entity_id == "deleted_feature"
    )));

    let saved_again = write_document_to_vec(
        &loaded.document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save again");
    let reopened = read_document_from_slice(&saved_again, &LoadOptions::default()).expect("reopen");
    assert_eq!(reopened.document.state.created_unix, Some(1_700_000_123));
}

#[test]
fn every_feature_payload_round_trips_and_rebuilds_identically() {
    for feature in feature_payload_corpus() {
        let kind_id = feature.kind_id();
        let mut graph = ParametricGraph::new();
        let feature_id = if matches!(feature, FeatureType::Origin) {
            "origin"
        } else {
            graph.add_feature(FeatureNode {
                id: "feature_1".into(),
                name: kind_id.into(),
                feature: feature.clone(),
            });
            "feature_1"
        };
        graph
            .validate_semantic_contracts()
            .unwrap_or_else(|error| panic!("{kind_id}: invalid source contract: {error}"));
        let before = graph
            .evaluate_bodies_with_warnings(&HashSet::new())
            .unwrap_or_else(|error| panic!("{kind_id}: source rebuild failed: {error}"));
        let document = Document::from_graph(graph, Unit::Millimeter);
        let options = SaveOptions::default();
        let first = write_document_to_vec(&document, &options, &HydrationBundle::default())
            .unwrap_or_else(|error| panic!("{kind_id}: save failed: {error}"));
        let second = write_document_to_vec(&document, &options, &HydrationBundle::default())
            .unwrap_or_else(|error| panic!("{kind_id}: repeated save failed: {error}"));
        assert_eq!(first, second, "{kind_id}: compact bytes are not stable");

        let loaded = read_document_from_slice(&first, &LoadOptions::default())
            .unwrap_or_else(|error| panic!("{kind_id}: load failed: {error}"));
        let restored = loaded
            .document
            .graph
            .node_weights()
            .find(|node| node.id == feature_id)
            .unwrap_or_else(|| panic!("{kind_id}: restored feature is missing"));
        assert_eq!(restored.feature.kind_id(), kind_id);
        assert_eq!(
            serde_json::to_value(&restored.feature).unwrap(),
            serde_json::to_value(&feature).unwrap(),
            "{kind_id}: feature payload changed"
        );

        let after = loaded
            .document
            .evaluate_bodies_with_warnings(&HashSet::new())
            .unwrap_or_else(|error| panic!("{kind_id}: restored rebuild failed: {error}"));
        assert_eq!(before.1, after.1, "{kind_id}: diagnostics changed");
        assert_eq!(
            before.0.len(),
            after.0.len(),
            "{kind_id}: body count changed"
        );
        for ((before_id, before_mesh), (after_id, after_mesh)) in before.0.iter().zip(&after.0) {
            assert_eq!(before_id, after_id, "{kind_id}: body identity changed");
            assert_eq!(
                before_mesh.vertices, after_mesh.vertices,
                "{kind_id}: vertices changed"
            );
            assert_eq!(
                before_mesh.indices, after_mesh.indices,
                "{kind_id}: indices changed"
            );
        }
    }
}

#[test]
fn stl_asset_chain_round_trips_with_sequence_suppression_and_exact_bytes() {
    let stl = br#"solid tetrahedron
facet normal 0 0 -1
 outer loop
  vertex 0 0 0
  vertex 0 1 0
  vertex 1 0 0
 endloop
endfacet
facet normal 0 -1 0
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 0 1
 endloop
endfacet
facet normal -1 0 0
 outer loop
  vertex 0 0 0
  vertex 0 0 1
  vertex 0 1 0
 endloop
endfacet
facet normal 1 1 1
 outer loop
  vertex 1 0 0
  vertex 0 1 0
  vertex 0 0 1
 endloop
endfacet
endsolid tetrahedron
"#
    .to_vec();
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "mesh".into(),
        name: "Mesh".into(),
        feature: FeatureType::ImportStl {
            stl_data: stl.clone(),
            label: "tetrahedron.stl".into(),
        },
    });
    graph.add_feature(FeatureNode {
        id: "mesh_move".into(),
        name: "Move mesh".into(),
        feature: FeatureType::BodyTransform {
            source: "mesh".into(),
            translation: [2.0, 3.0, 4.0],
            copy: false,
        },
    });
    graph.add_dependency("mesh", "mesh_move");
    assert!(graph.set_feature_sequence("mesh_move", zerocad_core::document::SequenceKey(1)));
    assert!(graph.set_feature_sequence("mesh", zerocad_core::document::SequenceKey(2)));
    assert!(graph.set_feature_suppressed("mesh_move", true));
    graph
        .validate_semantic_contracts()
        .expect("mesh-chain semantic contract");

    let bytes = write_zcad(&doc_for(&graph)).expect("write mesh document");
    let mut loaded = read_zcad(&bytes).expect("read mesh document").graph;
    assert_eq!(
        loaded.feature_state("mesh_move"),
        Some(FeatureState::Suppressed)
    );
    let mesh_node = loaded
        .graph
        .node_indices()
        .find(|&index| loaded.graph[index].id == "mesh")
        .expect("loaded mesh feature");
    assert_eq!(
        match &loaded.graph[mesh_node].feature {
            FeatureType::ImportStl { stl_data, .. } => Some(stl_data.as_slice()),
            _ => None,
        },
        Some(stl.as_slice())
    );
    assert_eq!(
        loaded.feature_sequence("mesh_move"),
        Some(zerocad_core::document::SequenceKey(1))
    );
    let suppressed = loaded
        .evaluate_bodies(&HashSet::new())
        .expect("suppressed loaded mesh chain");
    assert_eq!(suppressed.len(), 1);
    assert_eq!(suppressed[0].0, "mesh");

    assert!(loaded.set_feature_suppressed("mesh_move", false));
    let evaluated = loaded
        .evaluate_bodies(&HashSet::new())
        .expect("restored loaded mesh chain");
    assert_eq!(evaluated.len(), 1);
    assert_eq!(evaluated[0].0, "mesh_move");
    assert!(loaded.inspect_body("mesh_move").is_ok());
}

#[test]
fn hydrated_budget_is_hard_and_profile_comes_from_content() {
    let graph = sample_graph();
    let bodies = graph.evaluate_bodies(&HashSet::new()).expect("bodies");
    let document = Document::from_graph(graph, Unit::Millimeter);
    let accelerators = HydrationBundle {
        small_preview_png: None,
        large_preview_png: Some(vec![1; 512]),
        display_meshes: Some(bodies),
        evaluation_cache: Some(document.evaluation_cache_snapshot()),
        world_bbox: None,
    };
    let profile = SaveProfile::Hydrated {
        total_accelerator_budget: 1,
    };
    let bytes = write_document_to_vec(&document, &SaveOptions { profile }, &accelerators)
        .expect("hydrated save");
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default()).expect("load");
    assert_eq!(loaded.profile, profile);
    assert!(loaded.accelerators.display_meshes.is_none());
    assert!(loaded.accelerators.evaluation_cache.is_none());
    assert!(loaded.accelerators.large_preview_png.is_none());
}

#[test]
fn compact_profile_has_bounded_overhead_and_no_forbidden_sections() {
    let document = Document::from_graph(sample_graph(), Unit::Millimeter);
    let preview = vec![7; 32 * 1024];
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle {
            small_preview_png: Some(preview),
            large_preview_png: Some(vec![1; 4096]),
            display_meshes: Some(document.evaluate_bodies(&HashSet::new()).unwrap()),
            evaluation_cache: Some(document.evaluation_cache_snapshot()),
            world_bbox: None,
        },
    )
    .unwrap();
    let ids = section_ids(&bytes);
    assert!(!ids.contains(&4), "compact file contains display meshes");
    assert!(!ids.contains(&6), "compact file contains checkpoints");
    assert!(!ids.contains(&7), "compact file contains a large preview");

    let required_payload_bytes: usize = ids
        .iter()
        .filter_map(|id| section_entry(&bytes, *id))
        .filter(|(entry, _, _)| bytes[*entry + 2] & 1 != 0)
        .map(|(_, _, len)| len)
        .sum();
    assert!(
        bytes.len() - required_payload_bytes <= 40 * 1024,
        "container framing plus tiny preview exceeds 40 KiB"
    );
}

#[test]
fn streaming_apis_round_trip_without_a_full_file_adapter() {
    let document = Document::from_graph(sample_graph(), Unit::Millimeter);
    let mut cursor = std::io::Cursor::new(Vec::new());
    write_document(
        &mut cursor,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    cursor.set_position(11);
    let loaded = read_document(&mut cursor, &LoadOptions::default()).unwrap();
    assert_eq!(loaded.document.graph.node_count(), 3);
}

#[test]
fn hydrated_sections_fail_independently_without_losing_the_recipe() {
    let graph = sample_graph();
    let bodies = graph.evaluate_bodies(&HashSet::new()).unwrap();
    let expected = bodies.clone();
    let document = Document::from_graph(graph, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions {
            profile: SaveProfile::Hydrated {
                total_accelerator_budget: 128 * 1024 * 1024,
            },
        },
        &HydrationBundle {
            small_preview_png: None,
            large_preview_png: Some(vec![3; 4096]),
            display_meshes: Some(bodies),
            evaluation_cache: Some(document.evaluation_cache_snapshot()),
            world_bbox: None,
        },
    )
    .unwrap();

    for section in [4u16, 6, 7] {
        let mut corrupt = bytes.clone();
        let (_, offset, _) = section_entry(&corrupt, section)
            .unwrap_or_else(|| panic!("hydrated section {section} was not emitted"));
        corrupt[offset] ^= 0x80;
        let loaded = read_document_from_slice(&corrupt, &LoadOptions::default())
            .unwrap_or_else(|error| panic!("section {section} broke recipe load: {error}"));
        assert!(loaded.diagnostics.iter().any(|diagnostic| matches!(
            diagnostic,
            LoadDiagnostic::DiscardedDisposableSection { section: actual, .. }
                if *actual == section
        )));
        let rebuilt = loaded.document.evaluate_bodies(&HashSet::new()).unwrap();
        assert_eq!(rebuilt.len(), expected.len());
        for ((expected_id, expected_mesh), (actual_id, actual_mesh)) in
            expected.iter().zip(&rebuilt)
        {
            assert_eq!(expected_id, actual_id);
            assert_eq!(expected_mesh.vertices, actual_mesh.vertices);
            assert_eq!(expected_mesh.indices, actual_mesh.indices);
        }
    }
}

#[test]
fn invalid_mesh_accelerator_is_discarded_by_content_validation() {
    let document = Document::from_graph(sample_graph(), Unit::Millimeter);
    let mut invalid = zerocad_core::MockMesh::empty();
    invalid.vertices = vec![0.0; 6];
    invalid.indices = vec![0, 1, 0];
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions {
            profile: SaveProfile::Hydrated {
                total_accelerator_budget: 1024 * 1024,
            },
        },
        &HydrationBundle {
            display_meshes: Some(vec![("box1".into(), invalid)]),
            ..HydrationBundle::default()
        },
    )
    .unwrap();
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default()).unwrap();
    assert!(loaded.accelerators.display_meshes.is_none());
    assert!(loaded.diagnostics.iter().any(|diagnostic| matches!(
        diagnostic,
        LoadDiagnostic::DiscardedDisposableSection { section: 4, reason }
            if reason.contains("invalid index")
    )));
}

#[test]
fn extension_mismatch_opens_by_content_and_warns() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("hydrated-content.zcad");
    let document = Document::from_graph(sample_graph(), Unit::Millimeter);
    write_document_file(
        &path,
        &document,
        &SaveOptions {
            profile: SaveProfile::Hydrated {
                total_accelerator_budget: 64 * 1024 * 1024,
            },
        },
        &HydrationBundle::default(),
    )
    .unwrap();
    let loaded = read_document_file(&path, &LoadOptions::default()).unwrap();
    assert!(matches!(loaded.profile, SaveProfile::Hydrated { .. }));
    assert!(loaded
        .diagnostics
        .iter()
        .any(|diagnostic| matches!(diagnostic, LoadDiagnostic::ExtensionProfileMismatch { .. })));
}

#[test]
fn failed_atomic_save_preserves_the_previous_complete_file() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("part.zcad");
    let original = Document::from_graph(sample_graph(), Unit::Millimeter);
    write_document_file(
        &path,
        &original,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();

    let mut invalid_graph = ParametricGraph::new();
    invalid_graph.add_feature(FeatureNode {
        id: "invalid".into(),
        name: "Invalid".into(),
        feature: FeatureType::Box {
            w: f32::NAN,
            h: 1.0,
            d: 1.0,
        },
    });
    let invalid = Document::from_graph(invalid_graph, Unit::Millimeter);
    assert!(write_document_file(
        &path,
        &invalid,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .is_err());

    let loaded = read_document_file(&path, &LoadOptions::default()).unwrap();
    assert_eq!(loaded.document.graph.node_count(), 3);
}

#[test]
fn non_finite_parameters_are_rejected_before_storage() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "bad".into(),
        name: "Bad box".into(),
        feature: FeatureType::Box {
            w: f32::NAN,
            h: 1.0,
            d: 1.0,
        },
    });
    let document = Document::from_graph(graph, Unit::Millimeter);
    assert!(matches!(
        write_document_to_vec(&document, &SaveOptions::default(), &HydrationBundle::default()),
        Err(ZcadError::Decode(message)) if message.contains("non-finite")
    ));
}

#[test]
fn load_limits_fail_before_decoding_large_sections() {
    let document = Document::from_graph(sample_graph(), Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    let mut options = LoadOptions::default();
    options.limits.max_recipe_bytes = 1;
    assert!(matches!(
        read_document_from_slice(&bytes, &options),
        Err(ZcadError::LimitExceeded {
            what: "decoded section length",
            ..
        })
    ));
}

#[test]
fn corrupt_disposable_preview_is_dropped_with_a_diagnostic() {
    let graph = sample_graph();
    let document = Document::from_graph(graph, Unit::Millimeter);
    let mut bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle {
            small_preview_png: Some(vec![0x89, b'P', b'N', b'G', 1, 2, 3]),
            ..HydrationBundle::default()
        },
    )
    .unwrap();
    let (_, offset, _) = section_entry(&bytes, 3).expect("preview section");
    bytes[offset] ^= 0xff;

    let loaded = read_document_from_slice(&bytes, &LoadOptions::default()).expect("recipe loads");
    assert!(loaded.accelerators.small_preview_png.is_none());
    assert!(loaded.diagnostics.iter().any(|diagnostic| matches!(
        diagnostic,
        LoadDiagnostic::DiscardedDisposableSection { section: 3, .. }
    )));
}

#[test]
fn connected_component_face_identity_round_trips() {
    let mut pg = sample_graph();
    pg.sketch_face_refs.insert(
        "attached_sketch".into(),
        FaceRef {
            centroid: [4.0, 5.0, 6.0],
            normal: [1.0, 0.0, 0.0],
            topology: Some(TopologyFaceRef {
                body_id: Some("box1".to_string()),
                component_id: Some("0:0:0:400000:200000:300000".to_string()),
                topology_version: Some(0),
                face_id: Some("box_box1:face:+x".to_string()),
                surface_kind: Some("plane".to_string()),
                producer_feature_id: Some("box1".to_string()),
                source_entity_id: None,
            }),
        },
    );

    let bytes = write_zcad(&doc_for(&pg)).expect("write component-aware document");
    let loaded = read_zcad(&bytes).expect("read component-aware document");
    let component_id = loaded
        .graph
        .sketch_face_refs
        .get("attached_sketch")
        .and_then(|face| face.topology.as_ref())
        .and_then(|topology| topology.component_id.as_deref());
    assert_eq!(
        component_id,
        Some("0:0:0:400000:200000:300000"),
        "component identity is authoritative recipe data"
    );
}

#[test]
fn incompatible_binary_contract_is_rejected() {
    let pg = sample_graph();
    let mut bytes = write_zcad(&doc_for(&pg)).expect("write");
    let obsolete = CURRENT_VERSION - 1;
    bytes[4..6].copy_from_slice(&obsolete.to_le_bytes());
    let digest = blake3::hash(&bytes[0..12]);
    bytes[12..28].copy_from_slice(&digest.as_bytes()[..16]);

    assert!(
        matches!(read_zcad(&bytes), Err(ZcadError::UnsupportedVersion(v)) if v == obsolete),
        "old geometry semantics must not be loaded as though they were current"
    );
}

#[test]
fn path_save_replaces_an_existing_document_safely() {
    let dir = tempfile::tempdir().expect("temporary directory");
    let path = dir.path().join("part.zcad");

    let first = sample_graph();
    write_zcad_file(&path, &doc_for(&first)).expect("initial atomic save");

    let mut second = sample_graph();
    second.add_feature(FeatureNode {
        id: "box2".into(),
        name: "Second block".into(),
        feature: FeatureType::Box {
            w: 5.0,
            h: 6.0,
            d: 7.0,
        },
    });
    write_zcad_file(&path, &doc_for(&second)).expect("replacement atomic save");

    let loaded = read_zcad_file(&path).expect("read replacement");
    assert_eq!(loaded.graph.graph.node_count(), second.graph.node_count());
    assert!(
        dir.path()
            .read_dir()
            .expect("list temp directory")
            .all(|entry| !entry
                .expect("directory entry")
                .file_name()
                .to_string_lossy()
                .contains(".tmp-")),
        "successful save must not leave temporary files"
    );
}

#[test]
fn round_trip_with_thumbnail_and_mesh_cache() {
    let pg = sample_graph();
    let bodies = pg.evaluate_bodies(&Default::default()).expect("bodies");
    let thumb = vec![0x89, b'P', b'N', b'G', 1, 2, 3, 4]; // stand-in PNG bytes
    let doc = ZcadDocument {
        graph: &pg,
        thumbnail_png: Some(thumb.clone()),
        mesh_cache: Some(&bodies),
        units: Unit::Inch,
        bbox: [-1.0, -2.0, -3.0, 4.0, 5.0, 6.0],
        created_unix: Some(1_700_000_000),
        hidden_nodes: HashSet::new(),
        evaluation_cache: None,
        hydrated_cache_limit: None,
    };
    let bytes = write_zcad(&doc).expect("write");
    let loaded = read_zcad(&bytes).expect("read");

    assert_eq!(loaded.thumbnail_png.as_deref(), Some(thumb.as_slice()));
    assert_eq!(loaded.metadata.units, Unit::Inch);
    assert_eq!(loaded.metadata.created_unix, 1_700_000_000);
    assert_eq!(loaded.metadata.bbox, [-1.0, -2.0, -3.0, 4.0, 5.0, 6.0]);

    let cache = loaded.mesh_cache.expect("fresh mesh cache present");
    assert_eq!(cache.len(), bodies.len());
}

#[test]
fn hydrated_checkpoint_cache_round_trips_and_rebuilds_identically() {
    let pg = sample_graph();
    let bodies = pg
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("populate cache")
        .0;
    let snapshot = pg.evaluation_cache_snapshot();
    let doc = ZcadDocument {
        graph: &pg,
        thumbnail_png: None,
        mesh_cache: Some(&bodies),
        units: Unit::Millimeter,
        bbox: [0.0; 6],
        created_unix: None,
        hidden_nodes: HashSet::new(),
        evaluation_cache: Some(&snapshot),
        hydrated_cache_limit: Some(128 * 1024 * 1024),
    };
    let bytes = write_zcad(&doc).expect("write hydrated document");
    let loaded = read_zcad(&bytes).expect("read hydrated document");
    let cache = loaded
        .evaluation_cache
        .expect("fresh hydrated checkpoint cache");
    loaded.graph.install_evaluation_cache(cache);
    let rebuilt = loaded
        .graph
        .evaluate_bodies(&HashSet::new())
        .expect("evaluate from hydrated cache");
    assert_eq!(rebuilt.len(), bodies.len());
    for ((expected_id, expected), (actual_id, actual)) in bodies.iter().zip(&rebuilt) {
        assert_eq!(expected_id, actual_id);
        assert_eq!(expected.indices, actual.indices);
        assert_eq!(expected.vertices, actual.vertices);
    }
}

#[test]
fn round_trip_overlapping_boolean_extrude() {
    use std::collections::HashMap;
    use zerocad_core::{
        CoordinateSystem, Dimension, ExtrudeMode, Region, SketchCurves, SketchShape,
    };

    // A rectangle (kept) with a circle overlapping its right edge (cut): the
    // overlapping-shapes-as-boolean case must survive a save/load round-trip,
    // because the boolean is recomputed from the persisted `shapes` +
    // `region_indices`.
    let rect = SketchShape::Rectangle {
        origin: (0.0, 0.0),
        sx: 1.0,
        sy: 1.0,
        w: Dimension::literal(20.0),
        h: Dimension::literal(10.0),
        from_center: false,
    };
    let circle = SketchShape::Circle {
        center: (18.0, 5.0),
        diameter: Dimension::literal(8.0),
    };
    let shapes = vec![rect, circle];

    // The rect-only material region (selected base).
    let curves = zerocad_core::build_sketch_curves(&shapes, &HashMap::new());
    let base: Vec<usize> = zerocad_core::detect_regions(&curves)
        .iter()
        .enumerate()
        .filter(|(_, r): &(usize, &Region)| r.contains((5.0, 5.0)))
        .map(|(i, _)| i)
        .collect();
    assert_eq!(base.len(), 1);

    let mut pg = ParametricGraph::new();
    pg.add_feature(FeatureNode {
        id: "s".to_string(),
        name: "Sketch".to_string(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes,
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    pg.add_feature(FeatureNode {
        id: "e".to_string(),
        name: "Extrude".to_string(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: base,
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    pg.add_dependency("s", "e");

    let before = pg.evaluate_bodies(&HashSet::new()).expect("eval original");
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    let loaded = read_zcad(&bytes).expect("read");
    let after = loaded
        .graph
        .evaluate_bodies(&HashSet::new())
        .expect("eval restored");

    assert_eq!(before.len(), 1, "boolean extrude makes one body");
    assert_eq!(after.len(), before.len(), "body count survives round-trip");
    assert_eq!(
        after[0].1.indices.len(),
        before[0].1.indices.len(),
        "boolean result mesh identical after reload"
    );
    assert_eq!(after[0].1.vertices.len(), before[0].1.vertices.len());
}

#[test]
fn round_trip_regular_polygon_keeps_expression_and_no_circle() {
    use zerocad_core::{CoordinateSystem, Dimension, SketchCurves, SketchShape};

    let mut pg = ParametricGraph::new();
    pg.add_feature(FeatureNode {
        id: "polygon_sketch".to_string(),
        name: "Polygon Sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: vec![SketchShape::RegularPolygon {
                center: (1.0, 2.0),
                sides: 7,
                diameter: Dimension {
                    value: 21.5,
                    expr: Some("43/2".to_string()),
                },
                rotation_deg: 15.0,
                circumscribed: false,
            }],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });

    let bytes = write_zcad(&doc_for(&pg)).expect("write polygon");
    let loaded = read_zcad(&bytes).expect("read polygon");
    let node = loaded
        .graph
        .graph
        .node_weights()
        .find(|node| node.id == "polygon_sketch")
        .expect("polygon sketch node");
    let FeatureType::Sketch { shapes, .. } = &node.feature else {
        panic!("expected sketch feature");
    };
    let SketchShape::RegularPolygon { diameter, .. } = &shapes[0] else {
        panic!("expected regular polygon");
    };
    assert_eq!(diameter.expr.as_deref(), Some("43/2"));
    let curves = shapes[0].build(&loaded.graph.variable_map());
    assert_eq!(curves.segments.len(), 7);
    assert!(curves.circles.is_empty());
}

#[test]
fn wave2_payloads_survive_real_save_to_disk_and_reload() {
    use zerocad_core::sketch::{
        EntityId, SketchEntity, SketchOffsetOperation, SketchPoint, SketchSolverModel,
    };
    use zerocad_core::{CoordinateSystem, Dimension, ExtrudeMode, SketchCurves, SketchShape};

    let offset = SketchOffsetOperation {
        id: EntityId(20),
        sources: vec![EntityId(12)],
        distance: Dimension {
            value: 2.5,
            expr: Some("wall/2".to_string()),
        },
        creation_side_seed: (4.0, 3.0),
    };
    let solver = SketchSolverModel {
        points: vec![
            SketchPoint {
                id: EntityId(10),
                pos: (0.0, 0.0),
            },
            SketchPoint {
                id: EntityId(11),
                pos: (10.0, 0.0),
            },
        ],
        entities: vec![SketchEntity::Line {
            id: EntityId(12),
            p0: EntityId(10),
            p1: EntityId(11),
            derived_from: None,
        }],
        offsets: vec![offset.clone()],
        ..SketchSolverModel::default()
    };

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "wave2_sketch".to_string(),
        name: "Slot and Offset".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: vec![SketchShape::Slot {
                start: (1.0, 2.0),
                end: (8.0, 5.0),
                width: Dimension {
                    value: 4.0,
                    expr: Some("slot_width".to_string()),
                },
            }],
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: vec![EntityId(1)],
            next_entity_id: 21,
            solver: Some(solver),
        },
    });
    graph.add_feature(FeatureNode {
        id: "wave2_extrude".to_string(),
        name: "Drafted Extrude".to_string(),
        feature: FeatureType::Extrude {
            depth: 12.0,
            region_indices: vec![0],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: Some("height".to_string()),
            draft_angle_deg: 3.25,
            draft_angle_expr: Some("draft".to_string()),
        },
    });
    graph.add_dependency("wave2_sketch", "wave2_extrude");

    let directory = tempfile::tempdir().expect("temporary directory");
    let path = directory.path().join("wave2-payloads.zcad");
    let document = Document::from_graph(graph, Unit::Millimeter);
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save Wave 2 document to disk");
    let loaded = read_document_file(&path, &LoadOptions::default())
        .expect("reload Wave 2 document from disk");

    let sketch = loaded
        .document
        .graph
        .node_weights()
        .find(|node| node.id == "wave2_sketch")
        .expect("reloaded sketch");
    // Canonical saves stamp the registry's current Sketch schema. V3 is a
    // strict superset of the Wave 2 solver/offset payload exercised below.
    assert_eq!(sketch.payload_version, 3);
    let FeatureType::Sketch { shapes, solver, .. } = &sketch.feature else {
        panic!("expected sketch feature");
    };
    assert!(matches!(
        shapes.as_slice(),
        [SketchShape::Slot { start, end, width }]
            if *start == (1.0, 2.0)
                && *end == (8.0, 5.0)
                && width.expr.as_deref() == Some("slot_width")
    ));
    let reloaded_offset = &solver.as_ref().expect("solver model").offsets[0];
    assert_eq!(reloaded_offset, &offset);

    let extrude = loaded
        .document
        .graph
        .node_weights()
        .find(|node| node.id == "wave2_extrude")
        .expect("reloaded extrude");
    assert_eq!(extrude.payload_version, 2);
    assert!(matches!(
        &extrude.feature,
        FeatureType::Extrude {
            draft_angle_deg,
            draft_angle_expr,
            ..
        } if (draft_angle_deg - 3.25).abs() < f32::EPSILON
            && draft_angle_expr.as_deref() == Some("draft")
    ));
}

#[test]
fn wave3_payloads_survive_real_save_to_disk_and_reload() {
    use zerocad_core::parametric::{
        DraftNeutral, FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind,
    };
    use zerocad_core::sketch::{
        EntityId, SketchPatternKind, SketchPatternOperation, SketchSolverModel,
    };
    use zerocad_core::{
        AxisBase, CoordinateSystem, DatumPlaneDef, Dimension, ExtrudeMode, HoleKind, SketchCurves,
    };

    let selected_face = FaceRef {
        centroid: [10.0, 0.0, 4.0],
        normal: [1.0, 0.0, 0.0],
        topology: Some(TopologyFaceRef {
            body_id: Some("draft_base".into()),
            face_id: Some("box:draft_base:face:x_max".into()),
            producer_feature_id: Some("draft_base".into()),
            ..TopologyFaceRef::default()
        }),
    };
    let neutral_face = FaceRef {
        centroid: [5.0, 5.0, 8.0],
        normal: [0.0, 0.0, 1.0],
        topology: Some(TopologyFaceRef {
            body_id: Some("datum_base".into()),
            face_id: Some("box:datum_base:face:z_max".into()),
            producer_feature_id: Some("datum_base".into()),
            ..TopologyFaceRef::default()
        }),
    };
    let pattern = SketchPatternOperation {
        id: EntityId(50),
        sources: vec![EntityId(2)],
        kind: SketchPatternKind::Linear {
            direction: [1.0, 0.0],
            spacing: Dimension {
                value: 6.0,
                expr: Some("pitch".into()),
            },
            count: 4,
        },
    };

    let mut graph = ParametricGraph::new();
    for id in ["datum_base", "draft_base", "pattern_base"] {
        graph.add_feature(FeatureNode {
            id: id.into(),
            name: id.into(),
            feature: FeatureType::Box {
                w: 20.0,
                h: 10.0,
                d: 8.0,
            },
        });
    }
    graph.add_feature(FeatureNode {
        id: "wave3_sketch".into(),
        name: "Patterned Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 51,
            solver: Some(SketchSolverModel {
                patterns: vec![pattern.clone()],
                ..SketchSolverModel::default()
            }),
        },
    });
    graph.add_feature(FeatureNode {
        id: "wave3_sweep".into(),
        name: "Twisted Sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "wave3_sketch".into(),
            profile_region: 0,
            path_sketch: "wave3_sketch".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: -90.0,
            total_twist_expr: Some("-twist".into()),
            guide: None,
        },
    });
    graph.add_dependency("wave3_sketch", "wave3_sweep");
    graph.add_feature(FeatureNode {
        id: "face_plane".into(),
        name: "Face Plane".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace {
                face: neutral_face.clone(),
            },
        },
    });
    graph.add_dependency("datum_base", "face_plane");
    graph.add_feature(FeatureNode {
        id: "hole_source".into(),
        name: "Source Hole".into(),
        feature: FeatureType::Hole {
            target: "pattern_base".into(),
            position: [5.0, 5.0, 8.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 2.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency("pattern_base", "hole_source");
    graph.add_feature(FeatureNode {
        id: "wave3_pattern".into(),
        name: "Feature Pattern".into(),
        feature: FeatureType::FeaturePattern {
            target: "pattern_base".into(),
            source_feature: "hole_source".into(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::Z,
                total_angle_deg: 270.0,
                total_angle_expr: Some("pattern_angle".into()),
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::SourceExtent,
        },
    });
    graph.add_dependency("hole_source", "wave3_pattern");
    graph.add_feature(FeatureNode {
        id: "wave3_draft".into(),
        name: "Standalone Draft".into(),
        feature: FeatureType::Draft {
            target: "draft_base".into(),
            faces: vec![selected_face.clone()],
            neutral: DraftNeutral::Datum("face_plane".into()),
            angle_deg: 3.0,
            angle_expr: Some("draft_angle".into()),
            flip_pull: true,
        },
    });
    graph.add_dependency("draft_base", "wave3_draft");
    graph.add_dependency("face_plane", "wave3_draft");

    let directory = tempfile::tempdir().expect("temporary directory");
    let path = directory.path().join("wave3-payloads.zcad");
    let document = Document::from_graph(graph, Unit::Millimeter);
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save Wave 3 document to disk");
    let loaded = read_document_file(&path, &LoadOptions::default())
        .expect("reload Wave 3 document from disk");
    let node = |id: &str| {
        loaded
            .document
            .graph
            .node_weights()
            .find(|node| node.id == id)
            .unwrap_or_else(|| panic!("reloaded {id}"))
    };

    assert_eq!(node("wave3_sketch").payload_version, 3);
    assert!(matches!(
        &node("wave3_sketch").feature,
        FeatureType::Sketch {
            solver: Some(SketchSolverModel { patterns, .. }),
            ..
        } if patterns == &vec![pattern]
    ));
    assert_eq!(node("wave3_sweep").payload_version, 3);
    assert!(matches!(
        &node("wave3_sweep").feature,
        FeatureType::Sweep {
            total_twist_deg,
            total_twist_expr: Some(expression),
            ..
        } if *total_twist_deg == -90.0 && expression == "-twist"
    ));
    assert_eq!(node("face_plane").payload_version, 2);
    assert!(matches!(
        &node("face_plane").feature,
        FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace { face }
        } if face == &neutral_face
    ));
    assert_eq!(node("wave3_pattern").payload_version, 1);
    assert!(matches!(
        &node("wave3_pattern").feature,
        FeatureType::FeaturePattern {
            kind: FeaturePatternKind::Circular { count: 4, .. },
            ..
        }
    ));
    assert_eq!(node("wave3_draft").payload_version, 1);
    assert!(matches!(
        &node("wave3_draft").feature,
        FeatureType::Draft {
            faces,
            neutral: DraftNeutral::Datum(datum),
            angle_deg,
            angle_expr: Some(expression),
            flip_pull: true,
            ..
        } if faces == &vec![selected_face]
            && datum == "face_plane"
            && *angle_deg == 3.0
            && expression == "draft_angle"
    ));
}

#[test]
fn guided_sweep_v3_survives_real_disk_save_reload_and_rebuild() {
    use zerocad_core::geometry::Vec3;
    use zerocad_core::sketch::{EntityId, SketchShape};
    use zerocad_core::{CoordinateSystem, ExtrudeMode, SketchCurves, SweepGuide};

    let mut graph = ParametricGraph::new();
    let mut profile = SketchCurves::new();
    profile.add_rectangle((-1.0, -1.0), (1.0, 1.0));
    graph.add_feature(FeatureNode {
        id: "profile".into(),
        name: "Profile".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: profile.clone(),
            shapes: vec![SketchShape::Raw { curves: profile }],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![EntityId(42)],
            next_entity_id: 43,
            solver: None,
        },
    });
    for (id, start, end) in [
        ("spine", (0.0, 0.0), (0.0, 10.0)),
        (
            "guide",
            (std::f32::consts::SQRT_2, 0.0),
            (2.0 * std::f32::consts::SQRT_2, 10.0),
        ),
    ] {
        let mut curves = SketchCurves::new();
        curves.add_line(start, end);
        graph.add_feature(FeatureNode {
            id: id.into(),
            name: id.into(),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XZ.with_origin(Vec3::ZERO),
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
    }
    graph.add_feature(FeatureNode {
        id: "guided_sweep".into(),
        name: "Guided Sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "spine".into(),
            guide: Some(SweepGuide {
                sketch: "guide".into(),
                profile_entity: EntityId(42),
                profile_parameter: 0.0,
            }),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
        },
    });
    for dependency in ["profile", "spine", "guide"] {
        graph.add_dependency(dependency, "guided_sweep");
    }
    let document = Document::from_graph(graph, Unit::Millimeter);
    let before = document
        .evaluate_bodies(&HashSet::new())
        .expect("guided Sweep before save");

    let directory = tempfile::tempdir().expect("temporary directory");
    let path = directory.path().join("guided-sweep-v3.zcad");
    write_document_file(
        &path,
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .expect("save guided Sweep to disk");
    let loaded =
        read_document_file(&path, &LoadOptions::default()).expect("reload guided Sweep from disk");
    let sweep = loaded
        .document
        .graph
        .node_weights()
        .find(|node| node.id == "guided_sweep")
        .expect("reloaded guided Sweep");
    assert_eq!(sweep.payload_version, 3);
    assert!(matches!(
        &sweep.feature,
        FeatureType::Sweep {
            guide: Some(SweepGuide {
                sketch,
                profile_entity: EntityId(42),
                profile_parameter: 0.0,
            }),
            ..
        } if sketch == "guide"
    ));
    let after = loaded
        .document
        .evaluate_bodies(&HashSet::new())
        .expect("guided Sweep after reload");
    assert_eq!(before.len(), 1);
    assert_eq!(after.len(), 1);
    assert_eq!(before[0].0, after[0].0);
    assert_eq!(before[0].1.indices, after[0].1.indices);
    assert_eq!(before[0].1.face_ids, after[0].1.face_ids);
}

#[test]
fn round_trip_body_transform_keeps_copy_and_translation() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 2.0,
            h: 3.0,
            d: 4.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "copy_2".to_string(),
        name: "Copy".to_string(),
        feature: FeatureType::BodyTransform {
            source: "box_1".to_string(),
            translation: [5.0, 1.0, -2.0],
            copy: true,
        },
    });
    graph.add_dependency("box_1", "copy_2");

    let bytes = write_zcad(&doc_for(&graph)).expect("write body transform");
    let loaded = read_zcad(&bytes).expect("read body transform");
    let node = loaded
        .graph
        .graph
        .node_weights()
        .find(|node| node.id == "copy_2")
        .expect("copy node");
    let FeatureType::BodyTransform {
        source,
        translation,
        copy,
    } = &node.feature
    else {
        panic!("expected body transform");
    };
    assert_eq!(source, "box_1");
    assert_eq!(*translation, [5.0, 1.0, -2.0]);
    assert!(*copy);
    let bodies = loaded
        .graph
        .evaluate_bodies(&HashSet::new())
        .expect("evaluate restored transform");
    assert_eq!(bodies.len(), 2);
}

#[test]
fn body_ops_features_save_load_with_equivalent_outputs_and_provenance() {
    use zerocad_core::{DatumPlaneDef, PlaneBase};

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "intersect_target".into(),
        name: "Intersect target".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "intersect_tool".into(),
        name: "Intersect tool".into(),
        feature: FeatureType::BodyTransform {
            source: "intersect_target".into(),
            translation: [5.0, 0.0, 0.0],
            copy: true,
        },
    });
    graph.add_dependency("intersect_target", "intersect_tool");
    graph.add_feature(FeatureNode {
        id: "intersect".into(),
        name: "Intersect".into(),
        feature: FeatureType::BodyIntersect {
            target: "intersect_target".into(),
            tool: "intersect_tool".into(),
            keep_tool: false,
        },
    });
    graph.add_dependency("intersect_target", "intersect");
    graph.add_dependency("intersect_tool", "intersect");

    graph.add_feature(FeatureNode {
        id: "split_target".into(),
        name: "Split target".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "split_plane".into(),
        name: "Split plane".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 5.0,
                distance_expr: None,
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "split".into(),
        name: "Split".into(),
        feature: FeatureType::BodySplit {
            target: "split_target".into(),
            plane: PlaneBase::Datum("split_plane".into()),
            face: None,
        },
    });
    graph.add_dependency("split_target", "split");
    graph.add_dependency("split_plane", "split");

    graph.add_feature(FeatureNode {
        id: "scale_target".into(),
        name: "Scale target".into(),
        feature: FeatureType::Box {
            w: 4.0,
            h: 6.0,
            d: 8.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "scale".into(),
        name: "Scale".into(),
        feature: FeatureType::BodyScale {
            source: "scale_target".into(),
            factor: 0.5,
            factor_expr: Some("1/2".into()),
            center: [2.0, 3.0, 4.0],
        },
    });
    graph.add_dependency("scale_target", "scale");

    // Sequence intent is independent of dependency order. Persist the three
    // operations ahead of their inputs and prove the DAG still governs safe
    // evaluation before and after save/load.
    for (id, sequence) in [
        ("intersect", 1),
        ("intersect_target", 2),
        ("intersect_tool", 3),
        ("split", 4),
        ("split_target", 5),
        ("split_plane", 6),
        ("scale", 7),
        ("scale_target", 8),
    ] {
        assert!(graph.set_feature_sequence(id, zerocad_core::document::SequenceKey(sequence)));
    }

    let summarize = |graph: &ParametricGraph| {
        let mut summary = graph
            .evaluate_bodies(&HashSet::new())
            .expect("evaluate Phase 3.5 graph")
            .into_iter()
            .map(|(id, mesh)| (id, mesh.mass_properties().unwrap().volume))
            .collect::<Vec<_>>();
        summary.sort_by(|left, right| left.0.cmp(&right.0));
        summary
    };
    let before = summarize(&graph);
    assert_eq!(
        graph.body_producer_feature_id("split::body:2"),
        Some("split")
    );

    let bytes = write_zcad(&doc_for(&graph)).expect("write Phase 3.5 document");
    let loaded = read_zcad(&bytes).expect("read Phase 3.5 document");
    let after = summarize(&loaded.graph);
    assert_eq!(
        loaded.graph.body_producer_feature_id("split::body:2"),
        Some("split")
    );
    assert_eq!(before.len(), after.len());
    for ((before_id, before_volume), (after_id, after_volume)) in before.iter().zip(after.iter()) {
        assert_eq!(before_id, after_id);
        assert!((before_volume - after_volume).abs() < 1.0e-5);
    }
}

#[test]
fn corrupt_payload_is_detected_not_panicked() {
    let pg = sample_graph();
    let mut bytes = write_zcad(&doc_for(&pg)).expect("write");

    // Flip a byte well past the header/table, inside a section payload.
    let last = bytes.len() - 1;
    bytes[last] ^= 0xFF;

    match read_zcad(&bytes) {
        Err(ZcadError::BadChecksum { .. }) => {}
        other => panic!("expected BadChecksum, got {other:?}"),
    }
}

#[test]
fn corrupt_header_is_detected() {
    let pg = sample_graph();
    let mut bytes = write_zcad(&doc_for(&pg)).expect("write");
    bytes[5] ^= 0xFF; // mangle the version field inside the header crc range
    match read_zcad(&bytes) {
        Err(ZcadError::BadChecksum { section: 0 }) => {}
        other => panic!("expected header BadChecksum, got {other:?}"),
    }
}

#[test]
fn truncated_file_is_an_error() {
    let pg = sample_graph();
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    let truncated = &bytes[..bytes.len() / 2];
    match read_zcad(truncated) {
        Err(ZcadError::Truncated) | Err(ZcadError::BadChecksum { .. }) => {}
        other => panic!("expected Truncated/BadChecksum, got {other:?}"),
    }
}

#[test]
fn legacy_json_is_rejected_by_the_new_contract() {
    let pg = sample_graph();
    let json = serde_json::to_string_pretty(&pg).expect("json");
    assert!(matches!(
        read_zcad(json.as_bytes()),
        Err(ZcadError::NotZcad)
    ));
}

#[test]
fn garbage_is_not_zcad() {
    for bad in [
        b"not a cad file".as_slice(),
        b"\x00\x01\x02\x03".as_slice(),
        b"".as_slice(),
    ] {
        match read_zcad(bad) {
            Err(ZcadError::NotZcad) | Err(ZcadError::Truncated) => {}
            other => panic!("expected NotZcad for {bad:?}, got {other:?}"),
        }
    }
}

#[test]
fn unknown_section_is_skipped() {
    // Write a normal file, then splice in an extra section with an unknown id so
    // an older reader (this one) must ignore it and still recover the graph.
    let pg = sample_graph();
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    let spliced = splice_unknown_section(&bytes);

    let loaded = read_zcad(&spliced).expect("read with unknown section");
    assert_eq!(
        loaded.graph.evaluate().unwrap().indices.len(),
        pg.evaluate().unwrap().indices.len()
    );
}

#[test]
fn unknown_required_section_is_rejected() {
    let pg = sample_graph();
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    let mut spliced = splice_unknown_section(&bytes);
    let count = u16::from_le_bytes(spliced[8..10].try_into().unwrap()) as usize;
    let last_entry = 32 + (count - 1) * 48;
    spliced[last_entry + 2] = 1; // required rather than disposable
    assert!(matches!(
        read_zcad(&spliced),
        Err(ZcadError::UnknownRequiredSection(0xffff))
    ));
}

#[test]
fn duplicate_and_overlapping_sections_are_rejected_before_payload_decode() {
    let bytes = write_zcad(&doc_for(&sample_graph())).unwrap();
    let mut duplicate = bytes.clone();
    let first_id = duplicate[32..34].to_vec();
    duplicate[32 + 48..32 + 48 + 2].copy_from_slice(&first_id);
    assert!(matches!(
        read_zcad(&duplicate),
        Err(ZcadError::DuplicateSection(_))
    ));

    let mut overlapping = bytes;
    let first_offset = overlapping[32 + 4..32 + 12].to_vec();
    overlapping[32 + 48 + 4..32 + 48 + 12].copy_from_slice(&first_offset);
    assert!(matches!(
        read_zcad(&overlapping),
        Err(ZcadError::OverlappingSections { .. })
    ));
}

#[test]
fn integer_overflow_and_impossible_lengths_are_rejected() {
    let bytes = write_zcad(&doc_for(&sample_graph())).unwrap();
    let mut overflow = bytes.clone();
    overflow[32 + 4..32 + 12].copy_from_slice(&u64::MAX.to_le_bytes());
    overflow[32 + 12..32 + 20].copy_from_slice(&2u64.to_le_bytes());
    assert!(matches!(read_zcad(&overflow), Err(ZcadError::Truncated)));

    let mut impossible = bytes;
    let graph_entry = section_entry(&impossible, 2).unwrap().0;
    impossible[graph_entry + 20..graph_entry + 28]
        .copy_from_slice(&(257u64 * 1024 * 1024).to_le_bytes());
    assert!(matches!(
        read_zcad(&impossible),
        Err(ZcadError::LimitExceeded {
            what: "decoded section length",
            ..
        })
    ));
}

#[test]
fn malformed_cbor_and_decompression_length_mismatch_are_rejected() {
    let bytes = write_zcad(&doc_for(&sample_graph())).unwrap();
    let mut malformed = bytes.clone();
    let (metadata_entry, metadata_offset, metadata_len) = section_entry(&malformed, 1).unwrap();
    malformed[metadata_offset..metadata_offset + metadata_len].fill(0xff);
    let digest = blake3::hash(&malformed[metadata_offset..metadata_offset + metadata_len]);
    malformed[metadata_entry + 28..metadata_entry + 44].copy_from_slice(&digest.as_bytes()[..16]);
    assert!(matches!(read_zcad(&malformed), Err(ZcadError::Decode(_))));

    let mut mismatch = bytes;
    let graph_entry = section_entry(&mismatch, 2).unwrap().0;
    mismatch[graph_entry + 20..graph_entry + 28].copy_from_slice(&1u64.to_le_bytes());
    assert!(matches!(read_zcad(&mismatch), Err(ZcadError::Decode(_))));
}

fn section_entry(bytes: &[u8], id: u16) -> Option<(usize, usize, usize)> {
    let count = u16::from_le_bytes(bytes[8..10].try_into().ok()?) as usize;
    for index in 0..count {
        let base = 32 + index * 48;
        let entry_id = u16::from_le_bytes(bytes[base..base + 2].try_into().ok()?);
        if entry_id == id {
            let offset = u64::from_le_bytes(bytes[base + 4..base + 12].try_into().ok()?) as usize;
            let len = u64::from_le_bytes(bytes[base + 12..base + 20].try_into().ok()?) as usize;
            return Some((base, offset, len));
        }
    }
    None
}

fn section_ids(bytes: &[u8]) -> Vec<u16> {
    let count = u16::from_le_bytes(bytes[8..10].try_into().unwrap()) as usize;
    (0..count)
        .map(|index| {
            let base = 32 + index * 48;
            u16::from_le_bytes(bytes[base..base + 2].try_into().unwrap())
        })
        .collect()
}

/// Insert a fabricated section with id 0xFFFF after the real sections, fixing up
/// the header section count, the section table, and appending a payload.
fn splice_unknown_section(bytes: &[u8]) -> Vec<u8> {
    const HEADER_LEN: usize = 32;
    const ENTRY_LEN: usize = 48;
    let count = u16::from_le_bytes(bytes[8..10].try_into().unwrap()) as usize;
    let table_end = HEADER_LEN + count * ENTRY_LEN;

    // The fabricated payload and where it will live (after all existing data),
    // shifted by one extra table entry that we are inserting.
    let extra = b"unknown junk payload".to_vec();
    let shift = ENTRY_LEN; // existing payloads move down by one new table row
    let new_payload_offset = bytes.len() + shift;

    let mut out = Vec::with_capacity(bytes.len() + ENTRY_LEN + extra.len());

    // Header with count+1, and a recomputed header crc over [0..12).
    out.extend_from_slice(&bytes[0..HEADER_LEN]);
    out[8..10].copy_from_slice(&((count + 1) as u16).to_le_bytes());
    let digest = blake3::hash(&out[0..12]);
    out[12..28].copy_from_slice(&digest.as_bytes()[..16]);

    // Existing table entries, with their payload offsets shifted by `shift`.
    for i in 0..count {
        let base = HEADER_LEN + i * ENTRY_LEN;
        let mut entry = bytes[base..base + ENTRY_LEN].to_vec();
        let off = u64::from_le_bytes(entry[4..12].try_into().unwrap()) + shift as u64;
        entry[4..12].copy_from_slice(&off.to_le_bytes());
        out.extend_from_slice(&entry);
    }

    // The new unknown-section table entry.
    let digest = blake3::hash(&extra);
    out.extend_from_slice(&0xFFFFu16.to_le_bytes()); // id
    out.push(2u8); // disposable
    out.push(0u8); // codec = store
    out.extend_from_slice(&(new_payload_offset as u64).to_le_bytes());
    out.extend_from_slice(&(extra.len() as u64).to_le_bytes());
    out.extend_from_slice(&(extra.len() as u64).to_le_bytes());
    out.extend_from_slice(&digest.as_bytes()[..16]);
    out.extend_from_slice(&[0u8; 4]);

    // Existing payloads (everything after the original table), then the new one.
    out.extend_from_slice(&bytes[table_end..]);
    out.extend_from_slice(&extra);
    out
}
