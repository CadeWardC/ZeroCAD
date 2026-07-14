//! `.zcad` container guarantees: a document round-trips through the binary
//! format, corruption is detected (never a panic), incompatible prototype files
//! are rejected, and unknown container sections remain safely skippable.

use std::collections::HashSet;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::zcad_format::{
    read_zcad, read_zcad_file, write_zcad, write_zcad_file, ZcadDocument, ZcadError,
    CURRENT_VERSION, MAGIC,
};
use zerocad_core::{FeatureNode, FeatureType, ParametricGraph, Unit};

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
fn connected_component_face_identity_round_trips() {
    let mut pg = sample_graph();
    pg.sketch_face_refs.insert(
        "attached_sketch".to_string(),
        FaceRef {
            centroid: [4.0, 5.0, 6.0],
            normal: [1.0, 0.0, 0.0],
            topology: Some(TopologyFaceRef {
                body_id: Some("box1".to_string()),
                component_id: Some("0:0:0:400000:200000:300000".to_string()),
                topology_version: Some(0),
                face_id: Some("box_box1:face:+x".to_string()),
                surface_kind: Some("plane".to_string()),
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
    let checksum = crc32fast::hash(&bytes[0..12]);
    bytes[12..16].copy_from_slice(&checksum.to_le_bytes());

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

/// Insert a fabricated section with id 0xFFFF after the real sections, fixing up
/// the header section count, the section table, and appending a payload.
fn splice_unknown_section(bytes: &[u8]) -> Vec<u8> {
    const HEADER_LEN: usize = 32;
    const ENTRY_LEN: usize = 32;
    let count = bytes[8] as usize;
    let table_end = HEADER_LEN + count * ENTRY_LEN;

    // The fabricated payload and where it will live (after all existing data),
    // shifted by one extra table entry that we are inserting.
    let extra = b"unknown junk payload".to_vec();
    let shift = ENTRY_LEN; // existing payloads move down by one new table row
    let new_payload_offset = bytes.len() + shift;

    let mut out = Vec::with_capacity(bytes.len() + ENTRY_LEN + extra.len());

    // Header with count+1, and a recomputed header crc over [0..12).
    out.extend_from_slice(&bytes[0..HEADER_LEN]);
    out[8] = (count + 1) as u8;
    let crc = crc32fast::hash(&out[0..12]);
    out[12..16].copy_from_slice(&crc.to_le_bytes());

    // Existing table entries, with their payload offsets shifted by `shift`.
    for i in 0..count {
        let base = HEADER_LEN + i * ENTRY_LEN;
        let mut entry = bytes[base..base + ENTRY_LEN].to_vec();
        let off = u64::from_le_bytes(entry[4..12].try_into().unwrap()) + shift as u64;
        entry[4..12].copy_from_slice(&off.to_le_bytes());
        out.extend_from_slice(&entry);
    }

    // The new unknown-section table entry.
    let checksum = crc32fast::hash(&extra);
    out.extend_from_slice(&0xFFFFu16.to_le_bytes()); // id
    out.push(0u8); // codec = store
    out.push(0u8); // flags
    out.extend_from_slice(&(new_payload_offset as u64).to_le_bytes());
    out.extend_from_slice(&(extra.len() as u64).to_le_bytes());
    out.extend_from_slice(&(extra.len() as u64).to_le_bytes());
    out.extend_from_slice(&checksum.to_le_bytes());

    // Existing payloads (everything after the original table), then the new one.
    out.extend_from_slice(&bytes[table_end..]);
    out.extend_from_slice(&extra);
    out
}
