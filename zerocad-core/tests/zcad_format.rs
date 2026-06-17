//! `.zcad` container guarantees: a document round-trips through the binary
//! format, corruption is detected (never a panic), old plain-JSON files still
//! load, and unknown sections/fields are tolerated for forward compatibility.

use std::collections::HashSet;
use zerocad_core::zcad_format::{read_zcad, write_zcad, ZcadDocument, ZcadError, MAGIC};
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
    }
}

#[test]
fn round_trip_recipe_only() {
    let pg = sample_graph();
    let bytes = write_zcad(&doc_for(&pg)).expect("write");
    assert_eq!(&bytes[0..4], MAGIC, "file must start with the magic bytes");

    let loaded = read_zcad(&bytes).expect("read");
    assert!(!loaded.was_legacy_json);
    assert!(loaded.mesh_cache.is_none());
    assert_eq!(loaded.metadata.feature_count, pg.graph.node_count() as u32);

    // The restored graph must evaluate to the same geometry.
    let before = pg.evaluate().expect("eval original");
    let after = loaded.graph.evaluate().expect("eval restored");
    assert_eq!(before.indices.len(), after.indices.len());
    assert_eq!(before.vertices.len(), after.vertices.len());
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
fn legacy_json_still_loads() {
    let pg = sample_graph();
    let json = serde_json::to_string_pretty(&pg).expect("json");
    let loaded = read_zcad(json.as_bytes()).expect("read legacy");
    assert!(loaded.was_legacy_json);
    assert_eq!(
        loaded.graph.evaluate().unwrap().indices.len(),
        pg.evaluate().unwrap().indices.len()
    );
}

#[test]
fn garbage_is_not_zcad() {
    for bad in [b"not a cad file".as_slice(), b"\x00\x01\x02\x03".as_slice(), b"".as_slice()] {
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
