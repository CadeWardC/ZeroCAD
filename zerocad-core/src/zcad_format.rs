//! The `.zcad` file format — a versioned, compressed, self-contained container
//! for a ZeroCAD document.
//!
//! A `.zcad` file is a small binary container:
//!
//! ```text
//! FILE HEADER (32 bytes, never compressed)
//!   0   4   magic           = b"ZCAD"
//!   4   2   format_version  u16  = CURRENT_VERSION
//!   6   2   container_flags u16  (reserved)
//!   8   1   section_count   u8
//!   9   3   reserved        = 0
//!   12  4   header_crc32    u32  crc32 of bytes [0..12)
//!   16  16  reserved padding= 0
//!
//! SECTION TABLE (section_count × 32 bytes, never compressed)
//!   0   2   section_id        u16
//!   2   1   codec             u8    (0 = store, 1 = zstd)
//!   3   1   flags             u8
//!   4   8   offset            u64   absolute file offset of payload
//!   12  8   stored_len        u64   on-disk length (compressed if zstd)
//!   20  8   uncompressed_len  u64   length after decompression
//!   28  4   checksum          u32   crc32 of the stored (on-disk) bytes
//!
//! PAYLOADS: concatenated after the table, in table order.
//! ```
//!
//! The parametric [`ParametricGraph`] is the **source of truth** — re-evaluating
//! it regenerates all geometry. The thumbnail and the optional mesh cache are
//! conveniences: a self-contained preview and an instant-open / fallback render.
//!
//! Forward compatibility comes from two independent mechanisms:
//! * **Container level** — unknown `section_id`s are skipped using their
//!   `offset`/`stored_len`, so an old reader tolerates new sections.
//! * **Payload level** — sections are CBOR (self-describing), so `#[serde(default)]`
//!   fields decode exactly as they do on the legacy JSON path.

use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::HashSet;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::mock_kernel::MockMesh;
use crate::parametric::ParametricGraph;
use crate::units::Unit;

/// Magic bytes at the start of every binary `.zcad` file.
pub const MAGIC: &[u8; 4] = b"ZCAD";
/// Current container framing version. Bumped only when the header/table/section
/// framing changes — never for payload schema changes (those are absorbed by
/// serde `#[serde(default)]`).
pub const CURRENT_VERSION: u16 = 2;

const HEADER_LEN: usize = 32;
const TABLE_ENTRY_LEN: usize = 32;

// Section ids.
const SEC_METADATA: u16 = 1;
const SEC_GRAPH: u16 = 2;
const SEC_THUMBNAIL: u16 = 3;
const SEC_MESH_CACHE: u16 = 4;
const SEC_HIDDEN_NODES: u16 = 5;

// Codecs.
const CODEC_STORE: u8 = 0;
const CODEC_ZSTD: u8 = 1;

// zstd levels. The recipe is small, so level 9 already gives a size within a
// hair of the maximum while compressing ~10-20x faster than level 19 on large
// assemblies. The mesh cache is the bulky section, kept fast at level 3.
const GRAPH_LEVEL: i32 = 9;
const MESH_LEVEL: i32 = 3;

/// Document-level metadata, stored uncompressed and first so a file browser can
/// read it without inflating the rest of the file.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct ZcadMetadata {
    #[serde(default)]
    pub format_version: u16,
    #[serde(default)]
    pub app_version: String,
    #[serde(default)]
    pub created_unix: u64,
    #[serde(default)]
    pub modified_unix: u64,
    #[serde(default)]
    pub units: Unit,
    #[serde(default)]
    pub feature_count: u32,
    #[serde(default)]
    pub bbox: [f32; 6],
}

/// What the caller hands to [`write_zcad`].
pub struct ZcadDocument<'a> {
    /// The parametric recipe — always written, always authoritative.
    pub graph: &'a ParametricGraph,
    /// PNG-encoded preview. Stored verbatim (PNG is already compressed).
    pub thumbnail_png: Option<Vec<u8>>,
    /// Evaluated body meshes. `None` produces a "lightweight" file that
    /// regenerates geometry on open.
    pub mesh_cache: Option<&'a [(String, MockMesh)]>,
    pub units: Unit,
    pub bbox: [f32; 6],
    /// Creation timestamp to preserve across re-saves. `None` stamps "now".
    pub created_unix: Option<u64>,
    /// Node ids the user has hidden in the feature tree. Persisted so visibility
    /// state survives save/open.
    pub hidden_nodes: HashSet<String>,
}

/// What [`read_zcad`] returns. `graph` is authoritative; everything else is
/// best-effort and may be absent.
#[derive(Debug)]
pub struct LoadedZcad {
    pub graph: ParametricGraph,
    pub metadata: ZcadMetadata,
    pub thumbnail_png: Option<Vec<u8>>,
    /// Present and fresh only when the embedded cache's `graph_hash` matches the
    /// graph that was loaded; a stale cache is discarded (left `None`).
    pub mesh_cache: Option<Vec<(String, MockMesh)>>,
    /// True when the file was an old plain-JSON `.zcad` loaded via the legacy path.
    pub was_legacy_json: bool,
    /// Node ids that were hidden when the file was saved.
    pub hidden_nodes: HashSet<String>,
}

#[derive(Debug)]
pub enum ZcadError {
    /// Not a `.zcad` file (no magic bytes and not legacy JSON).
    NotZcad,
    /// The file ends before a declared structure — truncated or partial write.
    Truncated,
    /// A CRC mismatch. `section` is 0 for the file header, else the section id.
    BadChecksum { section: u16 },
    /// The framing version is newer than this build can read.
    UnsupportedVersion(u16),
    /// A payload failed to decode (bad CBOR, bad zstd stream, missing graph).
    Decode(String),
    Io(std::io::Error),
}

impl std::fmt::Display for ZcadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ZcadError::NotZcad => write!(f, "not a ZeroCAD (.zcad) file"),
            ZcadError::Truncated => write!(f, "file is truncated or incomplete"),
            ZcadError::BadChecksum { section: 0 } => write!(f, "corrupt file header (checksum mismatch)"),
            ZcadError::BadChecksum { section } => {
                write!(f, "corrupt data in section {section} (checksum mismatch)")
            }
            ZcadError::UnsupportedVersion(v) => {
                write!(f, "file format version {v} is newer than this build supports")
            }
            ZcadError::Decode(msg) => write!(f, "could not decode file: {msg}"),
            ZcadError::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

impl std::error::Error for ZcadError {}

/// Mesh cache payload: the evaluated bodies plus a hash of the graph they were
/// derived from, so a cache made stale by an out-of-band edit can be discarded.
#[derive(serde::Serialize, serde::Deserialize)]
struct MeshCachePayload {
    graph_hash: u64,
    bodies: Vec<(String, MockMesh)>,
}

/// Hash of the graph's CBOR bytes — used to tie a mesh cache to a specific graph.
///
/// FNV-1a 64-bit: a true 64-bit fingerprint (vs. a widened 32-bit CRC) so the
/// mesh-cache freshness check has negligible collision probability. This is an
/// identity fingerprint, not an integrity/tamper check — corruption is caught by
/// the per-section CRC, so a fast non-cryptographic hash is the right tool here.
fn graph_hash(graph_cbor: &[u8]) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut h = OFFSET;
    for &b in graph_cbor {
        h ^= b as u64;
        h = h.wrapping_mul(PRIME);
    }
    h
}

/// Whether a mesh cache carrying `stored_hash` belongs to the graph whose CBOR
/// is `graph_cbor`. A mismatch means the graph was edited out-of-band, so the
/// cache is stale and must be discarded (the GUI re-evaluates instead).
fn mesh_cache_fresh(stored_hash: u64, graph_cbor: &[u8]) -> bool {
    stored_hash == graph_hash(graph_cbor)
}

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn cbor_to_vec<T: Serialize>(v: &T) -> Result<Vec<u8>, ZcadError> {
    let mut out = Vec::new();
    ciborium::into_writer(v, &mut out).map_err(|e| ZcadError::Decode(e.to_string()))?;
    Ok(out)
}

fn cbor_from_slice<T: DeserializeOwned>(b: &[u8]) -> Result<T, ZcadError> {
    ciborium::from_reader(b).map_err(|e| ZcadError::Decode(e.to_string()))
}

fn zstd_compress(data: &[u8], level: i32) -> Result<Vec<u8>, ZcadError> {
    zstd::encode_all(data, level).map_err(ZcadError::Io)
}

fn zstd_decompress(data: &[u8], expected_len: usize) -> Result<Vec<u8>, ZcadError> {
    let out = zstd::decode_all(data).map_err(ZcadError::Io)?;
    if out.len() != expected_len {
        return Err(ZcadError::Decode(format!(
            "decompressed length {} != declared {expected_len}",
            out.len()
        )));
    }
    Ok(out)
}

/// One section, ready to be laid out in the file.
struct StagedSection {
    id: u16,
    codec: u8,
    /// Bytes as they go on disk (compressed if `codec == CODEC_ZSTD`).
    stored: Vec<u8>,
    /// Length after decompression (== `stored.len()` when stored).
    uncompressed_len: usize,
}

/// Serialize a [`ZcadDocument`] into the binary `.zcad` representation.
pub fn write_zcad(doc: &ZcadDocument) -> Result<Vec<u8>, ZcadError> {
    let mut sections: Vec<StagedSection> = Vec::new();

    // --- GRAPH (source of truth) ---
    let graph_cbor = cbor_to_vec(doc.graph)?;
    let graph_hash = graph_hash(&graph_cbor);
    let graph_uncompressed = graph_cbor.len();
    let graph_stored = zstd_compress(&graph_cbor, GRAPH_LEVEL)?;

    // --- METADATA (uncompressed, written first) ---
    let now = now_unix();
    let meta = ZcadMetadata {
        format_version: CURRENT_VERSION,
        app_version: env!("CARGO_PKG_VERSION").to_string(),
        created_unix: doc.created_unix.unwrap_or(now),
        modified_unix: now,
        units: doc.units,
        feature_count: doc.graph.graph.node_count() as u32,
        bbox: doc.bbox,
    };
    let meta_cbor = cbor_to_vec(&meta)?;
    sections.push(StagedSection {
        id: SEC_METADATA,
        codec: CODEC_STORE,
        uncompressed_len: meta_cbor.len(),
        stored: meta_cbor,
    });

    sections.push(StagedSection {
        id: SEC_GRAPH,
        codec: CODEC_ZSTD,
        uncompressed_len: graph_uncompressed,
        stored: graph_stored,
    });

    // --- THUMBNAIL (stored; PNG is already compressed) ---
    if let Some(png) = &doc.thumbnail_png {
        if !png.is_empty() {
            sections.push(StagedSection {
                id: SEC_THUMBNAIL,
                codec: CODEC_STORE,
                uncompressed_len: png.len(),
                stored: png.clone(),
            });
        }
    }

    // --- MESH_CACHE (optional, zstd) ---
    if let Some(bodies) = doc.mesh_cache {
        let payload = MeshCachePayload {
            graph_hash,
            bodies: bodies.to_vec(),
        };
        let cbor = cbor_to_vec(&payload)?;
        let uncompressed_len = cbor.len();
        let stored = zstd_compress(&cbor, MESH_LEVEL)?;
        sections.push(StagedSection {
            id: SEC_MESH_CACHE,
            codec: CODEC_ZSTD,
            uncompressed_len,
            stored,
        });
    }

    // --- HIDDEN_NODES (optional, zstd) ---
    if !doc.hidden_nodes.is_empty() {
        let cbor = cbor_to_vec(&doc.hidden_nodes)?;
        let uncompressed_len = cbor.len();
        let stored = zstd_compress(&cbor, GRAPH_LEVEL)?;
        sections.push(StagedSection {
            id: SEC_HIDDEN_NODES,
            codec: CODEC_ZSTD,
            uncompressed_len,
            stored,
        });
    }

    // --- Lay out the file ---
    let section_count = sections.len();
    let table_len = section_count * TABLE_ENTRY_LEN;
    let mut payload_offset = HEADER_LEN + table_len;

    let mut out = Vec::new();

    // Header bytes [0..12) — magic, version, flags, count, reserved.
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&CURRENT_VERSION.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes()); // container_flags
    out.push(section_count as u8);
    out.extend_from_slice(&[0u8; 3]); // reserved
    debug_assert_eq!(out.len(), 12);
    // header_crc32 over [0..12), then 16 bytes reserved padding.
    let header_crc = crc32fast::hash(&out[0..12]);
    out.extend_from_slice(&header_crc.to_le_bytes());
    out.extend_from_slice(&[0u8; 16]);
    debug_assert_eq!(out.len(), HEADER_LEN);

    // Section table.
    for s in &sections {
        let checksum = crc32fast::hash(&s.stored);
        out.extend_from_slice(&s.id.to_le_bytes());
        out.push(s.codec);
        out.push(0u8); // per-section flags
        out.extend_from_slice(&(payload_offset as u64).to_le_bytes());
        out.extend_from_slice(&(s.stored.len() as u64).to_le_bytes());
        out.extend_from_slice(&(s.uncompressed_len as u64).to_le_bytes());
        out.extend_from_slice(&checksum.to_le_bytes());
        payload_offset += s.stored.len();
    }
    debug_assert_eq!(out.len(), HEADER_LEN + table_len);

    // Payloads.
    for s in &sections {
        out.extend_from_slice(&s.stored);
    }

    Ok(out)
}

/// Parse a `.zcad` file. Accepts the binary container, an old plain-JSON
/// `.zcad`, and rejects anything else with [`ZcadError::NotZcad`].
pub fn read_zcad(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() >= 4 && &bytes[0..4] == MAGIC {
        return read_binary(bytes);
    }
    // Legacy plain-JSON `.zcad` files start with `{` (after optional whitespace).
    if let Some(&b) = bytes.iter().find(|b| !b.is_ascii_whitespace()) {
        if b == b'{' {
            let graph: ParametricGraph =
                serde_json::from_slice(bytes).map_err(|e| ZcadError::Decode(e.to_string()))?;
            return Ok(LoadedZcad {
                graph,
                metadata: ZcadMetadata::default(),
                thumbnail_png: None,
                mesh_cache: None,
                was_legacy_json: true,
                hidden_nodes: HashSet::new(),
            });
        }
    }
    Err(ZcadError::NotZcad)
}

fn le_u16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}
fn le_u32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}
fn le_u64(b: &[u8]) -> u64 {
    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
}

struct SectionRef {
    id: u16,
    codec: u8,
    stored: Vec<u8>,
    uncompressed_len: usize,
}

fn read_binary(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() < HEADER_LEN {
        return Err(ZcadError::Truncated);
    }
    // Verify header checksum before trusting any header field.
    let header_crc = le_u32(&bytes[12..16]);
    if crc32fast::hash(&bytes[0..12]) != header_crc {
        return Err(ZcadError::BadChecksum { section: 0 });
    }
    let format_version = le_u16(&bytes[4..6]);
    if format_version > CURRENT_VERSION {
        // Best-effort: a strictly-newer framing may not be parseable. We attempt
        // it anyway (unknown sections are skipped), but if the table doesn't fit
        // we surface the version rather than a confusing truncation error.
        if bytes.len() < HEADER_LEN + (bytes[8] as usize) * TABLE_ENTRY_LEN {
            return Err(ZcadError::UnsupportedVersion(format_version));
        }
    }
    let section_count = bytes[8] as usize;
    let table_end = HEADER_LEN + section_count * TABLE_ENTRY_LEN;
    if bytes.len() < table_end {
        return Err(ZcadError::Truncated);
    }

    let mut sections: Vec<SectionRef> = Vec::with_capacity(section_count);
    for i in 0..section_count {
        let base = HEADER_LEN + i * TABLE_ENTRY_LEN;
        let entry = &bytes[base..base + TABLE_ENTRY_LEN];
        let id = le_u16(&entry[0..2]);
        let codec = entry[2];
        let offset = le_u64(&entry[4..12]) as usize;
        let stored_len = le_u64(&entry[12..20]) as usize;
        let uncompressed_len = le_u64(&entry[20..28]) as usize;
        let checksum = le_u32(&entry[28..32]);

        let end = offset.checked_add(stored_len).ok_or(ZcadError::Truncated)?;
        if offset < table_end || end > bytes.len() {
            return Err(ZcadError::Truncated);
        }
        let stored = &bytes[offset..end];
        if crc32fast::hash(stored) != checksum {
            return Err(ZcadError::BadChecksum { section: id });
        }
        sections.push(SectionRef {
            id,
            codec,
            stored: stored.to_vec(),
            uncompressed_len,
        });
    }

    // Decode each section into its slot. Unknown ids are skipped silently.
    let mut metadata = ZcadMetadata::default();
    let mut graph: Option<ParametricGraph> = None;
    let mut graph_bytes: Option<Vec<u8>> = None;
    let mut thumbnail_png: Option<Vec<u8>> = None;
    let mut mesh_payload: Option<MeshCachePayload> = None;
    let mut hidden_nodes: HashSet<String> = HashSet::new();

    for s in &sections {
        match s.id {
            SEC_METADATA => {
                let raw = decode_section(s)?;
                metadata = cbor_from_slice(&raw)?;
            }
            SEC_GRAPH => {
                let raw = decode_section(s)?;
                graph = Some(cbor_from_slice(&raw)?);
                graph_bytes = Some(raw);
            }
            SEC_THUMBNAIL => {
                thumbnail_png = Some(decode_section(s)?);
            }
            SEC_MESH_CACHE => {
                let raw = decode_section(s)?;
                mesh_payload = Some(cbor_from_slice(&raw)?);
            }
            SEC_HIDDEN_NODES => {
                let raw = decode_section(s)?;
                hidden_nodes = cbor_from_slice(&raw)?;
            }
            _ => { /* unknown section — skip for forward compatibility */ }
        }
    }

    let graph = graph.ok_or_else(|| ZcadError::Decode("file has no graph section".into()))?;

    // Keep the mesh cache only if it matches the graph we actually loaded.
    let mesh_cache = match (mesh_payload, &graph_bytes) {
        (Some(p), Some(gb)) if mesh_cache_fresh(p.graph_hash, gb) => Some(p.bodies),
        _ => None,
    };

    Ok(LoadedZcad {
        graph,
        metadata,
        thumbnail_png,
        mesh_cache,
        was_legacy_json: false,
        hidden_nodes,
    })
}

fn decode_section(s: &SectionRef) -> Result<Vec<u8>, ZcadError> {
    match s.codec {
        CODEC_STORE => Ok(s.stored.clone()),
        CODEC_ZSTD => zstd_decompress(&s.stored, s.uncompressed_len),
        other => Err(ZcadError::Decode(format!("unknown codec {other}"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parametric::{FeatureNode, FeatureType};

    fn box_cbor(w: f32) -> Vec<u8> {
        let mut pg = ParametricGraph::new();
        pg.add_feature(FeatureNode {
            id: "b".into(),
            name: "B".into(),
            feature: FeatureType::Box { w, h: 1.0, d: 1.0 },
        });
        cbor_to_vec(&pg).unwrap()
    }

    #[test]
    fn graph_hash_is_deterministic_and_distinguishing() {
        let a = box_cbor(1.0);
        let a2 = box_cbor(1.0);
        let b = box_cbor(2.0);
        assert_eq!(graph_hash(&a), graph_hash(&a2), "same graph → same hash");
        assert_ne!(graph_hash(&a), graph_hash(&b), "different graph → different hash");
    }

    #[test]
    fn fresh_only_when_hash_matches() {
        let g = box_cbor(1.0);
        assert!(mesh_cache_fresh(graph_hash(&g), &g), "matching hash → cache kept");
        // A cache stamped with some other graph's hash is stale → discarded.
        let stale = graph_hash(&box_cbor(2.0));
        assert!(!mesh_cache_fresh(stale, &g), "mismatched hash → cache discarded");
    }
}
