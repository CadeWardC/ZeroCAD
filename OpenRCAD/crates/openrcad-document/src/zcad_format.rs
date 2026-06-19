//! `.zcad` / `.zcadh` document container.
//!
//! `.zcad` is the lightweight form: it stores the OpenRCAD [`Document`] recipe
//! only, with cached feature results stripped. `.zcadh` uses the same container
//! and may include optional preview data such as a thumbnail and tessellated mesh
//! cache. In both cases the document recipe is authoritative.

use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

use crate::Document;

/// Magic bytes at the start of every binary `.zcad` / `.zcadh` file.
pub const MAGIC: &[u8; 4] = b"ZCAD";
/// Current container framing version.
pub const CURRENT_VERSION: u16 = 1;

const HEADER_LEN: usize = 32;
const TABLE_ENTRY_LEN: usize = 32;

const SEC_METADATA: u16 = 1;
const SEC_DOCUMENT: u16 = 2;
const SEC_THUMBNAIL: u16 = 3;
const SEC_MESH_CACHE: u16 = 4;

const CODEC_STORE: u8 = 0;

/// Document-level metadata stored first for cheap inspection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ZcadMetadata {
    /// Container framing version.
    #[serde(default)]
    pub format_version: u16,
    /// OpenRCAD crate version that wrote the file.
    #[serde(default)]
    pub app_version: String,
    /// Creation timestamp, seconds since Unix epoch.
    #[serde(default)]
    pub created_unix: u64,
    /// Last modification timestamp, seconds since Unix epoch.
    #[serde(default)]
    pub modified_unix: u64,
    /// Number of features in the document.
    #[serde(default)]
    pub feature_count: u32,
    /// Whether this file carries a display mesh cache.
    #[serde(default)]
    pub has_mesh_cache: bool,
}

impl Default for ZcadMetadata {
    fn default() -> Self {
        Self {
            format_version: CURRENT_VERSION,
            app_version: String::new(),
            created_unix: 0,
            modified_unix: 0,
            feature_count: 0,
            has_mesh_cache: false,
        }
    }
}

/// Lightweight serializable mesh cache for `.zcadh` files.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CachedMesh {
    /// Body or feature name.
    pub name: String,
    /// Vertex positions as `[x, y, z]`.
    pub vertices: Vec<[f32; 3]>,
    /// Triangle indices into [`vertices`](Self::vertices).
    pub triangles: Vec<[u32; 3]>,
    /// Optional per-triangle source face ids for picking.
    #[serde(default)]
    pub face_ids: Vec<u32>,
}

/// What callers hand to [`write_zcad`].
pub struct ZcadDocument<'a> {
    /// Authoritative parametric recipe.
    pub document: &'a Document,
    /// PNG preview bytes. Stored verbatim.
    pub thumbnail_png: Option<Vec<u8>>,
    /// Optional display cache. `None` writes lightweight `.zcad` content.
    pub mesh_cache: Option<&'a [CachedMesh]>,
    /// Creation timestamp to preserve across re-saves. `None` stamps "now".
    pub created_unix: Option<u64>,
}

/// What [`read_zcad`] returns.
#[derive(Clone, Debug)]
pub struct LoadedZcad {
    /// Authoritative document recipe. Call [`Document::recompute`] after loading
    /// when solid results are needed.
    pub document: Document,
    /// Stored metadata.
    pub metadata: ZcadMetadata,
    /// Optional PNG thumbnail.
    pub thumbnail_png: Option<Vec<u8>>,
    /// Optional display cache from `.zcadh`.
    pub mesh_cache: Option<Vec<CachedMesh>>,
    /// True when loading an old plain-JSON document instead of the binary
    /// section container.
    pub was_legacy_json: bool,
}

/// `.zcad` read/write error.
#[derive(Debug)]
pub enum ZcadError {
    /// Not a `.zcad` / `.zcadh` file.
    NotZcad,
    /// File ended before declared structures were complete.
    Truncated,
    /// Header or section checksum mismatch. Section `0` is the file header.
    BadChecksum { section: u16 },
    /// Framing version is newer than this reader can understand.
    UnsupportedVersion(u16),
    /// Payload decode failed.
    Decode(String),
    /// I/O failed.
    Io(std::io::Error),
}

impl std::fmt::Display for ZcadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotZcad => write!(f, "not a ZCAD (.zcad/.zcadh) file"),
            Self::Truncated => write!(f, "file is truncated or incomplete"),
            Self::BadChecksum { section: 0 } => {
                write!(f, "corrupt ZCAD header checksum")
            }
            Self::BadChecksum { section } => {
                write!(f, "corrupt ZCAD section {section} checksum")
            }
            Self::UnsupportedVersion(v) => {
                write!(f, "ZCAD format version {v} is newer than this reader")
            }
            Self::Decode(msg) => write!(f, "could not decode ZCAD payload: {msg}"),
            Self::Io(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for ZcadError {}

impl From<std::io::Error> for ZcadError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

struct StagedSection {
    id: u16,
    codec: u8,
    stored: Vec<u8>,
    uncompressed_len: usize,
}

struct SectionRef<'a> {
    id: u16,
    codec: u8,
    stored: &'a [u8],
    uncompressed_len: usize,
}

/// Serialize a document into the binary `.zcad` / `.zcadh` representation.
pub fn write_zcad(doc: &ZcadDocument<'_>) -> Result<Vec<u8>, ZcadError> {
    let recipe = doc.document.without_cached_results();
    let document_json =
        serde_json::to_vec(&recipe).map_err(|err| ZcadError::Decode(err.to_string()))?;

    let now = now_unix();
    let metadata = ZcadMetadata {
        format_version: CURRENT_VERSION,
        app_version: env!("CARGO_PKG_VERSION").to_string(),
        created_unix: doc.created_unix.unwrap_or(now),
        modified_unix: now,
        feature_count: recipe.features().len() as u32,
        has_mesh_cache: doc.mesh_cache.is_some(),
    };
    let metadata_json =
        serde_json::to_vec(&metadata).map_err(|err| ZcadError::Decode(err.to_string()))?;

    let mut sections = vec![
        StagedSection {
            id: SEC_METADATA,
            codec: CODEC_STORE,
            uncompressed_len: metadata_json.len(),
            stored: metadata_json,
        },
        StagedSection {
            id: SEC_DOCUMENT,
            codec: CODEC_STORE,
            uncompressed_len: document_json.len(),
            stored: document_json,
        },
    ];

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

    if let Some(cache) = doc.mesh_cache {
        let cache_json =
            serde_json::to_vec(cache).map_err(|err| ZcadError::Decode(err.to_string()))?;
        sections.push(StagedSection {
            id: SEC_MESH_CACHE,
            codec: CODEC_STORE,
            uncompressed_len: cache_json.len(),
            stored: cache_json,
        });
    }

    write_sections(&sections)
}

/// Parse a `.zcad` / `.zcadh` byte buffer.
pub fn read_zcad(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() >= MAGIC.len() && &bytes[..MAGIC.len()] == MAGIC {
        return read_binary(bytes);
    }

    if let Some(&b) = bytes.iter().find(|b| !b.is_ascii_whitespace()) {
        if b == b'{' {
            let document: Document =
                serde_json::from_slice(bytes).map_err(|err| ZcadError::Decode(err.to_string()))?;
            return Ok(LoadedZcad {
                document,
                metadata: ZcadMetadata::default(),
                thumbnail_png: None,
                mesh_cache: None,
                was_legacy_json: true,
            });
        }
    }

    Err(ZcadError::NotZcad)
}

/// Save a `.zcad` / `.zcadh` file to disk.
pub fn save_zcad(path: impl AsRef<Path>, doc: &ZcadDocument<'_>) -> Result<(), ZcadError> {
    let bytes = write_zcad(doc)?;
    std::fs::write(path, bytes)?;
    Ok(())
}

/// Load a `.zcad` / `.zcadh` file from disk.
pub fn load_zcad(path: impl AsRef<Path>) -> Result<LoadedZcad, ZcadError> {
    let bytes = std::fs::read(path)?;
    read_zcad(&bytes)
}

fn write_sections(sections: &[StagedSection]) -> Result<Vec<u8>, ZcadError> {
    if sections.len() > u8::MAX as usize {
        return Err(ZcadError::Decode("too many ZCAD sections".into()));
    }

    let table_len = sections.len() * TABLE_ENTRY_LEN;
    let mut payload_offset = HEADER_LEN + table_len;
    let mut out = Vec::with_capacity(payload_offset);

    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&CURRENT_VERSION.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.push(sections.len() as u8);
    out.extend_from_slice(&[0u8; 3]);
    debug_assert_eq!(out.len(), 12);

    let header_checksum = checksum32(&out[0..12]);
    out.extend_from_slice(&header_checksum.to_le_bytes());
    out.extend_from_slice(&[0u8; 16]);
    debug_assert_eq!(out.len(), HEADER_LEN);

    for section in sections {
        let checksum = checksum32(&section.stored);
        out.extend_from_slice(&section.id.to_le_bytes());
        out.push(section.codec);
        out.push(0u8);
        out.extend_from_slice(&(payload_offset as u64).to_le_bytes());
        out.extend_from_slice(&(section.stored.len() as u64).to_le_bytes());
        out.extend_from_slice(&(section.uncompressed_len as u64).to_le_bytes());
        out.extend_from_slice(&checksum.to_le_bytes());
        payload_offset += section.stored.len();
    }
    debug_assert_eq!(out.len(), HEADER_LEN + table_len);

    for section in sections {
        out.extend_from_slice(&section.stored);
    }

    Ok(out)
}

fn read_binary(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() < HEADER_LEN {
        return Err(ZcadError::Truncated);
    }
    if checksum32(&bytes[0..12]) != le_u32(&bytes[12..16]) {
        return Err(ZcadError::BadChecksum { section: 0 });
    }

    let version = le_u16(&bytes[4..6]);
    if version > CURRENT_VERSION {
        return Err(ZcadError::UnsupportedVersion(version));
    }

    let section_count = bytes[8] as usize;
    let table_end = HEADER_LEN + section_count * TABLE_ENTRY_LEN;
    if bytes.len() < table_end {
        return Err(ZcadError::Truncated);
    }

    let mut sections = Vec::with_capacity(section_count);
    for index in 0..section_count {
        let base = HEADER_LEN + index * TABLE_ENTRY_LEN;
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
        if checksum32(stored) != checksum {
            return Err(ZcadError::BadChecksum { section: id });
        }

        sections.push(SectionRef {
            id,
            codec,
            stored,
            uncompressed_len,
        });
    }

    let mut metadata = ZcadMetadata::default();
    let mut document = None;
    let mut thumbnail_png = None;
    let mut mesh_cache = None;

    for section in sections {
        match section.id {
            SEC_METADATA => {
                metadata = json_section(&section)?;
            }
            SEC_DOCUMENT => {
                document = Some(json_section(&section)?);
            }
            SEC_THUMBNAIL => {
                thumbnail_png = Some(decode_section(&section)?.to_vec());
            }
            SEC_MESH_CACHE => {
                mesh_cache = Some(json_section(&section)?);
            }
            _ => {}
        }
    }

    let document =
        document.ok_or_else(|| ZcadError::Decode("file has no document section".into()))?;

    Ok(LoadedZcad {
        document,
        metadata,
        thumbnail_png,
        mesh_cache,
        was_legacy_json: false,
    })
}

fn json_section<T>(section: &SectionRef<'_>) -> Result<T, ZcadError>
where
    T: for<'de> Deserialize<'de>,
{
    let bytes = decode_section(section)?;
    serde_json::from_slice(bytes).map_err(|err| ZcadError::Decode(err.to_string()))
}

fn decode_section<'a>(section: &'a SectionRef<'_>) -> Result<&'a [u8], ZcadError> {
    match section.codec {
        CODEC_STORE => {
            if section.stored.len() != section.uncompressed_len {
                return Err(ZcadError::Decode(format!(
                    "stored length {} != declared {}",
                    section.stored.len(),
                    section.uncompressed_len
                )));
            }
            Ok(section.stored)
        }
        other => Err(ZcadError::Decode(format!("unknown ZCAD codec {other}"))),
    }
}

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn checksum32(bytes: &[u8]) -> u32 {
    let mut hash = 0x811c_9dc5u32;
    for &byte in bytes {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash
}

fn le_u16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

fn le_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

fn le_u64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Operation;
    use openrcad_sketch::SketchPlane;

    fn sample_document() -> Document {
        let mut doc = Document::new();
        let sketch = doc.add_sketch("base", SketchPlane::XY);
        let rect = doc
            .sketch_mut(sketch)
            .unwrap()
            .rectangle(0.0, 0.0, 10.0, 20.0)
            .unwrap();
        doc.extrude("base", sketch, rect, 30.0, Operation::NewBody)
            .unwrap();
        doc
    }

    #[test]
    fn lightweight_zcad_round_trips_recipe_without_results() {
        let doc = sample_document();
        assert!(doc.features()[0].result().is_some());

        let bytes = write_zcad(&ZcadDocument {
            document: &doc,
            thumbnail_png: None,
            mesh_cache: None,
            created_unix: Some(1_700_000_000),
        })
        .unwrap();

        assert_eq!(&bytes[0..4], MAGIC);
        let loaded = read_zcad(&bytes).unwrap();
        assert_eq!(loaded.metadata.feature_count, 1);
        assert!(!loaded.metadata.has_mesh_cache);
        assert_eq!(loaded.metadata.created_unix, 1_700_000_000);
        assert!(loaded.document.features()[0].result().is_none());

        let mut recomputed = loaded.document;
        recomputed.recompute().unwrap();
        assert!(recomputed.features()[0].result().is_some());
    }

    #[test]
    fn zcadh_round_trips_thumbnail_and_mesh_cache() {
        let doc = sample_document();
        let thumbnail = vec![0x89, b'P', b'N', b'G'];
        let cache = vec![CachedMesh {
            name: "base".to_string(),
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            triangles: vec![[0, 1, 2]],
            face_ids: vec![0],
        }];

        let bytes = write_zcad(&ZcadDocument {
            document: &doc,
            thumbnail_png: Some(thumbnail.clone()),
            mesh_cache: Some(&cache),
            created_unix: None,
        })
        .unwrap();
        let loaded = read_zcad(&bytes).unwrap();

        assert!(loaded.metadata.has_mesh_cache);
        assert_eq!(loaded.thumbnail_png.as_deref(), Some(thumbnail.as_slice()));
        assert_eq!(loaded.mesh_cache.as_deref(), Some(cache.as_slice()));
    }

    #[test]
    fn corrupt_payload_is_detected() {
        let doc = sample_document();
        let mut bytes = write_zcad(&ZcadDocument {
            document: &doc,
            thumbnail_png: None,
            mesh_cache: None,
            created_unix: None,
        })
        .unwrap();

        let last = bytes.len() - 1;
        bytes[last] ^= 0xff;

        assert!(matches!(
            read_zcad(&bytes),
            Err(ZcadError::BadChecksum { .. })
        ));
    }

    #[test]
    fn legacy_plain_json_document_still_loads() {
        let doc = sample_document().without_cached_results();
        let json = serde_json::to_vec(&doc).unwrap();
        let loaded = read_zcad(&json).unwrap();

        assert!(loaded.was_legacy_json);
        assert_eq!(loaded.document.features().len(), 1);
    }
}
