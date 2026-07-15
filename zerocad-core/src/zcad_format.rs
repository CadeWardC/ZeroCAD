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
//! Container extensibility comes from two independent mechanisms:
//! * **Container level** — unknown `section_id`s are skipped using their
//!   `offset`/`stored_len`, so an old reader tolerates new sections.
//! * **Payload level** — sections are self-describing CBOR. The document contract
//!   version still has to match exactly because topology semantics are authoritative.

use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::HashSet;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::mock_kernel::MockMesh;
use crate::parametric::{FaceRef, FeatureNode, ParametricGraph};
use crate::sketch::SketchCurves;
use crate::units::Unit;
use crate::EvaluationCacheSnapshot;

/// Magic bytes at the start of every binary `.zcad` file.
pub const MAGIC: &[u8; 4] = b"ZCAD";
/// Current document contract. Version 4 introduces explicit connected-component
/// identity for persistent face references and strict transactional Join
/// semantics. Readers intentionally require an exact match rather than guessing
/// across incompatible geometry semantics.
pub const CURRENT_VERSION: u16 = 4;
const DOCUMENT_RECIPE_SCHEMA: u16 = 2;

const HEADER_LEN: usize = 32;
const TABLE_ENTRY_LEN: usize = 32;

// Section ids.
const SEC_METADATA: u16 = 1;
const SEC_GRAPH: u16 = 2;
const SEC_THUMBNAIL: u16 = 3;
const SEC_MESH_CACHE: u16 = 4;
const SEC_HIDDEN_NODES: u16 = 5;
const SEC_HYDRATED_CHECKPOINTS: u16 = 6;
const HYDRATED_CACHE_SCHEMA: u16 = 2;
const OPENRCAD_CACHE_ABI: u16 = 2;
const MESH_CACHE_ABI: u16 = 3;
pub const DEFAULT_HYDRATED_CACHE_LIMIT: usize = 128 * 1024 * 1024;

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
    /// Optional reusable kernel checkpoints for `.zcadh`. These are derived,
    /// versioned accelerators and never part of the authoritative recipe.
    pub evaluation_cache: Option<&'a EvaluationCacheSnapshot>,
    /// Compressed section limit. `None` uses 128 MiB.
    pub hydrated_cache_limit: Option<usize>,
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
    /// Node ids that were hidden when the file was saved.
    pub hidden_nodes: HashSet<String>,
    pub evaluation_cache: Option<EvaluationCacheSnapshot>,
}

/// Stable, deterministic document recipe. This deliberately avoids serializing
/// petgraph's arena/index representation: node records and dependencies are
/// explicit, sorted, and independently migratable in future schema versions.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DocumentRecipeV2 {
    pub schema_version: u16,
    pub features: Vec<RecipeFeature>,
    pub dependencies: Vec<RecipeDependency>,
    pub sketch_face_refs: Vec<(String, FaceRef)>,
    pub sketch_datum_refs: Vec<(String, String)>,
    pub sketch_face_boundaries: Vec<(String, SketchCurves)>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RecipeFeature {
    pub id: String,
    pub name: String,
    pub creation_order: u64,
    pub payload_schema: u16,
    pub feature: crate::parametric::FeatureType,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct RecipeDependency {
    pub parent: String,
    pub child: String,
}

impl DocumentRecipeV2 {
    pub fn from_graph(graph: &ParametricGraph) -> Self {
        use petgraph::visit::EdgeRef as _;

        let creation_order = |id: &str| {
            id.rsplit('_')
                .next()
                .and_then(|suffix| suffix.parse::<u64>().ok())
                .unwrap_or(0)
        };
        let mut features: Vec<RecipeFeature> = graph
            .graph
            .node_indices()
            .map(|idx| {
                let node = &graph.graph[idx];
                RecipeFeature {
                    id: node.id.clone(),
                    name: node.name.clone(),
                    creation_order: creation_order(&node.id),
                    payload_schema: 1,
                    feature: node.feature.clone(),
                }
            })
            .collect();
        features.sort_by(|a, b| {
            (a.creation_order, a.id.as_str()).cmp(&(b.creation_order, b.id.as_str()))
        });

        let mut dependencies: Vec<RecipeDependency> = graph
            .graph
            .edge_references()
            .map(|edge| RecipeDependency {
                parent: graph.graph[edge.source()].id.clone(),
                child: graph.graph[edge.target()].id.clone(),
            })
            .collect();
        dependencies.sort_by(|a, b| {
            (a.parent.as_str(), a.child.as_str()).cmp(&(b.parent.as_str(), b.child.as_str()))
        });

        let sorted_pairs = |map: &std::collections::HashMap<String, String>| {
            let mut pairs: Vec<(String, String)> =
                map.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
            pairs.sort_by(|a, b| a.0.cmp(&b.0));
            pairs
        };
        let mut sketch_face_refs: Vec<(String, FaceRef)> = graph
            .sketch_face_refs
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        sketch_face_refs.sort_by(|a, b| a.0.cmp(&b.0));
        let mut sketch_face_boundaries: Vec<(String, SketchCurves)> = graph
            .sketch_face_boundaries
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        sketch_face_boundaries.sort_by(|a, b| a.0.cmp(&b.0));

        Self {
            schema_version: DOCUMENT_RECIPE_SCHEMA,
            features,
            dependencies,
            sketch_face_refs,
            sketch_datum_refs: sorted_pairs(&graph.sketch_datum_refs),
            sketch_face_boundaries,
        }
    }

    pub fn into_graph(self) -> Result<ParametricGraph, ZcadError> {
        if self.schema_version != DOCUMENT_RECIPE_SCHEMA {
            return Err(ZcadError::Decode(format!(
                "unsupported document recipe schema {}",
                self.schema_version
            )));
        }
        let mut graph = ParametricGraph::new();
        for record in self.features {
            if record.id == "origin" {
                continue;
            }
            graph.add_feature(FeatureNode {
                id: record.id,
                name: record.name,
                feature: record.feature,
            });
        }
        for dependency in self.dependencies {
            graph.add_dependency(&dependency.parent, &dependency.child);
        }
        graph.sketch_face_refs = self.sketch_face_refs.into_iter().collect();
        graph.sketch_datum_refs = self.sketch_datum_refs.into_iter().collect();
        graph.sketch_face_boundaries = self.sketch_face_boundaries.into_iter().collect();
        graph.rebuild_node_map();
        Ok(graph)
    }
}

#[derive(Debug)]
pub enum ZcadError {
    /// Not a `.zcad` file (no binary magic bytes).
    NotZcad,
    /// The file ends before a declared structure — truncated or partial write.
    Truncated,
    /// A CRC mismatch. `section` is 0 for the file header, else the section id.
    BadChecksum {
        section: u16,
    },
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
            ZcadError::BadChecksum { section: 0 } => {
                write!(f, "corrupt file header (checksum mismatch)")
            }
            ZcadError::BadChecksum { section } => {
                write!(f, "corrupt data in section {section} (checksum mismatch)")
            }
            ZcadError::UnsupportedVersion(v) => {
                write!(
                    f,
                    "file format version {v} is newer than this build supports"
                )
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
    #[serde(default)]
    mesh_cache_abi: u16,
    recipe_hash: [u8; 32],
    bodies: Vec<(String, MockMesh)>,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct HydratedCheckpointPayloadV1 {
    schema_version: u16,
    openrcad_cache_abi: u16,
    recipe_hash: [u8; 32],
    hidden_hash: [u8; 32],
    checkpoint_slots: Vec<Option<[u8; 32]>>,
    blobs: Vec<HydratedCheckpointBlob>,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct HydratedCheckpointBlob {
    hash: [u8; 32],
    cbor: Vec<u8>,
}

fn hydrated_payload(
    cache: &EvaluationCacheSnapshot,
    recipe_hash: [u8; 32],
    hidden_hash: [u8; 32],
) -> Result<HydratedCheckpointPayloadV1, ZcadError> {
    let mut checkpoint_slots = Vec::with_capacity(cache.cache.checkpoints.len());
    let mut blobs = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for checkpoint in &cache.cache.checkpoints {
        if let Some(checkpoint) = checkpoint {
            let cbor = cbor_to_vec(checkpoint)?;
            let hash = *blake3::hash(&cbor).as_bytes();
            checkpoint_slots.push(Some(hash));
            if seen.insert(hash) {
                blobs.push(HydratedCheckpointBlob { hash, cbor });
            }
        } else {
            checkpoint_slots.push(None);
        }
    }
    Ok(HydratedCheckpointPayloadV1 {
        schema_version: HYDRATED_CACHE_SCHEMA,
        openrcad_cache_abi: OPENRCAD_CACHE_ABI,
        recipe_hash,
        hidden_hash,
        checkpoint_slots,
        blobs,
    })
}

fn decode_hydrated_cache(payload: HydratedCheckpointPayloadV1) -> Option<EvaluationCacheSnapshot> {
    let mut decoded = std::collections::HashMap::new();
    for blob in payload.blobs {
        if blob.hash != *blake3::hash(&blob.cbor).as_bytes() {
            return None;
        }
        let checkpoint = cbor_from_slice(&blob.cbor).ok()?;
        decoded.insert(blob.hash, checkpoint);
    }
    let checkpoints = payload
        .checkpoint_slots
        .into_iter()
        .map(|slot| slot.and_then(|hash| decoded.get(&hash).cloned()))
        .collect();
    let snapshot = EvaluationCacheSnapshot {
        cache: std::sync::Arc::new(crate::parametric::EvalCache { checkpoints }),
    };
    let healthy = snapshot
        .cache
        .checkpoints
        .iter()
        .flatten()
        .all(|checkpoint| {
            checkpoint.live.iter().all(|body| {
                body.parts
                    .iter()
                    .all(|solid| solid.health_report().is_healthy() && solid.is_watertight())
            })
        });
    healthy.then_some(snapshot)
}

/// Restore the final checkpoint's display meshes from the separately stored
/// mesh-cache section. They are omitted from the hydrated checkpoint payload to
/// avoid storing the same large mesh twice in one document.
fn restore_final_checkpoint_meshes(
    snapshot: &mut EvaluationCacheSnapshot,
    mesh_cache: Option<&[(String, MockMesh)]>,
) {
    let Some(mesh_cache) = mesh_cache else {
        return;
    };
    let cache = std::sync::Arc::make_mut(&mut snapshot.cache);
    let Some(checkpoint) = cache.checkpoints.last_mut().and_then(Option::as_mut) else {
        return;
    };
    for body in &mut checkpoint.live {
        if body.pristine.is_none() {
            if let Some((_, mesh)) = mesh_cache.iter().find(|(id, _)| id == &body.id) {
                body.pristine = Some(std::sync::Arc::new(mesh.clone()));
            }
        }
    }
}

fn hidden_hash(hidden: &HashSet<String>) -> [u8; 32] {
    let mut ids: Vec<&str> = hidden.iter().map(String::as_str).collect();
    ids.sort_unstable();
    let mut hasher = blake3::Hasher::new();
    for id in ids {
        hasher.update(id.as_bytes());
        hasher.update(&[0]);
    }
    *hasher.finalize().as_bytes()
}

fn sparse_hydrated_cache(
    source: &EvaluationCacheSnapshot,
    mesh_cache: Option<&[(String, MockMesh)]>,
) -> EvaluationCacheSnapshot {
    let mut sparse = source.clone();
    let len = sparse.cache.checkpoints.len();
    if len > 1 {
        let mut keep = std::collections::HashSet::new();
        keep.insert(len - 1);
        for numerator in [1usize, 2, 3] {
            keep.insert((len - 1) * numerator / 4);
        }
        let mut expensive: Vec<(usize, std::time::Duration)> = sparse
            .cache
            .checkpoints
            .iter()
            .enumerate()
            .filter_map(|(i, cp)| cp.as_ref().map(|cp| (i, cp.feature_duration)))
            .collect();
        expensive.sort_by_key(|(_, duration)| std::cmp::Reverse(*duration));
        keep.extend(expensive.into_iter().take(8).map(|(i, _)| i));
        let cache = std::sync::Arc::make_mut(&mut sparse.cache);
        for (i, checkpoint) in cache.checkpoints.iter_mut().enumerate() {
            if !keep.contains(&i) {
                *checkpoint = None;
            }
        }
    }

    // The final evaluated mesh already lives in SEC_MESH_CACHE. Clear only the
    // matching final checkpoint copies; the reader reattaches them after both
    // independently versioned sections have passed freshness checks.
    if let Some(mesh_cache) = mesh_cache {
        let cache = std::sync::Arc::make_mut(&mut sparse.cache);
        if let Some(checkpoint) = cache.checkpoints.last_mut().and_then(Option::as_mut) {
            for body in &mut checkpoint.live {
                if mesh_cache.iter().any(|(id, _)| id == &body.id) {
                    body.pristine = None;
                }
            }
        }
    }
    sparse
}

/// Hash of the graph's CBOR bytes — used to tie a mesh cache to a specific graph.
///
/// FNV-1a 64-bit: a true 64-bit fingerprint (vs. a widened 32-bit CRC) so the
/// mesh-cache freshness check has negligible collision probability. This is an
/// identity fingerprint, not an integrity/tamper check — corruption is caught by
/// the per-section CRC, so a fast non-cryptographic hash is the right tool here.
fn recipe_hash(recipe_cbor: &[u8]) -> [u8; 32] {
    *blake3::hash(recipe_cbor).as_bytes()
}

#[cfg(test)]
fn graph_hash(recipe_cbor: &[u8]) -> [u8; 32] {
    recipe_hash(recipe_cbor)
}

/// Whether a mesh cache carrying `stored_hash` belongs to the graph whose CBOR
/// is `graph_cbor`. A mismatch means the graph was edited out-of-band, so the
/// cache is stale and must be discarded (the GUI re-evaluates instead).
fn mesh_cache_fresh(stored_hash: [u8; 32], recipe_cbor: &[u8]) -> bool {
    stored_hash == recipe_hash(recipe_cbor)
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
    let recipe_cbor = cbor_to_vec(&DocumentRecipeV2::from_graph(doc.graph))?;
    let recipe_hash = recipe_hash(&recipe_cbor);
    let recipe_uncompressed = recipe_cbor.len();
    let recipe_stored = zstd_compress(&recipe_cbor, GRAPH_LEVEL)?;

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
        uncompressed_len: recipe_uncompressed,
        stored: recipe_stored,
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
            mesh_cache_abi: MESH_CACHE_ABI,
            recipe_hash,
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

    // --- HYDRATED CHECKPOINTS (optional, disposable) ---
    if let Some(cache) = doc.evaluation_cache {
        let mut sparse = sparse_hydrated_cache(cache, doc.mesh_cache);
        let limit = doc
            .hydrated_cache_limit
            .unwrap_or(DEFAULT_HYDRATED_CACHE_LIMIT);
        loop {
            let payload = hydrated_payload(&sparse, recipe_hash, hidden_hash(&doc.hidden_nodes))?;
            let cbor = cbor_to_vec(&payload)?;
            let stored = zstd_compress(&cbor, MESH_LEVEL)?;
            if stored.len() <= limit {
                sections.push(StagedSection {
                    id: SEC_HYDRATED_CHECKPOINTS,
                    codec: CODEC_ZSTD,
                    uncompressed_len: cbor.len(),
                    stored,
                });
                break;
            }
            let final_index = sparse.cache.checkpoints.len().saturating_sub(1);
            let remove = sparse
                .cache
                .checkpoints
                .iter()
                .enumerate()
                .filter_map(|(i, cp)| {
                    (i != final_index)
                        .then(|| cp.as_ref().map(|cp| (i, cp.feature_duration)))
                        .flatten()
                })
                .min_by_key(|(_, duration)| *duration)
                .map(|(i, _)| i);
            match remove {
                Some(i) => std::sync::Arc::make_mut(&mut sparse.cache).checkpoints[i] = None,
                None => break, // final state alone exceeds the configured cap
            }
        }
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

/// Crash-resilient path writer used by the GUI. The new bytes are fully written
/// and synced before the previous document is moved aside; if the final rename
/// fails, the previous file is restored from the sibling backup.
pub fn write_zcad_file(path: impl AsRef<Path>, doc: &ZcadDocument) -> Result<(), ZcadError> {
    let path = path.as_ref();
    let bytes = write_zcad(doc)?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("document.zcad");
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let temp: PathBuf = parent.join(format!(
        ".{file_name}.tmp-{}-{}",
        std::process::id(),
        now_unix()
    ));
    let backup: PathBuf = parent.join(format!(".{file_name}.previous"));

    let write_result = (|| -> Result<(), std::io::Error> {
        let mut file = std::fs::File::create(&temp)?;
        file.write_all(&bytes)?;
        file.sync_all()?;

        if backup.exists() {
            std::fs::remove_file(&backup)?;
        }
        let had_previous = path.exists();
        if had_previous {
            std::fs::rename(path, &backup)?;
        }
        if let Err(error) = std::fs::rename(&temp, path) {
            if had_previous {
                let _ = std::fs::rename(&backup, path);
            }
            return Err(error);
        }
        if had_previous {
            let _ = std::fs::remove_file(&backup);
        }
        Ok(())
    })();

    if write_result.is_err() {
        let _ = std::fs::remove_file(&temp);
    }
    write_result.map_err(ZcadError::Io)
}

pub fn read_zcad_file(path: impl AsRef<Path>) -> Result<LoadedZcad, ZcadError> {
    let bytes = std::fs::read(path).map_err(ZcadError::Io)?;
    read_zcad(&bytes)
}

/// Parse a version-4 binary `.zcad` file. Earlier binary contracts and the old
/// plain-JSON prototype format are intentionally rejected.
pub fn read_zcad(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() >= 4 && &bytes[0..4] == MAGIC {
        return read_binary(bytes);
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
    if format_version != CURRENT_VERSION {
        return Err(ZcadError::UnsupportedVersion(format_version));
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
    let mut recipe: Option<DocumentRecipeV2> = None;
    let mut recipe_bytes: Option<Vec<u8>> = None;
    let mut thumbnail_png: Option<Vec<u8>> = None;
    let mut mesh_payload: Option<MeshCachePayload> = None;
    let mut hidden_nodes: HashSet<String> = HashSet::new();
    let mut hydrated_payload: Option<HydratedCheckpointPayloadV1> = None;

    for s in &sections {
        match s.id {
            SEC_METADATA => {
                let raw = decode_section(s)?;
                metadata = cbor_from_slice(&raw)?;
            }
            SEC_GRAPH => {
                let raw = decode_section(s)?;
                recipe = Some(cbor_from_slice(&raw)?);
                recipe_bytes = Some(raw);
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
            SEC_HYDRATED_CHECKPOINTS => {
                if let Ok(raw) = decode_section(s) {
                    hydrated_payload = cbor_from_slice(&raw).ok();
                }
            }
            _ => { /* unknown section — skip for forward compatibility */ }
        }
    }

    let graph = recipe
        .ok_or_else(|| ZcadError::Decode("file has no document recipe".into()))?
        .into_graph()?;

    // Keep the mesh cache only if it matches the graph we actually loaded.
    let mesh_cache = match (mesh_payload, &recipe_bytes) {
        (Some(p), Some(bytes))
            if p.mesh_cache_abi == MESH_CACHE_ABI && mesh_cache_fresh(p.recipe_hash, bytes) =>
        {
            log::debug!(
                "[zcad_cache] accepted embedded mesh cache: abi={} bodies={}",
                p.mesh_cache_abi,
                p.bodies.len()
            );
            Some(p.bodies)
        }
        (Some(p), Some(bytes)) => {
            log::debug!(
                "[zcad_cache] discarded embedded mesh cache: stored_abi={} expected_abi={} recipe_match={}",
                p.mesh_cache_abi,
                MESH_CACHE_ABI,
                mesh_cache_fresh(p.recipe_hash, bytes)
            );
            None
        }
        (Some(_), None) => {
            log::debug!(
                "[zcad_cache] discarded embedded mesh cache: document recipe bytes missing"
            );
            None
        }
        _ => None,
    };

    let evaluation_cache = match (hydrated_payload, &recipe_bytes) {
        (Some(payload), Some(bytes))
            if payload.schema_version == HYDRATED_CACHE_SCHEMA
                && payload.openrcad_cache_abi == OPENRCAD_CACHE_ABI
                && payload.recipe_hash == recipe_hash(bytes)
                && payload.hidden_hash == hidden_hash(&hidden_nodes) =>
        {
            log::debug!(
                "[zcad_cache] accepted hydrated evaluation checkpoints: abi={} slots={}",
                payload.openrcad_cache_abi,
                payload.checkpoint_slots.len()
            );
            decode_hydrated_cache(payload).map(|mut snapshot| {
                restore_final_checkpoint_meshes(&mut snapshot, mesh_cache.as_deref());
                snapshot
            })
        }
        (Some(payload), Some(bytes)) => {
            log::debug!(
                "[zcad_cache] discarded hydrated checkpoints: schema={}/{} abi={}/{} recipe_match={} hidden_match={}",
                payload.schema_version,
                HYDRATED_CACHE_SCHEMA,
                payload.openrcad_cache_abi,
                OPENRCAD_CACHE_ABI,
                payload.recipe_hash == recipe_hash(bytes),
                payload.hidden_hash == hidden_hash(&hidden_nodes)
            );
            None
        }
        (Some(_), None) => {
            log::debug!(
                "[zcad_cache] discarded hydrated checkpoints: document recipe bytes missing"
            );
            None
        }
        _ => None,
    };

    Ok(LoadedZcad {
        graph,
        metadata,
        thumbnail_png,
        mesh_cache,
        hidden_nodes,
        evaluation_cache,
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
        cbor_to_vec(&DocumentRecipeV2::from_graph(&pg)).unwrap()
    }

    #[test]
    fn graph_hash_is_deterministic_and_distinguishing() {
        let a = box_cbor(1.0);
        let a2 = box_cbor(1.0);
        let b = box_cbor(2.0);
        assert_eq!(graph_hash(&a), graph_hash(&a2), "same graph → same hash");
        assert_ne!(
            graph_hash(&a),
            graph_hash(&b),
            "different graph → different hash"
        );
    }

    #[test]
    fn fresh_only_when_hash_matches() {
        let g = box_cbor(1.0);
        assert!(
            mesh_cache_fresh(graph_hash(&g), &g),
            "matching hash → cache kept"
        );
        // A cache stamped with some other graph's hash is stale → discarded.
        let stale = graph_hash(&box_cbor(2.0));
        assert!(
            !mesh_cache_fresh(stale, &g),
            "mismatched hash → cache discarded"
        );
    }
}
