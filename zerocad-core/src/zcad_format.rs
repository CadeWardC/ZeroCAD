//! The `.zcad` file format — a versioned, compressed, self-contained container
//! for a ZeroCAD document.
//!
//! A `.zcad` file is a small binary container. Compact `.zcad` and hydrated
//! `.zcadh` documents share this framing; the embedded profile is authoritative.
//!
//! ```text
//! FILE HEADER (32 bytes, never compressed)
//!   0   4   magic           = b"ZCAD"
//!   4   2   format_version  u16  = CURRENT_VERSION
//!   6   1   save_profile    u8   (0 = compact, 1 = hydrated)
//!   7   1   container_flags u8   (reserved)
//!   8   2   section_count   u16
//!   10  2   reserved        = 0
//!   12  16  header_digest   truncated BLAKE3 of bytes [0..12)
//!   28  4   reserved        = 0
//!
//! SECTION TABLE (section_count × 48 bytes, never compressed)
//!   0   2   section_id        u16
//!   2   1   flags             u8    (required or disposable)
//!   3   1   codec             u8    (0 = store, 1 = zstd)
//!   4   8   offset            u64   absolute file offset of payload
//!   12  8   stored_len        u64   on-disk length (compressed if zstd)
//!   20  8   uncompressed_len  u64   length after decompression
//!   28  16  payload_digest    truncated BLAKE3 of stored bytes
//!   44  4   reserved          = 0
//!
//! PAYLOADS: concatenated after the table, in table order.
//! ```
//!
//! The parametric [`ParametricGraph`] is the **source of truth** — re-evaluating
//! it regenerates all geometry. The thumbnail and the optional mesh cache are
//! conveniences: a self-contained preview and validated instant-open data.
//!
//! Container extensibility comes from two independent mechanisms:
//! * **Container level** — unknown disposable `section_id`s are skipped using
//!   their `offset`/`stored_len`; unknown required sections fail clearly.
//! * **Payload level** — sections are self-describing CBOR. The document contract
//!   version still has to match exactly because topology semantics are authoritative.

use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::{BTreeMap, HashSet};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::document::FeatureId;
use crate::mock_kernel::MockMesh;
use crate::parametric::{FaceRef, FeatureNode, ParametricGraph};
use crate::sketch::SketchCurves;
use crate::units::Unit;
use crate::EvaluationCacheSnapshot;

/// Magic bytes at the start of every binary `.zcad` file.
pub const MAGIC: &[u8; 4] = b"ZCAD";
/// The one deliberately incompatible Part Design document reset.
pub const CURRENT_VERSION: u16 = 5;
const DOCUMENT_RECIPE_SCHEMA: u16 = 3;
const REQUIRED_ASSETS_SCHEMA: u16 = 1;
const FEATURE_PAYLOAD_ABI: u16 = 1;
const TOLERANCE_POLICY_ABI: u16 = 1;

const HEADER_LEN: usize = 32;
const TABLE_ENTRY_LEN: usize = 48;

const SECTION_REQUIRED: u8 = 1 << 0;
const SECTION_DISPOSABLE: u8 = 1 << 1;
const MAX_TINY_PREVIEW_BYTES: usize = 32 * 1024;

// Section ids.
const SEC_METADATA: u16 = 1;
const SEC_GRAPH: u16 = 2;
const SEC_THUMBNAIL: u16 = 3;
const SEC_MESH_CACHE: u16 = 4;
const SEC_HIDDEN_NODES: u16 = 5;
const SEC_HYDRATED_CHECKPOINTS: u16 = 6;
const SEC_LARGE_PREVIEW: u16 = 7;
const SEC_REQUIRED_ASSETS: u16 = 8;
const HYDRATED_CACHE_SCHEMA: u16 = 3;
const OPENRCAD_CACHE_ABI: u16 = 2;
const MESH_CACHE_ABI: u16 = 3;
pub const DEFAULT_HYDRATED_CACHE_LIMIT: usize = 128 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SaveProfile {
    Compact,
    Hydrated { total_accelerator_budget: u64 },
}

impl SaveProfile {
    fn header_tag(self) -> u8 {
        match self {
            Self::Compact => 0,
            Self::Hydrated { .. } => 1,
        }
    }

    fn from_header(tag: u8, budget: u64) -> Result<Self, ZcadError> {
        match tag {
            0 => Ok(Self::Compact),
            1 => Ok(Self::Hydrated {
                total_accelerator_budget: budget,
            }),
            other => Err(ZcadError::Decode(format!("unknown save profile {other}"))),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SaveOptions {
    pub profile: SaveProfile,
}

impl Default for SaveOptions {
    fn default() -> Self {
        Self {
            profile: SaveProfile::Compact,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadLimits {
    pub max_sections: u16,
    pub max_manifest_bytes: u64,
    pub max_recipe_bytes: u64,
    pub max_required_assets_bytes: u64,
    pub max_accelerator_bytes: u64,
}

impl Default for LoadLimits {
    fn default() -> Self {
        Self {
            max_sections: 64,
            max_manifest_bytes: 64 * 1024,
            max_recipe_bytes: 256 * 1024 * 1024,
            max_required_assets_bytes: 2 * 1024 * 1024 * 1024,
            max_accelerator_bytes: 1024 * 1024 * 1024,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LoadOptions {
    pub limits: LoadLimits,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoadDiagnostic {
    DiscardedDisposableSection {
        section: u16,
        reason: String,
    },
    ExtensionProfileMismatch {
        expected: String,
        actual: SaveProfile,
    },
    PrunedStaleVisibility {
        entity_id: String,
    },
}

#[derive(Debug, Clone, Default)]
pub struct HydrationBundle {
    pub small_preview_png: Option<Vec<u8>>,
    pub large_preview_png: Option<Vec<u8>>,
    pub display_meshes: Option<Vec<(String, MockMesh)>>,
    pub evaluation_cache: Option<EvaluationCacheSnapshot>,
}

#[derive(Debug)]
pub struct LoadedDocument {
    pub document: crate::document::Document,
    pub profile: SaveProfile,
    pub accelerators: HydrationBundle,
    pub diagnostics: Vec<LoadDiagnostic>,
}

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
    #[serde(default)]
    pub profile: u8,
    #[serde(default)]
    pub accelerator_budget: u64,
    #[serde(default)]
    pub recipe_schema: u16,
    #[serde(default)]
    pub feature_payload_abi: u16,
    #[serde(default)]
    pub required_assets_abi: u16,
    #[serde(default)]
    pub openrcad_cache_abi: u16,
    #[serde(default)]
    pub tessellation_abi: u16,
    #[serde(default)]
    pub tolerance_policy_abi: u16,
    #[serde(default)]
    pub model_hash: [u8; 32],
    #[serde(default)]
    pub presentation_hash: [u8; 32],
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
    /// Creation timestamp to preserve across re-saves. `None` writes the legacy
    /// unspecified value (`0`); applications assign a timestamp on first save.
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
    pub large_preview_png: Option<Vec<u8>>,
    /// Present and fresh only when the embedded cache's `graph_hash` matches the
    /// graph that was loaded; a stale cache is discarded (left `None`).
    pub mesh_cache: Option<Vec<(String, MockMesh)>>,
    /// Node ids that were hidden when the file was saved.
    pub hidden_nodes: HashSet<String>,
    pub evaluation_cache: Option<EvaluationCacheSnapshot>,
    pub profile: SaveProfile,
    pub diagnostics: Vec<LoadDiagnostic>,
}

/// Stable, deterministic document recipe. This deliberately avoids serializing
/// petgraph's arena/index representation: node records and dependencies are
/// explicit, sorted, and independently migratable in future schema versions.
#[derive(Debug, Clone)]
pub struct DocumentRecipeV3 {
    pub schema_version: u16,
    pub features: Vec<RecipeFeature>,
    pub dependencies: Vec<RecipeDependency>,
    pub sketch_face_refs: Vec<(String, FaceRef)>,
    pub sketch_datum_refs: Vec<(String, String)>,
    pub sketch_face_boundaries: Vec<(String, SketchCurves)>,
    pub bodies: Vec<crate::document::BodyRecord>,
}

#[derive(Debug, Clone)]
pub struct RecipeFeature {
    pub id: String,
    pub name: String,
    pub creation_order: u64,
    pub payload_schema: u16,
    pub payload: RecipeFeaturePayload,
    pub kind_id: crate::document::FeatureKindId,
    pub sequence: crate::document::SequenceKey,
    pub inputs: Vec<crate::document::FeatureInput>,
    pub state: crate::document::FeatureState,
    pub body: Option<crate::document::BodyId>,
}

/// Versioned feature payload envelope. It serializes as a fixed tuple
/// `[tag, numeric_fields, content_hash, label]`, so neither Rust enum names nor
/// field names are part of the v5 persistence ABI.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RecipeFeaturePayload(
    u8,
    crate::feature_dto::NumericFeatureFields,
    Option<[u8; 32]>,
    Option<String>,
);

impl RecipeFeaturePayload {
    fn inline(feature: &crate::parametric::FeatureType) -> Self {
        Self(0, crate::feature_dto::encode(feature), None, None)
    }

    fn step_import(content_hash: [u8; 32], label: String) -> Self {
        Self(
            1,
            crate::feature_dto::NumericFeatureFields::new(),
            Some(content_hash),
            Some(label),
        )
    }

    fn stl_import(content_hash: [u8; 32], label: String) -> Self {
        Self(
            2,
            crate::feature_dto::NumericFeatureFields::new(),
            Some(content_hash),
            Some(label),
        )
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct RequiredAssetsV1 {
    schema_version: u16,
    blobs: Vec<RequiredAssetBlob>,
}

impl Default for RequiredAssetsV1 {
    fn default() -> Self {
        Self {
            schema_version: REQUIRED_ASSETS_SCHEMA,
            blobs: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct RequiredAssetBlob {
    content_hash: [u8; 32],
    bytes: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecipeDependency {
    pub parent: String,
    pub child: String,
}

impl Serialize for DocumentRecipeV3 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeMap as _;
        let mut map = serializer.serialize_map(Some(7))?;
        map.serialize_entry(&0u8, &self.schema_version)?;
        map.serialize_entry(&1u8, &self.features)?;
        map.serialize_entry(&2u8, &self.dependencies)?;
        map.serialize_entry(&3u8, &self.sketch_face_refs)?;
        map.serialize_entry(&4u8, &self.sketch_datum_refs)?;
        map.serialize_entry(&5u8, &self.sketch_face_boundaries)?;
        map.serialize_entry(&6u8, &self.bodies)?;
        map.end()
    }
}

impl<'de> serde::Deserialize<'de> for DocumentRecipeV3 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor;
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = DocumentRecipeV3;

            fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                formatter.write_str("a numeric-key ZeroCAD recipe map")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::MapAccess<'de>,
            {
                let mut schema_version = None;
                let mut features = None;
                let mut dependencies = None;
                let mut sketch_face_refs = None;
                let mut sketch_datum_refs = None;
                let mut sketch_face_boundaries = None;
                let mut bodies = None;
                while let Some(key) = map.next_key::<u8>()? {
                    match key {
                        0 if schema_version.is_none() => schema_version = Some(map.next_value()?),
                        1 if features.is_none() => features = Some(map.next_value()?),
                        2 if dependencies.is_none() => dependencies = Some(map.next_value()?),
                        3 if sketch_face_refs.is_none() => {
                            sketch_face_refs = Some(map.next_value()?)
                        }
                        4 if sketch_datum_refs.is_none() => {
                            sketch_datum_refs = Some(map.next_value()?)
                        }
                        5 if sketch_face_boundaries.is_none() => {
                            sketch_face_boundaries = Some(map.next_value()?)
                        }
                        6 if bodies.is_none() => bodies = Some(map.next_value()?),
                        0..=6 => {
                            return Err(serde::de::Error::custom(format!(
                                "duplicate recipe field {key}"
                            )))
                        }
                        _ => {
                            return Err(serde::de::Error::custom(format!(
                                "unknown recipe field {key}"
                            )))
                        }
                    }
                }
                Ok(DocumentRecipeV3 {
                    schema_version: schema_version
                        .ok_or_else(|| serde::de::Error::missing_field("0"))?,
                    features: features.ok_or_else(|| serde::de::Error::missing_field("1"))?,
                    dependencies: dependencies
                        .ok_or_else(|| serde::de::Error::missing_field("2"))?,
                    sketch_face_refs: sketch_face_refs
                        .ok_or_else(|| serde::de::Error::missing_field("3"))?,
                    sketch_datum_refs: sketch_datum_refs
                        .ok_or_else(|| serde::de::Error::missing_field("4"))?,
                    sketch_face_boundaries: sketch_face_boundaries
                        .ok_or_else(|| serde::de::Error::missing_field("5"))?,
                    bodies: bodies.ok_or_else(|| serde::de::Error::missing_field("6"))?,
                })
            }
        }
        deserializer.deserialize_map(Visitor)
    }
}

impl Serialize for RecipeFeature {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeMap as _;
        let mut map = serializer.serialize_map(Some(10))?;
        map.serialize_entry(&0u8, &self.id)?;
        map.serialize_entry(&1u8, &self.name)?;
        map.serialize_entry(&2u8, &self.creation_order)?;
        map.serialize_entry(&3u8, &self.payload_schema)?;
        map.serialize_entry(&4u8, &self.payload)?;
        map.serialize_entry(&5u8, &self.kind_id)?;
        map.serialize_entry(&6u8, &self.sequence)?;
        map.serialize_entry(&7u8, &self.inputs)?;
        map.serialize_entry(&8u8, &self.state)?;
        map.serialize_entry(&9u8, &self.body)?;
        map.end()
    }
}

impl<'de> serde::Deserialize<'de> for RecipeFeature {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor;
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = RecipeFeature;

            fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                formatter.write_str("a numeric-key feature record")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::MapAccess<'de>,
            {
                let mut id = None;
                let mut name = None;
                let mut creation_order = None;
                let mut payload_schema = None;
                let mut payload = None;
                let mut kind_id = None;
                let mut sequence = None;
                let mut inputs = None;
                let mut state = None;
                let mut body = None;
                let mut body_seen = false;
                while let Some(key) = map.next_key::<u8>()? {
                    match key {
                        0 if id.is_none() => id = Some(map.next_value()?),
                        1 if name.is_none() => name = Some(map.next_value()?),
                        2 if creation_order.is_none() => creation_order = Some(map.next_value()?),
                        3 if payload_schema.is_none() => payload_schema = Some(map.next_value()?),
                        4 if payload.is_none() => payload = Some(map.next_value()?),
                        5 if kind_id.is_none() => kind_id = Some(map.next_value()?),
                        6 if sequence.is_none() => sequence = Some(map.next_value()?),
                        7 if inputs.is_none() => inputs = Some(map.next_value()?),
                        8 if state.is_none() => state = Some(map.next_value()?),
                        9 if !body_seen => {
                            body = map.next_value()?;
                            body_seen = true;
                        }
                        0..=9 => {
                            return Err(serde::de::Error::custom(format!(
                                "duplicate feature field {key}"
                            )))
                        }
                        _ => {
                            return Err(serde::de::Error::custom(format!(
                                "unknown feature field {key}"
                            )))
                        }
                    }
                }
                Ok(RecipeFeature {
                    id: id.ok_or_else(|| serde::de::Error::missing_field("0"))?,
                    name: name.ok_or_else(|| serde::de::Error::missing_field("1"))?,
                    creation_order: creation_order
                        .ok_or_else(|| serde::de::Error::missing_field("2"))?,
                    payload_schema: payload_schema
                        .ok_or_else(|| serde::de::Error::missing_field("3"))?,
                    payload: payload.ok_or_else(|| serde::de::Error::missing_field("4"))?,
                    kind_id: kind_id.ok_or_else(|| serde::de::Error::missing_field("5"))?,
                    sequence: sequence.ok_or_else(|| serde::de::Error::missing_field("6"))?,
                    inputs: inputs.ok_or_else(|| serde::de::Error::missing_field("7"))?,
                    state: state.ok_or_else(|| serde::de::Error::missing_field("8"))?,
                    body: if body_seen {
                        body
                    } else {
                        return Err(serde::de::Error::missing_field("9"));
                    },
                })
            }
        }
        deserializer.deserialize_map(Visitor)
    }
}

impl Serialize for RecipeDependency {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&self.parent, &self.child).serialize(serializer)
    }
}

impl<'de> serde::Deserialize<'de> for RecipeDependency {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (parent, child) = <(String, String)>::deserialize(deserializer)?;
        Ok(Self { parent, child })
    }
}

impl DocumentRecipeV3 {
    pub fn from_graph(graph: &ParametricGraph) -> Self {
        Self::from_graph_with_assets(graph).0
    }

    fn from_graph_with_assets(graph: &ParametricGraph) -> (Self, RequiredAssetsV1) {
        use petgraph::visit::EdgeRef as _;

        let mut assets: BTreeMap<[u8; 32], Vec<u8>> = BTreeMap::new();
        let mut features: Vec<RecipeFeature> = graph
            .graph
            .node_indices()
            .map(|idx| {
                let node = &graph.graph[idx];
                let sequence = node.sequence;
                let payload = match &node.feature {
                    crate::parametric::FeatureType::Import { step_data, label } => {
                        let bytes = step_data.as_bytes().to_vec();
                        let content_hash = *blake3::hash(&bytes).as_bytes();
                        assets.entry(content_hash).or_insert(bytes);
                        RecipeFeaturePayload::step_import(content_hash, label.clone())
                    }
                    crate::parametric::FeatureType::ImportStl { stl_data, label } => {
                        let bytes = stl_data.clone();
                        let content_hash = *blake3::hash(&bytes).as_bytes();
                        assets.entry(content_hash).or_insert(bytes);
                        RecipeFeaturePayload::stl_import(content_hash, label.clone())
                    }
                    feature => RecipeFeaturePayload::inline(feature),
                };
                RecipeFeature {
                    id: node.id.clone(),
                    name: node.name.clone(),
                    creation_order: sequence.0,
                    payload_schema: node.payload_version,
                    payload,
                    kind_id: node.kind_id.clone(),
                    sequence,
                    inputs: node.inputs.clone(),
                    state: node.state,
                    body: node.body.clone(),
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

        let sorted_pairs = |map: &std::collections::HashMap<FeatureId, FeatureId>| {
            let mut pairs: Vec<(String, String)> = map
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect();
            pairs.sort_by(|a, b| a.0.cmp(&b.0));
            pairs
        };
        let mut sketch_face_refs: Vec<(String, FaceRef)> = graph
            .sketch_face_refs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect();
        sketch_face_refs.sort_by(|a, b| a.0.cmp(&b.0));
        let mut sketch_face_boundaries: Vec<(String, SketchCurves)> = graph
            .sketch_face_boundaries
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect();
        sketch_face_boundaries.sort_by(|a, b| a.0.cmp(&b.0));

        (
            Self {
                schema_version: DOCUMENT_RECIPE_SCHEMA,
                features,
                dependencies,
                sketch_face_refs,
                sketch_datum_refs: sorted_pairs(&graph.sketch_datum_refs),
                sketch_face_boundaries,
                bodies: graph.semantics.bodies.values().cloned().collect(),
            },
            RequiredAssetsV1 {
                schema_version: REQUIRED_ASSETS_SCHEMA,
                blobs: assets
                    .into_iter()
                    .map(|(content_hash, bytes)| RequiredAssetBlob {
                        content_hash,
                        bytes,
                    })
                    .collect(),
            },
        )
    }

    pub fn into_graph(self) -> Result<ParametricGraph, ZcadError> {
        self.into_graph_with_assets(&RequiredAssetsV1::default())
    }

    fn into_graph_with_assets(
        self,
        assets: &RequiredAssetsV1,
    ) -> Result<ParametricGraph, ZcadError> {
        self.into_graph_with_assets_and_registry(assets, crate::document::FeatureRegistry::get)
    }

    fn into_graph_with_assets_and_registry(
        self,
        assets: &RequiredAssetsV1,
        registration_for: impl Fn(&str) -> Option<&'static crate::document::FeatureRegistration>,
    ) -> Result<ParametricGraph, ZcadError> {
        if self.schema_version != DOCUMENT_RECIPE_SCHEMA {
            return Err(ZcadError::Decode(format!(
                "unsupported document recipe schema {}",
                self.schema_version
            )));
        }
        if assets.schema_version != REQUIRED_ASSETS_SCHEMA {
            return Err(ZcadError::Decode(format!(
                "unsupported required-assets schema {}",
                assets.schema_version
            )));
        }
        let DocumentRecipeV3 {
            features,
            dependencies,
            sketch_face_refs,
            sketch_datum_refs,
            sketch_face_boundaries,
            bodies,
            ..
        } = self;
        let mut asset_map: BTreeMap<[u8; 32], &[u8]> = BTreeMap::new();
        for asset in &assets.blobs {
            if *blake3::hash(&asset.bytes).as_bytes() != asset.content_hash {
                return Err(ZcadError::Decode(
                    "required asset content hash does not match its bytes".into(),
                ));
            }
            if asset_map
                .insert(asset.content_hash, asset.bytes.as_slice())
                .is_some()
            {
                return Err(ZcadError::Decode(
                    "duplicate required asset content hash".into(),
                ));
            }
        }
        // Decode every feature before constructing the graph. A malformed late
        // record therefore cannot leave even an internal partially built graph;
        // graph construction begins only after the complete payload set passes.
        let mut decoded_features = Vec::with_capacity(features.len());
        let mut feature_ids = HashSet::new();
        for record in features {
            let RecipeFeature {
                id,
                name,
                creation_order: _,
                payload_schema,
                payload,
                kind_id,
                sequence,
                inputs,
                state,
                body,
            } = record;
            if !feature_ids.insert(id.clone()) {
                return Err(ZcadError::Decode(format!("duplicate feature id '{id}'")));
            }
            let registration = registration_for(kind_id.as_str())
                .ok_or_else(|| ZcadError::Decode(format!("unknown feature kind '{kind_id}'")))?;
            let decoder = registration
                .payload_decoder(payload_schema)
                .ok_or_else(|| {
                    ZcadError::Decode(crate::feature_dto::unsupported_payload_schema(
                        registration,
                        payload_schema,
                    ))
                })?;
            let feature = match (decoder, payload) {
                (
                    crate::document::FeaturePayloadDecoder::NumericFieldsV1,
                    RecipeFeaturePayload(0, fields, None, None),
                ) => crate::feature_dto::decode_with_decoder(
                    kind_id.as_str(),
                    payload_schema,
                    decoder,
                    fields,
                )
                .map_err(ZcadError::Decode)?,
                (
                    crate::document::FeaturePayloadDecoder::StepAssetV1,
                    RecipeFeaturePayload(1, fields, Some(content_hash), Some(label)),
                ) if fields.is_empty() => {
                    let bytes = asset_map.get(&content_hash).ok_or_else(|| {
                        ZcadError::Decode(format!(
                            "STEP import '{id}' references missing required asset"
                        ))
                    })?;
                    let step_data = String::from_utf8(bytes.to_vec()).map_err(|_| {
                        ZcadError::Decode(format!(
                            "STEP import '{id}' required asset is not UTF-8"
                        ))
                    })?;
                    crate::parametric::FeatureType::Import { step_data, label }
                }
                (
                    crate::document::FeaturePayloadDecoder::StlAssetV1,
                    RecipeFeaturePayload(2, fields, Some(content_hash), Some(label)),
                ) if fields.is_empty() => {
                    let bytes = asset_map.get(&content_hash).ok_or_else(|| {
                        ZcadError::Decode(format!(
                            "STL import '{id}' references missing required asset"
                        ))
                    })?;
                    crate::parametric::FeatureType::ImportStl {
                        stl_data: bytes.to_vec(),
                        label,
                    }
                }
                (_, RecipeFeaturePayload(tag, _, _, _)) => {
                    return Err(ZcadError::Decode(format!(
                        "feature '{id}' kind '{kind_id}' schema {payload_schema} has malformed payload envelope tag {tag}"
                    )))
                }
            };
            let expected = feature.kind_id();
            if kind_id.as_str() != expected {
                return Err(ZcadError::Decode(format!(
                    "feature '{id}' declares kind '{kind_id}' but contains '{expected}' payload"
                )));
            }
            decoded_features.push((
                FeatureNode { id, name, feature },
                kind_id,
                registration.payload_version,
                sequence,
                inputs,
                state,
                body,
            ));
        }

        let mut graph = ParametricGraph::new();
        for (node, kind_id, payload_version, sequence, inputs, state, body) in decoded_features {
            if node.id == "origin" {
                let origin = graph
                    .graph
                    .node_weights_mut()
                    .find(|feature| feature.id == "origin")
                    .expect("new graph must contain origin");
                origin.name = node.name;
                origin.kind_id = kind_id;
                origin.payload_version = payload_version;
                origin.sequence = sequence;
                origin.inputs = inputs;
                origin.state = state;
                origin.body = body;
                continue;
            }
            let idx = graph.add_feature(node);
            let stored = &mut graph.graph[idx];
            stored.kind_id = kind_id;
            // Decoders always produce the current in-memory representation. A
            // later explicit save therefore writes the registry's current
            // schema instead of preserving a historical schema number.
            stored.payload_version = payload_version;
            stored.sequence = sequence;
            stored.inputs = inputs;
            stored.state = state;
            stored.body = body;
        }
        for dependency in dependencies {
            graph
                .add_dependency_for_load(&dependency.parent, &dependency.child)
                .map_err(ZcadError::Decode)?;
        }
        graph.sketch_face_refs = sketch_face_refs
            .into_iter()
            .map(|(feature_id, reference)| (feature_id.into(), reference))
            .collect();
        graph.sketch_datum_refs = sketch_datum_refs
            .into_iter()
            .map(|(sketch_id, datum_id)| (sketch_id.into(), datum_id.into()))
            .collect();
        graph.sketch_face_boundaries = sketch_face_boundaries
            .into_iter()
            .map(|(feature_id, curves)| (feature_id.into(), curves))
            .collect();
        let mut body_records = BTreeMap::new();
        for body in bodies {
            if body_records.insert(body.id.clone(), body).is_some() {
                return Err(ZcadError::Decode("duplicate semantic body id".into()));
            }
        }
        graph.semantics.bodies = body_records;
        graph
            .validate_semantic_contracts()
            .map_err(ZcadError::Decode)?;
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
    /// A BLAKE3 digest mismatch. `section` is 0 for the file header, else the section id.
    BadChecksum {
        section: u16,
    },
    /// The framing version is newer than this build can read.
    UnsupportedVersion(u16),
    LimitExceeded {
        what: &'static str,
        limit: u64,
        actual: u64,
    },
    DuplicateSection(u16),
    OverlappingSections {
        first: u16,
        second: u16,
    },
    UnknownRequiredSection(u16),
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
            ZcadError::LimitExceeded {
                what,
                limit,
                actual,
            } => write!(f, "{what} exceeds load limit ({actual} > {limit})"),
            ZcadError::DuplicateSection(section) => {
                write!(f, "duplicate section {section}")
            }
            ZcadError::OverlappingSections { first, second } => {
                write!(f, "sections {first} and {second} overlap")
            }
            ZcadError::UnknownRequiredSection(section) => {
                write!(f, "unknown required section {section}")
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
/// BLAKE3 fingerprint used to tie derived caches to their exact recipe. This is an
/// identity check in addition to the container's independent section digests.
fn recipe_hash(recipe_cbor: &[u8]) -> [u8; 32] {
    *blake3::hash(recipe_cbor).as_bytes()
}

fn model_hash(recipe_cbor: &[u8], assets: &RequiredAssetsV1) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(recipe_cbor);
    let mut hashes: Vec<_> = assets
        .blobs
        .iter()
        .map(|asset| asset.content_hash)
        .collect();
    hashes.sort_unstable();
    hashes.dedup();
    for hash in hashes {
        hasher.update(&hash);
    }
    *hasher.finalize().as_bytes()
}

fn presentation_hash(units: Unit, hidden: &HashSet<String>) -> Result<[u8; 32], ZcadError> {
    let mut hidden: Vec<&str> = hidden.iter().map(String::as_str).collect();
    hidden.sort_unstable();
    Ok(*blake3::hash(&cbor_to_vec(&(units, hidden))?).as_bytes())
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

fn mesh_cache_valid(bodies: &[(String, MockMesh)], graph: &ParametricGraph) -> Result<(), String> {
    let mut body_ids = HashSet::with_capacity(bodies.len());
    for (body_id, mesh) in bodies {
        if body_id.is_empty() || !body_ids.insert(body_id.as_str()) {
            return Err(format!("invalid or duplicate display body id '{body_id}'"));
        }
        if graph.body_producer_feature_id(body_id).is_none() {
            return Err(format!("display body '{body_id}' has no owning feature"));
        }
        if mesh.vertices.len() % 6 != 0
            || mesh.indices.len() % 3 != 0
            || mesh.edge_vertices.len() % 3 != 0
            || mesh.edge_indices.len() % 2 != 0
        {
            return Err(format!(
                "display body '{body_id}' has malformed buffer lengths"
            ));
        }
        if !mesh.vertices.iter().all(|value| value.is_finite())
            || !mesh.edge_vertices.iter().all(|value| value.is_finite())
            || !mesh.edge_face_normals.iter().all(|value| value.is_finite())
        {
            return Err(format!(
                "display body '{body_id}' contains non-finite values"
            ));
        }
        let vertex_count = mesh.vertices.len() / 6;
        let edge_vertex_count = mesh.edge_vertices.len() / 3;
        if mesh
            .indices
            .iter()
            .any(|index| *index as usize >= vertex_count)
            || mesh
                .edge_indices
                .iter()
                .any(|index| *index as usize >= edge_vertex_count)
        {
            return Err(format!(
                "display body '{body_id}' contains an invalid index"
            ));
        }
        let triangle_count = mesh.indices.len() / 3;
        let edge_count = mesh.edge_indices.len() / 2;
        if (!mesh.face_ids.is_empty() && mesh.face_ids.len() != triangle_count)
            || (!mesh.edge_groups.is_empty() && mesh.edge_groups.len() != edge_count)
            || (!mesh.edge_face_normals.is_empty()
                && mesh.edge_face_normals.len() != edge_count.saturating_mul(6))
        {
            return Err(format!(
                "display body '{body_id}' has inconsistent topology metadata"
            ));
        }
        let finite3 = |point: &[f32; 3]| point.iter().all(|value| value.is_finite());
        if mesh.edge_refs.iter().any(|edge| {
            !finite3(&edge.p0)
                || !finite3(&edge.p1)
                || !finite3(&edge.n1)
                || !finite3(&edge.n2)
                || edge.curve.as_ref().is_some_and(|curve| match curve {
                    crate::mock_kernel::EdgeCurveHint::Line => false,
                    crate::mock_kernel::EdgeCurveHint::Circle {
                        center,
                        axis,
                        x_dir,
                        radius,
                        start,
                        end,
                        ..
                    } => {
                        !finite3(center)
                            || !finite3(axis)
                            || !finite3(x_dir)
                            || !radius.is_finite()
                            || !start.is_finite()
                            || !end.is_finite()
                    }
                })
        }) || mesh
            .face_refs
            .iter()
            .any(|face| !finite3(&face.centroid) || !finite3(&face.normal))
        {
            return Err(format!(
                "display body '{body_id}' has invalid selection metadata"
            ));
        }
    }
    Ok(())
}

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn cbor_to_vec<T: Serialize>(v: &T) -> Result<Vec<u8>, ZcadError> {
    let mut raw = Vec::new();
    ciborium::into_writer(v, &mut raw).map_err(|e| ZcadError::Decode(e.to_string()))?;
    let mut value: ciborium::value::Value =
        ciborium::from_reader(raw.as_slice()).map_err(|e| ZcadError::Decode(e.to_string()))?;
    canonicalize_cbor(&mut value)?;
    let mut canonical = Vec::new();
    ciborium::into_writer(&value, &mut canonical).map_err(|e| ZcadError::Decode(e.to_string()))?;
    Ok(canonical)
}

fn canonicalize_cbor(value: &mut ciborium::value::Value) -> Result<(), ZcadError> {
    use ciborium::value::Value;
    match value {
        Value::Float(number) => {
            if !number.is_finite() {
                return Err(ZcadError::Decode(
                    "non-finite numeric parameter cannot be stored".into(),
                ));
            }
            if *number == 0.0 {
                *number = 0.0;
            }
        }
        Value::Array(values) => {
            for value in values {
                canonicalize_cbor(value)?;
            }
        }
        Value::Map(entries) => {
            for (key, value) in entries.iter_mut() {
                canonicalize_cbor(key)?;
                canonicalize_cbor(value)?;
            }
            entries.sort_by(|a, b| {
                let a: ciborium::value::CanonicalValue = a.0.clone().into();
                let b: ciborium::value::CanonicalValue = b.0.clone().into();
                a.cmp(&b)
            });
        }
        Value::Tag(_, inner) => canonicalize_cbor(inner)?,
        _ => {}
    }
    Ok(())
}

/// Stream a profile-enforced v5 document to a seekable destination.
pub fn write_document<W: Write + Seek>(
    writer: &mut W,
    document: &crate::document::Document,
    options: &SaveOptions,
    accelerators: &HydrationBundle,
) -> Result<(), ZcadError> {
    document
        .validate_semantic_contracts()
        .map_err(|error| ZcadError::Decode(format!("invalid semantic document: {error}")))?;
    let budget = match options.profile {
        SaveProfile::Compact => None,
        SaveProfile::Hydrated {
            total_accelerator_budget,
        } => Some(usize::try_from(total_accelerator_budget).map_err(|_| {
            ZcadError::LimitExceeded {
                what: "accelerator budget",
                limit: usize::MAX as u64,
                actual: total_accelerator_budget,
            }
        })?),
    };
    let hidden_nodes = document.hidden_entities();
    let legacy = ZcadDocument {
        graph: document.evaluator_graph(),
        thumbnail_png: accelerators.small_preview_png.clone(),
        mesh_cache: budget.and(accelerators.display_meshes.as_deref()),
        units: document.state.units,
        bbox: accelerators
            .display_meshes
            .as_deref()
            .map(meshes_bbox)
            .unwrap_or([0.0; 6]),
        created_unix: document.state.created_unix,
        hidden_nodes,
        evaluation_cache: budget.and(accelerators.evaluation_cache.as_ref()),
        hydrated_cache_limit: budget,
    };
    let sections = stage_zcad_with_profile(
        &legacy,
        options.profile,
        accelerators.large_preview_png.as_deref(),
    )?;
    write_staged_sections(writer, options.profile, &sections)
}

pub fn write_document_to_vec(
    document: &crate::document::Document,
    options: &SaveOptions,
    accelerators: &HydrationBundle,
) -> Result<Vec<u8>, ZcadError> {
    let mut cursor = std::io::Cursor::new(Vec::new());
    write_document(&mut cursor, document, options, accelerators)?;
    Ok(cursor.into_inner())
}

pub fn read_document<R: Read + Seek>(
    reader: &mut R,
    options: &LoadOptions,
) -> Result<LoadedDocument, ZcadError> {
    loaded_zcad_to_document(read_binary_reader(reader, options)?)
}

pub fn read_document_from_slice(
    bytes: &[u8],
    options: &LoadOptions,
) -> Result<LoadedDocument, ZcadError> {
    loaded_zcad_to_document(read_binary(bytes, options)?)
}

fn loaded_zcad_to_document(loaded: LoadedZcad) -> Result<LoadedDocument, ZcadError> {
    let mut document = crate::document::Document::from_graph(loaded.graph, loaded.metadata.units);
    document.state.created_unix =
        (loaded.metadata.created_unix != 0).then_some(loaded.metadata.created_unix);
    let mut diagnostics = loaded.diagnostics;
    let mut hidden_nodes: Vec<_> = loaded.hidden_nodes.into_iter().collect();
    hidden_nodes.sort_unstable();
    for hidden in hidden_nodes {
        let valid = document.body_producer_feature_id(&hidden).is_some()
            || document
                .semantics
                .bodies
                .contains_key(&crate::document::BodyId::from(hidden.as_str()));
        if valid {
            document.set_visible(hidden, false);
        } else {
            diagnostics.push(LoadDiagnostic::PrunedStaleVisibility { entity_id: hidden });
        }
    }
    Ok(LoadedDocument {
        document,
        profile: loaded.profile,
        accelerators: HydrationBundle {
            small_preview_png: loaded.thumbnail_png,
            large_preview_png: loaded.large_preview_png,
            display_meshes: loaded.mesh_cache,
            evaluation_cache: loaded.evaluation_cache,
        },
        diagnostics,
    })
}

fn meshes_bbox(bodies: &[(String, MockMesh)]) -> [f32; 6] {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for (_, mesh) in bodies {
        for vertex in mesh.vertices.chunks_exact(6) {
            for axis in 0..3 {
                min[axis] = min[axis].min(vertex[axis]);
                max[axis] = max[axis].max(vertex[axis]);
            }
        }
    }
    if min[0].is_finite() {
        [min[0], min[1], min[2], max[0], max[1], max[2]]
    } else {
        [0.0; 6]
    }
}

fn cbor_from_slice<T: DeserializeOwned>(b: &[u8]) -> Result<T, ZcadError> {
    let mut cursor = std::io::Cursor::new(b);
    let value = ciborium::from_reader(&mut cursor).map_err(|e| ZcadError::Decode(e.to_string()))?;
    if cursor.position() != b.len() as u64 {
        return Err(ZcadError::Decode(
            "CBOR payload contains trailing bytes".into(),
        ));
    }
    Ok(value)
}

fn zstd_compress(data: &[u8], level: i32) -> Result<Vec<u8>, ZcadError> {
    zstd::encode_all(data, level).map_err(ZcadError::Io)
}

fn zstd_decompress(data: &[u8], expected_len: usize) -> Result<Vec<u8>, ZcadError> {
    let decoder = zstd::Decoder::new(data).map_err(ZcadError::Io)?;
    let limit = expected_len
        .checked_add(1)
        .ok_or_else(|| ZcadError::Decode("declared decoded length overflows".into()))?;
    let mut out = Vec::with_capacity(expected_len.min(1024 * 1024));
    decoder
        .take(limit as u64)
        .read_to_end(&mut out)
        .map_err(ZcadError::Io)?;
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
    flags: u8,
    codec: u8,
    /// Bytes as they go on disk (compressed if `codec == CODEC_ZSTD`).
    stored: Vec<u8>,
    /// Length after decompression (== `stored.len()` when stored).
    uncompressed_len: usize,
}

/// Source-compatible wrapper during the GUI migration. It always emits v5 and
/// derives the profile from the presence of accelerators. It stages the whole
/// file in memory and cannot preserve the explicit presentation state carried
/// by [`crate::document::Document`].
#[deprecated(
    since = "0.1.0",
    note = "use write_document_to_vec; this wrapper infers the save profile and drops explicit document presentation state"
)]
pub fn write_zcad(doc: &ZcadDocument) -> Result<Vec<u8>, ZcadError> {
    let profile = if doc.mesh_cache.is_some() || doc.evaluation_cache.is_some() {
        SaveProfile::Hydrated {
            total_accelerator_budget: doc
                .hydrated_cache_limit
                .unwrap_or(DEFAULT_HYDRATED_CACHE_LIMIT)
                as u64,
        }
    } else {
        SaveProfile::Compact
    };
    write_zcad_with_profile(doc, profile, None)
}

fn stage_zcad_with_profile(
    doc: &ZcadDocument,
    profile: SaveProfile,
    large_preview_png: Option<&[u8]>,
) -> Result<Vec<StagedSection>, ZcadError> {
    let mut sections: Vec<StagedSection> = Vec::new();
    let accelerator_budget = match profile {
        SaveProfile::Compact => 0,
        SaveProfile::Hydrated {
            total_accelerator_budget,
        } => usize::try_from(total_accelerator_budget).unwrap_or(usize::MAX),
    };
    let mut accelerator_used = 0usize;

    // --- GRAPH (source of truth) ---
    let (recipe, required_assets) = DocumentRecipeV3::from_graph_with_assets(doc.graph);
    let recipe_cbor = cbor_to_vec(&recipe)?;
    let recipe_hash = recipe_hash(&recipe_cbor);
    let model_hash = model_hash(&recipe_cbor, &required_assets);
    let recipe_uncompressed = recipe_cbor.len();
    let recipe_stored = zstd_compress(&recipe_cbor, GRAPH_LEVEL)?;

    // --- METADATA (uncompressed, written first) ---
    let meta = ZcadMetadata {
        format_version: CURRENT_VERSION,
        app_version: env!("CARGO_PKG_VERSION").to_string(),
        created_unix: doc.created_unix.unwrap_or(0),
        modified_unix: 0,
        units: doc.units,
        feature_count: doc.graph.graph.node_count() as u32,
        bbox: doc.bbox,
        profile: profile.header_tag(),
        accelerator_budget: match profile {
            SaveProfile::Compact => 0,
            SaveProfile::Hydrated {
                total_accelerator_budget,
            } => total_accelerator_budget,
        },
        recipe_schema: DOCUMENT_RECIPE_SCHEMA,
        feature_payload_abi: FEATURE_PAYLOAD_ABI,
        required_assets_abi: REQUIRED_ASSETS_SCHEMA,
        openrcad_cache_abi: OPENRCAD_CACHE_ABI,
        tessellation_abi: MESH_CACHE_ABI,
        tolerance_policy_abi: TOLERANCE_POLICY_ABI,
        model_hash,
        presentation_hash: presentation_hash(doc.units, &doc.hidden_nodes)?,
    };
    let meta_cbor = cbor_to_vec(&meta)?;
    sections.push(StagedSection {
        id: SEC_METADATA,
        flags: SECTION_REQUIRED,
        codec: CODEC_STORE,
        uncompressed_len: meta_cbor.len(),
        stored: meta_cbor,
    });

    sections.push(StagedSection {
        id: SEC_GRAPH,
        flags: SECTION_REQUIRED,
        codec: CODEC_ZSTD,
        uncompressed_len: recipe_uncompressed,
        stored: recipe_stored,
    });

    if !required_assets.blobs.is_empty() {
        let cbor = cbor_to_vec(&required_assets)?;
        let uncompressed_len = cbor.len();
        let stored = zstd_compress(&cbor, GRAPH_LEVEL)?;
        sections.push(StagedSection {
            id: SEC_REQUIRED_ASSETS,
            flags: SECTION_REQUIRED,
            codec: CODEC_ZSTD,
            uncompressed_len,
            stored,
        });
    }

    // --- THUMBNAIL (stored; PNG is already compressed) ---
    if let Some(png) = &doc.thumbnail_png {
        if !png.is_empty() && png.len() <= MAX_TINY_PREVIEW_BYTES {
            sections.push(StagedSection {
                id: SEC_THUMBNAIL,
                flags: SECTION_DISPOSABLE,
                codec: CODEC_STORE,
                uncompressed_len: png.len(),
                stored: png.clone(),
            });
        }
    }

    // --- MESH_CACHE (optional, zstd) ---
    if matches!(profile, SaveProfile::Hydrated { .. }) {
        if let Some(bodies) = doc.mesh_cache {
            let payload = MeshCachePayload {
                mesh_cache_abi: MESH_CACHE_ABI,
                recipe_hash,
                bodies: bodies.to_vec(),
            };
            let cbor = cbor_to_vec(&payload)?;
            let uncompressed_len = cbor.len();
            let stored = zstd_compress(&cbor, MESH_LEVEL)?;
            if stored.len() <= accelerator_budget {
                accelerator_used += stored.len();
                sections.push(StagedSection {
                    id: SEC_MESH_CACHE,
                    flags: SECTION_DISPOSABLE,
                    codec: CODEC_ZSTD,
                    uncompressed_len,
                    stored,
                });
            }
        }
    }

    // --- HIDDEN_NODES (optional, zstd) ---
    if !doc.hidden_nodes.is_empty() {
        let mut hidden_nodes: Vec<&str> = doc.hidden_nodes.iter().map(String::as_str).collect();
        hidden_nodes.sort_unstable();
        let cbor = cbor_to_vec(&hidden_nodes)?;
        let uncompressed_len = cbor.len();
        let stored = zstd_compress(&cbor, GRAPH_LEVEL)?;
        sections.push(StagedSection {
            id: SEC_HIDDEN_NODES,
            flags: SECTION_REQUIRED,
            codec: CODEC_ZSTD,
            uncompressed_len,
            stored,
        });
    }

    // --- HYDRATED CHECKPOINTS (optional, disposable) ---
    if matches!(profile, SaveProfile::Hydrated { .. }) {
        if let Some(cache) = doc.evaluation_cache {
            let mut sparse = sparse_hydrated_cache(cache, doc.mesh_cache);
            let limit = accelerator_budget.saturating_sub(accelerator_used);
            loop {
                let payload = hydrated_payload(&sparse, recipe_hash)?;
                let cbor = cbor_to_vec(&payload)?;
                let stored = zstd_compress(&cbor, MESH_LEVEL)?;
                if stored.len() <= limit {
                    accelerator_used += stored.len();
                    sections.push(StagedSection {
                        id: SEC_HYDRATED_CHECKPOINTS,
                        flags: SECTION_DISPOSABLE,
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
    }

    if matches!(profile, SaveProfile::Hydrated { .. }) {
        if let Some(png) = large_preview_png {
            if !png.is_empty() && png.len() <= accelerator_budget.saturating_sub(accelerator_used) {
                sections.push(StagedSection {
                    id: SEC_LARGE_PREVIEW,
                    flags: SECTION_DISPOSABLE,
                    codec: CODEC_STORE,
                    uncompressed_len: png.len(),
                    stored: png.to_vec(),
                });
            }
        }
    }

    if sections.len() > u16::MAX as usize {
        return Err(ZcadError::LimitExceeded {
            what: "section count",
            limit: u16::MAX as u64,
            actual: sections.len() as u64,
        });
    }
    Ok(sections)
}

/// Write already-compressed sections directly to their destination. The
/// section table is emitted first and payloads are streamed without assembling
/// a second full-file buffer.
fn write_staged_sections<W: Write + Seek>(
    writer: &mut W,
    profile: SaveProfile,
    sections: &[StagedSection],
) -> Result<(), ZcadError> {
    let section_count = sections.len();
    let table_len = section_count
        .checked_mul(TABLE_ENTRY_LEN)
        .ok_or(ZcadError::LimitExceeded {
            what: "section table length",
            limit: usize::MAX as u64,
            actual: u64::MAX,
        })?;
    let mut payload_offset = HEADER_LEN
        .checked_add(table_len)
        .ok_or(ZcadError::LimitExceeded {
            what: "container length",
            limit: usize::MAX as u64,
            actual: u64::MAX,
        })?;

    let mut header = Vec::with_capacity(HEADER_LEN);
    header.extend_from_slice(MAGIC);
    header.extend_from_slice(&CURRENT_VERSION.to_le_bytes());
    header.push(profile.header_tag());
    header.push(0);
    header.extend_from_slice(&(section_count as u16).to_le_bytes());
    header.extend_from_slice(&[0u8; 2]);
    let header_digest = blake3::hash(&header);
    header.extend_from_slice(&header_digest.as_bytes()[..16]);
    header.extend_from_slice(&[0u8; 4]);

    writer.seek(SeekFrom::Start(0)).map_err(ZcadError::Io)?;
    writer.write_all(&header).map_err(ZcadError::Io)?;
    for section in sections {
        let next_offset =
            payload_offset
                .checked_add(section.stored.len())
                .ok_or(ZcadError::LimitExceeded {
                    what: "container length",
                    limit: usize::MAX as u64,
                    actual: u64::MAX,
                })?;
        let digest = blake3::hash(&section.stored);
        writer
            .write_all(&section.id.to_le_bytes())
            .and_then(|_| writer.write_all(&[section.flags, section.codec]))
            .and_then(|_| writer.write_all(&(payload_offset as u64).to_le_bytes()))
            .and_then(|_| writer.write_all(&(section.stored.len() as u64).to_le_bytes()))
            .and_then(|_| writer.write_all(&(section.uncompressed_len as u64).to_le_bytes()))
            .and_then(|_| writer.write_all(&digest.as_bytes()[..16]))
            .and_then(|_| writer.write_all(&[0u8; 4]))
            .map_err(ZcadError::Io)?;
        payload_offset = next_offset;
    }
    for section in sections {
        writer.write_all(&section.stored).map_err(ZcadError::Io)?;
    }
    writer.flush().map_err(ZcadError::Io)
}

fn write_zcad_with_profile(
    doc: &ZcadDocument,
    profile: SaveProfile,
    large_preview_png: Option<&[u8]>,
) -> Result<Vec<u8>, ZcadError> {
    let sections = stage_zcad_with_profile(doc, profile, large_preview_png)?;
    let mut cursor = std::io::Cursor::new(Vec::new());
    write_staged_sections(&mut cursor, profile, &sections)?;
    Ok(cursor.into_inner())
}

/// Legacy crash-resilient path writer. The new bytes are fully written and
/// synced before the previous document is moved aside; if the final rename
/// fails, the previous file is restored from the sibling backup.
#[deprecated(
    since = "0.1.0",
    note = "use write_document_file; this wrapper stages the full file, infers the profile, and drops explicit document presentation state"
)]
#[allow(deprecated)]
pub fn write_zcad_file(path: impl AsRef<Path>, doc: &ZcadDocument) -> Result<(), ZcadError> {
    let bytes = write_zcad(doc)?;
    write_atomic_bytes(path.as_ref(), &bytes)
}

fn write_atomic_bytes(path: &Path, bytes: &[u8]) -> Result<(), ZcadError> {
    write_atomic(path, |file| file.write_all(bytes).map_err(ZcadError::Io))
}

fn write_atomic(
    path: &Path,
    write: impl FnOnce(&mut std::fs::File) -> Result<(), ZcadError>,
) -> Result<(), ZcadError> {
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

    let write_result = (|| -> Result<(), ZcadError> {
        let mut file = std::fs::File::create(&temp).map_err(ZcadError::Io)?;
        write(&mut file)?;
        file.sync_all().map_err(ZcadError::Io)?;

        if backup.exists() {
            std::fs::remove_file(&backup).map_err(ZcadError::Io)?;
        }
        let had_previous = path.exists();
        if had_previous {
            std::fs::rename(path, &backup).map_err(ZcadError::Io)?;
        }
        if let Err(error) = std::fs::rename(&temp, path) {
            if had_previous {
                let _ = std::fs::rename(&backup, path);
            }
            return Err(ZcadError::Io(error));
        }
        // Persist the directory entry where the platform supports syncing a
        // directory handle. Windows may reject it, so replacement correctness
        // still relies on the fully-synced file plus backup protocol there.
        let _ = std::fs::File::open(parent).and_then(|directory| directory.sync_all());
        if had_previous {
            let _ = std::fs::remove_file(&backup);
        }
        Ok(())
    })();

    if write_result.is_err() {
        let _ = std::fs::remove_file(&temp);
    }
    write_result
}

pub fn write_document_file(
    path: impl AsRef<Path>,
    document: &crate::document::Document,
    options: &SaveOptions,
    accelerators: &HydrationBundle,
) -> Result<(), ZcadError> {
    write_atomic(path.as_ref(), |file| {
        write_document(file, document, options, accelerators)
    })
}

pub fn read_document_file(
    path: impl AsRef<Path>,
    options: &LoadOptions,
) -> Result<LoadedDocument, ZcadError> {
    let path = path.as_ref();
    let mut file = std::fs::File::open(path).map_err(ZcadError::Io)?;
    let mut loaded = read_document(&mut file, options)?;
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(str::to_ascii_lowercase);
    let mismatch = match (extension.as_deref(), loaded.profile) {
        (Some("zcad"), SaveProfile::Hydrated { .. }) => Some("Compact"),
        (Some("zcadh"), SaveProfile::Compact) => Some("Hydrated"),
        _ => None,
    };
    if let Some(expected) = mismatch {
        loaded
            .diagnostics
            .push(LoadDiagnostic::ExtensionProfileMismatch {
                expected: expected.to_owned(),
                actual: loaded.profile,
            });
    }
    Ok(loaded)
}

#[deprecated(
    since = "0.1.0",
    note = "use read_document_file; this wrapper returns the legacy projection and default load policy"
)]
pub fn read_zcad_file(path: impl AsRef<Path>) -> Result<LoadedZcad, ZcadError> {
    let mut file = std::fs::File::open(path).map_err(ZcadError::Io)?;
    read_binary_reader(&mut file, &LoadOptions::default())
}

/// Parse a v5 binary document. v4 and the old JSON prototype are intentionally
/// rejected by the one authorized compatibility reset.
#[deprecated(
    since = "0.1.0",
    note = "use read_document_from_slice; this wrapper returns the legacy projection and default load policy"
)]
#[allow(deprecated)]
pub fn read_zcad(bytes: &[u8]) -> Result<LoadedZcad, ZcadError> {
    read_zcad_with_options(bytes, &LoadOptions::default())
}

#[deprecated(
    since = "0.1.0",
    note = "use read_document_from_slice; this wrapper returns the legacy document projection"
)]
pub fn read_zcad_with_options(
    bytes: &[u8],
    options: &LoadOptions,
) -> Result<LoadedZcad, ZcadError> {
    if bytes.len() >= 4 && &bytes[0..4] == MAGIC {
        return read_binary(bytes, options);
    }
    Err(ZcadError::NotZcad)
}

fn le_u16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}
fn le_u64(b: &[u8]) -> u64 {
    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
}

struct SectionRef {
    id: u16,
    flags: u8,
    codec: u8,
    stored: Vec<u8>,
    uncompressed_len: usize,
}

fn read_binary(bytes: &[u8], options: &LoadOptions) -> Result<LoadedZcad, ZcadError> {
    let mut cursor = std::io::Cursor::new(bytes);
    read_binary_reader(&mut cursor, options)
}

#[derive(Debug)]
struct SectionDescriptor {
    id: u16,
    flags: u8,
    codec: u8,
    offset: u64,
    stored_len: usize,
    uncompressed_len: usize,
    expected_digest: [u8; 16],
}

fn read_exact_or_truncated(reader: &mut impl Read, bytes: &mut [u8]) -> Result<(), ZcadError> {
    reader.read_exact(bytes).map_err(|error| {
        if error.kind() == std::io::ErrorKind::UnexpectedEof {
            ZcadError::Truncated
        } else {
            ZcadError::Io(error)
        }
    })
}

fn decoded_section_limit(id: u16, flags: u8, limits: &LoadLimits) -> u64 {
    match id {
        SEC_METADATA => limits.max_manifest_bytes,
        SEC_GRAPH | SEC_HIDDEN_NODES => limits.max_recipe_bytes,
        SEC_REQUIRED_ASSETS => limits.max_required_assets_bytes,
        SEC_THUMBNAIL | SEC_MESH_CACHE | SEC_HYDRATED_CHECKPOINTS | SEC_LARGE_PREVIEW => {
            limits.max_accelerator_bytes
        }
        _ if flags & SECTION_REQUIRED != 0 => limits.max_required_assets_bytes,
        _ => limits.max_accelerator_bytes,
    }
}

fn known_section(id: u16) -> bool {
    matches!(
        id,
        SEC_METADATA
            | SEC_GRAPH
            | SEC_THUMBNAIL
            | SEC_MESH_CACHE
            | SEC_HIDDEN_NODES
            | SEC_HYDRATED_CHECKPOINTS
            | SEC_LARGE_PREVIEW
            | SEC_REQUIRED_ASSETS
    )
}

fn read_binary_reader<R: Read + Seek>(
    reader: &mut R,
    options: &LoadOptions,
) -> Result<LoadedZcad, ZcadError> {
    let file_len = reader.seek(SeekFrom::End(0)).map_err(ZcadError::Io)?;
    reader.seek(SeekFrom::Start(0)).map_err(ZcadError::Io)?;
    let mut header = [0u8; HEADER_LEN];
    read_exact_or_truncated(reader, &mut header)?;
    if &header[0..4] != MAGIC {
        return Err(ZcadError::NotZcad);
    }
    let expected_header = blake3::hash(&header[0..12]);
    if header[12..28] != expected_header.as_bytes()[..16] {
        return Err(ZcadError::BadChecksum { section: 0 });
    }
    if header[7] != 0 || header[10..12] != [0, 0] || header[28..32] != [0; 4] {
        return Err(ZcadError::Decode(
            "non-zero reserved container header bytes".into(),
        ));
    }
    let format_version = le_u16(&header[4..6]);
    if format_version != CURRENT_VERSION {
        return Err(ZcadError::UnsupportedVersion(format_version));
    }
    let profile_tag = header[6];
    let section_count = le_u16(&header[8..10]) as usize;
    if section_count > options.limits.max_sections as usize {
        return Err(ZcadError::LimitExceeded {
            what: "section count",
            limit: options.limits.max_sections as u64,
            actual: section_count as u64,
        });
    }
    let table_len = section_count
        .checked_mul(TABLE_ENTRY_LEN)
        .ok_or(ZcadError::Truncated)?;
    let table_end = HEADER_LEN
        .checked_add(table_len)
        .ok_or(ZcadError::Truncated)?;
    if file_len < table_end as u64 {
        return Err(ZcadError::Truncated);
    }
    let mut table = vec![0u8; table_len];
    read_exact_or_truncated(reader, &mut table)?;

    let mut descriptors = Vec::with_capacity(section_count);
    let mut seen = HashSet::with_capacity(section_count);
    let mut ranges = Vec::with_capacity(section_count);
    let mut decoded_accelerator_total = 0u64;
    let mut accelerator_stored_total = 0u64;
    for entry in table.chunks_exact(TABLE_ENTRY_LEN) {
        let id = le_u16(&entry[0..2]);
        if !seen.insert(id) {
            return Err(ZcadError::DuplicateSection(id));
        }
        let flags = entry[2];
        let codec = entry[3];
        let required = flags & SECTION_REQUIRED != 0;
        let disposable = flags & SECTION_DISPOSABLE != 0;
        if flags & !(SECTION_REQUIRED | SECTION_DISPOSABLE) != 0 || required == disposable {
            return Err(ZcadError::Decode(format!(
                "section {id} must be exactly one of required or disposable"
            )));
        }
        if entry[44..48] != [0; 4] {
            return Err(ZcadError::Decode(format!(
                "section {id} has non-zero reserved table bytes"
            )));
        }
        if required && !known_section(id) {
            return Err(ZcadError::UnknownRequiredSection(id));
        }

        let offset = le_u64(&entry[4..12]);
        let stored_len_u64 = le_u64(&entry[12..20]);
        let uncompressed_len_u64 = le_u64(&entry[20..28]);
        let decoded_limit = decoded_section_limit(id, flags, &options.limits);
        if uncompressed_len_u64 > decoded_limit {
            return Err(ZcadError::LimitExceeded {
                what: "decoded section length",
                limit: decoded_limit,
                actual: uncompressed_len_u64,
            });
        }
        let stored_limit = decoded_limit.saturating_add(64 * 1024);
        if stored_len_u64 > stored_limit {
            return Err(ZcadError::LimitExceeded {
                what: "stored section length",
                limit: stored_limit,
                actual: stored_len_u64,
            });
        }
        let stored_len = usize::try_from(stored_len_u64).map_err(|_| ZcadError::LimitExceeded {
            what: "stored section allocation",
            limit: usize::MAX as u64,
            actual: stored_len_u64,
        })?;
        let uncompressed_len =
            usize::try_from(uncompressed_len_u64).map_err(|_| ZcadError::LimitExceeded {
                what: "decoded section allocation",
                limit: usize::MAX as u64,
                actual: uncompressed_len_u64,
            })?;
        let end = offset
            .checked_add(stored_len_u64)
            .ok_or(ZcadError::Truncated)?;
        if offset < table_end as u64 || end > file_len {
            return Err(ZcadError::Truncated);
        }
        ranges.push((offset, end, id));

        if disposable {
            decoded_accelerator_total = decoded_accelerator_total
                .checked_add(uncompressed_len_u64)
                .ok_or(ZcadError::LimitExceeded {
                    what: "decoded accelerator total",
                    limit: options.limits.max_accelerator_bytes,
                    actual: u64::MAX,
                })?;
            if decoded_accelerator_total > options.limits.max_accelerator_bytes {
                return Err(ZcadError::LimitExceeded {
                    what: "decoded accelerator total",
                    limit: options.limits.max_accelerator_bytes,
                    actual: decoded_accelerator_total,
                });
            }
        }
        if matches!(
            id,
            SEC_MESH_CACHE | SEC_HYDRATED_CHECKPOINTS | SEC_LARGE_PREVIEW
        ) {
            accelerator_stored_total = accelerator_stored_total
                .checked_add(stored_len_u64)
                .ok_or(ZcadError::Truncated)?;
        }

        let mut expected_digest = [0u8; 16];
        expected_digest.copy_from_slice(&entry[28..44]);
        descriptors.push(SectionDescriptor {
            id,
            flags,
            codec,
            offset,
            stored_len,
            uncompressed_len,
            expected_digest,
        });
    }

    ranges.sort_unstable_by_key(|range| range.0);
    for pair in ranges.windows(2) {
        if pair[1].0 < pair[0].1 {
            return Err(ZcadError::OverlappingSections {
                first: pair[0].2,
                second: pair[1].2,
            });
        }
    }

    let mut sections = Vec::with_capacity(descriptors.len());
    let mut diagnostics = Vec::new();
    for descriptor in descriptors {
        if !known_section(descriptor.id) {
            continue;
        }
        reader
            .seek(SeekFrom::Start(descriptor.offset))
            .map_err(ZcadError::Io)?;
        let mut stored = Vec::new();
        stored
            .try_reserve_exact(descriptor.stored_len)
            .map_err(|_| ZcadError::LimitExceeded {
                what: "stored section allocation",
                limit: usize::MAX as u64,
                actual: descriptor.stored_len as u64,
            })?;
        stored.resize(descriptor.stored_len, 0);
        read_exact_or_truncated(reader, &mut stored)?;
        let digest = blake3::hash(&stored);
        if descriptor.expected_digest != digest.as_bytes()[..16] {
            if descriptor.flags & SECTION_REQUIRED != 0 {
                return Err(ZcadError::BadChecksum {
                    section: descriptor.id,
                });
            }
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: descriptor.id,
                reason: "integrity digest mismatch".to_owned(),
            });
            continue;
        }
        sections.push(SectionRef {
            id: descriptor.id,
            flags: descriptor.flags,
            codec: descriptor.codec,
            stored,
            uncompressed_len: descriptor.uncompressed_len,
        });
    }

    // Decode each section into its slot. Unknown ids are skipped silently.
    let mut metadata = ZcadMetadata::default();
    let mut recipe: Option<DocumentRecipeV3> = None;
    let mut recipe_bytes: Option<Vec<u8>> = None;
    let mut required_assets = RequiredAssetsV1::default();
    let mut thumbnail_png: Option<Vec<u8>> = None;
    let mut large_preview_png: Option<Vec<u8>> = None;
    let mut mesh_payload: Option<MeshCachePayload> = None;
    let mut hidden_nodes: HashSet<String> = HashSet::new();
    let mut hydrated_payload: Option<HydratedCheckpointPayloadV1> = None;

    for s in &sections {
        match s.id {
            SEC_METADATA => {
                if s.flags & SECTION_REQUIRED == 0 {
                    return Err(ZcadError::Decode("manifest is not marked required".into()));
                }
                let raw = decode_section(s)?;
                metadata = cbor_from_slice(&raw)?;
            }
            SEC_GRAPH => {
                if s.flags & SECTION_REQUIRED == 0 {
                    return Err(ZcadError::Decode("recipe is not marked required".into()));
                }
                let raw = decode_section(s)?;
                recipe = Some(cbor_from_slice(&raw)?);
                recipe_bytes = Some(raw);
            }
            SEC_REQUIRED_ASSETS => {
                if s.flags & SECTION_REQUIRED == 0 {
                    return Err(ZcadError::Decode(
                        "required assets are not marked required".into(),
                    ));
                }
                let raw = decode_section(s)?;
                required_assets = cbor_from_slice(&raw)?;
            }
            SEC_THUMBNAIL => match decode_section(s) {
                Ok(raw) => thumbnail_png = Some(raw),
                Err(error) => diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                    section: s.id,
                    reason: error.to_string(),
                }),
            },
            SEC_LARGE_PREVIEW => match decode_section(s) {
                Ok(raw) => large_preview_png = Some(raw),
                Err(error) => diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                    section: s.id,
                    reason: error.to_string(),
                }),
            },
            SEC_MESH_CACHE => match decode_section(s).and_then(|raw| cbor_from_slice(&raw)) {
                Ok(payload) => mesh_payload = Some(payload),
                Err(error) => diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                    section: s.id,
                    reason: error.to_string(),
                }),
            },
            SEC_HIDDEN_NODES => {
                if s.flags & SECTION_REQUIRED == 0 {
                    return Err(ZcadError::Decode(
                        "document state is not marked required".into(),
                    ));
                }
                let raw = decode_section(s)?;
                hidden_nodes = cbor_from_slice(&raw)?;
            }
            SEC_HYDRATED_CHECKPOINTS => {
                match decode_section(s).and_then(|raw| cbor_from_slice(&raw)) {
                    Ok(payload) => hydrated_payload = Some(payload),
                    Err(error) => diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                        section: s.id,
                        reason: error.to_string(),
                    }),
                }
            }
            _ if s.flags & SECTION_REQUIRED != 0 => {
                return Err(ZcadError::UnknownRequiredSection(s.id));
            }
            _ => {}
        }
    }

    if metadata.format_version != CURRENT_VERSION {
        return Err(ZcadError::Decode(format!(
            "manifest version {} does not match container version {CURRENT_VERSION}",
            metadata.format_version
        )));
    }
    if metadata.profile != profile_tag {
        return Err(ZcadError::Decode(
            "manifest profile does not match container header".into(),
        ));
    }
    if metadata.recipe_schema != DOCUMENT_RECIPE_SCHEMA
        || metadata.feature_payload_abi != FEATURE_PAYLOAD_ABI
        || metadata.required_assets_abi != REQUIRED_ASSETS_SCHEMA
        || metadata.tolerance_policy_abi != TOLERANCE_POLICY_ABI
    {
        return Err(ZcadError::Decode(format!(
            "unsupported authoritative ABI set (recipe={}, payload={}, assets={}, tolerance={})",
            metadata.recipe_schema,
            metadata.feature_payload_abi,
            metadata.required_assets_abi,
            metadata.tolerance_policy_abi
        )));
    }
    let profile = SaveProfile::from_header(profile_tag, metadata.accelerator_budget)?;

    if thumbnail_png
        .as_ref()
        .is_some_and(|preview| preview.len() > MAX_TINY_PREVIEW_BYTES)
    {
        thumbnail_png = None;
        diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
            section: SEC_THUMBNAIL,
            reason: "tiny preview exceeds the 32 KiB profile cap".into(),
        });
    }

    let profile_allows_accelerators = match profile {
        SaveProfile::Compact => false,
        SaveProfile::Hydrated {
            total_accelerator_budget,
        } => accelerator_stored_total <= total_accelerator_budget,
    };
    if !profile_allows_accelerators {
        let reason = match profile {
            SaveProfile::Compact => "compact profile forbids accelerator sections".to_owned(),
            SaveProfile::Hydrated {
                total_accelerator_budget,
            } => format!(
                "accelerator sections exceed the embedded budget ({accelerator_stored_total} > {total_accelerator_budget})"
            ),
        };
        if mesh_payload.take().is_some() {
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: SEC_MESH_CACHE,
                reason: reason.clone(),
            });
        }
        if hydrated_payload.take().is_some() {
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: SEC_HYDRATED_CHECKPOINTS,
                reason: reason.clone(),
            });
        }
        if large_preview_png.take().is_some() {
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: SEC_LARGE_PREVIEW,
                reason,
            });
        }
    }

    let recipe_bytes_ref = recipe_bytes
        .as_deref()
        .ok_or_else(|| ZcadError::Decode("file has no document recipe bytes".into()))?;
    if metadata.model_hash != model_hash(recipe_bytes_ref, &required_assets) {
        return Err(ZcadError::Decode(
            "manifest model hash does not match recipe and required assets".into(),
        ));
    }
    if metadata.presentation_hash != presentation_hash(metadata.units, &hidden_nodes)? {
        return Err(ZcadError::Decode(
            "manifest presentation hash does not match document state".into(),
        ));
    }
    let graph = recipe
        .ok_or_else(|| ZcadError::Decode("file has no document recipe".into()))?
        .into_graph_with_assets(&required_assets)?;

    // Keep derived data only when its independent ABI, recipe identity, body
    // identity, buffer shape, and finite-value checks all pass.
    let mesh_cache = mesh_payload.and_then(|payload| {
        let reason = if metadata.tessellation_abi != MESH_CACHE_ABI
            || payload.mesh_cache_abi != MESH_CACHE_ABI
        {
            Some("tessellation ABI mismatch".to_owned())
        } else if !mesh_cache_fresh(payload.recipe_hash, recipe_bytes_ref) {
            Some("mesh cache is stale for this recipe".to_owned())
        } else {
            mesh_cache_valid(&payload.bodies, &graph).err()
        };
        if let Some(reason) = reason {
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: SEC_MESH_CACHE,
                reason,
            });
            None
        } else {
            Some(payload.bodies)
        }
    });

    let evaluation_cache = hydrated_payload.and_then(|payload| {
        let reason = if metadata.openrcad_cache_abi != OPENRCAD_CACHE_ABI
            || payload.openrcad_cache_abi != OPENRCAD_CACHE_ABI
            || payload.schema_version != HYDRATED_CACHE_SCHEMA
        {
            Some("OpenRCAD checkpoint ABI mismatch".to_owned())
        } else if payload.recipe_hash != recipe_hash(recipe_bytes_ref) {
            Some("evaluation checkpoints are stale for this recipe".to_owned())
        } else {
            None
        };
        if let Some(reason) = reason {
            diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                section: SEC_HYDRATED_CHECKPOINTS,
                reason,
            });
            return None;
        }
        match decode_hydrated_cache(payload) {
            Some(mut snapshot) => {
                restore_final_checkpoint_meshes(&mut snapshot, mesh_cache.as_deref());
                Some(snapshot)
            }
            None => {
                diagnostics.push(LoadDiagnostic::DiscardedDisposableSection {
                    section: SEC_HYDRATED_CHECKPOINTS,
                    reason: "checkpoint content is corrupt or contains unhealthy solids".into(),
                });
                None
            }
        }
    });

    Ok(LoadedZcad {
        graph,
        metadata,
        thumbnail_png,
        large_preview_png,
        mesh_cache,
        hidden_nodes,
        evaluation_cache,
        profile,
        diagnostics,
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
        cbor_to_vec(&DocumentRecipeV3::from_graph(&pg)).unwrap()
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

    #[test]
    fn identical_step_imports_share_one_required_asset() {
        let mut graph = ParametricGraph::new();
        for id in ["import_1", "import_2"] {
            graph.add_feature(FeatureNode {
                id: id.into(),
                name: id.into(),
                feature: FeatureType::Import {
                    step_data: "ISO-10303-21;END-ISO-10303-21;".into(),
                    label: id.into(),
                },
            });
        }
        let (recipe, assets) = DocumentRecipeV3::from_graph_with_assets(&graph);
        assert_eq!(assets.blobs.len(), 1);
        let restored = recipe.into_graph_with_assets(&assets).unwrap();
        let imports: Vec<_> = restored
            .graph
            .node_weights()
            .filter_map(|node| match &node.feature {
                FeatureType::Import { step_data, .. } => Some(step_data.as_str()),
                _ => None,
            })
            .collect();
        assert_eq!(
            imports,
            [
                "ISO-10303-21;END-ISO-10303-21;",
                "ISO-10303-21;END-ISO-10303-21;"
            ]
        );
    }

    #[test]
    fn binary_stl_imports_round_trip_as_deduplicated_required_assets() {
        let bytes = vec![0, 159, 146, 150, 255, 0, 42];
        let mut graph = ParametricGraph::new();
        for id in ["mesh_1", "mesh_2"] {
            graph.add_feature(FeatureNode {
                id: id.into(),
                name: id.into(),
                feature: FeatureType::ImportStl {
                    stl_data: bytes.clone(),
                    label: format!("{id}.stl"),
                },
            });
        }
        let (recipe, assets) = DocumentRecipeV3::from_graph_with_assets(&graph);
        assert_eq!(
            assets.blobs.len(),
            1,
            "identical mesh bytes are deduplicated"
        );
        let restored = recipe.into_graph_with_assets(&assets).unwrap();
        let meshes: Vec<_> = restored
            .graph
            .node_weights()
            .filter_map(|node| match &node.feature {
                FeatureType::ImportStl { stl_data, .. } => Some(stl_data.as_slice()),
                _ => None,
            })
            .collect();
        assert_eq!(meshes, [bytes.as_slice(), bytes.as_slice()]);
    }

    #[test]
    fn recipe_rejects_unregistered_payload_schemas_before_graph_construction() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 1.0,
                h: 2.0,
                d: 3.0,
            },
        });
        let (mut recipe, assets) = DocumentRecipeV3::from_graph_with_assets(&graph);
        let box_record = recipe
            .features
            .iter_mut()
            .find(|record| record.id == "box")
            .expect("box record");
        box_record.payload_schema = 2;

        let error = recipe
            .into_graph_with_assets(&assets)
            .expect_err("unknown newer feature schema must fail");
        assert!(matches!(
            error,
            ZcadError::Decode(message)
                if message.contains("unsupported payload schema 2")
                    && message.contains("part.box")
                    && message.contains("supported schemas: [1]")
        ));
    }

    #[test]
    fn two_phase_decode_accepts_an_injected_test_only_schema_table() {
        use crate::document::{
            FeatureEditorGroup, FeatureEvaluatorKind, FeaturePayloadDecoder,
            FeaturePayloadDecoderRegistration, FeatureRegistration, FeatureRegistry,
        };

        static TEST_DECODERS: &[FeaturePayloadDecoderRegistration] = &[
            FeaturePayloadDecoderRegistration {
                schema: 1,
                decoder: FeaturePayloadDecoder::NumericFieldsV1,
            },
            FeaturePayloadDecoderRegistration {
                schema: 2,
                decoder: FeaturePayloadDecoder::NumericFieldsV1,
            },
        ];
        static TEST_BOX_REGISTRATION: FeatureRegistration = FeatureRegistration {
            kind_id: "part.box",
            payload_version: 1,
            payload_decoders: TEST_DECODERS,
            display_name: "Box",
            evaluator: FeatureEvaluatorKind::Box,
            editor_group: FeatureEditorGroup::Solid,
        };

        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 1.0,
                h: 2.0,
                d: 3.0,
            },
        });
        let (mut recipe, assets) = DocumentRecipeV3::from_graph_with_assets(&graph);
        recipe
            .features
            .iter_mut()
            .find(|record| record.id == "box")
            .expect("box record")
            .payload_schema = 2;

        let restored = recipe
            .into_graph_with_assets_and_registry(&assets, |kind| {
                if kind == "part.box" {
                    Some(&TEST_BOX_REGISTRATION)
                } else {
                    FeatureRegistry::get(kind)
                }
            })
            .expect("the injected schema must pass the real two-phase reader");
        let box_record = restored
            .graph
            .node_weights()
            .find(|record| record.id == "box")
            .expect("decoded box");
        assert_eq!(
            box_record.payload_version, 1,
            "the test-only schema is normalized to the real current schema"
        );
        assert!(matches!(
            box_record.feature,
            FeatureType::Box {
                w: 1.0,
                h: 2.0,
                d: 3.0
            }
        ));
        assert_eq!(FeatureRegistry::get("part.box").unwrap().payload_version, 1);
    }

    #[test]
    fn negative_zero_is_canonicalized() {
        assert_eq!(box_cbor(-0.0), box_cbor(0.0));
    }

    #[test]
    fn recipe_and_feature_payload_use_numeric_cbor_keys() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 1.0,
                h: 2.0,
                d: 3.0,
            },
        });
        let bytes = cbor_to_vec(&DocumentRecipeV3::from_graph(&graph)).unwrap();
        let value: ciborium::value::Value = ciborium::from_reader(bytes.as_slice()).unwrap();
        let ciborium::value::Value::Map(recipe) = value else {
            panic!("recipe must be a numeric-key map");
        };
        assert!(recipe
            .iter()
            .all(|(key, _)| matches!(key, ciborium::value::Value::Integer(_))));
        let features = recipe
            .iter()
            .find_map(|(key, value)| match key {
                ciborium::value::Value::Integer(key) if u64::try_from(*key).ok() == Some(1) => {
                    Some(value)
                }
                _ => None,
            })
            .unwrap();
        let ciborium::value::Value::Array(features) = features else {
            panic!("features must be an array");
        };
        let ciborium::value::Value::Map(box_record) = &features[1] else {
            panic!("feature record must be a numeric-key map");
        };
        assert!(box_record
            .iter()
            .all(|(key, _)| matches!(key, ciborium::value::Value::Integer(_))));
        let payload = box_record
            .iter()
            .find_map(|(key, value)| match key {
                ciborium::value::Value::Integer(key) if u64::try_from(*key).ok() == Some(4) => {
                    Some(value)
                }
                _ => None,
            })
            .unwrap();
        let ciborium::value::Value::Array(payload) = payload else {
            panic!("payload must be a numeric tuple");
        };
        let ciborium::value::Value::Map(fields) = &payload[1] else {
            panic!("payload fields must be a numeric-key map");
        };
        assert!(fields
            .iter()
            .all(|(key, _)| matches!(key, ciborium::value::Value::Integer(_))));
    }
}
