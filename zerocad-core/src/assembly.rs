//! Authoritative assembly document contracts.
//!
//! Assembly geometry is deliberately absent from these types. Definitions own
//! compact, authoritative Part snapshots; evaluators and renderers build
//! disposable geometry pools from those snapshots.

use crate::{Document, ScenePlacement, Unit};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::fmt;
use std::sync::Arc;

pub type ModelHash = [u8; 32];
pub type AssetHash = [u8; 32];
pub type OccurrenceId = u64;

const QUATERNION_SIGN_EPSILON: f64 = 1.0e-12;
const QUATERNION_NORM_EPSILON: f64 = 1.0e-15;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ProjectKind {
    Part = 0,
    Assembly = 1,
}

impl ProjectKind {
    pub fn from_header_bits(bits: u8) -> Result<Self, AssemblyError> {
        match bits {
            0 => Ok(Self::Part),
            1 => Ok(Self::Assembly),
            other => Err(AssemblyError::UnsupportedProjectKind(other)),
        }
    }
}

/// A canonical f64 rigid-body placement.
///
/// Quaternion storage order is `[w, x, y, z]`. Construction and
/// deserialization normalize it, canonicalize its double-cover sign, and erase
/// signed zero so equal placements serialize and hash identically.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidPlacement {
    translation: [f64; 3],
    rotation: [f64; 4],
}

impl RigidPlacement {
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [1.0, 0.0, 0.0, 0.0],
    };

    pub fn new(mut translation: [f64; 3], mut rotation: [f64; 4]) -> Result<Self, AssemblyError> {
        if !translation
            .iter()
            .chain(rotation.iter())
            .all(|value| value.is_finite())
        {
            return Err(AssemblyError::NonFinitePlacement);
        }
        let norm_squared = rotation.iter().map(|value| value * value).sum::<f64>();
        if norm_squared <= QUATERNION_NORM_EPSILON * QUATERNION_NORM_EPSILON {
            return Err(AssemblyError::DegenerateQuaternion);
        }
        let inverse_norm = norm_squared.sqrt().recip();
        for component in &mut rotation {
            *component *= inverse_norm;
        }

        let sign_component = rotation
            .iter()
            .copied()
            .find(|component| component.abs() > QUATERNION_SIGN_EPSILON)
            .unwrap_or(1.0);
        if sign_component.is_sign_negative() {
            for component in &mut rotation {
                *component = -*component;
            }
        }
        canonicalize_signed_zero(&mut translation);
        canonicalize_signed_zero(&mut rotation);
        Ok(Self {
            translation,
            rotation,
        })
    }

    pub fn translation(self) -> [f64; 3] {
        self.translation
    }

    pub fn rotation(self) -> [f64; 4] {
        self.rotation
    }

    pub fn with_translation(self, translation: [f64; 3]) -> Result<Self, AssemblyError> {
        Self::new(translation, self.rotation)
    }

    /// Rotate this placement about a normalized axis. In world space the
    /// delta pre-multiplies the authored rotation; in local space it
    /// post-multiplies it.
    pub fn rotated_about_axis(
        self,
        axis: [f64; 3],
        radians: f64,
        world_space: bool,
    ) -> Result<Self, AssemblyError> {
        if !radians.is_finite() || !axis.iter().all(|value| value.is_finite()) {
            return Err(AssemblyError::NonFinitePlacement);
        }
        let length = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
        if length <= QUATERNION_NORM_EPSILON {
            return Err(AssemblyError::DegenerateQuaternion);
        }
        let [ax, ay, az] = axis.map(|value| value / length);
        let (sine, cosine) = (0.5 * radians).sin_cos();
        let delta = [cosine, ax * sine, ay * sine, az * sine];
        let rotation = if world_space {
            quaternion_product(delta, self.rotation)
        } else {
            quaternion_product(self.rotation, delta)
        };
        Self::new(self.translation, rotation)
    }

    /// Build an intrinsic XYZ rotation from UI-friendly degrees.
    pub fn from_euler_xyz_degrees(
        translation: [f64; 3],
        euler_degrees: [f64; 3],
    ) -> Result<Self, AssemblyError> {
        if !euler_degrees.iter().all(|angle| angle.is_finite()) {
            return Err(AssemblyError::NonFinitePlacement);
        }
        let [rx, ry, rz] = euler_degrees.map(f64::to_radians);
        let (sx, cx) = (rx * 0.5).sin_cos();
        let (sy, cy) = (ry * 0.5).sin_cos();
        let (sz, cz) = (rz * 0.5).sin_cos();
        Self::new(
            translation,
            [
                cx * cy * cz + sx * sy * sz,
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
            ],
        )
    }

    /// Intrinsic XYZ Euler angles in degrees. This is a presentation helper;
    /// the quaternion remains authoritative at the gimbal seam.
    pub fn euler_xyz_degrees(self) -> [f64; 3] {
        let [w, x, y, z] = self.rotation;
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let pitch_sine = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
        let pitch = pitch_sine.asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()]
    }

    pub fn is_identity(self) -> bool {
        self == Self::IDENTITY
    }

    /// Convert once at the authoritative-to-evaluated scene boundary.
    pub fn to_scene_placement(self) -> ScenePlacement {
        let [w, x, y, z] = self.rotation;
        let linear = [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
        .map(|row| row.map(|value| value as f32));
        let translation = self.translation.map(|value| value as f32);
        ScenePlacement::from_rotation_translation(linear, translation)
            .expect("a normalized quaternion must produce a rigid scene placement")
    }
}

fn quaternion_product(first: [f64; 4], second: [f64; 4]) -> [f64; 4] {
    let [aw, ax, ay, az] = first;
    let [bw, bx, by, bz] = second;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

#[derive(Serialize, Deserialize)]
struct RigidPlacementWire {
    translation: [f64; 3],
    rotation: [f64; 4],
}

impl Serialize for RigidPlacement {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        RigidPlacementWire {
            translation: self.translation,
            rotation: self.rotation,
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for RigidPlacement {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = RigidPlacementWire::deserialize(deserializer)?;
        Self::new(wire.translation, wire.rotation).map_err(serde::de::Error::custom)
    }
}

impl Default for RigidPlacement {
    fn default() -> Self {
        Self::IDENTITY
    }
}

fn canonicalize_signed_zero<const N: usize>(values: &mut [f64; N]) {
    for value in values {
        if *value == 0.0 {
            *value = 0.0;
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssemblyDefinition {
    pub model_hash: ModelHash,
    pub name: String,
    pub source_basename: Option<String>,
    pub compact_snapshot: Arc<Vec<u8>>,
}

impl AssemblyDefinition {
    pub fn snapshot_asset_hash(&self) -> AssetHash {
        *blake3::hash(&self.compact_snapshot).as_bytes()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct AssemblyOccurrence {
    pub id: OccurrenceId,
    pub definition_model_hash: ModelHash,
    pub name: String,
    pub manual_placement: RigidPlacement,
    pub grounded: bool,
    /// Derived V2 solver output. This is never serialized; `None` is the V1
    /// boundary where resolved and manual placement are equal.
    pub resolved_placement_override: Option<RigidPlacement>,
}

impl AssemblyOccurrence {
    /// V1's solver boundary: downstream consumers never read
    /// `manual_placement` directly.
    pub fn resolved_placement(&self) -> RigidPlacement {
        self.resolved_placement_override
            .unwrap_or(self.manual_placement)
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssemblyPresentationV1 {
    pub units: Unit,
    pub hidden_occurrences: BTreeSet<OccurrenceId>,
    pub hidden_bodies: BTreeSet<(OccurrenceId, String)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AssemblyDocument {
    pub definitions: BTreeMap<ModelHash, Arc<AssemblyDefinition>>,
    pub occurrences: BTreeMap<OccurrenceId, AssemblyOccurrence>,
    pub next_occurrence_id: OccurrenceId,
    pub presentation: AssemblyPresentationV1,
    pub created_unix: Option<u64>,
    /// `None` is an AssemblyRecipeV1 document. `Some` opts into the strictly
    /// versioned V2 mate schema.
    pub mates: Option<crate::AssemblyMateSet>,
}

impl Default for AssemblyDocument {
    fn default() -> Self {
        Self::new()
    }
}

impl AssemblyDocument {
    pub fn new() -> Self {
        Self {
            definitions: BTreeMap::new(),
            occurrences: BTreeMap::new(),
            next_occurrence_id: 1,
            presentation: AssemblyPresentationV1 {
                units: Unit::Millimeter,
                ..AssemblyPresentationV1::default()
            },
            created_unix: None,
            mates: None,
        }
    }

    /// Clone authoritative state while retaining Arc-shared immutable snapshots.
    pub fn clone_authoritative(&self) -> Self {
        let mut cloned = self.clone();
        for occurrence in cloned.occurrences.values_mut() {
            occurrence.resolved_placement_override = None;
        }
        cloned
    }

    pub fn validate_structural_contracts(&self) -> Result<(), AssemblyError> {
        const MAX_DEFINITIONS: usize = 2_048;
        const MAX_OCCURRENCES: usize = 10_000;
        if self.definitions.len() > MAX_DEFINITIONS {
            return Err(AssemblyError::DefinitionLimitExceeded {
                actual: self.definitions.len(),
                limit: MAX_DEFINITIONS,
            });
        }
        if self.occurrences.len() > MAX_OCCURRENCES {
            return Err(AssemblyError::OccurrenceLimitExceeded {
                actual: self.occurrences.len(),
                limit: MAX_OCCURRENCES,
            });
        }

        let mut definition_names = HashSet::with_capacity(self.definitions.len());
        for (key, definition) in &self.definitions {
            if key != &definition.model_hash {
                return Err(AssemblyError::DefinitionKeyMismatch);
            }
            validate_name(&definition.name)?;
            if !definition_names.insert(definition.name.to_lowercase()) {
                return Err(AssemblyError::DuplicateDefinitionName(
                    definition.name.clone(),
                ));
            }
            if definition.compact_snapshot.is_empty() {
                return Err(AssemblyError::EmptyDefinitionSnapshot(
                    definition.name.clone(),
                ));
            }
        }

        let mut occurrence_names = HashSet::with_capacity(self.occurrences.len());
        for (key, occurrence) in &self.occurrences {
            if *key != occurrence.id {
                return Err(AssemblyError::OccurrenceKeyMismatch);
            }
            validate_name(&occurrence.name)?;
            if !occurrence_names.insert(occurrence.name.to_lowercase()) {
                return Err(AssemblyError::DuplicateOccurrenceName(
                    occurrence.name.clone(),
                ));
            }
            if !self
                .definitions
                .contains_key(&occurrence.definition_model_hash)
            {
                return Err(AssemblyError::MissingDefinition(occurrence.id));
            }
        }

        if let Some(maximum) = self.occurrences.keys().next_back().copied() {
            if self.next_occurrence_id <= maximum {
                return Err(AssemblyError::InvalidNextOccurrenceId {
                    next: self.next_occurrence_id,
                    maximum,
                });
            }
        } else if self.next_occurrence_id == 0 {
            return Err(AssemblyError::InvalidNextOccurrenceId {
                next: 0,
                maximum: 0,
            });
        }

        for id in &self.presentation.hidden_occurrences {
            if !self.occurrences.contains_key(id) {
                return Err(AssemblyError::DanglingHiddenOccurrence(*id));
            }
        }
        // Body selectors intentionally are not checked against evaluated
        // geometry. A definition may fail to evaluate while the authoritative
        // assembly remains loadable; those selectors are retained and diagnosed
        // by hydration/evaluation instead.
        for (id, _) in &self.presentation.hidden_bodies {
            if !self.occurrences.contains_key(id) {
                return Err(AssemblyError::DanglingHiddenBodyOccurrence(*id));
            }
        }
        if let Some(mates) = &self.mates {
            mates
                .validate(self)
                .map_err(|error| AssemblyError::InvalidMateSet(error.to_string()))?;
        }
        Ok(())
    }
}

fn validate_name(name: &str) -> Result<(), AssemblyError> {
    if name.trim().is_empty() {
        Err(AssemblyError::EmptyName)
    } else {
        Ok(())
    }
}

#[allow(clippy::large_enum_variant)] // The public boundary intentionally keeps Part(Document) direct.
#[derive(Debug, Clone)]
pub enum ProjectDocument {
    Part(Document),
    Assembly(AssemblyDocument),
}

impl ProjectDocument {
    pub fn kind(&self) -> ProjectKind {
        match self {
            Self::Part(_) => ProjectKind::Part,
            Self::Assembly(_) => ProjectKind::Assembly,
        }
    }

    pub fn clone_authoritative(&self) -> Self {
        match self {
            Self::Part(document) => Self::Part(document.clone_authoritative()),
            Self::Assembly(document) => Self::Assembly(document.clone_authoritative()),
        }
    }

    pub fn units(&self) -> Unit {
        match self {
            Self::Part(document) => document.state.units,
            Self::Assembly(document) => document.presentation.units,
        }
    }

    pub fn created_unix(&self) -> Option<u64> {
        match self {
            Self::Part(document) => document.state.created_unix,
            Self::Assembly(document) => document.created_unix,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssemblyError {
    UnsupportedProjectKind(u8),
    NonFinitePlacement,
    DegenerateQuaternion,
    EmptyName,
    DefinitionKeyMismatch,
    OccurrenceKeyMismatch,
    DuplicateDefinitionName(String),
    DuplicateOccurrenceName(String),
    EmptyDefinitionSnapshot(String),
    MissingDefinition(OccurrenceId),
    InvalidNextOccurrenceId {
        next: OccurrenceId,
        maximum: OccurrenceId,
    },
    DanglingHiddenOccurrence(OccurrenceId),
    DanglingHiddenBodyOccurrence(OccurrenceId),
    DefinitionLimitExceeded {
        actual: usize,
        limit: usize,
    },
    OccurrenceLimitExceeded {
        actual: usize,
        limit: usize,
    },
    InvalidMateSet(String),
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedProjectKind(kind) => {
                write!(formatter, "unsupported project kind {kind}")
            }
            Self::NonFinitePlacement => formatter.write_str("placement contains non-finite values"),
            Self::DegenerateQuaternion => formatter.write_str("placement quaternion is degenerate"),
            Self::EmptyName => formatter.write_str("assembly names cannot be empty"),
            Self::DefinitionKeyMismatch => {
                formatter.write_str("definition map key does not match its model hash")
            }
            Self::OccurrenceKeyMismatch => {
                formatter.write_str("occurrence map key does not match its id")
            }
            Self::DuplicateDefinitionName(name) => {
                write!(formatter, "duplicate definition name `{name}`")
            }
            Self::DuplicateOccurrenceName(name) => {
                write!(formatter, "duplicate occurrence name `{name}`")
            }
            Self::EmptyDefinitionSnapshot(name) => {
                write!(formatter, "definition `{name}` has an empty Part snapshot")
            }
            Self::MissingDefinition(id) => {
                write!(formatter, "occurrence {id} references a missing definition")
            }
            Self::InvalidNextOccurrenceId { next, maximum } => write!(
                formatter,
                "next occurrence id {next} must be greater than existing maximum {maximum}"
            ),
            Self::DanglingHiddenOccurrence(id) => {
                write!(formatter, "visibility references missing occurrence {id}")
            }
            Self::DanglingHiddenBodyOccurrence(id) => write!(
                formatter,
                "body visibility references missing occurrence {id}"
            ),
            Self::DefinitionLimitExceeded { actual, limit } => {
                write!(formatter, "definition count {actual} exceeds limit {limit}")
            }
            Self::OccurrenceLimitExceeded { actual, limit } => {
                write!(formatter, "occurrence count {actual} exceeds limit {limit}")
            }
            Self::InvalidMateSet(error) => write!(formatter, "invalid mate set: {error}"),
        }
    }
}

impl std::error::Error for AssemblyError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_double_cover_and_signed_zero_canonicalize() {
        let first = RigidPlacement::new([-0.0, 2.0, -0.0], [0.0, -1.0, 0.0, 0.0]).unwrap();
        let second = RigidPlacement::new([0.0, 2.0, 0.0], [-0.0, 1.0, -0.0, -0.0]).unwrap();
        assert_eq!(first, second);
        assert_eq!(
            first.translation().map(f64::to_bits),
            [0, 2.0f64.to_bits(), 0]
        );
        assert_eq!(first.rotation(), [0.0, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn quaternion_sign_ignores_noisy_near_zero_w() {
        let first = RigidPlacement::new([0.0; 3], [1.0e-14, -1.0, 0.0, 0.0]).unwrap();
        let second = RigidPlacement::new([0.0; 3], [-1.0e-14, 1.0, 0.0, 0.0]).unwrap();
        assert_eq!(first.rotation()[1], 1.0);
        assert_eq!(second.rotation()[1], 1.0);
    }

    #[test]
    fn world_and_local_axis_rotations_use_the_documented_composition_order() {
        let authored = RigidPlacement::from_euler_xyz_degrees([0.0; 3], [0.0, 0.0, 90.0]).unwrap();
        let world = authored
            .rotated_about_axis([1.0, 0.0, 0.0], 90.0f64.to_radians(), true)
            .unwrap();
        let local = authored
            .rotated_about_axis([1.0, 0.0, 0.0], 90.0f64.to_radians(), false)
            .unwrap();
        assert_ne!(world, local);
        for placement in [world, local] {
            let norm = placement
                .rotation()
                .iter()
                .map(|value| value * value)
                .sum::<f64>();
            assert!((norm - 1.0).abs() < 1.0e-12);
        }
    }

    #[test]
    fn next_occurrence_id_must_exceed_existing_ids() {
        let snapshot = Arc::new(vec![1]);
        let hash = [7; 32];
        let mut assembly = AssemblyDocument::new();
        assembly.definitions.insert(
            hash,
            Arc::new(AssemblyDefinition {
                model_hash: hash,
                name: "Bracket".into(),
                source_basename: None,
                compact_snapshot: snapshot,
            }),
        );
        assembly.occurrences.insert(
            9,
            AssemblyOccurrence {
                id: 9,
                definition_model_hash: hash,
                name: "Bracket:1".into(),
                manual_placement: RigidPlacement::IDENTITY,
                grounded: false,
                resolved_placement_override: None,
            },
        );
        assembly.next_occurrence_id = 9;
        assert!(matches!(
            assembly.validate_structural_contracts(),
            Err(AssemblyError::InvalidNextOccurrenceId {
                next: 9,
                maximum: 9
            })
        ));
    }

    #[test]
    fn authoritative_clone_drops_derived_resolved_placements() {
        let snapshot = Arc::new(vec![1]);
        let hash = [9; 32];
        let mut assembly = AssemblyDocument::new();
        assembly.definitions.insert(
            hash,
            Arc::new(AssemblyDefinition {
                model_hash: hash,
                name: "Bracket".into(),
                source_basename: None,
                compact_snapshot: snapshot,
            }),
        );
        assembly.occurrences.insert(
            1,
            AssemblyOccurrence {
                id: 1,
                definition_model_hash: hash,
                name: "Bracket:1".into(),
                manual_placement: RigidPlacement::IDENTITY,
                grounded: false,
                resolved_placement_override: Some(
                    RigidPlacement::new([5.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
                ),
            },
        );
        assembly.next_occurrence_id = 2;
        assert!(assembly.clone_authoritative().occurrences[&1]
            .resolved_placement_override
            .is_none());
    }
}
