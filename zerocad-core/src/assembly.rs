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

pub const MAX_ASSEMBLY_DEFINITIONS: usize = 2_048;
pub const MAX_ASSEMBLY_OCCURRENCES: usize = 10_000;

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
        self.clone().into_authoritative()
    }

    /// Strip disposable solved placements without cloning the document.
    pub fn into_authoritative(mut self) -> Self {
        for occurrence in self.occurrences.values_mut() {
            occurrence.resolved_placement_override = None;
        }
        self
    }

    pub fn validate_structural_contracts(&self) -> Result<(), AssemblyError> {
        if self.definitions.len() > MAX_ASSEMBLY_DEFINITIONS {
            return Err(AssemblyError::DefinitionLimitExceeded {
                actual: self.definitions.len(),
                limit: MAX_ASSEMBLY_DEFINITIONS,
            });
        }
        if self.occurrences.len() > MAX_ASSEMBLY_OCCURRENCES {
            return Err(AssemblyError::OccurrenceLimitExceeded {
                actual: self.occurrences.len(),
                limit: MAX_ASSEMBLY_OCCURRENCES,
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

    /// Stage one authoritative edit against an Arc-cheap clone.
    ///
    /// The returned transaction owns a structurally valid candidate and does
    /// not mutate this document. Callers may run additional fallible work with
    /// [`PreparedAssemblyCommand::finalize`] and should create an undo snapshot
    /// only after every such check succeeds. Prepared commands are synchronous
    /// edit objects and must not be held across another document mutation.
    pub fn prepare_command(
        &self,
        command: AssemblyCommand,
    ) -> Result<PreparedAssemblyCommand, AssemblyCommandError> {
        self.validate_structural_contracts()
            .map_err(AssemblyCommandError::InvalidAssembly)?;
        let mut candidate = self.clone();
        let changes = apply_command_to_candidate(&mut candidate, command)?;
        candidate
            .validate_structural_contracts()
            .map_err(AssemblyCommandError::InvalidAssembly)?;
        Ok(PreparedAssemblyCommand { candidate, changes })
    }
}

/// A small, typed mutation of authoritative assembly state.
///
/// Geometry preparation, definition replacement, and mate authoring retain
/// their dedicated transactional APIs. This command surface covers the shared
/// document edits that must produce consistent invalidation information for
/// the UI and the incremental assembly runtime.
#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyCommand {
    SetOccurrenceVisible {
        occurrence_id: OccurrenceId,
        visible: bool,
    },
    SetBodyVisible {
        occurrence_id: OccurrenceId,
        local_body_id: String,
        visible: bool,
    },
    SetOccurrenceGrounded {
        occurrence_id: OccurrenceId,
        grounded: bool,
    },
    RenameOccurrence {
        occurrence_id: OccurrenceId,
        name: String,
    },
    SetOccurrencePlacement {
        occurrence_id: OccurrenceId,
        placement: RigidPlacement,
    },
    DuplicateOccurrence {
        occurrence_id: OccurrenceId,
    },
    DeleteOccurrence {
        occurrence_id: OccurrenceId,
    },
    EditMate {
        mate_id: crate::MateId,
        edit: AssemblyMateEdit,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyMateEdit {
    Delete,
    SetSuppressed(bool),
    FlipSense,
    SetDistanceMillimeters(f64),
    SetAngleRadians(f64),
    RepairEntities {
        first: crate::AssemblyEntityRef,
        second: crate::AssemblyEntityRef,
    },
}

/// Exact invalidation produced by one committed assembly command.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AssemblyChangeSet {
    pub added_occurrences: BTreeSet<OccurrenceId>,
    pub removed_occurrences: BTreeSet<OccurrenceId>,
    pub modified_occurrences: BTreeSet<OccurrenceId>,
    pub removed_definitions: BTreeSet<ModelHash>,
    pub modified_mates: BTreeSet<crate::MateId>,
    pub removed_mates: BTreeSet<crate::MateId>,
    pub presentation_changed: bool,
    pub placements_changed: bool,
    pub solver_inputs_changed: bool,
    pub mate_graph_changed: bool,
}

impl AssemblyChangeSet {
    pub fn is_empty(&self) -> bool {
        self.added_occurrences.is_empty()
            && self.removed_occurrences.is_empty()
            && self.modified_occurrences.is_empty()
            && self.removed_definitions.is_empty()
            && self.modified_mates.is_empty()
            && self.removed_mates.is_empty()
            && !self.presentation_changed
            && !self.placements_changed
            && !self.solver_inputs_changed
            && !self.mate_graph_changed
    }

    pub fn requires_scene_rebuild(&self) -> bool {
        self.presentation_changed
            || self.placements_changed
            || self.solver_inputs_changed
            || !self.added_occurrences.is_empty()
            || !self.removed_occurrences.is_empty()
    }
}

/// A validated candidate which has not yet touched the live document.
#[derive(Debug)]
pub struct PreparedAssemblyCommand {
    candidate: AssemblyDocument,
    changes: AssemblyChangeSet,
}

impl PreparedAssemblyCommand {
    pub fn document(&self) -> &AssemblyDocument {
        &self.candidate
    }

    pub fn changes(&self) -> &AssemblyChangeSet {
        &self.changes
    }

    /// Run an additional fallible phase, such as a committed mate solve,
    /// against the staged document. Failure consumes the candidate and leaves
    /// the live document untouched.
    pub fn finalize<T, E, F>(
        mut self,
        finalize: F,
    ) -> Result<(Self, T), AssemblyCommandFinalizeError<E>>
    where
        F: FnOnce(&mut AssemblyDocument) -> Result<T, E>,
    {
        let output =
            finalize(&mut self.candidate).map_err(AssemblyCommandFinalizeError::Finalize)?;
        self.candidate
            .validate_structural_contracts()
            .map_err(AssemblyCommandFinalizeError::InvalidAssembly)?;
        Ok((self, output))
    }

    /// Commit a synchronously prepared command. This is infallible because the
    /// candidate was validated after both the command and its optional
    /// finalizer.
    pub fn commit(self, document: &mut AssemblyDocument) -> (AssemblyChangeSet, AssemblyDocument) {
        let previous = std::mem::replace(document, self.candidate);
        (self.changes, previous)
    }
}

fn apply_command_to_candidate(
    document: &mut AssemblyDocument,
    command: AssemblyCommand,
) -> Result<AssemblyChangeSet, AssemblyCommandError> {
    let mut changes = AssemblyChangeSet::default();
    match command {
        AssemblyCommand::SetOccurrenceVisible {
            occurrence_id,
            visible,
        } => {
            require_occurrence(document, occurrence_id)?;
            let changed = if visible {
                document
                    .presentation
                    .hidden_occurrences
                    .remove(&occurrence_id)
            } else {
                document
                    .presentation
                    .hidden_occurrences
                    .insert(occurrence_id)
            };
            if changed {
                changes.modified_occurrences.insert(occurrence_id);
                changes.presentation_changed = true;
            }
        }
        AssemblyCommand::SetBodyVisible {
            occurrence_id,
            local_body_id,
            visible,
        } => {
            require_occurrence(document, occurrence_id)?;
            if local_body_id.trim().is_empty() {
                return Err(AssemblyCommandError::EmptyLocalBodyId);
            }
            let selector = (occurrence_id, local_body_id);
            let changed = if visible {
                document.presentation.hidden_bodies.remove(&selector)
            } else {
                document.presentation.hidden_bodies.insert(selector)
            };
            if changed {
                changes.modified_occurrences.insert(occurrence_id);
                changes.presentation_changed = true;
            }
        }
        AssemblyCommand::SetOccurrenceGrounded {
            occurrence_id,
            grounded,
        } => {
            let occurrence = require_occurrence_mut(document, occurrence_id)?;
            if occurrence.grounded != grounded {
                occurrence.grounded = grounded;
                changes.modified_occurrences.insert(occurrence_id);
                changes.solver_inputs_changed = true;
            }
        }
        AssemblyCommand::RenameOccurrence {
            occurrence_id,
            name,
        } => {
            let name = name.trim().to_owned();
            validate_name(&name).map_err(AssemblyCommandError::InvalidAssembly)?;
            require_occurrence(document, occurrence_id)?;
            if document.occurrences.values().any(|occurrence| {
                occurrence.id != occurrence_id && occurrence.name.eq_ignore_ascii_case(&name)
            }) {
                return Err(AssemblyCommandError::OccurrenceNameCollision(name));
            }
            let occurrence = require_occurrence_mut(document, occurrence_id)?;
            if occurrence.name != name {
                occurrence.name = name;
                changes.modified_occurrences.insert(occurrence_id);
            }
        }
        AssemblyCommand::SetOccurrencePlacement {
            occurrence_id,
            placement,
        } => {
            let occurrence = require_occurrence_mut(document, occurrence_id)?;
            if occurrence.grounded {
                return Err(AssemblyCommandError::GroundedOccurrence(occurrence_id));
            }
            if occurrence.manual_placement != placement
                || occurrence.resolved_placement_override.is_some()
            {
                occurrence.manual_placement = placement;
                occurrence.resolved_placement_override = None;
                changes.modified_occurrences.insert(occurrence_id);
                changes.placements_changed = true;
            }
        }
        AssemblyCommand::DuplicateOccurrence { occurrence_id } => {
            let source = require_occurrence(document, occurrence_id)?.clone();
            if document.occurrences.len() >= MAX_ASSEMBLY_OCCURRENCES {
                return Err(AssemblyCommandError::OccurrenceLimitReached);
            }
            let new_id = document.next_occurrence_id;
            let next_id = new_id
                .checked_add(1)
                .ok_or(AssemblyCommandError::OccurrenceIdExhausted)?;
            if document.occurrences.contains_key(&new_id) {
                return Err(AssemblyCommandError::OccurrenceIdCollision(new_id));
            }
            let definition_name = &document
                .definitions
                .get(&source.definition_model_hash)
                .ok_or(AssemblyCommandError::MissingDefinition(
                    source.definition_model_hash,
                ))?
                .name;
            let name = format!("{definition_name}:{new_id}");
            if document
                .occurrences
                .values()
                .any(|occurrence| occurrence.name.eq_ignore_ascii_case(&name))
            {
                return Err(AssemblyCommandError::OccurrenceNameCollision(name));
            }

            document.occurrences.insert(
                new_id,
                AssemblyOccurrence {
                    id: new_id,
                    definition_model_hash: source.definition_model_hash,
                    name,
                    manual_placement: source.manual_placement,
                    grounded: false,
                    resolved_placement_override: None,
                },
            );
            document.next_occurrence_id = next_id;
            if document
                .presentation
                .hidden_occurrences
                .contains(&occurrence_id)
            {
                document.presentation.hidden_occurrences.insert(new_id);
                changes.presentation_changed = true;
            }
            let hidden_bodies: Vec<String> = document
                .presentation
                .hidden_bodies
                .iter()
                .filter(|(id, _)| *id == occurrence_id)
                .map(|(_, body_id)| body_id.clone())
                .collect();
            if !hidden_bodies.is_empty() {
                document
                    .presentation
                    .hidden_bodies
                    .extend(hidden_bodies.into_iter().map(|body_id| (new_id, body_id)));
                changes.presentation_changed = true;
            }
            changes.added_occurrences.insert(new_id);
        }
        AssemblyCommand::DeleteOccurrence { occurrence_id } => {
            let occurrence = require_occurrence(document, occurrence_id)?.clone();
            document.occurrences.remove(&occurrence_id);
            if document
                .presentation
                .hidden_occurrences
                .remove(&occurrence_id)
            {
                changes.presentation_changed = true;
            }
            let hidden_body_count = document.presentation.hidden_bodies.len();
            document
                .presentation
                .hidden_bodies
                .retain(|(id, _)| *id != occurrence_id);
            if document.presentation.hidden_bodies.len() != hidden_body_count {
                changes.presentation_changed = true;
            }
            if let Some(mate_set) = document.mates.as_mut() {
                let mate_count = mate_set.mates.len();
                mate_set.mates.retain(|_, mate| {
                    mate.first.occurrence_id != occurrence_id
                        && mate
                            .second
                            .as_ref()
                            .is_none_or(|entity| entity.occurrence_id != occurrence_id)
                });
                changes.mate_graph_changed = mate_set.mates.len() != mate_count;
                changes.solver_inputs_changed = changes.mate_graph_changed;
            }

            let definition_still_used = document.occurrences.values().any(|remaining| {
                remaining.definition_model_hash == occurrence.definition_model_hash
            });
            if !definition_still_used {
                document
                    .definitions
                    .remove(&occurrence.definition_model_hash);
                changes
                    .removed_definitions
                    .insert(occurrence.definition_model_hash);
            }
            changes.removed_occurrences.insert(occurrence_id);
        }
        AssemblyCommand::EditMate { mate_id, edit } => {
            let mate_set = document
                .mates
                .as_mut()
                .ok_or(AssemblyCommandError::MatesUnavailable)?;
            match edit {
                AssemblyMateEdit::Delete => {
                    if mate_set.mates.remove(&mate_id).is_none() {
                        return Err(AssemblyCommandError::MissingMate(mate_id));
                    }
                    changes.removed_mates.insert(mate_id);
                    changes.mate_graph_changed = true;
                    changes.solver_inputs_changed = true;
                }
                AssemblyMateEdit::SetSuppressed(suppressed) => {
                    let mate = mate_set
                        .mates
                        .get_mut(&mate_id)
                        .ok_or(AssemblyCommandError::MissingMate(mate_id))?;
                    if mate.suppressed != suppressed {
                        mate.suppressed = suppressed;
                        changes.modified_mates.insert(mate_id);
                        changes.solver_inputs_changed = true;
                    }
                }
                AssemblyMateEdit::FlipSense => {
                    let mate = mate_set
                        .mates
                        .get_mut(&mate_id)
                        .ok_or(AssemblyCommandError::MissingMate(mate_id))?;
                    mate.sense = match mate.sense {
                        crate::MateSense::Aligned => crate::MateSense::AntiAligned,
                        crate::MateSense::AntiAligned => crate::MateSense::Aligned,
                    };
                    changes.modified_mates.insert(mate_id);
                    changes.solver_inputs_changed = true;
                }
                AssemblyMateEdit::SetDistanceMillimeters(value) => {
                    let value = canonical_finite_mate_parameter(value)?;
                    let mate = mate_set
                        .mates
                        .get_mut(&mate_id)
                        .ok_or(AssemblyCommandError::MissingMate(mate_id))?;
                    let crate::AssemblyMateKind::SignedDistance { millimeters } = &mut mate.kind
                    else {
                        return Err(AssemblyCommandError::WrongMateKind(mate_id));
                    };
                    if *millimeters != value {
                        *millimeters = value;
                        changes.modified_mates.insert(mate_id);
                        changes.solver_inputs_changed = true;
                    }
                }
                AssemblyMateEdit::SetAngleRadians(value) => {
                    let value = canonical_finite_mate_parameter(value)?;
                    let mate = mate_set
                        .mates
                        .get_mut(&mate_id)
                        .ok_or(AssemblyCommandError::MissingMate(mate_id))?;
                    let crate::AssemblyMateKind::Angle { radians } = &mut mate.kind else {
                        return Err(AssemblyCommandError::WrongMateKind(mate_id));
                    };
                    if *radians != value {
                        *radians = value;
                        changes.modified_mates.insert(mate_id);
                        changes.solver_inputs_changed = true;
                    }
                }
                AssemblyMateEdit::RepairEntities { first, second } => {
                    if first.occurrence_id == second.occurrence_id {
                        return Err(AssemblyCommandError::SameOccurrenceMateEntities(
                            first.occurrence_id,
                        ));
                    }
                    let mate = mate_set
                        .mates
                        .get_mut(&mate_id)
                        .ok_or(AssemblyCommandError::MissingMate(mate_id))?;
                    if matches!(mate.kind, crate::AssemblyMateKind::Fixed) {
                        return Err(AssemblyCommandError::CannotRepairFixedMate(mate_id));
                    }
                    if mate.first != first || mate.second.as_ref() != Some(&second) {
                        mate.first = first;
                        mate.second = Some(second);
                        changes.modified_mates.insert(mate_id);
                        changes.solver_inputs_changed = true;
                    }
                }
            }
        }
    }
    Ok(changes)
}

fn canonical_finite_mate_parameter(value: f64) -> Result<f64, AssemblyCommandError> {
    if !value.is_finite() {
        return Err(AssemblyCommandError::NonFiniteMateParameter);
    }
    Ok(if value == 0.0 { 0.0 } else { value })
}

fn require_occurrence(
    document: &AssemblyDocument,
    occurrence_id: OccurrenceId,
) -> Result<&AssemblyOccurrence, AssemblyCommandError> {
    document
        .occurrences
        .get(&occurrence_id)
        .ok_or(AssemblyCommandError::MissingOccurrence(occurrence_id))
}

fn require_occurrence_mut(
    document: &mut AssemblyDocument,
    occurrence_id: OccurrenceId,
) -> Result<&mut AssemblyOccurrence, AssemblyCommandError> {
    document
        .occurrences
        .get_mut(&occurrence_id)
        .ok_or(AssemblyCommandError::MissingOccurrence(occurrence_id))
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssemblyCommandError {
    InvalidAssembly(AssemblyError),
    MissingOccurrence(OccurrenceId),
    MissingDefinition(ModelHash),
    EmptyLocalBodyId,
    OccurrenceLimitReached,
    OccurrenceIdExhausted,
    OccurrenceIdCollision(OccurrenceId),
    OccurrenceNameCollision(String),
    MatesUnavailable,
    MissingMate(crate::MateId),
    WrongMateKind(crate::MateId),
    CannotRepairFixedMate(crate::MateId),
    SameOccurrenceMateEntities(OccurrenceId),
    NonFiniteMateParameter,
    GroundedOccurrence(OccurrenceId),
}

impl fmt::Display for AssemblyCommandError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAssembly(error) => write!(formatter, "invalid assembly: {error}"),
            Self::MissingOccurrence(id) => write!(formatter, "occurrence {id} does not exist"),
            Self::MissingDefinition(hash) => {
                write!(formatter, "definition {} does not exist", hex_prefix(hash))
            }
            Self::EmptyLocalBodyId => formatter.write_str("body identity cannot be empty"),
            Self::OccurrenceLimitReached => {
                formatter.write_str("assembly occurrence limit has been reached")
            }
            Self::OccurrenceIdExhausted => {
                formatter.write_str("assembly occurrence id space is exhausted")
            }
            Self::OccurrenceIdCollision(id) => {
                write!(formatter, "next occurrence id {id} is already in use")
            }
            Self::OccurrenceNameCollision(name) => {
                write!(formatter, "occurrence name `{name}` is already in use")
            }
            Self::MatesUnavailable => formatter.write_str("this assembly has no mate set"),
            Self::MissingMate(id) => write!(formatter, "mate {id} does not exist"),
            Self::WrongMateKind(id) => {
                write!(formatter, "mate {id} does not accept that parameter")
            }
            Self::CannotRepairFixedMate(id) => {
                write!(
                    formatter,
                    "fixed mate {id} has no second selector to repair"
                )
            }
            Self::SameOccurrenceMateEntities(id) => write!(
                formatter,
                "mate entities must reference different occurrences, but both use {id}"
            ),
            Self::NonFiniteMateParameter => formatter.write_str("mate parameter must be finite"),
            Self::GroundedOccurrence(id) => {
                write!(formatter, "occurrence {id} is grounded and cannot be moved")
            }
        }
    }
}

impl std::error::Error for AssemblyCommandError {}

#[derive(Debug)]
pub enum AssemblyCommandFinalizeError<E> {
    Finalize(E),
    InvalidAssembly(AssemblyError),
}

impl<E: fmt::Display> fmt::Display for AssemblyCommandFinalizeError<E> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Finalize(error) => {
                write!(formatter, "assembly command finalization failed: {error}")
            }
            Self::InvalidAssembly(error) => {
                write!(formatter, "invalid finalized assembly: {error}")
            }
        }
    }
}

impl<E: fmt::Debug + fmt::Display> std::error::Error for AssemblyCommandFinalizeError<E> {}

fn hex_prefix(hash: &ModelHash) -> String {
    hash.iter()
        .take(6)
        .map(|byte| format!("{byte:02x}"))
        .collect()
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

    fn one_occurrence_assembly() -> AssemblyDocument {
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
            1,
            AssemblyOccurrence {
                id: 1,
                definition_model_hash: hash,
                name: "Bracket:1".into(),
                manual_placement: RigidPlacement::IDENTITY,
                grounded: false,
                resolved_placement_override: None,
            },
        );
        assembly.next_occurrence_id = 2;
        assembly
    }

    fn mated_assembly() -> AssemblyDocument {
        let mut assembly = one_occurrence_assembly();
        let hash = [7; 32];
        assembly.occurrences.insert(
            2,
            AssemblyOccurrence {
                id: 2,
                definition_model_hash: hash,
                name: "Bracket:2".into(),
                manual_placement: RigidPlacement::IDENTITY,
                grounded: false,
                resolved_placement_override: None,
            },
        );
        assembly.next_occurrence_id = 3;
        assembly.mates = Some(crate::AssemblyMateSet {
            next_mate_id: 2,
            mates: BTreeMap::from([(
                1,
                crate::AssemblyMate {
                    id: 1,
                    name: "Distance 1".into(),
                    suppressed: false,
                    first: crate::AssemblyEntityRef {
                        occurrence_id: 1,
                        local_body_id: "body-a".into(),
                        local_selector: crate::AssemblyLocalSelector::Face {
                            stable_id: "face-a".into(),
                        },
                    },
                    second: Some(crate::AssemblyEntityRef {
                        occurrence_id: 2,
                        local_body_id: "body-a".into(),
                        local_selector: crate::AssemblyLocalSelector::Face {
                            stable_id: "face-b".into(),
                        },
                    }),
                    kind: crate::AssemblyMateKind::SignedDistance { millimeters: 5.0 },
                    sense: crate::MateSense::Aligned,
                },
            )]),
        });
        assembly
    }

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

    #[test]
    fn staged_command_does_not_mutate_until_commit() {
        let mut assembly = one_occurrence_assembly();
        let prepared = assembly
            .prepare_command(AssemblyCommand::RenameOccurrence {
                occurrence_id: 1,
                name: "  Mounting Bracket  ".into(),
            })
            .unwrap();

        assert_eq!(assembly.occurrences[&1].name, "Bracket:1");
        assert_eq!(prepared.document().occurrences[&1].name, "Mounting Bracket");
        assert_eq!(prepared.changes().modified_occurrences, BTreeSet::from([1]));
        assert!(!prepared.changes().requires_scene_rebuild());

        let (changes, previous) = prepared.commit(&mut assembly);
        assert_eq!(assembly.occurrences[&1].name, "Mounting Bracket");
        assert_eq!(previous.occurrences[&1].name, "Bracket:1");
        assert_eq!(changes.modified_occurrences, BTreeSet::from([1]));
    }

    #[test]
    fn failed_command_and_finalizer_leave_live_document_untouched() {
        let assembly = one_occurrence_assembly();
        let original = assembly.clone();
        assert!(matches!(
            assembly.prepare_command(AssemblyCommand::RenameOccurrence {
                occurrence_id: 99,
                name: "Missing".into(),
            }),
            Err(AssemblyCommandError::MissingOccurrence(99))
        ));
        assert_eq!(assembly, original);

        let prepared = assembly
            .prepare_command(AssemblyCommand::SetOccurrenceGrounded {
                occurrence_id: 1,
                grounded: true,
            })
            .unwrap();
        let finalized = prepared.finalize(|_| Err::<(), _>("solver conflict"));
        assert!(matches!(
            finalized,
            Err(AssemblyCommandFinalizeError::Finalize("solver conflict"))
        ));
        assert_eq!(assembly, original);
    }

    #[test]
    fn duplicate_and_delete_report_precise_runtime_invalidation() {
        let mut assembly = one_occurrence_assembly();
        assembly.presentation.hidden_occurrences.insert(1);
        assembly
            .presentation
            .hidden_bodies
            .insert((1, "body-a".into()));

        let duplicate = assembly
            .prepare_command(AssemblyCommand::DuplicateOccurrence { occurrence_id: 1 })
            .unwrap();
        assert_eq!(duplicate.changes().added_occurrences, BTreeSet::from([2]));
        assert!(duplicate.changes().presentation_changed);
        assert!(duplicate.changes().requires_scene_rebuild());
        let (_, _) = duplicate.commit(&mut assembly);
        assert_eq!(assembly.next_occurrence_id, 3);
        assert_eq!(assembly.occurrences[&2].name, "Bracket:2");
        assert!(!assembly.occurrences[&2].grounded);
        assert!(assembly.presentation.hidden_occurrences.contains(&2));
        assert!(assembly
            .presentation
            .hidden_bodies
            .contains(&(2, "body-a".into())));

        let delete = assembly
            .prepare_command(AssemblyCommand::DeleteOccurrence { occurrence_id: 2 })
            .unwrap();
        assert_eq!(delete.changes().removed_occurrences, BTreeSet::from([2]));
        assert!(delete.changes().removed_definitions.is_empty());
        let (_, _) = delete.commit(&mut assembly);
        assert_eq!(assembly.next_occurrence_id, 3);

        let delete_last = assembly
            .prepare_command(AssemblyCommand::DeleteOccurrence { occurrence_id: 1 })
            .unwrap();
        assert_eq!(
            delete_last.changes().removed_definitions,
            BTreeSet::from([[7; 32]])
        );
        let (_, _) = delete_last.commit(&mut assembly);
        assert!(assembly.definitions.is_empty());
        assert_eq!(assembly.next_occurrence_id, 3);
    }

    #[test]
    fn unchanged_presentation_command_has_an_empty_change_set() {
        let assembly = one_occurrence_assembly();
        let prepared = assembly
            .prepare_command(AssemblyCommand::SetOccurrenceVisible {
                occurrence_id: 1,
                visible: true,
            })
            .unwrap();
        assert!(prepared.changes().is_empty());
    }

    #[test]
    fn mate_edits_are_staged_and_report_solver_invalidation() {
        let mut assembly = mated_assembly();
        let prepared = assembly
            .prepare_command(AssemblyCommand::EditMate {
                mate_id: 1,
                edit: AssemblyMateEdit::SetDistanceMillimeters(-0.0),
            })
            .unwrap();
        assert_eq!(prepared.changes().modified_mates, BTreeSet::from([1]));
        assert!(prepared.changes().solver_inputs_changed);
        assert_eq!(
            assembly.mates.as_ref().unwrap().mates[&1].kind,
            crate::AssemblyMateKind::SignedDistance { millimeters: 5.0 }
        );

        let (_, _) = prepared.commit(&mut assembly);
        let crate::AssemblyMateKind::SignedDistance { millimeters } =
            assembly.mates.as_ref().unwrap().mates[&1].kind
        else {
            panic!("distance mate changed kind");
        };
        assert_eq!(millimeters.to_bits(), 0.0f64.to_bits());

        let deleted = assembly
            .prepare_command(AssemblyCommand::EditMate {
                mate_id: 1,
                edit: AssemblyMateEdit::Delete,
            })
            .unwrap();
        assert_eq!(deleted.changes().removed_mates, BTreeSet::from([1]));
        assert!(deleted.changes().mate_graph_changed);
    }

    #[test]
    fn invalid_mate_edits_leave_the_document_untouched() {
        let assembly = mated_assembly();
        let original = assembly.clone();
        assert!(matches!(
            assembly.prepare_command(AssemblyCommand::EditMate {
                mate_id: 1,
                edit: AssemblyMateEdit::SetAngleRadians(f64::NAN),
            }),
            Err(AssemblyCommandError::NonFiniteMateParameter)
        ));
        assert_eq!(assembly, original);

        let same = crate::AssemblyEntityRef {
            occurrence_id: 1,
            local_body_id: "body-a".into(),
            local_selector: crate::AssemblyLocalSelector::Origin,
        };
        assert!(matches!(
            assembly.prepare_command(AssemblyCommand::EditMate {
                mate_id: 1,
                edit: AssemblyMateEdit::RepairEntities {
                    first: same.clone(),
                    second: same,
                },
            }),
            Err(AssemblyCommandError::SameOccurrenceMateEntities(1))
        ));
        assert_eq!(assembly, original);
    }

    #[test]
    fn placement_command_respects_grounding_and_marks_only_placement_state() {
        let mut assembly = one_occurrence_assembly();
        let placement = RigidPlacement::new([4.0, 5.0, 6.0], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let prepared = assembly
            .prepare_command(AssemblyCommand::SetOccurrencePlacement {
                occurrence_id: 1,
                placement,
            })
            .unwrap();
        assert!(prepared.changes().placements_changed);
        assert!(prepared.changes().requires_scene_rebuild());
        let (_, _) = prepared.commit(&mut assembly);
        assert_eq!(assembly.occurrences[&1].manual_placement, placement);

        assembly.occurrences.get_mut(&1).unwrap().grounded = true;
        let original = assembly.clone();
        assert!(matches!(
            assembly.prepare_command(AssemblyCommand::SetOccurrencePlacement {
                occurrence_id: 1,
                placement: RigidPlacement::IDENTITY,
            }),
            Err(AssemblyCommandError::GroundedOccurrence(1))
        ));
        assert_eq!(assembly, original);
    }
}
