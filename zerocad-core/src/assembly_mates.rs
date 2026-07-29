//! Assembly V2 mate-authoring schema.
//!
//! Solver state and diagnostics are derived and deliberately absent from these
//! serializable contracts. Selectors remain intact when topology cannot be
//! resolved so repair never destroys authored intent.

use crate::{AssemblyDocument, OccurrenceId};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
use std::fmt;

pub type MateId = u64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MateSense {
    Aligned,
    AntiAligned,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssemblyLocalSelector {
    Face { stable_id: String },
    Edge { stable_id: String },
    Axis { stable_id: String },
    Origin,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssemblyEntityRef {
    pub occurrence_id: OccurrenceId,
    pub local_body_id: String,
    pub local_selector: AssemblyLocalSelector,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AssemblyMateKind {
    Fixed,
    Coincident,
    Concentric,
    SignedDistance { millimeters: f64 },
    Angle { radians: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyMate {
    pub id: MateId,
    pub name: String,
    pub suppressed: bool,
    pub first: AssemblyEntityRef,
    /// `None` only for a fixed mate.
    pub second: Option<AssemblyEntityRef>,
    pub kind: AssemblyMateKind,
    pub sense: MateSense,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyMateSet {
    pub next_mate_id: MateId,
    pub mates: BTreeMap<MateId, AssemblyMate>,
}

impl Default for AssemblyMateSet {
    fn default() -> Self {
        Self {
            next_mate_id: 1,
            mates: BTreeMap::new(),
        }
    }
}

impl AssemblyMateSet {
    pub fn validate(&self, assembly: &AssemblyDocument) -> Result<(), AssemblyMateError> {
        if let Some(maximum) = self.mates.keys().next_back().copied() {
            if self.next_mate_id <= maximum {
                return Err(AssemblyMateError::InvalidNextMateId {
                    next: self.next_mate_id,
                    maximum,
                });
            }
        } else if self.next_mate_id == 0 {
            return Err(AssemblyMateError::InvalidNextMateId {
                next: 0,
                maximum: 0,
            });
        }
        let mut names = HashSet::with_capacity(self.mates.len());
        for (key, mate) in &self.mates {
            if *key != mate.id {
                return Err(AssemblyMateError::MateKeyMismatch);
            }
            if mate.name.trim().is_empty() {
                return Err(AssemblyMateError::EmptyMateName);
            }
            if !names.insert(mate.name.to_lowercase()) {
                return Err(AssemblyMateError::DuplicateMateName(mate.name.clone()));
            }
            validate_entity(&mate.first, assembly)?;
            if matches!(mate.kind, AssemblyMateKind::Fixed) {
                if mate.second.is_some() {
                    return Err(AssemblyMateError::FixedMateHasSecondEntity(mate.id));
                }
            } else {
                let second = mate
                    .second
                    .as_ref()
                    .ok_or(AssemblyMateError::MissingSecondEntity(mate.id))?;
                validate_entity(second, assembly)?;
            }
            match mate.kind {
                AssemblyMateKind::SignedDistance { millimeters } if !millimeters.is_finite() => {
                    return Err(AssemblyMateError::NonFiniteParameter(mate.id));
                }
                AssemblyMateKind::Angle { radians } if !radians.is_finite() => {
                    return Err(AssemblyMateError::NonFiniteParameter(mate.id));
                }
                _ => {}
            }
        }
        Ok(())
    }
}

fn validate_entity(
    entity: &AssemblyEntityRef,
    assembly: &AssemblyDocument,
) -> Result<(), AssemblyMateError> {
    if !assembly.occurrences.contains_key(&entity.occurrence_id) {
        return Err(AssemblyMateError::MissingOccurrence(entity.occurrence_id));
    }
    if entity.local_body_id.trim().is_empty() {
        return Err(AssemblyMateError::EmptyLocalBody);
    }
    // Stable selectors are intentionally not resolved here. Evaluation or
    // replacement may leave them dangling; that is an unresolved mate, not
    // corrupt authoritative data.
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssemblyMateError {
    InvalidNextMateId { next: MateId, maximum: MateId },
    MateKeyMismatch,
    EmptyMateName,
    DuplicateMateName(String),
    MissingOccurrence(OccurrenceId),
    EmptyLocalBody,
    MissingSecondEntity(MateId),
    FixedMateHasSecondEntity(MateId),
    NonFiniteParameter(MateId),
}

impl fmt::Display for AssemblyMateError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNextMateId { next, maximum } => write!(
                formatter,
                "next mate id {next} must be greater than existing maximum {maximum}"
            ),
            Self::MateKeyMismatch => formatter.write_str("mate map key does not match mate id"),
            Self::EmptyMateName => formatter.write_str("mate names cannot be empty"),
            Self::DuplicateMateName(name) => write!(formatter, "duplicate mate name `{name}`"),
            Self::MissingOccurrence(id) => {
                write!(formatter, "mate references missing occurrence {id}")
            }
            Self::EmptyLocalBody => formatter.write_str("mate local body id cannot be empty"),
            Self::MissingSecondEntity(id) => {
                write!(formatter, "mate {id} requires a second entity")
            }
            Self::FixedMateHasSecondEntity(id) => {
                write!(formatter, "fixed mate {id} cannot have a second entity")
            }
            Self::NonFiniteParameter(id) => {
                write!(formatter, "mate {id} contains a non-finite parameter")
            }
        }
    }
}

impl std::error::Error for AssemblyMateError {}
