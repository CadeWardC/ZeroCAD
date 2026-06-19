//! Topological orientation (OCCT `TopAbs_Orientation`).

use serde::{Deserialize, Serialize};

/// The orientation of a topological entity within its parent.
///
/// `Forward` means the entity is used as defined; `Reversed` means its sense is
/// flipped (e.g. an edge traversed end-to-start). `Internal`/`External` cover the
/// rarer embedded cases.
#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub enum Orientation {
    /// Used as defined (the default).
    #[default]
    Forward,
    /// Sense flipped.
    Reversed,
    /// Embedded inside its parent.
    Internal,
    /// External to its parent.
    External,
}

impl Orientation {
    /// True when `Forward`.
    #[inline]
    pub const fn is_forward(&self) -> bool {
        matches!(self, Self::Forward)
    }

    /// The opposite orientation.
    #[inline]
    pub const fn reversed(&self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
            Self::Internal => Self::Internal,
            Self::External => Self::External,
        }
    }
}
