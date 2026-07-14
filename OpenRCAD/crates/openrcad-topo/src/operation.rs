//! Shared result contract for kernel operations.
//!
//! The contract lives in the topology crate so primitive builders, exchange
//! readers, and modeling algorithms can all use it without dependency cycles.

use openrcad_foundation::TolerancePolicy;

use crate::{HealthReport, Solid, TopologyHistory, TopologyRef};

/// Severity of a non-fatal operation diagnostic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagnosticSeverity {
    Info,
    Warning,
    Error,
}

/// Structured, stable diagnostic attached to an operation result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Diagnostic {
    /// Stable machine-readable identifier.
    pub code: String,
    pub severity: DiagnosticSeverity,
    /// Concise human-readable explanation.
    pub message: String,
    /// Result entity most directly associated with the diagnostic.
    pub entity: Option<TopologyRef>,
}

impl Diagnostic {
    pub fn info(
        code: impl Into<String>,
        message: impl Into<String>,
        entity: Option<TopologyRef>,
    ) -> Self {
        Self {
            code: code.into(),
            severity: DiagnosticSeverity::Info,
            message: message.into(),
            entity,
        }
    }

    pub fn warning(
        code: impl Into<String>,
        message: impl Into<String>,
        entity: Option<TopologyRef>,
    ) -> Self {
        Self {
            code: code.into(),
            severity: DiagnosticSeverity::Warning,
            message: message.into(),
            entity,
        }
    }
}

/// Recovery successfully applied while producing an operation result.
#[derive(Clone, Debug, PartialEq)]
pub enum RecoveryAction {
    NearCoincidentSnap { distance: f64 },
    CloseGap { distance: f64 },
    HealTJunctions,
    CollapseSmallEdge { length: f64 },
    ReconstructPcurve { face: usize, edge: usize },
    ReconstructPcurves { count: usize },
    MergeCoplanarFaces { removed_faces: usize },
    MergeCocylindricalFaces { removed_faces: usize },
}

/// Ordered recovery work applied by an operation.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RecoveryReport {
    pub actions: Vec<RecoveryAction>,
}

impl RecoveryReport {
    #[inline]
    pub fn was_modified(&self) -> bool {
        !self.actions.is_empty()
    }
}

/// Final topology validation attached to a successful solid operation.
#[derive(Clone, Debug, PartialEq)]
pub struct ValidationReport {
    pub health: HealthReport,
    pub watertight: bool,
    pub pcurves_complete: bool,
}

impl ValidationReport {
    pub fn for_solid(solid: &Solid, policy: &TolerancePolicy) -> Self {
        let health = solid.health_report_with_policy(policy);
        Self {
            pcurves_complete: solid.has_complete_pcurves(),
            watertight: solid.is_watertight_with_policy(policy),
            health,
        }
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.health.is_healthy() && self.watertight && self.pcurves_complete
    }
}

/// Standard successful modeling-operation result.
#[must_use = "operation metadata contains validation, recovery, diagnostics, and topology history"]
#[derive(Clone, Debug, PartialEq)]
pub struct OperationResult<T> {
    pub value: T,
    pub history: TopologyHistory,
    pub diagnostics: Vec<Diagnostic>,
    pub recovery: RecoveryReport,
    pub validation: ValidationReport,
}
