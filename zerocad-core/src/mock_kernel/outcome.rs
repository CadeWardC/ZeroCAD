//! One ZeroCAD integration boundary for canonical OpenRCAD operation results.

use std::cell::RefCell;
use std::fmt::Display;

use openrcad::topo::{
    DiagnosticSeverity as KernelSeverity, OperationResult, Solid, TopologyChange, TopologyHistory,
    TopologyKind,
};

use crate::{DiagnosticSeverity, EvaluationDiagnostic};

use super::{BooleanFaceHistory, BooleanFaceSource};

thread_local! {
    static ACTIVE_FEATURE: RefCell<Option<String>> = const { RefCell::new(None) };
    static PENDING_DIAGNOSTICS: RefCell<Vec<EvaluationDiagnostic>> = const { RefCell::new(Vec::new()) };
}

/// A canonical solid plus the topology lineage used by ZeroCAD's naming layer.
#[must_use]
pub(crate) struct KernelOutcome {
    pub solid: Solid,
    pub history: TopologyHistory,
}

/// Canonical connected boolean results plus face lineage local to each solid.
#[must_use]
pub(crate) struct KernelBooleanBodiesOutcome {
    pub bodies: Vec<Solid>,
    pub face_history: Vec<BooleanFaceHistory>,
}

/// Canonical two-sided planar split consumed at ZeroCAD's single kernel
/// outcome boundary.
#[must_use]
pub(crate) struct KernelPlaneSplitOutcome {
    pub negative: Vec<Solid>,
    pub positive: Vec<Solid>,
    pub negative_face_history: Vec<BooleanFaceHistory>,
    pub positive_face_history: Vec<BooleanFaceHistory>,
}

impl KernelOutcome {
    /// Translate shared topology lineage into ZeroCAD's current face naming map.
    pub fn boolean_face_history(&self) -> BooleanFaceHistory {
        boolean_face_history_from_topology(&self.history, self.solid.face_count())
    }
}

/// Consume all validation, history, diagnostics, and recovery metadata at one
/// application boundary. An operation cannot enter ZeroCAD's live-body state if
/// its representation or history coverage is incomplete.
pub(crate) fn consume_operation<E: Display>(
    operation: &str,
    result: Result<OperationResult<Solid>, E>,
) -> Result<KernelOutcome, String> {
    let result = result.map_err(|error| format!("{operation}: {error}"))?;
    if !result.validation.is_valid() {
        return Err(format!(
            "{operation}: kernel returned an invalid representation ({:?})",
            result.validation
        ));
    }
    // Canonical OpenRCAD entry points produce `OperationResult` only after
    // strict, policy-aware validation. Treat that report as the application
    // boundary contract instead of immediately repeating the full topology
    // audit on every primitive, import, and boolean result.
    let coverage = result.history.coverage_for_solid(&result.value);
    if !coverage.is_complete() {
        return Err(format!(
            "{operation}: topology history omitted result entities: {:?}",
            coverage.missing_results
        ));
    }

    record_kernel_metadata(operation, result.diagnostics, result.recovery);

    Ok(KernelOutcome {
        solid: result.value,
        history: result.history,
    })
}

/// Wrap a validated local unary builder in the same canonical result contract
/// used by OpenRCAD entry points. ZeroCAD still owns a few domain-specific
/// constructors (notably modeled threads), but they must not bypass history or
/// representation validation at the evaluator boundary.
pub(crate) fn consume_local_unary_operation(
    operation: &str,
    input: &Solid,
    value: Solid,
    policy: &openrcad::foundation::TolerancePolicy,
) -> Result<KernelOutcome, String> {
    let (value, reconstructed) = value
        .repair_pcurves(policy)
        .map_err(|error| format!("{operation}: pcurve construction failed: {error}"))?;
    let validation = openrcad::topo::ValidationReport::for_solid(&value, policy);
    let mut recovery = openrcad::topo::RecoveryReport::default();
    if reconstructed > 0 {
        recovery
            .actions
            .push(openrcad::topo::RecoveryAction::ReconstructPcurves {
                count: reconstructed,
            });
    }
    consume_operation::<String>(
        operation,
        Ok(OperationResult {
            history: TopologyHistory::conservative_unary(input, &value),
            value,
            diagnostics: Vec::new(),
            recovery,
            validation,
        }),
    )
}

/// Consume a canonical multi-body boolean result at the same application
/// boundary used by every single-solid operation.
pub(crate) fn consume_boolean_bodies_operation<E: Display>(
    operation: &str,
    result: Result<OperationResult<openrcad::algo::BooleanBodies>, E>,
) -> Result<KernelBooleanBodiesOutcome, String> {
    let result = result.map_err(|error| format!("{operation}: {error}"))?;
    if !result.validation.is_valid() {
        return Err(format!(
            "{operation}: kernel returned invalid connected results ({:?})",
            result.validation
        ));
    }
    if result.value.bodies.is_empty()
        || result.value.bodies.len() != result.value.face_history.len()
        || result
            .value
            .bodies
            .iter()
            .zip(&result.value.face_history)
            .any(|(body, history)| history.face_source.len() != body.face_count())
    {
        return Err(format!(
            "{operation}: kernel returned incomplete per-body face history"
        ));
    }
    result
        .history
        .validate()
        .map_err(|error| format!("{operation}: invalid topology history: {error}"))?;
    record_kernel_metadata(operation, result.diagnostics, result.recovery);
    Ok(KernelBooleanBodiesOutcome {
        bodies: result.value.bodies,
        face_history: result.value.face_history,
    })
}

pub(crate) fn consume_plane_split_operation<E: Display>(
    operation: &str,
    result: Result<OperationResult<openrcad::algo::PlaneSplitBodies>, E>,
) -> Result<KernelPlaneSplitOutcome, String> {
    let result = result.map_err(|error| format!("{operation}: {error}"))?;
    if !result.validation.is_valid() {
        return Err(format!(
            "{operation}: kernel returned invalid split results ({:?})",
            result.validation
        ));
    }
    let all = result
        .value
        .negative
        .iter()
        .chain(&result.value.positive)
        .cloned()
        .collect::<Vec<_>>();
    if result.value.negative.is_empty()
        || result.value.positive.is_empty()
        || result.value.negative.len() != result.value.negative_face_history.len()
        || result.value.positive.len() != result.value.positive_face_history.len()
    {
        return Err(format!(
            "{operation}: kernel returned an incomplete two-sided split"
        ));
    }
    let histories_complete = result
        .value
        .negative
        .iter()
        .zip(&result.value.negative_face_history)
        .chain(
            result
                .value
                .positive
                .iter()
                .zip(&result.value.positive_face_history),
        )
        .all(|(solid, history)| solid.face_count() == history.face_source.len());
    if !histories_complete {
        return Err(format!(
            "{operation}: kernel returned incomplete per-body face history"
        ));
    }
    result
        .history
        .validate()
        .map_err(|error| format!("{operation}: invalid topology history: {error}"))?;
    let coverage = result.history.coverage_for_solids(&all);
    if !coverage.is_complete() {
        return Err(format!(
            "{operation}: topology history omitted result entities: {:?}",
            coverage.missing_results
        ));
    }
    record_kernel_metadata(operation, result.diagnostics, result.recovery);
    Ok(KernelPlaneSplitOutcome {
        negative: result.value.negative,
        positive: result.value.positive,
        negative_face_history: result.value.negative_face_history,
        positive_face_history: result.value.positive_face_history,
    })
}

fn record_kernel_metadata(
    operation: &str,
    diagnostics: Vec<openrcad::topo::Diagnostic>,
    recovery: openrcad::topo::RecoveryReport,
) {
    for diagnostic in diagnostics {
        let severity = match diagnostic.severity {
            KernelSeverity::Info => DiagnosticSeverity::Info,
            KernelSeverity::Warning => DiagnosticSeverity::Warning,
            KernelSeverity::Error => DiagnosticSeverity::Error,
        };
        record_diagnostic(EvaluationDiagnostic {
            feature_id: active_feature(),
            operation: operation.to_string(),
            failure_class: diagnostic.code,
            fallback: None,
            severity,
            message: diagnostic.message,
        });
    }
    for action in recovery.actions {
        record_diagnostic(EvaluationDiagnostic {
            feature_id: active_feature(),
            operation: operation.to_string(),
            failure_class: "kernel_recovery".to_string(),
            fallback: Some(format!("{action:?}")),
            severity: DiagnosticSeverity::Info,
            message: format!("OpenRCAD applied recovery: {action:?}"),
        });
    }
}

pub(crate) fn set_feature_context(feature_id: Option<&str>) {
    ACTIVE_FEATURE.with(|active| {
        *active.borrow_mut() = feature_id.map(str::to_owned);
    });
}

pub(crate) fn reset_diagnostics() {
    set_feature_context(None);
    PENDING_DIAGNOSTICS.with(|pending| pending.borrow_mut().clear());
}

pub(crate) fn take_diagnostics() -> Vec<EvaluationDiagnostic> {
    PENDING_DIAGNOSTICS.with(|pending| std::mem::take(&mut *pending.borrow_mut()))
}

fn active_feature() -> String {
    ACTIVE_FEATURE.with(|active| active.borrow().clone().unwrap_or_default())
}

pub(crate) fn current_feature_context() -> Option<String> {
    ACTIVE_FEATURE.with(|active| active.borrow().clone())
}

fn record_diagnostic(diagnostic: EvaluationDiagnostic) {
    PENDING_DIAGNOSTICS.with(|pending| pending.borrow_mut().push(diagnostic));
}

fn boolean_face_history_from_topology(
    history: &TopologyHistory,
    result_faces: usize,
) -> BooleanFaceHistory {
    let mut face_source = vec![None; result_faces];
    let mut assign = |result: openrcad::topo::TopologyRef,
                      source: openrcad::topo::InputTopologyRef| {
        if result.kind != TopologyKind::Face || source.entity.kind != TopologyKind::Face {
            return;
        }
        let mapped = match source.operand {
            0 => Some(BooleanFaceSource::Object(source.entity.index)),
            1 => Some(BooleanFaceSource::Tool(source.entity.index)),
            _ => None,
        };
        if let (Some(slot), Some(mapped)) = (face_source.get_mut(result.index), mapped) {
            slot.get_or_insert(mapped);
        }
    };

    for change in &history.changes {
        match change {
            TopologyChange::Generated { sources, result }
            | TopologyChange::Merged { sources, result } => {
                if let Some(source) = sources
                    .iter()
                    .copied()
                    .find(|source| source.entity.kind == TopologyKind::Face)
                {
                    assign(*result, source);
                }
            }
            TopologyChange::Modified { source, result } => assign(*result, *source),
            TopologyChange::Split { source, results } => {
                for result in results {
                    assign(*result, *source);
                }
            }
            TopologyChange::Deleted { .. } => {}
        }
    }
    BooleanFaceHistory { face_source }
}

#[cfg(test)]
mod tests {
    use openrcad::algo::{boolean_operation, BooleanOp};
    use openrcad::foundation::Pnt;
    use openrcad::primitives::make_box_operation;

    use super::*;

    #[test]
    fn adapter_requires_complete_history_and_maps_boolean_faces() {
        reset_diagnostics();
        let object = consume_operation("box", make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0))
            .unwrap()
            .solid;
        let tool = consume_operation(
            "box",
            make_box_operation(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0),
        )
        .unwrap()
        .solid;
        let result = consume_operation(
            "boolean fuse",
            boolean_operation(&object, &tool, BooleanOp::Fuse),
        )
        .unwrap();

        assert!(result
            .history
            .coverage_for_solid(&result.solid)
            .is_complete());
        assert!(result
            .boolean_face_history()
            .face_source
            .iter()
            .all(Option::is_some));
    }
}
