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

    for diagnostic in result.diagnostics {
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
    for action in result.recovery.actions {
        record_diagnostic(EvaluationDiagnostic {
            feature_id: active_feature(),
            operation: operation.to_string(),
            failure_class: "kernel_recovery".to_string(),
            fallback: Some(format!("{action:?}")),
            severity: DiagnosticSeverity::Info,
            message: format!("OpenRCAD applied recovery: {action:?}"),
        });
    }

    Ok(KernelOutcome {
        solid: result.value,
        history: result.history,
    })
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
