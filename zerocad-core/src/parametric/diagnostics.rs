use super::{
    DiagnosticCode, DiagnosticSeverity, EvaluationDiagnostic, FeatureStatus, ResolutionState,
};

/// Convert a compatibility feature status into the typed diagnostic contract.
///
/// Feature implementations still populate their historical warning strings
/// while Foundation 1B migrates evaluator families. Classification happens at
/// this single boundary, so new consumers never need to parse UI copy. Each
/// family can replace the classifier with a directly constructed code without
/// changing callers.
pub(crate) fn diagnostic_for_status(status: &FeatureStatus) -> Option<EvaluationDiagnostic> {
    let ResolutionState::Unresolved(message) = &status.state else {
        return None;
    };
    Some(
        EvaluationDiagnostic::new(
            status.feature_id.clone(),
            "feature evaluation",
            classify_unresolved_message(message),
            DiagnosticSeverity::Warning,
            message,
        )
        .with_parameter("feature_id", status.feature_id.to_string())
        .with_parameter("feature_name", status.feature_name.clone())
        // `unresolved_feature` was the public pre-typed classification. Keep it
        // for one compatibility cycle while `code` carries the new contract.
        .with_failure_class("unresolved_feature")
        .with_fallback("kept last valid body"),
    )
}

fn classify_unresolved_message(message: &str) -> DiagnosticCode {
    let message = message.to_ascii_lowercase();
    if message.contains("ambiguous") || message.contains("more than one") {
        DiagnosticCode::reference_ambiguous()
    } else if [
        "no longer exists",
        "does not exist",
        "missing",
        "not found",
        "could not find",
        "unknown target",
        "no target body",
        "no cylindrical face",
    ]
    .iter()
    .any(|needle| message.contains(needle))
    {
        DiagnosticCode::reference_missing()
    } else if [
        "must be positive",
        "must be greater than zero",
        "invalid parameter",
        "invalid radius",
        "invalid distance",
        "invalid thickness",
    ]
    .iter()
    .any(|needle| message.contains(needle))
    {
        DiagnosticCode::parameter_invalid()
    } else if [
        "invalid solid topology",
        "invalid topology",
        "positive-volume",
        "non-manifold",
        "not watertight",
        "display-mesh crack",
    ]
    .iter()
    .any(|needle| message.contains(needle))
    {
        DiagnosticCode::result_invalid_topology()
    } else if [
        "failed",
        "couldn't be",
        "could not be",
        "was not applied",
        "must touch or overlap",
        "must overlap",
        "operation declined",
        "candidate validation failed",
        "cut trim requires",
    ]
    .iter()
    .any(|needle| message.contains(needle))
    {
        DiagnosticCode::operation_failed()
    } else {
        DiagnosticCode::feature_unresolved()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status(message: &str) -> FeatureStatus {
        FeatureStatus {
            feature_id: "edge_mod_7".into(),
            feature_name: "Fillet".into(),
            state: ResolutionState::Unresolved(message.into()),
        }
    }

    #[test]
    fn behavioral_classification_is_independent_of_renderer_copy() {
        let cases = [
            ("target no longer exists", DiagnosticCode::REFERENCE_MISSING),
            (
                "reference is ambiguous",
                DiagnosticCode::REFERENCE_AMBIGUOUS,
            ),
            (
                "distance must be positive",
                DiagnosticCode::PARAMETER_INVALID,
            ),
            ("native operation failed", DiagnosticCode::OPERATION_FAILED),
            (
                "feature produced invalid solid topology",
                DiagnosticCode::RESULT_INVALID_TOPOLOGY,
            ),
            (
                "feature remains unresolved",
                DiagnosticCode::FEATURE_UNRESOLVED,
            ),
        ];
        for (message, expected) in cases {
            let diagnostic = diagnostic_for_status(&status(message)).unwrap();
            assert_eq!(diagnostic.code.as_str(), expected);
            assert_eq!(diagnostic.failure_class, "unresolved_feature");
            assert_eq!(
                diagnostic.parameters.get("feature_id"),
                Some(&super::super::DiagnosticParameterValue::Text(
                    "edge_mod_7".into()
                ))
            );
        }
    }

    #[test]
    fn renderer_copy_has_a_small_explicit_contract_suite() {
        let diagnostic = diagnostic_for_status(&status("distance must be positive")).unwrap();
        assert_eq!(diagnostic.rendered_message(), "distance must be positive");
        assert_eq!(diagnostic.fallback.as_deref(), Some("kept last valid body"));
    }

    #[test]
    fn resolved_and_suppressed_statuses_do_not_emit_diagnostics() {
        for state in [ResolutionState::Resolved, ResolutionState::Suppressed] {
            let status = FeatureStatus {
                feature_id: "box_1".into(),
                feature_name: "Box".into(),
                state,
            };
            assert!(diagnostic_for_status(&status).is_none());
        }
    }
}
