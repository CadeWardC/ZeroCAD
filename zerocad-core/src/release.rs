//! Part Design 1.0 release contracts.
//!
//! Phase 7 deliberately separates code-complete checks from evidence that must
//! come from a signed package, reference hardware, the public alpha, and the
//! external OCCT oracle.  Keeping the thresholds here makes a missing result a
//! hard failure instead of a sentence that can accidentally be checked off.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// A retained, structured capability boundary at the Part Design 1.0 gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct CompatibilityException {
    pub id: &'static str,
    pub capability: &'static str,
    pub owner: &'static str,
    pub reason: &'static str,
    pub user_impact: &'static str,
    pub removal_trigger: &'static str,
}

/// Phase 7's complete retained-exception ledger. These are explicit failures,
/// never silent compatibility fallbacks or production allowlist entries.
pub const PHASE7_COMPATIBILITY_EXCEPTIONS: &[CompatibilityException] = &[
    CompatibilityException {
        id: "P7-E1",
        capability: "general tangential Move Face",
        owner: "OpenRCAD local-topology surgery",
        reason: "arbitrary neighborhoods require extending and re-intersecting adjacent surfaces",
        user_impact: "non-planar or blend-adjacent neighborhoods fail atomically with a diagnostic",
        removal_trigger: "the imported-part analytic, blend-adjacent, concave, and multi-loop fixture matrix passes with stable naming",
    },
    CompatibilityException {
        id: "P7-E2",
        capability: "general Delete Face patch-and-sew",
        owner: "OpenRCAD healing and sewing",
        reason: "mixed-surface gaps require deterministic surface extension, trimming, and patch history",
        user_impact: "unsupported selections remain unchanged and receive a structured diagnostic",
        removal_trigger: "planar, filleted, conical, and multi-face deletion fixtures pass strict watertight and pcurve gates",
    },
    CompatibilityException {
        id: "P7-E3",
        capability: "non-analytic offset and thicken",
        owner: "OpenRCAD surface-offset algorithms",
        reason: "toroidal, spline, and mixed neighborhoods require exact intersections and self-intersection classification",
        user_impact: "unsupported surfaces fail atomically rather than returning approximate solids",
        removal_trigger: "the exact offset/intersection service passes scale, seam, collapse, and self-intersection matrices",
    },
    CompatibilityException {
        id: "P7-E4",
        capability: "difficult non-analytic bounded splits",
        owner: "OpenRCAD boolean arrangements",
        reason: "B-spline-heavy tools exceed the current split/imprint intersection envelope",
        user_impact: "an unsupported bounded tool fails before either output is committed",
        removal_trigger: "curated STEP B-spline tools pass conservation, pcurve, naming, and downstream-operation gates",
    },
    CompatibilityException {
        id: "P7-E5",
        capability: "automatic STL interpretation and repair",
        owner: "post-1.0 mesh operations",
        reason: "STL has no units and automatic repair or decimation changes source geometry",
        user_impact: "input is interpreted in document units and diagnosed exactly as supplied",
        removal_trigger: "explicit provenance-bearing repair and decimation features ship; surface-recognizing mesh-to-BRep remains separately scoped",
    },
];

/// Validate that the retained-exception ledger cannot degrade into anonymous
/// debt. This is also exercised by the Phase 7 behavioral gate.
pub fn validate_phase7_exception_ledger() -> Result<(), String> {
    validate_compatibility_exception_ledger(PHASE7_COMPATIBILITY_EXCEPTIONS)
}

/// Validate any compatibility-exception ledger.
///
/// Keeping this data-driven makes the duplicate and missing-field checks
/// independently testable instead of relying on the current constant already
/// being well formed.
pub fn validate_compatibility_exception_ledger(
    exceptions: &[CompatibilityException],
) -> Result<(), String> {
    let mut ids = HashSet::new();
    for exception in exceptions {
        if !ids.insert(exception.id) {
            return Err(format!("duplicate Phase 7 exception id '{}'", exception.id));
        }
        for (field, value) in [
            ("id", exception.id),
            ("capability", exception.capability),
            ("owner", exception.owner),
            ("reason", exception.reason),
            ("user impact", exception.user_impact),
            ("removal trigger", exception.removal_trigger),
        ] {
            if value.trim().is_empty() {
                return Err(format!("{} has an empty {field}", exception.id));
            }
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Phase7ReleaseEvidence {
    pub schema: u32,
    pub artifact: ArtifactEvidence,
    pub startup: StartupEvidence,
    pub interaction: InteractionEvidence,
    pub documents: DocumentPerformanceEvidence,
    pub validation: ValidationEvidence,
    pub alpha: AlphaEvidence,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArtifactEvidence {
    pub signed_package: bool,
    pub stripped_executable_bytes: u64,
    pub compressed_distribution_bytes: u64,
    pub installed_footprint_bytes: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StartupEvidence {
    pub sample_count: u32,
    pub includes_installed_first_launch: bool,
    pub median_ms: f64,
    pub p95_ms: f64,
    pub warm_ms: f64,
    pub idle_working_set_bytes: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InteractionEvidence {
    pub trailing_edit_ms: f64,
    pub full_rebuild_ms: f64,
    pub viewport_triangles: u64,
    pub viewport_fps: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocumentPerformanceEvidence {
    pub save_100_ms: f64,
    pub open_100_ms: f64,
    pub save_500_ms: f64,
    pub open_500_ms: f64,
    pub hydrated_first_display_ms: f64,
    pub hydrated_first_edit_warm_ratio: f64,
    pub streaming_memory_contract_passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationEvidence {
    pub zero_cad_workspace_passed: bool,
    pub openrcad_workspace_passed: bool,
    pub long_form_stress_passed: bool,
    pub occt_differential_passed: bool,
    pub occt_case_count: u32,
    pub frozen_phase0_passed: bool,
    pub maximum_relative_regression_percent: f64,
    pub warning_review_complete: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlphaEvidence {
    pub participant_sessions: u32,
    pub curated_foreign_models: u32,
    pub accepted_defects: u32,
    pub frozen_regressions: u32,
    pub unresolved_release_blockers: u32,
}

/// Validate evidence for the Part Design 1.0 release. Every failure is returned
/// so release engineering sees the complete checklist in one run.
pub fn validate_phase7_release_evidence(evidence: &Phase7ReleaseEvidence) -> Vec<String> {
    const MIB: u64 = 1024 * 1024;
    let mut failures = Vec::new();
    let mut require = |condition: bool, message: &str| {
        if !condition {
            failures.push(message.to_string());
        }
    };

    require(evidence.schema == 1, "release evidence schema must be 1");
    require(
        evidence.artifact.signed_package,
        "release package is not signed",
    );
    require(
        evidence.artifact.stripped_executable_bytes > 0
            && evidence.artifact.compressed_distribution_bytes > 0
            && evidence.artifact.installed_footprint_bytes > 0,
        "artifact size evidence must contain measured non-zero values",
    );
    require(
        evidence.artifact.stripped_executable_bytes < 30 * MIB,
        "stripped executable must remain below 30 MiB",
    );
    require(
        evidence.artifact.compressed_distribution_bytes <= 25 * MIB,
        "compressed distribution must remain at or below 25 MiB",
    );
    require(
        evidence.artifact.installed_footprint_bytes < 50 * MIB,
        "installed footprint must remain below 50 MiB",
    );
    require(
        evidence.startup.sample_count >= 7,
        "startup evidence needs at least seven samples",
    );
    require(
        evidence.startup.idle_working_set_bytes > 0,
        "startup memory evidence must be measured",
    );
    for (name, value) in [
        ("startup median", evidence.startup.median_ms),
        ("startup p95", evidence.startup.p95_ms),
        ("warm start", evidence.startup.warm_ms),
        ("trailing edit", evidence.interaction.trailing_edit_ms),
        ("full rebuild", evidence.interaction.full_rebuild_ms),
        ("viewport FPS", evidence.interaction.viewport_fps),
        ("100-feature save", evidence.documents.save_100_ms),
        ("100-feature open", evidence.documents.open_100_ms),
        ("500-feature save", evidence.documents.save_500_ms),
        ("500-feature open", evidence.documents.open_500_ms),
        (
            "hydrated first display",
            evidence.documents.hydrated_first_display_ms,
        ),
        (
            "hydrated first-edit ratio",
            evidence.documents.hydrated_first_edit_warm_ratio,
        ),
    ] {
        require(
            value.is_finite() && value > 0.0,
            &format!("{name} evidence must be finite, measured, and positive"),
        );
    }
    require(
        evidence
            .validation
            .maximum_relative_regression_percent
            .is_finite()
            && evidence.validation.maximum_relative_regression_percent >= 0.0,
        "relative regression evidence must be finite and non-negative",
    );
    require(
        evidence.startup.includes_installed_first_launch,
        "startup evidence must include the installed artifact's first launch",
    );
    require(
        evidence.startup.median_ms < 1_000.0,
        "cold-start median must be below 1 second",
    );
    require(
        evidence.startup.p95_ms < 1_000.0,
        "cold-start p95 must be below 1 second",
    );
    require(
        evidence.startup.warm_ms < 500.0,
        "warm start must be below 500 ms",
    );
    require(
        evidence.startup.idle_working_set_bytes <= 150 * MIB,
        "idle working set must not exceed 150 MiB",
    );
    require(
        evidence.interaction.trailing_edit_ms < 50.0,
        "trailing edit must be below 50 ms",
    );
    require(
        evidence.interaction.full_rebuild_ms < 250.0,
        "representative rebuild must be below 250 ms",
    );
    require(
        evidence.interaction.viewport_triangles >= 1_000_000,
        "viewport evidence must contain at least one million triangles",
    );
    require(
        evidence.interaction.viewport_fps >= 60.0,
        "one-million-triangle viewport must sustain 60 FPS",
    );
    require(
        evidence.documents.save_100_ms < 100.0,
        "100-feature save must be below 100 ms",
    );
    require(
        evidence.documents.open_100_ms < 100.0,
        "100-feature open must be below 100 ms",
    );
    require(
        evidence.documents.save_500_ms < 250.0,
        "500-feature save must be below 250 ms",
    );
    require(
        evidence.documents.open_500_ms < 250.0,
        "500-feature open must be below 250 ms",
    );
    require(
        evidence.documents.hydrated_first_display_ms < 250.0,
        "hydrated first display must be below 250 ms",
    );
    require(
        evidence.documents.hydrated_first_edit_warm_ratio <= 2.0,
        "hydrated first edit must be at most twice the warm edit",
    );
    require(
        evidence.documents.streaming_memory_contract_passed,
        "streaming document memory contract has not passed",
    );
    require(
        evidence.validation.zero_cad_workspace_passed,
        "ZeroCAD workspace gate has not passed",
    );
    require(
        evidence.validation.openrcad_workspace_passed,
        "OpenRCAD workspace gate has not passed",
    );
    require(
        evidence.validation.long_form_stress_passed,
        "long-form stress suite has not passed",
    );
    require(
        evidence.validation.occt_differential_passed,
        "OCCT differential suite has not passed",
    );
    require(
        evidence.validation.occt_case_count >= 6,
        "OCCT differential evidence needs at least six cases",
    );
    require(
        evidence.validation.frozen_phase0_passed,
        "frozen Phase 0 gate has not passed",
    );
    require(
        evidence.validation.maximum_relative_regression_percent <= 10.0,
        "relative performance regression exceeds 10%",
    );
    require(
        evidence.validation.warning_review_complete,
        "compiler warning review is incomplete",
    );
    require(
        evidence.alpha.participant_sessions > 0,
        "public-alpha evidence contains no participant sessions",
    );
    require(
        evidence.alpha.curated_foreign_models > 0,
        "public-alpha evidence contains no curated foreign models",
    );
    require(
        evidence.alpha.accepted_defects == evidence.alpha.frozen_regressions,
        "every accepted alpha defect must have a frozen regression",
    );
    require(
        evidence.alpha.unresolved_release_blockers == 0,
        "public alpha still has unresolved release blockers",
    );
    failures
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_EXCEPTION: CompatibilityException = CompatibilityException {
        id: "P7-TEST",
        capability: "test capability",
        owner: "test owner",
        reason: "test reason",
        user_impact: "test impact",
        removal_trigger: "test trigger",
    };

    #[test]
    fn compatibility_ledger_validator_rejects_bad_supplied_data() {
        let duplicate = [VALID_EXCEPTION, VALID_EXCEPTION];
        let duplicate_error = validate_compatibility_exception_ledger(&duplicate).unwrap_err();
        assert!(duplicate_error.contains("duplicate Phase 7 exception id"));

        let missing_owner = CompatibilityException {
            id: "P7-EMPTY",
            owner: "  ",
            ..VALID_EXCEPTION
        };
        let empty_error = validate_compatibility_exception_ledger(&[missing_owner]).unwrap_err();
        assert!(empty_error.contains("empty owner"));
    }

    fn passing_evidence() -> Phase7ReleaseEvidence {
        Phase7ReleaseEvidence {
            schema: 1,
            artifact: ArtifactEvidence {
                signed_package: true,
                stripped_executable_bytes: 22 * 1024 * 1024,
                compressed_distribution_bytes: 23 * 1024 * 1024,
                installed_footprint_bytes: 45 * 1024 * 1024,
            },
            startup: StartupEvidence {
                sample_count: 7,
                includes_installed_first_launch: true,
                median_ms: 700.0,
                p95_ms: 900.0,
                warm_ms: 150.0,
                idle_working_set_bytes: 120 * 1024 * 1024,
            },
            interaction: InteractionEvidence {
                trailing_edit_ms: 20.0,
                full_rebuild_ms: 200.0,
                viewport_triangles: 1_000_000,
                viewport_fps: 60.0,
            },
            documents: DocumentPerformanceEvidence {
                save_100_ms: 20.0,
                open_100_ms: 10.0,
                save_500_ms: 80.0,
                open_500_ms: 40.0,
                hydrated_first_display_ms: 100.0,
                hydrated_first_edit_warm_ratio: 1.5,
                streaming_memory_contract_passed: true,
            },
            validation: ValidationEvidence {
                zero_cad_workspace_passed: true,
                openrcad_workspace_passed: true,
                long_form_stress_passed: true,
                occt_differential_passed: true,
                occt_case_count: 7,
                frozen_phase0_passed: true,
                maximum_relative_regression_percent: 9.0,
                warning_review_complete: true,
            },
            alpha: AlphaEvidence {
                participant_sessions: 1,
                curated_foreign_models: 1,
                accepted_defects: 2,
                frozen_regressions: 2,
                unresolved_release_blockers: 0,
            },
        }
    }

    #[test]
    fn complete_release_evidence_passes() {
        assert!(validate_phase7_release_evidence(&passing_evidence()).is_empty());
    }

    #[test]
    fn unsigned_first_launch_and_unfrozen_alpha_bug_are_hard_failures() {
        let mut evidence = passing_evidence();
        evidence.artifact.signed_package = false;
        evidence.startup.includes_installed_first_launch = false;
        evidence.alpha.accepted_defects += 1;
        let failures = validate_phase7_release_evidence(&evidence).join("\n");
        assert!(failures.contains("not signed"));
        assert!(failures.contains("first launch"));
        assert!(failures.contains("frozen regression"));
    }

    #[test]
    fn zero_or_negative_placeholder_measurements_cannot_pass() {
        let mut evidence = passing_evidence();
        evidence.artifact.compressed_distribution_bytes = 0;
        evidence.interaction.trailing_edit_ms = -1.0;
        let failures = validate_phase7_release_evidence(&evidence).join("\n");
        assert!(failures.contains("non-zero"));
        assert!(failures.contains("trailing edit evidence"));
    }
}
