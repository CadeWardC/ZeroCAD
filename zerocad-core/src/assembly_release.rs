//! Assembly release-performance evidence.
//!
//! Normal CI proves the shape of the scalable path. Wall-clock claims are
//! accepted only from an explicit, versioned 31-sample run on named reference
//! hardware.

use serde::{Deserialize, Serialize};

pub const ASSEMBLY_V1_EVIDENCE_SCHEMA: u16 = 1;
pub const ASSEMBLY_V1_REFERENCE_OCCURRENCES: u32 = 1_000;
pub const ASSEMBLY_V1_REQUIRED_SAMPLES: usize = 31;
pub const ASSEMBLY_V1_OPEN_MEDIAN_BUDGET_MS: f64 = 1_500.0;
pub const ASSEMBLY_V1_DRAG_MEDIAN_BUDGET_MS: f64 = 16.7;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyV1ReleaseEvidence {
    pub schema: u16,
    pub commit: String,
    pub build_profile: String,
    pub machine_fingerprint: String,
    pub corpus: String,
    pub occurrence_count: u32,
    pub hydrated_open_ms: Vec<f64>,
    pub manipulation_frame_ms: Vec<f64>,
}

impl AssemblyV1ReleaseEvidence {
    pub fn hydrated_open_median_ms(&self) -> Option<f64> {
        median(&self.hydrated_open_ms)
    }

    pub fn manipulation_frame_median_ms(&self) -> Option<f64> {
        median(&self.manipulation_frame_ms)
    }

    pub fn hydrated_open_max_ms(&self) -> Option<f64> {
        finite_max(&self.hydrated_open_ms)
    }

    pub fn manipulation_frame_max_ms(&self) -> Option<f64> {
        finite_max(&self.manipulation_frame_ms)
    }
}

pub fn validate_assembly_v1_release_evidence(evidence: &AssemblyV1ReleaseEvidence) -> Vec<String> {
    let mut failures = Vec::new();
    require(
        &mut failures,
        evidence.schema == ASSEMBLY_V1_EVIDENCE_SCHEMA,
        "assembly evidence schema is unsupported or stale",
    );
    for (label, value) in [
        ("commit", evidence.commit.as_str()),
        ("machine fingerprint", evidence.machine_fingerprint.as_str()),
        ("corpus", evidence.corpus.as_str()),
    ] {
        let normalized = value.trim().to_ascii_lowercase();
        require(
            &mut failures,
            !normalized.is_empty()
                && !normalized.contains("placeholder")
                && normalized != "todo"
                && normalized != "unknown",
            &format!("{label} must be measured and cannot be a placeholder"),
        );
    }
    require(
        &mut failures,
        evidence.build_profile.eq_ignore_ascii_case("release"),
        "assembly performance evidence must use a release build",
    );
    require(
        &mut failures,
        evidence.occurrence_count == ASSEMBLY_V1_REFERENCE_OCCURRENCES,
        "assembly evidence must use the pinned 1,000-occurrence corpus",
    );
    validate_samples("hydrated open", &evidence.hydrated_open_ms, &mut failures);
    validate_samples(
        "manipulation frame",
        &evidence.manipulation_frame_ms,
        &mut failures,
    );
    if let Some(median) = evidence.hydrated_open_median_ms() {
        require(
            &mut failures,
            median <= ASSEMBLY_V1_OPEN_MEDIAN_BUDGET_MS,
            "hydrated-open median exceeds 1,500 ms",
        );
    }
    if let Some(median) = evidence.manipulation_frame_median_ms() {
        require(
            &mut failures,
            median <= ASSEMBLY_V1_DRAG_MEDIAN_BUDGET_MS,
            "manipulation-frame median exceeds 16.7 ms",
        );
    }
    // Maxima are intentionally diagnostic. A compositor or scheduler hitch
    // must remain visible in evidence without turning one noisy sample into the
    // release decision.
    failures
}

pub fn validate_assembly_v1_release_evidence_for_commit(
    evidence: &AssemblyV1ReleaseEvidence,
    expected_commit: &str,
) -> Vec<String> {
    let mut failures = validate_assembly_v1_release_evidence(evidence);
    let expected = expected_commit.trim();
    require(
        &mut failures,
        !expected.is_empty() && evidence.commit.trim().eq_ignore_ascii_case(expected),
        "assembly performance evidence is stale for the current commit",
    );
    failures
}

fn require(failures: &mut Vec<String>, condition: bool, message: &str) {
    if !condition {
        failures.push(message.to_owned());
    }
}

fn validate_samples(label: &str, samples: &[f64], failures: &mut Vec<String>) {
    if samples.len() != ASSEMBLY_V1_REQUIRED_SAMPLES {
        failures.push(format!(
            "{label} evidence requires exactly {ASSEMBLY_V1_REQUIRED_SAMPLES} samples"
        ));
    }
    if samples
        .iter()
        .any(|sample| !sample.is_finite() || *sample <= 0.0)
    {
        failures.push(format!(
            "{label} samples must all be finite, measured, and positive"
        ));
    }
}

fn median(samples: &[f64]) -> Option<f64> {
    if samples.is_empty() || samples.iter().any(|sample| !sample.is_finite()) {
        return None;
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(f64::total_cmp);
    Some(sorted[sorted.len() / 2])
}

fn finite_max(samples: &[f64]) -> Option<f64> {
    samples
        .iter()
        .copied()
        .filter(|sample| sample.is_finite())
        .max_by(f64::total_cmp)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn passing() -> AssemblyV1ReleaseEvidence {
        AssemblyV1ReleaseEvidence {
            schema: ASSEMBLY_V1_EVIDENCE_SCHEMA,
            commit: "0123456789abcdef".into(),
            build_profile: "release".into(),
            machine_fingerprint: "zerocad-reference-2026-01".into(),
            corpus: "assembly-v1-1000-repeated-bracket".into(),
            occurrence_count: 1_000,
            hydrated_open_ms: vec![1_000.0; 31],
            manipulation_frame_ms: vec![10.0; 31],
        }
    }

    #[test]
    fn complete_measured_evidence_passes() {
        assert!(validate_assembly_v1_release_evidence(&passing()).is_empty());
    }

    #[test]
    fn placeholder_or_missing_run_cannot_pass() {
        let mut evidence = passing();
        evidence.commit = "PLACEHOLDER".into();
        evidence.hydrated_open_ms.clear();
        let failures = validate_assembly_v1_release_evidence(&evidence).join("\n");
        assert!(failures.contains("commit"));
        assert!(failures.contains("exactly 31 samples"));
    }

    #[test]
    fn median_is_gated_but_maximum_is_diagnostic() {
        let mut evidence = passing();
        evidence.hydrated_open_ms[0] = 50_000.0;
        assert!(validate_assembly_v1_release_evidence(&evidence).is_empty());
        assert_eq!(evidence.hydrated_open_max_ms(), Some(50_000.0));
        evidence.hydrated_open_ms.fill(1_501.0);
        assert!(validate_assembly_v1_release_evidence(&evidence)
            .iter()
            .any(|failure| failure.contains("median")));
    }

    #[test]
    fn current_commit_check_rejects_stale_evidence() {
        let failures =
            validate_assembly_v1_release_evidence_for_commit(&passing(), "different-commit");
        assert!(failures.iter().any(|failure| failure.contains("stale")));
    }
}
