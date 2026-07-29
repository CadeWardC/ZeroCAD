//! Assembly V2 solver calibration and release evidence.
//!
//! The solver tolerances are runtime policy rather than serialized contracts.
//! This validator makes their calibration explicit and refuses missing,
//! placeholder, stale, non-reference, or accuracy-relaxing evidence.

use crate::{
    MateSolvePath, ANGLE_TOLERANCE_CEILING_RAD, ANGLE_TOLERANCE_RAD, TRANSLATION_TOLERANCE_ABS_MM,
    TRANSLATION_TOLERANCE_CEILING_ABS_MM, TRANSLATION_TOLERANCE_CEILING_RELATIVE,
    TRANSLATION_TOLERANCE_RELATIVE,
};
use serde::{Deserialize, Serialize};

pub const ASSEMBLY_V2_EVIDENCE_SCHEMA: u16 = 1;
pub const ASSEMBLY_V2_REQUIRED_SAMPLES: usize = 31;
pub const ASSEMBLY_V2_STRESS_OCCURRENCES: u32 = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MateStressCorpusKind {
    SparseChain,
    DenseConstraintGraph,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MateCalibrationCaseEvidence {
    pub name: String,
    pub path: MateSolvePath,
    pub assembly_diagonal_mm: f64,
    pub final_translation_residual_mm: Vec<f64>,
    pub final_angle_residual_rad: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MateCalibrationDecision {
    pub translation_p99_floor_mm: f64,
    pub angle_p99_floor_rad: f64,
    pub candidate_translation_tolerance_mm: f64,
    pub candidate_angle_tolerance_rad: f64,
    pub committed_translation_tolerance_mm: f64,
    pub committed_angle_tolerance_rad: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MateStressEvidence {
    pub name: String,
    pub kind: MateStressCorpusKind,
    pub occurrence_count: u32,
    pub mate_count: u32,
    pub offline_solve_ms: Vec<f64>,
    pub drag_frame_ms: Vec<f64>,
    pub nonlinear_iterations: Vec<u32>,
    pub damping_retries: Vec<u32>,
    pub sparse_iterations: Vec<u32>,
}

impl MateStressEvidence {
    pub fn offline_solve_median_ms(&self) -> Option<f64> {
        median(&self.offline_solve_ms)
    }

    pub fn drag_frame_median_ms(&self) -> Option<f64> {
        median(&self.drag_frame_ms)
    }

    pub fn offline_solve_max_ms(&self) -> Option<f64> {
        finite_max(&self.offline_solve_ms)
    }

    pub fn drag_frame_max_ms(&self) -> Option<f64> {
        finite_max(&self.drag_frame_ms)
    }

    pub fn maximum_nonlinear_iterations(&self) -> Option<u32> {
        self.nonlinear_iterations.iter().copied().max()
    }

    pub fn maximum_damping_retries(&self) -> Option<u32> {
        self.damping_retries.iter().copied().max()
    }

    pub fn maximum_sparse_iterations(&self) -> Option<u32> {
        self.sparse_iterations.iter().copied().max()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyV2ReleaseEvidence {
    pub schema: u16,
    pub commit: String,
    pub build_profile: String,
    pub machine_fingerprint: String,
    pub calibration_cases: Vec<MateCalibrationCaseEvidence>,
    pub stress_corpora: Vec<MateStressEvidence>,
}

pub fn calibration_decision(
    evidence: &MateCalibrationCaseEvidence,
) -> Option<MateCalibrationDecision> {
    let translation_p99_floor_mm = p99(&evidence.final_translation_residual_mm)?;
    let angle_p99_floor_rad = p99(&evidence.final_angle_residual_rad)?;
    let diagonal = evidence.assembly_diagonal_mm.abs();
    let committed_translation_tolerance_mm =
        (diagonal * TRANSLATION_TOLERANCE_RELATIVE).max(TRANSLATION_TOLERANCE_ABS_MM);
    Some(MateCalibrationDecision {
        translation_p99_floor_mm,
        angle_p99_floor_rad,
        candidate_translation_tolerance_mm: committed_translation_tolerance_mm
            .max(10.0 * translation_p99_floor_mm),
        candidate_angle_tolerance_rad: ANGLE_TOLERANCE_RAD.max(10.0 * angle_p99_floor_rad),
        committed_translation_tolerance_mm,
        committed_angle_tolerance_rad: ANGLE_TOLERANCE_RAD,
    })
}

pub fn validate_assembly_v2_release_evidence(evidence: &AssemblyV2ReleaseEvidence) -> Vec<String> {
    let mut failures = Vec::new();
    require(
        &mut failures,
        evidence.schema == ASSEMBLY_V2_EVIDENCE_SCHEMA,
        "Assembly V2 evidence schema is unsupported or stale",
    );
    for (label, value) in [
        ("commit", evidence.commit.as_str()),
        ("machine fingerprint", evidence.machine_fingerprint.as_str()),
    ] {
        require_non_placeholder(&mut failures, label, value);
    }
    require(
        &mut failures,
        evidence.build_profile.eq_ignore_ascii_case("release"),
        "Assembly V2 solver evidence must use a release build",
    );
    require(
        &mut failures,
        !evidence.calibration_cases.is_empty(),
        "Assembly V2 evidence requires satisfiable calibration cases",
    );

    let mut has_dense_calibration = false;
    let mut has_sparse_calibration = false;
    for case in &evidence.calibration_cases {
        require_non_placeholder(&mut failures, "calibration case name", &case.name);
        require(
            &mut failures,
            case.assembly_diagonal_mm.is_finite() && case.assembly_diagonal_mm >= 0.0,
            &format!(
                "calibration case '{}' requires a finite non-negative assembly diagonal",
                case.name
            ),
        );
        validate_residual_samples(
            &case.name,
            "translation",
            &case.final_translation_residual_mm,
            &mut failures,
        );
        validate_residual_samples(
            &case.name,
            "angle",
            &case.final_angle_residual_rad,
            &mut failures,
        );
        has_dense_calibration |= case.path == MateSolvePath::DenseSvd;
        has_sparse_calibration |= case.path == MateSolvePath::SparsePcg;

        if let Some(decision) = calibration_decision(case) {
            let translation_ceiling = (case.assembly_diagonal_mm.abs()
                * TRANSLATION_TOLERANCE_CEILING_RELATIVE)
                .max(TRANSLATION_TOLERANCE_CEILING_ABS_MM);
            require(
                &mut failures,
                decision.candidate_translation_tolerance_mm <= translation_ceiling,
                &format!(
                    "calibration case '{}' exceeds the translation accuracy ceiling",
                    case.name
                ),
            );
            require(
                &mut failures,
                decision.candidate_angle_tolerance_rad <= ANGLE_TOLERANCE_CEILING_RAD,
                &format!(
                    "calibration case '{}' exceeds the angular accuracy ceiling",
                    case.name
                ),
            );
            require(
                &mut failures,
                decision.candidate_translation_tolerance_mm
                    <= decision.committed_translation_tolerance_mm,
                &format!(
                    "calibration case '{}' requires committing a calibrated translation tolerance",
                    case.name
                ),
            );
            require(
                &mut failures,
                decision.candidate_angle_tolerance_rad <= decision.committed_angle_tolerance_rad,
                &format!(
                    "calibration case '{}' requires committing a calibrated angular tolerance",
                    case.name
                ),
            );
        }
    }
    require(
        &mut failures,
        has_dense_calibration && has_sparse_calibration,
        "calibration evidence must cover both dense-SVD and sparse-PCG solve paths",
    );

    let mut has_sparse_chain = false;
    let mut has_dense_graph = false;
    for stress in &evidence.stress_corpora {
        require_non_placeholder(&mut failures, "stress corpus name", &stress.name);
        require(
            &mut failures,
            stress.occurrence_count == ASSEMBLY_V2_STRESS_OCCURRENCES,
            &format!(
                "stress corpus '{}' must contain exactly 1,000 occurrences",
                stress.name
            ),
        );
        require(
            &mut failures,
            stress.mate_count > 0,
            &format!("stress corpus '{}' must contain mates", stress.name),
        );
        validate_positive_samples(
            &stress.name,
            "offline solve",
            &stress.offline_solve_ms,
            &mut failures,
        );
        validate_positive_samples(
            &stress.name,
            "drag frame",
            &stress.drag_frame_ms,
            &mut failures,
        );
        validate_count_samples(
            &stress.name,
            "nonlinear iteration",
            &stress.nonlinear_iterations,
            &mut failures,
        );
        validate_count_samples(
            &stress.name,
            "damping retry",
            &stress.damping_retries,
            &mut failures,
        );
        validate_count_samples(
            &stress.name,
            "sparse iteration",
            &stress.sparse_iterations,
            &mut failures,
        );
        has_sparse_chain |= stress.kind == MateStressCorpusKind::SparseChain;
        has_dense_graph |= stress.kind == MateStressCorpusKind::DenseConstraintGraph;
    }
    require(
        &mut failures,
        has_sparse_chain && has_dense_graph,
        "stress evidence must cover 1,000-occurrence sparse-chain and dense-constraint corpora",
    );
    failures
}

pub fn validate_assembly_v2_release_evidence_for_commit(
    evidence: &AssemblyV2ReleaseEvidence,
    expected_commit: &str,
) -> Vec<String> {
    let mut failures = validate_assembly_v2_release_evidence(evidence);
    let expected = expected_commit.trim();
    require(
        &mut failures,
        !expected.is_empty() && evidence.commit.trim().eq_ignore_ascii_case(expected),
        "Assembly V2 solver evidence is stale for the current commit",
    );
    failures
}

fn require(failures: &mut Vec<String>, condition: bool, message: &str) {
    if !condition {
        failures.push(message.to_owned());
    }
}

fn require_non_placeholder(failures: &mut Vec<String>, label: &str, value: &str) {
    let normalized = value.trim().to_ascii_lowercase();
    require(
        failures,
        !normalized.is_empty()
            && !normalized.contains("placeholder")
            && normalized != "todo"
            && normalized != "unknown",
        &format!("{label} must be measured and cannot be a placeholder"),
    );
}

fn validate_residual_samples(case: &str, unit: &str, samples: &[f64], failures: &mut Vec<String>) {
    if samples.len() != ASSEMBLY_V2_REQUIRED_SAMPLES {
        failures.push(format!(
            "calibration case '{case}' {unit} evidence requires exactly \
             {ASSEMBLY_V2_REQUIRED_SAMPLES} samples"
        ));
    }
    if samples
        .iter()
        .any(|sample| !sample.is_finite() || *sample < 0.0)
    {
        failures.push(format!(
            "calibration case '{case}' {unit} residuals must be finite and non-negative"
        ));
    }
}

fn validate_positive_samples(
    corpus: &str,
    metric: &str,
    samples: &[f64],
    failures: &mut Vec<String>,
) {
    if samples.len() != ASSEMBLY_V2_REQUIRED_SAMPLES {
        failures.push(format!(
            "stress corpus '{corpus}' {metric} evidence requires exactly \
             {ASSEMBLY_V2_REQUIRED_SAMPLES} samples"
        ));
    }
    if samples
        .iter()
        .any(|sample| !sample.is_finite() || *sample <= 0.0)
    {
        failures.push(format!(
            "stress corpus '{corpus}' {metric} samples must be finite and positive"
        ));
    }
}

fn validate_count_samples(corpus: &str, metric: &str, samples: &[u32], failures: &mut Vec<String>) {
    if samples.len() != ASSEMBLY_V2_REQUIRED_SAMPLES {
        failures.push(format!(
            "stress corpus '{corpus}' {metric} evidence requires exactly \
             {ASSEMBLY_V2_REQUIRED_SAMPLES} samples"
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

fn p99(samples: &[f64]) -> Option<f64> {
    if samples.is_empty()
        || samples
            .iter()
            .any(|sample| !sample.is_finite() || *sample < 0.0)
    {
        return None;
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(f64::total_cmp);
    let rank = ((sorted.len() as f64) * 0.99).ceil() as usize;
    Some(sorted[rank.saturating_sub(1).min(sorted.len() - 1)])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration(name: &str, path: MateSolvePath) -> MateCalibrationCaseEvidence {
        MateCalibrationCaseEvidence {
            name: name.into(),
            path,
            assembly_diagonal_mm: 1_000.0,
            final_translation_residual_mm: vec![5.0e-8; 31],
            final_angle_residual_rad: vec![5.0e-10; 31],
        }
    }

    fn stress(name: &str, kind: MateStressCorpusKind) -> MateStressEvidence {
        MateStressEvidence {
            name: name.into(),
            kind,
            occurrence_count: 1_000,
            mate_count: 999,
            offline_solve_ms: vec![25.0; 31],
            drag_frame_ms: vec![3.0; 31],
            nonlinear_iterations: vec![4; 31],
            damping_retries: vec![1; 31],
            sparse_iterations: vec![20; 31],
        }
    }

    fn passing() -> AssemblyV2ReleaseEvidence {
        AssemblyV2ReleaseEvidence {
            schema: ASSEMBLY_V2_EVIDENCE_SCHEMA,
            commit: "0123456789abcdef".into(),
            build_profile: "release".into(),
            machine_fingerprint: "zerocad-solver-reference-2026-01".into(),
            calibration_cases: vec![
                calibration("small-four-bar", MateSolvePath::DenseSvd),
                calibration("large-frame", MateSolvePath::SparsePcg),
            ],
            stress_corpora: vec![
                stress("1000-long-chain", MateStressCorpusKind::SparseChain),
                stress(
                    "1000-redundant-frame",
                    MateStressCorpusKind::DenseConstraintGraph,
                ),
            ],
        }
    }

    #[test]
    fn complete_measured_v2_evidence_passes() {
        assert!(validate_assembly_v2_release_evidence(&passing()).is_empty());
    }

    #[test]
    fn placeholders_or_missing_stress_runs_cannot_pass() {
        let mut evidence = passing();
        evidence.commit = "placeholder".into();
        evidence.stress_corpora.pop();
        let failures = validate_assembly_v2_release_evidence(&evidence).join("\n");
        assert!(failures.contains("commit"));
        assert!(failures.contains("dense-constraint"));
    }

    #[test]
    fn residual_floor_cannot_silently_relax_accuracy() {
        let mut evidence = passing();
        evidence.calibration_cases[0]
            .final_translation_residual_mm
            .fill(2.0e-5);
        let failures = validate_assembly_v2_release_evidence(&evidence).join("\n");
        assert!(failures.contains("translation accuracy ceiling"));
        assert!(failures.contains("committing a calibrated translation tolerance"));
    }

    #[test]
    fn p99_uses_the_measured_tail() {
        let mut case = calibration("tail", MateSolvePath::DenseSvd);
        case.final_translation_residual_mm[0] = 9.0e-8;
        assert_eq!(
            calibration_decision(&case)
                .unwrap()
                .translation_p99_floor_mm,
            9.0e-8
        );
    }

    #[test]
    fn current_commit_check_rejects_stale_solver_evidence() {
        let failures =
            validate_assembly_v2_release_evidence_for_commit(&passing(), "different-commit");
        assert!(failures.iter().any(|failure| failure.contains("stale")));
    }
}
