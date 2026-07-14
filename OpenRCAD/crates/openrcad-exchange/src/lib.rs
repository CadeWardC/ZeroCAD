#![forbid(unsafe_code)]
//! Data exchange for OpenRCAD (OCCT `TKSTEP` / `TKSTL`).
//!
//! Reads and writes CAD data. STL export supports ASCII and binary output from a
//! [`TriangleMesh`](openrcad_mesh::TriangleMesh). STEP read/write supports the
//! OpenRCAD B-Rep subset used by the primitive and analytic-surface tests.

pub mod step_reader;
pub mod step_writer;
pub mod stl;
pub mod threemf;

pub use step_writer::write_step;
pub use stl::{write_stl_ascii, write_stl_binary};
pub use threemf::{to_3mf_bytes, write_3mf};

use std::io;

use openrcad_foundation::{TolerancePolicy, TolerancePolicyError};
use openrcad_topo::{
    Diagnostic, HealthReport, OperationResult, PcurveBuildError, RecoveryAction, RecoveryReport,
    Solid, TopologyHistory, ValidationError, ValidationReport,
};

/// How a STEP operation handles legacy files without complete stored pcurves.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MissingPcurveCompatibility {
    /// Strict Phase 1 behavior: missing or inconsistent pcurves are rejected.
    #[default]
    Reject,
    /// Reconstruct missing/stale pcurves, validate them, and report recovery.
    ReconstructAndValidate,
}

/// STEP import behavior beyond the document tolerance policy.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepImportOptions {
    pub missing_pcurves: MissingPcurveCompatibility,
}

impl StepImportOptions {
    pub const STRICT: Self = Self {
        missing_pcurves: MissingPcurveCompatibility::Reject,
    };
    pub const LEGACY_PCURVE_RECONSTRUCTION: Self = Self {
        missing_pcurves: MissingPcurveCompatibility::ReconstructAndValidate,
    };
}

/// Failure from a canonical STEP import operation.
#[derive(Debug)]
pub enum StepImportError {
    Io(io::Error),
    InvalidTolerancePolicy(TolerancePolicyError),
    InvalidTopology(ValidationError),
    PcurveBuild(PcurveBuildError),
    InvalidOutput {
        report: HealthReport,
        watertight: bool,
        pcurves_complete: bool,
    },
}

impl core::fmt::Display for StepImportError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "STEP I/O failed: {error}"),
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "invalid STEP tolerance policy: {error}")
            }
            Self::InvalidTopology(error) => write!(f, "invalid STEP topology: {error}"),
            Self::PcurveBuild(error) => write!(f, "STEP pcurve recovery failed: {error}"),
            Self::InvalidOutput {
                report,
                watertight,
                pcurves_complete,
            } => write!(
                f,
                "invalid STEP solid (watertight={watertight}, pcurves_complete={pcurves_complete}): {report:?}"
            ),
        }
    }
}

impl std::error::Error for StepImportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::InvalidTolerancePolicy(error) => Some(error),
            Self::InvalidTopology(error) => Some(error),
            Self::PcurveBuild(error) => Some(error),
            Self::InvalidOutput { .. } => None,
        }
    }
}

impl From<io::Error> for StepImportError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

impl StepImportError {
    fn into_io(self) -> io::Error {
        match self {
            Self::Io(error) => error,
            error => io::Error::new(io::ErrorKind::InvalidData, error),
        }
    }
}

/// Strict STEP import using the standard document tolerance policy.
pub fn read_step_operation(path: &str) -> Result<OperationResult<Solid>, StepImportError> {
    read_step_operation_with_policy(path, &TolerancePolicy::STANDARD)
}

/// Strict policy-aware STEP import.
pub fn read_step_operation_with_policy(
    path: &str,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, StepImportError> {
    read_step_operation_with_policy_and_options(path, policy, StepImportOptions::STRICT)
}

/// Policy-aware STEP import with an explicit legacy-pcurve policy.
pub fn read_step_operation_with_policy_and_options(
    path: &str,
    policy: &TolerancePolicy,
    options: StepImportOptions,
) -> Result<OperationResult<Solid>, StepImportError> {
    let solid = step_reader::read_step_unchecked(path)?;
    finish_step_import(solid, policy, options)
}

/// Strict STEP string import using the standard policy.
pub fn read_step_str_operation(
    content: &str,
) -> Result<OperationResult<Solid>, StepImportError> {
    read_step_str_operation_with_policy(content, &TolerancePolicy::STANDARD)
}

/// Strict policy-aware STEP string import.
pub fn read_step_str_operation_with_policy(
    content: &str,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, StepImportError> {
    read_step_str_operation_with_policy_and_options(content, policy, StepImportOptions::STRICT)
}

/// Policy-aware STEP string import with explicit compatibility behavior.
pub fn read_step_str_operation_with_policy_and_options(
    content: &str,
    policy: &TolerancePolicy,
    options: StepImportOptions,
) -> Result<OperationResult<Solid>, StepImportError> {
    let solid = step_reader::read_step_str_unchecked(content)?;
    finish_step_import(solid, policy, options)
}

fn finish_step_import(
    solid: Solid,
    policy: &TolerancePolicy,
    options: StepImportOptions,
) -> Result<OperationResult<Solid>, StepImportError> {
    policy
        .validate()
        .map_err(StepImportError::InvalidTolerancePolicy)?;
    let mut recovery = RecoveryReport::default();
    let mut diagnostics = Vec::new();
    let solid = match options.missing_pcurves {
        MissingPcurveCompatibility::Reject => solid,
        MissingPcurveCompatibility::ReconstructAndValidate => {
            let (solid, reconstructed) = solid
                .repair_pcurves(policy)
                .map_err(StepImportError::PcurveBuild)?;
            if reconstructed > 0 {
                recovery
                    .actions
                    .push(RecoveryAction::ReconstructPcurves { count: reconstructed });
                diagnostics.push(Diagnostic::warning(
                    "openrcad.step.legacy-pcurve-reconstruction",
                    format!("reconstructed and validated {reconstructed} STEP coedge pcurves"),
                    None,
                ));
            }
            solid
        }
    };
    solid
        .validate_strict_with_policy(policy)
        .map_err(StepImportError::InvalidTopology)?;
    let validation = ValidationReport::for_solid(&solid, policy);
    if !validation.is_valid() {
        return Err(StepImportError::InvalidOutput {
            report: validation.health,
            watertight: validation.watertight,
            pcurves_complete: validation.pcurves_complete,
        });
    }
    let history = TopologyHistory::generated_solid(&solid);
    Ok(OperationResult {
        value: solid,
        history,
        diagnostics,
        recovery,
        validation,
    })
}

/// Deprecated compatibility reader. It reconstructs legacy missing pcurves and
/// discards validation, recovery, diagnostics, and topology history.
#[deprecated(note = "use read_step_operation; this wrapper enables legacy pcurve reconstruction and discards metadata")]
pub fn read_step(path: &str) -> io::Result<Solid> {
    read_step_operation_with_policy_and_options(
        path,
        &TolerancePolicy::STANDARD,
        StepImportOptions::LEGACY_PCURVE_RECONSTRUCTION,
    )
    .map(|result| result.value)
    .map_err(StepImportError::into_io)
}

/// String equivalent of [`read_step`].
#[deprecated(note = "use read_step_str_operation; this wrapper enables legacy pcurve reconstruction and discards metadata")]
pub fn read_step_str(content: &str) -> io::Result<Solid> {
    read_step_str_operation_with_policy_and_options(
        content,
        &TolerancePolicy::STANDARD,
        StepImportOptions::LEGACY_PCURVE_RECONSTRUCTION,
    )
    .map(|result| result.value)
    .map_err(StepImportError::into_io)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax2, Dir, Pnt};
    use openrcad_primitives::{
        make_box_operation, make_cylinder_operation, make_sphere_operation,
    };

    fn assert_close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-5, "Expected {} to be close to {}", a, b);
    }

    fn check_roundtrip(solid: &openrcad_topo::Solid, name: &str) {
        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join(format!("openrcad_test_{}.stp", name));
        let path_str = path.to_str().unwrap();

        // Write to STEP
        write_step(solid, path_str).expect("Failed to write STEP");

        // Read from STEP
        let parsed = read_step_operation(path_str).expect("Failed to read STEP");

        // Clean up
        let _ = std::fs::remove_file(path);

        // Compare topological counts
        assert!(parsed.validation.is_valid());
        assert!(parsed.history.coverage_for_solid(&parsed.value).is_complete());
        assert!(!parsed.recovery.was_modified());
        let parsed = parsed.value;
        assert_eq!(solid.vertex_count(), parsed.vertex_count());
        assert_eq!(solid.edge_count(), parsed.edge_count());
        assert_eq!(solid.face_count(), parsed.face_count());

        // Compare bounding box
        let (lo_orig, hi_orig) = solid.bounding_box().corners().unwrap();
        let (lo_parsed, hi_parsed) = parsed.bounding_box().corners().unwrap();
        assert_close(lo_orig.x(), lo_parsed.x());
        assert_close(lo_orig.y(), lo_parsed.y());
        assert_close(lo_orig.z(), lo_parsed.z());
        assert_close(hi_orig.x(), hi_parsed.x());
        assert_close(hi_orig.y(), hi_parsed.y());
        assert_close(hi_orig.z(), hi_parsed.z());
    }

    #[test]
    fn step_box_roundtrip() {
        let s = make_box_operation(&Pnt::origin(), 2.0, 3.0, 4.0)
            .unwrap()
            .value;
        check_roundtrip(&s, "box");
    }

    #[test]
    fn step_cylinder_roundtrip() {
        let s = make_cylinder_operation(
            &Ax2::new(Pnt::origin(), Dir::new(0.0, 0.0, 1.0)),
            1.5,
            5.0,
        )
        .unwrap()
        .value;
        check_roundtrip(&s, "cylinder");
    }

    #[test]
    fn step_sphere_roundtrip() {
        let s = make_sphere_operation(&Pnt::origin(), 2.5).unwrap().value;
        check_roundtrip(&s, "sphere");
    }

    #[test]
    fn read_step_str_matches_read_step() {
        let s = make_box_operation(&Pnt::origin(), 2.0, 3.0, 4.0)
            .unwrap()
            .value;
        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join("openrcad_test_read_str.stp");
        let path_str = path.to_str().unwrap();
        write_step(&s, path_str).expect("write");
        let via_path = read_step_operation(path_str).expect("read via path").value;
        let content = std::fs::read_to_string(&path).expect("read text");
        let _ = std::fs::remove_file(&path);
        let via_str = read_step_str_operation(&content)
            .expect("read via str")
            .value;
        assert_eq!(via_path.face_count(), via_str.face_count());
        assert_eq!(via_path.edge_count(), via_str.edge_count());
        assert_eq!(via_path.vertex_count(), via_str.vertex_count());
    }

    #[test]
    fn strict_step_rejects_missing_pcurves_and_named_compatibility_reports_recovery() {
        let solid = make_box_operation(&Pnt::origin(), 2.0, 3.0, 4.0)
            .unwrap()
            .value;
        let path = std::env::temp_dir().join("openrcad_test_legacy_pcurves.stp");
        write_step(&solid, path.to_str().unwrap()).unwrap();
        let content = std::fs::read_to_string(&path)
            .unwrap()
            .replace("PCURVE('',", "LEGACY_CURVE('',");
        let _ = std::fs::remove_file(path);

        assert!(matches!(
            read_step_str_operation(&content),
            Err(StepImportError::InvalidTopology(
                ValidationError::MissingPcurve { .. }
            ))
        ));

        let recovered = read_step_str_operation_with_policy_and_options(
            &content,
            &TolerancePolicy::STANDARD,
            StepImportOptions::LEGACY_PCURVE_RECONSTRUCTION,
        )
        .unwrap();
        assert!(recovered.validation.is_valid());
        assert!(recovered.recovery.was_modified());
        assert!(recovered.diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "openrcad.step.legacy-pcurve-reconstruction"
        }));
    }
}
