//! Deterministic, policy-driven healing for Phase 1 solid results.

use core::fmt;

use openrcad_foundation::{TolerancePolicy, TolerancePolicyError};
use openrcad_geom::GeomCurve;
use openrcad_topo::{
    Diagnostic, HealthReport, OperationResult, PcurveBuildError, RecoveryAction, RecoveryReport,
    Solid, TopologyHistory, ValidationReport,
};

use crate::merge::{
    consolidate_collinear_edges_with_policy, heal_tjunctions_with_policy,
    merge_cocylindrical_faces_classed_with_policy, merge_coplanar_faces_classed_with_policy,
};
use crate::sew::sew_with_policy;

/// Failure of the shared Phase 1 healing pipeline.
#[derive(Clone, Debug, PartialEq)]
pub enum HealingError {
    InvalidTolerancePolicy(TolerancePolicyError),
    PcurveBuild(PcurveBuildError),
    InvalidOutput {
        report: HealthReport,
        watertight: bool,
        pcurves_complete: bool,
    },
}

impl fmt::Display for HealingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "healing: invalid tolerance policy: {error}")
            }
            Self::PcurveBuild(error) => write!(f, "healing: pcurve repair failed: {error}"),
            Self::InvalidOutput {
                report,
                watertight,
                pcurves_complete,
            } => write!(
                f,
                "healing: invalid output (watertight={watertight}, pcurves_complete={pcurves_complete}): {report:?}"
            ),
        }
    }
}

impl std::error::Error for HealingError {}

/// Heal a solid using the standard document tolerance policy.
pub fn heal_solid_operation(solid: &Solid) -> Result<OperationResult<Solid>, HealingError> {
    heal_solid_operation_with_policy(solid, &TolerancePolicy::STANDARD)
}

/// Run the deterministic Phase 1 healing sequence.
///
/// Sewing rethreads wires, joins only gaps within `policy.sewing`, and orients
/// the shell. T-junctions are then split, safe collinear cleanup occurs as part
/// of same-domain merging, affected pcurves are rebuilt, and the complete strict
/// representation gate is applied. No intermediate candidate escapes on error.
pub fn heal_solid_operation_with_policy(
    solid: &Solid,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, HealingError> {
    policy
        .validate()
        .map_err(HealingError::InvalidTolerancePolicy)?;

    let mut recovery = RecoveryReport::default();
    let original_faces = solid.face_count();

    let sewn = Solid::new(
        sew_with_policy(&solid.shell().faces(), policy)
            .map_err(HealingError::InvalidTolerancePolicy)?,
    );
    if sewn != *solid {
        recovery.actions.push(RecoveryAction::SewFaces {
            face_count: original_faces,
        });
    }

    let before_edges = sewn.edge_count();
    let mut healed = heal_tjunctions_with_policy(&sewn, policy);
    if healed.edge_count() != before_edges {
        recovery.actions.push(RecoveryAction::HealTJunctions);
    }

    let before_edges = healed.edge_count();
    let before_small_edges = small_linear_edge_lengths(&healed, policy);
    healed = consolidate_collinear_edges_with_policy(&healed, policy);
    if healed.edge_count() < before_edges {
        let remaining_small_edges = small_linear_edge_lengths(&healed, policy).len();
        for length in before_small_edges.iter().take(
            before_small_edges
                .len()
                .saturating_sub(remaining_small_edges),
        ) {
            recovery
                .actions
                .push(RecoveryAction::CollapseSmallEdge { length: *length });
        }
        recovery
            .actions
            .push(RecoveryAction::ConsolidateCollinearEdges {
                removed_edges: before_edges - healed.edge_count(),
            });
    }

    // Splitting and consolidation can replace coedges. Rebuild those pcurves
    // before orientation and same-domain merging so every later stage operates
    // on a complete representation.
    let (repaired, rebuilt) = healed
        .repair_pcurves(policy)
        .map_err(HealingError::PcurveBuild)?;
    healed = repaired;
    if rebuilt > 0 {
        recovery
            .actions
            .push(RecoveryAction::ReconstructPcurves { count: rebuilt });
    }

    let oriented = Solid::new(
        sew_with_policy(&healed.shell().faces(), policy)
            .map_err(HealingError::InvalidTolerancePolicy)?,
    );
    if oriented != healed {
        recovery.actions.push(RecoveryAction::SewFaces {
            face_count: healed.face_count(),
        });
        healed = oriented;
    }

    let before_faces = healed.face_count();
    healed = merge_coplanar_faces_classed_with_policy(&healed, None, policy);
    if healed.face_count() < before_faces {
        recovery.actions.push(RecoveryAction::MergeCoplanarFaces {
            removed_faces: before_faces - healed.face_count(),
        });
    }

    let before_faces = healed.face_count();
    healed = merge_cocylindrical_faces_classed_with_policy(&healed, None, policy);
    if healed.face_count() < before_faces {
        recovery
            .actions
            .push(RecoveryAction::MergeCocylindricalFaces {
                removed_faces: before_faces - healed.face_count(),
            });
    }

    let (healed, rebuilt) = healed
        .repair_pcurves(policy)
        .map_err(HealingError::PcurveBuild)?;
    if rebuilt > 0 {
        recovery
            .actions
            .push(RecoveryAction::ReconstructPcurves { count: rebuilt });
    }

    let validation = ValidationReport::for_solid(&healed, policy);
    if !validation.is_valid() || healed.validate_strict_with_policy(policy).is_err() {
        return Err(HealingError::InvalidOutput {
            report: validation.health,
            watertight: validation.watertight,
            pcurves_complete: validation.pcurves_complete,
        });
    }

    let diagnostics = recovery
        .was_modified()
        .then(|| {
            Diagnostic::info(
                "healing.applied",
                format!(
                    "applied {} deterministic healing actions",
                    recovery.actions.len()
                ),
                None,
            )
        })
        .into_iter()
        .collect();

    Ok(OperationResult {
        history: TopologyHistory::generated_solid(&healed),
        value: healed,
        diagnostics,
        recovery,
        validation,
    })
}

fn small_linear_edge_lengths(solid: &Solid, policy: &TolerancePolicy) -> Vec<f64> {
    let model_scale = solid
        .bounding_box()
        .corners()
        .map_or(policy.resolution, |(minimum, maximum)| {
            minimum.distance(&maximum)
        });
    let limit = policy.sliver_tolerance(model_scale);
    let mut lengths: Vec<_> = solid
        .edges()
        .into_iter()
        .filter(|edge| matches!(edge.curve(), Some(GeomCurve::Line(_))))
        .map(|edge| edge.start().point().distance(&edge.end().point()))
        .filter(|length| *length <= limit)
        .collect();
    lengths.sort_by(f64::total_cmp);
    lengths
}

#[cfg(test)]
mod tests {
    use openrcad_foundation::Pnt;
    use openrcad_geom::Curve;
    use openrcad_primitives::make_box_operation;
    use openrcad_topo::{arena::VertexData, BRepBuilder};

    use super::*;

    #[test]
    fn canonical_healing_preserves_a_strict_box() {
        let source = make_box_operation(&Pnt::origin(), 4.0, 5.0, 6.0)
            .expect("box operation")
            .value;
        let result = heal_solid_operation(&source).expect("healing operation");

        assert!(result.validation.is_valid());
        assert!(result
            .value
            .validate_strict_with_policy(&TolerancePolicy::STANDARD)
            .is_ok());
        assert!(result
            .history
            .coverage_for_solid(&result.value)
            .is_complete());
    }

    #[test]
    fn canonical_healing_rejects_disconnected_closed_components() {
        let first = make_box_operation(&Pnt::origin(), 2.0, 2.0, 2.0)
            .unwrap()
            .value;
        let second = make_box_operation(&Pnt::new(5.0, 0.0, 0.0), 2.0, 2.0, 2.0)
            .unwrap()
            .value;
        let mut faces = first.shell().faces().to_vec();
        faces.extend(second.shell().faces().iter().cloned());
        let disconnected = Solid::new(crate::sew::sew(&faces, 1.0e-6));

        assert!(matches!(
            heal_solid_operation(&disconnected),
            Err(HealingError::InvalidOutput { .. })
        ));
    }

    #[test]
    fn canonical_healing_records_safe_small_collinear_collapse() {
        let source = make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0)
            .unwrap()
            .value;
        let edge = source
            .edges()
            .into_iter()
            .find(|edge| matches!(edge.curve(), Some(GeomCurve::Line(_))))
            .expect("box line edge");
        let mut builder = BRepBuilder::from_brep((**source.brep()).clone());
        let data = builder.brep().edges[edge.id()].clone();
        let fraction = 1.5e-6;
        let t = data.first + (data.last - data.first) * fraction;
        let point = data.curve.as_ref().expect("line support").point(t);
        let start = builder.brep().vertices[data.start].point;
        let end = builder.brep().vertices[data.end].point;
        let coincident_edges: Vec<_> = builder
            .brep()
            .edges
            .iter()
            .filter_map(|(id, candidate)| {
                let a = builder.brep().vertices[candidate.start].point;
                let b = builder.brep().vertices[candidate.end].point;
                ((a.distance(&start) <= 1.0e-9 && b.distance(&end) <= 1.0e-9)
                    || (a.distance(&end) <= 1.0e-9 && b.distance(&start) <= 1.0e-9))
                    .then_some(id)
            })
            .collect();
        let vertex = builder.brep_mut().vertices.insert(VertexData {
            point,
            tolerance: TolerancePolicy::STANDARD.linear,
        });
        for edge_id in coincident_edges {
            let candidate = builder.brep().edges[edge_id].clone();
            let candidate_start = builder.brep().vertices[candidate.start].point;
            let candidate_fraction = if candidate_start.distance(&start) <= 1.0e-9 {
                fraction
            } else {
                1.0 - fraction
            };
            let candidate_t =
                candidate.first + (candidate.last - candidate.first) * candidate_fraction;
            builder.split_edge(edge_id, vertex, candidate_t);
        }
        let split = Solid::from_id(builder.build(), source.id());

        let policy = TolerancePolicy {
            pcurve_consistency: 1.0e-5,
            ..TolerancePolicy::STANDARD
        };
        let result =
            heal_solid_operation_with_policy(&split, &policy).expect("safe split should heal");
        assert!(result.recovery.actions.iter().any(|action| matches!(
            action,
            RecoveryAction::CollapseSmallEdge { length } if *length > 0.0
        )));
        assert!(result
            .recovery
            .actions
            .iter()
            .any(|action| matches!(action, RecoveryAction::ConsolidateCollinearEdges { .. })));
        assert!(result.validation.is_valid());
    }
}
