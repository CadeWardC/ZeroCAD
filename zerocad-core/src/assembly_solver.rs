//! Deterministic Assembly V2 mate solver.
//!
//! Authoritative mates and manual placements stay untouched. This module
//! computes derived resolved placements component-by-component and returns them
//! for an explicit transactional commit by the caller.

use crate::{
    AssemblyDocument, AssemblyMate, AssemblyMateKind, MateId, MateSense, OccurrenceId,
    RigidPlacement,
};
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

pub const DENSE_FREE_OCCURRENCE_LIMIT: usize = 32;
pub const DENSE_RESIDUAL_ROW_LIMIT: usize = 512;
pub const COMMITTED_NONLINEAR_ITERATION_LIMIT: usize = 64;
pub const INTERACTIVE_NONLINEAR_ITERATION_LIMIT: usize = 4;
pub const INITIAL_DAMPING: f64 = 1.0e-4;
/// Calibration defaults. Milestone-10 evidence may tighten or relax these
/// within the hard release ceilings; they are runtime policy, not file-format
/// contracts.
pub const TRANSLATION_TOLERANCE_ABS_MM: f64 = 1.0e-6;
pub const TRANSLATION_TOLERANCE_RELATIVE: f64 = 1.0e-9;
pub const ANGLE_TOLERANCE_RAD: f64 = 1.0e-8;
pub const TRANSLATION_TOLERANCE_CEILING_ABS_MM: f64 = 1.0e-4;
pub const TRANSLATION_TOLERANCE_CEILING_RELATIVE: f64 = 1.0e-8;
pub const ANGLE_TOLERANCE_CEILING_RAD: f64 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MateFrame {
    pub origin: [f64; 3],
    pub axis: [f64; 3],
    pub radial: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedMateFrames {
    pub first: MateFrame,
    pub second: Option<MateFrame>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MateSolveContext {
    /// Resolved local frames keyed by mate id. Missing entries remain authored
    /// but are solver-disabled and reported unresolved.
    pub frames: BTreeMap<MateId, ResolvedMateFrames>,
    /// Half the local bbox diagonal, clamped to at least 1 mm by the solver.
    pub characteristic_lengths_mm: BTreeMap<OccurrenceId, f64>,
    /// World assembly diagonal used by the initial translation tolerance.
    pub assembly_diagonal_mm: f64,
    /// Ephemeral full-pose constraints used by interactive manipulation. They
    /// participate in solving but are never authored or serialized.
    pub temporary_pose_targets: BTreeMap<OccurrenceId, RigidPlacement>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MateSolvePath {
    DenseSvd,
    SparsePcg,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MateSolveStatus {
    Solved,
    Redundant,
    Suppressed,
    Unresolved,
    Conflicting,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MateComponentEvidence {
    pub occurrences: Vec<OccurrenceId>,
    pub path: MateSolvePath,
    pub nonlinear_iterations: usize,
    pub damping_retries: usize,
    pub sparse_iterations: usize,
    pub final_position_residual_mm: f64,
    pub final_angle_residual_rad: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MateSolveResult {
    pub converged: bool,
    pub placements: BTreeMap<OccurrenceId, RigidPlacement>,
    pub statuses: BTreeMap<MateId, MateSolveStatus>,
    pub evidence: Vec<MateComponentEvidence>,
    pub diagnostics: Vec<String>,
}

/// Solve every resolvable active connected component.
///
/// Non-convergence is data, not a panic and not an authoritative mutation. A
/// caller adding a new mate transactionally rejects `converged == false`; a
/// loader keeps its last valid resolved placements and attaches diagnostics.
pub fn solve_assembly_mates(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    nonlinear_iteration_limit: usize,
) -> MateSolveResult {
    solve_assembly_mates_with_deadline(assembly, context, nonlinear_iteration_limit, None)
}

fn solve_assembly_mates_with_deadline(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    nonlinear_iteration_limit: usize,
    deadline: Option<Instant>,
) -> MateSolveResult {
    let Some(mate_set) = &assembly.mates else {
        return MateSolveResult {
            converged: true,
            placements: assembly
                .occurrences
                .iter()
                .map(|(id, occurrence)| (*id, occurrence.manual_placement))
                .collect(),
            statuses: BTreeMap::new(),
            evidence: Vec::new(),
            diagnostics: Vec::new(),
        };
    };
    let mut statuses = BTreeMap::new();
    let mut active = Vec::<&AssemblyMate>::new();
    for mate in mate_set.mates.values() {
        if mate.suppressed {
            statuses.insert(mate.id, MateSolveStatus::Suppressed);
        } else if !mate_is_resolved(mate, context) {
            statuses.insert(mate.id, MateSolveStatus::Unresolved);
        } else {
            active.push(mate);
        }
    }
    let components = connected_components(&active, context.temporary_pose_targets.keys().copied());
    let mut placements: BTreeMap<OccurrenceId, RigidPlacement> = assembly
        .occurrences
        .iter()
        .map(|(id, occurrence)| (*id, occurrence.manual_placement))
        .collect();
    let mut evidence = Vec::new();
    let mut diagnostics = Vec::new();
    let mut all_converged = true;

    for component in components {
        let component_mates: Vec<&AssemblyMate> = active
            .iter()
            .copied()
            .filter(|mate| component.contains(&mate.first.occurrence_id))
            .collect();
        let outcome = solve_component(
            assembly,
            context,
            &component,
            &component_mates,
            nonlinear_iteration_limit.max(1),
            None,
            deadline,
        );
        if outcome.converged {
            placements.extend(outcome.placements);
            let redundant = redundant_mate_ids(&component_mates);
            for mate in component_mates {
                statuses.insert(
                    mate.id,
                    if redundant.contains(&mate.id) {
                        MateSolveStatus::Redundant
                    } else {
                        MateSolveStatus::Solved
                    },
                );
            }
        } else {
            all_converged = false;
            for mate in component_mates {
                statuses.insert(mate.id, MateSolveStatus::Conflicting);
            }
            diagnostics.push(format!(
                "mate component {:?} did not converge (position {:.3e} mm, angle {:.3e} rad)",
                component,
                outcome.evidence.final_position_residual_mm,
                outcome.evidence.final_angle_residual_rad
            ));
        }
        evidence.push(outcome.evidence);
    }
    MateSolveResult {
        converged: all_converged,
        placements,
        statuses,
        evidence,
        diagnostics,
    }
}

fn redundant_mate_ids(mates: &[&AssemblyMate]) -> BTreeSet<MateId> {
    let mut accepted = Vec::<&AssemblyMate>::new();
    let mut redundant = BTreeSet::new();
    for mate in mates.iter().copied() {
        let duplicate = accepted.iter().any(|existing| {
            existing.first == mate.first
                && existing.second == mate.second
                && existing.kind == mate.kind
                && existing.sense == mate.sense
        });
        if duplicate {
            redundant.insert(mate.id);
        } else {
            accepted.push(mate);
        }
    }
    redundant
}

pub fn solve_assembly_mates_committed(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
) -> MateSolveResult {
    solve_assembly_mates(assembly, context, COMMITTED_NONLINEAR_ITERATION_LIMIT)
}

pub fn solve_assembly_mates_interactive(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
) -> MateSolveResult {
    solve_assembly_mates_with_deadline(
        assembly,
        context,
        INTERACTIVE_NONLINEAR_ITERATION_LIMIT,
        Some(Instant::now() + Duration::from_millis(4)),
    )
}

/// Apply only a successful solution. Manual placements remain the authored
/// preference and are never overwritten here.
pub fn apply_mate_solution(
    assembly: &mut AssemblyDocument,
    solution: &MateSolveResult,
) -> Result<(), String> {
    if !solution.converged {
        return Err("cannot apply a non-converged mate solution".into());
    }
    for (id, occurrence) in &mut assembly.occurrences {
        occurrence.resolved_placement_override = solution.placements.get(id).copied();
    }
    Ok(())
}

/// Add one authored mate and solve it as an atomic document transaction.
pub fn add_mate_transactionally(
    assembly: &mut AssemblyDocument,
    mut mate: AssemblyMate,
    context: &MateSolveContext,
) -> Result<MateSolveResult, String> {
    let mut candidate = assembly.clone_authoritative();
    let mate_set = candidate.mates.get_or_insert_with(Default::default);
    let id = mate_set.next_mate_id;
    mate.id = id;
    if mate.name.trim().is_empty() {
        mate.name = format!("Mate {id}");
    }
    if mate_set
        .mates
        .values()
        .any(|existing| existing.name.eq_ignore_ascii_case(&mate.name))
    {
        return Err(format!("a mate named `{}` already exists", mate.name));
    }
    mate_set.next_mate_id = id
        .checked_add(1)
        .ok_or_else(|| "mate id space is exhausted".to_owned())?;
    mate_set.mates.insert(id, mate);
    candidate
        .validate_structural_contracts()
        .map_err(|error| error.to_string())?;
    let solution = solve_assembly_mates_committed(&candidate, context);
    if !solution.converged {
        return Err(solution
            .diagnostics
            .first()
            .cloned()
            .unwrap_or_else(|| "mate system did not converge".into()));
    }
    apply_mate_solution(&mut candidate, &solution)?;
    *assembly = candidate;
    Ok(solution)
}

/// Commit an interactive full-pose target without retaining the temporary
/// constraint. The requested target becomes the authored manual-pose
/// preference, then the persistent mate system is solved again.
pub fn apply_pose_target_transactionally(
    assembly: &mut AssemblyDocument,
    occurrence_id: OccurrenceId,
    target: RigidPlacement,
    context: &MateSolveContext,
) -> Result<MateSolveResult, String> {
    if !assembly.occurrences.contains_key(&occurrence_id) {
        return Err(format!("occurrence {occurrence_id} does not exist"));
    }
    let mut target_context = context.clone();
    target_context
        .temporary_pose_targets
        .insert(occurrence_id, target);
    let target_solution = solve_assembly_mates_committed(assembly, &target_context);
    if !target_solution.converged {
        return Err(target_solution
            .diagnostics
            .first()
            .cloned()
            .unwrap_or_else(|| "temporary placement target conflicts with active mates".into()));
    }

    let mut candidate = assembly.clone_authoritative();
    candidate
        .occurrences
        .get_mut(&occurrence_id)
        .expect("occurrence checked above")
        .manual_placement = target;
    let mut persistent_context = context.clone();
    persistent_context
        .temporary_pose_targets
        .remove(&occurrence_id);
    let persistent_solution = solve_assembly_mates_committed(&candidate, &persistent_context);
    if !persistent_solution.converged {
        return Err(persistent_solution
            .diagnostics
            .first()
            .cloned()
            .unwrap_or_else(|| {
                "persistent mate system did not converge after manipulation".into()
            }));
    }
    apply_mate_solution(&mut candidate, &persistent_solution)?;
    *assembly = candidate;
    Ok(persistent_solution)
}

/// Apply a one-shot face target alongside persistent mates, then discard the
/// temporary mate and retain only the resulting manual-pose preference.
pub fn apply_ephemeral_mate_target_transactionally(
    assembly: &mut AssemblyDocument,
    controlled_occurrence_id: OccurrenceId,
    mut temporary_mate: AssemblyMate,
    context: &MateSolveContext,
    frames: ResolvedMateFrames,
) -> Result<MateSolveResult, String> {
    if !assembly.occurrences.contains_key(&controlled_occurrence_id) {
        return Err(format!(
            "occurrence {controlled_occurrence_id} does not exist"
        ));
    }
    let mut candidate = assembly.clone_authoritative();
    let mate_set = candidate.mates.get_or_insert_with(Default::default);
    let temporary_id = mate_set.next_mate_id;
    temporary_mate.id = temporary_id;
    temporary_mate.name = format!("Ephemeral target {temporary_id}");
    temporary_mate.suppressed = false;
    mate_set.mates.insert(temporary_id, temporary_mate);
    let mut temporary_context = context.clone();
    temporary_context.frames.insert(temporary_id, frames);
    let temporary_solution = solve_assembly_mates_committed(&candidate, &temporary_context);
    if !temporary_solution.converged {
        return Err(temporary_solution
            .diagnostics
            .first()
            .cloned()
            .unwrap_or_else(|| "temporary face target conflicts with active mates".into()));
    }
    let solved_pose = temporary_solution
        .placements
        .get(&controlled_occurrence_id)
        .copied()
        .ok_or_else(|| "temporary target did not resolve the controlled occurrence".to_owned())?;
    candidate
        .occurrences
        .get_mut(&controlled_occurrence_id)
        .expect("occurrence checked above")
        .manual_placement = solved_pose;
    candidate
        .mates
        .as_mut()
        .expect("temporary mate set exists")
        .mates
        .remove(&temporary_id);
    if assembly.mates.is_none() {
        candidate.mates = None;
    }
    temporary_context.frames.remove(&temporary_id);
    let persistent_solution = solve_assembly_mates_committed(&candidate, &temporary_context);
    if !persistent_solution.converged {
        return Err(persistent_solution
            .diagnostics
            .first()
            .cloned()
            .unwrap_or_else(|| "persistent mate system did not converge after alignment".into()));
    }
    apply_mate_solution(&mut candidate, &persistent_solution)?;
    *assembly = candidate;
    Ok(persistent_solution)
}

fn mate_is_resolved(mate: &AssemblyMate, context: &MateSolveContext) -> bool {
    let Some(frames) = context.frames.get(&mate.id) else {
        return matches!(mate.kind, AssemblyMateKind::Fixed);
    };
    if matches!(mate.kind, AssemblyMateKind::Fixed) {
        true
    } else {
        frames.second.is_some()
    }
}

fn connected_components(
    active: &[&AssemblyMate],
    temporary_targets: impl IntoIterator<Item = OccurrenceId>,
) -> Vec<BTreeSet<OccurrenceId>> {
    let mut adjacency = BTreeMap::<OccurrenceId, BTreeSet<OccurrenceId>>::new();
    for mate in active {
        let first = mate.first.occurrence_id;
        adjacency.entry(first).or_default();
        if let Some(second) = &mate.second {
            let second = second.occurrence_id;
            adjacency.entry(first).or_default().insert(second);
            adjacency.entry(second).or_default().insert(first);
        }
    }
    for occurrence_id in temporary_targets {
        adjacency.entry(occurrence_id).or_default();
    }
    let mut remaining: BTreeSet<OccurrenceId> = adjacency.keys().copied().collect();
    let mut components = Vec::new();
    while let Some(start) = remaining.pop_first() {
        let mut component = BTreeSet::new();
        let mut pending = vec![start];
        while let Some(id) = pending.pop() {
            if !component.insert(id) {
                continue;
            }
            remaining.remove(&id);
            if let Some(neighbors) = adjacency.get(&id) {
                pending.extend(neighbors.iter().rev().copied());
            }
        }
        components.push(component);
    }
    components
}

struct ComponentOutcome {
    converged: bool,
    placements: BTreeMap<OccurrenceId, RigidPlacement>,
    evidence: MateComponentEvidence,
}

fn solve_component(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    component: &BTreeSet<OccurrenceId>,
    mates: &[&AssemblyMate],
    iteration_limit: usize,
    forced_path: Option<MateSolvePath>,
    deadline: Option<Instant>,
) -> ComponentOutcome {
    let fixed_by_mate: BTreeSet<OccurrenceId> = mates
        .iter()
        .filter(|mate| matches!(mate.kind, AssemblyMateKind::Fixed))
        .map(|mate| mate.first.occurrence_id)
        .collect();
    let free: Vec<OccurrenceId> = component
        .iter()
        .copied()
        .filter(|id| !assembly.occurrences[id].grounded && !fixed_by_mate.contains(id))
        .collect();
    let targets: Vec<(OccurrenceId, RigidPlacement)> = context
        .temporary_pose_targets
        .iter()
        .filter_map(|(id, placement)| component.contains(id).then_some((*id, *placement)))
        .collect();
    let variable_index: BTreeMap<OccurrenceId, usize> = free
        .iter()
        .enumerate()
        .map(|(index, id)| (*id, index))
        .collect();
    let mut x = DVector::<f64>::zeros(free.len() * 6);
    let mut lambda = INITIAL_DAMPING;
    let mut damping_retries = 0;
    let mut sparse_iterations = 0;
    let mut path = MateSolvePath::DenseSvd;
    let mut iterations = 0;
    let position_tolerance = (context.assembly_diagonal_mm.abs() * TRANSLATION_TOLERANCE_RELATIVE)
        .max(TRANSLATION_TOLERANCE_ABS_MM);
    let angle_tolerance = ANGLE_TOLERANCE_RAD;

    for iteration in 0..iteration_limit {
        if deadline.is_some_and(|deadline| Instant::now() >= deadline) {
            break;
        }
        iterations = iteration + 1;
        let linearized = linearize(
            assembly,
            context,
            mates,
            &targets,
            &free,
            &variable_index,
            &x,
        );
        if linearized.max_position_mm <= position_tolerance
            && linearized.max_angle_rad <= angle_tolerance
        {
            let placements = placements_from_variables(assembly, context, &free, &x);
            return ComponentOutcome {
                converged: true,
                placements,
                evidence: MateComponentEvidence {
                    occurrences: component.iter().copied().collect(),
                    path,
                    nonlinear_iterations: iterations,
                    damping_retries,
                    sparse_iterations,
                    final_position_residual_mm: linearized.max_position_mm,
                    final_angle_residual_rad: linearized.max_angle_rad,
                },
            };
        }
        if x.is_empty() {
            break;
        }
        path = forced_path.unwrap_or({
            if free.len() <= DENSE_FREE_OCCURRENCE_LIMIT
                && linearized.rows.len() <= DENSE_RESIDUAL_ROW_LIMIT
            {
                MateSolvePath::DenseSvd
            } else {
                MateSolvePath::SparsePcg
            }
        });
        let mut accepted = false;
        for _ in 0..8 {
            if deadline.is_some_and(|deadline| Instant::now() >= deadline) {
                break;
            }
            let (step, pcg_iterations) = match path {
                MateSolvePath::DenseSvd => (dense_damped_step(&linearized, &x, lambda), 0),
                MateSolvePath::SparsePcg => sparse_damped_step(&linearized, &x, lambda, deadline),
            };
            sparse_iterations += pcg_iterations;
            let Some(step) = step else {
                lambda *= 4.0;
                damping_retries += 1;
                continue;
            };
            let candidate = &x + step;
            let candidate_residual = evaluate_residuals(
                assembly,
                context,
                mates,
                &targets,
                &free,
                &variable_index,
                &candidate,
            );
            if residual_cost(&candidate_residual) < residual_cost(&linearized.residuals) {
                x = candidate;
                lambda = (lambda / 4.0).max(1.0e-12);
                accepted = true;
                break;
            }
            lambda *= 4.0;
            damping_retries += 1;
        }
        if !accepted {
            break;
        }
    }

    let final_residuals = evaluate_residuals(
        assembly,
        context,
        mates,
        &targets,
        &free,
        &variable_index,
        &x,
    );
    let (max_position_residual_mm, max_angle_residual_rad) = residual_maxima(&final_residuals);
    ComponentOutcome {
        converged: false,
        placements: BTreeMap::new(),
        evidence: MateComponentEvidence {
            occurrences: component.iter().copied().collect(),
            path,
            nonlinear_iterations: iterations,
            damping_retries,
            sparse_iterations,
            final_position_residual_mm: max_position_residual_mm,
            final_angle_residual_rad: max_angle_residual_rad,
        },
    }
}

#[derive(Clone, Copy)]
enum ResidualUnit {
    Position { scale_mm: f64 },
    Angle,
}

#[derive(Clone, Copy)]
struct ResidualValue {
    scaled: f64,
    physical: f64,
    unit: ResidualUnit,
}

struct SparseRow {
    residual: f64,
    terms: Vec<(usize, f64)>,
}

struct Linearized {
    residuals: Vec<ResidualValue>,
    rows: Vec<SparseRow>,
    diagonal: DVector<f64>,
    max_position_mm: f64,
    max_angle_rad: f64,
}

fn linearize(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    mates: &[&AssemblyMate],
    targets: &[(OccurrenceId, RigidPlacement)],
    free: &[OccurrenceId],
    variable_index: &BTreeMap<OccurrenceId, usize>,
    x: &DVector<f64>,
) -> Linearized {
    let poses = placements_from_variables(assembly, context, free, x);
    let mut residuals = Vec::new();
    let mut mate_ranges = Vec::with_capacity(mates.len());
    for mate in mates {
        let start = residuals.len();
        residuals.extend(evaluate_mate_residuals(context, mate, &poses, None));
        mate_ranges.push(start..residuals.len());
    }
    let mut target_ranges = Vec::with_capacity(targets.len());
    for (occurrence_id, target) in targets {
        let start = residuals.len();
        residuals.extend(evaluate_pose_target_residuals(
            context,
            *occurrence_id,
            *target,
            poses[occurrence_id],
        ));
        target_ranges.push(start..residuals.len());
    }
    let mut rows: Vec<SparseRow> = residuals
        .iter()
        .map(|residual| SparseRow {
            residual: residual.scaled,
            terms: Vec::new(),
        })
        .collect();
    const H: f64 = 1.0e-6;
    for (block, occurrence_id) in free.iter().copied().enumerate() {
        for local_dof in 0..6 {
            let dof = block * 6 + local_dof;
            let mut perturbed = x.clone();
            perturbed[dof] += H;
            let changed_pose =
                placement_for_variable(assembly, context, occurrence_id, block, &perturbed);
            for (mate_index, mate) in mates.iter().enumerate().filter(|(_, mate)| {
                mate.first.occurrence_id == occurrence_id
                    || mate
                        .second
                        .as_ref()
                        .is_some_and(|entity| entity.occurrence_id == occurrence_id)
            }) {
                let changed = evaluate_mate_residuals(
                    context,
                    mate,
                    &poses,
                    Some((occurrence_id, changed_pose)),
                );
                for (row_index, changed) in mate_ranges[mate_index].clone().zip(changed.iter()) {
                    let derivative = (changed.scaled - residuals[row_index].scaled) / H;
                    if derivative.abs() > 1.0e-14 {
                        rows[row_index].terms.push((dof, derivative));
                    }
                }
            }
            for (target_index, (_, target)) in targets
                .iter()
                .enumerate()
                .filter(|(_, (target_id, _))| *target_id == occurrence_id)
            {
                let changed =
                    evaluate_pose_target_residuals(context, occurrence_id, *target, changed_pose);
                for (row_index, changed) in target_ranges[target_index].clone().zip(changed.iter())
                {
                    let derivative = (changed.scaled - residuals[row_index].scaled) / H;
                    if derivative.abs() > 1.0e-14 {
                        rows[row_index].terms.push((dof, derivative));
                    }
                }
            }
        }
    }
    debug_assert_eq!(variable_index.len(), free.len());
    let mut diagonal = DVector::<f64>::zeros(x.len());
    for row in &rows {
        for (column, derivative) in &row.terms {
            diagonal[*column] += derivative * derivative;
        }
    }
    for value in diagonal.iter_mut() {
        *value = value.max(1.0e-12);
    }
    let (max_position_mm, max_angle_rad) = residual_maxima(&residuals);
    Linearized {
        residuals,
        rows,
        diagonal,
        max_position_mm,
        max_angle_rad,
    }
}

fn dense_damped_step(
    linearized: &Linearized,
    x: &DVector<f64>,
    lambda: f64,
) -> Option<DVector<f64>> {
    let row_count = linearized.rows.len();
    let column_count = x.len();
    let mut a = DMatrix::<f64>::zeros(row_count, column_count);
    let mut residual = DVector::<f64>::zeros(row_count);
    for (row_index, row) in linearized.rows.iter().enumerate() {
        residual[row_index] = row.residual;
        for (column, derivative) in &row.terms {
            a[(row_index, *column)] = *derivative / linearized.diagonal[*column].sqrt();
        }
    }
    let svd = a.clone().svd(true, true);
    let u = svd.u?;
    let v_t = svd.v_t?;
    let projection = u.transpose() * residual;
    let mut scaled_step = DVector::<f64>::zeros(column_count);
    for singular_index in 0..svd.singular_values.len() {
        let singular = svd.singular_values[singular_index];
        let coefficient = -singular * projection[singular_index] / (singular * singular + lambda);
        for column in 0..column_count {
            scaled_step[column] += v_t[(singular_index, column)] * coefficient;
        }
    }
    let mut step = DVector::<f64>::zeros(column_count);
    for column in 0..column_count {
        step[column] = scaled_step[column] / linearized.diagonal[column].sqrt();
    }

    // Exact numerical null-space projection toward the authored manual pose.
    // `AᵀA` supplies a full basis even when the thin SVD omits n-m vectors.
    let eigen = (a.transpose() * a).symmetric_eigen();
    let null_columns: Vec<usize> = eigen
        .eigenvalues
        .iter()
        .enumerate()
        .filter_map(|(index, value)| (*value <= 1.0e-20).then_some(index))
        .collect();
    if !null_columns.is_empty() {
        let mut null = DMatrix::<f64>::zeros(column_count, null_columns.len());
        for (target_column, source_column) in null_columns.iter().enumerate() {
            for row in 0..column_count {
                null[(row, target_column)] =
                    eigen.eigenvectors[(row, *source_column)] / linearized.diagonal[row].sqrt();
            }
        }
        let gram = null.transpose() * &null;
        if let Ok(coefficients) = gram
            .svd(true, true)
            .solve(&(-null.transpose() * (x + &step)), 1.0e-12)
        {
            step += null * coefficients;
        }
    }
    step.iter().all(|value| value.is_finite()).then_some(step)
}

fn sparse_damped_step(
    linearized: &Linearized,
    x: &DVector<f64>,
    lambda: f64,
    deadline: Option<Instant>,
) -> (Option<DVector<f64>>, usize) {
    let n = x.len();
    let soft = 1.0e-6;
    let mut rhs = DVector::<f64>::zeros(n);
    for row in &linearized.rows {
        for (column, derivative) in &row.terms {
            rhs[*column] -= derivative * row.residual;
        }
    }
    // The soft manual-pose preference regularizes each increment. Starting
    // from the manual pose means unconstrained directions remain there, while
    // repeated nonlinear steps can still drive satisfiable geometric
    // residuals below tolerance instead of retaining a scale-dependent bias.
    let apply = |vector: &DVector<f64>| {
        let mut result = DVector::<f64>::zeros(n);
        for row in &linearized.rows {
            let jv: f64 = row
                .terms
                .iter()
                .map(|(column, derivative)| derivative * vector[*column])
                .sum();
            for (column, derivative) in &row.terms {
                result[*column] += derivative * jv;
            }
        }
        for index in 0..n {
            result[index] += (lambda + soft) * linearized.diagonal[index] * vector[index];
        }
        result
    };
    let inverse_blocks = block_jacobi_inverse(linearized, lambda + soft, n);
    let precondition = |vector: &DVector<f64>| {
        let mut result = DVector::<f64>::zeros(n);
        for (block_index, inverse) in inverse_blocks.iter().enumerate() {
            let start = block_index * 6;
            for row in 0..6 {
                for column in 0..6 {
                    if start + row < n && start + column < n {
                        result[start + row] += inverse[(row, column)] * vector[start + column];
                    }
                }
            }
        }
        result
    };
    let mut solution = DVector::<f64>::zeros(n);
    let mut residual = rhs.clone();
    let mut z = precondition(&residual);
    let mut direction = z.clone();
    let mut rz = residual.dot(&z);
    let initial_norm = residual.norm().max(1.0e-30);
    let limit = (4 * n).clamp(1, 1_024);
    for iteration in 0..limit {
        if deadline.is_some_and(|deadline| Instant::now() >= deadline) {
            return (None, iteration);
        }
        let ad = apply(&direction);
        let denominator = direction.dot(&ad);
        if !denominator.is_finite() || denominator.abs() < 1.0e-30 {
            return (None, iteration);
        }
        let alpha = rz / denominator;
        solution += alpha * &direction;
        residual -= alpha * ad;
        if residual.norm() / initial_norm <= 1.0e-8 {
            return (
                solution
                    .iter()
                    .all(|value| value.is_finite())
                    .then_some(solution),
                iteration + 1,
            );
        }
        z = precondition(&residual);
        let next_rz = residual.dot(&z);
        if !next_rz.is_finite() {
            return (None, iteration + 1);
        }
        direction = &z + (next_rz / rz) * direction;
        rz = next_rz;
    }
    (None, limit)
}

fn block_jacobi_inverse(
    linearized: &Linearized,
    regularization: f64,
    dof_count: usize,
) -> Vec<nalgebra::SMatrix<f64, 6, 6>> {
    use nalgebra::SMatrix;
    let block_count = dof_count.div_ceil(6);
    let mut blocks = vec![SMatrix::<f64, 6, 6>::zeros(); block_count];
    for row in &linearized.rows {
        for (block_index, block) in blocks.iter_mut().enumerate() {
            let start = block_index * 6;
            for (column_a, derivative_a) in row
                .terms
                .iter()
                .filter(|(column, _)| *column / 6 == block_index)
            {
                for (column_b, derivative_b) in row
                    .terms
                    .iter()
                    .filter(|(column, _)| *column / 6 == block_index)
                {
                    block[(*column_a - start, *column_b - start)] += derivative_a * derivative_b;
                }
            }
        }
    }
    for (block_index, block) in blocks.iter_mut().enumerate() {
        for axis in 0..6 {
            let dof = block_index * 6 + axis;
            if dof < dof_count {
                block[(axis, axis)] += regularization * linearized.diagonal[dof];
            } else {
                block[(axis, axis)] = 1.0;
            }
        }
    }
    blocks
        .into_iter()
        .map(|block| {
            block.try_inverse().unwrap_or_else(|| {
                let mut inverse = SMatrix::<f64, 6, 6>::zeros();
                for axis in 0..6 {
                    inverse[(axis, axis)] = block[(axis, axis)].max(1.0e-12).recip();
                }
                inverse
            })
        })
        .collect()
}

fn evaluate_residuals(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    mates: &[&AssemblyMate],
    targets: &[(OccurrenceId, RigidPlacement)],
    free: &[OccurrenceId],
    variable_index: &BTreeMap<OccurrenceId, usize>,
    x: &DVector<f64>,
) -> Vec<ResidualValue> {
    let poses = placements_from_variables(assembly, context, free, x);
    let mut residuals = Vec::new();
    for mate in mates {
        residuals.extend(evaluate_mate_residuals(context, mate, &poses, None));
    }
    for (occurrence_id, target) in targets {
        residuals.extend(evaluate_pose_target_residuals(
            context,
            *occurrence_id,
            *target,
            poses[occurrence_id],
        ));
    }
    // Keep the variable map part of this boundary explicit: it ensures callers
    // cannot accidentally build residuals against a differently ordered DOF
    // set.
    debug_assert_eq!(variable_index.len(), free.len());
    residuals
}

fn evaluate_pose_target_residuals(
    context: &MateSolveContext,
    occurrence_id: OccurrenceId,
    target: RigidPlacement,
    pose: RigidPlacement,
) -> Vec<ResidualValue> {
    let pose_translation = pose.translation();
    let target_translation = target.translation();
    let translation = Vector3::new(
        pose_translation[0] - target_translation[0],
        pose_translation[1] - target_translation[1],
        pose_translation[2] - target_translation[2],
    );
    let [pw, px, py, pz] = pose.rotation();
    let [tw, tx, ty, tz] = target.rotation();
    let pose_rotation = UnitQuaternion::new_normalize(Quaternion::new(pw, px, py, pz));
    let target_rotation = UnitQuaternion::new_normalize(Quaternion::new(tw, tx, ty, tz));
    let rotation = (target_rotation.inverse() * pose_rotation).scaled_axis();
    let mut residuals = Vec::with_capacity(6);
    push_position_vector(
        &mut residuals,
        translation,
        characteristic_length(context, occurrence_id),
    );
    push_angle_vector(&mut residuals, rotation);
    residuals
}

fn evaluate_mate_residuals(
    context: &MateSolveContext,
    mate: &AssemblyMate,
    poses: &BTreeMap<OccurrenceId, RigidPlacement>,
    pose_override: Option<(OccurrenceId, RigidPlacement)>,
) -> Vec<ResidualValue> {
    if matches!(mate.kind, AssemblyMateKind::Fixed) {
        return Vec::new();
    }
    let pose = |id: OccurrenceId| {
        pose_override
            .filter(|(override_id, _)| *override_id == id)
            .map(|(_, placement)| placement)
            .unwrap_or(poses[&id])
    };
    let frames = &context.frames[&mate.id];
    let first = world_frame(frames.first, pose(mate.first.occurrence_id));
    let second_ref = mate.second.as_ref().expect("active non-fixed mate");
    let second = world_frame(
        frames.second.expect("active non-fixed frame"),
        pose(second_ref.occurrence_id),
    );
    let scale_mm = 0.5
        * (characteristic_length(context, mate.first.occurrence_id)
            + characteristic_length(context, second_ref.occurrence_id));
    let sense = match mate.sense {
        MateSense::Aligned => 1.0,
        MateSense::AntiAligned => -1.0,
    };
    let desired_second_axis = second.axis * sense;
    let axis_cross = first.axis.cross(&desired_second_axis);
    let delta = second.origin - first.origin;
    let mut residuals = Vec::new();
    match mate.kind {
        AssemblyMateKind::Coincident => {
            push_position_vector(&mut residuals, delta, scale_mm);
            push_angle_vector(&mut residuals, axis_cross);
        }
        AssemblyMateKind::Concentric => {
            let perpendicular = delta - first.axis * delta.dot(&first.axis);
            push_position_vector(&mut residuals, perpendicular, scale_mm);
            push_angle_vector(&mut residuals, axis_cross);
        }
        AssemblyMateKind::SignedDistance { millimeters } => {
            push_position_scalar(
                &mut residuals,
                delta.dot(&first.axis) - millimeters,
                scale_mm,
            );
            push_angle_vector(&mut residuals, axis_cross);
        }
        AssemblyMateKind::Angle { radians } => {
            let angle = first.axis.dot(&desired_second_axis).clamp(-1.0, 1.0).acos();
            push_angle_scalar(&mut residuals, angle - radians);
        }
        AssemblyMateKind::Fixed => {}
    }
    residuals
}

fn placements_from_variables(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    free: &[OccurrenceId],
    x: &DVector<f64>,
) -> BTreeMap<OccurrenceId, RigidPlacement> {
    let mut placements: BTreeMap<OccurrenceId, RigidPlacement> = assembly
        .occurrences
        .iter()
        .map(|(id, occurrence)| (*id, occurrence.manual_placement))
        .collect();
    for (block, occurrence_id) in free.iter().enumerate() {
        placements.insert(
            *occurrence_id,
            placement_for_variable(assembly, context, *occurrence_id, block, x),
        );
    }
    placements
}

fn placement_for_variable(
    assembly: &AssemblyDocument,
    context: &MateSolveContext,
    occurrence_id: OccurrenceId,
    block: usize,
    x: &DVector<f64>,
) -> RigidPlacement {
    let manual = assembly.occurrences[&occurrence_id].manual_placement;
    let length = characteristic_length(context, occurrence_id);
    let translation = manual.translation();
    let translated = [
        translation[0] + x[block * 6] * length,
        translation[1] + x[block * 6 + 1] * length,
        translation[2] + x[block * 6 + 2] * length,
    ];
    let rotation_vector = Vector3::new(x[block * 6 + 3], x[block * 6 + 4], x[block * 6 + 5]);
    let delta = UnitQuaternion::from_scaled_axis(rotation_vector);
    let [w, qx, qy, qz] = manual.rotation();
    let authored = UnitQuaternion::new_normalize(Quaternion::new(w, qx, qy, qz));
    let rotation = delta * authored;
    let quaternion = rotation.quaternion();
    RigidPlacement::new(
        translated,
        [quaternion.w, quaternion.i, quaternion.j, quaternion.k],
    )
    .expect("finite solver variables produce a valid rigid placement")
}

struct WorldFrame {
    origin: Vector3<f64>,
    axis: Vector3<f64>,
}

fn world_frame(frame: MateFrame, placement: RigidPlacement) -> WorldFrame {
    let [w, x, y, z] = placement.rotation();
    let rotation = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
    let translation = Vector3::from_row_slice(&placement.translation());
    WorldFrame {
        origin: rotation * Vector3::from_row_slice(&frame.origin) + translation,
        axis: (rotation * Vector3::from_row_slice(&frame.axis)).normalize(),
    }
}

fn characteristic_length(context: &MateSolveContext, occurrence_id: OccurrenceId) -> f64 {
    context
        .characteristic_lengths_mm
        .get(&occurrence_id)
        .copied()
        .filter(|length| length.is_finite())
        .unwrap_or(1.0)
        .abs()
        .max(1.0)
}

fn push_position_vector(residuals: &mut Vec<ResidualValue>, vector: Vector3<f64>, scale_mm: f64) {
    for value in vector.iter().copied() {
        push_position_scalar(residuals, value, scale_mm);
    }
}

fn push_position_scalar(residuals: &mut Vec<ResidualValue>, value: f64, scale_mm: f64) {
    residuals.push(ResidualValue {
        scaled: value / scale_mm,
        physical: value.abs(),
        unit: ResidualUnit::Position { scale_mm },
    });
}

fn push_angle_vector(residuals: &mut Vec<ResidualValue>, vector: Vector3<f64>) {
    for value in vector.iter().copied() {
        push_angle_scalar(residuals, value);
    }
}

fn push_angle_scalar(residuals: &mut Vec<ResidualValue>, value: f64) {
    residuals.push(ResidualValue {
        scaled: value,
        physical: value.abs(),
        unit: ResidualUnit::Angle,
    });
}

fn residual_cost(residuals: &[ResidualValue]) -> f64 {
    0.5 * residuals
        .iter()
        .map(|residual| residual.scaled * residual.scaled)
        .sum::<f64>()
}

fn residual_maxima(residuals: &[ResidualValue]) -> (f64, f64) {
    let mut position: f64 = 0.0;
    let mut angle: f64 = 0.0;
    for residual in residuals {
        match residual.unit {
            ResidualUnit::Position { scale_mm } => {
                debug_assert!(scale_mm >= 1.0);
                position = position.max(residual.physical);
            }
            ResidualUnit::Angle => angle = angle.max(residual.physical),
        }
    }
    (position, angle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AssemblyDefinition, AssemblyEntityRef, AssemblyLocalSelector, AssemblyMateSet,
        AssemblyOccurrence,
    };
    use std::sync::Arc;

    fn occurrence(id: u64, translation: [f64; 3], grounded: bool) -> AssemblyOccurrence {
        AssemblyOccurrence {
            id,
            definition_model_hash: [id as u8; 32],
            name: format!("Part:{id}"),
            manual_placement: RigidPlacement::new(translation, [1.0, 0.0, 0.0, 0.0]).unwrap(),
            grounded,
            resolved_placement_override: None,
        }
    }

    fn entity(id: u64) -> AssemblyEntityRef {
        AssemblyEntityRef {
            occurrence_id: id,
            local_body_id: "body".into(),
            local_selector: AssemblyLocalSelector::Origin,
        }
    }

    fn frame() -> MateFrame {
        MateFrame {
            origin: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            radial: [1.0, 0.0, 0.0],
        }
    }

    #[test]
    fn coincident_mate_moves_only_the_free_occurrence() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [10.0, 0.0, 0.0], false));
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Coincident".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::Coincident,
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context
            .characteristic_lengths_mm
            .extend([(1, 5.0), (2, 5.0)]);
        context.assembly_diagonal_mm = 10.0;

        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
        assert_eq!(solved.placements[&1].translation(), [0.0; 3]);
        assert!(solved.placements[&2].translation()[0].abs() < 1.0e-6);
        assert_eq!(solved.evidence[0].path, MateSolvePath::DenseSvd);
    }

    #[test]
    fn concentric_underconstraint_keeps_manual_slide_preference() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [5.0, 0.0, 7.0], false));
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Concentric".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::Concentric,
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context
            .characteristic_lengths_mm
            .extend([(1, 5.0), (2, 5.0)]);
        context.assembly_diagonal_mm = 10.0;

        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
        let translation = solved.placements[&2].translation();
        assert!(translation[0].abs() < 1.0e-6);
        assert!((translation[2] - 7.0).abs() < 1.0e-6);
    }

    #[test]
    fn signed_distance_mate_solves_in_millimetres() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [0.0; 3], false));
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Distance".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::SignedDistance { millimeters: 12.5 },
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context.assembly_diagonal_mm = 20.0;
        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
        assert!((solved.placements[&2].translation()[2] - 12.5).abs() < 1.0e-6);
    }

    #[test]
    fn multiple_grounded_components_can_form_a_satisfiable_frame() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [0.0; 3], true));
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Frame".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::Coincident,
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
    }

    #[test]
    fn inconsistent_fixed_component_is_reported_without_mutation() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [10.0, 0.0, 0.0], true));
        for id in [1u64, 2] {
            let hash = [id as u8; 32];
            assembly.definitions.insert(
                hash,
                Arc::new(AssemblyDefinition {
                    model_hash: hash,
                    name: format!("Definition {id}"),
                    source_basename: None,
                    compact_snapshot: Arc::new(vec![1]),
                }),
            );
        }
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Impossible".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::Coincident,
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context.assembly_diagonal_mm = 10.0;
        let before = assembly.clone();
        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(!solved.converged);
        assert_eq!(assembly, before);
        assert_eq!(solved.statuses[&1], MateSolveStatus::Conflicting);
    }

    #[test]
    fn dense_and_sparse_paths_agree_at_the_solver_boundary() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [3.0, -4.0, 2.0], false));
        let mate = AssemblyMate {
            id: 1,
            name: "Boundary".into(),
            suppressed: false,
            first: entity(1),
            second: Some(entity(2)),
            kind: AssemblyMateKind::Coincident,
            sense: MateSense::Aligned,
        };
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(1, mate);
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context
            .characteristic_lengths_mm
            .extend([(1, 5.0), (2, 5.0)]);
        context.assembly_diagonal_mm = 10.0;
        let component = BTreeSet::from([1, 2]);
        let mate_ref = &assembly.mates.as_ref().unwrap().mates[&1];
        let dense = solve_component(
            &assembly,
            &context,
            &component,
            &[mate_ref],
            64,
            Some(MateSolvePath::DenseSvd),
            None,
        );
        let sparse = solve_component(
            &assembly,
            &context,
            &component,
            &[mate_ref],
            64,
            Some(MateSolvePath::SparsePcg),
            None,
        );
        assert!(dense.converged);
        assert!(sparse.converged, "{:?}", sparse.evidence);
        for axis in 0..3 {
            assert!(
                (dense.placements[&2].translation()[axis]
                    - sparse.placements[&2].translation()[axis])
                    .abs()
                    < 1.0e-6
            );
        }
    }

    #[test]
    fn component_above_dense_threshold_uses_sparse_path() {
        let mut assembly = AssemblyDocument::new();
        let mut mate_set = AssemblyMateSet::default();
        let mut context = MateSolveContext::default();
        for id in 1..=34 {
            assembly.occurrences.insert(
                id,
                occurrence(id, [(id - 1) as f64 * 0.1, 0.0, 0.0], id == 1),
            );
            context.characteristic_lengths_mm.insert(id, 5.0);
            if id > 1 {
                let mate_id = id - 1;
                mate_set.mates.insert(
                    mate_id,
                    AssemblyMate {
                        id: mate_id,
                        name: format!("Chain {mate_id}"),
                        suppressed: false,
                        first: entity(id - 1),
                        second: Some(entity(id)),
                        kind: AssemblyMateKind::Coincident,
                        sense: MateSense::Aligned,
                    },
                );
                context.frames.insert(
                    mate_id,
                    ResolvedMateFrames {
                        first: frame(),
                        second: Some(frame()),
                    },
                );
            }
        }
        mate_set.next_mate_id = 34;
        assembly.mates = Some(mate_set);
        context.assembly_diagonal_mm = 10.0;
        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
        assert_eq!(solved.evidence[0].path, MateSolvePath::SparsePcg);
        assert!(solved.evidence[0].sparse_iterations > 0);
    }

    #[test]
    fn failed_new_mate_is_transactional() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [10.0, 0.0, 0.0], true));
        for id in [1u64, 2] {
            let hash = [id as u8; 32];
            assembly.definitions.insert(
                hash,
                Arc::new(AssemblyDefinition {
                    model_hash: hash,
                    name: format!("Definition {id}"),
                    source_basename: None,
                    compact_snapshot: Arc::new(vec![1]),
                }),
            );
        }
        let before = assembly.clone();
        let mate = AssemblyMate {
            id: 0,
            name: "Impossible".into(),
            suppressed: false,
            first: entity(1),
            second: Some(entity(2)),
            kind: AssemblyMateKind::Coincident,
            sense: MateSense::Aligned,
        };
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        let result = add_mate_transactionally(&mut assembly, mate, &context);
        assert!(result.is_err());
        assert_eq!(assembly, before);
    }

    #[test]
    fn later_exact_duplicate_mate_is_reported_redundant() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [0.0; 3], false));
        let first = AssemblyMate {
            id: 1,
            name: "Coincident 1".into(),
            suppressed: false,
            first: entity(1),
            second: Some(entity(2)),
            kind: AssemblyMateKind::Coincident,
            sense: MateSense::Aligned,
        };
        let mut second = first.clone();
        second.id = 2;
        second.name = "Coincident 2".into();
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(1, first);
        mate_set.mates.insert(2, second);
        mate_set.next_mate_id = 3;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        for mate_id in [1, 2] {
            context.frames.insert(
                mate_id,
                ResolvedMateFrames {
                    first: frame(),
                    second: Some(frame()),
                },
            );
        }
        let solution = solve_assembly_mates_committed(&assembly, &context);
        assert!(solution.converged);
        assert_eq!(solution.statuses[&1], MateSolveStatus::Solved);
        assert_eq!(solution.statuses[&2], MateSolveStatus::Redundant);
    }

    #[test]
    fn interactive_pose_target_is_ephemeral_and_commits_one_manual_preference() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [0.0, 0.0, 7.0], false));
        let mut mate_set = AssemblyMateSet::default();
        mate_set.mates.insert(
            1,
            AssemblyMate {
                id: 1,
                name: "Concentric".into(),
                suppressed: false,
                first: entity(1),
                second: Some(entity(2)),
                kind: AssemblyMateKind::Concentric,
                sense: MateSense::Aligned,
            },
        );
        mate_set.next_mate_id = 2;
        assembly.mates = Some(mate_set);
        let mut context = MateSolveContext::default();
        context.frames.insert(
            1,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        );
        context.assembly_diagonal_mm = 20.0;
        let target = RigidPlacement::new([0.0, 0.0, 12.0], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let solution =
            apply_pose_target_transactionally(&mut assembly, 2, target, &context).unwrap();
        assert!(solution.converged);
        assert_eq!(assembly.occurrences[&2].manual_placement, target);
        assert_eq!(assembly.occurrences[&2].resolved_placement(), target);
        assert!(context.temporary_pose_targets.is_empty());
    }

    #[test]
    fn one_shot_face_target_does_not_persist_a_mate() {
        let mut assembly = AssemblyDocument::new();
        assembly
            .occurrences
            .insert(1, occurrence(1, [0.0; 3], true));
        assembly
            .occurrences
            .insert(2, occurrence(2, [5.0, 0.0, 0.0], false));
        let temporary = AssemblyMate {
            id: 0,
            name: String::new(),
            suppressed: false,
            first: entity(1),
            second: Some(entity(2)),
            kind: AssemblyMateKind::Coincident,
            sense: MateSense::Aligned,
        };
        let context = MateSolveContext {
            assembly_diagonal_mm: 5.0,
            ..MateSolveContext::default()
        };
        let solution = apply_ephemeral_mate_target_transactionally(
            &mut assembly,
            2,
            temporary,
            &context,
            ResolvedMateFrames {
                first: frame(),
                second: Some(frame()),
            },
        )
        .unwrap();
        assert!(solution.converged);
        assert!(assembly.mates.is_none());
        assert!(assembly.occurrences[&2].resolved_placement().translation()[0].abs() < 1.0e-6);
    }

    fn thousand_occurrence_stress_fixture(dense: bool) -> (AssemblyDocument, MateSolveContext) {
        let mut assembly = AssemblyDocument::new();
        let mut mate_set = AssemblyMateSet::default();
        let mut context = MateSolveContext::default();
        for occurrence_id in 1..=1_000u64 {
            assembly.occurrences.insert(
                occurrence_id,
                occurrence(
                    occurrence_id,
                    [occurrence_id as f64 * 1.0e-3, 0.0, 0.0],
                    occurrence_id == 1,
                ),
            );
            context
                .characteristic_lengths_mm
                .insert(occurrence_id, 10.0);
            if occurrence_id > 1 {
                let mate_id = mate_set.next_mate_id;
                mate_set.mates.insert(
                    mate_id,
                    AssemblyMate {
                        id: mate_id,
                        name: format!("Chain {mate_id}"),
                        suppressed: false,
                        first: entity(occurrence_id - 1),
                        second: Some(entity(occurrence_id)),
                        kind: AssemblyMateKind::Coincident,
                        sense: MateSense::Aligned,
                    },
                );
                context.frames.insert(
                    mate_id,
                    ResolvedMateFrames {
                        first: frame(),
                        second: Some(frame()),
                    },
                );
                mate_set.next_mate_id += 1;
            }
        }
        if dense {
            for occurrence_id in (3..=1_000u64).step_by(2) {
                let mate_id = mate_set.next_mate_id;
                mate_set.mates.insert(
                    mate_id,
                    AssemblyMate {
                        id: mate_id,
                        name: format!("Loop {mate_id}"),
                        suppressed: false,
                        first: entity(occurrence_id - 2),
                        second: Some(entity(occurrence_id)),
                        kind: AssemblyMateKind::Coincident,
                        sense: MateSense::Aligned,
                    },
                );
                context.frames.insert(
                    mate_id,
                    ResolvedMateFrames {
                        first: frame(),
                        second: Some(frame()),
                    },
                );
                mate_set.next_mate_id += 1;
            }
        }
        assembly.next_occurrence_id = 1_001;
        assembly.mates = Some(mate_set);
        context.assembly_diagonal_mm = 100.0;
        (assembly, context)
    }

    #[test]
    fn thousand_occurrence_sparse_chain_stays_on_sparse_path() {
        if std::env::var_os("ZEROCAD_RUN_ASSEMBLY_V2_STRESS").is_none() {
            return;
        }
        let (assembly, context) = thousand_occurrence_stress_fixture(false);
        let solved = solve_assembly_mates_committed(&assembly, &context);
        assert!(solved.converged, "{:?}", solved.diagnostics);
        assert_eq!(solved.evidence[0].path, MateSolvePath::SparsePcg);
    }

    #[test]
    fn thousand_occurrence_dense_constraint_graph_is_deterministic() {
        if std::env::var_os("ZEROCAD_RUN_ASSEMBLY_V2_STRESS").is_none() {
            return;
        }
        let (assembly, context) = thousand_occurrence_stress_fixture(true);
        let first = solve_assembly_mates_committed(&assembly, &context);
        let second = solve_assembly_mates_committed(&assembly, &context);
        assert!(first.converged, "{:?}", first.diagnostics);
        assert_eq!(first.placements, second.placements);
        assert_eq!(first.evidence, second.evidence);
        assert_eq!(first.evidence[0].path, MateSolvePath::SparsePcg);
    }
}
