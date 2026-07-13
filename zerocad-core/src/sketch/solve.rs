//! The 2D sketch constraint solver: damped Gauss-Newton (Levenberg-Marquardt)
//! over the shared-point model, plus QR-rank DOF analysis.
//!
//! Design rules (they are what make the solver safe to put in the middle of a
//! parametric pipeline):
//! - **Pure**: solving never mutates the document. The caller decides what to
//!   write back (the GUI persists after an edit; evaluation uses the result
//!   transiently).
//! - **Deterministic**: parameters are laid out in EntityId order, constraints
//!   are assembled in stored order, no randomness, no threads — the same model
//!   solves to bit-identical results every time, which the eval cache and
//!   topological naming both rely on.
//! - **Warm-started**: iteration starts from the stored positions, so an
//!   under-constrained sketch moves minimally (the LM damping regularizes the
//!   null space toward "stay where you are") — exactly the drag feel you want.
//! - **Fail-soft**: a non-converged or conflicting solve reports its outcome;
//!   the caller bakes the last-valid stored positions instead (never blank
//!   geometry).

use super::constraints::{Constraint, EntityId, SketchEntity, SketchSolverModel};
use super::linalg::{cholesky_solve, qr_rank, Mat};
use std::collections::HashMap;

const MAX_ITERATIONS: usize = 50;
/// Convergence: every residual row below this (mm / mm² scale).
const RESIDUAL_TOL: f64 = 1.0e-9;
/// Gradient stagnation: ‖Jᵀr‖∞ below this with a large residual means the
/// solver is at a local minimum of an inconsistent system → Conflicting.
const GRADIENT_TOL: f64 = 1.0e-10;
/// Relative singular-value cutoff for the DOF rank.
const RANK_TOL: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveOutcome {
    /// All constraints satisfied to tolerance.
    Converged,
    /// Iteration budget exhausted without satisfying the constraints.
    DidNotConverge,
    /// The constraint set is inconsistent (over-constrained with conflicting
    /// values) — the solver reached a minimum that still violates it.
    Conflicting,
}

#[derive(Debug, Clone)]
pub struct SolveReport {
    pub outcome: SolveOutcome,
    /// Solved point positions, id → position (id order).
    pub positions: Vec<(EntityId, (f64, f64))>,
    /// Solved radii for circle/arc entities, id → radius (id order).
    pub radii: Vec<(EntityId, f64)>,
    /// Remaining degrees of freedom (`n_params − rank(J)`), from the solved
    /// configuration. A fully constrained sketch reports 0.
    pub dof: usize,
    /// When `Conflicting`, the constraint whose removal makes the rest
    /// satisfiable (found by leave-one-out), if one exists.
    pub conflicting: Option<EntityId>,
    /// Max residual magnitude at exit (diagnostics).
    pub residual: f64,
}

/// Solve `model` against the document variables. Pure — see module docs.
pub fn solve_model(model: &SketchSolverModel, vars: &HashMap<String, f64>) -> SolveReport {
    let mut system = System::build(model, vars);
    let outcome = system.run_lm();
    let residual = system.max_residual();
    let rank = if system.params.is_empty() {
        0
    } else {
        qr_rank(&system.jacobian(), RANK_TOL)
    };
    let dof = system.params.len().saturating_sub(rank);

    let conflicting = if outcome == SolveOutcome::Conflicting {
        find_conflicting_constraint(model, vars)
    } else {
        None
    };

    SolveReport {
        outcome,
        positions: system.positions(),
        radii: system.radii(),
        dof,
        conflicting,
        residual,
    }
}

/// Leave-one-out: the first constraint whose removal lets the rest converge.
fn find_conflicting_constraint(
    model: &SketchSolverModel,
    vars: &HashMap<String, f64>,
) -> Option<EntityId> {
    for skip in 0..model.constraints.len() {
        let mut reduced = model.clone();
        let removed = reduced.constraints.remove(skip);
        let mut system = System::build(&reduced, vars);
        if system.run_lm() == SolveOutcome::Converged {
            return Some(removed.id());
        }
    }
    None
}

/// One scalar unknown: a point coordinate or an entity radius.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Param {
    PointX(EntityId),
    PointY(EntityId),
    Radius(EntityId),
}

struct System {
    params: Vec<Param>,
    /// Current values, parallel to `params`.
    x: Vec<f64>,
    /// Index of each param for Jacobian assembly.
    index: HashMap<Param, usize>,
    rows: Vec<ResidualRow>,
}

/// One residual row: closed-form value + sparse analytic gradient.
struct ResidualRow {
    value_and_grad: Box<dyn Fn(&System) -> (f64, Vec<(usize, f64)>)>,
}

impl System {
    fn build(model: &SketchSolverModel, vars: &HashMap<String, f64>) -> System {
        // Parameter layout in EntityId order — deterministic across runs.
        let mut point_ids: Vec<EntityId> = model.points.iter().map(|p| p.id).collect();
        point_ids.sort();
        let mut radius_ids: Vec<EntityId> = model
            .entities
            .iter()
            .filter(|e| matches!(e, SketchEntity::Circle { .. } | SketchEntity::Arc { .. }))
            .map(|e| e.id())
            .collect();
        radius_ids.sort();

        let mut params = Vec::new();
        let mut x = Vec::new();
        for &id in &point_ids {
            let p = model.point(id).expect("point ids come from the model");
            params.push(Param::PointX(id));
            x.push(p.pos.0);
            params.push(Param::PointY(id));
            x.push(p.pos.1);
        }
        for &id in &radius_ids {
            let r = model
                .entities
                .iter()
                .find_map(|e| match e {
                    SketchEntity::Circle {
                        id: eid, radius, ..
                    }
                    | SketchEntity::Arc {
                        id: eid, radius, ..
                    } if *eid == id => Some(*radius),
                    _ => None,
                })
                .expect("radius ids come from the model");
            params.push(Param::Radius(id));
            x.push(r);
        }
        let index: HashMap<Param, usize> =
            params.iter().enumerate().map(|(i, &p)| (p, i)).collect();

        let mut system = System {
            params,
            x,
            index,
            rows: Vec::new(),
        };
        system.assemble_rows(model, vars);
        system
    }

    fn px(&self, id: EntityId) -> Option<usize> {
        self.index.get(&Param::PointX(id)).copied()
    }
    fn py(&self, id: EntityId) -> Option<usize> {
        self.index.get(&Param::PointY(id)).copied()
    }
    fn pr(&self, id: EntityId) -> Option<usize> {
        self.index.get(&Param::Radius(id)).copied()
    }

    /// Line endpoints (p0, p1) as param indices.
    fn line_params(
        &self,
        model: &SketchSolverModel,
        line: EntityId,
    ) -> Option<(usize, usize, usize, usize)> {
        model.entities.iter().find_map(|e| match e {
            SketchEntity::Line { id, p0, p1, .. } if *id == line => {
                Some((self.px(*p0)?, self.py(*p0)?, self.px(*p1)?, self.py(*p1)?))
            }
            _ => None,
        })
    }

    fn assemble_rows(&mut self, model: &SketchSolverModel, vars: &HashMap<String, f64>) {
        // Constraints assemble in stored order; unresolvable references (a
        // deleted entity) contribute no row — degraded, never wrong.
        for constraint in &model.constraints {
            match constraint {
                Constraint::Coincident { a, b, .. } => {
                    if let (Some(ax), Some(ay), Some(bx), Some(by)) =
                        (self.px(*a), self.py(*a), self.px(*b), self.py(*b))
                    {
                        self.push_diff(ax, bx);
                        self.push_diff(ay, by);
                    }
                }
                Constraint::Horizontal { line, .. } => {
                    if let Some((_, y0, _, y1)) = self.line_params(model, *line) {
                        self.push_diff(y1, y0);
                    }
                }
                Constraint::Vertical { line, .. } => {
                    if let Some((x0, _, x1, _)) = self.line_params(model, *line) {
                        self.push_diff(x1, x0);
                    }
                }
                Constraint::Distance { a, b, d, .. } => {
                    if let (Some(ax), Some(ay), Some(bx), Some(by)) =
                        (self.px(*a), self.py(*a), self.px(*b), self.py(*b))
                    {
                        let target = d.resolve(vars) as f64;
                        self.rows.push(ResidualRow {
                            value_and_grad: Box::new(move |s| {
                                let dx = s.x[bx] - s.x[ax];
                                let dy = s.x[by] - s.x[ay];
                                let f = dx * dx + dy * dy - target * target;
                                (
                                    f,
                                    vec![
                                        (ax, -2.0 * dx),
                                        (ay, -2.0 * dy),
                                        (bx, 2.0 * dx),
                                        (by, 2.0 * dy),
                                    ],
                                )
                            }),
                        });
                    }
                }
                Constraint::Radius { circle, r, .. } => {
                    if let Some(ri) = self.pr(*circle) {
                        let target = r.resolve(vars) as f64;
                        self.rows.push(ResidualRow {
                            value_and_grad: Box::new(move |s| (s.x[ri] - target, vec![(ri, 1.0)])),
                        });
                    }
                }
                Constraint::Parallel { a, b, .. } => {
                    if let (Some(pa), Some(pb)) =
                        (self.line_params(model, *a), self.line_params(model, *b))
                    {
                        self.push_cross_or_dot(pa, pb, true);
                    }
                }
                Constraint::Perpendicular { a, b, .. } => {
                    if let (Some(pa), Some(pb)) =
                        (self.line_params(model, *a), self.line_params(model, *b))
                    {
                        self.push_cross_or_dot(pa, pb, false);
                    }
                }
                Constraint::Tangent { line, circle, .. } => {
                    let center = model.entities.iter().find_map(|e| match e {
                        SketchEntity::Circle { id, center, .. }
                        | SketchEntity::Arc { id, center, .. }
                            if id == circle =>
                        {
                            Some(*center)
                        }
                        _ => None,
                    });
                    if let (Some((x0, y0, x1, y1)), Some(center), Some(ri)) =
                        (self.line_params(model, *line), center, self.pr(*circle))
                    {
                        if let (Some(cx), Some(cy)) = (self.px(center), self.py(center)) {
                            // f = cross(u, w)² − r²·‖u‖², u = p1−p0, w = c−p0.
                            self.rows.push(ResidualRow {
                                value_and_grad: Box::new(move |s| {
                                    let ux = s.x[x1] - s.x[x0];
                                    let uy = s.x[y1] - s.x[y0];
                                    let wx = s.x[cx] - s.x[x0];
                                    let wy = s.x[cy] - s.x[y0];
                                    let r = s.x[ri];
                                    let k = ux * wy - uy * wx;
                                    let u2 = ux * ux + uy * uy;
                                    let f = k * k - r * r * u2;
                                    (
                                        f,
                                        vec![
                                            (x0, 2.0 * k * (uy - wy) + 2.0 * r * r * ux),
                                            (y0, 2.0 * k * (wx - ux) + 2.0 * r * r * uy),
                                            (x1, 2.0 * k * wy - 2.0 * r * r * ux),
                                            (y1, -2.0 * k * wx - 2.0 * r * r * uy),
                                            (cx, -2.0 * k * uy),
                                            (cy, 2.0 * k * ux),
                                            (ri, -2.0 * r * u2),
                                        ],
                                    )
                                }),
                            });
                        }
                    }
                }
                Constraint::Equal { a, b, .. } => {
                    match (self.pr(*a), self.pr(*b)) {
                        (Some(ra), Some(rb)) => {
                            self.push_diff(ra, rb);
                        }
                        _ => {
                            if let (Some(pa), Some(pb)) =
                                (self.line_params(model, *a), self.line_params(model, *b))
                            {
                                // ‖ua‖² − ‖ub‖².
                                let (ax0, ay0, ax1, ay1) = pa;
                                let (bx0, by0, bx1, by1) = pb;
                                self.rows.push(ResidualRow {
                                    value_and_grad: Box::new(move |s| {
                                        let uax = s.x[ax1] - s.x[ax0];
                                        let uay = s.x[ay1] - s.x[ay0];
                                        let ubx = s.x[bx1] - s.x[bx0];
                                        let uby = s.x[by1] - s.x[by0];
                                        let f = uax * uax + uay * uay - (ubx * ubx + uby * uby);
                                        (
                                            f,
                                            vec![
                                                (ax0, -2.0 * uax),
                                                (ay0, -2.0 * uay),
                                                (ax1, 2.0 * uax),
                                                (ay1, 2.0 * uay),
                                                (bx0, 2.0 * ubx),
                                                (by0, 2.0 * uby),
                                                (bx1, -2.0 * ubx),
                                                (by1, -2.0 * uby),
                                            ],
                                        )
                                    }),
                                });
                            }
                        }
                    }
                }
                Constraint::Fixed { p, .. } => {
                    if let (Some(xi), Some(yi)) = (self.px(*p), self.py(*p)) {
                        // Anchor at the position at solve START (warm-start value).
                        let x0 = self.x[xi];
                        let y0 = self.x[yi];
                        self.rows.push(ResidualRow {
                            value_and_grad: Box::new(move |s| (s.x[xi] - x0, vec![(xi, 1.0)])),
                        });
                        self.rows.push(ResidualRow {
                            value_and_grad: Box::new(move |s| (s.x[yi] - y0, vec![(yi, 1.0)])),
                        });
                    }
                }
            }
        }
    }

    /// Row `x[i] − x[j]`.
    fn push_diff(&mut self, i: usize, j: usize) {
        self.rows.push(ResidualRow {
            value_and_grad: Box::new(move |s| (s.x[i] - s.x[j], vec![(i, 1.0), (j, -1.0)])),
        });
    }

    /// Row `cross(ua, ub)` (parallel) or `dot(ua, ub)` (perpendicular).
    fn push_cross_or_dot(
        &mut self,
        (ax0, ay0, ax1, ay1): (usize, usize, usize, usize),
        (bx0, by0, bx1, by1): (usize, usize, usize, usize),
        cross: bool,
    ) {
        self.rows.push(ResidualRow {
            value_and_grad: Box::new(move |s| {
                let uax = s.x[ax1] - s.x[ax0];
                let uay = s.x[ay1] - s.x[ay0];
                let ubx = s.x[bx1] - s.x[bx0];
                let uby = s.x[by1] - s.x[by0];
                if cross {
                    let f = uax * uby - uay * ubx;
                    (
                        f,
                        vec![
                            (ax0, -uby),
                            (ay0, ubx),
                            (ax1, uby),
                            (ay1, -ubx),
                            (bx0, uay),
                            (by0, -uax),
                            (bx1, -uay),
                            (by1, uax),
                        ],
                    )
                } else {
                    let f = uax * ubx + uay * uby;
                    (
                        f,
                        vec![
                            (ax0, -ubx),
                            (ay0, -uby),
                            (ax1, ubx),
                            (ay1, uby),
                            (bx0, -uax),
                            (by0, -uay),
                            (bx1, uax),
                            (by1, uay),
                        ],
                    )
                }
            }),
        });
    }

    fn residuals(&self) -> Vec<f64> {
        self.rows
            .iter()
            .map(|r| (r.value_and_grad)(self).0)
            .collect()
    }

    fn max_residual(&self) -> f64 {
        self.residuals().iter().fold(0.0f64, |m, r| m.max(r.abs()))
    }

    fn jacobian(&self) -> Mat {
        let mut j = Mat::zeros(self.rows.len(), self.params.len());
        for (ri, row) in self.rows.iter().enumerate() {
            let (_, grad) = (row.value_and_grad)(self);
            for (ci, g) in grad {
                j.add_at(ri, ci, g);
            }
        }
        j
    }

    /// Levenberg-Marquardt: `(JᵀJ + λI)·δ = −Jᵀr`, λ adapting to progress.
    fn run_lm(&mut self) -> SolveOutcome {
        if self.rows.is_empty() || self.params.is_empty() {
            return SolveOutcome::Converged;
        }
        let mut lambda = 1.0e-6;
        let mut cost = self.residuals().iter().map(|r| r * r).sum::<f64>();
        for _ in 0..MAX_ITERATIONS {
            if self.max_residual() < RESIDUAL_TOL {
                return SolveOutcome::Converged;
            }
            let j = self.jacobian();
            let r = self.residuals();
            let grad = j.atv(&r);
            let grad_inf = grad.iter().fold(0.0f64, |m, g| m.max(g.abs()));
            if grad_inf < GRADIENT_TOL {
                // Stationary but unsatisfied: inconsistent constraint set.
                return SolveOutcome::Conflicting;
            }
            let mut jtj = j.ata();
            let neg_grad: Vec<f64> = grad.iter().map(|g| -g).collect();

            // Inner loop: raise damping until a step reduces the cost.
            let mut stepped = false;
            for _ in 0..16 {
                let mut damped = jtj.clone();
                for i in 0..damped.rows {
                    damped.add_at(i, i, lambda);
                }
                if let Some(delta) = cholesky_solve(&damped, &neg_grad) {
                    let saved = self.x.clone();
                    for (xi, d) in self.x.iter_mut().zip(&delta) {
                        *xi += d;
                    }
                    let new_cost = self.residuals().iter().map(|r| r * r).sum::<f64>();
                    if new_cost.is_finite() && new_cost < cost {
                        cost = new_cost;
                        lambda = (lambda * 0.5).max(1.0e-12);
                        stepped = true;
                        break;
                    }
                    self.x = saved;
                }
                lambda *= 8.0;
                if lambda > 1.0e12 {
                    break;
                }
            }
            if !stepped {
                // No downhill step exists: either done or inconsistent.
                return if self.max_residual() < RESIDUAL_TOL {
                    SolveOutcome::Converged
                } else {
                    SolveOutcome::Conflicting
                };
            }
            // Reuse jtj to appease the borrow of clarity; nothing else.
            let _ = &mut jtj;
        }
        if self.max_residual() < RESIDUAL_TOL {
            SolveOutcome::Converged
        } else {
            SolveOutcome::DidNotConverge
        }
    }

    fn positions(&self) -> Vec<(EntityId, (f64, f64))> {
        let mut out: Vec<(EntityId, (f64, f64))> = Vec::new();
        for (i, p) in self.params.iter().enumerate() {
            if let Param::PointX(id) = p {
                let y = self.x[i + 1]; // PointY is always laid out right after.
                out.push((*id, (self.x[i], y)));
            }
        }
        out
    }

    fn radii(&self) -> Vec<(EntityId, f64)> {
        self.params
            .iter()
            .enumerate()
            .filter_map(|(i, p)| match p {
                Param::Radius(id) => Some((*id, self.x[i])),
                _ => None,
            })
            .collect()
    }
}

/// Apply a solve result to a model (the write-back the GUI performs after an
/// edit-time solve).
pub fn apply_solution(model: &mut SketchSolverModel, report: &SolveReport) {
    for (id, pos) in &report.positions {
        if let Some(p) = model.points.iter_mut().find(|p| p.id == *id) {
            p.pos = *pos;
        }
    }
    for (id, r) in &report.radii {
        for e in model.entities.iter_mut() {
            match e {
                SketchEntity::Circle {
                    id: eid, radius, ..
                }
                | SketchEntity::Arc {
                    id: eid, radius, ..
                } if eid == id => {
                    *radius = *r;
                }
                _ => {}
            }
        }
    }
}

/// True when any dimensional constraint is expression-bound: its target can
/// change with the document variables, so evaluation must re-solve instead of
/// trusting the stored positions.
pub fn has_variable_bound_constraint(model: &SketchSolverModel) -> bool {
    model.constraints.iter().any(|c| match c {
        Constraint::Distance { d, .. } => d.expr.is_some(),
        Constraint::Radius { r, .. } => r.expr.is_some(),
        _ => false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::constraints::{promote_shapes_to_entities, SketchPoint};
    use crate::sketch::{Dimension, SketchShape};

    fn rect_model(w_expr: Option<&str>) -> (SketchSolverModel, HashMap<String, f64>) {
        let mut vars = HashMap::new();
        vars.insert("w".to_string(), 20.0);
        let shapes = vec![SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension {
                value: 20.0,
                expr: w_expr.map(str::to_string),
            },
            h: Dimension::literal(12.0),
            from_center: false,
        }];
        let ids = EntityId::sequence(1);
        let (model, _) = promote_shapes_to_entities(&shapes, &ids, &vars, 1);
        (model, vars)
    }

    #[test]
    fn solved_rectangle_holds_its_dimensions() {
        let (model, vars) = rect_model(Some("w"));
        let report = solve_model(&model, &vars);
        assert_eq!(report.outcome, SolveOutcome::Converged, "{report:?}");
        // Width between p0 and p1 is exactly w.
        let p: HashMap<EntityId, (f64, f64)> = report.positions.iter().copied().collect();
        let ids: Vec<EntityId> = report.positions.iter().map(|(id, _)| *id).collect();
        let d = |a: usize, b: usize| {
            let (ax, ay) = p[&ids[a]];
            let (bx, by) = p[&ids[b]];
            ((bx - ax).powi(2) + (by - ay).powi(2)).sqrt()
        };
        assert!((d(0, 1) - 20.0).abs() < 1e-6, "width {}", d(0, 1));
        assert!((d(1, 2) - 12.0).abs() < 1e-6, "height {}", d(1, 2));
    }

    #[test]
    fn variable_edit_drives_the_solve() {
        let (mut model, mut vars) = rect_model(Some("w"));
        assert!(has_variable_bound_constraint(&model));
        vars.insert("w".to_string(), 30.0);
        let report = solve_model(&model, &vars);
        assert_eq!(report.outcome, SolveOutcome::Converged);
        apply_solution(&mut model, &report);
        let xs: Vec<f64> = model.points.iter().map(|p| p.pos.0).collect();
        let width = xs.iter().cloned().fold(f64::MIN, f64::max)
            - xs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((width - 30.0).abs() < 1e-6, "width followed w: {width}");
    }

    #[test]
    fn solve_is_bit_stable_across_repeats() {
        let (model, vars) = rect_model(Some("w"));
        let a = solve_model(&model, &vars);
        let b = solve_model(&model, &vars);
        assert_eq!(a.positions, b.positions, "deterministic warm-started solve");
        assert_eq!(a.radii, b.radii);
    }

    #[test]
    fn dof_counts_are_pinned_for_canonical_sketches() {
        // A promoted rectangle is FULLY constrained: H/V + w/h dims + the
        // origin anchor (legacy shapes grow from their origin, so promotion
        // anchors it).
        let (model, vars) = rect_model(None);
        let report = solve_model(&model, &vars);
        assert_eq!(report.outcome, SolveOutcome::Converged);
        assert_eq!(report.dof, 0, "promoted rectangle is fully constrained");

        // Delete the anchor: the rectangle floats in translation only (H/V
        // kill rotation, the dims kill stretch) → 2 DOF.
        let mut floating = model.clone();
        floating
            .constraints
            .retain(|c| !matches!(c, Constraint::Fixed { .. }));
        let report = solve_model(&floating, &vars);
        assert_eq!(report.outcome, SolveOutcome::Converged);
        assert_eq!(report.dof, 2, "unanchored rectangle floats in translation");
    }

    #[test]
    fn conflicting_constraints_are_detected_and_attributed() {
        // Two points, coincident AND 10mm apart — inconsistent.
        let mut model = SketchSolverModel::default();
        model.points.push(SketchPoint {
            id: EntityId(0),
            pos: (0.0, 0.0),
        });
        model.points.push(SketchPoint {
            id: EntityId(1),
            pos: (5.0, 0.0),
        });
        model.constraints.push(Constraint::Coincident {
            id: EntityId(2),
            a: EntityId(0),
            b: EntityId(1),
        });
        model.constraints.push(Constraint::Distance {
            id: EntityId(3),
            a: EntityId(0),
            b: EntityId(1),
            d: Dimension::literal(10.0),
        });
        let report = solve_model(&model, &HashMap::new());
        assert_eq!(report.outcome, SolveOutcome::Conflicting, "{report:?}");
        assert!(
            report.conflicting.is_some(),
            "leave-one-out should attribute a culprit"
        );
    }

    #[test]
    fn tangent_line_touches_the_circle() {
        // Horizontal line y≈4.9 near a circle r=5 at origin: tangency solves to
        // distance(center, line) == r.
        let mut model = SketchSolverModel::default();
        model.points.push(SketchPoint {
            id: EntityId(0),
            pos: (-10.0, 4.9),
        });
        model.points.push(SketchPoint {
            id: EntityId(1),
            pos: (10.0, 4.9),
        });
        model.points.push(SketchPoint {
            id: EntityId(2),
            pos: (0.0, 0.0),
        });
        model.entities.push(SketchEntity::Line {
            id: EntityId(3),
            p0: EntityId(0),
            p1: EntityId(1),
            derived_from: None,
        });
        model.entities.push(SketchEntity::Circle {
            id: EntityId(4),
            center: EntityId(2),
            radius: 5.0,
            derived_from: None,
        });
        model.constraints.push(Constraint::Fixed {
            id: EntityId(5),
            p: EntityId(2),
        });
        model.constraints.push(Constraint::Radius {
            id: EntityId(6),
            circle: EntityId(4),
            r: Dimension::literal(5.0),
        });
        model.constraints.push(Constraint::Tangent {
            id: EntityId(7),
            line: EntityId(3),
            circle: EntityId(4),
        });
        let report = solve_model(&model, &HashMap::new());
        assert_eq!(report.outcome, SolveOutcome::Converged, "{report:?}");
        let p: HashMap<EntityId, (f64, f64)> = report.positions.iter().copied().collect();
        let (x0, y0) = p[&EntityId(0)];
        let (x1, y1) = p[&EntityId(1)];
        let (cx, cy) = p[&EntityId(2)];
        // Perpendicular distance from center to the line == 5.
        let (ux, uy) = (x1 - x0, y1 - y0);
        let dist = ((ux * (cy - y0) - uy * (cx - x0)) / (ux * ux + uy * uy).sqrt()).abs();
        assert!((dist - 5.0).abs() < 1e-6, "tangent distance {dist}");
    }
}
