//! 2D sketch curves and planar region (face) detection.
//!
//! A sketch holds a collection of straight line segments and full circles
//! (drawn in 2D plane coordinates). When the sketch is finalized we compute
//! the *planar subdivision* of those curves and return the bounded faces
//! ("regions"). Each region is a closed CCW polygon the user can pick and
//! extrude — this is what makes intersecting shapes in Fusion 360 split
//! into independently selectable pieces.
//!
//! The algorithm is the standard half-edge / DCEL face traversal:
//!
//! 1. Discretize circles to polylines.
//! 2. Compute all pairwise segment intersections; snap-deduplicate vertices.
//! 3. Split each input segment at every intersection that lies on it.
//! 4. For each split sub-segment, emit two directed half-edges (twins).
//! 5. At each vertex, sort outgoing half-edges by polar angle.
//! 6. For half-edge `h` with destination `v` and twin at sorted index `i`,
//!    set `next(h)` to the outgoing half-edge at index `(i - 1) mod k` —
//!    the one immediately clockwise of `twin(h)`. Walking `h → next(h) → …`
//!    traces the face on the LEFT of `h`.
//! 7. Walk all half-edge cycles. Cycles with positive signed area are
//!    bounded faces; the one negative-area cycle is the outer face — drop it.
//!
//! For Phase 2a this only handles line segments and circles (no arcs yet).
//! Collinear/overlapping input segments are not split against each other —
//! sketch tools currently can't produce them in practice. Adding that and
//! arc support are tracked as follow-ups in the plan.

use std::collections::HashMap;

/// Sketch entity identity + the constraint-solver data model (shared points,
/// entities, constraints). Lives in its own file; re-exported here so callers
/// keep one `crate::sketch::` namespace.
pub mod constraints;
pub mod linalg;
pub mod offset;
pub mod pattern;
pub mod projection;
pub mod solve;
pub mod trim;
pub use constraints::{
    bake_construction_curves, effective_shape_ids, Constraint, EntityId, ProjectedEdgeReference,
    SketchEntity, SketchPoint, SketchSolverModel,
};
pub use offset::{
    apply_associative_offsets, evaluate_associative_offset, SketchOffsetError,
    SketchOffsetOperation,
};
pub use pattern::{
    apply_associative_patterns, dissolve_associative_pattern, evaluate_associative_pattern,
    SketchPatternError, SketchPatternEvaluation, SketchPatternKind, SketchPatternOperation,
    SketchPatternSpanId,
};
pub use projection::{append_projected_edge, rebuild_projected_edge};
pub use solve::{solve_model, SolveOutcome, SolveReport};
pub use trim::{
    apply_trim_preview, preview_trim, TrimConstraintReport, TrimError, TrimOutcome, TrimPreview,
};

/// Vertex coordinate snap tolerance (in sketch plane units / mm).
const VERTEX_TOL: f64 = 1e-3;
/// Cross-product epsilon for "parallel" classification.
const PARALLEL_EPS: f64 = 1e-9;
/// Discretization for circles when computing planar arrangement. Shared crate
/// constant so sketch arrangement and the kernel's cylinder solids agree.
use crate::CIRCLE_SEGS;
/// Facet count for an ellipse drawn via [`SketchCurves::add_ellipse`] — matches
/// the circle discretization so the two read consistently.
const ELLIPSE_SEGS: usize = CIRCLE_SEGS;

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LineSegment {
    pub a: (f32, f32),
    pub b: (f32, f32),
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Circle {
    pub center: (f32, f32),
    pub radius: f32,
}

/// A circular **arc** boundary fragment (currently produced by a sketch corner
/// fillet). Stored analytically — center + radius + the two endpoints — so the
/// extrude wire builder can sweep it to an exact cylindrical wall instead of
/// relying on [`crate::mock_kernel`]'s sample-based arc refit (which can't segment
/// the tangent-connected arcs of a rounded rectangle). The arc runs the short way
/// from `start` to `end` about `center`; `clockwise` disambiguates an exact
/// semicircle (and other explicitly clockwise spans). Region detection tessellates it via
/// [`SketchCurves`]'s flattening, so the DCEL still sees only line segments.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Arc {
    pub center: (f32, f32),
    pub radius: f32,
    pub start: (f32, f32),
    pub end: (f32, f32),
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub clockwise: bool,
}

impl Arc {
    fn parameter_bounds(&self) -> (f64, f64) {
        let angle = |point: (f32, f32)| {
            f64::from(point.1 - self.center.1).atan2(f64::from(point.0 - self.center.0))
        };
        let first = angle(self.start);
        let mut last = angle(self.end);
        while last - first > std::f64::consts::PI {
            last -= std::f64::consts::TAU;
        }
        while last - first < -std::f64::consts::PI {
            last += std::f64::consts::TAU;
        }
        if self.clockwise && last > first {
            last -= std::f64::consts::TAU;
        }
        (first, last)
    }
}

/// How a spline's stored points define the curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SplineKind {
    /// The stored points are B-spline/NURBS control points. The curve generally
    /// does not pass through interior points.
    ControlPoint,
    /// The stored points are interpolation points through which the curve must
    /// pass.
    FitPoint,
}

/// Continuity requested between fit-point spline spans. Control-point splines
/// derive their continuity from degree and knot multiplicity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum SplineContinuity {
    /// Connected spans only (C0).
    Position,
    /// Shared tangent direction (C1).
    Tangent,
    /// Shared first and second derivatives (C2).
    #[default]
    Curvature,
}

/// A durable editable sketch spline.
///
/// DXF NURBS data can be stored without throwing away degree, knots, or
/// weights. Native fit-point curves use `points` as interpolation handles and
/// the selected [`SplineContinuity`]. `trim` is a normalized inclusive
/// parameter interval; untrimmed splines use the full `[0, 1]` domain.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Spline {
    pub kind: SplineKind,
    pub points: Vec<(f32, f32)>,
    #[serde(default = "default_spline_degree")]
    pub degree: u8,
    #[serde(default)]
    pub knots: Vec<f32>,
    #[serde(default)]
    pub weights: Vec<f32>,
    #[serde(default)]
    pub closed: bool,
    #[serde(default)]
    pub periodic: bool,
    #[serde(default)]
    pub continuity: SplineContinuity,
    #[serde(default)]
    pub trim: Option<(f32, f32)>,
}

const fn default_spline_degree() -> u8 {
    3
}

impl Spline {
    pub fn control_points(points: Vec<(f32, f32)>, degree: u8, closed: bool) -> Self {
        Self {
            kind: SplineKind::ControlPoint,
            points,
            degree,
            knots: Vec::new(),
            weights: Vec::new(),
            closed,
            periodic: false,
            continuity: SplineContinuity::Curvature,
            trim: None,
        }
    }

    pub fn fit_points(points: Vec<(f32, f32)>, closed: bool, continuity: SplineContinuity) -> Self {
        Self {
            kind: SplineKind::FitPoint,
            points,
            degree: 3,
            knots: Vec::new(),
            weights: Vec::new(),
            closed,
            periodic: closed,
            continuity,
            trim: None,
        }
    }

    pub fn is_valid(&self) -> bool {
        let min_points = match self.kind {
            SplineKind::ControlPoint => self.degree.max(1) as usize + 1,
            SplineKind::FitPoint => 2,
        };
        self.points.len() >= min_points
            && self
                .points
                .iter()
                .all(|(x, y)| x.is_finite() && y.is_finite())
            && self.weights.iter().all(|w| w.is_finite() && *w > 0.0)
            && self.knots.iter().all(|k| k.is_finite())
            && self.knots.windows(2).all(|window| window[0] <= window[1])
            && self.trim.is_none_or(|(a, b)| {
                a.is_finite() && b.is_finite() && a >= 0.0 && b <= 1.0 && b > a
            })
    }

    /// Deterministically sample this spline for sketch display, planar-region
    /// construction, and legacy polyline consumers. The budget scales with
    /// control-polygon length and is bounded so malformed imports cannot cause
    /// unbounded work.
    pub fn sampled_points(&self, tolerance: f32) -> Vec<(f32, f32)> {
        if !self.is_valid() {
            return Vec::new();
        }
        let polygon_length: f32 = self
            .points
            .windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum();
        let tolerance = tolerance.max(1.0e-4);
        let spans = ((polygon_length / tolerance).sqrt().ceil() as usize)
            .max(self.points.len().saturating_mul(8))
            .clamp(16, 512);
        let (start, end) = self.trim.unwrap_or((0.0, 1.0));
        let mut sampled = Vec::with_capacity(spans + 1);
        for index in 0..=spans {
            let fraction = index as f32 / spans as f32;
            let t = start + (end - start) * fraction;
            if let Some(point) = self.evaluate(t) {
                if sampled.last().is_none_or(|previous: &(f32, f32)| {
                    (previous.0 - point.0).hypot(previous.1 - point.1) > 1.0e-6
                }) {
                    sampled.push(point);
                }
            }
        }
        if self.closed && self.trim.is_none() && sampled.len() > 2 {
            let first = sampled[0];
            if sampled
                .last()
                .is_some_and(|last| (last.0 - first.0).hypot(last.1 - first.1) > 1.0e-5)
            {
                sampled.push(first);
            }
        }
        sampled
    }

    /// Point on the spline at normalized parameter `t` in [0, 1], if the
    /// control data is well-formed. Public for lightweight display sampling
    /// (e.g. the sketch-text preview).
    pub fn evaluate(&self, t: f32) -> Option<(f32, f32)> {
        match self.kind {
            SplineKind::ControlPoint => self.evaluate_control_point(t),
            SplineKind::FitPoint => self.evaluate_fit_point(t),
        }
    }

    fn evaluate_control_point(&self, t: f32) -> Option<(f32, f32)> {
        if t <= 0.0 {
            return self.points.first().copied();
        }
        if t >= 1.0 {
            return if self.closed {
                self.points.first().copied()
            } else {
                self.points.last().copied()
            };
        }
        let mut points = self.points.clone();
        let degree = usize::from(self.degree.max(1));
        if self.closed && self.knots.is_empty() {
            for point in self.points.iter().take(degree) {
                points.push(*point);
            }
        }
        if points.len() <= degree {
            return None;
        }
        let expected_knots = points.len() + degree + 1;
        let knots = if self.knots.len() == expected_knots {
            self.knots.clone()
        } else {
            clamped_uniform_knots(points.len(), degree)
        };
        let weights = if self.weights.len() == self.points.len() {
            let mut weights = self.weights.clone();
            if points.len() > weights.len() {
                weights.extend(self.weights.iter().copied().take(degree));
            }
            weights
        } else {
            vec![1.0; points.len()]
        };
        let domain_start = knots[degree];
        let domain_end = knots[points.len()];
        if domain_end <= domain_start {
            return None;
        }
        let u = domain_start + t.clamp(0.0, 1.0) * (domain_end - domain_start);
        let mut basis = vec![0.0_f32; points.len()];
        for index in 0..points.len() {
            let at_end = t >= 1.0 && index + 1 == points.len();
            if (knots[index] <= u && u < knots[index + 1]) || at_end {
                basis[index] = 1.0;
            }
        }
        for order in 1..=degree {
            let previous = basis.clone();
            for index in 0..points.len() {
                let left_denominator = knots[index + order] - knots[index];
                let left = if left_denominator.abs() > f32::EPSILON {
                    (u - knots[index]) / left_denominator * previous[index]
                } else {
                    0.0
                };
                let right = if index + 1 < points.len() {
                    let right_denominator = knots[index + order + 1] - knots[index + 1];
                    if right_denominator.abs() > f32::EPSILON {
                        (knots[index + order + 1] - u) / right_denominator * previous[index + 1]
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };
                basis[index] = left + right;
            }
        }
        let mut numerator = (0.0_f32, 0.0_f32);
        let mut denominator = 0.0_f32;
        for ((point, weight), basis) in points.iter().zip(weights).zip(basis) {
            let factor = weight * basis;
            numerator.0 += point.0 * factor;
            numerator.1 += point.1 * factor;
            denominator += factor;
        }
        (denominator.abs() > f32::EPSILON)
            .then_some((numerator.0 / denominator, numerator.1 / denominator))
    }

    fn evaluate_fit_point(&self, t: f32) -> Option<(f32, f32)> {
        match self.continuity {
            SplineContinuity::Position => evaluate_linear(&self.points, t, self.closed),
            SplineContinuity::Tangent => evaluate_catmull_rom(&self.points, t, self.closed),
            SplineContinuity::Curvature if !self.closed => evaluate_natural_cubic(&self.points, t),
            SplineContinuity::Curvature => evaluate_catmull_rom(&self.points, t, true),
        }
    }
}

fn clamped_uniform_knots(point_count: usize, degree: usize) -> Vec<f32> {
    let knot_count = point_count + degree + 1;
    let interior_count = point_count.saturating_sub(degree + 1);
    let mut knots = Vec::with_capacity(knot_count);
    knots.extend(std::iter::repeat_n(0.0, degree + 1));
    for index in 1..=interior_count {
        knots.push(index as f32 / (interior_count + 1) as f32);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

fn evaluate_linear(points: &[(f32, f32)], t: f32, closed: bool) -> Option<(f32, f32)> {
    let span_count = if closed {
        points.len()
    } else {
        points.len().saturating_sub(1)
    };
    if span_count == 0 {
        return None;
    }
    let scaled = t.clamp(0.0, 1.0) * span_count as f32;
    let span = (scaled.floor() as usize).min(span_count - 1);
    let local = if t >= 1.0 { 1.0 } else { scaled.fract() };
    let a = points[span];
    let b = points[(span + 1) % points.len()];
    Some((a.0 + (b.0 - a.0) * local, a.1 + (b.1 - a.1) * local))
}

fn evaluate_catmull_rom(points: &[(f32, f32)], t: f32, closed: bool) -> Option<(f32, f32)> {
    if points.len() < 2 {
        return None;
    }
    let span_count = if closed {
        points.len()
    } else {
        points.len() - 1
    };
    let scaled = t.clamp(0.0, 1.0) * span_count as f32;
    let span = (scaled.floor() as usize).min(span_count - 1);
    let u = if t >= 1.0 { 1.0 } else { scaled.fract() };
    let at = |index: isize| -> (f32, f32) {
        if closed {
            points[index.rem_euclid(points.len() as isize) as usize]
        } else {
            points[index.clamp(0, points.len() as isize - 1) as usize]
        }
    };
    let p0 = at(span as isize - 1);
    let p1 = at(span as isize);
    let p2 = at(span as isize + 1);
    let p3 = at(span as isize + 2);
    let u2 = u * u;
    let u3 = u2 * u;
    let component = |a: f32, b: f32, c: f32, d: f32| {
        0.5 * ((2.0 * b)
            + (-a + c) * u
            + (2.0 * a - 5.0 * b + 4.0 * c - d) * u2
            + (-a + 3.0 * b - 3.0 * c + d) * u3)
    };
    Some((
        component(p0.0, p1.0, p2.0, p3.0),
        component(p0.1, p1.1, p2.1, p3.1),
    ))
}

fn evaluate_natural_cubic(points: &[(f32, f32)], t: f32) -> Option<(f32, f32)> {
    if points.len() < 3 {
        return evaluate_linear(points, t, false);
    }
    let second_derivatives = |coordinate: fn(&(f32, f32)) -> f32| {
        let count = points.len();
        let mut lower = vec![0.0_f32; count];
        let mut diagonal = vec![1.0_f32; count];
        let mut upper = vec![0.0_f32; count];
        let mut rhs = vec![0.0_f32; count];
        for index in 1..count - 1 {
            lower[index] = 1.0;
            diagonal[index] = 4.0;
            upper[index] = 1.0;
            rhs[index] = 6.0
                * (coordinate(&points[index + 1]) - 2.0 * coordinate(&points[index])
                    + coordinate(&points[index - 1]));
        }
        for index in 1..count {
            let factor = lower[index] / diagonal[index - 1];
            diagonal[index] -= factor * upper[index - 1];
            rhs[index] -= factor * rhs[index - 1];
        }
        let mut solved = vec![0.0_f32; count];
        solved[count - 1] = rhs[count - 1] / diagonal[count - 1];
        for index in (0..count - 1).rev() {
            solved[index] = (rhs[index] - upper[index] * solved[index + 1]) / diagonal[index];
        }
        solved
    };
    let mx = second_derivatives(|point| point.0);
    let my = second_derivatives(|point| point.1);
    let span_count = points.len() - 1;
    let scaled = t.clamp(0.0, 1.0) * span_count as f32;
    let span = (scaled.floor() as usize).min(span_count - 1);
    let u = if t >= 1.0 { 1.0 } else { scaled.fract() };
    let a = 1.0 - u;
    let b = u;
    let component = |index: usize, second: &[f32], coordinate: fn(&(f32, f32)) -> f32| {
        a * coordinate(&points[index])
            + b * coordinate(&points[index + 1])
            + ((a * a * a - a) * second[index] + (b * b * b - b) * second[index + 1]) / 6.0
    };
    Some((
        component(span, &mx, |point| point.0),
        component(span, &my, |point| point.1),
    ))
}

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchCurves {
    pub segments: Vec<LineSegment>,
    pub circles: Vec<Circle>,
    /// Analytic arc fragments (sketch fillets). `#[serde(default)]` so `.zcad`
    /// files written before arcs existed still deserialize.
    #[serde(default)]
    pub arcs: Vec<Arc>,
    /// Analytic editable spline records. Every existing consumer either reads
    /// these directly or uses the deterministic sampling path.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub splines: Vec<Spline>,
}

impl SketchCurves {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
            && self.circles.is_empty()
            && self.arcs.is_empty()
            && self.splines.is_empty()
    }

    /// Append every curve of `other` after this set's own curves. Used to fold
    /// a sketch's projected **face boundary** (reference geometry from the body
    /// face the sketch sits on) into region detection: the merge order —
    /// drawn curves first, boundary last — must be identical everywhere
    /// ([`detect_regions`] output order depends on input order, and extrude
    /// features store region *indices*).
    pub fn extend_curves(&mut self, other: &SketchCurves) {
        self.segments.extend(other.segments.iter().copied());
        self.circles.extend(other.circles.iter().copied());
        self.arcs.extend(other.arcs.iter().copied());
        self.splines.extend(other.splines.iter().cloned());
    }

    /// Append a rectangle as four segments around the two opposite corners.
    pub fn add_rectangle(&mut self, p0: (f32, f32), p2: (f32, f32)) {
        let p1 = (p2.0, p0.1);
        let p3 = (p0.0, p2.1);
        self.segments.push(LineSegment { a: p0, b: p1 });
        self.segments.push(LineSegment { a: p1, b: p2 });
        self.segments.push(LineSegment { a: p2, b: p3 });
        self.segments.push(LineSegment { a: p3, b: p0 });
    }

    pub fn add_line(&mut self, a: (f32, f32), b: (f32, f32)) {
        self.segments.push(LineSegment { a, b });
    }

    /// Chain the segments and arcs into one ordered OPEN polyline (2D sketch
    /// coordinates), for use as a sweep path. Curves are joined end-to-end by
    /// matching endpoints within `tol`; the walk starts at a degree-1 endpoint
    /// (an open chain has exactly two). Arcs are sampled along their minor arc.
    /// Returns `None` when the curves don't form a single simple chain (a
    /// branch, a closed loop with no free end, or a disjoint set). Circles are
    /// ignored (a closed path needs no free end and isn't a v1 sweep path).
    pub fn path_polyline(&self, tol: f32) -> Option<Vec<(f32, f32)>> {
        let (points, closed) = self.sweep_path_polyline(tol)?;
        (!closed).then_some(points)
    }

    /// Chain a Wave 3 sweep path and report whether it is closed. One circle
    /// is accepted as a closed path; mixed circles or branched/disjoint chains
    /// remain invalid instead of being partially consumed.
    pub fn sweep_path_polyline(&self, tol: f32) -> Option<(Vec<(f32, f32)>, bool)> {
        #[derive(Clone)]
        enum Seg {
            Line((f32, f32), (f32, f32)),
            Arc(Arc),
        }
        let mut segs: Vec<Seg> = Vec::new();
        for l in &self.segments {
            segs.push(Seg::Line(l.a, l.b));
        }
        for a in &self.arcs {
            segs.push(Seg::Arc(*a));
        }
        for spline in &self.splines {
            for pair in spline.sampled_points(0.01).windows(2) {
                segs.push(Seg::Line(pair[0], pair[1]));
            }
        }
        if !self.circles.is_empty() {
            if self.circles.len() != 1 || !segs.is_empty() {
                return None;
            }
            let circle = self.circles[0];
            let points = (0..48)
                .map(|index| {
                    let angle = std::f32::consts::TAU * index as f32 / 48.0;
                    (
                        circle.center.0 + circle.radius * angle.cos(),
                        circle.center.1 + circle.radius * angle.sin(),
                    )
                })
                .collect();
            return Some((points, true));
        }
        if segs.is_empty() {
            return None;
        }
        let ends = |s: &Seg| -> ((f32, f32), (f32, f32)) {
            match s {
                Seg::Line(a, b) => (*a, *b),
                Seg::Arc(a) => (a.start, a.end),
            }
        };
        let near = |a: (f32, f32), b: (f32, f32)| {
            ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt() <= tol
        };

        // Endpoint degree: find the two free ends of an open chain.
        let mut endpoints: Vec<(f32, f32)> = Vec::new();
        for s in &segs {
            let (a, b) = ends(s);
            endpoints.push(a);
            endpoints.push(b);
        }
        let degree =
            |p: (f32, f32), eps: &[(f32, f32)]| eps.iter().filter(|&&q| near(p, q)).count();
        if endpoints
            .iter()
            .any(|&point| !matches!(degree(point, &endpoints), 1 | 2))
        {
            return None;
        }
        let mut free: Vec<_> = endpoints
            .iter()
            .copied()
            .filter(|&point| degree(point, &endpoints) == 1)
            .collect();
        // Closed loops (every endpoint degree 2) have no free end → not a v1
        // open path.
        if !matches!(free.len(), 0 | 2) {
            return None;
        }
        let closed = free.is_empty();
        free.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
        let start = free.first().copied().unwrap_or_else(|| {
            endpoints
                .iter()
                .copied()
                .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)))
                .expect("non-empty curve collection has endpoints")
        });

        let mut used = vec![false; segs.len()];
        let mut chain_pts: Vec<(f32, f32)> = Vec::new();
        let mut cursor = start;
        chain_pts.push(cursor);
        for _ in 0..segs.len() {
            let Some(next_i) = (0..segs.len()).find(|&i| {
                !used[i] && {
                    let (a, b) = ends(&segs[i]);
                    near(a, cursor) || near(b, cursor)
                }
            }) else {
                break;
            };
            used[next_i] = true;
            let (a, b) = ends(&segs[next_i]);
            let (from, to) = if near(a, cursor) { (a, b) } else { (b, a) };
            match &segs[next_i] {
                Seg::Line(_, _) => chain_pts.push(to),
                Seg::Arc(arc) => {
                    let (stored_first, stored_last) = arc.parameter_bounds();
                    let forward = near(from, arc.start);
                    let (a0, a1) = if forward {
                        (stored_first, stored_last)
                    } else {
                        (stored_last, stored_first)
                    };
                    let steps = 12;
                    for k in 1..=steps {
                        let t = a0 + (a1 - a0) * (k as f64 / steps as f64);
                        chain_pts.push((
                            arc.center.0 + arc.radius * t.cos() as f32,
                            arc.center.1 + arc.radius * t.sin() as f32,
                        ));
                    }
                }
            }
            cursor = to;
        }
        // Every curve must have been consumed (a single simple chain).
        if used.iter().any(|&u| !u)
            || chain_pts.len() < 2
            || (closed && !near(*chain_pts.last().unwrap(), start))
        {
            return None;
        }
        if closed {
            chain_pts.pop();
        }
        Some((chain_pts, closed))
    }

    /// Resolve a normalized arc-length parameter on one connected curve chain.
    ///
    /// This is used by guided Sweep anchors. It deliberately shares the same
    /// ordering and connectivity rules as the path/guide evaluator, so a
    /// durable entity cannot preview one point and evaluate another.
    pub fn normalized_chain_point(&self, parameter: f32, tol: f32) -> Option<(f32, f32)> {
        if !parameter.is_finite() || !(0.0..=1.0).contains(&parameter) {
            return None;
        }
        let (points, closed) = self.sweep_path_polyline(tol)?;
        let segment_count = points.len().saturating_sub(1) + usize::from(closed);
        if segment_count == 0 {
            return None;
        }
        let segment = |index: usize| {
            let a = points[index];
            let b = if index + 1 < points.len() {
                points[index + 1]
            } else {
                points[0]
            };
            (a, b)
        };
        let mut lengths = Vec::with_capacity(segment_count);
        let mut total = 0.0f32;
        for index in 0..segment_count {
            let (a, b) = segment(index);
            let length = (b.0 - a.0).hypot(b.1 - a.1);
            lengths.push(length);
            total += length;
        }
        if !total.is_finite() || total <= tol.max(f32::EPSILON) {
            return None;
        }
        let target = parameter * total;
        let mut traversed = 0.0f32;
        for (index, length) in lengths.iter().copied().enumerate() {
            if target <= traversed + length || index + 1 == segment_count {
                let fraction = if length > 0.0 {
                    ((target - traversed) / length).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (a, b) = segment(index);
                return Some((a.0 + (b.0 - a.0) * fraction, a.1 + (b.1 - a.1) * fraction));
            }
            traversed += length;
        }
        None
    }

    pub fn add_circle(&mut self, center: (f32, f32), radius: f32) {
        if radius > 0.0 {
            self.circles.push(Circle { center, radius });
        }
    }

    pub fn add_spline(&mut self, spline: Spline) {
        if spline.is_valid() {
            self.splines.push(spline);
        }
    }

    /// Append an ellipse as a faceted closed polyline (the same polygon-
    /// approximation strategy circles/cylinders already use for the boolean
    /// kernel — there is no analytic ellipse primitive). `major` is the
    /// half-axis vector from the center to the end of the major axis (its length
    /// is the major radius and its direction sets the rotation); `minor_radius`
    /// is the perpendicular half-axis length. Emits [`ELLIPSE_SEGS`] segments.
    pub fn add_ellipse(&mut self, center: (f32, f32), major: (f32, f32), minor_radius: f32) {
        let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
        if rx <= 1e-4 || minor_radius <= 1e-4 {
            return;
        }
        // Unit major axis and the unit minor axis (90° CCW from it).
        let (ux, uy) = (major.0 / rx, major.1 / rx);
        let (px, py) = (-uy, ux);
        let ry = minor_radius;
        let n = ELLIPSE_SEGS;
        let pts: Vec<(f32, f32)> = (0..n)
            .map(|k| {
                let t = (k as f32) / (n as f32) * std::f32::consts::TAU;
                let (ct, st) = (t.cos(), t.sin());
                (
                    center.0 + rx * ct * ux + ry * st * px,
                    center.1 + rx * ct * uy + ry * st * py,
                )
            })
            .collect();
        for k in 0..n {
            self.segments.push(LineSegment {
                a: pts[k],
                b: pts[(k + 1) % n],
            });
        }
    }

    /// Remove the most recently added primitive (LIFO across circles, arcs, then
    /// segments). Rectangles count as 4 segments — call 4× to undo one.
    pub fn pop_last(&mut self) -> bool {
        self.splines.pop().is_some()
            || self.circles.pop().is_some()
            || self.arcs.pop().is_some()
            || self.segments.pop().is_some()
    }
}

// ---------------------------------------------------------------------------
// Parametric sketch shapes
// ---------------------------------------------------------------------------

/// A sketch dimension that may be a literal or an arithmetic expression.
/// `value` is the resolved fallback (the value drawn / last known); `expr`, when
/// set, is kept as the editable source and re-evaluated every build. Expressions
/// may contain plain arithmetic and/or document variables.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Dimension {
    pub value: f32,
    #[serde(default)]
    pub expr: Option<String>,
}

impl Dimension {
    /// A plain literal dimension (no variable binding).
    pub fn literal(value: f32) -> Self {
        Self { value, expr: None }
    }

    /// Resolve to a number in base units (mm): the expression if it still
    /// evaluates, otherwise the stored fallback value.
    pub fn resolve(&self, vars: &HashMap<String, f64>) -> f32 {
        self.expr
            .as_ref()
            .and_then(|e| crate::expr::eval(e, vars).ok())
            .map(|v| v as f32)
            .unwrap_or(self.value)
    }
}

/// A parametric primitive in a sketch: the construction (anchor points + named
/// dimensions) rather than baked coordinates, so it can be rebuilt against the
/// current variables. Tools without dimension fields (3-point shapes, ellipses)
/// are stored pre-built as [`SketchShape::Raw`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SketchImportFormat {
    Dxf,
}

/// Source identity retained with imported sketch geometry. Geometry is stored
/// in ZeroCAD's millimetre base unit, while these fields preserve the source
/// layer and unit declaration for inspection and deterministic re-export.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ImportedSketchMetadata {
    pub format: SketchImportFormat,
    pub source_name: String,
    pub layer: String,
    pub source_unit_code: u16,
    pub source_unit_name: String,
    pub source_scale_to_mm: f64,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SketchShape {
    /// Axis-aligned rectangle. When `from_center`, `origin` is the center and
    /// `w`/`h` are full extents; otherwise `origin` is a corner and the opposite
    /// corner is `origin + (sx·w, sy·h)` (`sx`/`sy` record the drawn direction).
    Rectangle {
        origin: (f32, f32),
        sx: f32,
        sy: f32,
        w: Dimension,
        h: Dimension,
        from_center: bool,
    },
    Circle {
        center: (f32, f32),
        diameter: Dimension,
    },
    Line {
        start: (f32, f32),
        length: Dimension,
        angle_deg: Dimension,
    },
    /// Regular polygon sized from a construction circle's diameter. The guide
    /// circle is never emitted as sketch geometry: when `circumscribed` is
    /// false the vertices lie on it; when true the polygon's edges are tangent
    /// to it.
    RegularPolygon {
        center: (f32, f32),
        sides: u32,
        diameter: Dimension,
        rotation_deg: f32,
        circumscribed: bool,
    },
    /// Center-to-center slot. `start` and `end` locate the semicircle centers;
    /// `width` is the full slot width. Coincident centers intentionally produce
    /// a circle so the three-click tool remains well-defined at zero length.
    Slot {
        start: (f32, f32),
        end: (f32, f32),
        width: Dimension,
    },
    /// An editable native spline. Its defining points remain available after
    /// save/load instead of being reduced to an anonymous polyline.
    Spline { spline: Spline },
    /// One source layer from an imported sketch file. Keeping layers as
    /// separate shape records makes layer identity stable and inspectable while
    /// still feeding the normal sketch-region pipeline.
    Imported {
        curves: SketchCurves,
        metadata: ImportedSketchMetadata,
    },
    /// Pre-built geometry with no variable bindings (3-point rect/circle,
    /// ellipses). Stored as-is and emitted verbatim.
    Raw { curves: SketchCurves },
    /// Shaped text. `curves` are the BAKED glyph outlines — authoritative
    /// forever: they are never re-shaped on load, so a document renders
    /// identically on a machine without the font. The semantic fields exist so
    /// an edit session with a matching installed font (verified through the
    /// fingerprint's content hash, never the family name) can re-shape and
    /// re-bake; without it the shape is displayed read-only.
    Text {
        text: String,
        font: crate::text::FontFingerprint,
        params: crate::text::TextParams,
        placement: crate::text::TextPlacement,
        curves: SketchCurves,
    },
}

impl SketchShape {
    /// Build this shape's curves, resolving any dimension expressions against
    /// `vars` (variable values in base units / mm).
    pub fn build(&self, vars: &HashMap<String, f64>) -> SketchCurves {
        let mut c = SketchCurves::new();
        match self {
            SketchShape::Rectangle {
                origin,
                sx,
                sy,
                w,
                h,
                from_center,
            } => {
                let (wv, hv) = (w.resolve(vars), h.resolve(vars));
                if *from_center {
                    let (hw, hh) = (wv * 0.5, hv * 0.5);
                    c.add_rectangle(
                        (origin.0 - hw, origin.1 - hh),
                        (origin.0 + hw, origin.1 + hh),
                    );
                } else {
                    c.add_rectangle(*origin, (origin.0 + sx * wv, origin.1 + sy * hv));
                }
            }
            SketchShape::Circle { center, diameter } => {
                c.add_circle(*center, (diameter.resolve(vars) * 0.5).max(0.0));
            }
            SketchShape::Line {
                start,
                length,
                angle_deg,
            } => {
                let len = length.resolve(vars);
                let ang = angle_deg.resolve(vars).to_radians();
                c.add_line(
                    *start,
                    (start.0 + len * ang.cos(), start.1 + len * ang.sin()),
                );
            }
            SketchShape::RegularPolygon {
                center,
                sides,
                diameter,
                rotation_deg,
                circumscribed,
            } => {
                let n = (*sides).clamp(3, 64) as usize;
                let guide_radius = (diameter.resolve(vars) * 0.5).max(0.0);
                if guide_radius > 1.0e-4 {
                    let step = std::f32::consts::TAU / n as f32;
                    let guide_angle = rotation_deg.to_radians();
                    let (radius, first_angle) = if *circumscribed {
                        (guide_radius / (step * 0.5).cos(), guide_angle - step * 0.5)
                    } else {
                        (guide_radius, guide_angle)
                    };
                    let vertices: Vec<(f32, f32)> = (0..n)
                        .map(|k| {
                            let angle = first_angle + step * k as f32;
                            (
                                center.0 + radius * angle.cos(),
                                center.1 + radius * angle.sin(),
                            )
                        })
                        .collect();
                    for k in 0..n {
                        c.add_line(vertices[k], vertices[(k + 1) % n]);
                    }
                }
            }
            SketchShape::Slot { start, end, width } => {
                let width = width.resolve(vars);
                if !width.is_finite() || width <= 0.0 {
                    return c;
                }
                let radius = width * 0.5;
                let dx = end.0 - start.0;
                let dy = end.1 - start.1;
                let centerline_length = dx.hypot(dy);
                if centerline_length <= 1.0e-5 {
                    c.add_circle(*start, radius);
                    return c;
                }

                let normal = (-dy / centerline_length, dx / centerline_length);
                let start_left = (start.0 + normal.0 * radius, start.1 + normal.1 * radius);
                let end_left = (end.0 + normal.0 * radius, end.1 + normal.1 * radius);
                let end_right = (end.0 - normal.0 * radius, end.1 - normal.1 * radius);
                let start_right = (start.0 - normal.0 * radius, start.1 - normal.1 * radius);

                c.add_line(start_left, end_left);
                c.arcs.push(Arc {
                    center: *end,
                    radius,
                    start: end_left,
                    end: end_right,
                    clockwise: true,
                });
                c.add_line(end_right, start_right);
                c.arcs.push(Arc {
                    center: *start,
                    radius,
                    start: start_right,
                    end: start_left,
                    clockwise: true,
                });
            }
            SketchShape::Spline { spline } => c.add_spline(spline.clone()),
            SketchShape::Imported { curves, .. } => c = curves.clone(),
            SketchShape::Raw { curves } => c = curves.clone(),
            // Baked text is authoritative: never re-shaped here, even when the
            // font is installed — a font update must not silently move geometry.
            SketchShape::Text { curves, .. } => c = curves.clone(),
        }
        c
    }
}

/// Build the full set of sketch curves from a parametric shape list, resolving
/// every dimension expression against `vars`. This is the single source of
/// truth for a parametric sketch's geometry (region detection, rendering, and
/// extrusion all consume the result).
pub fn build_sketch_curves(shapes: &[SketchShape], vars: &HashMap<String, f64>) -> SketchCurves {
    let mut out = SketchCurves::new();
    for s in shapes {
        let c = s.build(vars);
        out.segments.extend(c.segments);
        out.circles.extend(c.circles);
        out.arcs.extend(c.arcs);
        out.splines.extend(c.splines);
    }
    out
}

/// Whether a corner modifier rounds the corner (fillet) or bevels it (chamfer).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CornerKind {
    Fillet,
    Chamfer,
}

/// A fillet/chamfer applied to a sketch corner — the vertex where two segments
/// meet. Stored at the sketch level (the corner may join segments from different
/// shapes) and applied after the shapes are built, so the underlying shapes stay
/// parametric. `radius` may itself be variable-bound.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CornerMod {
    /// The corner location in sketch coordinates, snapped to the nearest shared
    /// vertex when applied (so it survives the geometry being rebuilt).
    pub at: (f32, f32),
    pub radius: Dimension,
    pub kind: CornerKind,
}

/// An **associative** sketch mirror: a reflection axis (two points in sketch
/// coordinates). Stored at the sketch level and applied *after* the shapes and
/// corner mods are built — it reflects the current curve set across the axis and
/// appends the copy, so editing the source geometry updates the mirror on every
/// rebuild. Multiple mirrors compose in order (a second mirror reflects the
/// accumulated result, e.g. mirror-X then mirror-Y gives four copies).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchMirror {
    pub a: (f32, f32),
    pub b: (f32, f32),
}

/// Reflect point `p` across the line through `a` with unit direction `d`.
fn reflect_pt(p: (f32, f32), a: (f32, f32), d: (f32, f32)) -> (f32, f32) {
    let v = (p.0 - a.0, p.1 - a.1);
    let proj = v.0 * d.0 + v.1 * d.1;
    (
        2.0 * (a.0 + proj * d.0) - p.0,
        2.0 * (a.1 + proj * d.1) - p.1,
    )
}

/// Reflect an entire curve set across the axis `a`→`b`. Reflection is an
/// isometry, so radii are preserved and a minor arc stays a minor arc — only the
/// defining points move. A degenerate axis (`a == b`) returns an empty set.
pub fn reflect_curves_across(curves: &SketchCurves, a: (f32, f32), b: (f32, f32)) -> SketchCurves {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt();
    let mut out = SketchCurves::new();
    if len < 1.0e-6 {
        return out;
    }
    let d = (dx / len, dy / len);
    for s in &curves.segments {
        out.add_line(reflect_pt(s.a, a, d), reflect_pt(s.b, a, d));
    }
    for c in &curves.circles {
        out.add_circle(reflect_pt(c.center, a, d), c.radius);
    }
    for arc in &curves.arcs {
        out.arcs.push(Arc {
            center: reflect_pt(arc.center, a, d),
            radius: arc.radius,
            start: reflect_pt(arc.start, a, d),
            end: reflect_pt(arc.end, a, d),
            // Reflection reverses orientation. Leaving the reflected arc in
            // automatic-shortest mode preserves its geometric half/span.
            clockwise: false,
        });
    }
    for spline in &curves.splines {
        let mut reflected = spline.clone();
        reflected.points = spline
            .points
            .iter()
            .map(|point| reflect_pt(*point, a, d))
            .collect();
        out.splines.push(reflected);
    }
    out
}

/// Apply every mirror in `mirrors` to `c` in order, each reflecting the current
/// accumulated curves across its axis and appending the copy.
pub fn apply_mirrors(c: &mut SketchCurves, mirrors: &[SketchMirror]) {
    for m in mirrors {
        let reflected = reflect_curves_across(c, m.a, m.b);
        c.extend_curves(&reflected);
    }
}

/// How many straight segments to approximate an arc of `angle` radians and
/// radius `r` so it reads as a *smooth* curve. Combines an angular budget
/// (~3.6°/segment, so the polyline corners are imperceptible at any size) with a
/// chord-tolerance budget (finer for large radii, where 3.6° would still bow
/// visibly), clamped to a sane range so tiny fillets stay cheap and huge ones
/// don't explode region detection.
fn arc_segments(angle: f32, r: f32) -> usize {
    let angle = angle.abs();
    // ~3.6° per segment.
    let by_angle = (angle / 0.063).ceil() as usize;
    // Chord error ≤ 0.01mm: each step subtends ≤ 2·acos(1 − tol/r).
    let tol = 0.01_f32;
    let by_chord = if r > tol {
        let step = 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos();
        if step > 1.0e-4 {
            (angle / step).ceil() as usize
        } else {
            2
        }
    } else {
        2
    };
    by_angle.max(by_chord).clamp(6, 160)
}

/// The live geometry of a sketch: rebuilt from its parametric `shapes` against
/// `vars` when present (else the baked `curves` for legacy documents), then with
/// any fillet/chamfer `corner_mods` applied.
pub fn effective_curves(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    corner_mods: &[CornerMod],
    vars: &HashMap<String, f64>,
) -> SketchCurves {
    effective_curves_solved(curves, shapes, corner_mods, &[], None, vars)
}

/// [`effective_curves`] with the constraint-solver model. When `solver` is
/// present (and non-empty) it is the source of truth: entities bake to curves
/// in stored (id) order — the same `SketchCurves` every downstream consumer
/// already reads, so regions/extrude/booleans are untouched by the solver's
/// existence.
///
/// Evaluation stays **pure**: the stored point positions are trusted as-is
/// (they are the last committed solve) unless a dimensional constraint is
/// variable-bound — then the model is re-solved against the current variables
/// so editing a variable moves the geometry, and the result is used without
/// mutating the document. A failed re-solve bakes the stored last-valid
/// positions — a bad constraint edit degrades the sketch, never blanks it.
pub fn effective_curves_solved(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    corner_mods: &[CornerMod],
    mirrors: &[SketchMirror],
    solver: Option<&SketchSolverModel>,
    vars: &HashMap<String, f64>,
) -> SketchCurves {
    effective_curves_solved_checked(curves, shapes, corner_mods, mirrors, solver, vars).0
}

/// Checked sketch rebuild used by the evaluator. Compatibility/UI callers may
/// use [`effective_curves_solved`] when only last-valid geometry is required.
pub fn effective_curves_solved_checked(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    corner_mods: &[CornerMod],
    mirrors: &[SketchMirror],
    solver: Option<&SketchSolverModel>,
    vars: &HashMap<String, f64>,
) -> (SketchCurves, Vec<(EntityId, SketchOffsetError)>) {
    if let Some(model) = solver.filter(|m| !m.is_empty()) {
        let mut c = if solve::has_variable_bound_constraint(model) {
            let report = solve::solve_model(model, vars);
            if report.outcome == SolveOutcome::Converged {
                let mut solved = model.clone();
                solve::apply_solution(&mut solved, &report);
                constraints::bake_entities_to_curves(&solved)
            } else {
                constraints::bake_entities_to_curves(model)
            }
        } else {
            constraints::bake_entities_to_curves(model)
        };
        for m in corner_mods {
            let r = m.radius.resolve(vars);
            apply_corner_mod(&mut c, m.at, r, m.kind);
        }
        let failures = apply_associative_offsets(&mut c, model, vars);
        let mut failures = failures;
        failures.extend(
            apply_associative_patterns(&mut c, model, vars)
                .into_iter()
                .map(|(id, error)| (id, SketchOffsetError::Pattern(error))),
        );
        // Text shapes are rigid one-item blocks: the solver model holds only
        // their anchor point (see `promote_shapes_to_entities`), so their baked
        // outlines are appended here from the authoritative shape records.
        for shape in shapes {
            if let SketchShape::Text { curves, .. } = shape {
                c.extend_curves(curves);
            }
        }
        apply_mirrors(&mut c, mirrors);
        return (c, failures);
    }
    let mut c = if shapes.is_empty() {
        curves.clone()
    } else {
        build_sketch_curves(shapes, vars)
    };
    for m in corner_mods {
        let r = m.radius.resolve(vars);
        apply_corner_mod(&mut c, m.at, r, m.kind);
    }
    apply_mirrors(&mut c, mirrors);
    (c, Vec::new())
}

/// Resolve only the curve(s) owned by one durable sketch entity.
///
/// Solver-backed legacy shapes may expand into several low-level entities;
/// their `derived_from` provenance keeps the original shape id addressable.
/// Construction geometry is intentionally excluded from material anchors.
pub fn entity_curves_solved(
    shapes: &[SketchShape],
    entity_ids: &[EntityId],
    solver: Option<&SketchSolverModel>,
    entity: EntityId,
    vars: &HashMap<String, f64>,
) -> Option<SketchCurves> {
    if let Some(model) = solver.filter(|model| !model.is_empty()) {
        let mut solved = model.clone();
        if solve::has_variable_bound_constraint(model) {
            let report = solve::solve_model(model, vars);
            if report.outcome == SolveOutcome::Converged {
                solve::apply_solution(&mut solved, &report);
            }
        }
        let construction = solved.construction.clone();
        solved.entities.retain(|candidate| {
            !construction.contains(&candidate.id())
                && (candidate.id() == entity || candidate.derived_from() == Some(entity))
        });
        solved.construction.clear();
        let curves = constraints::bake_entities_to_curves(&solved);
        return (!curves.is_empty()).then_some(curves);
    }

    let ids = effective_shape_ids(shapes.len(), entity_ids);
    ids.iter()
        .position(|candidate| *candidate == entity)
        .map(|index| shapes[index].build(vars))
        .filter(|curves| !curves.is_empty())
}

fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    let (dx, dy) = (a.0 - b.0, a.1 - b.1);
    dx * dx + dy * dy
}

/// Round or bevel the corner of `curves` nearest `at`. Snaps `at` to the closest
/// segment endpoint, finds the two segments meeting there, trims them back, and
/// inserts a faceted arc (fillet) or a straight bevel (chamfer). No-ops unless
/// exactly two segments share the corner and the radius is usable.
fn apply_corner_mod(curves: &mut SketchCurves, at: (f32, f32), radius: f32, kind: CornerKind) {
    if radius <= 1e-4 {
        return;
    }
    // Snap to the nearest existing vertex.
    let mut v = None;
    let mut best = f32::MAX;
    for s in &curves.segments {
        for &p in &[s.a, s.b] {
            let d = dist2(p, at);
            if d < best {
                best = d;
                v = Some(p);
            }
        }
    }
    let Some(v) = v else {
        return;
    };

    // The (at most two) segments touching that vertex, with which endpoint.
    const TOL2: f32 = 1e-6;
    let mut touching: Vec<(usize, bool)> = Vec::new(); // (index, endpoint_is_a)
    for (i, s) in curves.segments.iter().enumerate() {
        if dist2(s.a, v) < TOL2 {
            touching.push((i, true));
        } else if dist2(s.b, v) < TOL2 {
            touching.push((i, false));
        }
    }
    if touching.len() != 2 {
        return;
    }
    let (i1, a1) = touching[0];
    let (i2, a2) = touching[1];
    let far1 = if a1 {
        curves.segments[i1].b
    } else {
        curves.segments[i1].a
    };
    let far2 = if a2 {
        curves.segments[i2].b
    } else {
        curves.segments[i2].a
    };

    let len1 = dist2(far1, v).sqrt();
    let len2 = dist2(far2, v).sqrt();
    if len1 < 1e-4 || len2 < 1e-4 {
        return;
    }
    let u = ((far1.0 - v.0) / len1, (far1.1 - v.1) / len1);
    let w = ((far2.0 - v.0) / len2, (far2.1 - v.1) / len2);
    let cos_t = (u.0 * w.0 + u.1 * w.1).clamp(-1.0, 1.0);
    let theta = cos_t.acos();
    if theta < 1e-3 || (std::f32::consts::PI - theta) < 1e-3 {
        return; // collinear — nothing to round
    }
    let half = theta * 0.5;
    // Setback distance along each edge; shrink the radius if it won't fit.
    let mut t = radius / half.tan();
    let max_t = len1.min(len2) * 0.95;
    let radius = if t > max_t {
        t = max_t;
        max_t * half.tan()
    } else {
        radius
    };
    let p1 = (v.0 + u.0 * t, v.1 + u.1 * t);
    let p2 = (v.0 + w.0 * t, v.1 + w.1 * t);

    // Trim the two segments back to the setback points.
    if a1 {
        curves.segments[i1].a = p1;
    } else {
        curves.segments[i1].b = p1;
    }
    if a2 {
        curves.segments[i2].a = p2;
    } else {
        curves.segments[i2].b = p2;
    }

    match kind {
        CornerKind::Chamfer => {
            curves.segments.push(LineSegment { a: p1, b: p2 });
        }
        CornerKind::Fillet => {
            // Arc center sits along the angle bisector, distance r/sin(half).
            let bl = ((u.0 + w.0).powi(2) + (u.1 + w.1).powi(2)).sqrt();
            if bl < 1e-5 {
                curves.segments.push(LineSegment { a: p1, b: p2 });
                return;
            }
            let bis = ((u.0 + w.0) / bl, (u.1 + w.1) / bl);
            let cd = radius / half.sin();
            let c = (v.0 + bis.0 * cd, v.1 + bis.1 * cd);
            push_arc(curves, c, p1, p2, radius);
            // Record the analytic arc alongside the tessellated segments so the
            // extrude wire builder can sweep it to an exact cylindrical wall (the
            // segments still drive 2D rendering, region detection, and provenance).
            curves.arcs.push(Arc {
                center: c,
                radius,
                start: p1,
                end: p2,
                clockwise: false,
            });
        }
    }
}

/// Append an arc from `p1` to `p2` about center `c` (radius `r`), taking the
/// short way around. Used to draw a fillet. The arc is tessellated finely enough
/// (see [`arc_segments`]) to read as a smooth curve rather than a few flats.
fn push_arc(curves: &mut SketchCurves, c: (f32, f32), p1: (f32, f32), p2: (f32, f32), r: f32) {
    use std::f32::consts::{PI, TAU};
    let a0 = (p1.1 - c.1).atan2(p1.0 - c.0);
    let a1 = (p2.1 - c.1).atan2(p2.0 - c.0);
    let mut da = a1 - a0;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let n = arc_segments(da.abs(), r);
    let mut prev = p1;
    for k in 1..=n {
        let a = a0 + da * (k as f32 / n as f32);
        let pt = (c.0 + r * a.cos(), c.1 + r * a.sin());
        curves.segments.push(LineSegment { a: prev, b: pt });
        prev = pt;
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Region {
    /// Closed CCW outer boundary polygon in sketch plane coordinates.
    pub boundary: Vec<(f32, f32)>,
    /// Inner boundaries (holes) — e.g. a shape drawn fully inside this face.
    /// Each is a closed loop; the face is the outer area minus the holes.
    #[serde(default)]
    pub holes: Vec<Vec<(f32, f32)>>,
    /// Net area: outer boundary area minus the area of the holes.
    pub area: f32,
    /// Runtime analytic boundary used by B-Rep consumers. It is rebuilt from
    /// persisted sketch intent and never changes `.zcad` payload bytes.
    #[serde(skip)]
    pub analytic: Option<AnalyticSketchRegion>,
}

impl Region {
    /// Number of straight boundary chords needed by the legacy sampled prism
    /// builder. Analytic consumers should prefer [`Self::analytic`]; this count
    /// is used to keep compatibility/display fallbacks within an interactive
    /// complexity budget.
    pub fn sampled_edge_count(&self) -> usize {
        self.boundary.len() + self.holes.iter().map(Vec::len).sum::<usize>()
    }

    /// True if `p` is inside this face: within the outer boundary and not in
    /// any hole.
    pub fn contains(&self, p: (f32, f32)) -> bool {
        if !point_in_polygon(p, &self.boundary) {
            return false;
        }
        !self.holes.iter().any(|h| point_in_polygon(p, h))
    }

    /// Why this face cannot be swept into a solid, when that is decidable from
    /// its boundary alone. Sweeping needs a boundary with locally positive
    /// width; the shapes below have none, so a solid builder can only return
    /// "no solid" and the caller would otherwise drop the face in silence.
    pub fn degeneracy(&self) -> Option<RegionDegeneracy> {
        if let Some(cusp) = boundary_cusp(&self.boundary) {
            return Some(cusp);
        }
        self.holes.iter().find_map(|hole| boundary_cusp(hole))
    }
}

/// A boundary shape that cannot carry a swept wall.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegionDegeneracy {
    /// The boundary reverses on itself, pinching the face to zero width. Exact
    /// tangential contact produces this: a circle resting on the edge it
    /// touches leaves the contact point along the same line it arrived on, so
    /// the face has a zero interior angle there. Perturbing either curve by a
    /// fraction of a micron removes it, which is what makes it so easy to hit
    /// by construction and so confusing to diagnose.
    Cusp { at: (f32, f32), turn_degrees: f32 },
}

/// Smallest turn that counts as the boundary doubling back. A discretized
/// tangential contact measures ~176°; ordinary corners (including the sharp
/// ones a user draws deliberately) stay well below this, and a coarsely
/// sampled arc turns only a few degrees per step.
const CUSP_TURN_DEGREES: f32 = 165.0;

fn boundary_cusp(boundary: &[(f32, f32)]) -> Option<RegionDegeneracy> {
    let count = boundary.len();
    if count < 3 {
        return None;
    }
    let mut sharpest: Option<((f32, f32), f32)> = None;
    for index in 0..count {
        let previous = boundary[(index + count - 1) % count];
        let point = boundary[index];
        let next = boundary[(index + 1) % count];
        let incoming = (point.0 - previous.0, point.1 - previous.1);
        let outgoing = (next.0 - point.0, next.1 - point.1);
        let incoming_length = incoming.0.hypot(incoming.1);
        let outgoing_length = outgoing.0.hypot(outgoing.1);
        // A repeated point carries no direction; it is the arrangement's own
        // seam bookkeeping, not a cusp.
        if incoming_length <= f32::EPSILON || outgoing_length <= f32::EPSILON {
            continue;
        }
        let cosine = (incoming.0 * outgoing.0 + incoming.1 * outgoing.1)
            / (incoming_length * outgoing_length);
        let turn_degrees = cosine.clamp(-1.0, 1.0).acos().to_degrees();
        let sharper = match sharpest {
            Some((_, best)) => turn_degrees > best,
            None => true,
        };
        if turn_degrees >= CUSP_TURN_DEGREES && sharper {
            sharpest = Some((point, turn_degrees));
        }
    }
    sharpest.map(|(at, turn_degrees)| RegionDegeneracy::Cusp { at, turn_degrees })
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RegionWithProvenance {
    pub region: Region,
    pub provenance: RegionProvenance,
}

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RegionProvenance {
    #[serde(default)]
    pub fragments: Vec<RegionProvenanceFragment>,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RegionProvenanceFragment {
    RectangleEdge {
        shape_id: Option<usize>,
        edge_index: usize,
        rect_min: (f32, f32),
        rect_max: (f32, f32),
    },
    CircleArc {
        shape_id: Option<usize>,
        center: (f32, f32),
        radius: f32,
    },
    SketchFilletArc {
        shape_id: Option<usize>,
    },
    SketchChamferEdge {
        shape_id: Option<usize>,
    },
    Slot {
        shape_id: Option<usize>,
        #[serde(default)]
        boundary: SlotBoundary,
    },
    RoundedRectangle {
        shape_id: Option<usize>,
    },
    RawPolyline {
        shape_id: Option<usize>,
    },
}

/// Stable identity for each native-slot boundary component.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize,
)]
pub enum SlotBoundary {
    #[default]
    LeftSide,
    EndCap,
    RightSide,
    StartCap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SketchCurveFamily {
    Line,
    Circle,
    Arc,
    Spline,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SketchCurveProvenance {
    pub family: SketchCurveFamily,
    pub index: usize,
}

pub type AnalyticSketchRegion = openrcad::sketch::ArrangementRegion<SketchCurveProvenance>;

#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticSketchError {
    UnsupportedSpline { index: usize, kind: SplineKind },
    Arrangement(openrcad::sketch::ArrangementError),
}

impl std::fmt::Display for AnalyticSketchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedSpline { index, kind } => {
                write!(
                    formatter,
                    "spline {index} ({kind:?}) is not supported analytically"
                )
            }
            Self::Arrangement(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for AnalyticSketchError {}

pub fn detect_regions_with_provenance(
    curves: &SketchCurves,
    shapes: &[SketchShape],
) -> Vec<RegionWithProvenance> {
    let regions = detect_regions(curves);
    let ids = EntityId::sequence(shapes.len());
    let provenance = build_region_provenance(curves, shapes, &ids, &regions);
    regions
        .into_iter()
        .zip(provenance)
        .map(|(region, provenance)| RegionWithProvenance { region, provenance })
        .collect()
}

pub fn build_region_provenance(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    entity_ids: &[EntityId],
    regions: &[Region],
) -> Vec<RegionProvenance> {
    let fragments = sketch_provenance_fragments(curves, shapes, entity_ids);
    regions
        .iter()
        .map(|_| RegionProvenance {
            fragments: fragments.clone(),
        })
        .collect()
}

/// Provenance fragments key their `shape_id` by the shape's durable
/// [`EntityId`] — never the `shapes` Vec position — so deleting one shape does
/// not re-key every reference captured against its neighbours. Legacy sketches
/// (no stored ids) backfill positionally, which reproduces the pre-id grammar
/// exactly.
fn sketch_provenance_fragments(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    entity_ids: &[EntityId],
) -> Vec<RegionProvenanceFragment> {
    let ids = effective_shape_ids(shapes.len(), entity_ids);
    let id_at = |pos: usize| ids.get(pos).map(|id| id.0 as usize);
    let mut fragments = Vec::new();
    let rectangle_shape_id = shapes
        .iter()
        .position(|shape| matches!(shape, SketchShape::Rectangle { .. }))
        .and_then(id_at);
    let circle_shape_ids: Vec<usize> = shapes
        .iter()
        .enumerate()
        .filter_map(|(i, shape)| {
            matches!(shape, SketchShape::Circle { .. })
                .then(|| id_at(i))
                .flatten()
        })
        .collect();
    for (position, shape) in shapes.iter().enumerate() {
        if matches!(shape, SketchShape::Slot { .. }) {
            for boundary in [
                SlotBoundary::LeftSide,
                SlotBoundary::EndCap,
                SlotBoundary::RightSide,
                SlotBoundary::StartCap,
            ] {
                fragments.push(RegionProvenanceFragment::Slot {
                    shape_id: id_at(position),
                    boundary,
                });
            }
        }
    }
    if let Some((rect_min, rect_max)) = rectangle_bounds_from_segments(&curves.segments) {
        for edge_index in 0..4 {
            fragments.push(RegionProvenanceFragment::RectangleEdge {
                shape_id: rectangle_shape_id,
                edge_index,
                rect_min,
                rect_max,
            });
        }
    }
    for (i, circle) in curves.circles.iter().enumerate() {
        fragments.push(RegionProvenanceFragment::CircleArc {
            shape_id: circle_shape_ids.get(i).copied(),
            center: circle.center,
            radius: circle.radius,
        });
    }
    if fragments.is_empty()
        && (!curves.segments.is_empty() || !curves.arcs.is_empty() || !curves.splines.is_empty())
    {
        fragments.push(RegionProvenanceFragment::RawPolyline {
            shape_id: shapes
                .iter()
                .position(|shape| {
                    matches!(
                        shape,
                        SketchShape::RegularPolygon { .. }
                            | SketchShape::Slot { .. }
                            | SketchShape::Spline { .. }
                            | SketchShape::Imported { .. }
                            | SketchShape::Raw { .. }
                    )
                })
                .and_then(id_at),
        });
    }
    fragments
}

fn rectangle_bounds_from_segments(segments: &[LineSegment]) -> Option<((f32, f32), (f32, f32))> {
    if segments.len() != 4 {
        return None;
    }
    let mut pts: Vec<(f32, f32)> = Vec::new();
    for seg in segments {
        for p in [seg.a, seg.b] {
            if !pts.iter().any(|q| (q.0 - p.0).hypot(q.1 - p.1) <= 1.0e-4) {
                pts.push(p);
            }
        }
    }
    if pts.len() != 4 {
        return None;
    }

    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for (x, y) in pts {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    if max_x - min_x <= 1.0e-3 || max_y - min_y <= 1.0e-3 {
        return None;
    }

    let has_corner = |p: (f32, f32)| {
        segments
            .iter()
            .flat_map(|s| [s.a, s.b])
            .any(|q| (q.0 - p.0).abs() <= 1.0e-4 && (q.1 - p.1).abs() <= 1.0e-4)
    };
    for corner in [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ] {
        if !has_corner(corner) {
            return None;
        }
    }
    Some(((min_x, min_y), (max_x, max_y)))
}

// ---------------------------------------------------------------------------
// Region detection
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct P {
    x: f64,
    y: f64,
}

impl P {
    fn from(p: (f32, f32)) -> Self {
        Self {
            x: p.0 as f64,
            y: p.1 as f64,
        }
    }
}

pub fn detect_regions(curves: &SketchCurves) -> Vec<Region> {
    match detect_regions_analytic(curves) {
        Ok(regions) => regions,
        Err(_) => detect_regions_legacy(curves),
    }
}

/// Build sketch faces from bounded analytic curves. Unsupported spline/NURBS
/// pairs are returned as typed results; the compatibility [`detect_regions`]
/// adapter may then use its historical sampled path without pretending the
/// curve was handled analytically.
pub fn detect_regions_analytic(curves: &SketchCurves) -> Result<Vec<Region>, AnalyticSketchError> {
    use openrcad::foundation::{Dir2d, Pnt2d};
    use openrcad::geom2d::{BSplineCurve2d, Circle2d, Curve2d, CurveSpan, GeomCurve2d, Line2d};

    let tolerance = sketch_linear_tolerance(curves);
    let mut spans = Vec::new();
    for (index, segment) in curves.segments.iter().enumerate() {
        // Corner fillets retain their historical display/containment chords
        // alongside one exact Arc record. The analytic arrangement consumes
        // only that Arc; admitting its chords too would create a thin stack of
        // duplicate faces between the curve and its approximation.
        if curves
            .arcs
            .iter()
            .any(|arc| segment_is_arc_display_chord(segment, arc))
        {
            continue;
        }
        let dx = f64::from(segment.b.0 - segment.a.0);
        let dy = f64::from(segment.b.1 - segment.a.1);
        let length = dx.hypot(dy);
        if length <= tolerance {
            continue;
        }
        let Some(direction) = Dir2d::try_new(dx / length, dy / length) else {
            continue;
        };
        spans.push(CurveSpan::new(
            GeomCurve2d::line(Line2d::from_point_dir(
                Pnt2d::new(f64::from(segment.a.0), f64::from(segment.a.1)),
                direction,
            )),
            0.0,
            length,
            SketchCurveProvenance {
                family: SketchCurveFamily::Line,
                index,
            },
        ));
    }
    for (index, circle) in curves.circles.iter().enumerate() {
        if circle.radius <= 0.0 {
            continue;
        }
        spans.push(CurveSpan::new(
            GeomCurve2d::circle(Circle2d::from_center(
                Pnt2d::new(f64::from(circle.center.0), f64::from(circle.center.1)),
                f64::from(circle.radius),
            )),
            0.0,
            std::f64::consts::TAU,
            SketchCurveProvenance {
                family: SketchCurveFamily::Circle,
                index,
            },
        ));
    }
    for (index, arc) in curves.arcs.iter().enumerate() {
        if arc.radius <= 0.0 {
            continue;
        }
        let (first, last) = arc.parameter_bounds();
        spans.push(CurveSpan::new(
            GeomCurve2d::circle(Circle2d::from_center(
                Pnt2d::new(f64::from(arc.center.0), f64::from(arc.center.1)),
                f64::from(arc.radius),
            )),
            first,
            last,
            SketchCurveProvenance {
                family: SketchCurveFamily::Arc,
                index,
            },
        ));
    }
    for (index, spline) in curves.splines.iter().enumerate() {
        if spline.kind != SplineKind::ControlPoint || spline.closed || spline.periodic {
            return Err(AnalyticSketchError::UnsupportedSpline {
                index,
                kind: spline.kind,
            });
        }
        let degree = usize::from(spline.degree.max(1));
        let flat_knots: Vec<f64> = if spline.knots.len() == spline.points.len() + degree + 1 {
            spline.knots.iter().map(|value| f64::from(*value)).collect()
        } else {
            clamped_uniform_knots(spline.points.len(), degree)
                .into_iter()
                .map(f64::from)
                .collect()
        };
        let (knots, multiplicities) = distinct_knots(&flat_knots);
        let poles = spline
            .points
            .iter()
            .map(|point| Pnt2d::new(f64::from(point.0), f64::from(point.1)))
            .collect();
        let weights = (!spline.weights.is_empty()).then(|| {
            spline
                .weights
                .iter()
                .map(|value| f64::from(*value))
                .collect()
        });
        let curve = BSplineCurve2d::new(degree, poles, weights, knots, multiplicities);
        let (first, last) = curve.bounds();
        spans.push(CurveSpan::new(
            GeomCurve2d::bspline(curve),
            first,
            last,
            SketchCurveProvenance {
                family: SketchCurveFamily::Spline,
                index,
            },
        ));
    }

    if spans.is_empty() {
        return Ok(Vec::new());
    }
    let arrangement = openrcad::sketch::arrange_curve_spans(
        &spans,
        openrcad::sketch::ArrangementOptions {
            // Persisted sketch coordinates are f32; leave enough room for an
            // analytic arc reconstructed from those values to rejoin its
            // trimmed line endpoint without treating a real modeling gap as
            // closed.
            tolerance,
            root_subdivisions: 256,
        },
    )
    .map_err(AnalyticSketchError::Arrangement)?;
    Ok(arrangement
        .regions
        .into_iter()
        .filter(|region| region.area > tolerance * tolerance)
        .map(|analytic| Region {
            boundary: sample_analytic_loop(&analytic.outer),
            holes: analytic.holes.iter().map(sample_analytic_loop).collect(),
            area: analytic.area as f32,
            analytic: Some(analytic),
        })
        .collect())
}

fn segment_is_arc_display_chord(segment: &LineSegment, arc: &Arc) -> bool {
    let radial_error =
        |point: (f32, f32)| (point.0 - arc.center.0).hypot(point.1 - arc.center.1) - arc.radius;
    if radial_error(segment.a).abs() > 1.0e-3 || radial_error(segment.b).abs() > 1.0e-3 {
        return false;
    }
    let chord_length = (segment.b.0 - segment.a.0).hypot(segment.b.1 - segment.a.1);
    let midpoint = (
        (segment.a.0 + segment.b.0) * 0.5,
        (segment.a.1 + segment.b.1) * 0.5,
    );
    // Fillet display chords are deliberately short. Requiring both a short
    // chord and a near-circular midpoint prevents a genuine user line whose
    // endpoints happen to lie on the same arc (for example a diameter) from
    // being mistaken for the arc's sampled representation.
    if chord_length > arc.radius * 0.35 || radial_error(midpoint).abs() > arc.radius * 0.02 {
        return false;
    }
    let angle = |point: (f32, f32)| {
        f64::from(point.1 - arc.center.1).atan2(f64::from(point.0 - arc.center.0))
    };
    let (first, last) = arc.parameter_bounds();
    let within = |point: (f32, f32)| {
        let base = angle(point);
        (-1..=1).any(|turn| {
            let parameter = base + f64::from(turn) * std::f64::consts::TAU;
            parameter >= first.min(last) - 1.0e-6 && parameter <= first.max(last) + 1.0e-6
        })
    };
    within(segment.a) && within(segment.b)
}

fn distinct_knots(flat: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut knots = Vec::new();
    let mut multiplicities = Vec::new();
    for &value in flat {
        if knots
            .last()
            .is_some_and(|last: &f64| (*last - value).abs() <= f64::EPSILON)
        {
            *multiplicities.last_mut().expect("knot multiplicity") += 1;
        } else {
            knots.push(value);
            multiplicities.push(1);
        }
    }
    (knots, multiplicities)
}

fn sample_analytic_loop(
    loop_: &openrcad::sketch::ArrangementLoop<SketchCurveProvenance>,
) -> Vec<(f32, f32)> {
    use openrcad::geom2d::{Curve2d, CurveKind2d};
    let mut points = Vec::new();
    for span in &loop_.spans {
        let steps = match span.kind() {
            CurveKind2d::Line => 1,
            CurveKind2d::Circle | CurveKind2d::Ellipse => {
                ((span.parameter_length() / std::f64::consts::TAU * CIRCLE_SEGS as f64).ceil()
                    as usize)
                    .max(1)
            }
            _ => 24,
        };
        for step in 0..steps {
            let parameter = span.first + (span.last - span.first) * step as f64 / steps as f64;
            let point = span.curve.point(parameter);
            points.push((point.x() as f32, point.y() as f32));
        }
    }
    points
}

fn detect_regions_legacy(curves: &SketchCurves) -> Vec<Region> {
    let raw_segs = flatten_curves(curves);
    if raw_segs.is_empty() {
        return Vec::new();
    }

    let mut vertices: Vec<P> = Vec::new();
    let split = split_at_intersections(&raw_segs, &mut vertices);
    if split.is_empty() {
        return Vec::new();
    }

    // Deduplicate undirected sub-segments — collinear/duplicate input would
    // otherwise yield bogus zero-area cycles. Key by sorted endpoint indices.
    let mut seen: HashMap<(usize, usize), ()> = HashMap::new();
    let mut unique: Vec<(usize, usize)> = Vec::new();
    for (u, v) in split {
        if u == v {
            continue;
        }
        let key = if u < v { (u, v) } else { (v, u) };
        if seen.insert(key, ()).is_none() {
            unique.push((u, v));
        }
    }

    let half_edges = build_half_edges(&unique);
    let next = compute_next_pointers(&half_edges, &vertices);
    let cycles = walk_cycles(&half_edges, &next);

    let area_tolerance = sketch_linear_tolerance(curves).powi(2);
    let mut regions: Vec<Region> = Vec::new();
    for cycle in cycles {
        let pts: Vec<(f32, f32)> = cycle
            .iter()
            .map(|&h| {
                let v = vertices[half_edges[h].from];
                (v.x as f32, v.y as f32)
            })
            .collect();
        let area = signed_area_f64(&cycle, &half_edges, &vertices);
        if area > area_tolerance {
            regions.push(Region {
                boundary: pts,
                holes: Vec::new(),
                area: area as f32,
                analytic: None,
            });
        }
    }

    assign_holes(&mut regions, area_tolerance);
    regions
}

/// Turn nesting into holes. When one face lies fully inside another (a shape
/// drawn inside another, with no intersecting edges), the inner face becomes a
/// hole of its immediate (smallest) container, and the container's area is
/// reduced accordingly. Every face is kept — so a circle with a rectangle drawn
/// inside it yields BOTH the inner rectangle face AND the annular face around it
/// (rather than the annulus vanishing).
///
/// Adjacent faces produced by intersecting shapes share edges, so neither
/// contains the other's interior point — they are never turned into holes.
fn assign_holes(regions: &mut [Region], area_tolerance: f64) {
    let n = regions.len();
    if n < 2 {
        return;
    }

    // A point guaranteed strictly inside each face's outer boundary.
    let interior_points: Vec<(f32, f32)> = regions
        .iter()
        .map(|r| polygon_interior_point(&r.boundary))
        .collect();
    // Gross outer-boundary areas (before hole subtraction).
    let gross: Vec<f64> = regions.iter().map(|r| r.area as f64).collect();

    // Immediate parent = smallest face that strictly contains this face.
    let mut parent: Vec<Option<usize>> = vec![None; n];
    for j in 0..n {
        let mut best: Option<usize> = None;
        for i in 0..n {
            if i == j {
                continue;
            }
            if gross[i] > gross[j] + area_tolerance
                && point_in_polygon(interior_points[j], &regions[i].boundary)
            {
                best = match best {
                    None => Some(i),
                    Some(b) if gross[i] < gross[b] => Some(i),
                    other => other,
                };
            }
        }
        parent[j] = best;
    }

    let boundaries: Vec<Vec<(f32, f32)>> = regions.iter().map(|r| r.boundary.clone()).collect();
    for j in 0..n {
        if let Some(p) = parent[j] {
            regions[p].holes.push(boundaries[j].clone());
            regions[p].area -= gross[j] as f32;
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Tolerance for persisted f32 sketch coordinates without a model-unit floor.
/// Scaling a sketch scales this tolerance and its corresponding area cutoff,
/// so small but well-conditioned profiles are not discarded merely because
/// their dimensions happen to be below one millimetre.
fn sketch_linear_tolerance(curves: &SketchCurves) -> f64 {
    let scale = flatten_curves(curves)
        .into_iter()
        .flat_map(|(a, b)| [a.x.abs(), a.y.abs(), b.x.abs(), b.y.abs()])
        .fold(0.0_f64, f64::max);
    (scale * f64::from(f32::EPSILON) * 8.0).max(f64::EPSILON * 64.0)
}

fn flatten_curves(curves: &SketchCurves) -> Vec<(P, P)> {
    let mut out: Vec<(P, P)> = Vec::new();
    for s in &curves.segments {
        let a = P::from(s.a);
        let b = P::from(s.b);
        if (a.x - b.x).abs() > VERTEX_TOL || (a.y - b.y).abs() > VERTEX_TOL {
            out.push((a, b));
        }
    }
    for c in &curves.circles {
        let cx = c.center.0 as f64;
        let cy = c.center.1 as f64;
        let r = c.radius as f64;
        let mut prev = P { x: cx + r, y: cy };
        for i in 1..=CIRCLE_SEGS {
            let theta = (i as f64 / CIRCLE_SEGS as f64) * std::f64::consts::TAU;
            let cur = P {
                x: cx + r * theta.cos(),
                y: cy + r * theta.sin(),
            };
            out.push((prev, cur));
            prev = cur;
        }
    }
    for arc in &curves.arcs {
        if curves
            .segments
            .iter()
            .any(|segment| segment_is_arc_display_chord(segment, arc))
        {
            continue;
        }
        let (first, last) = arc.parameter_bounds();
        let steps = arc_segments((last - first).abs() as f32, arc.radius);
        let mut previous = P::from(arc.start);
        for index in 1..=steps {
            let parameter = first + (last - first) * index as f64 / steps as f64;
            let current = P {
                x: f64::from(arc.center.0) + f64::from(arc.radius) * parameter.cos(),
                y: f64::from(arc.center.1) + f64::from(arc.radius) * parameter.sin(),
            };
            out.push((previous, current));
            previous = current;
        }
    }
    for spline in &curves.splines {
        for pair in spline.sampled_points(0.01).windows(2) {
            let a = P::from(pair[0]);
            let b = P::from(pair[1]);
            if (a.x - b.x).abs() > VERTEX_TOL || (a.y - b.y).abs() > VERTEX_TOL {
                out.push((a, b));
            }
        }
    }
    out
}

fn add_vertex(p: P, vertices: &mut Vec<P>) -> usize {
    for (i, q) in vertices.iter().enumerate() {
        if (p.x - q.x).abs() < VERTEX_TOL && (p.y - q.y).abs() < VERTEX_TOL {
            return i;
        }
    }
    vertices.push(p);
    vertices.len() - 1
}

/// For each input segment, compute all interior intersection points with
/// every other segment, sort them along the segment, and emit consecutive
/// sub-segments as index pairs into the vertex pool.
fn split_at_intersections(raw: &[(P, P)], vertices: &mut Vec<P>) -> Vec<(usize, usize)> {
    let mut out: Vec<(usize, usize)> = Vec::new();

    for (i, &(a, b)) in raw.iter().enumerate() {
        // (t-param-on-AB, vertex_index) for each split point including the endpoints.
        let mut pts: Vec<(f64, usize)> = Vec::new();
        pts.push((0.0, add_vertex(a, vertices)));
        pts.push((1.0, add_vertex(b, vertices)));

        for (j, &(c, d)) in raw.iter().enumerate() {
            if i == j {
                continue;
            }
            if let Some((t, _u, p)) = intersect(a, b, c, d) {
                if t > VERTEX_TOL && t < 1.0 - VERTEX_TOL {
                    let idx = add_vertex(p, vertices);
                    pts.push((t, idx));
                }
            }
        }

        pts.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
        // Emit consecutive pairs, skipping duplicate vertex indices.
        for w in pts.windows(2) {
            let (_, u) = w[0];
            let (_, v) = w[1];
            if u != v {
                out.push((u, v));
            }
        }
    }
    out
}

fn intersect(a: P, b: P, c: P, d: P) -> Option<(f64, f64, P)> {
    let rx = b.x - a.x;
    let ry = b.y - a.y;
    let sx = d.x - c.x;
    let sy = d.y - c.y;
    let denom = rx * sy - ry * sx;
    if denom.abs() < PARALLEL_EPS {
        return None;
    }
    let qpx = c.x - a.x;
    let qpy = c.y - a.y;
    let t = (qpx * sy - qpy * sx) / denom;
    let u = (qpx * ry - qpy * rx) / denom;
    let tol = 1e-7;
    if t >= -tol && t <= 1.0 + tol && u >= -tol && u <= 1.0 + tol {
        Some((
            t,
            u,
            P {
                x: a.x + t * rx,
                y: a.y + t * ry,
            },
        ))
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy)]
struct HEdge {
    from: usize,
    to: usize,
    twin: usize,
}

fn build_half_edges(undirected: &[(usize, usize)]) -> Vec<HEdge> {
    let mut he = Vec::with_capacity(undirected.len() * 2);
    for &(u, v) in undirected {
        let i = he.len();
        he.push(HEdge {
            from: u,
            to: v,
            twin: i + 1,
        });
        he.push(HEdge {
            from: v,
            to: u,
            twin: i,
        });
    }
    he
}

fn compute_next_pointers(half_edges: &[HEdge], vertices: &[P]) -> Vec<usize> {
    // outgoing[v] = sorted list of half-edge indices originating at v
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    for (i, h) in half_edges.iter().enumerate() {
        outgoing[h.from].push(i);
    }
    for v in 0..vertices.len() {
        let p = vertices[v];
        outgoing[v].sort_by(|&a, &b| {
            let pa = vertices[half_edges[a].to];
            let pb = vertices[half_edges[b].to];
            let ang_a = (pa.y - p.y).atan2(pa.x - p.x);
            let ang_b = (pb.y - p.y).atan2(pb.x - p.x);
            ang_a
                .partial_cmp(&ang_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    // pos_in_outgoing[h] = index of h within outgoing[h.from]
    let mut pos_in_outgoing = vec![0usize; half_edges.len()];
    for v in 0..vertices.len() {
        for (idx, &h) in outgoing[v].iter().enumerate() {
            pos_in_outgoing[h] = idx;
        }
    }

    let mut next = vec![0usize; half_edges.len()];
    for h in 0..half_edges.len() {
        let v_dest = half_edges[h].to;
        let twin = half_edges[h].twin;
        let i = pos_in_outgoing[twin];
        let k = outgoing[v_dest].len();
        let prev_i = (i + k - 1) % k;
        next[h] = outgoing[v_dest][prev_i];
    }
    next
}

fn walk_cycles(half_edges: &[HEdge], next: &[usize]) -> Vec<Vec<usize>> {
    let mut visited = vec![false; half_edges.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..half_edges.len() {
        if visited[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut cur = start;
        loop {
            if visited[cur] {
                break;
            }
            visited[cur] = true;
            cycle.push(cur);
            cur = next[cur];
            if cur == start {
                break;
            }
        }
        if !cycle.is_empty() {
            cycles.push(cycle);
        }
    }
    cycles
}

fn signed_area_f64(cycle: &[usize], he: &[HEdge], vertices: &[P]) -> f64 {
    let mut sum = 0.0;
    for &h in cycle {
        let p = vertices[he[h].from];
        let q = vertices[he[h].to];
        sum += p.x * q.y - q.x * p.y;
    }
    sum * 0.5
}

fn centroid(poly: &[(f32, f32)]) -> (f32, f32) {
    let (mut sx, mut sy) = (0.0f32, 0.0f32);
    for p in poly {
        sx += p.0;
        sy += p.1;
    }
    let n = poly.len() as f32;
    (sx / n, sy / n)
}

/// A point guaranteed to lie strictly inside a simple (possibly concave)
/// polygon. The vertex centroid can fall outside a concave polygon, so we find
/// an "ear" (a convex corner whose triangle contains no other vertex) and
/// return that triangle's centroid — which is always interior. Falls back to
/// the vertex centroid only for degenerate input.
pub(crate) fn polygon_interior_point(poly: &[(f32, f32)]) -> (f32, f32) {
    let n = poly.len();
    if n < 3 {
        return centroid(poly);
    }

    let cross = |o: (f32, f32), a: (f32, f32), b: (f32, f32)| -> f64 {
        (a.0 - o.0) as f64 * (b.1 - o.1) as f64 - (a.1 - o.1) as f64 * (b.0 - o.0) as f64
    };
    let in_tri = |p: (f32, f32), a: (f32, f32), b: (f32, f32), c: (f32, f32)| -> bool {
        let d1 = cross(a, b, p);
        let d2 = cross(b, c, p);
        let d3 = cross(c, a, p);
        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
        !(has_neg && has_pos)
    };

    // Work on a CCW copy so a convex corner has positive cross product.
    let mut order: Vec<usize> = (0..n).collect();
    let mut area2 = 0.0f64;
    for i in 0..n {
        let (ax, ay) = poly[i];
        let (bx, by) = poly[(i + 1) % n];
        area2 += ax as f64 * by as f64 - bx as f64 * ay as f64;
    }
    if area2 < 0.0 {
        order.reverse();
    }

    let m = order.len();
    for i in 0..m {
        let a = poly[order[(i + m - 1) % m]];
        let b = poly[order[i]];
        let c = poly[order[(i + 1) % m]];
        if cross(a, b, c) <= 0.0 {
            continue; // reflex (or collinear) corner — not an ear
        }
        let mut contains = false;
        for &k in &order {
            let pk = poly[k];
            if pk == a || pk == b || pk == c {
                continue;
            }
            if in_tri(pk, a, b, c) {
                contains = true;
                break;
            }
        }
        if contains {
            continue;
        }
        return ((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0);
    }

    centroid(poly)
}

pub fn point_in_polygon(p: (f32, f32), poly: &[(f32, f32)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        let intersect_y = (yi > p.1) != (yj > p.1)
            && (p.0) < (xj - xi) * (p.1 - yi) / ((yj - yi) + f32::EPSILON) + xi;
        if intersect_y {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ---------------------------------------------------------------------------
// Whole-shape recovery and overlap detection (boolean extrude)
// ---------------------------------------------------------------------------

/// Distance (mm) within which a point is considered to lie *on* a polygon edge
/// rather than strictly inside — keeps boundary-only-touching shapes from
/// registering as overlapping.
const BOUNDARY_TOL: f32 = 1e-3;

/// The full closed outline of one drawn sketch shape (rectangle, circle, …),
/// recovered before region-splitting so overlapping shapes can be combined as a
/// boolean. `circle` is set for true circles so the kernel can keep a smooth
/// analytic cylinder instead of a faceted prism.
#[derive(Debug, Clone, PartialEq)]
pub struct ShapeLoop {
    /// Closed boundary polygon in sketch plane coordinates (circles are
    /// discretized to [`CIRCLE_SEGS`] points so all overlap tests are uniform).
    pub boundary: Vec<(f32, f32)>,
    /// `Some((center, radius))` when this loop is a true circle.
    pub circle: Option<((f32, f32), f32)>,
}

/// Recover each drawn shape's full closed outline (before region-splitting).
/// Circles become smooth-flagged loops; rectangles / closed polylines (ellipses,
/// 3-point shapes) are chained into one loop. Open profiles (lone lines) and any
/// shape that does not close are skipped — they form no region and take no part
/// in booleans.
pub fn shape_loops(shapes: &[SketchShape], vars: &HashMap<String, f64>) -> Vec<ShapeLoop> {
    let mut out = Vec::new();
    for shape in shapes {
        // Text is a compound non-zero-winding shape with many independent
        // contours. `segments_to_loop` can only recover one closed outline;
        // feeding the first line-only glyph contour into the shape-boolean
        // planner misclassifies the rest of the word as ordinary rectangle
        // material. Typography is classified separately by
        // `sketch_region_ink_mask`, so it must not masquerade as one primitive
        // boolean loop here.
        if matches!(shape, SketchShape::Text { .. }) {
            continue;
        }
        let curves = shape.build(vars);
        for c in &curves.circles {
            if c.radius > 0.0 {
                out.push(ShapeLoop {
                    boundary: circle_boundary(c.center, c.radius),
                    circle: Some((c.center, c.radius)),
                });
            }
        }
        if let Some(boundary) = segments_to_loop(&curves) {
            out.push(ShapeLoop {
                boundary,
                circle: None,
            });
        }
    }
    out
}

fn circle_boundary(center: (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    (0..CIRCLE_SEGS)
        .map(|i| {
            let t = (i as f32 / CIRCLE_SEGS as f32) * std::f32::consts::TAU;
            (center.0 + radius * t.cos(), center.1 + radius * t.sin())
        })
        .collect()
}

/// Chain a shape's line segments into a single closed loop by matching shared
/// endpoints. Returns `None` if the segments don't form one closed ring.
fn segments_to_loop(curves: &SketchCurves) -> Option<Vec<(f32, f32)>> {
    let segs = &curves.segments;
    if segs.len() < 3 {
        return None;
    }
    let tol = 1e-4f32;
    let close = |a: (f32, f32), b: (f32, f32)| (a.0 - b.0).abs() <= tol && (a.1 - b.1).abs() <= tol;

    let mut used = vec![false; segs.len()];
    let mut loop_pts: Vec<(f32, f32)> = Vec::with_capacity(segs.len());
    used[0] = true;
    loop_pts.push(segs[0].a);
    let start = segs[0].a;
    let mut current = segs[0].b;

    while !close(current, start) {
        let mut found = false;
        for i in 0..segs.len() {
            if used[i] {
                continue;
            }
            if close(segs[i].a, current) {
                used[i] = true;
                loop_pts.push(segs[i].a);
                current = segs[i].b;
                found = true;
                break;
            } else if close(segs[i].b, current) {
                used[i] = true;
                loop_pts.push(segs[i].b);
                current = segs[i].a;
                found = true;
                break;
            }
        }
        if !found {
            return None;
        }
    }
    if loop_pts.len() < 3 {
        return None;
    }
    Some(loop_pts)
}

/// True if segments `a0a1` and `b0b1` cross at an interior point of *both*
/// (shared endpoints of one closed loop don't count as crossings).
pub fn segments_cross_2d(a0: (f32, f32), a1: (f32, f32), b0: (f32, f32), b1: (f32, f32)) -> bool {
    if let Some((t, u, _)) = intersect(P::from(a0), P::from(a1), P::from(b0), P::from(b1)) {
        let e = 1e-6;
        t > e && t < 1.0 - e && u > e && u < 1.0 - e
    } else {
        false
    }
}

fn midpoint(a: (f32, f32), b: (f32, f32)) -> (f32, f32) {
    ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5)
}

fn seg_point_dist(p: (f32, f32), a: (f32, f32), b: (f32, f32)) -> f32 {
    let (vx, vy) = (b.0 - a.0, b.1 - a.1);
    let (wx, wy) = (p.0 - a.0, p.1 - a.1);
    let len2 = vx * vx + vy * vy;
    if len2 <= 1e-12 {
        return (wx * wx + wy * wy).sqrt();
    }
    let t = ((wx * vx + wy * vy) / len2).clamp(0.0, 1.0);
    let (cx, cy) = (a.0 + t * vx, a.1 + t * vy);
    ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt()
}

fn point_on_boundary(p: (f32, f32), poly: &[(f32, f32)], tol: f32) -> bool {
    let n = poly.len();
    (0..n).any(|i| seg_point_dist(p, poly[i], poly[(i + 1) % n]) <= tol)
}

/// Inside the polygon and not within [`BOUNDARY_TOL`] of its boundary.
fn point_strictly_inside(p: (f32, f32), poly: &[(f32, f32)]) -> bool {
    point_in_polygon(p, poly) && !point_on_boundary(p, poly, BOUNDARY_TOL)
}

/// True if two shape outlines overlap by area. Covers three cases: a proper edge
/// crossing; one loop's vertex strictly inside the other (fully-contained
/// shapes); or one loop's edge midpoint strictly inside the other (edge-aligned
/// rectangles that share a span — collinear edges never "cross"). Boundary-only
/// touching is *not* an overlap.
pub fn shapes_overlap(a: &ShapeLoop, b: &ShapeLoop) -> bool {
    let (pa, pb) = (&a.boundary, &b.boundary);
    let (na, nb) = (pa.len(), pb.len());
    if na < 3 || nb < 3 {
        return false;
    }
    for i in 0..na {
        let (a0, a1) = (pa[i], pa[(i + 1) % na]);
        for j in 0..nb {
            if segments_cross_2d(a0, a1, pb[j], pb[(j + 1) % nb]) {
                return true;
            }
        }
    }
    if pa.iter().any(|&v| point_strictly_inside(v, pb)) {
        return true;
    }
    if pb.iter().any(|&v| point_strictly_inside(v, pa)) {
        return true;
    }
    if (0..na).any(|i| point_strictly_inside(midpoint(pa[i], pa[(i + 1) % na]), pb)) {
        return true;
    }
    if (0..nb).any(|j| point_strictly_inside(midpoint(pb[j], pb[(j + 1) % nb]), pa)) {
        return true;
    }
    false
}

/// True when two overlapping loops are in a *pure containment* relationship —
/// one lies entirely inside the other with no boundary crossing. That makes the
/// inner loop a HOLE (a circle drawn inside a rectangle), which
/// [`detect_regions`]/`assign_holes` already turns into a hole; it needs no
/// boolean split. Assumes the two loops overlap.
fn loops_purely_contained(a: &ShapeLoop, b: &ShapeLoop) -> bool {
    let (pa, pb) = (&a.boundary, &b.boundary);
    let (na, nb) = (pa.len(), pb.len());
    if na < 3 || nb < 3 {
        return false;
    }
    // Any proper edge crossing means the boundaries interpenetrate — a genuine
    // partial overlap, not containment.
    for i in 0..na {
        let (a0, a1) = (pa[i], pa[(i + 1) % na]);
        for j in 0..nb {
            if segments_cross_2d(a0, a1, pb[j], pb[(j + 1) % nb]) {
                return false;
            }
        }
    }
    // No crossings: it is containment iff every vertex of one loop lies inside
    // the other (so the whole loop is nested, not merely edge-touching).
    let b_in_a = pb.iter().all(|&v| point_in_polygon(v, pa));
    let a_in_b = pa.iter().all(|&v| point_in_polygon(v, pb));
    b_in_a || a_in_b
}

/// True when two shape outlines genuinely interpenetrate and so require a
/// boolean split/fusion to resolve (an edge crossing, or a partial/edge-aligned
/// overlap). This is [`shapes_overlap`] minus the pure-containment case: a loop
/// fully nested inside another is a hole, which `detect_regions` handles
/// directly and the instant extrude ghost can render exactly — no boolean worker
/// needed. Used by the live extrude preview to decide when the fast hole-aware
/// ghost suffices versus when the evaluated boolean result must be shown.
pub fn shapes_cross(a: &ShapeLoop, b: &ShapeLoop) -> bool {
    shapes_overlap(a, b) && !loops_purely_contained(a, b)
}

/// Group shape loops into connected components by the overlap relation. A
/// singleton cluster is a non-overlapping shape (extrudes independently); a
/// cluster of ≥2 becomes one boolean solid.
pub fn overlap_clusters(loops: &[ShapeLoop]) -> Vec<Vec<usize>> {
    let n = loops.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while parent[r] != r {
            r = parent[r];
        }
        let mut c = x;
        while parent[c] != c {
            let nx = parent[c];
            parent[c] = r;
            c = nx;
        }
        r
    }
    for i in 0..n {
        for j in (i + 1)..n {
            if shapes_overlap(&loops[i], &loops[j]) {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }
    }
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n {
        let r = find(&mut parent, i);
        groups.entry(r).or_default().push(i);
    }
    let mut clusters: Vec<Vec<usize>> = groups.into_values().collect();
    for c in &mut clusters {
        c.sort_unstable();
    }
    clusters.sort_by_key(|c| c[0]);
    clusters
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn single_rectangle_one_region() {
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 8.0));
        let regions = detect_regions(&c);
        assert_eq!(
            regions.len(),
            1,
            "single rect should give one region, got {:?}",
            regions
        );
        assert!(approx(regions[0].area, 80.0, 0.1));
    }

    #[test]
    fn native_slot_is_two_tangent_sides_and_two_analytic_caps() {
        let slot = SketchShape::Slot {
            start: (0.0, 0.0),
            end: (20.0, 0.0),
            width: Dimension::literal(6.0),
        };
        let curves = slot.build(&HashMap::new());
        assert_eq!(curves.segments.len(), 2);
        assert_eq!(curves.arcs.len(), 2);
        assert!(curves.circles.is_empty());
        assert_eq!(curves.segments[0].b, curves.arcs[0].start);
        assert_eq!(curves.arcs[0].end, curves.segments[1].a);
        assert_eq!(curves.segments[1].b, curves.arcs[1].start);
        assert_eq!(curves.arcs[1].end, curves.segments[0].a);

        let regions = detect_regions_analytic(&curves).expect("slot must arrange analytically");
        assert_eq!(regions.len(), 1);
        let expected_area = 20.0 * 6.0 + std::f32::consts::PI * 3.0 * 3.0;
        assert!(
            approx(regions[0].area, expected_area, 0.05),
            "slot area was {}, expected {expected_area}",
            regions[0].area
        );
    }

    #[test]
    fn zero_length_slot_becomes_circle_and_nonpositive_width_is_rejected() {
        let circle = SketchShape::Slot {
            start: (4.0, -3.0),
            end: (4.0, -3.0),
            width: Dimension::literal(8.0),
        }
        .build(&HashMap::new());
        assert!(circle.segments.is_empty());
        assert!(circle.arcs.is_empty());
        assert_eq!(circle.circles.len(), 1);
        assert_eq!(circle.circles[0].center, (4.0, -3.0));
        assert_eq!(circle.circles[0].radius, 4.0);

        for width in [0.0, -1.0, f32::NAN] {
            let rejected = SketchShape::Slot {
                start: (0.0, 0.0),
                end: (10.0, 0.0),
                width: Dimension::literal(width),
            }
            .build(&HashMap::new());
            assert!(rejected.is_empty(), "width {width:?} must be rejected");
        }
    }

    #[test]
    fn slot_provenance_and_shape_identity_survive_round_trip() {
        let shape = SketchShape::Slot {
            start: (1.0, 2.0),
            end: (9.0, 5.0),
            width: Dimension {
                value: 4.0,
                expr: Some("slot_width".to_string()),
            },
        };
        let encoded = serde_json::to_vec(&shape).unwrap();
        let decoded: SketchShape = serde_json::from_slice(&encoded).unwrap();
        assert_eq!(decoded, shape);

        let curves = shape.build(&HashMap::from([("slot_width".to_string(), 6.0)]));
        let regions = detect_regions(&curves);
        let provenance = build_region_provenance(&curves, &[shape], &[EntityId(42)], &regions);
        let boundaries: Vec<SlotBoundary> = provenance[0]
            .fragments
            .iter()
            .filter_map(|fragment| match fragment {
                RegionProvenanceFragment::Slot {
                    shape_id: Some(42),
                    boundary,
                } => Some(*boundary),
                _ => None,
            })
            .collect();
        assert_eq!(
            boundaries,
            vec![
                SlotBoundary::LeftSide,
                SlotBoundary::EndCap,
                SlotBoundary::RightSide,
                SlotBoundary::StartCap,
            ]
        );

        let (model, _) = constraints::promote_shapes_to_entities(
            &[decoded],
            &[EntityId(42)],
            &HashMap::from([("slot_width".to_string(), 6.0)]),
            43,
        );
        assert_eq!(model.entities.len(), 4);
        assert!(model
            .entities
            .iter()
            .all(|entity| entity.derived_from() == Some(EntityId(42))));
    }

    #[test]
    fn regular_polygon_uses_expression_diameter_without_emitting_guide_circle() {
        let polygon = SketchShape::RegularPolygon {
            center: (2.0, 3.0),
            sides: 6,
            diameter: Dimension {
                value: 1.0,
                expr: Some("43/2".to_string()),
            },
            rotation_deg: 0.0,
            circumscribed: false,
        };
        let curves = polygon.build(&HashMap::new());
        assert_eq!(curves.segments.len(), 6);
        assert!(
            curves.circles.is_empty(),
            "guide circle must remain construction-only"
        );
        let first = curves.segments[0].a;
        assert!(approx(first.0, 12.75, 1.0e-4));
        assert!(approx(first.1, 3.0, 1.0e-4));
        assert_eq!(detect_regions(&curves).len(), 1);
    }

    #[test]
    fn circumscribed_polygon_edges_are_tangent_to_guide_circle() {
        let polygon = SketchShape::RegularPolygon {
            center: (0.0, 0.0),
            sides: 4,
            diameter: Dimension::literal(20.0),
            rotation_deg: 0.0,
            circumscribed: true,
        };
        let curves = polygon.build(&HashMap::new());
        assert_eq!(curves.segments.len(), 4);
        assert!(curves.circles.is_empty());
        let edge = curves.segments[0];
        let midpoint = ((edge.a.0 + edge.b.0) * 0.5, (edge.a.1 + edge.b.1) * 0.5);
        assert!(approx(midpoint.0.hypot(midpoint.1), 10.0, 1.0e-4));
    }

    #[test]
    fn axis_aligned_ellipse_is_one_region_with_right_area() {
        // A faceted ellipse (rx=10, ry=5) should close into one region whose
        // area is close to π·rx·ry (a 48-gon slightly under-estimates it).
        let mut c = SketchCurves::new();
        c.add_ellipse((0.0, 0.0), (10.0, 0.0), 5.0);
        assert_eq!(c.segments.len(), 48, "ellipse should emit 48 facets");
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 1, "ellipse should be one closed region");
        let expected = std::f32::consts::PI * 10.0 * 5.0;
        assert!(
            (regions[0].area - expected).abs() < expected * 0.02,
            "ellipse area {} should be within 2% of {}",
            regions[0].area,
            expected
        );
    }

    #[test]
    fn rotated_ellipse_closes_into_one_region() {
        // Major axis at 45°, so the polyline is genuinely rotated.
        let mut c = SketchCurves::new();
        c.add_ellipse((3.0, 3.0), (7.07, 7.07), 4.0);
        let regions = detect_regions(&c);
        assert_eq!(
            regions.len(),
            1,
            "rotated ellipse should still be one region"
        );
    }

    #[test]
    fn fillet_and_chamfer_round_a_square_corner() {
        use std::collections::HashMap;
        let vars: HashMap<String, f64> = HashMap::new();
        let square = vec![SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(10.0),
            h: Dimension::literal(10.0),
            from_center: false,
        }];

        // Baseline: sharp square, area 100, one region.
        let base = effective_curves(&SketchCurves::new(), &square, &[], &vars);
        let base_area = detect_regions(&base)[0].area;
        assert!(approx(base_area, 100.0, 0.1));

        // Fillet the (0,0) corner with r=2 — one region, slightly less area.
        let fillet = CornerMod {
            at: (0.0, 0.0),
            radius: Dimension::literal(2.0),
            kind: CornerKind::Fillet,
        };
        let filleted = effective_curves(&SketchCurves::new(), &square, &[fillet], &vars);
        let regions = detect_regions(&filleted);
        assert_eq!(regions.len(), 1, "filleted square is still one region");
        let fa = regions[0].area;
        assert!(
            fa < base_area && fa > base_area - 2.0,
            "fillet trims a small bite: base={base_area} filleted={fa}"
        );

        // Chamfer removes a 45° triangle of area r²/2·tan(45)=2 at a right angle.
        let chamfer = CornerMod {
            at: (0.0, 0.0),
            radius: Dimension::literal(2.0),
            kind: CornerKind::Chamfer,
        };
        let chamfered = effective_curves(&SketchCurves::new(), &square, &[chamfer], &vars);
        let cregions = detect_regions(&chamfered);
        assert_eq!(cregions.len(), 1, "chamfered square is still one region");
        assert!(cregions[0].area < base_area);
    }

    #[test]
    fn associative_mirror_reflects_and_follows_source() {
        let vars = HashMap::new();
        // A 4×2 rectangle whose left edge is at x=4 (fully on +x side).
        let rect = vec![SketchShape::Rectangle {
            origin: (4.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(4.0),
            h: Dimension::literal(2.0),
            from_center: false,
        }];
        // Mirror across the Y axis (x = 0).
        let mirror = SketchMirror {
            a: (0.0, 0.0),
            b: (0.0, 1.0),
        };
        let out = effective_curves_solved(
            &SketchCurves::new(),
            &rect,
            &[],
            std::slice::from_ref(&mirror),
            None,
            &vars,
        );
        // Two disjoint regions (source + reflected copy), equal total area.
        let regions = detect_regions(&out);
        assert_eq!(regions.len(), 2, "mirror yields source + reflected region");
        let total: f32 = regions.iter().map(|r| r.area).sum();
        assert!(approx(total, 16.0, 0.1), "both 4×2 rects: {total}");
        // Associativity: widening the source (via a fresh build) moves the mirror
        // too — the reflected copy is re-derived, never stale.
        let wider = vec![SketchShape::Rectangle {
            origin: (4.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(6.0),
            h: Dimension::literal(2.0),
            from_center: false,
        }];
        let out2 = effective_curves_solved(
            &SketchCurves::new(),
            &wider,
            &[],
            std::slice::from_ref(&mirror),
            None,
            &vars,
        );
        let total2: f32 = detect_regions(&out2).iter().map(|r| r.area).sum();
        assert!(
            approx(total2, 24.0, 0.1),
            "widened source + mirror: {total2}"
        );
        // The reflected copy must sit on the −x side (min x is negative).
        let min_x = out2
            .segments
            .iter()
            .flat_map(|s| [s.a.0, s.b.0])
            .fold(f32::INFINITY, f32::min);
        assert!(min_x < -0.5, "reflected copy lands on the −x side: {min_x}");
    }

    #[test]
    fn degenerate_ellipse_adds_nothing() {
        let mut c = SketchCurves::new();
        c.add_ellipse((0.0, 0.0), (0.0, 0.0), 5.0); // zero major axis
        c.add_ellipse((0.0, 0.0), (10.0, 0.0), 0.0); // zero minor radius
        assert!(c.segments.is_empty(), "degenerate ellipses must be skipped");
    }

    #[test]
    fn two_disjoint_rectangles_two_regions() {
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_rectangle((20.0, 0.0), (30.0, 10.0));
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 2);
        let total: f32 = regions.iter().map(|r| r.area).sum();
        assert!(approx(total, 200.0, 0.1));
    }

    #[test]
    fn overlapping_rectangles_split_into_three_regions() {
        // Two unit squares overlapping in a 5x10 strip → 3 regions:
        // left-only, overlap, right-only.
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_rectangle((5.0, 0.0), (15.0, 10.0));
        let regions = detect_regions(&c);
        assert_eq!(
            regions.len(),
            3,
            "expected 3 sub-regions, got {:?}",
            regions
        );
        // Each sub-region should be 50 area; total = 150
        let mut areas: Vec<f32> = regions.iter().map(|r| r.area).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for a in &areas {
            assert!(approx(*a, 50.0, 0.5), "region area {} not ~50", a);
        }
    }

    #[test]
    fn circle_alone_one_region() {
        let mut c = SketchCurves::new();
        c.add_circle((0.0, 0.0), 5.0);
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 1);
        // Polygon approximation of πr² ≈ 78.5
        assert!(regions[0].area > 70.0 && regions[0].area < 80.0);
    }

    #[test]
    fn circle_inside_rectangle_makes_annulus_and_inner_face() {
        // No edge intersections → two disjoint bounded cycles. With hole support
        // we keep BOTH faces: the inner circle, and the rectangle-with-a-circular
        // hole (the annulus) around it — instead of the outer face vanishing.
        let mut c = SketchCurves::new();
        c.add_rectangle((-10.0, -10.0), (10.0, 10.0)); // area 400
        c.add_circle((0.0, 0.0), 5.0); // area ≈ 78.5
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 2, "nested rect+circle gave: {:?}", regions);

        let annulus = regions
            .iter()
            .find(|r| !r.holes.is_empty())
            .expect("expected one face with a hole");
        let inner = regions
            .iter()
            .find(|r| r.holes.is_empty())
            .expect("expected one face without holes");

        assert_eq!(annulus.holes.len(), 1);
        assert!(
            approx(annulus.area, 400.0 - 78.5, 6.0),
            "annulus net area {} should be ~321.5",
            annulus.area
        );
        assert!(
            inner.area > 70.0 && inner.area < 82.0,
            "inner circle area {} should be ~78.5",
            inner.area
        );

        // The inner circle's centre is in the inner face, not the annulus.
        assert!(inner.contains((0.0, 0.0)));
        assert!(!annulus.contains((0.0, 0.0)));
        // A point near the rectangle corner is in the annulus, not the inner.
        assert!(annulus.contains((9.0, 9.0)));
        assert!(!inner.contains((9.0, 9.0)));
    }

    #[test]
    fn interior_point_is_inside_concave_polygon() {
        // A chevron whose vertex centroid falls OUTSIDE the polygon (in the
        // notch). This is exactly the situation that made the shell filter drop
        // a valid neighbouring face. `polygon_interior_point` must return a
        // point that is actually inside.
        let poly = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 1.0), (0.0, 4.0)];
        let vc = centroid(&poly);
        assert!(
            !point_in_polygon(vc, &poly),
            "test premise: vertex centroid {:?} should be outside the chevron",
            vc
        );
        let ip = polygon_interior_point(&poly);
        assert!(
            point_in_polygon(ip, &poly),
            "interior point {:?} must be inside the chevron",
            ip
        );
    }

    #[test]
    fn circle_overlapping_two_rectangles_keeps_all_faces() {
        // Reproduces the reported case: two overlapping rectangles plus a circle
        // crossing both. The detected faces tile the union of the shapes, so the
        // sum of their areas must equal the union's area. A missing face (the
        // reported bug) would make the total fall short.
        let mut c = SketchCurves::new();
        c.add_rectangle((-6.0, -2.0), (4.0, 10.0));
        c.add_rectangle((0.0, -8.0), (12.0, 4.0));
        c.add_circle((2.0, 0.0), 6.0);
        let regions = detect_regions(&c);
        assert!(!regions.is_empty());

        let in_union = |x: f32, y: f32| -> bool {
            let in_r1 = x >= -6.0 && x <= 4.0 && y >= -2.0 && y <= 10.0;
            let in_r2 = x >= 0.0 && x <= 12.0 && y >= -8.0 && y <= 4.0;
            let in_c = (x - 2.0).powi(2) + (y - 0.0).powi(2) <= 6.0 * 6.0;
            in_r1 || in_r2 || in_c
        };

        // Monte-Carlo-on-a-grid estimate of the union area.
        let cell = 0.1f32;
        let mut area_union = 0.0f32;
        let mut x = -8.0;
        while x <= 14.0 {
            let mut y = -10.0;
            while y <= 12.0 {
                if in_union(x, y) {
                    area_union += cell * cell;
                }
                y += cell;
            }
            x += cell;
        }

        let area_regions: f32 = regions.iter().map(|r| r.area).sum();
        let tol = 0.05 * area_union; // grid + circle-polygon discretisation slack
        assert!(
            (area_regions - area_union).abs() < tol,
            "regions should tile the union: sum(region areas)={:.1} vs union≈{:.1} (tol {:.1}); \
             a shortfall means a face is missing",
            area_regions,
            area_union,
            tol,
        );
    }

    #[test]
    fn circle_crossing_rectangle_edge_creates_multiple_regions() {
        // A circle straddling one edge of a rectangle DOES intersect that
        // edge, so the planar arrangement now connects the two shapes and
        // produces the full set of sub-regions (no nesting collapse).
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_circle((10.0, 5.0), 3.0);
        let regions = detect_regions(&c);
        assert!(
            regions.len() >= 2,
            "circle straddling rect edge should split into multiple regions, got {:?}",
            regions
        );
    }

    // -- Whole-shape recovery + overlap detection (boolean extrude) ----------

    fn rect_shape(x0: f32, y0: f32, x1: f32, y1: f32) -> SketchShape {
        SketchShape::Rectangle {
            origin: (x0, y0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(x1 - x0),
            h: Dimension::literal(y1 - y0),
            from_center: false,
        }
    }

    fn circle_shape(cx: f32, cy: f32, r: f32) -> SketchShape {
        SketchShape::Circle {
            center: (cx, cy),
            diameter: Dimension::literal(r * 2.0),
        }
    }

    #[test]
    fn shape_loops_recovers_full_outlines() {
        let vars: HashMap<String, f64> = HashMap::new();
        let shapes = vec![
            rect_shape(0.0, 0.0, 10.0, 10.0),
            circle_shape(8.0, 5.0, 4.0),
        ];
        let loops = shape_loops(&shapes, &vars);
        assert_eq!(loops.len(), 2);
        // Rectangle loop is the 4-corner polygon, no circle flag.
        assert!(loops[0].circle.is_none());
        assert_eq!(loops[0].boundary.len(), 4);
        // Circle loop keeps the analytic flag for smooth cylinders.
        assert_eq!(loops[1].circle, Some(((8.0, 5.0), 4.0)));
        assert_eq!(loops[1].boundary.len(), CIRCLE_SEGS);
    }

    #[test]
    fn overlap_partial_circle_and_rect() {
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 10.0, 10.0),
                circle_shape(10.0, 5.0, 4.0),
            ],
            &vars,
        );
        assert!(shapes_overlap(&loops[0], &loops[1]));
        let clusters = overlap_clusters(&loops);
        assert_eq!(clusters, vec![vec![0, 1]], "one cluster of two");
    }

    #[test]
    fn disjoint_shapes_are_singletons() {
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 10.0, 10.0),
                circle_shape(30.0, 5.0, 4.0),
            ],
            &vars,
        );
        assert!(!shapes_overlap(&loops[0], &loops[1]));
        assert_eq!(overlap_clusters(&loops), vec![vec![0], vec![1]]);
    }

    #[test]
    fn circle_fully_inside_rect_overlaps_via_containment() {
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(-10.0, -10.0, 10.0, 10.0),
                circle_shape(0.0, 0.0, 4.0),
            ],
            &vars,
        );
        assert!(
            shapes_overlap(&loops[0], &loops[1]),
            "fully-contained circle must register as overlap (→ cut)"
        );
    }

    #[test]
    fn contained_circle_overlaps_but_does_not_cross() {
        // A circle fully inside a rectangle is a HOLE, not a boolean split: it
        // overlaps (so it clusters and commits as a hole) but does NOT cross, so
        // the live extrude ghost may render it directly (prism-with-hole) instead
        // of waiting on the boolean worker.
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(-10.0, -10.0, 10.0, 10.0),
                circle_shape(0.0, 0.0, 4.0),
            ],
            &vars,
        );
        assert!(shapes_overlap(&loops[0], &loops[1]));
        assert!(
            !shapes_cross(&loops[0], &loops[1]),
            "a fully-contained circle is a hole, not a crossing boolean"
        );
        // Containment still clusters (so the commit drops the tool-lens disc).
        assert_eq!(overlap_clusters(&loops), vec![vec![0, 1]]);
    }

    #[test]
    fn straddling_circle_crosses() {
        // A circle straddling a rectangle edge genuinely interpenetrates — this
        // must still route through the boolean worker (shapes_cross = true).
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 10.0, 10.0),
                circle_shape(10.0, 5.0, 4.0),
            ],
            &vars,
        );
        assert!(shapes_cross(&loops[0], &loops[1]));
    }

    #[test]
    fn edge_aligned_rects_cross() {
        // Horizontally overlapping rectangles (partial overlap, no containment)
        // are a genuine boolean — they cross.
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 2.0, 2.0),
                rect_shape(1.0, 0.0, 3.0, 2.0),
            ],
            &vars,
        );
        assert!(shapes_cross(&loops[0], &loops[1]));
    }

    #[test]
    fn disjoint_shapes_do_not_cross() {
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 10.0, 10.0),
                circle_shape(30.0, 5.0, 4.0),
            ],
            &vars,
        );
        assert!(!shapes_cross(&loops[0], &loops[1]));
    }

    #[test]
    fn edge_aligned_rects_overlap_via_midpoint() {
        // Same vertical span, horizontally overlapping: no proper crossing and no
        // strictly-interior vertex — only the edge-midpoint test catches it.
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 2.0, 2.0),
                rect_shape(1.0, 0.0, 3.0, 2.0),
            ],
            &vars,
        );
        assert!(shapes_overlap(&loops[0], &loops[1]));
    }

    #[test]
    fn boundary_only_touching_is_not_overlap() {
        // Shared edge x=2, no area overlap → must stay independent.
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 2.0, 2.0),
                rect_shape(2.0, 0.0, 4.0, 2.0),
            ],
            &vars,
        );
        assert!(!shapes_overlap(&loops[0], &loops[1]));
        assert_eq!(overlap_clusters(&loops), vec![vec![0], vec![1]]);
    }

    #[test]
    fn three_chained_shapes_one_cluster() {
        let vars: HashMap<String, f64> = HashMap::new();
        let loops = shape_loops(
            &[
                rect_shape(0.0, 0.0, 4.0, 4.0),
                rect_shape(3.0, 0.0, 7.0, 4.0),
                rect_shape(6.0, 0.0, 10.0, 4.0),
            ],
            &vars,
        );
        assert_eq!(overlap_clusters(&loops), vec![vec![0, 1, 2]]);
    }

    #[test]
    fn control_point_spline_preserves_endpoints_and_trim() {
        let mut spline = Spline::control_points(
            vec![(0.0, 0.0), (3.0, 5.0), (7.0, -2.0), (10.0, 0.0)],
            3,
            false,
        );
        let full = spline.sampled_points(0.01);
        assert_eq!(full.first().copied(), Some((0.0, 0.0)));
        assert_eq!(full.last().copied(), Some((10.0, 0.0)));
        spline.trim = Some((0.25, 0.75));
        let trimmed = spline.sampled_points(0.01);
        assert_ne!(trimmed.first(), full.first());
        assert_ne!(trimmed.last(), full.last());
    }

    #[test]
    fn closed_fit_spline_participates_in_region_detection() {
        let mut curves = SketchCurves::new();
        curves.add_spline(Spline::fit_points(
            vec![(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0)],
            true,
            SplineContinuity::Tangent,
        ));
        let regions = detect_regions(&curves);
        assert_eq!(regions.len(), 1);
        assert!(regions[0].area > 90.0);
    }

    #[test]
    fn bugcase1_near_tangent_duplicate_has_no_tolerance_sliver() {
        let mut curves = SketchCurves::new();
        for (a, b) in [
            ((-13.2, -13.4), (8.2, -13.4)),
            ((8.2, -13.4), (8.2, -6.8999996)),
            ((8.2, -6.8999996), (-13.2, -6.8999996)),
            ((-13.2, -6.8999996), (-13.2, -13.4)),
            ((8.2, -6.8999996), (-10.2, -6.900001)),
            ((-10.2, -6.900001), (-10.2, -6.500001)),
            ((-10.2, -6.900001), (8.2, -6.900001)),
            ((8.2, -6.900001), (8.2, -6.100001)),
            ((8.2, -6.100001), (-10.2, -6.100001)),
            ((-10.2, -6.100001), (-10.2, -6.900001)),
            ((8.2, -6.100001), (-10.3, -6.100001)),
            ((-10.3, -6.100001), (-10.3, -2.9000008)),
            ((-10.3, -2.9000008), (8.2, -2.9000008)),
            ((8.2, -2.9000008), (8.2, -6.100001)),
        ] {
            curves.add_line(a, b);
        }
        curves.add_circle((-10.2, -6.500001), 0.4);

        let regions = detect_regions(&curves);
        let areas: Vec<f32> = regions.iter().map(|region| region.area).collect();
        assert_eq!(
            regions.len(),
            5,
            "the tolerance sliver must be removed: {areas:?}"
        );
        assert!(
            areas.iter().all(|area| *area > 0.1),
            "no microscopic region may survive: {areas:?}"
        );
    }
}
