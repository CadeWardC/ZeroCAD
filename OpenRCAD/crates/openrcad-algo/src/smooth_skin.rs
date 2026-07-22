//! Exact rational Smooth Loft skinning.
//!
//! Each input loop is a sequence of bounded rational B-spline spans carrying
//! stable caller provenance. Corresponding spans are decomposed at the union
//! of their normalized knots, degree-elevated exactly in homogeneous space,
//! and interpolated across section parameters by a clamped B-spline. Accepted
//! output therefore contains the input section curves exactly. Accepted skins
//! also carry a control-net separation certificate: unresolved overlap is a
//! typed rejection, so display tessellation samples never decide correctness.

use core::fmt;

use openrcad_foundation::{
    Ax3, Dir, Dir2d, Pnt, Pnt2d, ToleranceContext, TolerancePolicy, Vec as GeomVec,
};
use openrcad_geom::{BSplineCurve, BSplineSurface, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_geom2d::{GeomCurve2d, Line2d};
use openrcad_topo::{Edge, Face, Orientation, PcurveData, Solid, Vertex, Wire};

use crate::band_topology::{GeometryWorkBudget, GeometryWorkStage};
use crate::native_pcurve::planar_face_with_pcurves;
use crate::sew::sew_shell_with_policy;

/// Stable analytic family used to reject accidental cross-family
/// correspondence before any topology is built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SmoothSpanKind {
    Line,
    Circle,
    Ellipse,
    BSpline,
}

/// One bounded exact section span. `provenance` is caller-owned and used only
/// to choose a deterministic seam; geometry remains independent of traversal
/// order.
#[derive(Clone, Debug, PartialEq)]
pub struct SmoothCurveSpan {
    pub curve: BSplineCurve,
    pub first: f64,
    pub last: f64,
    pub provenance: u64,
    pub kind: SmoothSpanKind,
}

impl SmoothCurveSpan {
    pub fn new(
        curve: BSplineCurve,
        first: f64,
        last: f64,
        provenance: u64,
        kind: SmoothSpanKind,
    ) -> Self {
        Self {
            curve,
            first,
            last,
            provenance,
            kind,
        }
    }

    fn start(&self) -> Pnt {
        self.curve.point(self.first)
    }

    fn end(&self) -> Pnt {
        self.curve.point(self.last)
    }

    fn reversed(mut self) -> Self {
        core::mem::swap(&mut self.first, &mut self.last);
        self
    }
}

/// One complete analytic material section. Hole order may differ between
/// sections; correspondence is resolved uniquely before span matching.
#[derive(Clone, Debug, PartialEq)]
pub struct SmoothSectionLoops {
    pub outer: Vec<SmoothCurveSpan>,
    pub holes: Vec<Vec<SmoothCurveSpan>>,
}

/// Typed fail-closed Smooth Loft failures.
#[derive(Clone, Debug, PartialEq)]
pub enum SmoothLoftError {
    InvalidTolerancePolicy(String),
    TooFewSections,
    EmptyLoop {
        section: usize,
        loop_index: usize,
    },
    OpenLoop {
        section: usize,
        loop_index: usize,
        gap: f64,
    },
    HoleCountMismatch {
        section: usize,
        expected: usize,
        actual: usize,
    },
    AmbiguousHoleCorrespondence {
        section: usize,
    },
    AmbiguousSeam {
        section: usize,
        loop_index: usize,
        provenance: u64,
    },
    SpanCountMismatch {
        section: usize,
        loop_index: usize,
        expected: usize,
        actual: usize,
    },
    SpanKindMismatch {
        section: usize,
        loop_index: usize,
        span: usize,
        expected: SmoothSpanKind,
        actual: SmoothSpanKind,
    },
    InvalidCurve {
        section: usize,
        loop_index: usize,
        span: usize,
    },
    CoincidentSections {
        section: usize,
    },
    NonParallelSections {
        section: usize,
    },
    ReversedSectionOrder {
        section: usize,
    },
    WorkBudgetExhausted {
        stage: GeometryWorkStage,
        limit: u64,
        consumed: u64,
    },
    SingularInterpolation,
    NonPositiveWeight {
        span: usize,
        control: usize,
        weight: f64,
    },
    SelfIntersection {
        section_interval: usize,
    },
    SelfIntersectionUnresolved {
        first_loop: usize,
        second_loop: usize,
        first_span: usize,
        second_span: usize,
        stage: &'static str,
    },
    FaceBuild(String),
    Sew(String),
    InvalidTopology,
}

impl SmoothLoftError {
    pub const fn diagnostic_code(&self) -> &'static str {
        match self {
            Self::InvalidTolerancePolicy(_) => "parameter.invalid",
            Self::TooFewSections
            | Self::EmptyLoop { .. }
            | Self::OpenLoop { .. }
            | Self::HoleCountMismatch { .. }
            | Self::SpanCountMismatch { .. }
            | Self::SpanKindMismatch { .. }
            | Self::CoincidentSections { .. }
            | Self::NonParallelSections { .. }
            | Self::ReversedSectionOrder { .. } => "loft.section_mismatch",
            Self::AmbiguousHoleCorrespondence { .. } | Self::AmbiguousSeam { .. } => {
                "loft.ambiguous_correspondence"
            }
            Self::InvalidCurve { .. } => "loft.unsupported_span",
            Self::WorkBudgetExhausted { .. } => "operation.budget_exhausted",
            Self::SingularInterpolation | Self::FaceBuild(_) | Self::Sew(_) => "operation.failed",
            Self::NonPositiveWeight { .. } => "loft.non_positive_weight",
            Self::SelfIntersection { .. } => "loft.self_intersection",
            Self::SelfIntersectionUnresolved { .. } => "loft.self_intersection_unresolved",
            Self::InvalidTopology => "result.invalid_topology",
        }
    }
}

impl fmt::Display for SmoothLoftError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(reason) => {
                write!(formatter, "Smooth Loft tolerance policy is invalid: {reason}")
            }
            Self::TooFewSections => formatter.write_str("Smooth Loft needs at least two sections"),
            Self::EmptyLoop {
                section,
                loop_index,
            } => write!(formatter, "section {section} loop {loop_index} has no analytic spans"),
            Self::OpenLoop {
                section,
                loop_index,
                gap,
            } => write!(
                formatter,
                "section {section} loop {loop_index} is open by {gap}"
            ),
            Self::HoleCountMismatch {
                section,
                expected,
                actual,
            } => write!(
                formatter,
                "section {section} has {actual} holes; expected {expected}"
            ),
            Self::AmbiguousHoleCorrespondence { section } => {
                write!(formatter, "section {section} has ambiguous hole correspondence")
            }
            Self::AmbiguousSeam {
                section,
                loop_index,
                provenance,
            } => write!(
                formatter,
                "section {section} loop {loop_index} has ambiguous seam provenance {provenance}"
            ),
            Self::SpanCountMismatch {
                section,
                loop_index,
                expected,
                actual,
            } => write!(
                formatter,
                "section {section} loop {loop_index} has {actual} spans; expected {expected}"
            ),
            Self::SpanKindMismatch {
                section,
                loop_index,
                span,
                expected,
                actual,
            } => write!(
                formatter,
                "section {section} loop {loop_index} span {span} is {actual:?}; expected {expected:?}"
            ),
            Self::InvalidCurve {
                section,
                loop_index,
                span,
            } => write!(
                formatter,
                "section {section} loop {loop_index} span {span} is not a finite bounded rational curve"
            ),
            Self::CoincidentSections { section } => {
                write!(formatter, "sections {} and {section} are coincident", section - 1)
            }
            Self::NonParallelSections { section } => write!(
                formatter,
                "section {section} is outside the verified parallel-section Smooth Loft subset"
            ),
            Self::ReversedSectionOrder { section } => write!(
                formatter,
                "section {section} reverses the monotonic Loft section order"
            ),
            Self::WorkBudgetExhausted {
                stage,
                limit,
                consumed,
            } => write!(
                formatter,
                "Smooth Loft work budget exhausted during {stage:?} ({consumed}/{limit})"
            ),
            Self::SingularInterpolation => {
                formatter.write_str("Smooth Loft interpolation system is singular")
            }
            Self::NonPositiveWeight {
                span,
                control,
                weight,
            } => write!(
                formatter,
                "Smooth Loft span {span} control {control} produced non-positive weight {weight}"
            ),
            Self::SelfIntersection { section_interval } => write!(
                formatter,
                "Smooth Loft self-intersects in section interval {section_interval}"
            ),
            Self::SelfIntersectionUnresolved {
                first_loop,
                second_loop,
                first_span,
                second_span,
                stage,
            } => write!(
                formatter,
                "Smooth Loft could not certify loop/span {first_loop}/{first_span} and {second_loop}/{second_span} as non-self-intersecting during {stage}"
            ),
            Self::FaceBuild(reason) => write!(formatter, "Smooth Loft face build failed: {reason}"),
            Self::Sew(reason) => write!(formatter, "Smooth Loft sewing failed: {reason}"),
            Self::InvalidTopology => {
                formatter.write_str("Smooth Loft result did not pass strict topology validation")
            }
        }
    }
}

impl std::error::Error for SmoothLoftError {}

fn charge_work(
    budget: &mut GeometryWorkBudget,
    stage: GeometryWorkStage,
    units: u64,
) -> Result<(), SmoothLoftError> {
    budget.charge(stage, units).map_err(|error| match error {
        crate::band_topology::BandTopologyError::OperationBudgetExhausted {
            stage,
            limit,
            consumed,
        } => SmoothLoftError::WorkBudgetExhausted {
            stage,
            limit,
            consumed,
        },
        _ => unreachable!("GeometryWorkBudget::charge has one failure variant"),
    })
}

#[derive(Clone, Debug)]
struct BezierSpan {
    degree: usize,
    poles: Vec<Pnt>,
    weights: Vec<f64>,
}

impl BezierSpan {
    fn as_curve(&self) -> BSplineCurve {
        BSplineCurve::new(
            self.degree,
            self.poles.clone(),
            Some(self.weights.clone()),
            vec![0.0, 1.0],
            vec![self.degree + 1, self.degree + 1],
        )
    }

    fn elevated(mut self, target: usize) -> Self {
        while self.degree < target {
            let next_degree = self.degree + 1;
            let homogeneous = self
                .poles
                .iter()
                .zip(&self.weights)
                .map(|(point, weight)| {
                    [
                        point.x() * weight,
                        point.y() * weight,
                        point.z() * weight,
                        *weight,
                    ]
                })
                .collect::<Vec<_>>();
            let mut elevated = Vec::with_capacity(homogeneous.len() + 1);
            elevated.push(homogeneous[0]);
            for index in 1..=self.degree {
                let alpha = index as f64 / next_degree as f64;
                let mut value = [0.0; 4];
                for coordinate in 0..4 {
                    value[coordinate] = alpha * homogeneous[index - 1][coordinate]
                        + (1.0 - alpha) * homogeneous[index][coordinate];
                }
                elevated.push(value);
            }
            elevated.push(homogeneous[self.degree]);
            self.poles = elevated
                .iter()
                .map(|value| {
                    Pnt::new(
                        value[0] / value[3],
                        value[1] / value[3],
                        value[2] / value[3],
                    )
                })
                .collect();
            self.weights = elevated.iter().map(|value| value[3]).collect();
            self.degree = next_degree;
        }
        self
    }
}

fn loop_points(spans: &[SmoothCurveSpan]) -> Vec<Pnt> {
    // A closed analytic loop may legitimately consist of one periodic span
    // (for example, a full circle).  Sampling only span starts makes that
    // loop appear degenerate and also biases centroids toward segmentation.
    // These points are used for orientation/correspondence certificates only;
    // they never become durable topology.
    const CERTIFICATE_SAMPLES_PER_SPAN: usize = 4;
    spans
        .iter()
        .flat_map(|span| {
            (0..CERTIFICATE_SAMPLES_PER_SPAN).map(move |sample| {
                let fraction = sample as f64 / CERTIFICATE_SAMPLES_PER_SPAN as f64;
                span.curve
                    .point(span.first + (span.last - span.first) * fraction)
            })
        })
        .collect()
}

fn loop_normal(spans: &[SmoothCurveSpan]) -> Option<Dir> {
    let points = loop_points(spans);
    if points.len() < 3 {
        return None;
    }
    let mut normal = openrcad_foundation::Vec::ZERO;
    for index in 0..points.len() {
        let first = points[index];
        let second = points[(index + 1) % points.len()];
        normal += openrcad_foundation::Vec::new(
            (first.y() - second.y()) * (first.z() + second.z()),
            (first.z() - second.z()) * (first.x() + second.x()),
            (first.x() - second.x()) * (first.y() + second.y()),
        );
    }
    normal.normalized()
}

fn loop_centroid(spans: &[SmoothCurveSpan]) -> Pnt {
    let points = loop_points(spans);
    let count = points.len() as f64;
    Pnt::new(
        points.iter().map(Pnt::x).sum::<f64>() / count,
        points.iter().map(Pnt::y).sum::<f64>() / count,
        points.iter().map(Pnt::z).sum::<f64>() / count,
    )
}

fn validate_and_canonicalize_loop(
    mut spans: Vec<SmoothCurveSpan>,
    section: usize,
    loop_index: usize,
    reference_normal: Option<Dir>,
    policy: &TolerancePolicy,
) -> Result<Vec<SmoothCurveSpan>, SmoothLoftError> {
    if spans.is_empty() {
        return Err(SmoothLoftError::EmptyLoop {
            section,
            loop_index,
        });
    }
    for (span_index, span) in spans.iter().enumerate() {
        let (lower, upper) = span.curve.bounds();
        if !span.first.is_finite()
            || !span.last.is_finite()
            || (span.last - span.first).abs() <= policy.resolution
            || span.first.min(span.last) < lower - policy.resolution
            || span.first.max(span.last) > upper + policy.resolution
            || !span.start().x().is_finite()
            || !span.end().x().is_finite()
        {
            return Err(SmoothLoftError::InvalidCurve {
                section,
                loop_index,
                span: span_index,
            });
        }
        let next = &spans[(span_index + 1) % spans.len()];
        let gap = span.end().distance(&next.start());
        if gap > policy.linear {
            return Err(SmoothLoftError::OpenLoop {
                section,
                loop_index,
                gap,
            });
        }
    }
    let normal = loop_normal(&spans).ok_or(SmoothLoftError::InvalidCurve {
        section,
        loop_index,
        span: 0,
    })?;
    if reference_normal.is_some_and(|reference| {
        openrcad_foundation::Vec::from_dir(reference)
            .dot(&openrcad_foundation::Vec::from_dir(normal))
            < 0.0
    }) {
        spans = spans
            .into_iter()
            .rev()
            .map(SmoothCurveSpan::reversed)
            .collect();
    }

    let minimum = spans
        .iter()
        .map(|span| span.provenance)
        .min()
        .expect("non-empty loop");
    let candidates = spans
        .iter()
        .enumerate()
        .filter(|(_, span)| span.provenance == minimum)
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    if candidates.len() != 1 {
        return Err(SmoothLoftError::AmbiguousSeam {
            section,
            loop_index,
            provenance: minimum,
        });
    }
    spans.rotate_left(candidates[0]);
    Ok(spans)
}

fn hole_assignment(
    previous: &[Vec<SmoothCurveSpan>],
    current: &[Vec<SmoothCurveSpan>],
    section: usize,
    tolerance: f64,
) -> Result<Vec<usize>, SmoothLoftError> {
    let previous_points = previous
        .iter()
        .map(|loop_| loop_points(loop_))
        .collect::<Vec<_>>();
    let current_points = current
        .iter()
        .map(|loop_| loop_points(loop_))
        .collect::<Vec<_>>();
    crate::skin::match_holes(&previous_points, &current_points, section, tolerance)
        .map_err(|_| SmoothLoftError::AmbiguousHoleCorrespondence { section })
}

fn normalized_breaks(span: &SmoothCurveSpan, policy: &TolerancePolicy) -> Vec<f64> {
    let denominator = span.last - span.first;
    let mut breaks = vec![0.0, 1.0];
    for &knot in span.curve.knots() {
        let normalized = (knot - span.first) / denominator;
        if normalized > policy.resolution && normalized < 1.0 - policy.resolution {
            breaks.push(normalized);
        }
    }
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|left, right| (*left - *right).abs() <= policy.resolution);
    breaks
}

fn knot_multiplicity(curve: &BSplineCurve, knot: f64, policy: &TolerancePolicy) -> usize {
    curve
        .knots()
        .iter()
        .zip(curve.multiplicities())
        .find(|(candidate, _)| (**candidate - knot).abs() <= policy.resolution)
        .map(|(_, multiplicity)| *multiplicity)
        .unwrap_or(0)
}

fn decompose_span(
    span: &SmoothCurveSpan,
    normalized: &[f64],
    policy: &TolerancePolicy,
) -> Result<Vec<BezierSpan>, SmoothLoftError> {
    let mut curve = span.curve.clone();
    let degree = curve.degree();
    let mut split_parameters = curve.knots().to_vec();
    split_parameters.extend(
        normalized
            .iter()
            .map(|fraction| span.first + (span.last - span.first) * fraction),
    );
    split_parameters.sort_by(f64::total_cmp);
    split_parameters.dedup_by(|left, right| (*left - *right).abs() <= policy.resolution);
    for parameter in split_parameters {
        while knot_multiplicity(&curve, parameter, policy) < degree {
            curve.insert_knot(parameter, 1);
        }
    }

    let weights = curve
        .weights()
        .map(ToOwned::to_owned)
        .unwrap_or_else(|| vec![1.0; curve.poles().len()]);
    let knots = curve.knots();
    let mut pieces = Vec::with_capacity(normalized.len().saturating_sub(1));
    for pair in normalized.windows(2) {
        let first = span.first + (span.last - span.first) * pair[0];
        let last = span.first + (span.last - span.first) * pair[1];
        let lower = first.min(last);
        let upper = first.max(last);
        let Some(interval) = knots.windows(2).position(|window| {
            (window[0] - lower).abs() <= policy.resolution
                && (window[1] - upper).abs() <= policy.resolution
        }) else {
            return Err(SmoothLoftError::SingularInterpolation);
        };
        let start = interval * degree;
        let end = start + degree + 1;
        if end > curve.poles().len() {
            return Err(SmoothLoftError::SingularInterpolation);
        }
        let mut poles = curve.poles()[start..end].to_vec();
        let mut piece_weights = weights[start..end].to_vec();
        if first > last {
            poles.reverse();
            piece_weights.reverse();
        }
        pieces.push(BezierSpan {
            degree,
            poles,
            weights: piece_weights,
        });
    }
    Ok(pieces)
}

fn prepare_loop(
    loops: &[Vec<SmoothCurveSpan>],
    loop_index: usize,
    policy: &TolerancePolicy,
) -> Result<Vec<Vec<BezierSpan>>, SmoothLoftError> {
    let expected = loops[0].len();
    for (section, loop_) in loops.iter().enumerate().skip(1) {
        if loop_.len() != expected {
            return Err(SmoothLoftError::SpanCountMismatch {
                section,
                loop_index,
                expected,
                actual: loop_.len(),
            });
        }
        for span in 0..expected {
            if loop_[span].kind != loops[0][span].kind {
                return Err(SmoothLoftError::SpanKindMismatch {
                    section,
                    loop_index,
                    span,
                    expected: loops[0][span].kind,
                    actual: loop_[span].kind,
                });
            }
        }
    }

    let mut prepared = vec![Vec::new(); loops.len()];
    for span_index in 0..expected {
        let mut breaks = Vec::new();
        for loop_ in loops {
            breaks.extend(normalized_breaks(&loop_[span_index], policy));
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup_by(|left, right| (*left - *right).abs() <= policy.resolution);
        let pieces = loops
            .iter()
            .map(|loop_| decompose_span(&loop_[span_index], &breaks, policy))
            .collect::<Result<Vec<_>, _>>()?;
        for piece_index in 0..breaks.len().saturating_sub(1) {
            let target_degree = pieces
                .iter()
                .map(|section| section[piece_index].degree)
                .max()
                .expect("at least one section");
            for section in 0..pieces.len() {
                prepared[section]
                    .push(pieces[section][piece_index].clone().elevated(target_degree));
            }
        }
    }
    Ok(prepared)
}

fn section_parameters(
    sections: &[Vec<BezierSpan>],
    policy: &TolerancePolicy,
) -> Result<Vec<f64>, SmoothLoftError> {
    let mut parameters = vec![0.0];
    let mut accumulated = 0.0;
    for section in 1..sections.len() {
        // Chord length is measured over the compatible exact control net, not
        // just one seam vertex.  A spline can change shape while keeping that
        // vertex fixed; treating such sections as coincident would reject a
        // valid interpolation or produce unstable V spacing.
        let samples = sections[section]
            .iter()
            .map(|span| span.poles.len())
            .sum::<usize>();
        let squared = sections[section - 1]
            .iter()
            .zip(&sections[section])
            .flat_map(|(previous, current)| previous.poles.iter().zip(&current.poles))
            .map(|(previous, current)| previous.distance(current).powi(2))
            .sum::<f64>();
        let chord = (squared / samples as f64).sqrt();
        if !chord.is_finite() || chord <= policy.linear {
            return Err(SmoothLoftError::CoincidentSections { section });
        }
        accumulated += chord;
        parameters.push(accumulated);
    }
    for parameter in &mut parameters {
        *parameter /= accumulated;
    }
    Ok(parameters)
}

fn interpolation_knots(parameters: &[f64], degree: usize) -> Vec<f64> {
    let controls = parameters.len();
    let mut flat = vec![0.0; controls + degree + 1];
    for value in flat.iter_mut().skip(controls) {
        *value = 1.0;
    }
    for index in 1..(controls - degree) {
        flat[index + degree] =
            parameters[index..index + degree].iter().sum::<f64>() / degree as f64;
    }
    flat
}

fn basis_row(parameter: f64, degree: usize, knots: &[f64], controls: usize) -> Vec<f64> {
    let mut values = vec![0.0; controls];
    for index in 0..controls {
        if (knots[index] <= parameter && parameter < knots[index + 1])
            || (parameter == 1.0 && index + 1 == controls)
        {
            values[index] = 1.0;
        }
    }
    for order in 1..=degree {
        let previous = values.clone();
        for index in 0..controls {
            let left_denominator = knots[index + order] - knots[index];
            let left = if left_denominator > 0.0 {
                (parameter - knots[index]) / left_denominator * previous[index]
            } else {
                0.0
            };
            let right_denominator = knots[index + order + 1] - knots[index + 1];
            let right = if right_denominator > 0.0 && index + 1 < controls {
                (knots[index + order + 1] - parameter) / right_denominator * previous[index + 1]
            } else {
                0.0
            };
            values[index] = left + right;
        }
    }
    values
}

fn solve_system(
    matrix: &[Vec<f64>],
    values: &[f64],
    pivot_tolerance: f64,
) -> Result<Vec<f64>, SmoothLoftError> {
    let count = values.len();
    let mut augmented = matrix
        .iter()
        .zip(values)
        .map(|(row, value)| {
            let mut row = row.clone();
            row.push(*value);
            row
        })
        .collect::<Vec<_>>();
    for column in 0..count {
        let pivot = (column..count)
            .max_by(|left, right| {
                augmented[*left][column]
                    .abs()
                    .total_cmp(&augmented[*right][column].abs())
            })
            .expect("non-empty pivot range");
        if augmented[pivot][column].abs() <= pivot_tolerance {
            return Err(SmoothLoftError::SingularInterpolation);
        }
        augmented.swap(column, pivot);
        let divisor = augmented[column][column];
        for entry in &mut augmented[column][column..=count] {
            *entry /= divisor;
        }
        for row in 0..count {
            if row == column {
                continue;
            }
            let factor = augmented[row][column];
            for entry in column..=count {
                augmented[row][entry] -= factor * augmented[column][entry];
            }
        }
    }
    Ok(augmented.into_iter().map(|row| row[count]).collect())
}

fn distinct_knots(flat: &[f64], tolerance: f64) -> (Vec<f64>, Vec<usize>) {
    let mut knots = Vec::new();
    let mut multiplicities = Vec::new();
    for &value in flat {
        if knots
            .last()
            .is_some_and(|previous: &f64| (*previous - value).abs() <= tolerance)
        {
            *multiplicities.last_mut().expect("knot has multiplicity") += 1;
        } else {
            knots.push(value);
            multiplicities.push(1);
        }
    }
    (knots, multiplicities)
}

fn interpolate_surface(
    sections: &[BezierSpan],
    section_parameters: &[f64],
    span_index: usize,
    context: &ToleranceContext,
) -> Result<BSplineSurface, SmoothLoftError> {
    let u_degree = sections[0].degree;
    let v_degree = usize::min(3, sections.len() - 1);
    let v_flat = interpolation_knots(section_parameters, v_degree);
    let matrix = section_parameters
        .iter()
        .map(|parameter| basis_row(*parameter, v_degree, &v_flat, sections.len()))
        .collect::<Vec<_>>();
    let origin = sections[0].poles[0];
    let mut poles = vec![vec![Pnt::origin(); sections.len()]; u_degree + 1];
    let mut weights = vec![vec![0.0; sections.len()]; u_degree + 1];
    for u_control in 0..=u_degree {
        let data = sections
            .iter()
            .map(|section| {
                let point = section.poles[u_control];
                let weight = section.weights[u_control];
                [
                    (point.x() - origin.x()) * weight,
                    (point.y() - origin.y()) * weight,
                    (point.z() - origin.z()) * weight,
                    weight,
                ]
            })
            .collect::<Vec<_>>();
        let solved = (0..4)
            .map(|coordinate| {
                solve_system(
                    &matrix,
                    &data
                        .iter()
                        .map(|value| value[coordinate])
                        .collect::<Vec<_>>(),
                    context.arithmetic_floor,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        for v_control in 0..sections.len() {
            let weight = solved[3][v_control];
            if !weight.is_finite() || weight <= context.policy.resolution {
                return Err(SmoothLoftError::NonPositiveWeight {
                    span: span_index,
                    control: u_control * sections.len() + v_control,
                    weight,
                });
            }
            poles[u_control][v_control] = Pnt::new(
                origin.x() + solved[0][v_control] / weight,
                origin.y() + solved[1][v_control] / weight,
                origin.z() + solved[2][v_control] / weight,
            );
            weights[u_control][v_control] = weight;
        }
    }
    let (v_knots, v_multiplicities) = distinct_knots(&v_flat, context.policy.resolution);
    Ok(BSplineSurface::new(
        u_degree,
        v_degree,
        poles,
        Some(weights),
        vec![0.0, 1.0],
        vec![u_degree + 1, u_degree + 1],
        v_knots,
        v_multiplicities,
    ))
}

fn edge_from_curve(curve: BSplineCurve) -> Edge {
    let (first, last) = curve.bounds();
    let start = Vertex::new(curve.point(first));
    let end = Vertex::new(curve.point(last));
    Edge::new(Some(GeomCurve::bspline(curve)), first, last, start, end)
}

fn uv_line(start: Pnt2d, end: Pnt2d) -> PcurveData {
    let delta = end - start;
    let length = delta.magnitude();
    let direction = Dir2d::new(delta.x() / length, delta.y() / length);
    PcurveData::new(
        GeomCurve2d::line(Line2d::from_point_dir(start, direction)),
        0.0,
        length,
    )
}

fn surface_face(surface: BSplineSurface) -> Result<Face, SmoothLoftError> {
    let u_degree = surface.u_degree();
    let v_degree = surface.v_degree();
    let u_controls = surface.poles().len();
    let v_controls = surface.poles()[0].len();
    let surface_weights = surface.weights().expect("Smooth Loft is rational");
    let bottom = BSplineCurve::new(
        u_degree,
        surface.poles().iter().map(|row| row[0]).collect(),
        Some(surface_weights.iter().map(|row| row[0]).collect()),
        surface.u_knots().to_vec(),
        surface.u_multiplicities().to_vec(),
    );
    let top = BSplineCurve::new(
        u_degree,
        surface
            .poles()
            .iter()
            .map(|row| row[v_controls - 1])
            .collect(),
        Some(
            surface_weights
                .iter()
                .map(|row| row[v_controls - 1])
                .collect(),
        ),
        surface.u_knots().to_vec(),
        surface.u_multiplicities().to_vec(),
    );
    let left = BSplineCurve::new(
        v_degree,
        surface.poles()[0].clone(),
        Some(surface_weights[0].clone()),
        surface.v_knots().to_vec(),
        surface.v_multiplicities().to_vec(),
    );
    let right = BSplineCurve::new(
        v_degree,
        surface.poles()[u_controls - 1].clone(),
        Some(surface_weights[u_controls - 1].clone()),
        surface.v_knots().to_vec(),
        surface.v_multiplicities().to_vec(),
    );
    let bottom = edge_from_curve(bottom);
    let right = edge_from_curve(right);
    let top = edge_from_curve(top).reversed();
    let left = edge_from_curve(left).reversed();
    let wire = Wire::from_edges([bottom, right, top, left]);
    Face::with_pcurves(
        GeomSurface::bspline(surface),
        wire,
        vec![
            uv_line(Pnt2d::new(0.0, 0.0), Pnt2d::new(1.0, 0.0)),
            uv_line(Pnt2d::new(1.0, 0.0), Pnt2d::new(1.0, 1.0)),
            uv_line(Pnt2d::new(0.0, 1.0), Pnt2d::new(1.0, 1.0)),
            uv_line(Pnt2d::new(0.0, 0.0), Pnt2d::new(0.0, 1.0)),
        ],
    )
    .map_err(|error| SmoothLoftError::FaceBuild(error.to_string()))
}

fn wire_from_beziers(spans: &[BezierSpan]) -> Wire {
    Wire::from_edges(spans.iter().map(|span| edge_from_curve(span.as_curve())))
}

fn smooth_loop_surfaces(
    prepared: &[Vec<BezierSpan>],
    parameters: &[f64],
    context: &ToleranceContext,
) -> Result<Vec<BSplineSurface>, SmoothLoftError> {
    (0..prepared[0].len())
        .map(|span| {
            let sections = prepared
                .iter()
                .map(|section| section[span].clone())
                .collect::<Vec<_>>();
            interpolate_surface(&sections, parameters, span, context)
        })
        .collect()
}

fn append_smooth_loop_faces(
    surfaces: &[BSplineSurface],
    faces: &mut Vec<Face>,
) -> Result<(), SmoothLoftError> {
    for surface in surfaces {
        faces.push(surface_face(surface.clone())?);
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct CertificateFrame {
    origin: Pnt,
    x: GeomVec,
    y: GeomVec,
    normal: GeomVec,
}

impl CertificateFrame {
    fn new(origin: Pnt, normal: Dir) -> Self {
        let axes = Ax3::new(origin, normal);
        Self {
            origin,
            x: GeomVec::from_dir(axes.x_direction()),
            y: GeomVec::from_dir(axes.y_direction()),
            normal: GeomVec::from_dir(normal),
        }
    }

    fn project(self, point: Pnt) -> [f64; 2] {
        let offset = point - self.origin;
        [offset.dot(&self.x), offset.dot(&self.y)]
    }

    fn axial(self, point: Pnt) -> f64 {
        (point - self.origin).dot(&self.normal)
    }
}

fn unresolved_self_intersection(
    first_loop: usize,
    second_loop: usize,
    first_span: usize,
    second_span: usize,
    stage: &'static str,
) -> SmoothLoftError {
    SmoothLoftError::SelfIntersectionUnresolved {
        first_loop,
        second_loop,
        first_span,
        second_span,
        stage,
    }
}

fn relative_close(first: f64, second: f64, tolerance: f64) -> bool {
    (first - second).abs() <= tolerance * first.abs().max(second.abs()).max(1.0)
}

fn certify_common_axial_parameter(
    loops: &[Vec<BSplineSurface>],
    frame: CertificateFrame,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<(), SmoothLoftError> {
    let reference_surface = &loops[0][0];
    let reference_weights = &reference_surface
        .weights()
        .expect("Smooth Loft surfaces are rational")[0];
    let v_controls = reference_weights.len();
    let reference_heights = reference_surface.poles()[0]
        .iter()
        .map(|point| frame.axial(*point))
        .collect::<Vec<_>>();
    let weight_tolerance = context.policy.resolution * 64.0;

    for (loop_index, surfaces) in loops.iter().enumerate() {
        for (span_index, surface) in surfaces.iter().enumerate() {
            let weights = surface
                .weights()
                .expect("Smooth Loft surfaces are rational");
            if surface.poles()[0].len() != v_controls {
                return Err(unresolved_self_intersection(
                    loop_index,
                    loop_index,
                    span_index,
                    span_index,
                    "axial_control_count",
                ));
            }
            for (poles, row_weights) in surface.poles().iter().zip(weights) {
                charge_work(
                    budget,
                    GeometryWorkStage::Classification,
                    u64::try_from(v_controls).unwrap_or(u64::MAX),
                )?;
                let scale = row_weights[0] / reference_weights[0];
                for control in 0..v_controls {
                    if !relative_close(
                        row_weights[control],
                        scale * reference_weights[control],
                        weight_tolerance,
                    ) {
                        return Err(unresolved_self_intersection(
                            loop_index,
                            loop_index,
                            span_index,
                            span_index,
                            "axial_weight_factorization",
                        ));
                    }
                    if (frame.axial(poles[control]) - reference_heights[control]).abs()
                        > context.policy.classification
                    {
                        return Err(unresolved_self_intersection(
                            loop_index,
                            loop_index,
                            span_index,
                            span_index,
                            "common_axial_control",
                        ));
                    }
                }
            }
        }
    }

    let direction = (reference_heights[v_controls - 1] - reference_heights[0]).signum();
    if direction == 0.0
        || reference_heights
            .windows(2)
            .any(|pair| (pair[1] - pair[0]) * direction <= context.policy.linear)
    {
        return Err(unresolved_self_intersection(
            0,
            0,
            0,
            0,
            "axial_monotonicity",
        ));
    }
    Ok(())
}

fn normalize_2d(value: [f64; 2], tolerance: f64) -> Option<[f64; 2]> {
    let length = value[0].hypot(value[1]);
    (length > tolerance).then_some([value[0] / length, value[1] / length])
}

fn dot_2d(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[0] + first[1] * second[1]
}

fn sub_2d(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] - second[0], first[1] - second[1]]
}

fn certify_span_injective(
    surface: &BSplineSurface,
    loop_index: usize,
    span_index: usize,
    frame: CertificateFrame,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<(), SmoothLoftError> {
    let poles = surface.poles();
    let mut candidates = Vec::new();
    let mut average = [0.0, 0.0];
    for control in 0..poles[0].len() {
        let direction = sub_2d(
            frame.project(poles[poles.len() - 1][control]),
            frame.project(poles[0][control]),
        );
        average[0] += direction[0];
        average[1] += direction[1];
        candidates.push(direction);
    }
    candidates.push(average);

    for axis in candidates
        .into_iter()
        .filter_map(|axis| normalize_2d(axis, context.policy.linear))
    {
        let mut positive = true;
        let mut negative = true;
        for rows in poles.windows(2) {
            for control in 0..rows[0].len() {
                charge_work(budget, GeometryWorkStage::Classification, 1)?;
                let delta = sub_2d(
                    frame.project(rows[1][control]),
                    frame.project(rows[0][control]),
                );
                let projection = dot_2d(delta, axis);
                positive &= projection > context.policy.linear;
                negative &= projection < -context.policy.linear;
            }
        }
        if positive || negative {
            return Ok(());
        }
    }
    Err(unresolved_self_intersection(
        loop_index,
        loop_index,
        span_index,
        span_index,
        "span_injectivity",
    ))
}

fn certify_adjacent_seam(
    first: &BSplineSurface,
    second: &BSplineSurface,
    loop_index: usize,
    first_span: usize,
    second_span: usize,
    frame: CertificateFrame,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<(), SmoothLoftError> {
    let first_poles = first.poles();
    let second_poles = second.poles();
    let seam_first = &first_poles[first_poles.len() - 1];
    let seam_second = &second_poles[0];
    if seam_first.len() != seam_second.len()
        || seam_first
            .iter()
            .zip(seam_second)
            .any(|(left, right)| left.distance(right) > context.policy.pcurve_consistency)
    {
        return Err(unresolved_self_intersection(
            loop_index,
            loop_index,
            first_span,
            second_span,
            "adjacent_seam_agreement",
        ));
    }

    let mut first_offsets = Vec::new();
    let mut second_offsets = Vec::new();
    for control in 0..seam_first.len() {
        let seam = frame.project(seam_first[control]);
        first_offsets.extend(
            first_poles[..first_poles.len() - 1]
                .iter()
                .map(|row| sub_2d(frame.project(row[control]), seam)),
        );
        second_offsets.extend(
            second_poles[1..]
                .iter()
                .map(|row| sub_2d(frame.project(row[control]), seam)),
        );
    }
    let mut first_average = [0.0, 0.0];
    let mut second_average = [0.0, 0.0];
    for offset in &first_offsets {
        first_average[0] += offset[0];
        first_average[1] += offset[1];
    }
    for offset in &second_offsets {
        second_average[0] += offset[0];
        second_average[1] += offset[1];
    }
    let mut candidates = vec![sub_2d(second_average, first_average)];
    for first_offset in &first_offsets {
        for second_offset in &second_offsets {
            candidates.push(sub_2d(*second_offset, *first_offset));
        }
    }

    for axis in candidates
        .into_iter()
        .filter_map(|axis| normalize_2d(axis, context.policy.linear))
    {
        charge_work(
            budget,
            GeometryWorkStage::Classification,
            u64::try_from(first_offsets.len().saturating_add(second_offsets.len()))
                .unwrap_or(u64::MAX),
        )?;
        let first_negative = first_offsets
            .iter()
            .all(|offset| dot_2d(*offset, axis) < -context.policy.linear);
        let second_positive = second_offsets
            .iter()
            .all(|offset| dot_2d(*offset, axis) > context.policy.linear);
        let first_positive = first_offsets
            .iter()
            .all(|offset| dot_2d(*offset, axis) > context.policy.linear);
        let second_negative = second_offsets
            .iter()
            .all(|offset| dot_2d(*offset, axis) < -context.policy.linear);
        if (first_negative && second_positive) || (first_positive && second_negative) {
            return Ok(());
        }
    }
    Err(unresolved_self_intersection(
        loop_index,
        loop_index,
        first_span,
        second_span,
        "adjacent_seam_separation",
    ))
}

fn certify_equal_parameter_separation(
    first: &BSplineSurface,
    second: &BSplineSurface,
    frame: CertificateFrame,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<bool, SmoothLoftError> {
    let first_poles = first.poles();
    let second_poles = second.poles();
    if first_poles[0].len() != second_poles[0].len() {
        return Ok(false);
    }
    let v_controls = first_poles[0].len();
    let center = |poles: &[Vec<Pnt>], control: usize| {
        let mut result = [0.0, 0.0];
        for row in poles {
            let point = frame.project(row[control]);
            result[0] += point[0];
            result[1] += point[1];
        }
        [
            result[0] / poles.len() as f64,
            result[1] / poles.len() as f64,
        ]
    };
    let mut candidates = vec![[1.0, 0.0], [0.0, 1.0]];
    let mut average = [0.0, 0.0];
    for control in 0..v_controls {
        let direction = sub_2d(center(second_poles, control), center(first_poles, control));
        average[0] += direction[0];
        average[1] += direction[1];
        candidates.push(direction);
        for poles in [first_poles, second_poles] {
            for rows in poles.windows(2) {
                let edge = sub_2d(
                    frame.project(rows[1][control]),
                    frame.project(rows[0][control]),
                );
                candidates.push(edge);
                candidates.push([-edge[1], edge[0]]);
            }
        }
    }
    candidates.push(average);

    for axis in candidates
        .into_iter()
        .filter_map(|axis| normalize_2d(axis, context.policy.linear))
    {
        let mut first_before_second = true;
        let mut second_before_first = true;
        for control in 0..v_controls {
            charge_work(
                budget,
                GeometryWorkStage::Classification,
                u64::try_from(first_poles.len().saturating_add(second_poles.len()))
                    .unwrap_or(u64::MAX),
            )?;
            let interval = |poles: &[Vec<Pnt>]| {
                poles.iter().fold(
                    (f64::INFINITY, f64::NEG_INFINITY),
                    |(minimum, maximum), row| {
                        let value = dot_2d(frame.project(row[control]), axis);
                        (minimum.min(value), maximum.max(value))
                    },
                )
            };
            let (first_min, first_max) = interval(first_poles);
            let (second_min, second_max) = interval(second_poles);
            first_before_second &= first_max + context.policy.classification < second_min;
            second_before_first &= second_max + context.policy.classification < first_min;
        }
        if first_before_second || second_before_first {
            return Ok(true);
        }
    }
    Ok(false)
}

fn certify_smooth_surface_family(
    loops: &[Vec<BSplineSurface>],
    origin: Pnt,
    normal: Dir,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<(), SmoothLoftError> {
    let frame = CertificateFrame::new(origin, normal);
    certify_common_axial_parameter(loops, frame, context, budget)?;

    for (loop_index, surfaces) in loops.iter().enumerate() {
        for (span_index, surface) in surfaces.iter().enumerate() {
            certify_span_injective(surface, loop_index, span_index, frame, context, budget)?;
            let next = (span_index + 1) % surfaces.len();
            certify_adjacent_seam(
                surface,
                &surfaces[next],
                loop_index,
                span_index,
                next,
                frame,
                context,
                budget,
            )?;
        }
        for first in 0..surfaces.len() {
            for second in first + 1..surfaces.len() {
                let adjacent = second == first + 1 || (first == 0 && second + 1 == surfaces.len());
                if adjacent {
                    continue;
                }
                if !certify_equal_parameter_separation(
                    &surfaces[first],
                    &surfaces[second],
                    frame,
                    context,
                    budget,
                )? {
                    return Err(unresolved_self_intersection(
                        loop_index,
                        loop_index,
                        first,
                        second,
                        "nonadjacent_control_bounds",
                    ));
                }
            }
        }
    }

    for first_loop in 0..loops.len() {
        for second_loop in first_loop + 1..loops.len() {
            for first_span in 0..loops[first_loop].len() {
                for second_span in 0..loops[second_loop].len() {
                    if !certify_equal_parameter_separation(
                        &loops[first_loop][first_span],
                        &loops[second_loop][second_span],
                        frame,
                        context,
                        budget,
                    )? {
                        return Err(unresolved_self_intersection(
                            first_loop,
                            second_loop,
                            first_span,
                            second_span,
                            "cross_loop_control_bounds",
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

/// Build a Smooth Loft through exact analytic section loops.
///
/// The verified release subset requires parallel, monotonically ordered
/// sections. That restriction turns ambiguous folded/self-crossing cases into
/// typed rejection while still covering the common multi-section product
/// workflow. Arbitrary orientation changes remain available through Ruled
/// Loft until a stronger envelope certificate is implemented.
pub fn skin_smooth_section_loops_with_policy(
    sections: &[SmoothSectionLoops],
    policy: &TolerancePolicy,
) -> Result<Solid, SmoothLoftError> {
    policy
        .validate()
        .map_err(|error| SmoothLoftError::InvalidTolerancePolicy(error.to_string()))?;
    if sections.len() < 2 {
        return Err(SmoothLoftError::TooFewSections);
    }
    let mut bounds = openrcad_foundation::BndBox::new();
    for section in sections {
        for span in section.outer.iter().chain(section.holes.iter().flatten()) {
            for point in span.curve.poles() {
                bounds.add(point);
            }
            bounds.add(&span.start());
            bounds.add(&span.end());
        }
    }
    let context = ToleranceContext::derive(policy, &[bounds], None, 1.0)
        .map_err(|error| SmoothLoftError::InvalidTolerancePolicy(error.to_string()))?;
    let mut budget = GeometryWorkBudget::intersection_default();
    for section in sections {
        for span in section.outer.iter().chain(section.holes.iter().flatten()) {
            let units = span
                .curve
                .poles()
                .len()
                .saturating_add(span.curve.knots().len());
            charge_work(
                &mut budget,
                GeometryWorkStage::RootIsolation,
                u64::try_from(units).unwrap_or(u64::MAX),
            )?;
        }
    }

    let first_normal = loop_normal(&sections[0].outer).ok_or(SmoothLoftError::InvalidCurve {
        section: 0,
        loop_index: 0,
        span: 0,
    })?;
    let mut aligned: Vec<SmoothSectionLoops> = Vec::with_capacity(sections.len());
    for (section_index, section) in sections.iter().enumerate() {
        if section.holes.len() != sections[0].holes.len() {
            return Err(SmoothLoftError::HoleCountMismatch {
                section: section_index,
                expected: sections[0].holes.len(),
                actual: section.holes.len(),
            });
        }
        let outer = validate_and_canonicalize_loop(
            section.outer.clone(),
            section_index,
            0,
            Some(first_normal),
            &context.policy,
        )?;
        let normal = loop_normal(&outer).expect("validated loop normal");
        if openrcad_foundation::Vec::from_dir(first_normal)
            .dot(&openrcad_foundation::Vec::from_dir(normal))
            < 1.0 - context.policy.angular * 8.0
        {
            return Err(SmoothLoftError::NonParallelSections {
                section: section_index,
            });
        }
        let mut holes = section.holes.clone();
        if let Some(previous) = aligned.last() {
            let assignment = hole_assignment(
                &previous.holes,
                &holes,
                section_index,
                context.policy.classification,
            )?;
            holes = assignment
                .into_iter()
                .map(|index| holes[index].clone())
                .collect();
        }
        let holes = holes
            .into_iter()
            .enumerate()
            .map(|(hole, loop_)| {
                let reference_normal = aligned
                    .last()
                    .and_then(|previous| loop_normal(&previous.holes[hole]));
                validate_and_canonicalize_loop(
                    loop_,
                    section_index,
                    hole + 1,
                    reference_normal,
                    &context.policy,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        aligned.push(SmoothSectionLoops { outer, holes });
    }

    let direction = openrcad_foundation::Vec::from_dir(first_normal);
    let centroids = aligned
        .iter()
        .map(|section| loop_centroid(&section.outer))
        .collect::<Vec<_>>();
    let first_step = (centroids[1] - centroids[0]).dot(&direction);
    if first_step.abs() <= context.policy.linear {
        return Err(SmoothLoftError::CoincidentSections { section: 1 });
    }
    let sign = first_step.signum();
    for section in 1..centroids.len() {
        let step = (centroids[section] - centroids[section - 1]).dot(&direction);
        if step.abs() <= context.policy.linear {
            return Err(SmoothLoftError::CoincidentSections { section });
        }
        if step.signum() != sign {
            return Err(SmoothLoftError::ReversedSectionOrder { section });
        }
    }

    let outer_loops = aligned
        .iter()
        .map(|section| section.outer.clone())
        .collect::<Vec<_>>();
    let prepared_outer = prepare_loop(&outer_loops, 0, &context.policy)?;
    let parameters = section_parameters(&prepared_outer, &context.policy)?;

    let mut prepared_holes = Vec::with_capacity(aligned[0].holes.len());
    for hole in 0..aligned[0].holes.len() {
        let loops = aligned
            .iter()
            .map(|section| section.holes[hole].clone())
            .collect::<Vec<_>>();
        let prepared = prepare_loop(&loops, hole + 1, &context.policy)?;
        prepared_holes.push(prepared);
    }

    let mut loop_surfaces = Vec::with_capacity(prepared_holes.len() + 1);
    loop_surfaces.push(smooth_loop_surfaces(
        &prepared_outer,
        &parameters,
        &context,
    )?);
    for hole in &prepared_holes {
        loop_surfaces.push(smooth_loop_surfaces(hole, &parameters, &context)?);
    }
    certify_smooth_surface_family(
        &loop_surfaces,
        aligned[0].outer[0].start(),
        first_normal,
        &context,
        &mut budget,
    )?;

    let mut faces = Vec::new();
    append_smooth_loop_faces(&loop_surfaces[0], &mut faces)?;
    for surfaces in &loop_surfaces[1..] {
        append_smooth_loop_faces(surfaces, &mut faces)?;
    }

    for section in [0, aligned.len() - 1] {
        let normal = loop_normal(&aligned[section].outer).expect("validated cap normal");
        let plane = Plane::from_point_normal(aligned[section].outer[0].start(), normal);
        let outer = wire_from_beziers(&prepared_outer[section]);
        let holes = prepared_holes
            .iter()
            .map(|hole| wire_from_beziers(&hole[section]))
            .collect();
        faces.push(
            planar_face_with_pcurves(plane, Some(outer), holes, Orientation::Forward)
                .map_err(|error| SmoothLoftError::FaceBuild(error.to_string()))?,
        );
    }

    let solid = Solid::new(
        sew_shell_with_policy(&faces, &context.policy)
            .map_err(|error| SmoothLoftError::Sew(error.to_string()))?,
    );
    if !solid.is_watertight_with_policy(&context.policy) {
        return Err(SmoothLoftError::InvalidTopology);
    }
    Ok(solid)
}

#[cfg(test)]
mod certificate_tests {
    use super::*;

    fn strip(x_controls: [f64; 4]) -> BSplineSurface {
        let poles = [0.0, 1.0]
            .into_iter()
            .map(|y| {
                x_controls
                    .into_iter()
                    .enumerate()
                    .map(|(control, x)| Pnt::new(x, y, control as f64 / 3.0))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        BSplineSurface::new(
            1,
            3,
            poles,
            Some(vec![vec![1.0; 4]; 2]),
            vec![0.0, 1.0],
            vec![2, 2],
            vec![0.0, 1.0],
            vec![4, 4],
        )
    }

    #[test]
    fn control_certificate_rejects_an_overlap_between_legacy_sample_stations() {
        let fixed = strip([0.0; 4]);
        let crossing = strip([0.01, -0.2, 1.0, 1.0]);
        for station in [0.25, 0.5, 0.75] {
            assert!(
                openrcad_geom::Surface::point(&crossing, 0.5, station).x() > 0.0,
                "the legacy station at {station} must miss the narrow crossing"
            );
        }

        let mut bounds = openrcad_foundation::BndBox::new();
        for point in fixed.poles().iter().chain(crossing.poles()).flatten() {
            bounds.add(point);
        }
        let context =
            ToleranceContext::derive(&TolerancePolicy::STANDARD, &[bounds], Some(1.0), 1.0)
                .expect("test tolerance context");
        let mut budget = GeometryWorkBudget::intersection_default();
        assert!(
            !certify_equal_parameter_separation(
                &fixed,
                &crossing,
                CertificateFrame::new(Pnt::origin(), Dir::dz()),
                &context,
                &mut budget,
            )
            .expect("certificate work budget"),
            "control-net overlap must reject even when fixed stations look clear"
        );
    }
}
