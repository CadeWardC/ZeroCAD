//! Analytic planar arrangement and DCEL face extraction.
//!
//! Curves are intersected in parameter space and remain analytic after every
//! split. Polygon sampling appears only in the containment helper used to
//! assign nested loops as holes; it never determines intersections or edges.

use core::f64::consts::{PI, TAU};
use core::fmt;
use std::collections::HashMap;

use openrcad_foundation::Pnt2d;
use openrcad_geom2d::{Curve2d, CurveKind2d, CurveSpan, Ellipse2d, GeomCurve2d, Line2d};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArrangementOptions {
    pub tolerance: f64,
    pub root_subdivisions: usize,
}

impl Default for ArrangementOptions {
    fn default() -> Self {
        Self {
            tolerance: 1.0e-9,
            root_subdivisions: 256,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ArrangementError {
    InvalidTolerance,
    InvalidSpan {
        index: usize,
    },
    UnsupportedPair {
        first: usize,
        second: usize,
        first_kind: CurveKind2d,
        second_kind: CurveKind2d,
    },
}

impl fmt::Display for ArrangementError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerance => {
                formatter.write_str("arrangement tolerance must be finite and positive")
            }
            Self::InvalidSpan { index } => {
                write!(formatter, "curve span {index} is invalid or degenerate")
            }
            Self::UnsupportedPair {
                first,
                second,
                first_kind,
                second_kind,
            } => write!(
                formatter,
                "curve pair {first}/{second} ({first_kind:?}/{second_kind:?}) is not supported"
            ),
        }
    }
}

impl std::error::Error for ArrangementError {}

#[derive(Clone, Debug, PartialEq)]
pub struct ArrangementLoop<P> {
    pub spans: Vec<CurveSpan<P>>,
    pub signed_area: f64,
}

impl<P> ArrangementLoop<P> {
    pub fn area(&self) -> f64 {
        self.signed_area.abs()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArrangementRegion<P> {
    pub outer: ArrangementLoop<P>,
    pub holes: Vec<ArrangementLoop<P>>,
    pub area: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Arrangement<P> {
    pub regions: Vec<ArrangementRegion<P>>,
    pub vertex_count: usize,
    pub edge_count: usize,
}

/// One analytic intersection between two bounded curve spans.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveSpanIntersection {
    pub first_parameter: f64,
    pub second_parameter: f64,
}

/// Pairwise intersection result shared by arrangement construction and editing
/// operations such as Trim. `coincident` distinguishes an overlapping locus
/// from a finite set of crossing points.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveSpanIntersections {
    pub points: Vec<CurveSpanIntersection>,
    pub coincident: bool,
}

#[derive(Clone, Copy, Debug)]
struct Intersection {
    first: f64,
    second: f64,
}

#[derive(Clone)]
struct AtomicSpan<P> {
    span: CurveSpan<P>,
    from: usize,
    to: usize,
}

#[derive(Clone, Copy)]
struct HalfEdge {
    atomic: usize,
    from: usize,
    to: usize,
    twin: usize,
    reversed: bool,
}

pub fn arrange_curve_spans<P: Clone>(
    spans: &[CurveSpan<P>],
    options: ArrangementOptions,
) -> Result<Arrangement<P>, ArrangementError> {
    if !options.tolerance.is_finite() || options.tolerance <= 0.0 {
        return Err(ArrangementError::InvalidTolerance);
    }
    for (index, span) in spans.iter().enumerate() {
        if !span.is_finite()
            || span.parameter_length() <= options.tolerance
            || span.start().distance(&span.end()) <= options.tolerance
                && !is_complete_period(span, options.tolerance)
        {
            return Err(ArrangementError::InvalidSpan { index });
        }
    }

    let mut cuts: Vec<Vec<f64>> = spans
        .iter()
        .map(|span| {
            let mut params = vec![span.first, span.last];
            if is_complete_period(span, options.tolerance) {
                for step in 1..4 {
                    params.push(span.first + (span.last - span.first) * step as f64 / 4.0);
                }
            }
            params
        })
        .collect();

    for first in 0..spans.len() {
        for second in first + 1..spans.len() {
            let intersections = span_intersections(&spans[first], &spans[second], options)
                .map_err(|()| ArrangementError::UnsupportedPair {
                    first,
                    second,
                    first_kind: spans[first].kind(),
                    second_kind: spans[second].kind(),
                })?;
            for intersection in intersections {
                push_unique_parameter(&mut cuts[first], intersection.first, options.tolerance);
                push_unique_parameter(&mut cuts[second], intersection.second, options.tolerance);
            }
        }
    }

    let mut vertices = Vec::new();
    let mut atomics = Vec::new();
    let mut unique_edges: HashMap<(usize, usize, i64, i64), usize> = HashMap::new();
    for (span_index, span) in spans.iter().enumerate() {
        cuts[span_index].sort_by(f64::total_cmp);
        cuts[span_index].dedup_by(|a, b| (*a - *b).abs() <= options.tolerance);
        if span.last < span.first {
            cuts[span_index].reverse();
        }
        for interval in cuts[span_index].windows(2) {
            let (first, last) = (interval[0], interval[1]);
            if (last - first).abs() <= options.tolerance {
                continue;
            }
            let candidate = span.subspan(first, last);
            let start = candidate.start();
            let end = candidate.end();
            if start.distance(&end) <= options.tolerance {
                continue;
            }
            let from = add_vertex(start, &mut vertices, options.tolerance);
            let to = add_vertex(end, &mut vertices, options.tolerance);
            if from == to {
                continue;
            }
            let midpoint = candidate.curve.point((first + last) * 0.5);
            let (low, high) = if from < to { (from, to) } else { (to, from) };
            let key = (
                low,
                high,
                quantize(midpoint.x(), options.tolerance),
                quantize(midpoint.y(), options.tolerance),
            );
            if unique_edges.contains_key(&key) {
                continue;
            }
            let index = atomics.len();
            unique_edges.insert(key, index);
            atomics.push(AtomicSpan {
                span: candidate,
                from,
                to,
            });
        }
    }

    let half_edges = build_half_edges(&atomics);
    let next = compute_next(&half_edges, &atomics, &vertices);
    let cycles = walk_cycles(&half_edges, &next);
    let minimum_area = options.tolerance * options.tolerance;
    let mut loops = Vec::new();
    for cycle in cycles {
        let mut cycle_spans = Vec::with_capacity(cycle.len());
        for half in cycle {
            let edge = half_edges[half];
            let mut span = atomics[edge.atomic].span.clone();
            if edge.reversed {
                span = span.reversed();
            }
            cycle_spans.push(span);
        }
        let signed_area: f64 = cycle_spans.iter().map(span_signed_area).sum();
        if signed_area > minimum_area {
            loops.push(ArrangementLoop {
                spans: cycle_spans,
                signed_area,
            });
        }
    }

    let regions = assign_holes(loops, options.tolerance);
    Ok(Arrangement {
        regions,
        vertex_count: vertices.len(),
        edge_count: atomics.len(),
    })
}

/// Intersect two bounded analytic spans using the same tolerance and root
/// isolation policy as [`arrange_curve_spans`].
pub fn intersect_curve_spans<A, B>(
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    options: ArrangementOptions,
) -> Result<CurveSpanIntersections, ArrangementError> {
    if !options.tolerance.is_finite() || options.tolerance <= 0.0 {
        return Err(ArrangementError::InvalidTolerance);
    }
    let first_invalid = !first.is_finite()
        || first.parameter_length() <= options.tolerance
        || first.start().distance(&first.end()) <= options.tolerance
            && !is_complete_period(first, options.tolerance);
    if first_invalid {
        return Err(ArrangementError::InvalidSpan { index: 0 });
    }
    let second_invalid = !second.is_finite()
        || second.parameter_length() <= options.tolerance
        || second.start().distance(&second.end()) <= options.tolerance
            && !is_complete_period(second, options.tolerance);
    if second_invalid {
        return Err(ArrangementError::InvalidSpan { index: 1 });
    }
    let intersections = span_intersections(first, second, options).map_err(|()| {
        ArrangementError::UnsupportedPair {
            first: 0,
            second: 1,
            first_kind: first.kind(),
            second_kind: second.kind(),
        }
    })?;
    let coincident = spans_are_coincident(first, second, &intersections, options.tolerance);
    Ok(CurveSpanIntersections {
        points: intersections
            .into_iter()
            .map(|intersection| CurveSpanIntersection {
                first_parameter: intersection.first,
                second_parameter: intersection.second,
            })
            .collect(),
        coincident,
    })
}

fn is_complete_period<P>(span: &CurveSpan<P>, tolerance: f64) -> bool {
    span.curve.is_periodic()
        && (span.parameter_length() - span.curve.period()).abs()
            <= tolerance.max(span.curve.period().abs() * 1.0e-9)
}

fn span_intersections<A, B>(
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    options: ArrangementOptions,
) -> Result<Vec<Intersection>, ()> {
    let raw = match (&first.curve, &second.curve) {
        (GeomCurve2d::Line(a), GeomCurve2d::Line(b)) => {
            line_line(a, b, first, second, options.tolerance)
        }
        (GeomCurve2d::Line(line), GeomCurve2d::Circle(circle)) => line_ellipse(
            line,
            circle_as_ellipse(circle),
            first,
            second,
            options.tolerance,
        ),
        (GeomCurve2d::Circle(circle), GeomCurve2d::Line(line)) => swap_intersections(line_ellipse(
            line,
            circle_as_ellipse(circle),
            second,
            first,
            options.tolerance,
        )),
        (GeomCurve2d::Line(line), GeomCurve2d::Ellipse(ellipse)) => {
            line_ellipse(line, *ellipse, first, second, options.tolerance)
        }
        (GeomCurve2d::Ellipse(ellipse), GeomCurve2d::Line(line)) => swap_intersections(
            line_ellipse(line, *ellipse, second, first, options.tolerance),
        ),
        (GeomCurve2d::Circle(a), GeomCurve2d::Circle(b)) => {
            circle_circle(a, b, first, second, options.tolerance)
        }
        (GeomCurve2d::Circle(_), GeomCurve2d::Ellipse(_))
        | (GeomCurve2d::Ellipse(_), GeomCurve2d::Circle(_))
        | (GeomCurve2d::Ellipse(_), GeomCurve2d::Ellipse(_)) => conic_conic(first, second, options),
        (GeomCurve2d::Line(line), GeomCurve2d::BSpline(spline)) => {
            line_spline(line, spline, first, second, options)
        }
        (GeomCurve2d::BSpline(spline), GeomCurve2d::Line(line)) => {
            swap_intersections(line_spline(line, spline, second, first, options))
        }
        (GeomCurve2d::BSpline(_), _) | (_, GeomCurve2d::BSpline(_)) => return Err(()),
        _ => return Err(()),
    };
    Ok(deduplicate_intersections(raw, options.tolerance))
}

fn spans_are_coincident<A, B>(
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    intersections: &[Intersection],
    tolerance: f64,
) -> bool {
    match (&first.curve, &second.curve) {
        (GeomCurve2d::Line(a), GeomCurve2d::Line(b)) => {
            let p = a.location();
            let q = b.location();
            let r = a.direction();
            let s = b.direction();
            if cross(r.x(), r.y(), s.x(), s.y()).abs() > tolerance
                || cross(q.x() - p.x(), q.y() - p.y(), r.x(), r.y()).abs() > tolerance
            {
                return false;
            }
            intersections.iter().enumerate().any(|(index, first_hit)| {
                intersections[index + 1..].iter().any(|second_hit| {
                    first
                        .curve
                        .point(first_hit.first)
                        .distance(&first.curve.point(second_hit.first))
                        > tolerance
                })
            })
        }
        (GeomCurve2d::Circle(a), GeomCurve2d::Circle(b)) => {
            a.center().distance(&b.center()) <= tolerance
                && (a.radius() - b.radius()).abs() <= tolerance
                && !intersections.is_empty()
        }
        _ => false,
    }
}

fn line_line<A, B>(
    first_line: &Line2d,
    second_line: &Line2d,
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    tolerance: f64,
) -> Vec<Intersection> {
    let p = first_line.location();
    let q = second_line.location();
    let r = first_line.direction();
    let s = second_line.direction();
    let denominator = cross(r.x(), r.y(), s.x(), s.y());
    let qmp = (q.x() - p.x(), q.y() - p.y());
    if denominator.abs() > tolerance {
        let first_parameter = cross(qmp.0, qmp.1, s.x(), s.y()) / denominator;
        let second_parameter = cross(qmp.0, qmp.1, r.x(), r.y()) / denominator;
        return (in_span(first_parameter, first, tolerance)
            && in_span(second_parameter, second, tolerance))
        .then_some(Intersection {
            first: first_parameter,
            second: second_parameter,
        })
        .into_iter()
        .collect();
    }
    if cross(qmp.0, qmp.1, r.x(), r.y()).abs() > tolerance {
        return Vec::new();
    }

    let mut intersections = Vec::new();
    for parameter in [first.first, first.last] {
        let point = first_line.point(parameter);
        let other = line_parameter(second_line, point);
        if in_span(other, second, tolerance) {
            intersections.push(Intersection {
                first: parameter,
                second: other,
            });
        }
    }
    for parameter in [second.first, second.last] {
        let point = second_line.point(parameter);
        let other = line_parameter(first_line, point);
        if in_span(other, first, tolerance) {
            intersections.push(Intersection {
                first: other,
                second: parameter,
            });
        }
    }
    intersections
}

fn line_ellipse<A, B>(
    line: &Line2d,
    ellipse: Ellipse2d,
    line_span: &CurveSpan<A>,
    ellipse_span: &CurveSpan<B>,
    tolerance: f64,
) -> Vec<Intersection> {
    let center = ellipse.center();
    let frame = ellipse.position();
    let x_axis = frame.x_direction();
    let y_axis = frame.y_direction();
    let origin = line.location();
    let direction = line.direction();
    let delta = (origin.x() - center.x(), origin.y() - center.y());
    let project = |x: f64, y: f64, axis_x: f64, axis_y: f64| x * axis_x + y * axis_y;
    let (major, minor) = (ellipse.major_radius(), ellipse.minor_radius());
    if major <= tolerance || minor <= tolerance {
        return Vec::new();
    }
    let px = project(delta.0, delta.1, x_axis.x(), x_axis.y());
    let py = project(delta.0, delta.1, y_axis.x(), y_axis.y());
    let dx = project(direction.x(), direction.y(), x_axis.x(), x_axis.y());
    let dy = project(direction.x(), direction.y(), y_axis.x(), y_axis.y());
    let qa = dx * dx / (major * major) + dy * dy / (minor * minor);
    let qb = 2.0 * (px * dx / (major * major) + py * dy / (minor * minor));
    let qc = px * px / (major * major) + py * py / (minor * minor) - 1.0;
    // Treat a tolerance-close grazing line as one tangent intersection. With
    // persisted f32 sketch coordinates, two edges that the arrangement already
    // regards as coincident can differ by a few ulp. If one is meant to be
    // tangent to a circle, that microscopic offset otherwise turns the tangent
    // into two roots about sqrt(radius * offset) apart—large enough to create a
    // long, needle-thin face even though the physical penetration is below the
    // modeling tolerance.
    //
    // Measure the miss in model space, not by comparing the quadratic
    // discriminant with a dimensionally unrelated epsilon. The radial
    // parameter is sufficient here because the candidate is already within one
    // tolerance of the ellipse; it also works for ordinary ellipses.
    if qa > f64::EPSILON {
        let line_parameter = -qb / (2.0 * qa);
        if in_span(line_parameter, line_span, tolerance) {
            let point = line.point(line_parameter);
            if let Some(ellipse_parameter) =
                ellipse_parameter(ellipse, point, ellipse_span, tolerance)
            {
                let on_ellipse = ellipse.point(ellipse_parameter);
                if point.distance(&on_ellipse) <= tolerance {
                    return vec![Intersection {
                        first: line_parameter,
                        second: ellipse_parameter,
                    }];
                }
            }
        }
    }
    quadratic_roots(qa, qb, qc, tolerance)
        .into_iter()
        .filter_map(|line_parameter| {
            let point = line.point(line_parameter);
            let ellipse_parameter = ellipse_parameter(ellipse, point, ellipse_span, tolerance)?;
            (in_span(line_parameter, line_span, tolerance)
                && in_span(ellipse_parameter, ellipse_span, tolerance))
            .then_some(Intersection {
                first: line_parameter,
                second: ellipse_parameter,
            })
        })
        .collect()
}

fn circle_circle<A, B>(
    first_circle: &openrcad_geom2d::Circle2d,
    second_circle: &openrcad_geom2d::Circle2d,
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    tolerance: f64,
) -> Vec<Intersection> {
    let a = first_circle.center();
    let b = second_circle.center();
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let distance = dx.hypot(dy);
    let (r0, r1) = (first_circle.radius(), second_circle.radius());
    if distance <= tolerance && (r0 - r1).abs() <= tolerance {
        let mut coincident = Vec::new();
        for parameter in [first.first, first.last] {
            if let Some(other) = circle_parameter(
                *second_circle,
                first_circle.point(parameter),
                second,
                tolerance,
            ) {
                coincident.push(Intersection {
                    first: parameter,
                    second: other,
                });
            }
        }
        for parameter in [second.first, second.last] {
            if let Some(other) = circle_parameter(
                *first_circle,
                second_circle.point(parameter),
                first,
                tolerance,
            ) {
                coincident.push(Intersection {
                    first: other,
                    second: parameter,
                });
            }
        }
        return coincident;
    }
    if distance <= tolerance
        || distance > r0 + r1 + tolerance
        || distance < (r0 - r1).abs() - tolerance
    {
        return Vec::new();
    }
    let along = (r0 * r0 - r1 * r1 + distance * distance) / (2.0 * distance);
    let height_sq = (r0 * r0 - along * along).max(0.0);
    let height = height_sq.sqrt();
    let base = (a.x() + along * dx / distance, a.y() + along * dy / distance);
    let perpendicular = (-dy / distance, dx / distance);
    let points = if height <= tolerance {
        vec![Pnt2d::new(base.0, base.1)]
    } else {
        vec![
            Pnt2d::new(
                base.0 + height * perpendicular.0,
                base.1 + height * perpendicular.1,
            ),
            Pnt2d::new(
                base.0 - height * perpendicular.0,
                base.1 - height * perpendicular.1,
            ),
        ]
    };
    points
        .into_iter()
        .filter_map(|point| {
            Some(Intersection {
                first: circle_parameter(*first_circle, point, first, tolerance)?,
                second: circle_parameter(*second_circle, point, second, tolerance)?,
            })
        })
        .collect()
}

fn conic_conic<A, B>(
    first: &CurveSpan<A>,
    second: &CurveSpan<B>,
    options: ArrangementOptions,
) -> Vec<Intersection> {
    let roots = isolate_roots(
        |parameter| conic_implicit(&second.curve, first.curve.point(parameter)),
        first.first,
        first.last,
        options.root_subdivisions,
        options.tolerance.max(1.0e-11),
    );
    roots
        .into_iter()
        .filter_map(|parameter| {
            let point = first.curve.point(parameter);
            let second_parameter =
                conic_parameter(&second.curve, point, second, options.tolerance)?;
            Some(Intersection {
                first: parameter,
                second: second_parameter,
            })
        })
        .collect()
}

fn line_spline<A, B>(
    line: &Line2d,
    spline: &openrcad_geom2d::BSplineCurve2d,
    line_span: &CurveSpan<A>,
    spline_span: &CurveSpan<B>,
    options: ArrangementOptions,
) -> Vec<Intersection> {
    let origin = line.location();
    let direction = line.direction();
    isolate_roots(
        |parameter| {
            let point = spline.point(parameter);
            cross(
                direction.x(),
                direction.y(),
                point.x() - origin.x(),
                point.y() - origin.y(),
            )
        },
        spline_span.first,
        spline_span.last,
        options.root_subdivisions,
        options.tolerance,
    )
    .into_iter()
    .filter_map(|spline_parameter| {
        let line_parameter = line_parameter(line, spline.point(spline_parameter));
        in_span(line_parameter, line_span, options.tolerance).then_some(Intersection {
            first: line_parameter,
            second: spline_parameter,
        })
    })
    .collect()
}

fn circle_as_ellipse(circle: &openrcad_geom2d::Circle2d) -> Ellipse2d {
    Ellipse2d::new(circle.position(), circle.radius(), circle.radius())
}

fn conic_implicit(curve: &GeomCurve2d, point: Pnt2d) -> f64 {
    let ellipse = match curve {
        GeomCurve2d::Circle(circle) => circle_as_ellipse(circle),
        GeomCurve2d::Ellipse(ellipse) => *ellipse,
        _ => return f64::NAN,
    };
    let delta = (
        point.x() - ellipse.center().x(),
        point.y() - ellipse.center().y(),
    );
    let frame = ellipse.position();
    let x = delta.0 * frame.x_direction().x() + delta.1 * frame.x_direction().y();
    let y = delta.0 * frame.y_direction().x() + delta.1 * frame.y_direction().y();
    (x / ellipse.major_radius()).powi(2) + (y / ellipse.minor_radius()).powi(2) - 1.0
}

fn conic_parameter<P>(
    curve: &GeomCurve2d,
    point: Pnt2d,
    span: &CurveSpan<P>,
    tolerance: f64,
) -> Option<f64> {
    match curve {
        GeomCurve2d::Circle(circle) => circle_parameter(*circle, point, span, tolerance),
        GeomCurve2d::Ellipse(ellipse) => ellipse_parameter(*ellipse, point, span, tolerance),
        _ => None,
    }
}

fn circle_parameter<P>(
    circle: openrcad_geom2d::Circle2d,
    point: Pnt2d,
    span: &CurveSpan<P>,
    tolerance: f64,
) -> Option<f64> {
    ellipse_parameter(circle_as_ellipse(&circle), point, span, tolerance)
}

fn ellipse_parameter<P>(
    ellipse: Ellipse2d,
    point: Pnt2d,
    span: &CurveSpan<P>,
    tolerance: f64,
) -> Option<f64> {
    let delta = (
        point.x() - ellipse.center().x(),
        point.y() - ellipse.center().y(),
    );
    let frame = ellipse.position();
    let x = (delta.0 * frame.x_direction().x() + delta.1 * frame.x_direction().y())
        / ellipse.major_radius();
    let y = (delta.0 * frame.y_direction().x() + delta.1 * frame.y_direction().y())
        / ellipse.minor_radius();
    periodic_parameter(y.atan2(x), span, TAU, tolerance)
}

fn periodic_parameter<P>(
    base: f64,
    span: &CurveSpan<P>,
    period: f64,
    tolerance: f64,
) -> Option<f64> {
    let midpoint = (span.first + span.last) * 0.5;
    (-3..=3)
        .map(|turn| base + turn as f64 * period)
        .filter(|parameter| in_span(*parameter, span, tolerance))
        .min_by(|a, b| (a - midpoint).abs().total_cmp(&(b - midpoint).abs()))
}

fn line_parameter(line: &Line2d, point: Pnt2d) -> f64 {
    let delta = (
        point.x() - line.location().x(),
        point.y() - line.location().y(),
    );
    delta.0 * line.direction().x() + delta.1 * line.direction().y()
}

fn quadratic_roots(a: f64, b: f64, c: f64, tolerance: f64) -> Vec<f64> {
    if a.abs() <= tolerance {
        return (b.abs() > tolerance)
            .then_some(-c / b)
            .into_iter()
            .collect();
    }
    let discriminant = b.mul_add(b, -4.0 * a * c);
    if discriminant < -tolerance {
        return Vec::new();
    }
    if discriminant.abs() <= tolerance {
        return vec![-b / (2.0 * a)];
    }
    let root = discriminant.sqrt();
    vec![(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
}

fn isolate_roots(
    function: impl Fn(f64) -> f64,
    first: f64,
    last: f64,
    subdivisions: usize,
    value_tolerance: f64,
) -> Vec<f64> {
    let subdivisions = subdivisions.clamp(16, 4096);
    let mut samples = Vec::with_capacity(subdivisions + 1);
    for index in 0..=subdivisions {
        let parameter = first + (last - first) * index as f64 / subdivisions as f64;
        samples.push((parameter, function(parameter)));
    }
    let mut roots = Vec::new();
    for window in samples.windows(2) {
        let ((mut a, mut fa), (mut b, fb)) = (window[0], window[1]);
        if !fa.is_finite() || !fb.is_finite() {
            continue;
        }
        if fa.abs() <= value_tolerance {
            roots.push(a);
        }
        if fa.signum() == fb.signum() && fb.abs() > value_tolerance {
            continue;
        }
        let mut right_value = fb;
        for _ in 0..64 {
            let midpoint = (a + b) * 0.5;
            let value = function(midpoint);
            if !value.is_finite() || value.abs() <= value_tolerance || (b - a).abs() <= 1.0e-12 {
                a = midpoint;
                b = midpoint;
                break;
            }
            if fa.signum() == value.signum() {
                a = midpoint;
                fa = value;
            } else {
                b = midpoint;
                right_value = value;
            }
        }
        let _ = right_value;
        roots.push((a + b) * 0.5);
    }
    if let Some(&(parameter, value)) = samples.last() {
        if value.abs() <= value_tolerance {
            roots.push(parameter);
        }
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|a, b| (*a - *b).abs() <= 1.0e-8);
    roots
}

fn in_span<P>(parameter: f64, span: &CurveSpan<P>, tolerance: f64) -> bool {
    let low = span.first.min(span.last) - tolerance;
    let high = span.first.max(span.last) + tolerance;
    parameter >= low && parameter <= high
}

fn swap_intersections(intersections: Vec<Intersection>) -> Vec<Intersection> {
    intersections
        .into_iter()
        .map(|intersection| Intersection {
            first: intersection.second,
            second: intersection.first,
        })
        .collect()
}

fn deduplicate_intersections(
    mut intersections: Vec<Intersection>,
    tolerance: f64,
) -> Vec<Intersection> {
    intersections.sort_by(|a, b| {
        a.first
            .total_cmp(&b.first)
            .then(a.second.total_cmp(&b.second))
    });
    intersections.dedup_by(|a, b| {
        (a.first - b.first).abs() <= tolerance && (a.second - b.second).abs() <= tolerance
    });
    intersections
}

fn push_unique_parameter(parameters: &mut Vec<f64>, parameter: f64, tolerance: f64) {
    if parameter.is_finite()
        && !parameters
            .iter()
            .any(|existing| (*existing - parameter).abs() <= tolerance)
    {
        parameters.push(parameter);
    }
}

fn add_vertex(point: Pnt2d, vertices: &mut Vec<Pnt2d>, tolerance: f64) -> usize {
    if let Some(index) = vertices
        .iter()
        .position(|candidate| point.distance(candidate) <= tolerance)
    {
        index
    } else {
        vertices.push(point);
        vertices.len() - 1
    }
}

fn quantize(value: f64, tolerance: f64) -> i64 {
    let scaled = (value / tolerance).round();
    scaled.clamp(i64::MIN as f64, i64::MAX as f64) as i64
}

fn build_half_edges<P>(atomics: &[AtomicSpan<P>]) -> Vec<HalfEdge> {
    let mut half_edges = Vec::with_capacity(atomics.len() * 2);
    for (atomic, span) in atomics.iter().enumerate() {
        let index = half_edges.len();
        half_edges.push(HalfEdge {
            atomic,
            from: span.from,
            to: span.to,
            twin: index + 1,
            reversed: false,
        });
        half_edges.push(HalfEdge {
            atomic,
            from: span.to,
            to: span.from,
            twin: index,
            reversed: true,
        });
    }
    half_edges
}

fn compute_next<P>(
    half_edges: &[HalfEdge],
    atomics: &[AtomicSpan<P>],
    vertices: &[Pnt2d],
) -> Vec<usize> {
    let mut outgoing = vec![Vec::new(); vertices.len()];
    for (index, edge) in half_edges.iter().enumerate() {
        outgoing[edge.from].push(index);
    }
    for edges in &mut outgoing {
        edges.sort_by(|first, second| {
            half_edge_angle(half_edges[*first], atomics, vertices).total_cmp(&half_edge_angle(
                half_edges[*second],
                atomics,
                vertices,
            ))
        });
        // Tangential contact leaves several half-edges on the SAME outgoing
        // tangent — a circle resting on the line it touches, or two arcs
        // meeting smoothly. Their angles are then equal (often bit-identical),
        // the sort above ties arbitrarily on input order, and `next` below can
        // step from one curve onto the other. The face walk crosses between the
        // regions either side of the contact and fuses them into one.
        //
        // Break those ties by how each curve *leaves* the shared tangent:
        // sample a short way along and compare signed lateral offset. That
        // orders them by curvature using only `point`/`d1`, which every curve
        // kind implements — no second derivative required. For edges whose
        // angles merely differ slightly the lateral offset has the same sign as
        // the angle difference, so widening the tie band cannot reorder a
        // genuinely non-degenerate fan.
        resolve_tangential_ties(edges, half_edges, atomics, vertices);
    }
    let mut position = vec![0; half_edges.len()];
    for edges in &outgoing {
        for (index, edge) in edges.iter().enumerate() {
            position[*edge] = index;
        }
    }
    let mut next = vec![0; half_edges.len()];
    for (index, edge) in half_edges.iter().enumerate() {
        let at_destination = &outgoing[edge.to];
        let twin_position = position[edge.twin];
        next[index] =
            at_destination[(twin_position + at_destination.len() - 1) % at_destination.len()];
    }
    next
}

fn half_edge_angle<P>(edge: HalfEdge, atomics: &[AtomicSpan<P>], vertices: &[Pnt2d]) -> f64 {
    let span = &atomics[edge.atomic].span;
    let tangent = if edge.reversed {
        let tangent = span.end_tangent();
        (-tangent.x(), -tangent.y())
    } else {
        let tangent = span.start_tangent();
        (tangent.x(), tangent.y())
    };
    let angle = if tangent.0.hypot(tangent.1) > 1.0e-15 {
        tangent.1.atan2(tangent.0)
    } else {
        let from = vertices[edge.from];
        let to = vertices[edge.to];
        (to.y() - from.y()).atan2(to.x() - from.x())
    };
    // Canonicalize the ±π seam. atan2 maps (-x, +0.0) to +π but (-x, -0.0) to
    // -π, and REVERSED half-edges of axis-aligned lines manufacture the -0.0
    // systematically (negating a +0.0 tangent). Left unfixed, two half-edges
    // leaving a vertex in the identical -x direction land at opposite ends of
    // the sorted fan, the tangential tie between them never forms, and the
    // resulting cyclic order is wrong — the face walk then swallows entire
    // faces as there-and-back excursions (a tangent contact at a shared edge
    // lost a whole rectangle this way).
    if angle == -PI {
        PI
    } else {
        angle
    }
}

/// Angular width within which two outgoing half-edges are treated as sharing a
/// tangent. Exact tangency makes the angles bit-identical; f32-rounded sketch
/// input spreads them by a few ulp, so the band must be wider than zero.
const TANGENT_TIE_RADIANS: f64 = 1.0e-6;

/// The half-edge's parameter interval, oriented along the direction of travel
/// (a reversed half-edge runs from `last` back to `first`).
fn half_edge_domain<P>(edge: HalfEdge, atomics: &[AtomicSpan<P>]) -> (f64, f64) {
    let span = &atomics[edge.atomic].span;
    if edge.reversed {
        (span.last, span.first)
    } else {
        (span.first, span.last)
    }
}

/// Approximate arc length, from the parameter span and the speed at the start.
/// Exact for lines and circles (the kinds that actually produce tangential
/// contact); for a spline it only has to be the right order of magnitude, since
/// it just sets how far along to probe.
fn half_edge_arc_length<P>(edge: HalfEdge, atomics: &[AtomicSpan<P>]) -> f64 {
    let span = &atomics[edge.atomic].span;
    let (start, end) = half_edge_domain(edge, atomics);
    let (_, derivative) = span.curve.d1(start);
    let speed = derivative.x().hypot(derivative.y());
    ((end - start).abs() * speed).max(f64::MIN_POSITIVE)
}

/// Signed lateral offset of the curve from `reference` after travelling roughly
/// `probe` arc length from the vertex. Positive means it bends to the left of
/// the shared tangent, which is the direction of increasing angle — so ordering
/// by this value continues the angle sort rather than fighting it.
fn half_edge_lateral<P>(
    edge: HalfEdge,
    atomics: &[AtomicSpan<P>],
    vertices: &[Pnt2d],
    reference: (f64, f64),
    probe: f64,
) -> f64 {
    let span = &atomics[edge.atomic].span;
    let (start, end) = half_edge_domain(edge, atomics);
    let fraction = (probe / half_edge_arc_length(edge, atomics)).clamp(0.0, 1.0);
    // Atomic spans carry no interior crossings, so sampling anywhere inside one
    // stays in the same face and cannot jump past another vertex.
    let sample = span.curve.point(start + (end - start) * fraction);
    let origin = vertices[edge.from];
    let (dx, dy) = (sample.x() - origin.x(), sample.y() - origin.y());
    reference.0 * dy - reference.1 * dx
}

/// Re-order runs of `edges` that leave the vertex on a shared tangent, in place.
/// `edges` must already be sorted by angle.
fn resolve_tangential_ties<P>(
    edges: &mut [usize],
    half_edges: &[HalfEdge],
    atomics: &[AtomicSpan<P>],
    vertices: &[Pnt2d],
) {
    let angle = |edge: usize| half_edge_angle(half_edges[edge], atomics, vertices);
    // The fan is CIRCULAR but the sorted list is linear, cut at the ±π seam. A
    // tangential tie whose members straddle that seam (one at π−ε, one at
    // −π+ε) would be invisible to the linear run scan below. Rotating the list
    // so it begins just after the LARGEST angular gap moves the cut into open
    // space where no tie can straddle it — a rotation changes nothing else,
    // since only cyclic adjacency feeds the next-pointer construction. If
    // every edge shares one tangent there is no gap to hide the seam in; the
    // run scan then covers the whole list in one pass, which is exactly right.
    let count = edges.len();
    if count > 1 {
        let mut split = 0;
        let mut largest = f64::MIN;
        for index in 0..count {
            let here = angle(edges[index]);
            let next = if index + 1 == count {
                angle(edges[0]) + TAU
            } else {
                angle(edges[index + 1])
            };
            if next - here > largest {
                largest = next - here;
                split = (index + 1) % count;
            }
        }
        if largest > TANGENT_TIE_RADIANS {
            edges.rotate_left(split);
        }
    }
    // After the rotation the list ascends in angle except for ONE descending
    // transition where the sorted order wraps (the old list head). Compare
    // consecutive angles CIRCULARLY: rem_euclid folds that wrap into its true
    // angular gap — the largest at the vertex, so it always terminates a run —
    // while ordinary ascending pairs are unaffected. A raw difference would be
    // hugely negative at the wrap and `<=` would silently fuse two unrelated
    // runs.
    let angles: Vec<f64> = edges.iter().map(|&edge| angle(edge)).collect();
    let circular_gap =
        |previous: usize, current: usize| (angles[current] - angles[previous]).rem_euclid(TAU);
    let mut start = 0;
    while start < edges.len() {
        let mut end = start + 1;
        while end < edges.len() && circular_gap(end - 1, end) <= TANGENT_TIE_RADIANS {
            end += 1;
        }
        if end - start > 1 {
            let run = &mut edges[start..end];
            // Compare every edge in the run at the same distance out, short
            // enough to stay well inside the shortest of them.
            let probe = run
                .iter()
                .map(|edge| half_edge_arc_length(half_edges[*edge], atomics))
                .fold(f64::INFINITY, f64::min)
                * 0.25;
            let first = half_edges[run[0]];
            let reference = {
                let theta = half_edge_angle(first, atomics, vertices);
                (theta.cos(), theta.sin())
            };
            run.sort_by(|first, second| {
                half_edge_lateral(half_edges[*first], atomics, vertices, reference, probe)
                    .total_cmp(&half_edge_lateral(
                        half_edges[*second],
                        atomics,
                        vertices,
                        reference,
                        probe,
                    ))
            });
        }
        start = end;
    }
}

fn walk_cycles(half_edges: &[HalfEdge], next: &[usize]) -> Vec<Vec<usize>> {
    let mut visited = vec![false; half_edges.len()];
    let mut cycles = Vec::new();
    for start in 0..half_edges.len() {
        if visited[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut current = start;
        loop {
            if visited[current] {
                if current == start && cycle.len() >= 2 {
                    cycles.push(cycle);
                }
                break;
            }
            visited[current] = true;
            cycle.push(current);
            current = next[current];
        }
    }
    cycles
}

fn span_signed_area<P>(span: &CurveSpan<P>) -> f64 {
    match &span.curve {
        GeomCurve2d::Line(_) => {
            let start = span.start();
            let end = span.end();
            0.5 * cross(start.x(), start.y(), end.x(), end.y())
        }
        GeomCurve2d::Circle(circle) => ellipse_span_area(
            circle.center(),
            circle.position().x_direction(),
            circle.position().y_direction(),
            circle.radius(),
            circle.radius(),
            span.first,
            span.last,
        ),
        GeomCurve2d::Ellipse(ellipse) => ellipse_span_area(
            ellipse.center(),
            ellipse.position().x_direction(),
            ellipse.position().y_direction(),
            ellipse.major_radius(),
            ellipse.minor_radius(),
            span.first,
            span.last,
        ),
        _ => quadrature_area(span),
    }
}

fn ellipse_span_area(
    center: Pnt2d,
    x_axis: openrcad_foundation::Dir2d,
    y_axis: openrcad_foundation::Dir2d,
    major: f64,
    minor: f64,
    first: f64,
    last: f64,
) -> f64 {
    let a = (major * x_axis.x(), major * x_axis.y());
    let b = (minor * y_axis.x(), minor * y_axis.y());
    0.5 * (cross(center.x(), center.y(), a.0, a.1) * (last.cos() - first.cos())
        + cross(center.x(), center.y(), b.0, b.1) * (last.sin() - first.sin())
        + cross(a.0, a.1, b.0, b.1) * (last - first))
}

fn quadrature_area<P>(span: &CurveSpan<P>) -> f64 {
    const STEPS: usize = 64;
    let step = (span.last - span.first) / STEPS as f64;
    let integrand = |parameter: f64| {
        let (point, tangent) = span.curve.d1(parameter);
        cross(point.x(), point.y(), tangent.x(), tangent.y())
    };
    let mut sum = integrand(span.first) + integrand(span.last);
    for index in 1..STEPS {
        let weight = if index % 2 == 0 { 2.0 } else { 4.0 };
        sum += weight * integrand(span.first + step * index as f64);
    }
    0.5 * sum * step / 3.0
}

fn assign_holes<P: Clone>(
    loops: Vec<ArrangementLoop<P>>,
    tolerance: f64,
) -> Vec<ArrangementRegion<P>> {
    let polygons: Vec<Vec<Pnt2d>> = loops.iter().map(sample_loop_for_containment).collect();
    let probes: Vec<Pnt2d> = polygons
        .iter()
        .map(|polygon| interior_point(polygon))
        .collect();
    let mut parents: Vec<Option<usize>> = vec![None; loops.len()];
    for child in 0..loops.len() {
        for candidate in 0..loops.len() {
            if child == candidate || loops[candidate].area() <= loops[child].area() + tolerance {
                continue;
            }
            if point_in_polygon(probes[child], &polygons[candidate])
                && parents[child]
                    .is_none_or(|parent| loops[candidate].area() < loops[parent].area())
            {
                parents[child] = Some(candidate);
            }
        }
    }
    loops
        .iter()
        .enumerate()
        .map(|(index, outer)| {
            let holes: Vec<_> = loops
                .iter()
                .enumerate()
                .filter(|(child, _)| parents[*child] == Some(index))
                .map(|(_, hole)| hole.clone())
                .collect();
            let area = outer.area() - holes.iter().map(ArrangementLoop::area).sum::<f64>();
            ArrangementRegion {
                outer: outer.clone(),
                holes,
                area,
            }
        })
        .collect()
}

fn sample_loop_for_containment<P>(loop_: &ArrangementLoop<P>) -> Vec<Pnt2d> {
    let mut points = Vec::new();
    for span in &loop_.spans {
        let steps = match span.kind() {
            CurveKind2d::Line => 1,
            CurveKind2d::Circle | CurveKind2d::Ellipse => {
                ((span.parameter_length() / (PI / 12.0)).ceil() as usize).clamp(1, 96)
            }
            _ => 24,
        };
        for step in 0..steps {
            let parameter = span.first + (span.last - span.first) * step as f64 / steps as f64;
            points.push(span.curve.point(parameter));
        }
    }
    points
}

fn interior_point(polygon: &[Pnt2d]) -> Pnt2d {
    if polygon.is_empty() {
        return Pnt2d::origin();
    }
    let average = Pnt2d::new(
        polygon.iter().map(Pnt2d::x).sum::<f64>() / polygon.len() as f64,
        polygon.iter().map(Pnt2d::y).sum::<f64>() / polygon.len() as f64,
    );
    if point_in_polygon(average, polygon) {
        return average;
    }
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let c = polygon[(index + 2) % polygon.len()];
        let candidate = Pnt2d::new((a.x() + b.x() + c.x()) / 3.0, (a.y() + b.y() + c.y()) / 3.0);
        if point_in_polygon(candidate, polygon) {
            return candidate;
        }
    }
    average
}

fn point_in_polygon(point: Pnt2d, polygon: &[Pnt2d]) -> bool {
    let mut inside = false;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if (a.y() > point.y()) != (b.y() > point.y())
            && point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x()
        {
            inside = !inside;
        }
    }
    inside
}

fn cross(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir2d, Pnt2d};
    use openrcad_geom2d::{BSplineCurve2d, Circle2d, Ellipse2d, Line2d};

    fn line(id: u32, start: (f64, f64), end: (f64, f64)) -> CurveSpan<u32> {
        let length = (end.0 - start.0).hypot(end.1 - start.1);
        let direction = Dir2d::new((end.0 - start.0) / length, (end.1 - start.1) / length);
        CurveSpan::new(
            GeomCurve2d::line(Line2d::from_point_dir(
                Pnt2d::new(start.0, start.1),
                direction,
            )),
            0.0,
            length,
            id,
        )
    }

    fn circle(id: u32, radius: f64) -> CurveSpan<u32> {
        CurveSpan::new(
            GeomCurve2d::circle(Circle2d::from_center(Pnt2d::origin(), radius)),
            0.0,
            TAU,
            id,
        )
    }

    fn circle_at(id: u32, center: (f64, f64), radius: f64) -> CurveSpan<u32> {
        CurveSpan::new(
            GeomCurve2d::circle(Circle2d::from_center(
                Pnt2d::new(center.0, center.1),
                radius,
            )),
            0.0,
            TAU,
            id,
        )
    }

    /// Two rectangles stacked on a shared edge, with a circle resting
    /// tangentially on that edge. The tangency leaves three half-edges on one
    /// outgoing tangent at the contact point; ordering them by angle alone tied
    /// arbitrarily, so the face walk stepped from one rectangle onto the other
    /// and reported a single fused face spanning both.
    #[test]
    fn circle_tangent_to_a_shared_edge_does_not_fuse_the_faces_either_side() {
        let spans = vec![
            line(1, (-3.0, -6.5), (40.0, -6.5)),
            line(2, (40.0, -6.5), (40.0, 0.0)),
            line(3, (40.0, 0.0), (-3.0, 0.0)),
            line(4, (-3.0, 0.0), (-3.0, -6.5)),
            line(5, (0.0, 0.0), (37.0, 0.0)),
            line(6, (37.0, 0.0), (37.0, 0.4)),
            line(7, (37.0, 0.4), (0.0, 0.4)),
            line(8, (0.0, 0.4), (0.0, 0.0)),
            // Lowest point is exactly (0.0, 0.0), on the shared edge.
            circle_at(9, (0.0, 0.4), 0.4),
        ];
        let arrangement = arrange_curve_spans(&spans, ArrangementOptions::default()).unwrap();
        let areas: Vec<f64> = arrangement.regions.iter().map(|r| r.area).collect();
        let lower = 43.0 * 6.5;
        assert!(
            areas.iter().any(|area| (area - lower).abs() < 1.0e-6),
            "the lower rectangle must remain its own face; got {areas:?}"
        );
        assert!(
            areas.iter().all(|area| *area < lower + 1.0e-6),
            "no face may span both rectangles; got {areas:?}"
        );
    }

    /// Three stacked rectangles with a circle tangent to BOTH edges of the
    /// thin middle band, so each contact point is a five-way vertex: two
    /// collinear line pieces (deduplicated to one), a vertical edge, and the
    /// circle's two tangent half-edges. The killer detail: the REVERSED
    /// half-edge of an axis-aligned line negates a +0.0 tangent into -0.0, and
    /// atan2(-0.0, -1) = -π while atan2(+0.0, -1) = +π — two half-edges
    /// leaving in the identical direction landed at OPPOSITE ends of the
    /// sorted fan, the tangential tie between them never formed, and the face
    /// walk swallowed the top rectangle and half the circle as a
    /// there-and-back excursion of the outer face (59.45 of 213.27 area
    /// vanished). Guards the ±π seam canonicalization in `half_edge_angle`
    /// plus the circular run detection in `resolve_tangential_ties`.
    #[test]
    fn four_way_tangential_contact_keeps_every_face() {
        let spans = vec![
            line(1, (-13.2, -13.4), (8.2, -13.4)),
            line(2, (8.2, -13.4), (8.2, -6.9)),
            line(3, (8.2, -6.9), (-13.2, -6.9)),
            line(4, (-13.2, -6.9), (-13.2, -13.4)),
            line(5, (-10.2, -6.9), (8.2, -6.9)),
            line(6, (8.2, -6.9), (8.2, -6.1)),
            line(7, (8.2, -6.1), (-10.2, -6.1)),
            line(8, (-10.2, -6.1), (-10.2, -6.9)),
            line(9, (-10.3, -6.1), (8.2, -6.1)),
            line(10, (8.2, -6.1), (8.2, -2.9)),
            line(11, (8.2, -2.9), (-10.3, -2.9)),
            line(12, (-10.3, -2.9), (-10.3, -6.1)),
            // Tangent to y = -6.9 at (-10.2, -6.9) and y = -6.1 at (-10.2, -6.1).
            circle_at(13, (-10.2, -6.5), 0.4),
        ];
        let arrangement = arrange_curve_spans(&spans, ArrangementOptions::default()).unwrap();
        let mut areas: Vec<f64> = arrangement.regions.iter().map(|r| r.area).collect();
        areas.sort_by(f64::total_cmp);
        let total: f64 = areas.iter().sum();
        let half_disc = core::f64::consts::PI * 0.4 * 0.4 * 0.5;
        let expected = [
            half_disc,
            half_disc,
            18.4 * 0.8 - half_disc, // the band minus the tangent circle's right half
            18.5 * 3.2,             // the top rectangle — the face the seam bug lost
            21.4 * 6.5,             // the bottom rectangle
        ];
        assert_eq!(
            areas.len(),
            expected.len(),
            "expected {} faces, got areas {areas:?}",
            expected.len()
        );
        for (area, want) in areas.iter().zip(expected) {
            assert!(
                (area - want).abs() < 1.0e-6,
                "face areas diverge: got {areas:?}"
            );
        }
        assert!(
            (total - 213.27).abs() < 0.01,
            "total area must be preserved; got {total}"
        );
    }

    /// Persisted BugCase1 coordinates: the long duplicate edge differs by only
    /// 1.43e-6 mm from the rectangle edge, but its tiny slope used to turn the
    /// intended circle tangency into two intersections 0.001 mm apart. That
    /// produced a sixth, 5.9e-6 mm² triangular face. The physical penetration
    /// is below the arrangement tolerance, so it must collapse to tangency.
    #[test]
    fn f32_near_tangent_duplicate_does_not_create_a_microscopic_region() {
        let spans = vec![
            line(1, (-13.2, -13.4), (8.2, -13.4)),
            line(2, (8.2, -13.4), (8.2, -6.899999619)),
            line(3, (8.2, -6.899999619), (-13.2, -6.899999619)),
            line(4, (-13.2, -6.899999619), (-13.2, -13.4)),
            // Duplicate support with the exact f32 slope from BugCase1.
            line(5, (8.2, -6.899999619), (-10.2, -6.900001049)),
            line(6, (-10.2, -6.900001049), (-10.2, -6.500000954)),
            line(7, (-10.2, -6.900001049), (8.2, -6.900001049)),
            line(8, (8.2, -6.900001049), (8.2, -6.100000858)),
            line(9, (8.2, -6.100000858), (-10.2, -6.100000858)),
            line(10, (-10.2, -6.100000858), (-10.2, -6.900001049)),
            line(11, (8.2, -6.100000858), (-10.3, -6.100000858)),
            line(12, (-10.3, -6.100000858), (-10.3, -2.900000811)),
            line(13, (-10.3, -2.900000811), (8.2, -2.900000811)),
            line(14, (8.2, -2.900000811), (8.2, -6.100000858)),
            circle_at(15, (-10.2, -6.500000954), 0.400000006),
        ];
        let tolerance = 1.3e-5;
        let arrangement = arrange_curve_spans(
            &spans,
            ArrangementOptions {
                tolerance,
                ..ArrangementOptions::default()
            },
        )
        .unwrap();
        let areas: Vec<f64> = arrangement
            .regions
            .iter()
            .map(|region| region.area)
            .collect();
        assert_eq!(
            areas.len(),
            5,
            "the tolerance sliver must disappear: {areas:?}"
        );
        assert!(
            areas.iter().all(|area| *area > 0.1),
            "no microscopic face may survive: {areas:?}"
        );
    }

    #[test]
    fn overlaps_and_coincident_line_spans_do_not_duplicate_dcel_edges() {
        let spans = vec![
            line(1, (0.0, 0.0), (10.0, 0.0)),
            line(2, (10.0, 0.0), (10.0, 10.0)),
            line(3, (10.0, 10.0), (0.0, 10.0)),
            line(4, (0.0, 10.0), (0.0, 0.0)),
            line(5, (2.0, 0.0), (8.0, 0.0)),
            line(6, (8.0, 0.0), (2.0, 0.0)),
        ];
        let arrangement = arrange_curve_spans(&spans, ArrangementOptions::default()).unwrap();
        assert_eq!(arrangement.regions.len(), 1);
        assert!((arrangement.regions[0].area - 100.0).abs() < 1.0e-8);
        assert!(arrangement.regions[0]
            .outer
            .spans
            .iter()
            .all(|span| span.provenance <= 6));
    }

    #[test]
    fn circle_line_arrangement_keeps_analytic_semicircles() {
        let spans = vec![circle(1, 5.0), line(2, (-5.0, 0.0), (5.0, 0.0))];
        let arrangement = arrange_curve_spans(&spans, ArrangementOptions::default()).unwrap();
        assert_eq!(arrangement.regions.len(), 2);
        for region in &arrangement.regions {
            assert!((region.area - PI * 25.0 * 0.5).abs() < 1.0e-7);
            assert!(region
                .outer
                .spans
                .iter()
                .any(|span| span.kind() == CurveKind2d::Circle));
        }
    }

    #[test]
    fn nested_circles_produce_disk_and_annulus() {
        let arrangement = arrange_curve_spans(
            &[circle(1, 5.0), circle(2, 2.0)],
            ArrangementOptions::default(),
        )
        .unwrap();
        assert_eq!(arrangement.regions.len(), 2);
        let annulus = arrangement
            .regions
            .iter()
            .find(|region| !region.holes.is_empty())
            .expect("annulus");
        assert!((annulus.area - PI * 21.0).abs() < 1.0e-7);
    }

    #[test]
    fn ellipse_line_uses_exact_line_case_and_controlled_conic_roots() {
        let ellipse = CurveSpan::new(
            GeomCurve2d::ellipse(Ellipse2d::from_center(Pnt2d::origin(), 4.0, 2.0)),
            0.0,
            TAU,
            1_u32,
        );
        let arrangement = arrange_curve_spans(
            &[ellipse, line(2, (-4.0, 0.0), (4.0, 0.0))],
            ArrangementOptions::default(),
        )
        .unwrap();
        assert_eq!(arrangement.regions.len(), 2);
        assert!(arrangement
            .regions
            .iter()
            .all(|region| (region.area - 4.0 * PI).abs() < 1.0e-7));
    }

    #[test]
    fn pairwise_curve_span_intersections_use_arrangement_parameters() {
        let horizontal = line(1, (-2.0, 0.0), (2.0, 0.0));
        let vertical = line(2, (0.0, -2.0), (0.0, 2.0));
        let result =
            intersect_curve_spans(&horizontal, &vertical, ArrangementOptions::default()).unwrap();
        assert!(!result.coincident);
        assert_eq!(result.points.len(), 1);
        assert!((result.points[0].first_parameter - 2.0).abs() < 1.0e-9);
        assert!((result.points[0].second_parameter - 2.0).abs() < 1.0e-9);
    }

    #[test]
    fn pairwise_curve_span_intersections_report_overlap() {
        let first = line(1, (0.0, 0.0), (4.0, 0.0));
        let second = line(2, (2.0, 0.0), (6.0, 0.0));
        let result = intersect_curve_spans(&first, &second, ArrangementOptions::default()).unwrap();
        assert!(result.coincident);
        assert!(result.points.len() >= 2);
    }

    #[test]
    fn unsupported_nurbs_pair_is_typed() {
        let spline = BSplineCurve2d::new(
            2,
            vec![
                Pnt2d::new(-2.0, 0.0),
                Pnt2d::new(0.0, 3.0),
                Pnt2d::new(2.0, 0.0),
            ],
            None,
            vec![0.0, 1.0],
            vec![3, 3],
        );
        let spline = CurveSpan::new(GeomCurve2d::bspline(spline), 0.0, 1.0, 7_u32);
        let error = arrange_curve_spans(&[spline, circle(8, 2.0)], ArrangementOptions::default())
            .unwrap_err();
        assert!(matches!(
            error,
            ArrangementError::UnsupportedPair {
                first_kind: CurveKind2d::BSpline,
                second_kind: CurveKind2d::Circle,
                ..
            }
        ));
    }
}
