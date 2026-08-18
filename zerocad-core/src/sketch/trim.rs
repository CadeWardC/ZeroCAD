//! Parametric line/arc/circle/ellipse/spline trimming for the constraint-backed
//! sketch model.

use std::collections::HashSet;

use openrcad::foundation::{Dir2d, Pnt2d};
use openrcad::geom2d::{Circle2d, Curve2d, CurveSpan, GeomCurve2d, Line2d};

use super::{
    Arc, Constraint, EntityId, SketchCurves, SketchEntity, SketchPoint, SketchSolverModel, Spline,
};

const PARAM_TOLERANCE: f64 = 1.0e-8;
const GEOMETRY_TOLERANCE: f64 = 1.0e-6;

#[derive(Debug, Clone, PartialEq)]
pub enum TrimError {
    NoEntity,
    UnsupportedEntity { id: EntityId, kind: &'static str },
    UnsupportedPair { first: EntityId, second: EntityId },
    AmbiguousOverlap { first: EntityId, second: EntityId },
    TangentialCircle { id: EntityId },
    TangentialClosedCurve { id: EntityId },
    InvalidGeometry { id: EntityId },
}

impl std::fmt::Display for TrimError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoEntity => formatter.write_str(
                "no trimmable line, arc, circle, ellipse, or spline is under the cursor",
            ),
            Self::UnsupportedEntity { id, kind } => {
                write!(
                    formatter,
                    "entity {} is an unsupported {kind} trim target",
                    id.0
                )
            }
            Self::UnsupportedPair { first, second } => write!(
                formatter,
                "entities {} and {} use a curve-pair intersection that is not supported yet",
                first.0, second.0
            ),
            Self::AmbiguousOverlap { first, second } => write!(
                formatter,
                "entities {} and {} overlap coincidently; the removable span is ambiguous",
                first.0, second.0
            ),
            Self::TangentialCircle { id } => write!(
                formatter,
                "circle {} has only one distinct trim intersection",
                id.0
            ),
            Self::TangentialClosedCurve { id } => write!(
                formatter,
                "closed curve {} has only one distinct trim intersection",
                id.0
            ),
            Self::InvalidGeometry { id } => {
                write!(formatter, "entity {} has invalid geometry", id.0)
            }
        }
    }
}

impl std::error::Error for TrimError {}

#[derive(Debug, Clone, PartialEq)]
enum TrimPiece {
    Line {
        start: (f32, f32),
        end: (f32, f32),
        contains_original_start: bool,
        contains_original_end: bool,
    },
    Arc {
        center: EntityId,
        center_point: (f32, f32),
        radius: f32,
        start: (f32, f32),
        end: (f32, f32),
        clockwise: bool,
        contains_original_start: bool,
        contains_original_end: bool,
    },
    Ellipse {
        center: EntityId,
        center_point: (f32, f32),
        major_axis: [f64; 2],
        minor_axis: [f64; 2],
        start_parameter: f64,
        end_parameter: f64,
        contains_original_start: bool,
    },
    Spline {
        point_ids: Vec<EntityId>,
        spline: Spline,
        contains_original_start: bool,
        contains_original_end: bool,
    },
}

/// Immutable hover result. The private replacement plan is consumed unchanged
/// by [`apply_trim_preview`], making the click action identical to the preview.
#[derive(Debug, Clone, PartialEq)]
pub struct TrimPreview {
    pub target: EntityId,
    pub removable: SketchCurves,
    pieces: Vec<TrimPiece>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TrimConstraintReport {
    pub removed: Vec<EntityId>,
    pub ambiguous: Vec<EntityId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TrimOutcome {
    pub target: EntityId,
    pub created_entities: Vec<EntityId>,
    pub constraints: TrimConstraintReport,
}

#[derive(Clone, Copy)]
enum Primitive {
    Line {
        id: EntityId,
        start: (f64, f64),
        end: (f64, f64),
    },
    Circle {
        id: EntityId,
        center: (f64, f64),
        radius: f64,
        bounds: Option<(f64, f64)>,
    },
}

impl Primitive {
    fn id(self) -> EntityId {
        match self {
            Self::Line { id, .. } | Self::Circle { id, .. } => id,
        }
    }

    fn contains(self, point: (f64, f64)) -> bool {
        match self {
            Self::Line { start, end, .. } => {
                let direction = (end.0 - start.0, end.1 - start.1);
                let length_squared = direction.0 * direction.0 + direction.1 * direction.1;
                if length_squared <= GEOMETRY_TOLERANCE * GEOMETRY_TOLERANCE {
                    return false;
                }
                let parameter = ((point.0 - start.0) * direction.0
                    + (point.1 - start.1) * direction.1)
                    / length_squared;
                (-PARAM_TOLERANCE..=1.0 + PARAM_TOLERANCE).contains(&parameter)
            }
            Self::Circle {
                center,
                radius,
                bounds,
                ..
            } => {
                if ((point.0 - center.0).hypot(point.1 - center.1) - radius).abs()
                    > GEOMETRY_TOLERANCE
                {
                    return false;
                }
                bounds.is_none_or(|bounds| angle_in_bounds(point_angle(center, point), bounds))
            }
        }
    }

    fn parameter(self, point: (f64, f64)) -> Option<f64> {
        match self {
            Self::Line { start, end, .. } => {
                let direction = (end.0 - start.0, end.1 - start.1);
                let length_squared = direction.0 * direction.0 + direction.1 * direction.1;
                (length_squared > GEOMETRY_TOLERANCE * GEOMETRY_TOLERANCE).then(|| {
                    ((point.0 - start.0) * direction.0 + (point.1 - start.1) * direction.1)
                        / length_squared
                })
            }
            Self::Circle {
                center,
                bounds: None,
                ..
            } => Some(point_angle(center, point).rem_euclid(std::f64::consts::TAU)),
            Self::Circle {
                center,
                bounds: Some((first, last)),
                ..
            } => angle_in_bounds_parameter(point_angle(center, point), (first, last))
                .map(|parameter| (parameter - first) / (last - first)),
        }
    }

    fn curve_span(self) -> Result<CurveSpan<EntityId>, TrimError> {
        match self {
            Self::Line { id, start, end } => {
                let dx = end.0 - start.0;
                let dy = end.1 - start.1;
                let length = dx.hypot(dy);
                if !length.is_finite() || length <= GEOMETRY_TOLERANCE {
                    return Err(TrimError::InvalidGeometry { id });
                }
                Ok(CurveSpan::new(
                    GeomCurve2d::line(Line2d::from_point_dir(
                        Pnt2d::new(start.0, start.1),
                        Dir2d::try_new(dx / length, dy / length)
                            .ok_or(TrimError::InvalidGeometry { id })?,
                    )),
                    0.0,
                    length,
                    id,
                ))
            }
            Self::Circle {
                id,
                center,
                radius,
                bounds,
            } => {
                if !radius.is_finite() || radius <= GEOMETRY_TOLERANCE {
                    return Err(TrimError::InvalidGeometry { id });
                }
                let (first, last) = bounds.unwrap_or((0.0, std::f64::consts::TAU));
                Ok(CurveSpan::new(
                    GeomCurve2d::circle(Circle2d::from_center(
                        Pnt2d::new(center.0, center.1),
                        radius,
                    )),
                    first,
                    last,
                    id,
                ))
            }
        }
    }
}

#[derive(Clone)]
enum ComplexCurve {
    Ellipse {
        id: EntityId,
        center: EntityId,
        center_point: (f64, f64),
        major_axis: [f64; 2],
        minor_axis: [f64; 2],
        first: f64,
        last: f64,
        closed: bool,
    },
    Spline {
        id: EntityId,
        point_ids: Vec<EntityId>,
        spline: Spline,
        first: f64,
        last: f64,
        closed: bool,
    },
}

impl ComplexCurve {
    fn id(&self) -> EntityId {
        match self {
            Self::Ellipse { id, .. } | Self::Spline { id, .. } => *id,
        }
    }

    fn bounds(&self) -> (f64, f64) {
        match self {
            Self::Ellipse { first, last, .. } | Self::Spline { first, last, .. } => (*first, *last),
        }
    }

    fn closed(&self) -> bool {
        match self {
            Self::Ellipse { closed, .. } | Self::Spline { closed, .. } => *closed,
        }
    }

    fn period(&self) -> f64 {
        match self {
            Self::Ellipse { .. } => std::f64::consts::TAU,
            Self::Spline { .. } => 1.0,
        }
    }

    fn point(&self, parameter: f64) -> Option<(f64, f64)> {
        match self {
            Self::Ellipse {
                center_point,
                major_axis,
                minor_axis,
                ..
            } => {
                let (sin, cos) = parameter.sin_cos();
                Some((
                    center_point.0 + major_axis[0] * cos + minor_axis[0] * sin,
                    center_point.1 + major_axis[1] * cos + minor_axis[1] * sin,
                ))
            }
            Self::Spline { spline, .. } => spline
                .evaluate(parameter as f32)
                .map(|point| (f64::from(point.0), f64::from(point.1))),
        }
    }

    fn ellipse_implicit(&self, point: (f64, f64)) -> Option<f64> {
        let Self::Ellipse {
            center_point,
            major_axis,
            minor_axis,
            ..
        } = self
        else {
            return None;
        };
        let determinant = major_axis[0] * minor_axis[1] - major_axis[1] * minor_axis[0];
        if determinant.abs() <= GEOMETRY_TOLERANCE {
            return None;
        }
        let delta = (point.0 - center_point.0, point.1 - center_point.1);
        let cosine = (delta.0 * minor_axis[1] - delta.1 * minor_axis[0]) / determinant;
        let sine = (major_axis[0] * delta.1 - major_axis[1] * delta.0) / determinant;
        Some(cosine * cosine + sine * sine - 1.0)
    }

    fn parameter(&self, point: (f64, f64)) -> Option<f64> {
        match self {
            Self::Ellipse {
                center_point,
                major_axis,
                minor_axis,
                first,
                last,
                closed,
                ..
            } => {
                let determinant = major_axis[0] * minor_axis[1] - major_axis[1] * minor_axis[0];
                if determinant.abs() <= GEOMETRY_TOLERANCE {
                    return None;
                }
                let delta = (point.0 - center_point.0, point.1 - center_point.1);
                let cosine = (delta.0 * minor_axis[1] - delta.1 * minor_axis[0]) / determinant;
                let sine = (major_axis[0] * delta.1 - major_axis[1] * delta.0) / determinant;
                let angle = sine.atan2(cosine);
                if *closed {
                    Some(*first + (angle - *first).rem_euclid(std::f64::consts::TAU))
                } else {
                    angle_in_bounds_parameter(angle, (*first, *last))
                }
            }
            Self::Spline { first, last, .. } => {
                let subdivisions = 512;
                let distance_squared = |parameter: f64| {
                    self.point(parameter)
                        .map(|candidate| {
                            (candidate.0 - point.0).powi(2) + (candidate.1 - point.1).powi(2)
                        })
                        .unwrap_or(f64::INFINITY)
                };
                let mut best = *first;
                let mut best_distance = distance_squared(best);
                for index in 1..=subdivisions {
                    let parameter = *first + (*last - *first) * index as f64 / subdivisions as f64;
                    let distance = distance_squared(parameter);
                    if distance < best_distance {
                        best = parameter;
                        best_distance = distance;
                    }
                }
                let step = (*last - *first).abs() / subdivisions as f64;
                let mut low = (best - step).max(first.min(*last));
                let mut high = (best + step).min(first.max(*last));
                for _ in 0..40 {
                    let left = low + (high - low) / 3.0;
                    let right = high - (high - low) / 3.0;
                    if distance_squared(left) <= distance_squared(right) {
                        high = right;
                    } else {
                        low = left;
                    }
                }
                Some((low + high) * 0.5)
            }
        }
    }
}

fn complex_curve(
    model: &SketchSolverModel,
    entity: &SketchEntity,
) -> Result<Option<ComplexCurve>, TrimError> {
    let point = |id: EntityId| {
        model
            .point(id)
            .map(|point| point.pos)
            .ok_or(TrimError::InvalidGeometry { id: entity.id() })
    };
    match entity {
        SketchEntity::Ellipse {
            id,
            center,
            major_axis,
            minor_axis,
            start_parameter,
            end_parameter,
            closed,
            ..
        } => {
            let determinant = major_axis[0] * minor_axis[1] - major_axis[1] * minor_axis[0];
            if !start_parameter.is_finite()
                || !end_parameter.is_finite()
                || major_axis
                    .iter()
                    .chain(minor_axis)
                    .any(|value| !value.is_finite())
                || determinant.abs() <= GEOMETRY_TOLERANCE
            {
                return Err(TrimError::InvalidGeometry { id: *id });
            }
            let first = *start_parameter;
            let last = if *closed {
                first + std::f64::consts::TAU
            } else {
                *end_parameter
            };
            if (last - first).abs() <= PARAM_TOLERANCE {
                return Err(TrimError::InvalidGeometry { id: *id });
            }
            Ok(Some(ComplexCurve::Ellipse {
                id: *id,
                center: *center,
                center_point: point(*center)?,
                major_axis: *major_axis,
                minor_axis: *minor_axis,
                first,
                last,
                closed: *closed,
            }))
        }
        SketchEntity::Spline {
            id,
            points,
            kind,
            degree,
            knots,
            weights,
            closed,
            periodic,
            continuity,
            trim,
            ..
        } => {
            let positions: Result<Vec<_>, _> = points
                .iter()
                .map(|id| point(*id).map(|point| (point.0 as f32, point.1 as f32)))
                .collect();
            let spline = Spline {
                kind: *kind,
                points: positions?,
                degree: *degree,
                knots: knots.clone(),
                weights: weights.clone(),
                closed: *closed,
                periodic: *periodic,
                continuity: *continuity,
                trim: *trim,
            };
            if !spline.is_valid() {
                return Err(TrimError::InvalidGeometry { id: *id });
            }
            let (first, last) = trim.unwrap_or((0.0, 1.0));
            Ok(Some(ComplexCurve::Spline {
                id: *id,
                point_ids: points.clone(),
                spline,
                first: f64::from(first),
                last: f64::from(last),
                closed: *closed && trim.is_none(),
            }))
        }
        _ => Ok(None),
    }
}

fn isolate_roots(function: impl Fn(f64) -> f64, first: f64, last: f64) -> Result<Vec<f64>, ()> {
    const SUBDIVISIONS: usize = 512;
    let mut samples = Vec::with_capacity(SUBDIVISIONS + 1);
    for index in 0..=SUBDIVISIONS {
        let parameter = first + (last - first) * index as f64 / SUBDIVISIONS as f64;
        samples.push((parameter, function(parameter)));
    }
    let finite: Vec<_> = samples
        .iter()
        .filter(|(_, value)| value.is_finite())
        .collect();
    if finite.is_empty() {
        return Ok(Vec::new());
    }
    if finite
        .iter()
        .all(|(_, value)| value.abs() <= GEOMETRY_TOLERANCE)
    {
        return Err(());
    }
    let mut roots = Vec::new();
    for window in samples.windows(2) {
        let ((mut a, mut fa), (mut b, fb)) = (window[0], window[1]);
        if !fa.is_finite() || !fb.is_finite() {
            continue;
        }
        if fa.abs() <= GEOMETRY_TOLERANCE {
            roots.push(a);
        }
        if fa.signum() == fb.signum() && fb.abs() > GEOMETRY_TOLERANCE {
            continue;
        }
        for _ in 0..64 {
            let midpoint = (a + b) * 0.5;
            let value = function(midpoint);
            if !value.is_finite() || value.abs() <= GEOMETRY_TOLERANCE || (b - a).abs() <= 1.0e-12 {
                a = midpoint;
                b = midpoint;
                break;
            }
            if fa.signum() == value.signum() {
                a = midpoint;
                fa = value;
            } else {
                b = midpoint;
            }
        }
        roots.push((a + b) * 0.5);
    }
    if let Some(&(parameter, value)) = samples.last() {
        if value.abs() <= GEOMETRY_TOLERANCE {
            roots.push(parameter);
        }
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|a, b| (*a - *b).abs() <= PARAM_TOLERANCE);
    Ok(roots)
}

fn primitive_equation(primitive: Primitive, point: (f64, f64)) -> f64 {
    match primitive {
        Primitive::Line { start, end, .. } => {
            let direction = (end.0 - start.0, end.1 - start.1);
            let length = direction.0.hypot(direction.1);
            if length <= GEOMETRY_TOLERANCE {
                f64::NAN
            } else {
                (direction.0 * (point.1 - start.1) - direction.1 * (point.0 - start.0)) / length
            }
        }
        Primitive::Circle { center, radius, .. } => {
            (point.0 - center.0).hypot(point.1 - center.1) - radius
        }
    }
}

fn roots_of_complex_against_primitive(
    complex: &ComplexCurve,
    primitive: Primitive,
) -> Result<Vec<f64>, TrimError> {
    let (first, last) = complex.bounds();
    isolate_roots(
        |parameter| {
            complex
                .point(parameter)
                .map(|point| primitive_equation(primitive, point))
                .unwrap_or(f64::NAN)
        },
        first,
        last,
    )
    .map_err(|()| TrimError::AmbiguousOverlap {
        first: complex.id(),
        second: primitive.id(),
    })
    .map(|parameters| {
        parameters
            .into_iter()
            .filter(|parameter| {
                complex
                    .point(*parameter)
                    .is_some_and(|point| primitive.contains(point))
            })
            .collect()
    })
}

fn roots_of_complex_against_ellipse(
    complex: &ComplexCurve,
    ellipse: &ComplexCurve,
) -> Result<Vec<f64>, TrimError> {
    let (first, last) = complex.bounds();
    isolate_roots(
        |parameter| {
            complex
                .point(parameter)
                .and_then(|point| ellipse.ellipse_implicit(point))
                .unwrap_or(f64::NAN)
        },
        first,
        last,
    )
    .map_err(|()| TrimError::AmbiguousOverlap {
        first: complex.id(),
        second: ellipse.id(),
    })
}

fn complex_target_intersections(
    model: &SketchSolverModel,
    target: &ComplexCurve,
    other_entity: &SketchEntity,
) -> Result<Vec<f64>, TrimError> {
    match primitive(model, other_entity) {
        Ok(primitive) => return roots_of_complex_against_primitive(target, primitive),
        Err(TrimError::UnsupportedEntity { .. }) => {}
        Err(error) => return Err(error),
    }
    let Some(other) = complex_curve(model, other_entity)? else {
        return Ok(Vec::new());
    };
    match (target, &other) {
        (ComplexCurve::Spline { .. }, ComplexCurve::Spline { .. }) => {
            Err(TrimError::UnsupportedPair {
                first: target.id(),
                second: other.id(),
            })
        }
        (ComplexCurve::Ellipse { .. }, ComplexCurve::Ellipse { .. })
        | (ComplexCurve::Spline { .. }, ComplexCurve::Ellipse { .. }) => {
            let mut parameters = roots_of_complex_against_ellipse(target, &other)?;
            parameters.retain(|parameter| {
                target
                    .point(*parameter)
                    .and_then(|point| other.parameter(point))
                    .is_some()
            });
            Ok(parameters)
        }
        (ComplexCurve::Ellipse { .. }, ComplexCurve::Spline { .. }) => {
            let parameters = roots_of_complex_against_ellipse(&other, target)?;
            Ok(parameters
                .into_iter()
                .filter_map(|parameter| other.point(parameter))
                .filter_map(|point| target.parameter(point))
                .collect())
        }
    }
}

fn primitive_target_complex_intersections(
    target: Primitive,
    other: &ComplexCurve,
) -> Result<Vec<(f64, f64)>, TrimError> {
    roots_of_complex_against_primitive(other, target).map(|parameters| {
        parameters
            .into_iter()
            .filter_map(|parameter| other.point(parameter))
            .filter(|point| target.contains(*point))
            .collect()
    })
}

fn point_angle(center: (f64, f64), point: (f64, f64)) -> f64 {
    (point.1 - center.1).atan2(point.0 - center.0)
}

fn angle_in_bounds_parameter(angle: f64, bounds: (f64, f64)) -> Option<f64> {
    let (first, last) = bounds;
    (-2..=2)
        .map(|turn| angle + f64::from(turn) * std::f64::consts::TAU)
        .find(|parameter| {
            *parameter >= first.min(last) - PARAM_TOLERANCE
                && *parameter <= first.max(last) + PARAM_TOLERANCE
        })
}

fn angle_in_bounds(angle: f64, bounds: (f64, f64)) -> bool {
    angle_in_bounds_parameter(angle, bounds).is_some()
}

fn primitive(model: &SketchSolverModel, entity: &SketchEntity) -> Result<Primitive, TrimError> {
    let point = |id: EntityId| {
        model
            .point(id)
            .map(|point| point.pos)
            .ok_or(TrimError::InvalidGeometry { id: entity.id() })
    };
    match entity {
        SketchEntity::Line { id, p0, p1, .. } => Ok(Primitive::Line {
            id: *id,
            start: point(*p0)?,
            end: point(*p1)?,
        }),
        SketchEntity::Circle {
            id, center, radius, ..
        } if radius.is_finite() && *radius > 0.0 => Ok(Primitive::Circle {
            id: *id,
            center: point(*center)?,
            radius: *radius,
            bounds: None,
        }),
        SketchEntity::Arc {
            id,
            center,
            start,
            end,
            radius,
            clockwise,
            ..
        } if radius.is_finite() && *radius > 0.0 => {
            let center_point = point(*center)?;
            let start_point = point(*start)?;
            let end_point = point(*end)?;
            let arc = Arc {
                center: (center_point.0 as f32, center_point.1 as f32),
                radius: *radius as f32,
                start: (start_point.0 as f32, start_point.1 as f32),
                end: (end_point.0 as f32, end_point.1 as f32),
                clockwise: *clockwise,
            };
            Ok(Primitive::Circle {
                id: *id,
                center: center_point,
                radius: *radius,
                bounds: Some(arc.parameter_bounds()),
            })
        }
        SketchEntity::Ellipse { id, .. } => Err(TrimError::UnsupportedEntity {
            id: *id,
            kind: "ellipse",
        }),
        SketchEntity::Spline { id, .. } => Err(TrimError::UnsupportedEntity {
            id: *id,
            kind: "spline",
        }),
        entity => Err(TrimError::InvalidGeometry { id: entity.id() }),
    }
}

fn intersections(first: Primitive, second: Primitive) -> Result<Vec<(f64, f64)>, TrimError> {
    let first_id = first.id();
    let second_id = second.id();
    let first_span = first.curve_span()?;
    let second_span = second.curve_span()?;
    let result = openrcad::sketch::intersect_curve_spans(
        &first_span,
        &second_span,
        openrcad::sketch::ArrangementOptions {
            tolerance: GEOMETRY_TOLERANCE,
            ..openrcad::sketch::ArrangementOptions::default()
        },
    )
    .map_err(|error| match error {
        openrcad::sketch::ArrangementError::UnsupportedPair { .. } => TrimError::UnsupportedPair {
            first: first_id,
            second: second_id,
        },
        openrcad::sketch::ArrangementError::InvalidSpan { index: 1 } => {
            TrimError::InvalidGeometry { id: second_id }
        }
        _ => TrimError::InvalidGeometry { id: first_id },
    })?;
    if result.coincident {
        return Err(TrimError::AmbiguousOverlap {
            first: first_id,
            second: second_id,
        });
    }
    Ok(result
        .points
        .into_iter()
        .map(|intersection| {
            let point = first_span.curve.point(intersection.first_parameter);
            (point.x(), point.y())
        })
        .collect())
}

fn split_arc_piece(
    pieces: &mut Vec<TrimPiece>,
    center: EntityId,
    center_point: (f32, f32),
    radius: f32,
    first: f64,
    last: f64,
    original_parameters: (f64, f64),
) {
    let subdivisions = ((last - first).abs() / std::f64::consts::FRAC_PI_2)
        .ceil()
        .max(1.0) as usize;
    for index in 0..subdivisions {
        let start_parameter = first + (last - first) * index as f64 / subdivisions as f64;
        let end_parameter = first + (last - first) * (index + 1) as f64 / subdivisions as f64;
        let at = |parameter: f64| {
            (
                center_point.0 + radius * parameter.cos() as f32,
                center_point.1 + radius * parameter.sin() as f32,
            )
        };
        let contains = |parameter: f64| {
            parameter >= start_parameter.min(end_parameter) - PARAM_TOLERANCE
                && parameter <= start_parameter.max(end_parameter) + PARAM_TOLERANCE
        };
        pieces.push(TrimPiece::Arc {
            center,
            center_point,
            radius,
            start: at(start_parameter),
            end: at(end_parameter),
            clockwise: end_parameter < start_parameter,
            contains_original_start: contains(original_parameters.0),
            contains_original_end: contains(original_parameters.1),
        });
    }
}

fn append_preview_piece(curves: &mut SketchCurves, piece: &TrimPiece) {
    match piece {
        TrimPiece::Line { start, end, .. } => curves.add_line(*start, *end),
        TrimPiece::Arc {
            center_point,
            radius,
            start,
            end,
            clockwise,
            ..
        } => curves.arcs.push(Arc {
            center: *center_point,
            radius: *radius,
            start: *start,
            end: *end,
            clockwise: *clockwise,
        }),
        TrimPiece::Ellipse {
            center_point,
            major_axis,
            minor_axis,
            start_parameter,
            end_parameter,
            ..
        } => {
            let sweep = end_parameter - start_parameter;
            let subdivisions =
                ((sweep.abs() / std::f64::consts::TAU * 96.0).ceil() as usize).clamp(4, 96);
            let at = |parameter: f64| {
                let (sin, cos) = parameter.sin_cos();
                (
                    center_point.0 + (major_axis[0] * cos + minor_axis[0] * sin) as f32,
                    center_point.1 + (major_axis[1] * cos + minor_axis[1] * sin) as f32,
                )
            };
            for index in 0..subdivisions {
                let first = start_parameter + sweep * index as f64 / subdivisions as f64;
                let last = start_parameter + sweep * (index + 1) as f64 / subdivisions as f64;
                curves.add_line(at(first), at(last));
            }
        }
        TrimPiece::Spline { spline, .. } => curves.splines.push(spline.clone()),
    }
}

fn complex_interval_pieces(
    curve: &ComplexCurve,
    first: f64,
    last: f64,
    contains_original_start: bool,
    contains_original_end: bool,
) -> Vec<TrimPiece> {
    match curve {
        ComplexCurve::Ellipse {
            center,
            center_point,
            major_axis,
            minor_axis,
            ..
        } => vec![TrimPiece::Ellipse {
            center: *center,
            center_point: (center_point.0 as f32, center_point.1 as f32),
            major_axis: *major_axis,
            minor_axis: *minor_axis,
            start_parameter: first,
            end_parameter: last,
            contains_original_start,
        }],
        ComplexCurve::Spline {
            point_ids, spline, ..
        } => {
            let mut intervals = Vec::new();
            if last > 1.0 + PARAM_TOLERANCE {
                intervals.push((first, 1.0, false, true));
                intervals.push((0.0, last - 1.0, contains_original_start, false));
            } else {
                intervals.push((first, last, contains_original_start, contains_original_end));
            }
            intervals
                .into_iter()
                .filter(|(first, last, ..)| last - first > PARAM_TOLERANCE)
                .map(
                    |(first, last, contains_original_start, contains_original_end)| {
                        let mut spline = spline.clone();
                        spline.trim = Some((first as f32, last as f32));
                        TrimPiece::Spline {
                            point_ids: point_ids.clone(),
                            spline,
                            contains_original_start,
                            contains_original_end,
                        }
                    },
                )
                .collect()
        }
    }
}

fn preview_complex_trim(
    model: &SketchSolverModel,
    target: ComplexCurve,
    cursor: (f32, f32),
) -> Result<TrimPreview, TrimError> {
    let (first, last) = target.bounds();
    let length = last - first;
    let mut parameters = Vec::new();
    for other in &model.entities {
        if other.id() == target.id() || model.construction.contains(&other.id()) {
            continue;
        }
        parameters.extend(complex_target_intersections(model, &target, other)?);
    }

    let cursor_parameter = target
        .parameter((f64::from(cursor.0), f64::from(cursor.1)))
        .ok_or(TrimError::InvalidGeometry { id: target.id() })?;
    let mut removable = SketchCurves::new();
    let mut pieces = Vec::new();
    if target.closed() {
        let period = target.period();
        let phase = |parameter: f64| (parameter - first).rem_euclid(period) / period;
        let mut cuts: Vec<_> = parameters.into_iter().map(phase).collect();
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|a, b| (*a - *b).abs() <= PARAM_TOLERANCE);
        if cuts.is_empty() {
            for piece in complex_interval_pieces(&target, first, last, true, true) {
                append_preview_piece(&mut removable, &piece);
            }
        } else {
            if cuts.len() == 1 {
                return Err(TrimError::TangentialClosedCurve { id: target.id() });
            }
            let cursor_phase = phase(cursor_parameter);
            let remove_index = (0..cuts.len())
                .find(|index| {
                    let start = cuts[*index];
                    let end = if *index + 1 < cuts.len() {
                        cuts[*index + 1]
                    } else {
                        cuts[0] + 1.0
                    };
                    let cursor = if cursor_phase < start {
                        cursor_phase + 1.0
                    } else {
                        cursor_phase
                    };
                    cursor >= start - PARAM_TOLERANCE && cursor <= end + PARAM_TOLERANCE
                })
                .unwrap_or(0);
            for index in 0..cuts.len() {
                let start_phase = cuts[index];
                let end_phase = if index + 1 < cuts.len() {
                    cuts[index + 1]
                } else {
                    cuts[0] + 1.0
                };
                let interval = complex_interval_pieces(
                    &target,
                    first + period * start_phase,
                    first + period * end_phase,
                    end_phase > 1.0 - PARAM_TOLERANCE,
                    false,
                );
                if index == remove_index {
                    for piece in &interval {
                        append_preview_piece(&mut removable, piece);
                    }
                } else {
                    pieces.extend(interval);
                }
            }
        }
    } else {
        let mut cuts = vec![0.0];
        cuts.extend(parameters.into_iter().filter_map(|parameter| {
            let fraction = (parameter - first) / length;
            (fraction > PARAM_TOLERANCE && fraction < 1.0 - PARAM_TOLERANCE).then_some(fraction)
        }));
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|a, b| (*a - *b).abs() <= PARAM_TOLERANCE);
        cuts.push(1.0);
        let cursor_fraction = ((cursor_parameter - first) / length).clamp(0.0, 1.0);
        let remove_index = cuts
            .windows(2)
            .position(|bounds| {
                cursor_fraction >= bounds[0] - PARAM_TOLERANCE
                    && cursor_fraction <= bounds[1] + PARAM_TOLERANCE
            })
            .unwrap_or(0);
        for (index, bounds) in cuts.windows(2).enumerate() {
            let interval = complex_interval_pieces(
                &target,
                first + length * bounds[0],
                first + length * bounds[1],
                bounds[0] <= PARAM_TOLERANCE,
                bounds[1] >= 1.0 - PARAM_TOLERANCE,
            );
            if index == remove_index {
                for piece in &interval {
                    append_preview_piece(&mut removable, piece);
                }
            } else {
                pieces.extend(interval);
            }
        }
    }

    Ok(TrimPreview {
        target: target.id(),
        removable,
        pieces,
    })
}

fn nearest_target(
    model: &SketchSolverModel,
    cursor: (f32, f32),
    tolerance: f32,
) -> Result<&SketchEntity, TrimError> {
    let point = |id| model.point(id).map(|point| point.pos);
    let segment_distance = |a: (f64, f64), b: (f64, f64)| {
        let direction = (b.0 - a.0, b.1 - a.1);
        let length_squared = direction.0 * direction.0 + direction.1 * direction.1;
        if length_squared <= PARAM_TOLERANCE {
            return (f64::from(cursor.0) - a.0).hypot(f64::from(cursor.1) - a.1);
        }
        let parameter = (((f64::from(cursor.0) - a.0) * direction.0
            + (f64::from(cursor.1) - a.1) * direction.1)
            / length_squared)
            .clamp(0.0, 1.0);
        (f64::from(cursor.0) - (a.0 + parameter * direction.0))
            .hypot(f64::from(cursor.1) - (a.1 + parameter * direction.1))
    };
    let result = model
        .entities
        .iter()
        .filter(|entity| !model.construction.contains(&entity.id()))
        .filter_map(|entity| {
            let distance = match entity {
                SketchEntity::Line { p0, p1, .. } => segment_distance(point(*p0)?, point(*p1)?),
                SketchEntity::Circle { center, radius, .. } => {
                    let center = point(*center)?;
                    ((f64::from(cursor.0) - center.0).hypot(f64::from(cursor.1) - center.1)
                        - radius)
                        .abs()
                }
                SketchEntity::Arc {
                    center,
                    start,
                    end,
                    radius,
                    clockwise,
                    ..
                } => {
                    let center_point = point(*center)?;
                    let start_point = point(*start)?;
                    let end_point = point(*end)?;
                    let arc = Arc {
                        center: (center_point.0 as f32, center_point.1 as f32),
                        radius: *radius as f32,
                        start: (start_point.0 as f32, start_point.1 as f32),
                        end: (end_point.0 as f32, end_point.1 as f32),
                        clockwise: *clockwise,
                    };
                    let angle = point_angle(center_point, (cursor.0.into(), cursor.1.into()));
                    if angle_in_bounds(angle, arc.parameter_bounds()) {
                        ((f64::from(cursor.0) - center_point.0)
                            .hypot(f64::from(cursor.1) - center_point.1)
                            - radius)
                            .abs()
                    } else {
                        segment_distance(start_point, start_point)
                            .min(segment_distance(end_point, end_point))
                    }
                }
                SketchEntity::Ellipse { .. } | SketchEntity::Spline { .. } => {
                    let curve = complex_curve(model, entity).ok().flatten()?;
                    let parameter = curve.parameter((f64::from(cursor.0), f64::from(cursor.1)))?;
                    let candidate = curve.point(parameter)?;
                    (candidate.0 - f64::from(cursor.0)).hypot(candidate.1 - f64::from(cursor.1))
                }
            };
            (distance <= f64::from(tolerance)).then_some((entity, distance))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(entity, _)| entity);
    result.ok_or(TrimError::NoEntity)
}

/// Compute the exact removable span beneath `cursor`.
pub fn preview_trim(
    model: &SketchSolverModel,
    cursor: (f32, f32),
    tolerance: f32,
) -> Result<TrimPreview, TrimError> {
    let target_entity = nearest_target(model, cursor, tolerance)?;
    if let Some(target) = complex_curve(model, target_entity)? {
        return preview_complex_trim(model, target, cursor);
    }
    let target = primitive(model, target_entity)?;
    let mut parameters = Vec::new();
    for other_entity in &model.entities {
        if other_entity.id() == target.id() || model.construction.contains(&other_entity.id()) {
            continue;
        }
        let points = match primitive(model, other_entity) {
            Ok(other) => intersections(target, other)?,
            Err(TrimError::UnsupportedEntity { .. }) => {
                let Some(other) = complex_curve(model, other_entity)? else {
                    continue;
                };
                primitive_target_complex_intersections(target, &other)?
            }
            Err(error) => return Err(error),
        };
        for point in points {
            if let Some(parameter) = target.parameter(point) {
                parameters.push(parameter);
            }
        }
    }
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= PARAM_TOLERANCE);

    let mut removable = SketchCurves::new();
    let mut pieces = Vec::new();
    match target_entity {
        SketchEntity::Line { p0, p1, .. } => {
            let start = model.point(*p0).unwrap().pos;
            let end = model.point(*p1).unwrap().pos;
            let cursor_parameter = target
                .parameter((f64::from(cursor.0), f64::from(cursor.1)))
                .unwrap()
                .clamp(0.0, 1.0);
            let mut bounds = vec![0.0];
            bounds.extend(parameters.into_iter().filter(|parameter| {
                *parameter > PARAM_TOLERANCE && *parameter < 1.0 - PARAM_TOLERANCE
            }));
            bounds.push(1.0);
            let remove_index = bounds
                .windows(2)
                .position(|bounds| {
                    cursor_parameter >= bounds[0] - PARAM_TOLERANCE
                        && cursor_parameter <= bounds[1] + PARAM_TOLERANCE
                })
                .unwrap_or(0);
            let at = |parameter: f64| {
                (
                    (start.0 + (end.0 - start.0) * parameter) as f32,
                    (start.1 + (end.1 - start.1) * parameter) as f32,
                )
            };
            removable.add_line(at(bounds[remove_index]), at(bounds[remove_index + 1]));
            for (index, interval) in bounds.windows(2).enumerate() {
                if index != remove_index && interval[1] - interval[0] > PARAM_TOLERANCE {
                    pieces.push(TrimPiece::Line {
                        start: at(interval[0]),
                        end: at(interval[1]),
                        contains_original_start: interval[0] <= PARAM_TOLERANCE,
                        contains_original_end: interval[1] >= 1.0 - PARAM_TOLERANCE,
                    });
                }
            }
        }
        SketchEntity::Circle { center, radius, .. } => {
            let center_point = model.point(*center).unwrap().pos;
            if parameters.is_empty() {
                removable.add_circle(
                    (center_point.0 as f32, center_point.1 as f32),
                    *radius as f32,
                );
            } else {
                if parameters.len() == 1 {
                    return Err(TrimError::TangentialCircle { id: target.id() });
                }
                let cursor_parameter =
                    point_angle(center_point, (f64::from(cursor.0), f64::from(cursor.1)))
                        .rem_euclid(std::f64::consts::TAU);
                let interval_contains = |start: f64, end: f64, parameter: f64| {
                    let parameter = if parameter < start {
                        parameter + std::f64::consts::TAU
                    } else {
                        parameter
                    };
                    parameter >= start - PARAM_TOLERANCE && parameter <= end + PARAM_TOLERANCE
                };
                let remove_index = (0..parameters.len())
                    .find(|index| {
                        let start = parameters[*index];
                        let end = if *index + 1 < parameters.len() {
                            parameters[*index + 1]
                        } else {
                            parameters[0] + std::f64::consts::TAU
                        };
                        interval_contains(start, end, cursor_parameter)
                    })
                    .unwrap_or(0);
                for index in 0..parameters.len() {
                    let first = parameters[index];
                    let last = if index + 1 < parameters.len() {
                        parameters[index + 1]
                    } else {
                        parameters[0] + std::f64::consts::TAU
                    };
                    let mut interval_pieces = Vec::new();
                    split_arc_piece(
                        &mut interval_pieces,
                        *center,
                        (center_point.0 as f32, center_point.1 as f32),
                        *radius as f32,
                        first,
                        last,
                        (std::f64::consts::TAU, f64::NAN),
                    );
                    if index == remove_index {
                        for piece in &interval_pieces {
                            append_preview_piece(&mut removable, piece);
                        }
                    } else {
                        pieces.extend(interval_pieces);
                    }
                }
            }
        }
        SketchEntity::Arc { center, radius, .. } => {
            let center_point = model.point(*center).unwrap().pos;
            let Primitive::Circle {
                bounds: Some((first, last)),
                ..
            } = target
            else {
                unreachable!()
            };
            let cursor_parameter = target
                .parameter((f64::from(cursor.0), f64::from(cursor.1)))
                .unwrap_or(0.5)
                .clamp(0.0, 1.0);
            let mut bounds = vec![0.0];
            bounds.extend(parameters.into_iter().filter(|parameter| {
                *parameter > PARAM_TOLERANCE && *parameter < 1.0 - PARAM_TOLERANCE
            }));
            bounds.push(1.0);
            let remove_index = bounds
                .windows(2)
                .position(|bounds| {
                    cursor_parameter >= bounds[0] - PARAM_TOLERANCE
                        && cursor_parameter <= bounds[1] + PARAM_TOLERANCE
                })
                .unwrap_or(0);
            for (index, interval) in bounds.windows(2).enumerate() {
                let mut interval_pieces = Vec::new();
                split_arc_piece(
                    &mut interval_pieces,
                    *center,
                    (center_point.0 as f32, center_point.1 as f32),
                    *radius as f32,
                    first + (last - first) * interval[0],
                    first + (last - first) * interval[1],
                    (first, last),
                );
                if index == remove_index {
                    for piece in &interval_pieces {
                        append_preview_piece(&mut removable, piece);
                    }
                } else {
                    pieces.extend(interval_pieces);
                }
            }
        }
        SketchEntity::Ellipse { .. } | SketchEntity::Spline { .. } => unreachable!(),
    }
    Ok(TrimPreview {
        target: target.id(),
        removable,
        pieces,
    })
}

fn constraint_references_entity(constraint: &Constraint, target: EntityId) -> bool {
    match constraint {
        Constraint::Horizontal { line, .. } | Constraint::Vertical { line, .. } => *line == target,
        Constraint::Radius { circle, .. } | Constraint::Diameter { circle, .. } => {
            *circle == target
        }
        Constraint::Parallel { a, b, .. }
        | Constraint::Perpendicular { a, b, .. }
        | Constraint::Equal { a, b, .. }
        | Constraint::Angle { a, b, .. }
        | Constraint::LineDistance { a, b, .. }
        | Constraint::Concentric { a, b, .. }
        | Constraint::Collinear { a, b, .. } => *a == target || *b == target,
        Constraint::Tangent { line, circle, .. } => *line == target || *circle == target,
        Constraint::Midpoint { line, .. } => *line == target,
        Constraint::PointOnObject { object, .. } => *object == target,
        Constraint::Symmetric { axis, .. } => *axis == target,
        Constraint::SplineTangent { spline, line, .. } => *spline == target || *line == target,
        Constraint::SplineCurvature { spline, .. } => *spline == target,
        Constraint::Coincident { .. }
        | Constraint::Distance { .. }
        | Constraint::Fixed { .. }
        | Constraint::DistanceX { .. }
        | Constraint::DistanceY { .. } => false,
    }
}

fn constraint_is_valid(
    constraint: &Constraint,
    entities: &HashSet<EntityId>,
    points: &HashSet<EntityId>,
) -> bool {
    match constraint {
        Constraint::Coincident { a, b, .. }
        | Constraint::Distance { a, b, .. }
        | Constraint::DistanceX { a, b, .. }
        | Constraint::DistanceY { a, b, .. } => points.contains(a) && points.contains(b),
        Constraint::Horizontal { line, .. } | Constraint::Vertical { line, .. } => {
            entities.contains(line)
        }
        Constraint::Radius { circle, .. } | Constraint::Diameter { circle, .. } => {
            entities.contains(circle)
        }
        Constraint::Parallel { a, b, .. }
        | Constraint::Perpendicular { a, b, .. }
        | Constraint::Equal { a, b, .. }
        | Constraint::Angle { a, b, .. }
        | Constraint::LineDistance { a, b, .. }
        | Constraint::Concentric { a, b, .. }
        | Constraint::Collinear { a, b, .. } => entities.contains(a) && entities.contains(b),
        Constraint::Tangent { line, circle, .. } => {
            entities.contains(line) && entities.contains(circle)
        }
        Constraint::Fixed { p, .. } => points.contains(p),
        Constraint::Midpoint { point, line, .. } => {
            points.contains(point) && entities.contains(line)
        }
        Constraint::PointOnObject { point, object, .. } => {
            points.contains(point) && entities.contains(object)
        }
        Constraint::Symmetric { a, b, axis, .. } => {
            points.contains(a) && points.contains(b) && entities.contains(axis)
        }
        Constraint::SplineTangent { spline, line, .. } => {
            entities.contains(spline) && entities.contains(line)
        }
        Constraint::SplineCurvature { spline, .. } => entities.contains(spline),
    }
}

/// Apply the exact preview plan as one model mutation.
pub fn apply_trim_preview(
    model: &mut SketchSolverModel,
    preview: TrimPreview,
    next_entity_id: &mut u32,
) -> TrimOutcome {
    let target = preview.target;
    let original_entity = model
        .entities
        .iter()
        .find(|entity| entity.id() == target)
        .cloned();
    let original_start = match &original_entity {
        Some(SketchEntity::Line { p0, .. }) => Some(*p0),
        Some(SketchEntity::Arc { start, .. }) => Some(*start),
        _ => None,
    };
    let original_end = match &original_entity {
        Some(SketchEntity::Line { p1, .. }) => Some(*p1),
        Some(SketchEntity::Arc { end, .. }) => Some(*end),
        _ => None,
    };
    let owner = original_entity
        .as_ref()
        .and_then(SketchEntity::derived_from);
    let floor = model
        .points
        .iter()
        .map(|point| point.id.0 + 1)
        .chain(model.entities.iter().map(|entity| entity.id().0 + 1))
        .max()
        .unwrap_or(0);
    *next_entity_id = (*next_entity_id).max(floor);
    let mut allocate = || {
        let id = EntityId(*next_entity_id);
        *next_entity_id += 1;
        id
    };
    model.entities.retain(|entity| entity.id() != target);

    let mut created = Vec::new();
    let multiple_pieces = preview.pieces.len() > 1;
    for piece in preview.pieces {
        let (contains_start, contains_end) = match &piece {
            TrimPiece::Line {
                contains_original_start,
                contains_original_end,
                ..
            }
            | TrimPiece::Arc {
                contains_original_start,
                contains_original_end,
                ..
            }
            | TrimPiece::Spline {
                contains_original_start,
                contains_original_end,
                ..
            } => (*contains_original_start, *contains_original_end),
            TrimPiece::Ellipse {
                contains_original_start,
                ..
            } => (*contains_original_start, false),
        };
        let entity_id = if contains_start { target } else { allocate() };
        let mut point_at = |position: (f32, f32), reuse: Option<EntityId>| {
            if let Some(id) = reuse {
                if let Some(point) = model.points.iter_mut().find(|point| point.id == id) {
                    point.pos = (f64::from(position.0), f64::from(position.1));
                    return id;
                }
            }
            if let Some(point) = model.points.iter().find(|point| {
                (point.pos.0 - f64::from(position.0)).hypot(point.pos.1 - f64::from(position.1))
                    <= GEOMETRY_TOLERANCE
            }) {
                return point.id;
            }
            let id = allocate();
            model.points.push(SketchPoint {
                id,
                pos: (f64::from(position.0), f64::from(position.1)),
            });
            id
        };
        let entity = match piece {
            TrimPiece::Line { start, end, .. } => SketchEntity::Line {
                id: entity_id,
                p0: point_at(start, contains_start.then_some(original_start).flatten()),
                p1: point_at(end, contains_end.then_some(original_end).flatten()),
                derived_from: owner,
            },
            TrimPiece::Arc {
                center,
                center_point,
                radius,
                start,
                end,
                clockwise,
                ..
            } => SketchEntity::Arc {
                id: entity_id,
                center: point_at(center_point, Some(center)),
                start: point_at(start, contains_start.then_some(original_start).flatten()),
                end: point_at(end, contains_end.then_some(original_end).flatten()),
                radius: f64::from(radius),
                clockwise,
                derived_from: owner,
            },
            TrimPiece::Ellipse {
                center,
                center_point,
                major_axis,
                minor_axis,
                start_parameter,
                end_parameter,
                ..
            } => SketchEntity::Ellipse {
                id: entity_id,
                center: point_at(center_point, Some(center)),
                major_axis,
                minor_axis,
                start_parameter,
                end_parameter,
                closed: false,
                derived_from: owner,
            },
            TrimPiece::Spline {
                point_ids, spline, ..
            } => SketchEntity::Spline {
                id: entity_id,
                points: point_ids,
                kind: spline.kind,
                degree: spline.degree,
                knots: spline.knots,
                weights: spline.weights,
                closed: spline.closed,
                periodic: spline.periodic,
                continuity: spline.continuity,
                trim: spline.trim,
                derived_from: owner,
            },
        };
        created.push(entity_id);
        model.entities.push(entity);
    }

    let mut report = TrimConstraintReport::default();
    if multiple_pieces {
        report.ambiguous = model
            .constraints
            .iter()
            .filter(|constraint| constraint_references_entity(constraint, target))
            .map(Constraint::id)
            .collect();
        let ambiguous: HashSet<_> = report.ambiguous.iter().copied().collect();
        model
            .constraints
            .retain(|constraint| !ambiguous.contains(&constraint.id()));
    }
    let entity_ids: HashSet<_> = model.entities.iter().map(SketchEntity::id).collect();
    let mut referenced_points = HashSet::new();
    for entity in &model.entities {
        match entity {
            SketchEntity::Line { p0, p1, .. } => {
                referenced_points.extend([*p0, *p1]);
            }
            SketchEntity::Circle { center, .. } | SketchEntity::Ellipse { center, .. } => {
                referenced_points.insert(*center);
            }
            SketchEntity::Arc {
                center, start, end, ..
            } => {
                referenced_points.extend([*center, *start, *end]);
            }
            SketchEntity::Spline { points, .. } => referenced_points.extend(points),
        }
    }
    let before_constraints: HashSet<_> = model.constraints.iter().map(Constraint::id).collect();
    model
        .constraints
        .retain(|constraint| constraint_is_valid(constraint, &entity_ids, &referenced_points));
    let after_constraints: HashSet<_> = model.constraints.iter().map(Constraint::id).collect();
    report.removed = before_constraints
        .difference(&after_constraints)
        .copied()
        .collect();
    report.removed.sort();
    report.ambiguous.sort();
    report.ambiguous.dedup();
    model
        .points
        .retain(|point| referenced_points.contains(&point.id));
    model.construction.retain(|id| entity_ids.contains(id));
    model
        .driven_dimensions
        .retain(|id| after_constraints.contains(id));
    model.offsets.retain(|offset| {
        !offset.sources.contains(&target)
            && offset
                .sources
                .iter()
                .all(|source| entity_ids.contains(source))
    });
    model.patterns.retain(|pattern| {
        !pattern.sources.contains(&target)
            && pattern
                .sources
                .iter()
                .all(|source| entity_ids.contains(source))
    });

    TrimOutcome {
        target,
        created_entities: created,
        constraints: report,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(id: u32, x: f64, y: f64) -> SketchPoint {
        SketchPoint {
            id: EntityId(id),
            pos: (x, y),
        }
    }

    fn line(id: u32, p0: u32, p1: u32) -> SketchEntity {
        SketchEntity::Line {
            id: EntityId(id),
            p0: EntityId(p0),
            p1: EntityId(p1),
            derived_from: None,
        }
    }

    #[test]
    fn line_preview_and_apply_share_the_exact_middle_span() {
        let mut model = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, 10.0, 0.0),
                point(3, 3.0, -2.0),
                point(4, 3.0, 2.0),
                point(5, 7.0, -2.0),
                point(6, 7.0, 2.0),
            ],
            entities: vec![line(10, 1, 2), line(11, 3, 4), line(12, 5, 6)],
            constraints: vec![Constraint::Horizontal {
                id: EntityId(50),
                line: EntityId(10),
            }],
            ..SketchSolverModel::default()
        };
        let preview = preview_trim(&model, (5.0, 0.0), 0.2).unwrap();
        assert_eq!(preview.target, EntityId(10));
        assert_eq!(preview.removable.segments.len(), 1);
        assert!((preview.removable.segments[0].a.0 - 3.0).abs() < 1.0e-5);
        assert!((preview.removable.segments[0].b.0 - 7.0).abs() < 1.0e-5);

        let mut next = 100;
        let outcome = apply_trim_preview(&mut model, preview, &mut next);
        assert_eq!(outcome.created_entities[0], EntityId(10));
        let second_remnant = outcome.created_entities[1];
        assert_ne!(second_remnant, EntityId(10));
        assert_eq!(outcome.constraints.ambiguous, vec![EntityId(50)]);
        assert!(outcome.constraints.removed.is_empty());
        assert!(model.constraints.is_empty());
        let target = model
            .entities
            .iter()
            .find(|entity| entity.id() == EntityId(10))
            .unwrap();
        assert!(matches!(
            target,
            SketchEntity::Line {
                p0: EntityId(1),
                ..
            }
        ));
        assert!(model.entities.iter().any(|entity| matches!(
            entity,
            SketchEntity::Line {
                id,
                p1: EntityId(2),
                ..
            } if *id == second_remnant
        )));
    }

    #[test]
    fn removing_original_line_end_removes_its_orphan_constraint() {
        let mut model = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, 10.0, 0.0),
                point(3, 3.0, -2.0),
                point(4, 3.0, 2.0),
                point(5, 7.0, -2.0),
                point(6, 7.0, 2.0),
            ],
            entities: vec![line(10, 1, 2), line(11, 3, 4), line(12, 5, 6)],
            constraints: vec![Constraint::Fixed {
                id: EntityId(51),
                p: EntityId(2),
            }],
            ..SketchSolverModel::default()
        };
        let preview = preview_trim(&model, (9.0, 0.0), 0.2).unwrap();
        let mut next = 100;
        let outcome = apply_trim_preview(&mut model, preview, &mut next);
        assert_eq!(outcome.constraints.removed, vec![EntityId(51)]);
        assert!(!model.points.iter().any(|point| point.id == EntityId(2)));
    }

    #[test]
    fn circle_trim_preserves_id_only_when_remnant_contains_parameter_zero() {
        let base = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, 0.0, -10.0),
                point(3, 0.0, 10.0),
            ],
            entities: vec![
                SketchEntity::Circle {
                    id: EntityId(10),
                    center: EntityId(1),
                    radius: 5.0,
                    derived_from: None,
                },
                line(11, 2, 3),
            ],
            ..SketchSolverModel::default()
        };

        let mut remove_left = base.clone();
        let preview = preview_trim(&remove_left, (-5.0, 0.0), 0.2).unwrap();
        assert_eq!(preview.removable.arcs.len(), 2);
        let mut next = 100;
        apply_trim_preview(&mut remove_left, preview, &mut next);
        assert!(remove_left
            .entities
            .iter()
            .any(|entity| entity.id() == EntityId(10)));

        let mut remove_right = base;
        let preview = preview_trim(&remove_right, (5.0, 0.0), 0.2).unwrap();
        let mut next = 100;
        apply_trim_preview(&mut remove_right, preview, &mut next);
        assert!(!remove_right
            .entities
            .iter()
            .any(|entity| entity.id() == EntityId(10)));
    }

    #[test]
    fn arc_trim_splits_at_exact_line_intersections() {
        let mut model = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, 5.0, 0.0),
                point(3, -5.0, 0.0),
                point(4, -2.0, -8.0),
                point(5, -2.0, 8.0),
                point(6, 2.0, -8.0),
                point(7, 2.0, 8.0),
            ],
            entities: vec![
                SketchEntity::Arc {
                    id: EntityId(10),
                    center: EntityId(1),
                    start: EntityId(2),
                    end: EntityId(3),
                    radius: 5.0,
                    clockwise: false,
                    derived_from: None,
                },
                line(11, 4, 5),
                line(12, 6, 7),
            ],
            ..SketchSolverModel::default()
        };
        let preview = preview_trim(&model, (0.0, 5.0), 0.25).unwrap();
        assert_eq!(preview.target, EntityId(10));
        assert!(!preview.removable.arcs.is_empty());
        let start_x = preview.removable.arcs.first().unwrap().start.0;
        let end_x = preview.removable.arcs.last().unwrap().end.0;
        assert!((start_x - 2.0).abs() < 1.0e-4 || (start_x + 2.0).abs() < 1.0e-4);
        assert!((end_x - 2.0).abs() < 1.0e-4 || (end_x + 2.0).abs() < 1.0e-4);
        let mut next = 100;
        let outcome = apply_trim_preview(&mut model, preview, &mut next);
        assert!(outcome.created_entities.contains(&EntityId(10)));
        assert!(outcome
            .created_entities
            .iter()
            .any(|id| *id != EntityId(10)));
    }

    #[test]
    fn coincident_line_overlap_is_typed_ambiguous() {
        let model = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, 10.0, 0.0),
                point(3, 2.0, 0.0),
                point(4, 8.0, 0.0),
            ],
            entities: vec![line(10, 1, 2), line(11, 3, 4)],
            ..SketchSolverModel::default()
        };
        assert!(matches!(
            preview_trim(&model, (5.0, 0.0), 0.2),
            Err(TrimError::AmbiguousOverlap {
                first: EntityId(10),
                second: EntityId(11)
            }) | Err(TrimError::AmbiguousOverlap {
                first: EntityId(11),
                second: EntityId(10)
            })
        ));
    }

    #[test]
    fn ellipse_trim_uses_controlled_conic_intersections_and_preserves_start_id() {
        let mut model = SketchSolverModel {
            points: vec![
                point(1, 0.0, 0.0),
                point(2, -2.0, -5.0),
                point(3, -2.0, 5.0),
                point(4, 2.0, -5.0),
                point(5, 2.0, 5.0),
            ],
            entities: vec![
                SketchEntity::Ellipse {
                    id: EntityId(10),
                    center: EntityId(1),
                    major_axis: [5.0, 0.0],
                    minor_axis: [0.0, 3.0],
                    start_parameter: 0.0,
                    end_parameter: std::f64::consts::TAU,
                    closed: true,
                    derived_from: None,
                },
                line(11, 2, 3),
                line(12, 4, 5),
            ],
            ..SketchSolverModel::default()
        };

        let preview = preview_trim(&model, (0.0, 3.0), 0.25).unwrap();
        assert_eq!(preview.target, EntityId(10));
        assert!(!preview.removable.segments.is_empty());
        let mut next = 100;
        let outcome = apply_trim_preview(&mut model, preview, &mut next);
        assert!(outcome.created_entities.contains(&EntityId(10)));
        assert!(model.entities.iter().filter(|entity| matches!(
            entity,
            SketchEntity::Ellipse { id, closed: false, .. } if outcome.created_entities.contains(id)
        )).count() >= 2);
    }

    #[test]
    fn control_and_fit_splines_trim_between_line_intersections() {
        for kind in [
            crate::sketch::SplineKind::ControlPoint,
            crate::sketch::SplineKind::FitPoint,
        ] {
            let mut model = SketchSolverModel {
                points: vec![
                    point(1, 0.0, 0.0),
                    point(2, 5.0, 10.0),
                    point(3, 10.0, 0.0),
                    point(4, 3.0, -2.0),
                    point(5, 3.0, 12.0),
                    point(6, 7.0, -2.0),
                    point(7, 7.0, 12.0),
                ],
                entities: vec![
                    SketchEntity::Spline {
                        id: EntityId(10),
                        points: vec![EntityId(1), EntityId(2), EntityId(3)],
                        kind,
                        degree: 2,
                        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                        weights: Vec::new(),
                        closed: false,
                        periodic: false,
                        continuity: crate::sketch::SplineContinuity::Tangent,
                        trim: None,
                        derived_from: None,
                    },
                    line(11, 4, 5),
                    line(12, 6, 7),
                ],
                ..SketchSolverModel::default()
            };

            let cursor = if kind == crate::sketch::SplineKind::ControlPoint {
                (5.0, 5.0)
            } else {
                (5.0, 10.0)
            };
            let preview = preview_trim(&model, cursor, 0.75)
                .unwrap_or_else(|error| panic!("{kind:?}: {error}"));
            assert_eq!(preview.target, EntityId(10));
            assert_eq!(preview.removable.splines.len(), 1);
            let mut next = 100;
            let outcome = apply_trim_preview(&mut model, preview, &mut next);
            assert_eq!(outcome.created_entities.len(), 2);
            assert_eq!(outcome.created_entities[0], EntityId(10));
            let trims: Vec<_> = model
                .entities
                .iter()
                .filter_map(|entity| match entity {
                    SketchEntity::Spline { trim, .. } => *trim,
                    _ => None,
                })
                .collect();
            assert_eq!(trims.len(), 2);
            assert!(trims.iter().all(|(first, last)| last > first));
        }
    }
}
