//! Associative linear and circular sketch patterns.

use std::collections::HashMap;

use super::{
    constraints::bake_entities_to_curves, Dimension, EntityId, SketchCurves, SketchSolverModel,
};

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchPatternOperation {
    pub id: EntityId,
    pub sources: Vec<EntityId>,
    pub kind: SketchPatternKind,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SketchPatternKind {
    Linear {
        direction: [f64; 2],
        spacing: Dimension,
        /// Total occurrences, including the source geometry.
        count: u32,
    },
    Circular {
        center: [f64; 2],
        total_angle_deg: Dimension,
        /// Total occurrences, including the source geometry.
        count: u32,
    },
}

/// Stable identity for one read-only generated span.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct SketchPatternSpanId {
    pub operation: EntityId,
    pub instance: u32,
    pub source_entity: EntityId,
    pub span: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SketchPatternEvaluation {
    pub curves: SketchCurves,
    pub spans: Vec<SketchPatternSpanId>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SketchPatternError {
    EmptySources,
    MissingSource(EntityId),
    InvalidCount,
    InvalidDirection,
    InvalidSpacing,
    InvalidAngle,
}

impl std::fmt::Display for SketchPatternError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptySources => formatter.write_str("sketch pattern needs source entities"),
            Self::MissingSource(id) => {
                write!(formatter, "sketch pattern source {} is missing", id.0)
            }
            Self::InvalidCount => formatter.write_str("sketch pattern count must be at least 1"),
            Self::InvalidDirection => {
                formatter.write_str("linear sketch pattern direction must be finite and non-zero")
            }
            Self::InvalidSpacing => {
                formatter.write_str("linear sketch pattern spacing must be finite and non-zero")
            }
            Self::InvalidAngle => {
                formatter.write_str("circular sketch pattern angle must be finite and non-zero")
            }
        }
    }
}

impl std::error::Error for SketchPatternError {}

fn transformed_curves(
    source: &SketchCurves,
    transform: impl Fn((f32, f32)) -> (f32, f32),
) -> SketchCurves {
    let mut result = SketchCurves::new();
    result
        .segments
        .extend(source.segments.iter().map(|segment| super::LineSegment {
            a: transform(segment.a),
            b: transform(segment.b),
        }));
    result
        .circles
        .extend(source.circles.iter().map(|circle| super::Circle {
            center: transform(circle.center),
            radius: circle.radius,
        }));
    result.arcs.extend(source.arcs.iter().map(|arc| super::Arc {
        center: transform(arc.center),
        radius: arc.radius,
        start: transform(arc.start),
        end: transform(arc.end),
        clockwise: arc.clockwise,
    }));
    result.splines.extend(source.splines.iter().map(|spline| {
        let mut spline = spline.clone();
        spline.points = spline.points.iter().copied().map(&transform).collect();
        spline
    }));
    result
}

fn span_count(curves: &SketchCurves) -> usize {
    curves.segments.len() + curves.circles.len() + curves.arcs.len() + curves.splines.len()
}

type PointTransform = Box<dyn Fn((f32, f32)) -> (f32, f32)>;

pub fn evaluate_associative_pattern(
    model: &SketchSolverModel,
    operation: &SketchPatternOperation,
    vars: &HashMap<String, f64>,
) -> Result<SketchPatternEvaluation, SketchPatternError> {
    if operation.sources.is_empty() {
        return Err(SketchPatternError::EmptySources);
    }
    let (count, transforms): (u32, Vec<PointTransform>) = match &operation.kind {
        SketchPatternKind::Linear {
            direction,
            spacing,
            count,
        } => {
            if *count == 0 {
                return Err(SketchPatternError::InvalidCount);
            }
            let length = direction[0].hypot(direction[1]);
            if !length.is_finite() || length <= f64::EPSILON {
                return Err(SketchPatternError::InvalidDirection);
            }
            let spacing = f64::from(spacing.resolve(vars));
            if !spacing.is_finite() || spacing.abs() <= f64::EPSILON {
                return Err(SketchPatternError::InvalidSpacing);
            }
            let unit = [direction[0] / length, direction[1] / length];
            let transforms = (1..*count)
                .map(|instance| {
                    let offset = [
                        unit[0] * spacing * f64::from(instance),
                        unit[1] * spacing * f64::from(instance),
                    ];
                    Box::new(move |point: (f32, f32)| {
                        (
                            (f64::from(point.0) + offset[0]) as f32,
                            (f64::from(point.1) + offset[1]) as f32,
                        )
                    }) as PointTransform
                })
                .collect();
            (*count, transforms)
        }
        SketchPatternKind::Circular {
            center,
            total_angle_deg,
            count,
        } => {
            if *count == 0 {
                return Err(SketchPatternError::InvalidCount);
            }
            let angle = f64::from(total_angle_deg.resolve(vars));
            if !angle.is_finite() || angle.abs() <= f64::EPSILON {
                return Err(SketchPatternError::InvalidAngle);
            }
            let full = (angle.abs() - 360.0).abs() <= 1.0e-9;
            let step = if *count == 1 {
                0.0
            } else if full {
                angle.to_radians() / f64::from(*count)
            } else {
                angle.to_radians() / f64::from(*count - 1)
            };
            let center = *center;
            let transforms = (1..*count)
                .map(|instance| {
                    let angle = step * f64::from(instance);
                    Box::new(move |point: (f32, f32)| {
                        let relative = [
                            f64::from(point.0) - center[0],
                            f64::from(point.1) - center[1],
                        ];
                        let (sin, cos) = angle.sin_cos();
                        (
                            (center[0] + relative[0] * cos - relative[1] * sin) as f32,
                            (center[1] + relative[0] * sin + relative[1] * cos) as f32,
                        )
                    }) as PointTransform
                })
                .collect();
            (*count, transforms)
        }
    };
    if count == 1 {
        return Ok(SketchPatternEvaluation {
            curves: SketchCurves::new(),
            spans: Vec::new(),
        });
    }

    let mut source_curves = Vec::new();
    for source in &operation.sources {
        let mut matches: Vec<_> = model
            .entities
            .iter()
            .filter(|entity| entity.id() == *source || entity.derived_from() == Some(*source))
            .cloned()
            .collect();
        matches.sort_by_key(|entity| entity.id());
        if matches.is_empty() {
            return Err(SketchPatternError::MissingSource(*source));
        }
        let mut curves = SketchCurves::new();
        for entity in matches {
            let mut isolated = model.clone();
            isolated.entities = vec![entity];
            isolated.construction.clear();
            isolated.offsets.clear();
            isolated.patterns.clear();
            curves.extend_curves(&bake_entities_to_curves(&isolated));
        }
        source_curves.push((*source, curves));
    }

    let mut curves = SketchCurves::new();
    let mut spans = Vec::new();
    for (instance_offset, transform) in transforms.iter().enumerate() {
        let instance = instance_offset as u32 + 1;
        for (source, source_geometry) in &source_curves {
            let transformed = transformed_curves(source_geometry, transform.as_ref());
            spans.extend(
                (0..span_count(&transformed)).map(|span| SketchPatternSpanId {
                    operation: operation.id,
                    instance,
                    source_entity: *source,
                    span: span as u32,
                }),
            );
            curves.extend_curves(&transformed);
        }
    }
    Ok(SketchPatternEvaluation { curves, spans })
}

pub fn apply_associative_patterns(
    curves: &mut SketchCurves,
    model: &SketchSolverModel,
    vars: &HashMap<String, f64>,
) -> Vec<(EntityId, SketchPatternError)> {
    let mut failures = Vec::new();
    for operation in &model.patterns {
        match evaluate_associative_pattern(model, operation, vars) {
            Ok(pattern) => curves.extend_curves(&pattern.curves),
            Err(error) => failures.push((operation.id, error)),
        }
    }
    failures
}

/// Explicit Dissolve boundary: callers materialize these curves into editable
/// solver entities and then remove the associative operation.
pub fn dissolve_associative_pattern(
    model: &SketchSolverModel,
    operation: &SketchPatternOperation,
    vars: &HashMap<String, f64>,
) -> Result<SketchPatternEvaluation, SketchPatternError> {
    evaluate_associative_pattern(model, operation, vars)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::{SketchEntity, SketchPoint};

    fn line_model() -> SketchSolverModel {
        SketchSolverModel {
            points: vec![
                SketchPoint {
                    id: EntityId(1),
                    pos: (0.0, 0.0),
                },
                SketchPoint {
                    id: EntityId(2),
                    pos: (2.0, 0.0),
                },
            ],
            entities: vec![SketchEntity::Line {
                id: EntityId(3),
                p0: EntityId(1),
                p1: EntityId(2),
                derived_from: None,
            }],
            ..SketchSolverModel::default()
        }
    }

    #[test]
    fn linear_pattern_follows_source_and_keeps_stable_span_identity() {
        let operation = SketchPatternOperation {
            id: EntityId(10),
            sources: vec![EntityId(3)],
            kind: SketchPatternKind::Linear {
                direction: [1.0, 0.0],
                spacing: Dimension::literal(5.0),
                count: 3,
            },
        };
        let mut model = line_model();
        let first = evaluate_associative_pattern(&model, &operation, &HashMap::new()).unwrap();
        assert_eq!(first.curves.segments.len(), 2);
        assert_eq!(first.curves.segments[0].a, (5.0, 0.0));
        assert_eq!(first.curves.segments[1].a, (10.0, 0.0));
        assert_eq!(
            first.spans,
            vec![
                SketchPatternSpanId {
                    operation: EntityId(10),
                    instance: 1,
                    source_entity: EntityId(3),
                    span: 0,
                },
                SketchPatternSpanId {
                    operation: EntityId(10),
                    instance: 2,
                    source_entity: EntityId(3),
                    span: 0,
                },
            ]
        );

        model.points[1].pos = (3.0, 1.0);
        let edited = evaluate_associative_pattern(&model, &operation, &HashMap::new()).unwrap();
        assert_eq!(edited.spans, first.spans);
        assert_eq!(edited.curves.segments[0].b, (8.0, 1.0));
        assert_eq!(
            dissolve_associative_pattern(&model, &operation, &HashMap::new()).unwrap(),
            edited
        );
    }

    #[test]
    fn circular_pattern_count_includes_source_and_uses_deterministic_instances() {
        let operation = SketchPatternOperation {
            id: EntityId(11),
            sources: vec![EntityId(3)],
            kind: SketchPatternKind::Circular {
                center: [0.0, 0.0],
                total_angle_deg: Dimension::literal(360.0),
                count: 4,
            },
        };
        let evaluated =
            evaluate_associative_pattern(&line_model(), &operation, &HashMap::new()).unwrap();
        assert_eq!(evaluated.curves.segments.len(), 3);
        assert!((evaluated.curves.segments[0].b.0).abs() < 1.0e-5);
        assert!((evaluated.curves.segments[0].b.1 - 2.0).abs() < 1.0e-5);
        assert_eq!(
            evaluated
                .spans
                .iter()
                .map(|span| span.instance)
                .collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
    }

    #[test]
    fn missing_source_is_typed_and_appends_no_partial_geometry() {
        let mut model = line_model();
        model.patterns.push(SketchPatternOperation {
            id: EntityId(12),
            sources: vec![EntityId(99)],
            kind: SketchPatternKind::Linear {
                direction: [1.0, 0.0],
                spacing: Dimension::literal(2.0),
                count: 2,
            },
        });
        let mut curves = SketchCurves::new();
        let failures = apply_associative_patterns(&mut curves, &model, &HashMap::new());
        assert!(curves.is_empty());
        assert_eq!(
            failures,
            vec![(
                EntityId(12),
                SketchPatternError::MissingSource(EntityId(99))
            )]
        );
    }
}
