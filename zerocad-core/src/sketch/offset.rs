//! Associative sketch offsets backed by OpenRCAD's analytic offset service.

use std::collections::HashMap;

use openrcad::foundation::{Dir2d, Pnt2d};
use openrcad::geom2d::{Circle2d, CurveSpan, GeomCurve2d, Line2d};
use openrcad::sketch::{offset_curve_chain, OffsetError, OffsetOptions};

use super::{Arc, Dimension, EntityId, SketchCurves, SketchEntity, SketchSolverModel};

/// Persisted intent for one associative offset operation.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchOffsetOperation {
    pub id: EntityId,
    pub sources: Vec<EntityId>,
    pub distance: Dimension,
    pub creation_side_seed: (f32, f32),
}

#[derive(Debug, Clone, PartialEq)]
pub enum SketchOffsetError {
    InvalidDistance,
    MissingSource(EntityId),
    UnsupportedSource { id: EntityId, kind: &'static str },
    Disconnected,
    AmbiguousChain,
    Kernel(OffsetError),
    Pattern(crate::sketch::SketchPatternError),
}

impl std::fmt::Display for SketchOffsetError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDistance => {
                formatter.write_str("offset distance must be finite and nonzero")
            }
            Self::MissingSource(id) => write!(formatter, "offset source {} is missing", id.0),
            Self::UnsupportedSource { id, kind } => {
                write!(
                    formatter,
                    "offset source {} has unsupported {kind} geometry",
                    id.0
                )
            }
            Self::Disconnected => {
                formatter.write_str("offset sources do not form one connected chain")
            }
            Self::AmbiguousChain => {
                formatter.write_str("offset sources contain an ambiguous branch")
            }
            Self::Kernel(error) => error.fmt(formatter),
            Self::Pattern(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for SketchOffsetError {}

fn point(model: &SketchSolverModel, id: EntityId) -> Result<(f32, f32), SketchOffsetError> {
    model
        .point(id)
        .map(|point| (point.pos.0 as f32, point.pos.1 as f32))
        .ok_or(SketchOffsetError::MissingSource(id))
}

fn entity_span(
    model: &SketchSolverModel,
    entity: &SketchEntity,
) -> Result<CurveSpan<EntityId>, SketchOffsetError> {
    match entity {
        SketchEntity::Line { id, p0, p1, .. } => {
            let start = point(model, *p0)?;
            let end = point(model, *p1)?;
            let dx = f64::from(end.0 - start.0);
            let dy = f64::from(end.1 - start.1);
            let length = dx.hypot(dy);
            if length <= 1.0e-9 {
                return Err(SketchOffsetError::Kernel(OffsetError::Collapse { span: 0 }));
            }
            Ok(CurveSpan::new(
                GeomCurve2d::line(Line2d::from_point_dir(
                    Pnt2d::new(f64::from(start.0), f64::from(start.1)),
                    Dir2d::new(dx / length, dy / length),
                )),
                0.0,
                length,
                *id,
            ))
        }
        SketchEntity::Circle {
            id, center, radius, ..
        } => {
            let center = point(model, *center)?;
            if !radius.is_finite() || *radius <= 0.0 {
                return Err(SketchOffsetError::Kernel(OffsetError::Collapse { span: 0 }));
            }
            Ok(CurveSpan::new(
                GeomCurve2d::circle(Circle2d::from_center(
                    Pnt2d::new(f64::from(center.0), f64::from(center.1)),
                    *radius,
                )),
                0.0,
                std::f64::consts::TAU,
                *id,
            ))
        }
        SketchEntity::Arc {
            id,
            center,
            start,
            end,
            radius,
            clockwise,
            ..
        } => {
            let center = point(model, *center)?;
            let start = point(model, *start)?;
            let end = point(model, *end)?;
            let arc = Arc {
                center,
                radius: *radius as f32,
                start,
                end,
                clockwise: *clockwise,
            };
            let (first, last) = arc.parameter_bounds();
            Ok(CurveSpan::new(
                GeomCurve2d::circle(Circle2d::from_center(
                    Pnt2d::new(f64::from(center.0), f64::from(center.1)),
                    *radius,
                )),
                first,
                last,
                *id,
            ))
        }
        SketchEntity::Ellipse { id, .. } => Err(SketchOffsetError::UnsupportedSource {
            id: *id,
            kind: "ellipse",
        }),
        SketchEntity::Spline { id, .. } => Err(SketchOffsetError::UnsupportedSource {
            id: *id,
            kind: "spline",
        }),
    }
}

fn ordered_source_spans(
    model: &SketchSolverModel,
    operation: &SketchOffsetOperation,
    tolerance: f64,
) -> Result<Vec<CurveSpan<EntityId>>, SketchOffsetError> {
    if operation.sources.is_empty() {
        return Err(SketchOffsetError::Disconnected);
    }
    let mut spans = Vec::new();
    for source in &operation.sources {
        let mut matches: Vec<&SketchEntity> = model
            .entities
            .iter()
            .filter(|entity| entity.id() == *source || entity.derived_from() == Some(*source))
            .collect();
        matches.sort_by_key(|entity| entity.id());
        if matches.is_empty() {
            return Err(SketchOffsetError::MissingSource(*source));
        }
        for entity in matches {
            if !model.construction.contains(&entity.id()) {
                spans.push(entity_span(model, entity)?);
            }
        }
    }
    if spans.is_empty() {
        return Err(SketchOffsetError::Disconnected);
    }
    if spans.len() == 1 {
        return Ok(spans);
    }

    let near = |first: Pnt2d, second: Pnt2d| first.distance(&second) <= tolerance;
    let endpoint_degree = |point: Pnt2d, candidates: &[CurveSpan<EntityId>]| {
        candidates
            .iter()
            .flat_map(|span| [span.start(), span.end()])
            .filter(|candidate| near(point, *candidate))
            .count()
    };
    if spans
        .iter()
        .flat_map(|span| [span.start(), span.end()])
        .any(|point| endpoint_degree(point, &spans) > 2)
    {
        return Err(SketchOffsetError::AmbiguousChain);
    }

    let start_index = spans
        .iter()
        .position(|span| endpoint_degree(span.start(), &spans) == 1)
        .or_else(|| {
            spans
                .iter()
                .position(|span| endpoint_degree(span.end(), &spans) == 1)
        })
        .unwrap_or(0);
    let reverse_first = endpoint_degree(spans[start_index].start(), &spans) != 1
        && endpoint_degree(spans[start_index].end(), &spans) == 1;
    let mut first = spans.remove(start_index);
    if reverse_first {
        first = first.reversed();
    }
    let mut ordered = vec![first];
    while !spans.is_empty() {
        let cursor = ordered.last().expect("ordered chain").end();
        let candidates: Vec<(usize, bool)> = spans
            .iter()
            .enumerate()
            .filter_map(|(index, span)| {
                if near(span.start(), cursor) {
                    Some((index, false))
                } else if near(span.end(), cursor) {
                    Some((index, true))
                } else {
                    None
                }
            })
            .collect();
        if candidates.is_empty() {
            return Err(SketchOffsetError::Disconnected);
        }
        if candidates.len() > 1 {
            return Err(SketchOffsetError::AmbiguousChain);
        }
        let (index, reverse) = candidates[0];
        let span = spans.remove(index);
        ordered.push(if reverse { span.reversed() } else { span });
    }
    Ok(ordered)
}

fn append_offset_spans(curves: &mut SketchCurves, spans: &[CurveSpan<EntityId>]) {
    for span in spans {
        match &span.curve {
            GeomCurve2d::Line(_) => {
                let start = span.start();
                let end = span.end();
                curves.add_line(
                    (start.x() as f32, start.y() as f32),
                    (end.x() as f32, end.y() as f32),
                );
            }
            GeomCurve2d::Circle(circle)
                if (span.parameter_length() - std::f64::consts::TAU).abs() <= 1.0e-7 =>
            {
                curves.add_circle(
                    (circle.center().x() as f32, circle.center().y() as f32),
                    circle.radius() as f32,
                );
            }
            GeomCurve2d::Circle(circle) => {
                let start = span.start();
                let end = span.end();
                curves.arcs.push(Arc {
                    center: (circle.center().x() as f32, circle.center().y() as f32),
                    radius: circle.radius() as f32,
                    start: (start.x() as f32, start.y() as f32),
                    end: (end.x() as f32, end.y() as f32),
                    clockwise: span.last < span.first,
                });
            }
            _ => unreachable!("offset engine returns only line/circle spans"),
        }
    }
}

/// Append every successfully rebuilt operation to `curves`. Each operation is
/// atomic: a collapse, branch, gap, or ambiguity appends no partial geometry.
pub fn apply_associative_offsets(
    curves: &mut SketchCurves,
    model: &SketchSolverModel,
    vars: &HashMap<String, f64>,
) -> Vec<(EntityId, SketchOffsetError)> {
    let mut failures = Vec::new();
    for operation in &model.offsets {
        match evaluate_associative_offset(model, operation, vars) {
            Ok(offset) => curves.extend_curves(&offset),
            Err(error) => failures.push((operation.id, error)),
        }
    }
    failures
}

/// Evaluate one operation without mutating the model. Used by live preview and
/// Dissolve as well as the whole-sketch rebuild.
pub fn evaluate_associative_offset(
    model: &SketchSolverModel,
    operation: &SketchOffsetOperation,
    vars: &HashMap<String, f64>,
) -> Result<SketchCurves, SketchOffsetError> {
    let distance = operation.distance.resolve(vars) as f64;
    if !distance.is_finite() || distance.abs() <= 1.0e-9 {
        return Err(SketchOffsetError::InvalidDistance);
    }
    let spans = ordered_source_spans(model, operation, 1.0e-6)?;
    let offset = offset_curve_chain(&spans, distance, OffsetOptions { tolerance: 1.0e-6 })
        .map_err(SketchOffsetError::Kernel)?;
    let mut curves = SketchCurves::new();
    append_offset_spans(&mut curves, &offset.spans);
    Ok(curves)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::SketchPoint;

    fn circle_model(radius: f64, distance: f32) -> SketchSolverModel {
        SketchSolverModel {
            points: vec![SketchPoint {
                id: EntityId(1),
                pos: (2.0, 3.0),
            }],
            entities: vec![SketchEntity::Circle {
                id: EntityId(2),
                center: EntityId(1),
                radius,
                derived_from: None,
            }],
            offsets: vec![SketchOffsetOperation {
                id: EntityId(3),
                sources: vec![EntityId(2)],
                distance: Dimension::literal(distance),
                creation_side_seed: (2.0, 3.0),
            }],
            ..SketchSolverModel::default()
        }
    }

    #[test]
    fn circle_offset_is_associative_and_read_only() {
        let mut model = circle_model(5.0, 2.0);
        let empty = SketchCurves::new();
        let (first, failures) = super::super::effective_curves_solved_checked(
            &empty,
            &[],
            &[],
            &[],
            Some(&model),
            &HashMap::new(),
        );
        assert!(failures.is_empty());
        assert_eq!(first.circles.len(), 2);
        assert_eq!(first.circles[0].radius, 5.0);
        assert_eq!(first.circles[1].radius, 3.0);
        assert_eq!(
            model.entities.len(),
            1,
            "derived geometry must stay read-only"
        );

        if let SketchEntity::Circle { radius, .. } = &mut model.entities[0] {
            *radius = 10.0;
        }
        let (rebuilt, failures) = super::super::effective_curves_solved_checked(
            &empty,
            &[],
            &[],
            &[],
            Some(&model),
            &HashMap::new(),
        );
        assert!(failures.is_empty());
        assert_eq!(rebuilt.circles[1].radius, 8.0);
    }

    #[test]
    fn connected_line_chain_is_ordered_by_topology_not_selection_order() {
        let points = vec![
            SketchPoint {
                id: EntityId(1),
                pos: (0.0, 0.0),
            },
            SketchPoint {
                id: EntityId(2),
                pos: (10.0, 0.0),
            },
            SketchPoint {
                id: EntityId(3),
                pos: (10.0, 10.0),
            },
            SketchPoint {
                id: EntityId(4),
                pos: (0.0, 10.0),
            },
        ];
        let line = |id, p0, p1| SketchEntity::Line {
            id: EntityId(id),
            p0: EntityId(p0),
            p1: EntityId(p1),
            derived_from: None,
        };
        let model = SketchSolverModel {
            points,
            entities: vec![
                line(11, 1, 2),
                line(12, 2, 3),
                line(13, 3, 4),
                line(14, 4, 1),
            ],
            offsets: vec![SketchOffsetOperation {
                id: EntityId(20),
                sources: vec![EntityId(13), EntityId(11), EntityId(14), EntityId(12)],
                distance: Dimension::literal(1.0),
                creation_side_seed: (5.0, 5.0),
            }],
            ..SketchSolverModel::default()
        };
        let mut curves = super::super::constraints::bake_entities_to_curves(&model);
        let failures = apply_associative_offsets(&mut curves, &model, &HashMap::new());
        assert!(failures.is_empty(), "{failures:?}");
        assert_eq!(curves.segments.len(), 8);
        let derived = &curves.segments[4..];
        let min_x = derived
            .iter()
            .flat_map(|segment| [segment.a.0, segment.b.0])
            .fold(f32::INFINITY, f32::min);
        let max_x = derived
            .iter()
            .flat_map(|segment| [segment.a.0, segment.b.0])
            .fold(f32::NEG_INFINITY, f32::max);
        assert!((min_x - 1.0).abs() < 1.0e-4);
        assert!((max_x - 9.0).abs() < 1.0e-4);
    }

    #[test]
    fn collapse_and_missing_source_leave_no_partial_geometry() {
        let mut collapse = circle_model(2.0, 2.0);
        let mut curves = super::super::constraints::bake_entities_to_curves(&collapse);
        let before = curves.clone();
        let failures = apply_associative_offsets(&mut curves, &collapse, &HashMap::new());
        assert_eq!(curves, before);
        assert!(matches!(
            failures.as_slice(),
            [(
                EntityId(3),
                SketchOffsetError::Kernel(OffsetError::Collapse { .. })
            )]
        ));

        collapse.offsets[0].sources = vec![EntityId(99)];
        let failures = apply_associative_offsets(&mut curves, &collapse, &HashMap::new());
        assert!(matches!(
            failures.as_slice(),
            [(EntityId(3), SketchOffsetError::MissingSource(EntityId(99)))]
        ));
    }

    #[test]
    fn offset_operation_round_trip_keeps_expression_and_seed() {
        let operation = SketchOffsetOperation {
            id: EntityId(8),
            sources: vec![EntityId(2), EntityId(4)],
            distance: Dimension {
                value: 3.0,
                expr: Some("wall + 0.5".to_string()),
            },
            creation_side_seed: (7.0, -2.0),
        };
        let bytes = serde_json::to_vec(&operation).unwrap();
        let decoded: SketchOffsetOperation = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, operation);
    }
}
