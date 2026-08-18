//! Analytic offsets for ordered line/arc/circle chains and material regions.
//!
//! The service returns bounded analytic spans and explicit topology failures.
//! It does not tessellate curves to disguise collapse, branching, gaps, or an
//! offset that changes the number of material regions.

use core::f64::consts::TAU;
use core::fmt;

use openrcad_foundation::{Dir2d, Pnt2d};
use openrcad_geom2d::{Circle2d, Curve2d, CurveKind2d, CurveSpan, GeomCurve2d, Line2d};

use crate::{arrange_curve_spans, ArrangementError, ArrangementOptions, ArrangementRegion};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OffsetOptions {
    pub tolerance: f64,
}

impl Default for OffsetOptions {
    fn default() -> Self {
        Self { tolerance: 1.0e-9 }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct OffsetChain<P> {
    pub spans: Vec<CurveSpan<P>>,
    pub closed: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum OffsetError {
    InvalidInput,
    UnsupportedCurve {
        index: usize,
        kind: CurveKind2d,
    },
    Collapse {
        span: usize,
    },
    TopologyChange {
        expected_regions: usize,
        actual_regions: usize,
        expected_holes: usize,
        actual_holes: usize,
    },
    Branch {
        vertex: Pnt2d,
        degree: usize,
    },
    Gap {
        first: usize,
        second: usize,
        distance: f64,
    },
    Ambiguous {
        first: usize,
        second: usize,
        candidates: usize,
    },
    Arrangement(ArrangementError),
}

impl fmt::Display for OffsetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput => formatter.write_str("offset distance and tolerance must be finite"),
            Self::UnsupportedCurve { index, kind } => {
                write!(formatter, "offset span {index} has unsupported {kind:?} geometry")
            }
            Self::Collapse { span } => write!(formatter, "offset collapses span {span}"),
            Self::TopologyChange {
                expected_regions,
                actual_regions,
                expected_holes,
                actual_holes,
            } => write!(
                formatter,
                "offset changes topology (regions {expected_regions}->{actual_regions}, holes {expected_holes}->{actual_holes})"
            ),
            Self::Branch { vertex, degree } => write!(
                formatter,
                "offset input branches at ({:.6}, {:.6}) with degree {degree}",
                vertex.x(),
                vertex.y()
            ),
            Self::Gap {
                first,
                second,
                distance,
            } => write!(
                formatter,
                "offset spans {first}/{second} leave a gap of {distance:.6}"
            ),
            Self::Ambiguous {
                first,
                second,
                candidates,
            } => write!(
                formatter,
                "offset join {first}/{second} has {candidates} equally valid intersections"
            ),
            Self::Arrangement(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for OffsetError {}

#[derive(Clone, Copy, Debug)]
struct JoinCandidate {
    first: f64,
    second: f64,
    point: Pnt2d,
}

/// Offset an already ordered curve chain to its traversal-left by `distance`.
/// Positive values therefore move a CCW material boundary inward; callers that
/// work in material terms should use [`offset_material_region`].
pub fn offset_curve_chain<P: Clone>(
    spans: &[CurveSpan<P>],
    distance: f64,
    options: OffsetOptions,
) -> Result<OffsetChain<P>, OffsetError> {
    if spans.is_empty()
        || !distance.is_finite()
        || !options.tolerance.is_finite()
        || options.tolerance <= 0.0
    {
        return Err(OffsetError::InvalidInput);
    }
    validate_topology(spans, options.tolerance)?;
    let closed = spans.len() == 1 && is_complete_circle(&spans[0], options.tolerance)
        || spans
            .first()
            .zip(spans.last())
            .is_some_and(|(first, last)| first.start().distance(&last.end()) <= options.tolerance);
    let pair_count = if closed { spans.len() } else { spans.len() - 1 };
    for first in 0..pair_count {
        let second = (first + 1) % spans.len();
        let gap = spans[first].end().distance(&spans[second].start());
        if gap > options.tolerance {
            return Err(OffsetError::Gap {
                first,
                second,
                distance: gap,
            });
        }
    }

    let mut offset = spans
        .iter()
        .enumerate()
        .map(|(index, span)| offset_span(span, distance, options.tolerance, index))
        .collect::<Result<Vec<_>, _>>()?;

    if offset.len() > 1 {
        for first in 0..pair_count {
            let second = (first + 1) % offset.len();
            let reference = spans[first].end();
            let candidates = join_candidates(&offset[first], &offset[second], reference);
            if candidates.is_empty() {
                return Err(OffsetError::Gap {
                    first,
                    second,
                    distance: offset[first].end().distance(&offset[second].start()),
                });
            }
            let chosen = choose_join(candidates, reference, options.tolerance, first, second)?;
            offset[first].last = chosen.first;
            offset[second].first = chosen.second;
        }
    }

    for (index, span) in offset.iter().enumerate() {
        if span.start().distance(&span.end()) <= options.tolerance
            && !is_complete_circle(span, options.tolerance)
        {
            return Err(OffsetError::Collapse { span: index });
        }
    }
    Ok(OffsetChain {
        spans: offset,
        closed,
    })
}

/// Offset a material region: positive distance expands its outer boundary and
/// shrinks its holes. Any merge, split, vanished hole, or new hole is reported
/// as [`OffsetError::TopologyChange`] instead of selecting a branch silently.
pub fn offset_material_region<P: Clone>(
    region: &ArrangementRegion<P>,
    distance: f64,
    options: OffsetOptions,
) -> Result<ArrangementRegion<P>, OffsetError> {
    if region.outer.spans.is_empty() {
        return Err(OffsetError::InvalidInput);
    }
    let outer_side = -region.outer.signed_area.signum() * distance;
    let mut spans = offset_curve_chain(&region.outer.spans, outer_side, options)?.spans;
    for hole in &region.holes {
        let hole_side = hole.signed_area.signum() * distance;
        spans.extend(offset_curve_chain(&hole.spans, hole_side, options)?.spans);
    }
    let arrangement = arrange_curve_spans(
        &spans,
        ArrangementOptions {
            tolerance: options.tolerance,
            ..ArrangementOptions::default()
        },
    )
    .map_err(OffsetError::Arrangement)?;
    let expected_regions = 1 + region.holes.len();
    let expected_holes = region.holes.len();
    let matching: Vec<_> = arrangement
        .regions
        .iter()
        .filter(|candidate| candidate.holes.len() == expected_holes)
        .collect();
    if arrangement.regions.len() != expected_regions || matching.len() != 1 {
        let actual_holes = arrangement
            .regions
            .iter()
            .map(|candidate| candidate.holes.len())
            .max()
            .unwrap_or(0);
        return Err(OffsetError::TopologyChange {
            expected_regions,
            actual_regions: arrangement.regions.len(),
            expected_holes,
            actual_holes,
        });
    }
    Ok(matching[0].clone())
}

fn validate_topology<P>(spans: &[CurveSpan<P>], tolerance: f64) -> Result<(), OffsetError> {
    let mut vertices: Vec<(Pnt2d, usize)> = Vec::new();
    for span in spans {
        for point in [span.start(), span.end()] {
            if let Some((_, degree)) = vertices
                .iter_mut()
                .find(|(candidate, _)| candidate.distance(&point) <= tolerance)
            {
                *degree += 1;
            } else {
                vertices.push((point, 1));
            }
        }
    }
    if let Some((vertex, degree)) = vertices.into_iter().find(|(_, degree)| *degree > 2) {
        return Err(OffsetError::Branch { vertex, degree });
    }
    Ok(())
}

fn offset_span<P: Clone>(
    span: &CurveSpan<P>,
    distance: f64,
    tolerance: f64,
    index: usize,
) -> Result<CurveSpan<P>, OffsetError> {
    match &span.curve {
        GeomCurve2d::Line(_) => {
            let start = span.start();
            let end = span.end();
            let dx = end.x() - start.x();
            let dy = end.y() - start.y();
            let length = dx.hypot(dy);
            if length <= tolerance {
                return Err(OffsetError::Collapse { span: index });
            }
            let shifted = Pnt2d::new(
                start.x() - dy / length * distance,
                start.y() + dx / length * distance,
            );
            let direction = Dir2d::try_new(dx / length, dy / length)
                .ok_or(OffsetError::Collapse { span: index })?;
            Ok(CurveSpan::new(
                GeomCurve2d::line(Line2d::from_point_dir(shifted, direction)),
                0.0,
                length,
                span.provenance.clone(),
            ))
        }
        GeomCurve2d::Circle(circle) => {
            let frame = circle.position();
            let determinant = frame.x_direction().x() * frame.y_direction().y()
                - frame.x_direction().y() * frame.y_direction().x();
            let traversal = (span.last - span.first).signum() * determinant.signum();
            let radius = circle.radius() - traversal * distance;
            if !radius.is_finite() || radius <= tolerance {
                return Err(OffsetError::Collapse { span: index });
            }
            Ok(CurveSpan::new(
                GeomCurve2d::circle(Circle2d::new(frame, radius)),
                span.first,
                span.last,
                span.provenance.clone(),
            ))
        }
        curve => Err(OffsetError::UnsupportedCurve {
            index,
            kind: curve.into(),
        }),
    }
}

fn choose_join(
    mut candidates: Vec<JoinCandidate>,
    reference: Pnt2d,
    tolerance: f64,
    first: usize,
    second: usize,
) -> Result<JoinCandidate, OffsetError> {
    candidates.sort_by(|a, b| {
        a.point
            .distance(&reference)
            .total_cmp(&b.point.distance(&reference))
    });
    if candidates.len() > 1
        && (candidates[0].point.distance(&reference) - candidates[1].point.distance(&reference))
            .abs()
            <= tolerance
    {
        return Err(OffsetError::Ambiguous {
            first,
            second,
            candidates: candidates.len(),
        });
    }
    Ok(candidates[0])
}

fn join_candidates<P, Q>(
    first: &CurveSpan<P>,
    second: &CurveSpan<Q>,
    reference: Pnt2d,
) -> Vec<JoinCandidate> {
    if first.end().distance(&second.start()) <= 1.0e-12 {
        return vec![JoinCandidate {
            first: first.last,
            second: second.first,
            point: first.end(),
        }];
    }
    match (&first.curve, &second.curve) {
        (GeomCurve2d::Line(a), GeomCurve2d::Line(b)) => line_line(a, b)
            .into_iter()
            .map(|(pa, pb, point)| JoinCandidate {
                first: pa,
                second: pb,
                point,
            })
            .collect(),
        (GeomCurve2d::Line(line), GeomCurve2d::Circle(circle)) => line_circle(line, circle)
            .into_iter()
            .map(|(line_parameter, circle_parameter, point)| JoinCandidate {
                first: line_parameter,
                second: nearest_periodic(circle_parameter, second.first),
                point,
            })
            .collect(),
        (GeomCurve2d::Circle(circle), GeomCurve2d::Line(line)) => line_circle(line, circle)
            .into_iter()
            .map(|(line_parameter, circle_parameter, point)| JoinCandidate {
                first: nearest_periodic(circle_parameter, first.last),
                second: line_parameter,
                point,
            })
            .collect(),
        (GeomCurve2d::Circle(a), GeomCurve2d::Circle(b)) => circle_circle(a, b)
            .into_iter()
            .map(|(pa, pb, point)| JoinCandidate {
                first: nearest_periodic(pa, first.last),
                second: nearest_periodic(pb, second.first),
                point,
            })
            .collect(),
        _ => {
            let distance = first.end().distance(&second.start());
            if distance <= 1.0e-12 {
                vec![JoinCandidate {
                    first: first.last,
                    second: second.first,
                    point: reference,
                }]
            } else {
                Vec::new()
            }
        }
    }
}

fn line_line(first: &Line2d, second: &Line2d) -> Option<(f64, f64, Pnt2d)> {
    let p = first.location();
    let q = second.location();
    let r = first.direction();
    let s = second.direction();
    let denominator = cross(r.x(), r.y(), s.x(), s.y());
    if denominator.abs() <= 1.0e-14 {
        return None;
    }
    let qmp = (q.x() - p.x(), q.y() - p.y());
    let first_parameter = cross(qmp.0, qmp.1, s.x(), s.y()) / denominator;
    let second_parameter = cross(qmp.0, qmp.1, r.x(), r.y()) / denominator;
    Some((
        first_parameter,
        second_parameter,
        first.point(first_parameter),
    ))
}

fn line_circle(line: &Line2d, circle: &Circle2d) -> Vec<(f64, f64, Pnt2d)> {
    let direction = line.direction();
    let delta = (
        line.location().x() - circle.center().x(),
        line.location().y() - circle.center().y(),
    );
    let b = 2.0 * (delta.0 * direction.x() + delta.1 * direction.y());
    let c = delta.0 * delta.0 + delta.1 * delta.1 - circle.radius() * circle.radius();
    let discriminant = b * b - 4.0 * c;
    if discriminant < -1.0e-12 {
        return Vec::new();
    }
    let root = discriminant.max(0.0).sqrt();
    let mut parameters = vec![(-b - root) * 0.5];
    if root > 1.0e-12 {
        parameters.push((-b + root) * 0.5);
    }
    parameters
        .into_iter()
        .map(|line_parameter| {
            let point = line.point(line_parameter);
            (line_parameter, circle_parameter(circle, point), point)
        })
        .collect()
}

fn circle_circle(first: &Circle2d, second: &Circle2d) -> Vec<(f64, f64, Pnt2d)> {
    let dx = second.center().x() - first.center().x();
    let dy = second.center().y() - first.center().y();
    let center_distance = dx.hypot(dy);
    if center_distance <= 1.0e-14
        || center_distance > first.radius() + second.radius() + 1.0e-12
        || center_distance < (first.radius() - second.radius()).abs() - 1.0e-12
    {
        return Vec::new();
    }
    let along = (first.radius() * first.radius() - second.radius() * second.radius()
        + center_distance * center_distance)
        / (2.0 * center_distance);
    let height = (first.radius() * first.radius() - along * along)
        .max(0.0)
        .sqrt();
    let base = Pnt2d::new(
        first.center().x() + along * dx / center_distance,
        first.center().y() + along * dy / center_distance,
    );
    let perpendicular = (-dy / center_distance, dx / center_distance);
    let mut points = vec![Pnt2d::new(
        base.x() + height * perpendicular.0,
        base.y() + height * perpendicular.1,
    )];
    if height > 1.0e-12 {
        points.push(Pnt2d::new(
            base.x() - height * perpendicular.0,
            base.y() - height * perpendicular.1,
        ));
    }
    points
        .into_iter()
        .map(|point| {
            (
                circle_parameter(first, point),
                circle_parameter(second, point),
                point,
            )
        })
        .collect()
}

fn circle_parameter(circle: &Circle2d, point: Pnt2d) -> f64 {
    let delta = (
        point.x() - circle.center().x(),
        point.y() - circle.center().y(),
    );
    let frame = circle.position();
    let x = delta.0 * frame.x_direction().x() + delta.1 * frame.x_direction().y();
    let y = delta.0 * frame.y_direction().x() + delta.1 * frame.y_direction().y();
    y.atan2(x)
}

fn nearest_periodic(parameter: f64, anchor: f64) -> f64 {
    parameter + ((anchor - parameter) / TAU).round() * TAU
}

fn is_complete_circle<P>(span: &CurveSpan<P>, tolerance: f64) -> bool {
    matches!(span.curve, GeomCurve2d::Circle(_))
        && ((span.last - span.first).abs() - TAU).abs() <= tolerance * 10.0
}

fn cross(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax22d, Dir2d};
    use openrcad_geom2d::Circle2d;

    fn line(id: u32, start: (f64, f64), end: (f64, f64)) -> CurveSpan<u32> {
        let dx = end.0 - start.0;
        let dy = end.1 - start.1;
        let length = dx.hypot(dy);
        CurveSpan::new(
            GeomCurve2d::line(Line2d::from_point_dir(
                Pnt2d::new(start.0, start.1),
                Dir2d::try_new(dx / length, dy / length).expect("test line is non-degenerate"),
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

    fn arc(id: u32, center: (f64, f64), radius: f64, first: f64, last: f64) -> CurveSpan<u32> {
        CurveSpan::new(
            GeomCurve2d::circle(Circle2d::from_center(
                Pnt2d::new(center.0, center.1),
                radius,
            )),
            first,
            last,
            id,
        )
    }

    fn rectangle(size: f64) -> Vec<CurveSpan<u32>> {
        vec![
            line(0, (0.0, 0.0), (size, 0.0)),
            line(1, (size, 0.0), (size, size)),
            line(2, (size, size), (0.0, size)),
            line(3, (0.0, size), (0.0, 0.0)),
        ]
    }

    #[test]
    fn line_chain_offsets_with_exact_corner_intersections() {
        let offset = offset_curve_chain(&rectangle(10.0), -1.0, OffsetOptions::default())
            .expect("outward rectangle offset");
        let arranged = arrange_curve_spans(&offset.spans, ArrangementOptions::default()).unwrap();
        assert_eq!(arranged.regions.len(), 1);
        assert!((arranged.regions[0].area - 144.0).abs() < 1.0e-8);
    }

    #[test]
    fn circle_offset_preserves_analytic_geometry_and_provenance() {
        let offset = offset_curve_chain(&[circle(7, 5.0)], -2.0, OffsetOptions::default())
            .expect("outward circle offset");
        let GeomCurve2d::Circle(result) = &offset.spans[0].curve else {
            panic!("circle must remain analytic");
        };
        assert_eq!(offset.spans[0].provenance, 7);
        assert!((result.radius() - 7.0).abs() < 1.0e-12);
    }

    #[test]
    fn line_arc_line_chain_joins_without_faceting() {
        let chain = vec![
            line(0, (0.0, 0.0), (2.0, 0.0)),
            arc(1, (2.0, 1.0), 1.0, -std::f64::consts::FRAC_PI_2, 0.0),
            line(2, (3.0, 1.0), (3.0, 3.0)),
        ];
        let offset = offset_curve_chain(&chain, 0.2, OffsetOptions::default()).unwrap();
        assert!(matches!(offset.spans[1].curve, GeomCurve2d::Circle(_)));
        assert!(offset.spans[0].end().distance(&offset.spans[1].start()) < 1.0e-10);
        assert!(offset.spans[1].end().distance(&offset.spans[2].start()) < 1.0e-10);
    }

    #[test]
    fn material_annulus_expands_outer_and_shrinks_hole() {
        let arrangement = arrange_curve_spans(
            &[circle(0, 10.0), circle(1, 4.0)],
            ArrangementOptions::default(),
        )
        .unwrap();
        let annulus = arrangement
            .regions
            .iter()
            .find(|region| region.holes.len() == 1)
            .unwrap();
        let offset = offset_material_region(annulus, 1.0, OffsetOptions::default()).unwrap();
        assert_eq!(offset.holes.len(), 1);
        let expected = std::f64::consts::PI * (11.0_f64.powi(2) - 3.0_f64.powi(2));
        assert!((offset.area - expected).abs() < 1.0e-7);
    }

    #[test]
    fn collapsing_circle_is_typed() {
        let error =
            offset_curve_chain(&[circle(0, 1.0)], 1.0, OffsetOptions::default()).unwrap_err();
        assert_eq!(error, OffsetError::Collapse { span: 0 });
    }

    #[test]
    fn coincident_material_boundaries_report_topology_change() {
        let arrangement = arrange_curve_spans(
            &[circle(0, 10.0), circle(1, 4.0)],
            ArrangementOptions::default(),
        )
        .unwrap();
        let annulus = arrangement
            .regions
            .iter()
            .find(|region| region.holes.len() == 1)
            .unwrap();
        assert!(matches!(
            offset_material_region(annulus, -3.0, OffsetOptions::default()),
            Err(OffsetError::TopologyChange { .. })
        ));
    }

    #[test]
    fn equal_join_candidates_are_typed_as_ambiguous() {
        let reference = Pnt2d::origin();
        let error = choose_join(
            vec![
                JoinCandidate {
                    first: 0.0,
                    second: 0.0,
                    point: Pnt2d::new(-1.0, 0.0),
                },
                JoinCandidate {
                    first: 1.0,
                    second: 1.0,
                    point: Pnt2d::new(1.0, 0.0),
                },
            ],
            reference,
            1.0e-9,
            2,
            3,
        )
        .unwrap_err();
        assert_eq!(
            error,
            OffsetError::Ambiguous {
                first: 2,
                second: 3,
                candidates: 2,
            }
        );
    }

    #[test]
    fn branch_and_gap_are_not_silently_healed() {
        let branch = vec![
            line(0, (-1.0, 0.0), (0.0, 0.0)),
            line(1, (0.0, 0.0), (1.0, 0.0)),
            line(2, (0.0, 0.0), (0.0, 1.0)),
        ];
        assert!(matches!(
            offset_curve_chain(&branch, 0.2, OffsetOptions::default()),
            Err(OffsetError::Branch { degree: 3, .. })
        ));

        let gap = vec![
            line(0, (0.0, 0.0), (1.0, 0.0)),
            line(1, (2.0, 0.0), (3.0, 0.0)),
        ];
        assert!(matches!(
            offset_curve_chain(&gap, 0.2, OffsetOptions::default()),
            Err(OffsetError::Gap { .. })
        ));
    }

    #[test]
    fn reversed_circle_uses_traversal_side() {
        let frame = Ax22d::new(Pnt2d::origin(), Dir2d::dx());
        let reversed = CurveSpan::new(
            GeomCurve2d::circle(Circle2d::new(frame, 5.0)),
            TAU,
            0.0,
            9_u32,
        );
        let offset = offset_curve_chain(&[reversed], 1.0, OffsetOptions::default()).unwrap();
        let GeomCurve2d::Circle(circle) = &offset.spans[0].curve else {
            unreachable!()
        };
        assert!((circle.radius() - 6.0).abs() < 1.0e-12);
    }
}
