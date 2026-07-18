//! Associative body-edge projection into sketch construction geometry.
//!
//! Lines and circular edges parallel to the sketch plane stay analytic. A
//! circle viewed obliquely becomes one associative analytic ellipse entity;
//! faceting happens only at the final `SketchCurves` compatibility boundary.

use super::{EntityId, ProjectedEdgeReference, SketchEntity, SketchPoint, SketchSolverModel};
use crate::geometry::{CoordinateSystem, Vec3};
use crate::mock_kernel::EdgeCurveHint;
use crate::parametric::EdgeRef;

#[derive(Clone, Copy)]
enum Primitive {
    Line((f32, f32), (f32, f32)),
    Circle((f32, f32), f32),
    Arc {
        center: (f32, f32),
        start: (f32, f32),
        end: (f32, f32),
        radius: f32,
    },
    Ellipse {
        center: (f32, f32),
        major_axis: [f32; 2],
        minor_axis: [f32; 2],
        start: f32,
        end: f32,
        closed: bool,
    },
}

fn alloc(next: &mut u32) -> EntityId {
    let id = EntityId(*next);
    *next = next.saturating_add(1);
    id
}

fn world_point(values: [f32; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

fn circle_point(center: Vec3, x_dir: Vec3, y_dir: Vec3, radius: f32, parameter: f32) -> Vec3 {
    center
        .add(x_dir.mul(radius * parameter.cos()))
        .add(y_dir.mul(radius * parameter.sin()))
}

fn projected_primitives(source: &EdgeRef, cs: CoordinateSystem) -> Result<Vec<Primitive>, String> {
    match &source.curve {
        Some(EdgeCurveHint::Circle {
            center,
            axis,
            x_dir,
            radius,
            start,
            end,
            closed,
        }) if radius.is_finite() && *radius > 0.0 => {
            let center = world_point(*center);
            let axis = world_point(*axis).normalize();
            let x_dir = world_point(*x_dir).normalize();
            let y_dir = axis.cross(x_dir).normalize();
            if axis == Vec3::ZERO || x_dir == Vec3::ZERO || y_dir == Vec3::ZERO {
                return Err("selected circular edge has a degenerate frame".to_string());
            }
            let center_2d = cs.project(center);
            let coplanar = axis.dot(cs.n).abs() >= 1.0 - 1.0e-4;
            if coplanar && *closed {
                let rim = cs.project(circle_point(center, x_dir, y_dir, *radius, 0.0));
                let projected_radius = (rim.0 - center_2d.0).hypot(rim.1 - center_2d.1);
                return (projected_radius > 1.0e-6)
                    .then_some(vec![Primitive::Circle(center_2d, projected_radius)])
                    .ok_or_else(|| "selected circle collapses in the sketch plane".to_string());
            }
            let sweep = if *closed {
                std::f32::consts::TAU
            } else {
                end - start
            };
            if coplanar && !closed && sweep.abs() <= std::f32::consts::PI + 1.0e-5 {
                let first = cs.project(circle_point(center, x_dir, y_dir, *radius, *start));
                let last = cs.project(circle_point(center, x_dir, y_dir, *radius, *end));
                let projected_radius = (first.0 - center_2d.0).hypot(first.1 - center_2d.1);
                return Ok(vec![Primitive::Arc {
                    center: center_2d,
                    start: first,
                    end: last,
                    radius: projected_radius,
                }]);
            }

            let x = cs.project(center.add(x_dir.mul(*radius)));
            let y = cs.project(center.add(y_dir.mul(*radius)));
            let major_axis = [x.0 - center_2d.0, x.1 - center_2d.1];
            let minor_axis = [y.0 - center_2d.0, y.1 - center_2d.1];
            let determinant = major_axis[0] * minor_axis[1] - major_axis[1] * minor_axis[0];
            if determinant.abs() <= 1.0e-7 {
                return Err("selected circle collapses to a line in the sketch plane".to_string());
            }
            Ok(vec![Primitive::Ellipse {
                center: center_2d,
                major_axis,
                minor_axis,
                start: *start,
                end: *end,
                closed: *closed,
            }])
        }
        _ => {
            let first = cs.project(world_point(source.p0));
            let last = cs.project(world_point(source.p1));
            if (last.0 - first.0).hypot(last.1 - first.1) <= 1.0e-6 {
                Err("selected edge collapses in the sketch plane".to_string())
            } else {
                Ok(vec![Primitive::Line(first, last)])
            }
        }
    }
}

fn build_projection(
    model: &mut SketchSolverModel,
    source_body: String,
    source: EdgeRef,
    cs: CoordinateSystem,
    next: &mut u32,
) -> Result<ProjectedEdgeReference, String> {
    let primitives = projected_primitives(&source, cs)?;
    let mut entity_ids = Vec::with_capacity(primitives.len());
    let mut point_ids = Vec::new();
    for primitive in primitives {
        match primitive {
            Primitive::Line(a, b) => {
                let p0 = alloc(next);
                let p1 = alloc(next);
                let id = alloc(next);
                model.points.extend([
                    SketchPoint {
                        id: p0,
                        pos: (f64::from(a.0), f64::from(a.1)),
                    },
                    SketchPoint {
                        id: p1,
                        pos: (f64::from(b.0), f64::from(b.1)),
                    },
                ]);
                model.entities.push(SketchEntity::Line {
                    id,
                    p0,
                    p1,
                    derived_from: None,
                });
                point_ids.extend([p0, p1]);
                entity_ids.push(id);
            }
            Primitive::Circle(center, radius) => {
                let point = alloc(next);
                let id = alloc(next);
                model.points.push(SketchPoint {
                    id: point,
                    pos: (f64::from(center.0), f64::from(center.1)),
                });
                model.entities.push(SketchEntity::Circle {
                    id,
                    center: point,
                    radius: f64::from(radius),
                    derived_from: None,
                });
                point_ids.push(point);
                entity_ids.push(id);
            }
            Primitive::Arc {
                center,
                start,
                end,
                radius,
            } => {
                let center_id = alloc(next);
                let start_id = alloc(next);
                let end_id = alloc(next);
                let id = alloc(next);
                model.points.extend([
                    SketchPoint {
                        id: center_id,
                        pos: (f64::from(center.0), f64::from(center.1)),
                    },
                    SketchPoint {
                        id: start_id,
                        pos: (f64::from(start.0), f64::from(start.1)),
                    },
                    SketchPoint {
                        id: end_id,
                        pos: (f64::from(end.0), f64::from(end.1)),
                    },
                ]);
                model.entities.push(SketchEntity::Arc {
                    id,
                    center: center_id,
                    start: start_id,
                    end: end_id,
                    radius: f64::from(radius),
                    derived_from: None,
                });
                point_ids.extend([center_id, start_id, end_id]);
                entity_ids.push(id);
            }
            Primitive::Ellipse {
                center,
                major_axis,
                minor_axis,
                start,
                end,
                closed,
            } => {
                let center_id = alloc(next);
                let id = alloc(next);
                model.points.push(SketchPoint {
                    id: center_id,
                    pos: (f64::from(center.0), f64::from(center.1)),
                });
                model.entities.push(SketchEntity::Ellipse {
                    id,
                    center: center_id,
                    major_axis: major_axis.map(f64::from),
                    minor_axis: minor_axis.map(f64::from),
                    start_parameter: f64::from(start),
                    end_parameter: f64::from(end),
                    closed,
                    derived_from: None,
                });
                point_ids.push(center_id);
                entity_ids.push(id);
            }
        }
    }
    model.construction.extend(entity_ids.iter().copied());
    model.construction.sort();
    model.construction.dedup();
    Ok(ProjectedEdgeReference {
        source_body,
        source,
        entity_ids,
        point_ids,
    })
}

/// Add a new associative projection to a live or persisted solver model.
pub fn append_projected_edge(
    model: &mut SketchSolverModel,
    source_body: String,
    source: EdgeRef,
    cs: CoordinateSystem,
    next: &mut u32,
) -> Result<(), String> {
    let projection = build_projection(model, source_body, source, cs, next)?;
    model.projected_edges.push(projection);
    Ok(())
}

/// Refresh one projection after its named source edge has changed. Compatible
/// curve representations update in place so attached constraints retain their
/// ids. A line/circle/arc kind change fails without modifying the old reference.
pub fn rebuild_projected_edge(
    model: &mut SketchSolverModel,
    index: usize,
    source: EdgeRef,
    cs: CoordinateSystem,
) -> Result<(), String> {
    let old = model
        .projected_edges
        .get(index)
        .cloned()
        .ok_or_else(|| format!("projected edge {index} does not exist"))?;
    let primitives = projected_primitives(&source, cs)?;
    let compatible = primitives.len() == old.entity_ids.len()
        && primitives
            .iter()
            .zip(&old.entity_ids)
            .all(|(primitive, id)| {
                model
                    .entities
                    .iter()
                    .find(|entity| entity.id() == *id)
                    .is_some_and(|entity| {
                        matches!(
                            (primitive, entity),
                            (Primitive::Line(..), SketchEntity::Line { .. })
                                | (Primitive::Circle(..), SketchEntity::Circle { .. })
                                | (Primitive::Arc { .. }, SketchEntity::Arc { .. })
                                | (Primitive::Ellipse { .. }, SketchEntity::Ellipse { .. })
                        )
                    })
            });
    if compatible {
        let update_point = |model: &mut SketchSolverModel, id: EntityId, value: (f32, f32)| {
            if let Some(point) = model.points.iter_mut().find(|point| point.id == id) {
                point.pos = (f64::from(value.0), f64::from(value.1));
            }
        };
        for (primitive, id) in primitives.into_iter().zip(&old.entity_ids) {
            let entity = model
                .entities
                .iter()
                .find(|entity| entity.id() == *id)
                .cloned()
                .expect("compatibility preflight found the entity");
            match (primitive, entity) {
                (Primitive::Line(a, b), SketchEntity::Line { p0, p1, .. }) => {
                    update_point(model, p0, a);
                    update_point(model, p1, b);
                }
                (Primitive::Circle(center, radius), SketchEntity::Circle { center: p, .. }) => {
                    update_point(model, p, center);
                    if let Some(SketchEntity::Circle { radius: stored, .. }) =
                        model.entities.iter_mut().find(|entity| entity.id() == *id)
                    {
                        *stored = f64::from(radius);
                    }
                }
                (
                    Primitive::Arc {
                        center,
                        start,
                        end,
                        radius,
                    },
                    SketchEntity::Arc {
                        center: center_id,
                        start: start_id,
                        end: end_id,
                        ..
                    },
                ) => {
                    update_point(model, center_id, center);
                    update_point(model, start_id, start);
                    update_point(model, end_id, end);
                    if let Some(SketchEntity::Arc { radius: stored, .. }) =
                        model.entities.iter_mut().find(|entity| entity.id() == *id)
                    {
                        *stored = f64::from(radius);
                    }
                }
                (
                    Primitive::Ellipse {
                        center,
                        major_axis,
                        minor_axis,
                        start,
                        end,
                        closed,
                    },
                    SketchEntity::Ellipse {
                        center: center_id, ..
                    },
                ) => {
                    update_point(model, center_id, center);
                    if let Some(SketchEntity::Ellipse {
                        major_axis: stored_major,
                        minor_axis: stored_minor,
                        start_parameter,
                        end_parameter,
                        closed: stored_closed,
                        ..
                    }) = model.entities.iter_mut().find(|entity| entity.id() == *id)
                    {
                        *stored_major = major_axis.map(f64::from);
                        *stored_minor = minor_axis.map(f64::from);
                        *start_parameter = f64::from(start);
                        *end_parameter = f64::from(end);
                        *stored_closed = closed;
                    }
                }
                _ => unreachable!("projection compatibility was checked above"),
            }
        }
        model.projected_edges[index].source = source;
        return Ok(());
    }

    Err(
        "projected source changed curve representation; the existing reference and its constraints were preserved"
            .to_string(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::Constraint;

    #[test]
    fn coplanar_circle_projects_as_locked_analytic_construction() {
        let edge = EdgeRef {
            p0: [5.0, 0.0, 0.0],
            p1: [5.0, 0.0, 0.0],
            n1: [0.0; 3],
            n2: [0.0; 3],
            curve: Some(EdgeCurveHint::Circle {
                center: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 5.0,
                start: 0.0,
                end: std::f32::consts::TAU,
                closed: true,
            }),
            topology: None,
        };
        let mut model = SketchSolverModel::default();
        let mut next = 1;
        append_projected_edge(
            &mut model,
            "cylinder".to_string(),
            edge,
            CoordinateSystem::XY,
            &mut next,
        )
        .unwrap();
        assert!(matches!(model.entities[0], SketchEntity::Circle { .. }));
        assert_eq!(model.projected_edges.len(), 1);
        assert!(model.is_projected_entity(model.entities[0].id()));
        assert!(model.construction.contains(&model.entities[0].id()));
    }

    #[test]
    fn rebuilding_compatible_projection_preserves_ids_and_constraints() {
        let source = |end: f32| EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [end, 0.0, 0.0],
            n1: [0.0; 3],
            n2: [0.0; 3],
            curve: Some(EdgeCurveHint::Line),
            topology: None,
        };
        let mut model = SketchSolverModel::default();
        let mut next = 10;
        append_projected_edge(
            &mut model,
            "box".to_string(),
            source(5.0),
            CoordinateSystem::XY,
            &mut next,
        )
        .unwrap();
        let old_ids = model.projected_edges[0].entity_ids.clone();
        let points = model.projected_edges[0].point_ids.clone();
        let follower = EntityId(next);
        next += 1;
        model.points.push(SketchPoint {
            id: follower,
            pos: (5.0, 1.0),
        });
        model.constraints.push(Constraint::Coincident {
            id: EntityId(next),
            a: follower,
            b: points[1],
        });
        rebuild_projected_edge(&mut model, 0, source(12.0), CoordinateSystem::XY).unwrap();
        assert_eq!(model.projected_edges[0].entity_ids, old_ids);
        assert_eq!(model.constraints.len(), 1);
        let report = crate::sketch::solve_model(&model, &std::collections::HashMap::new());
        assert_eq!(report.outcome, crate::sketch::SolveOutcome::Converged);
        crate::sketch::solve::apply_solution(&mut model, &report);
        let projected = model.point(points[1]).unwrap().pos;
        assert!((projected.0 - 12.0).abs() < 1.0e-8);
        assert!(projected.1.abs() < 1.0e-8);
        assert!((model.point(follower).unwrap().pos.0 - 12.0).abs() < 1.0e-8);
        assert!(model.point(follower).unwrap().pos.1.abs() < 1.0e-8);
        assert_eq!(model.projected_edges[0].source.p1, [12.0, 0.0, 0.0]);
    }

    #[test]
    fn oblique_circle_projects_as_one_associative_analytic_ellipse() {
        let edge = EdgeRef {
            p0: [5.0, 0.0, 0.0],
            p1: [5.0, 0.0, 0.0],
            n1: [0.0; 3],
            n2: [0.0; 3],
            curve: Some(EdgeCurveHint::Circle {
                center: [0.0, 0.0, 0.0],
                axis: [
                    0.0,
                    std::f32::consts::FRAC_1_SQRT_2,
                    std::f32::consts::FRAC_1_SQRT_2,
                ],
                x_dir: [1.0, 0.0, 0.0],
                radius: 5.0,
                start: 0.0,
                end: std::f32::consts::TAU,
                closed: true,
            }),
            topology: None,
        };
        let mut model = SketchSolverModel::default();
        let mut next = 100;
        append_projected_edge(
            &mut model,
            "tilted-cylinder".to_string(),
            edge.clone(),
            CoordinateSystem::XY,
            &mut next,
        )
        .expect("analytic ellipse projection");

        assert_eq!(model.entities.len(), 1);
        assert!(matches!(model.entities[0], SketchEntity::Ellipse { .. }));
        assert_eq!(model.projected_edges[0].entity_ids.len(), 1);
        assert_eq!(model.projected_edges[0].point_ids.len(), 1);
        let old_id = model.entities[0].id();
        rebuild_projected_edge(&mut model, 0, edge, CoordinateSystem::XY)
            .expect("compatible ellipse refresh");
        assert_eq!(model.entities[0].id(), old_id);

        let construction = super::super::bake_construction_curves(&model);
        assert_eq!(construction.segments.len(), 64);
        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];
        for point in construction
            .segments
            .iter()
            .flat_map(|segment| [segment.a, segment.b])
        {
            min[0] = min[0].min(point.0);
            min[1] = min[1].min(point.1);
            max[0] = max[0].max(point.0);
            max[1] = max[1].max(point.1);
        }
        assert!((min[0] + 5.0).abs() < 1.0e-4 && (max[0] - 5.0).abs() < 1.0e-4);
        assert!(
            min[1] > -4.0 && max[1] < 4.0,
            "oblique minor axis must contract"
        );
    }
}
