//! Resolve picked dimensions to existing drivers without adding conflicting equations.
use super::{Constraint, EntityId, SketchEntity, SketchSolverModel};

/// Return the existing driving constraint for a picked measurement. Radius
/// drivers are converted to diameter when a circle is picked. Coordinate
/// equivalence follows actual H/V constraints, never merely nearby geometry.
pub fn existing_driver(model: &SketchSolverModel, picked: &Constraint) -> Option<Constraint> {
    let lines: std::collections::HashMap<_, _> = model
        .entities
        .iter()
        .filter_map(|e| match e {
            SketchEntity::Line { id, p0, p1, .. } => Some((*id, (*p0, *p1))),
            _ => None,
        })
        .collect();
    let line = |id| lines.get(&id).copied();
    // Build coordinate components once per pick. Each subsequent equivalence
    // check is a lookup, even in a sketch with many driving dimensions.
    let classes = [false, true].map(|horizontal| {
        let mut edges = std::collections::HashMap::<EntityId, Vec<EntityId>>::new();
        for c in &model.constraints {
            let edge = match c {
                Constraint::Horizontal { line: id, .. } if horizontal => line(*id),
                Constraint::Vertical { line: id, .. } if !horizontal => line(*id),
                Constraint::Coincident { a, b, .. } => Some((*a, *b)),
                _ => None,
            };
            if let Some((a, b)) = edge {
                edges.entry(a).or_default().push(b);
                edges.entry(b).or_default().push(a);
            }
        }
        let mut labels = std::collections::HashMap::new();
        for point in &model.points {
            if labels.contains_key(&point.id) {
                continue;
            }
            let mut pending = vec![point.id];
            labels.insert(point.id, point.id);
            while let Some(id) = pending.pop() {
                for &next in edges.get(&id).into_iter().flatten() {
                    if let std::collections::hash_map::Entry::Vacant(entry) = labels.entry(next) {
                        entry.insert(point.id);
                        pending.push(next);
                    }
                }
            }
        }
        labels
    });
    let connected = |a, b, horizontal: bool| {
        a == b
            || classes[usize::from(horizontal)]
                .get(&a)
                .is_some_and(|label| classes[usize::from(horizontal)].get(&b) == Some(label))
    };
    let same_span = |a: EntityId, b: EntityId, u: EntityId, v: EntityId| {
        (a == u && b == v)
            || (a == v && b == u)
            || [true, false].into_iter().any(|horizontal| {
                connected(a, b, horizontal)
                    && connected(u, v, horizontal)
                    && ((connected(a, u, !horizontal) && connected(b, v, !horizontal))
                        || (connected(a, v, !horizontal) && connected(b, u, !horizontal)))
            })
    };
    for current in &model.constraints {
        if model.is_driven_dimension(current.id()) {
            continue;
        }
        match (picked, current) {
            (
                Constraint::Diameter { circle, .. },
                Constraint::Radius {
                    id,
                    circle: other,
                    r,
                },
            ) if circle == other => {
                let mut d = r.clone();
                d.value *= 2.0;
                d.expr = d.expr.map(|e| format!("2*({e})"));
                return Some(Constraint::Diameter {
                    id: *id,
                    circle: *circle,
                    d,
                });
            }
            (Constraint::Diameter { circle: a, .. }, Constraint::Diameter { circle: b, .. })
            | (Constraint::Radius { circle: a, .. }, Constraint::Radius { circle: b, .. })
                if a == b =>
            {
                return Some(current.clone())
            }
            (Constraint::Distance { a, b, .. }, Constraint::Distance { a: u, b: v, .. })
                if same_span(*a, *b, *u, *v) =>
            {
                return Some(current.clone())
            }
            (
                Constraint::LineDistance { a, b, .. },
                Constraint::LineDistance { a: u, b: v, .. },
            )
            | (Constraint::Angle { a, b, .. }, Constraint::Angle { a: u, b: v, .. })
                if (a == u && b == v) || (a == v && b == u) =>
            {
                return Some(current.clone())
            }
            (Constraint::LineDistance { a, b, .. }, Constraint::Distance { a: u, b: v, .. }) => {
                if let (Some((p, q)), Some((r, s))) = (line(*a), line(*b)) {
                    if [true, false].into_iter().any(|horizontal| {
                        connected(p, q, horizontal)
                            && connected(r, s, horizontal)
                            && connected(*u, *v, !horizontal)
                            && ((connected(p, *u, horizontal) && connected(r, *v, horizontal))
                                || (connected(p, *v, horizontal) && connected(r, *u, horizontal)))
                    }) {
                        return Some(current.clone());
                    }
                }
            }
            _ => {}
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::{constraints::promote_shapes_to_entities, Dimension, SketchShape};

    #[test]
    fn circle_driver_keeps_identity_and_expression_in_diameter_units() {
        let shapes = [SketchShape::Circle {
            center: (0.0, 0.0),
            diameter: Dimension {
                value: 10.0,
                expr: Some("size".into()),
            },
        }];
        let vars = [("size".into(), 10.0)].into_iter().collect();
        let (model, _) = promote_shapes_to_entities(&shapes, &[EntityId(0)], &vars, 1);
        let picked = Constraint::Diameter {
            id: EntityId(100),
            circle: model.entities[0].id(),
            d: Dimension::literal(10.0),
        };
        let resolved = existing_driver(&model, &picked).unwrap();
        assert_eq!(resolved.id(), model.constraints[0].id());
        let Constraint::Diameter { d, .. } = resolved else {
            panic!("expected diameter")
        };
        assert_eq!(d.resolve(&vars), 10.0);
        let changed_vars = [("size".into(), 20.0)].into_iter().collect();
        assert_eq!(d.resolve(&changed_vars), 20.0);
    }

    #[test]
    fn coincident_geometry_without_constraints_does_not_share_a_driver() {
        let shape = SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(10.0),
            h: Dimension::literal(6.0),
            from_center: false,
        };
        let (model, _) = promote_shapes_to_entities(
            &[shape.clone(), shape],
            &[EntityId(0), EntityId(1)],
            &Default::default(),
            2,
        );
        let SketchEntity::Line { p0, p1, .. } = model.entities[4] else {
            panic!("expected line")
        };
        let picked = Constraint::Distance {
            id: EntityId(100),
            a: p0,
            b: p1,
            d: Dimension::literal(10.0),
        };
        let resolved = existing_driver(&model, &picked).unwrap();
        assert!(matches!(resolved, Constraint::Distance { a, b, .. } if a == p0 && b == p1));
    }
}
