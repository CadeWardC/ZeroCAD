//! Datum resolution: turn [`FeatureType::DatumPlane`]/[`DatumAxis`]/[`DatumPoint`]
//! nodes into concrete frames/axes/points.
//!
//! Datums resolve in a pure pre-pass before the body loop — every input is a
//! base plane, another datum, an explicit point, or an expression over the
//! document variables, never live body geometry. That keeps the eval prefix
//! cache sound with one coarse rule: every datum feature is folded into the
//! cache's hash *seed* (like the variables), so any datum edit invalidates all
//! checkpoints instead of tracking per-consumer dependencies.

use super::*;
use crate::geometry::{CoordinateSystem, Vec3};

fn v3(p: [f32; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Rotate `v` around unit axis `k` by `angle` radians (Rodrigues).
fn rotate(v: Vec3, k: Vec3, angle: f32) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v.mul(c)
        .add(k.cross(v).mul(s))
        .add(k.mul(k.dot(v) * (1.0 - c)))
}

fn resolve_plane_base(
    base: &PlaneBase,
    resolved: &HashMap<String, DatumValue>,
) -> Option<CoordinateSystem> {
    match base {
        PlaneBase::XY => Some(CoordinateSystem::XY),
        PlaneBase::XZ => Some(CoordinateSystem::XZ),
        PlaneBase::YZ => Some(CoordinateSystem::YZ),
        PlaneBase::Datum(id) => match resolved.get(id) {
            Some(DatumValue::Plane(cs)) => Some(*cs),
            _ => None,
        },
    }
}

fn resolve_axis_base(
    axis: &AxisBase,
    resolved: &HashMap<String, DatumValue>,
) -> Option<(Vec3, Vec3)> {
    match axis {
        AxisBase::X => Some((Vec3::ZERO, Vec3::X)),
        AxisBase::Y => Some((Vec3::ZERO, Vec3::Y)),
        AxisBase::Z => Some((Vec3::ZERO, Vec3::Z)),
        AxisBase::Datum(id) => match resolved.get(id) {
            Some(DatumValue::Axis { origin, dir }) => Some((*origin, *dir)),
            _ => None,
        },
        AxisBase::TwoPoints { a, b } => {
            let dir = v3(*b).sub(v3(*a)).normalize();
            (dir != Vec3::ZERO).then_some((v3(*a), dir))
        }
    }
}

/// A dimension that may be expression-driven: resolve `expr` against `vars`,
/// falling back to the stored literal (and pushing a warning) when it no
/// longer evaluates — the same fail-loud rule as extrude depth.
fn resolve_dim(
    literal: f32,
    expr: Option<&String>,
    vars: &HashMap<String, f64>,
    node_id: &str,
    what: &str,
    warnings: &mut Vec<String>,
) -> f32 {
    match expr {
        Some(e) => match crate::expr::eval(e, vars) {
            Ok(v) => v as f32,
            Err(_) => {
                warnings.push(format!(
                    "Datum '{node_id}': {what} expression \"{e}\" no longer evaluates; \
                     using last value {literal:.3}."
                ));
                literal
            }
        },
        None => literal,
    }
}

impl ParametricGraph {
    /// Resolve every datum node into a [`DatumValue`], in creation order (so a
    /// datum may reference any earlier datum). A datum whose inputs are missing
    /// or degenerate resolves to nothing and pushes a warning — consumers then
    /// report Unresolved rather than silently landing on a wrong plane.
    pub fn resolve_datums(
        &self,
        vars: &HashMap<String, f64>,
        warnings: &mut Vec<String>,
    ) -> HashMap<String, DatumValue> {
        let mut nodes: Vec<_> = self
            .graph
            .node_indices()
            .filter(|&i| {
                matches!(
                    self.graph[i].feature,
                    FeatureType::DatumPlane { .. }
                        | FeatureType::DatumAxis { .. }
                        | FeatureType::DatumPoint { .. }
                )
            })
            .collect();
        nodes.sort_by_key(|&i| creation_key(&self.graph[i].id));

        let mut resolved: HashMap<String, DatumValue> = HashMap::new();
        for idx in nodes {
            let node = &self.graph[idx];
            let value = match &node.feature {
                FeatureType::DatumPlane { def } => {
                    self.resolve_plane_def(def, &resolved, vars, &node.id, warnings)
                        .map(DatumValue::Plane)
                }
                FeatureType::DatumAxis { def } => match def {
                    DatumAxisDef::TwoPoints { a, b } => {
                        let dir = v3(*b).sub(v3(*a)).normalize();
                        (dir != Vec3::ZERO).then_some(DatumValue::Axis {
                            origin: v3(*a),
                            dir,
                        })
                    }
                    DatumAxisDef::PlaneIntersection { a, b } => {
                        let ca = resolve_plane_base(a, &resolved);
                        let cb = resolve_plane_base(b, &resolved);
                        match (ca, cb) {
                            (Some(ca), Some(cb)) => plane_intersection_axis(&ca, &cb)
                                .map(|(origin, dir)| DatumValue::Axis { origin, dir }),
                            _ => None,
                        }
                    }
                },
                FeatureType::DatumPoint { def } => match def {
                    DatumPointDef::Coords { p } => Some(DatumValue::Point(v3(*p))),
                },
                _ => unreachable!(),
            };
            match value {
                Some(v) => {
                    resolved.insert(node.id.clone(), v);
                }
                None => warnings.push(format!(
                    "Datum '{}' ({}) could not be resolved from its inputs.",
                    node.id, node.name
                )),
            }
        }
        resolved
    }

    fn resolve_plane_def(
        &self,
        def: &DatumPlaneDef,
        resolved: &HashMap<String, DatumValue>,
        vars: &HashMap<String, f64>,
        node_id: &str,
        warnings: &mut Vec<String>,
    ) -> Option<CoordinateSystem> {
        match def {
            DatumPlaneDef::Offset {
                base,
                distance,
                distance_expr,
            } => {
                let cs = resolve_plane_base(base, resolved)?;
                let d = resolve_dim(
                    *distance,
                    distance_expr.as_ref(),
                    vars,
                    node_id,
                    "distance",
                    warnings,
                );
                // Offset along the STORED normal (with_origin keeps the axes):
                // the ground plane XZ is left-handed, and following a recomputed
                // u×v there would offset "up" downward.
                Some(cs.with_origin(cs.origin.add(cs.n.mul(d))))
            }
            DatumPlaneDef::Angle {
                base,
                axis,
                angle_deg,
                angle_expr,
            } => {
                let cs = resolve_plane_base(base, resolved)?;
                let (axis_origin, axis_dir) = resolve_axis_base(axis, resolved)?;
                let deg = resolve_dim(
                    *angle_deg,
                    angle_expr.as_ref(),
                    vars,
                    node_id,
                    "angle",
                    warnings,
                );
                let rad = deg.to_radians();
                // Rotate the frame's axes, and orbit its origin around the axis
                // line. Handedness is preserved: all three stored axes rotate
                // rigidly, none is recomputed.
                let rel = cs.origin.sub(axis_origin);
                Some(CoordinateSystem {
                    origin: axis_origin.add(rotate(rel, axis_dir, rad)),
                    u: rotate(cs.u, axis_dir, rad),
                    v: rotate(cs.v, axis_dir, rad),
                    n: rotate(cs.n, axis_dir, rad),
                })
            }
            DatumPlaneDef::ThreePoints { a, b, c } => {
                let (a, b, c) = (v3(*a), v3(*b), v3(*c));
                let u = b.sub(a).normalize();
                if u == Vec3::ZERO {
                    return None;
                }
                let w = c.sub(a);
                let v = w.sub(u.mul(w.dot(u))).normalize();
                if v == Vec3::ZERO {
                    return None;
                }
                Some(CoordinateSystem::new(a, u, v))
            }
            DatumPlaneDef::MidPlane { a, b } => {
                let ca = resolve_plane_base(a, resolved)?;
                let cb = resolve_plane_base(b, resolved)?;
                // Midway along a's normal between the two origins; a's axes.
                let mid = ca.origin.add(cb.origin).mul(0.5);
                Some(ca.with_origin(mid))
            }
        }
    }
}

/// Resolve an [`AxisBase`] against already-resolved datums to a world-space
/// `(origin, unit direction)`. Shared by revolve axes and pattern directions.
pub(crate) fn resolve_axis_base_world(
    axis: &AxisBase,
    datums: &HashMap<String, DatumValue>,
) -> Option<(Vec3, Vec3)> {
    resolve_axis_base(axis, datums)
}

/// Resolve a [`PlaneBase`] against already-resolved datums to a world frame.
pub(crate) fn resolve_plane_base_world(
    plane: &PlaneBase,
    datums: &HashMap<String, DatumValue>,
) -> Option<CoordinateSystem> {
    resolve_plane_base(plane, datums)
}

/// The intersection line of two planes, or `None` when (near-)parallel.
fn plane_intersection_axis(a: &CoordinateSystem, b: &CoordinateSystem) -> Option<(Vec3, Vec3)> {
    let dir = a.n.cross(b.n).normalize();
    if dir == Vec3::ZERO {
        return None;
    }
    // A point on both planes: solve within the span of the two normals.
    // p = c1*n1 + c2*n2 with n1·p = d1, n2·p = d2.
    let d1 = a.n.dot(a.origin);
    let d2 = b.n.dot(b.origin);
    let n1n2 = a.n.dot(b.n);
    let det = 1.0 - n1n2 * n1n2;
    if det.abs() < 1e-12 {
        return None;
    }
    let c1 = (d1 - d2 * n1n2) / det;
    let c2 = (d2 - d1 * n1n2) / det;
    Some((a.n.mul(c1).add(b.n.mul(c2)), dir))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane_of(g: &ParametricGraph, id: &str) -> CoordinateSystem {
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        assert!(warnings.is_empty(), "warnings: {warnings:?}");
        match &datums[id] {
            DatumValue::Plane(cs) => *cs,
            other => panic!("expected plane, got {other:?}"),
        }
    }

    fn add_plane(g: &mut ParametricGraph, id: &str, def: DatumPlaneDef) {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::DatumPlane { def },
        });
    }

    #[test]
    fn offset_follows_stored_normal_even_left_handed() {
        // XZ (ground) is LEFT-handed: stored n = +Y, u×v = −Y. Offsetting +5
        // must move UP (+Y), not follow a recomputed u×v.
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::XZ,
                distance: 5.0,
                distance_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_1");
        assert_eq!(cs.origin, Vec3::new(0.0, 5.0, 0.0));
        assert_eq!(cs.n, Vec3::Y);
        assert_eq!(cs.u, CoordinateSystem::XZ.u);
        assert_eq!(cs.v, CoordinateSystem::XZ.v);
    }

    #[test]
    fn datum_chain_resolves_in_creation_order() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 2.0,
                distance_expr: None,
            },
        );
        add_plane(
            &mut g,
            "datum_2",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("datum_1".to_string()),
                distance: 3.0,
                distance_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_2");
        assert_eq!(cs.origin, Vec3::new(0.0, 0.0, 5.0));
    }

    #[test]
    fn angle_plane_rotates_rigidly() {
        // XY rotated 90° about the X axis: n goes +Z → −Y (right-hand rule),
        // u stays +X.
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Angle {
                base: PlaneBase::XY,
                axis: AxisBase::X,
                angle_deg: 90.0,
                angle_expr: None,
            },
        );
        let cs = plane_of(&g, "datum_1");
        let close = |a: Vec3, b: Vec3| a.sub(b).length() < 1e-5;
        assert!(close(cs.u, Vec3::X), "u = {:?}", cs.u);
        assert!(close(cs.v, Vec3::Z), "v = {:?}", cs.v);
        assert!(close(cs.n, Vec3::new(0.0, -1.0, 0.0)), "n = {:?}", cs.n);
    }

    #[test]
    fn three_point_plane_and_midplane() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::ThreePoints {
                a: [0.0, 0.0, 1.0],
                b: [1.0, 0.0, 1.0],
                c: [0.0, 1.0, 1.0],
            },
        );
        add_plane(
            &mut g,
            "datum_2",
            DatumPlaneDef::MidPlane {
                a: PlaneBase::XY,
                b: PlaneBase::Datum("datum_1".to_string()),
            },
        );
        let p1 = plane_of(&g, "datum_1");
        assert_eq!(p1.n, Vec3::Z);
        let p2 = plane_of(&g, "datum_2");
        assert_eq!(p2.origin, Vec3::new(0.0, 0.0, 0.5));
    }

    #[test]
    fn axis_from_plane_intersection() {
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "datumaxis_1".to_string(),
            name: "axis".to_string(),
            feature: FeatureType::DatumAxis {
                def: DatumAxisDef::PlaneIntersection {
                    a: PlaneBase::XY,
                    b: PlaneBase::XZ,
                },
            },
        });
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        let DatumValue::Axis { origin, dir } = &datums["datumaxis_1"] else {
            panic!("expected axis");
        };
        // XY (n=+Z) ∩ XZ (n=+Y) = the X axis.
        assert!(dir.y.abs() < 1e-6 && dir.z.abs() < 1e-6 && dir.x.abs() > 0.99);
        assert!(origin.y.abs() < 1e-6 && origin.z.abs() < 1e-6);
    }

    #[test]
    fn unresolvable_datum_warns() {
        let mut g = ParametricGraph::new();
        add_plane(
            &mut g,
            "datum_1",
            DatumPlaneDef::Offset {
                base: PlaneBase::Datum("nonexistent".to_string()),
                distance: 1.0,
                distance_expr: None,
            },
        );
        let mut warnings = Vec::new();
        let datums = g.resolve_datums(&HashMap::new(), &mut warnings);
        assert!(datums.is_empty());
        assert_eq!(warnings.len(), 1);
    }
}
