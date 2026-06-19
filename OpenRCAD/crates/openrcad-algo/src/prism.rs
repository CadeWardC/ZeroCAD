//! Prism/extrusion sweeping for B-Rep faces.
//!
//! This is the OpenRCAD equivalent of the practical core of OCCT's
//! `BRepSweep_Prism`: duplicate the swept face for the far cap, generate one
//! lateral face per boundary edge, then sew the result into a watertight solid.

use core::fmt;

use openrcad_foundation::{tolerance, Ax3, Dir, Pnt, Trsf, Vec as GeomVec};
use openrcad_geom::{Curve, CylindricalSurface, GeomCurve, GeomSurface, Line, Plane, RuledSurface};
use openrcad_topo::{Edge, Face, Orientation, Solid, Wire};

use crate::sew::sew;

/// Errors reported by prism/extrusion sweeping.
#[derive(Clone, Debug, PartialEq)]
pub enum SweepError {
    /// The sweep vector has no usable length.
    DegenerateVector,
    /// The source face has no outer boundary.
    MissingOuterWire,
    /// A boundary wire is not closed.
    OpenWire,
}

impl fmt::Display for SweepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegenerateVector => f.write_str("prism: sweep vector must be non-zero"),
            Self::MissingOuterWire => f.write_str("prism: source face has no outer wire"),
            Self::OpenWire => f.write_str("prism: every swept wire must be closed"),
        }
    }
}

impl std::error::Error for SweepError {}

/// Sweep `face` by `vector` and return a sewn solid.
///
/// Straight boundary edges generate planar lateral faces. Circular arcs whose
/// plane normal is parallel to the sweep vector generate cylindrical faces.
/// Other curves generate ruled lateral faces between the base and translated
/// edge, which covers NURBS/B-spline boundaries and skew circular sweeps.
pub fn prism(face: &Face, vector: GeomVec) -> Result<Solid, SweepError> {
    if vector.magnitude() <= tolerance::CONFUSION {
        return Err(SweepError::DegenerateVector);
    }
    let Some(outer) = face.outer_wire() else {
        return Err(SweepError::MissingOuterWire);
    };
    if !outer.is_closed() {
        return Err(SweepError::OpenWire);
    }
    for wire in face.inner_wires() {
        if !wire.is_closed() {
            return Err(SweepError::OpenWire);
        }
    }

    let translation = Trsf::translation(vector);
    let mut faces = Vec::new();
    let face_normal = effective_face_normal(face);
    let sweep_points_along_normal = face_normal
        .map(|n| GeomVec::from_dir(n).dot(&vector) >= 0.0)
        .unwrap_or(true);

    faces.extend(cap_faces(face, &translation, sweep_points_along_normal));

    for wire in face.wires() {
        for edge in wire.edges() {
            if edge.length() <= tolerance::CONFUSION {
                continue;
            }
            faces.push(lateral_face(&edge, &translation, vector));
        }
    }

    Ok(Solid::new(sew(&faces, tolerance::CONFUSION * 10.0)))
}

/// Alias matching the OpenCASCADE class name in user-facing docs.
#[inline]
pub fn sweep_prism(face: &Face, vector: GeomVec) -> Result<Solid, SweepError> {
    prism(face, vector)
}

fn lateral_face(edge: &Edge, translation: &Trsf, vector: GeomVec) -> Face {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let q0 = translation.transform_point(&p0);
    let q1 = translation.transform_point(&p1);

    let top_edge = edge.transformed(translation);
    let wire = Wire::from_edges([
        edge.reversed(),
        Edge::between_points(p0, q0),
        top_edge.clone(),
        Edge::between_points(q1, p1),
    ]);

    Face::new(
        Some(lateral_surface(edge, p1, p0, vector, translation)),
        wire,
    )
}

fn cap_faces(face: &Face, translation: &Trsf, sweep_points_along_normal: bool) -> [Face; 2] {
    let top = face.transformed(translation);
    let Some(GeomSurface::Plane(_)) = face.surface() else {
        return if sweep_points_along_normal {
            [face.reversed(), top]
        } else {
            [face.clone(), top.reversed()]
        };
    };

    let Some(normal) = effective_face_normal(face) else {
        return if sweep_points_along_normal {
            [face.reversed(), top]
        } else {
            [face.clone(), top.reversed()]
        };
    };

    if sweep_points_along_normal {
        [
            planar_cap(face, normal.reversed(), None),
            planar_cap(face, normal, Some(translation)),
        ]
    } else {
        [
            planar_cap(face, normal, None),
            planar_cap(face, normal.reversed(), Some(translation)),
        ]
    }
}

fn planar_cap(face: &Face, normal: Dir, translation: Option<&Trsf>) -> Face {
    let transform_wire = |wire: Wire| match translation {
        Some(t) => wire.transformed(t),
        None => wire,
    };

    let outer = face.outer_wire().map(transform_wire);
    let inners = face
        .inner_wires()
        .into_iter()
        .map(transform_wire)
        .collect::<Vec<_>>();
    let point = outer
        .as_ref()
        .and_then(|wire| wire.edges().first().map(|edge| edge.source().point()))
        .unwrap_or(Pnt::origin());

    Face::with_wires(
        Some(GeomSurface::plane(Plane::from_point_normal(point, normal))),
        outer,
        inners,
        Orientation::Forward,
    )
}

fn lateral_surface(
    edge: &Edge,
    p0: Pnt,
    p1: Pnt,
    vector: GeomVec,
    translation: &Trsf,
) -> GeomSurface {
    if let Some(curve) = edge.curve() {
        match curve {
            GeomCurve::Line(_) => {
                if let Some(plane) = plane_for_sweep(p0, p1, vector) {
                    return GeomSurface::plane(plane);
                }
            }
            GeomCurve::Circle(circle) => {
                if let Some(axis) = vector.normalized() {
                    if circle.axis().is_parallel(&axis, 1e-8) {
                        return GeomSurface::cylinder(CylindricalSurface::new(
                            Ax3::new_axes(circle.center(), axis, circle.position().x_direction()),
                            circle.radius(),
                        ));
                    }
                }
            }
            _ => {}
        }

        let top_curve = curve.transformed(translation);
        return GeomSurface::ruled(RuledSurface::new(curve.clone(), top_curve));
    }

    if let Some(plane) = plane_for_sweep(p0, p1, vector) {
        return GeomSurface::plane(plane);
    }

    let base = line_curve(p0, p1);
    let top = base.transformed(translation);
    GeomSurface::ruled(RuledSurface::new(base, top))
}

fn plane_for_sweep(p0: Pnt, p1: Pnt, vector: GeomVec) -> Option<Plane> {
    let tangent = p1 - p0;
    let normal = tangent.cross(&vector).normalized()?;
    Some(Plane::from_point_normal(p0, normal))
}

fn line_curve(p0: Pnt, p1: Pnt) -> GeomCurve {
    let tangent = p1 - p0;
    let dir = tangent.normalized().unwrap_or(Dir::dx());
    GeomCurve::line(Line::from_point_dir(p0, dir))
}

fn effective_face_normal(face: &Face) -> Option<Dir> {
    let mut normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        _ => newell_normal(face)?,
    };
    if face.orientation() == Orientation::Reversed {
        normal = normal.reversed();
    }
    Some(normal)
}

fn newell_normal(face: &Face) -> Option<Dir> {
    let wire = face.outer_wire()?;
    let edges = wire.edges();
    if edges.len() < 3 {
        return None;
    }

    let points: Vec<Pnt> = edges.iter().map(|edge| edge.source().point()).collect();
    let mut n = GeomVec::ZERO;
    for i in 0..points.len() {
        let p = points[i];
        let q = points[(i + 1) % points.len()];
        n += GeomVec::new(
            (p.y() - q.y()) * (p.z() + q.z()),
            (p.z() - q.z()) * (p.x() + q.x()),
            (p.x() - q.x()) * (p.y() + q.y()),
        );
    }
    n.normalized()
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::TAU;
    use openrcad_foundation::Ax3;
    use openrcad_geom::Circle;
    use openrcad_topo::Vertex;

    fn square_face_with_hole() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(4.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let inner = Wire::from_edges([
            Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(1.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 3.0, 0.0), Pnt::new(3.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 3.0, 0.0), Pnt::new(3.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 1.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
        ]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![inner],
            Orientation::Forward,
        )
    }

    #[test]
    fn extrudes_triangle_to_watertight_prism() {
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(2.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(2.0, 0.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
            ]),
        );

        let solid = prism(&face, GeomVec::new(0.0, 0.0, 3.0)).unwrap();
        assert_eq!(solid.vertex_count(), 6);
        assert_eq!(solid.edge_count(), 9);
        assert_eq!(solid.face_count(), 5);
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
    }

    #[test]
    fn extrudes_face_with_hole() {
        let solid = prism(&square_face_with_hole(), GeomVec::new(0.0, 0.0, 2.0)).unwrap();
        assert_eq!(solid.face_count(), 10);
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
    }

    #[test]
    fn circular_boundary_generates_cylindrical_laterals() {
        let circle = Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        let edges: Vec<Edge> = [0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU]
            .windows(2)
            .map(|w| {
                Edge::new(
                    Some(GeomCurve::circle(circle)),
                    w[0],
                    w[1],
                    Vertex::new(circle.point(w[0])),
                    Vertex::new(circle.point(w[1])),
                )
            })
            .collect();
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges(edges),
        );

        let solid = prism(&face, GeomVec::new(0.0, 0.0, 5.0)).unwrap();
        assert_eq!(solid.vertex_count(), 6);
        assert_eq!(solid.edge_count(), 9);
        assert_eq!(solid.face_count(), 5);
        let cylinders = solid
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 3);
        assert!(solid.is_watertight());
    }
}
