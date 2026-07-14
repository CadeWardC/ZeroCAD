//! Shared construction helpers for the primitive builders.

use openrcad_foundation::{Ax22d, Dir, Dir2d, Pnt, Pnt2d, Vec as FVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_geom2d::{Circle2d, Ellipse2d, GeomCurve2d, Line2d};
use openrcad_topo::{Edge, Face, PcurveData, SurfacePeriodicity, Vertex, Wire};

/// A plane through `p` with normal `n`.
pub(crate) fn plane_at(p: Pnt, n: Dir) -> GeomSurface {
    GeomSurface::plane(Plane::from_point_normal(p, n))
}

/// Split a circle into arc edges at the ascending parameter `breaks`.
///
/// Consecutive break pairs become one arc edge each, so `[0, a, b, 2π]` yields
/// three arcs. Splitting keeps every arc's two endpoints distinct, which the
/// endpoint-based edge deduplication in [`openrcad_topo::Solid`] relies on (two
/// edges sharing both endpoints would otherwise be collapsed).
pub(crate) fn arc_edges(circle: Circle, breaks: &[f64]) -> Vec<Edge> {
    let mut edges = Vec::with_capacity(breaks.len().saturating_sub(1));
    for w in breaks.windows(2) {
        let (u0, u1) = (w[0], w[1]);
        let p0 = circle.point(u0);
        let p1 = circle.point(u1);
        edges.push(Edge::new(
            Some(GeomCurve::circle(circle)),
            u0,
            u1,
            Vertex::new(p0),
            Vertex::new(p1),
        ));
    }
    edges
}

/// A straight pcurve between two UV points. Its parameter interval is arc
/// length in UV space, independent of the corresponding 3D edge interval.
pub(crate) fn uv_line(
    start: Pnt2d,
    end: Pnt2d,
    periodicity: SurfacePeriodicity,
) -> PcurveData {
    let dx = end.x() - start.x();
    let dy = end.y() - start.y();
    let length = dx.hypot(dy);
    let direction = Dir2d::try_new(dx, dy).unwrap_or_else(Dir2d::dx);
    PcurveData::new(
        GeomCurve2d::line(Line2d::from_point_dir(start, direction)),
        0.0,
        length.max(openrcad_foundation::tolerance::CONFUSION),
    )
    .with_periodicity(periodicity)
}

/// Pcurve `u = first..last`, `v = constant` on a U-periodic surface.
pub(crate) fn angular_pcurve(first: f64, last: f64, v: f64) -> PcurveData {
    PcurveData::new(
        GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::new(0.0, v), Dir2d::dx())),
        first,
        last,
    )
    .with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU))
}

/// Pcurve `u = constant`, `v = first..last` on a U-periodic surface.
pub(crate) fn axial_pcurve(u: f64, first: f64, last: f64) -> PcurveData {
    PcurveData::new(
        GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::new(u, 0.0), Dir2d::dy())),
        first,
        last,
    )
    .with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU))
}

/// Build an exact pcurve-backed planar face for the line/circle/ellipse edges
/// used by native primitives.
pub(crate) fn planar_face(surface: GeomSurface, wire: Wire) -> Face {
    let GeomSurface::Plane(plane) = &surface else {
        panic!("planar_face requires a plane")
    };
    let pcurves = wire
        .edges()
        .iter()
        .map(|edge| planar_edge_pcurve(plane, edge))
        .collect();
    Face::with_pcurves(surface, wire, pcurves).expect("valid primitive planar pcurves")
}

fn planar_edge_pcurve(plane: &Plane, edge: &Edge) -> PcurveData {
    let project_point = |point: Pnt| {
        let offset = point - plane.location();
        Pnt2d::new(
            offset.dot(&FVec::from_dir(plane.position().x_direction())),
            offset.dot(&FVec::from_dir(plane.position().y_direction())),
        )
    };
    let project_direction = |direction: Dir| {
        Dir2d::new(
            direction.dot(&plane.position().x_direction()),
            direction.dot(&plane.position().y_direction()),
        )
    };
    match edge.curve() {
        Some(GeomCurve::Circle(circle)) => {
            let frame = Ax22d::new_axes(
                project_point(circle.center()),
                project_direction(circle.position().x_direction()),
                project_direction(circle.position().y_direction()),
            );
            PcurveData::new(
                GeomCurve2d::circle(Circle2d::new(frame, circle.radius())),
                edge.first(),
                edge.last(),
            )
        }
        Some(GeomCurve::Ellipse(ellipse)) => {
            let frame = Ax22d::new_axes(
                project_point(ellipse.center()),
                project_direction(ellipse.position().x_direction()),
                project_direction(ellipse.position().y_direction()),
            );
            PcurveData::new(
                GeomCurve2d::ellipse(Ellipse2d::new(
                    frame,
                    ellipse.major_radius(),
                    ellipse.minor_radius(),
                )),
                edge.first(),
                edge.last(),
            )
        }
        _ => uv_line(
            project_point(edge.start().point()),
            project_point(edge.end().point()),
            SurfacePeriodicity::NONE,
        ),
    }
}

/// A planar quadrilateral face through `a, b, c, d` (in winding order). The
/// supporting plane's normal is taken from the winding (Newell's method), so the
/// stored normal agrees with the boundary orientation.
pub(crate) fn quad_face(a: Pnt, b: Pnt, c: Pnt, d: Pnt) -> Face {
    let pts = [a, b, c, d];
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..4 {
        let p = pts[i];
        let q = pts[(i + 1) % 4];
        nx += (p.y() - q.y()) * (p.z() + q.z());
        ny += (p.z() - q.z()) * (p.x() + q.x());
        nz += (p.x() - q.x()) * (p.y() + q.y());
    }
    let n = FVec::new(nx, ny, nz)
        .normalized()
        .expect("degenerate quad face");
    let wire = Wire::from_edges([
        Edge::between_points(a, b),
        Edge::between_points(b, c),
        Edge::between_points(c, d),
        Edge::between_points(d, a),
    ]);
    planar_face(plane_at(a, n), wire)
}
