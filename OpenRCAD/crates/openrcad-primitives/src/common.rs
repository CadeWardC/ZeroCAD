//! Shared construction helpers for the primitive builders.

use openrcad_foundation::{Dir, Pnt, Vec as FVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_topo::{Edge, Face, Vertex, Wire};

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
    Face::new(Some(plane_at(a, n)), wire)
}
