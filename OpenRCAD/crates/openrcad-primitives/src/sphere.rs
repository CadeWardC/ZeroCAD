//! Sphere primitive (OCCT `BRepPrimAPI_MakeSphere`).
//!
//! Builds a watertight ball whose faces all lie on a single [`SphericalSurface`].
//! The sphere is cut into 4 longitude sectors × 2 latitude bands, with the polar
//! bands as triangles meeting at the poles. This keeps every edge's two
//! endpoints distinct (required by the endpoint-based dedup in [`Solid`]):
//! 6 vertices (2 poles + 4 equator), 12 edges, 8 faces — χ = 6 − 12 + 8 = 2.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Vec as FVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, SphericalSurface};
use openrcad_topo::{Edge, Face, Shell, Solid, Vertex, Wire};

use crate::common::arc_edges;

/// Build a sphere of `radius` centered at `center`.
pub fn make_sphere(center: &Pnt, radius: f64) -> Solid {
    assert!(radius > 0.0, "make_sphere: radius must be positive");

    let frame = Ax3::new(*center, openrcad_foundation::Dir::dz());
    let zdir = frame.direction();
    let xref = frame.x_direction();
    let x = FVec::from_dir(xref);
    let y = FVec::from_dir(frame.y_direction());
    let z = FVec::from_dir(zdir);

    let surf = GeomSurface::sphere(SphericalSurface::new(
        Ax3::new_axes(*center, zdir, xref),
        radius,
    ));

    // Equator split into four quarter-arcs.
    let eq_circle = Circle::new(Ax3::new_axes(*center, zdir, xref), radius);
    let eq = arc_edges(eq_circle, &[0.0, PI / 2.0, PI, 3.0 * PI / 2.0, 2.0 * PI]);

    // Meridian quarter-arcs (great circles through the poles), one per longitude.
    let mut upper: Vec<Edge> = Vec::with_capacity(4);
    let mut lower: Vec<Edge> = Vec::with_capacity(4);
    for i in 0..4 {
        let u = i as f64 * PI / 2.0;
        let radial = x * u.cos() + y * u.sin();
        let radial_dir = radial.normalized().expect("non-degenerate radial");
        let main = radial
            .cross(&z)
            .normalized()
            .expect("non-degenerate meridian normal");
        let gc = Circle::new(Ax3::new_axes(*center, main, radial_dir), radius);
        let e_i = gc.point(0.0);
        let north = gc.point(PI / 2.0);
        let south = gc.point(-PI / 2.0);
        upper.push(Edge::new(
            Some(GeomCurve::circle(gc)),
            0.0,
            PI / 2.0,
            Vertex::new(e_i),
            Vertex::new(north),
        ));
        lower.push(Edge::new(
            Some(GeomCurve::circle(gc)),
            0.0,
            -PI / 2.0,
            Vertex::new(e_i),
            Vertex::new(south),
        ));
    }

    let mut faces = Vec::with_capacity(8);
    for i in 0..4 {
        let next = (i + 1) % 4;
        // Upper triangle: e_i → e_{i+1} → N → e_i.
        faces.push(Face::new(
            Some(surf.clone()),
            Wire::from_edges([eq[i].clone(), upper[next].clone(), upper[i].reversed()]),
        ));
        // Lower triangle: e_i → S → e_{i+1} → e_i.
        faces.push(Face::new(
            Some(surf.clone()),
            Wire::from_edges([lower[i].clone(), lower[next].reversed(), eq[i].reversed()]),
        ));
    }

    Solid::new(Shell::from_faces(faces))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_counts_and_euler() {
        let s = make_sphere(&Pnt::new(1.0, 2.0, 3.0), 4.0);
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 8);
        let chi = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(chi, 2, "Euler characteristic of a sphere must be 2");
    }

    #[test]
    fn sphere_bounding_box_is_centered() {
        let s = make_sphere(&Pnt::origin(), 4.0);
        let (lo, hi) = s.bounding_box().corners().unwrap();
        for c in [lo.x(), lo.y(), lo.z()] {
            assert!((c - (-4.0)).abs() < 1e-9);
        }
        for c in [hi.x(), hi.y(), hi.z()] {
            assert!((c - 4.0).abs() < 1e-9);
        }
    }

    #[test]
    fn sphere_round_trips_through_serde() {
        let s = make_sphere(&Pnt::origin(), 4.0);
        let json = serde_json::to_string(&s).unwrap();
        let back: Solid = serde_json::from_str(&json).unwrap();
        assert_eq!(back.vertex_count(), 6);
        assert_eq!(back.edge_count(), 12);
        assert_eq!(back.face_count(), 8);
    }
}
