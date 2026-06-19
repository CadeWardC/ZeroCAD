//! Box primitive — an axis-aligned rectangular solid (OCCT `BRepPrimAPI_MakeBox`).
//!
//! Builds a watertight solid: 8 vertices, 12 edges, 6 quadrilateral faces. Each
//! face is wound counter-clockwise as seen from *outside* the box and carries a
//! [`Plane`] whose normal points outward, so the resulting B-Rep is
//! consistently oriented even though this first topology layer does not yet
//! enforce normal agreement.

use openrcad_foundation::{Dir, Pnt};
use openrcad_geom::{GeomSurface, Plane};
use openrcad_topo::{Edge, Face, Shell, Solid, Wire};

/// Build an axis-aligned box with one corner at `corner` and extents `dx`, `dy`,
/// `dz` along +X, +Y, +Z.
///
/// Matches OCCT's `BRepPrimAPI_MakeBox(Pnt, dx, dy, dz)`.
///
/// ```
/// use openrcad_primitives::make_box;
/// use openrcad_foundation::Pnt;
///
/// let b = make_box(&Pnt::origin(), 10.0, 20.0, 30.0);
/// assert_eq!(b.vertex_count(), 8);
/// assert_eq!(b.edge_count(), 12);
/// assert_eq!(b.face_count(), 6);
/// // A closed box satisfies Euler–Poincaré: V − E + F = 2.
/// assert_eq!(b.euler_characteristic(), 2);
/// ```
pub fn make_box(corner: &Pnt, dx: f64, dy: f64, dz: f64) -> Solid {
    assert!(dx > 0.0, "make_box: dx must be positive");
    assert!(dy > 0.0, "make_box: dy must be positive");
    assert!(dz > 0.0, "make_box: dz must be positive");

    let (x0, y0, z0) = (corner.x(), corner.y(), corner.z());
    let (x1, y1, z1) = (x0 + dx, y0 + dy, z0 + dz);

    // The 8 corners, named by binary (x,y,z) offsets from `corner`.
    let v000 = Pnt::new(x0, y0, z0);
    let v100 = Pnt::new(x1, y0, z0);
    let v110 = Pnt::new(x1, y1, z0);
    let v010 = Pnt::new(x0, y1, z0);
    let v001 = Pnt::new(x0, y0, z1);
    let v101 = Pnt::new(x1, y0, z1);
    let v111 = Pnt::new(x1, y1, z1);
    let v011 = Pnt::new(x0, y1, z1);

    let bottom = rect_face(
        v000,
        v100,
        v110,
        v010,
        plane_at(v000, Dir::new(0.0, 0.0, -1.0)),
    );
    let top = rect_face(v001, v011, v111, v101, plane_at(v001, Dir::dz()));
    let front = rect_face(
        v000,
        v001,
        v101,
        v100,
        plane_at(v000, Dir::new(0.0, -1.0, 0.0)),
    );
    let back = rect_face(v010, v110, v111, v011, plane_at(v010, Dir::dy()));
    let left = rect_face(
        v000,
        v010,
        v011,
        v001,
        plane_at(v000, Dir::new(-1.0, 0.0, 0.0)),
    );
    let right = rect_face(v100, v101, v111, v110, plane_at(v100, Dir::dx()));

    Solid::new(Shell::from_faces([bottom, top, front, back, left, right]))
}

/// A rectangular face through `a, b, c, d` (CCW from outside) on `surf`.
fn rect_face(a: Pnt, b: Pnt, c: Pnt, d: Pnt, surf: GeomSurface) -> Face {
    let wire = Wire::from_edges([
        Edge::between_points(a, b),
        Edge::between_points(b, c),
        Edge::between_points(c, d),
        Edge::between_points(d, a),
    ]);
    Face::new(Some(surf), wire)
}

/// A plane through `p` with outward normal `n`.
fn plane_at(p: Pnt, n: Dir) -> GeomSurface {
    GeomSurface::plane(Plane::from_point_normal(p, n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax1, Trsf};

    #[test]
    fn box_topology_counts() {
        let solid = make_box(&Pnt::origin(), 10.0, 20.0, 30.0);
        assert_eq!(solid.shell().faces().len(), 6);
        assert_eq!(solid.vertex_count(), 8);
        assert_eq!(solid.edge_count(), 12);
        assert_eq!(solid.face_count(), 6);
    }

    #[test]
    fn box_wires_are_closed() {
        let solid = make_box(&Pnt::origin(), 10.0, 20.0, 30.0);
        for face in solid.shell().faces() {
            let w = face.outer_wire().unwrap();
            assert!(w.is_closed(), "Wire is not closed!");
        }
    }

    #[test]
    fn box_bounding_box() {
        let solid = make_box(&Pnt::new(1.0, 2.0, 3.0), 10.0, 20.0, 30.0);
        let (lo, hi) = solid.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::new(1.0, 2.0, 3.0));
        assert_eq!(hi, Pnt::new(11.0, 22.0, 33.0));
    }

    #[test]
    fn rotated_box_refits_aabb() {
        // The README example, verbatim in spirit.
        let solid = make_box(&Pnt::origin(), 10.0, 20.0, 30.0);
        let rot = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            core::f64::consts::FRAC_PI_2,
        );
        let moved = solid.transformed(&rot);
        let (min, _max) = moved.bounding_box().corners().unwrap();
        // Box x in [0,10], y in [0,20]; 90° about Z sends (x,y) -> (-y, x),
        // so the new x range is [-20, 0].
        assert!((min.x() - (-20.0)).abs() < 1e-9);
        // Rotation preserves counts.
        assert_eq!(moved.vertex_count(), 8);
        assert_eq!(moved.edge_count(), 12);
    }
}
