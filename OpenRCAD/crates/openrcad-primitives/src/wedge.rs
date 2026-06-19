//! Wedge primitive (OCCT `BRepPrimAPI_MakeWedge`).
//!
//! An axis-aligned box `[0,dx] × [0,dy] × [0,dz]` whose top face (`z = dz`) has
//! its X-extent shrunk to `[0, ltx]`, producing two slanted side faces. Like the
//! box: 8 vertices, 12 edges, 6 planar faces (χ = 2).

use openrcad_foundation::Pnt;
use openrcad_topo::{Shell, Solid};

use crate::common::quad_face;

/// Build a wedge with base extents `dx`, `dy`, `dz` and top X-extent `ltx`
/// (`0 < ltx ≤ dx` gives the classic wedge; `ltx = dx` degenerates to a box).
pub fn make_wedge(dx: f64, dy: f64, dz: f64, ltx: f64) -> Solid {
    assert!(
        dx > 0.0 && dy > 0.0 && dz > 0.0,
        "make_wedge: extents must be positive"
    );
    assert!(ltx > 0.0 && ltx <= dx, "make_wedge: require 0 < ltx <= dx");

    // Base corners (z = 0).
    let v000 = Pnt::new(0.0, 0.0, 0.0);
    let v100 = Pnt::new(dx, 0.0, 0.0);
    let v110 = Pnt::new(dx, dy, 0.0);
    let v010 = Pnt::new(0.0, dy, 0.0);
    // Top corners (z = dz), X-extent shrunk to ltx.
    let v001 = Pnt::new(0.0, 0.0, dz);
    let v101 = Pnt::new(ltx, 0.0, dz);
    let v111 = Pnt::new(ltx, dy, dz);
    let v011 = Pnt::new(0.0, dy, dz);

    // Same winding scheme as make_box; quad_face derives each outward normal.
    let bottom = quad_face(v000, v100, v110, v010);
    let top = quad_face(v001, v011, v111, v101);
    let front = quad_face(v000, v001, v101, v100);
    let back = quad_face(v010, v110, v111, v011);
    let left = quad_face(v000, v010, v011, v001);
    let right = quad_face(v100, v101, v111, v110);

    Solid::new(Shell::from_faces([bottom, top, front, back, left, right]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wedge_counts_and_euler() {
        let s = make_wedge(10.0, 6.0, 4.0, 4.0);
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 6);
        let chi = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(chi, 2);
    }

    #[test]
    fn wedge_bounding_box() {
        let s = make_wedge(10.0, 6.0, 4.0, 4.0);
        let (lo, hi) = s.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::origin());
        assert_eq!(hi, Pnt::new(10.0, 6.0, 4.0));
    }

    #[test]
    fn wedge_round_trips_through_serde() {
        let s = make_wedge(10.0, 6.0, 4.0, 4.0);
        let json = serde_json::to_string(&s).unwrap();
        let back: Solid = serde_json::from_str(&json).unwrap();
        assert_eq!(back.vertex_count(), 8);
        assert_eq!(back.face_count(), 6);
    }
}
