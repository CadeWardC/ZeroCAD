//! Cylinder primitive (OCCT `BRepPrimAPI_MakeCylinder`).
//!
//! Builds a watertight solid from a [`CylindricalSurface`] lateral wall and two
//! planar end caps. To keep every edge's endpoints distinct (required by the
//! endpoint-based dedup in [`Solid`]), both circular rims are split into three
//! arcs, giving 6 vertices, 9 edges, 5 faces — Euler characteristic
//! χ = 6 − 9 + 5 = 2.

use core::f64::consts::PI;

use openrcad_foundation::{Ax2, Ax3, Vec as FVec};
use openrcad_geom::{Circle, Curve, CylindricalSurface, GeomSurface};
use openrcad_topo::{Edge, Face, Shell, Solid, Wire};

use crate::common::{arc_edges, plane_at};

/// Build a cylinder of `radius` and `height` along the main direction of `axis`,
/// based at `axis.location()`.
pub fn make_cylinder(axis: &Ax2, radius: f64, height: f64) -> Solid {
    assert!(radius > 0.0, "make_cylinder: radius must be positive");
    assert!(height > 0.0, "make_cylinder: height must be positive");

    let frame: Ax3 = (*axis).into();
    let base = frame.location();
    let zdir = frame.direction();
    let xref = frame.x_direction();
    let top = base + FVec::from_dir(zdir) * height;

    let base_frame = Ax3::new_axes(base, zdir, xref);
    let top_frame = Ax3::new_axes(top, zdir, xref);
    let base_circle = Circle::new(base_frame, radius);
    let top_circle = Circle::new(top_frame, radius);

    let thirds = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0, 2.0 * PI];
    let base_arcs = arc_edges(base_circle, &thirds);
    let top_arcs = arc_edges(top_circle, &thirds);

    // Three seam lines joining matching rim split points.
    let seam_params = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0];
    let seams: Vec<Edge> = seam_params
        .iter()
        .map(|&u| Edge::between_points(base_circle.point(u), top_circle.point(u)))
        .collect();

    let bottom = Face::new(
        Some(plane_at(base, zdir.reversed())),
        Wire::from_edges(base_arcs.clone()),
    );
    let top_face = Face::new(
        Some(plane_at(top, zdir)),
        Wire::from_edges(top_arcs.clone()),
    );

    let lateral = GeomSurface::cylinder(CylindricalSurface::new(base_frame, radius));
    let mut faces = vec![bottom, top_face];
    for i in 0..3 {
        let next = (i + 1) % 3;
        let wire = Wire::from_edges([
            base_arcs[i].clone(),
            seams[next].clone(),
            top_arcs[i].reversed(),
            seams[i].reversed(),
        ]);
        faces.push(Face::new(Some(lateral.clone()), wire));
    }

    Solid::new(Shell::from_faces(faces))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir, Pnt};

    fn unit_cylinder() -> Solid {
        make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 5.0)
    }

    #[test]
    fn cylinder_topology_counts_and_euler() {
        let s = unit_cylinder();
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.edge_count(), 9);
        assert_eq!(s.face_count(), 5);
        let chi = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(chi, 2, "Euler characteristic of a cylinder must be 2");
    }

    #[test]
    fn cylinder_bounding_box() {
        // NOTE: `bounding_box` covers the rim *vertices* (3 per circle), not the
        // swept surface, so the radial extent is sampled at the split points.
        let s = unit_cylinder();
        let (lo, hi) = s.bounding_box().corners().unwrap();
        assert!((lo.z() - 0.0).abs() < 1e-9);
        assert!((hi.z() - 5.0).abs() < 1e-9);
        // Every vertex stays within the radius in X/Y.
        for c in [lo.x(), lo.y(), hi.x(), hi.y()] {
            assert!(c.abs() <= 2.0 + 1e-9);
        }
    }

    #[test]
    fn cylinder_round_trips_through_serde() {
        let s = unit_cylinder();
        let json = serde_json::to_string(&s).unwrap();
        let back: Solid = serde_json::from_str(&json).unwrap();
        assert_eq!(back.vertex_count(), 6);
        assert_eq!(back.edge_count(), 9);
        assert_eq!(back.face_count(), 5);
    }
}
