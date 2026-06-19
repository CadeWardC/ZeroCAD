//! Cone primitive (OCCT `BRepPrimAPI_MakeCone`).
//!
//! Builds a (possibly truncated) cone from a [`ConicalSurface`] lateral wall and
//! planar end cap(s). Both rims are split into three arcs to keep edge endpoints
//! distinct.
//!
//! - **Truncated** (`r2 > 0`): 6 vertices, 9 edges, 5 faces (χ = 2), like a
//!   cylinder but with a tapered wall.
//! - **Apex** (`r2 = 0`): the top rim collapses to a single apex vertex, giving
//!   4 vertices, 6 edges, 4 faces (χ = 2).

use core::f64::consts::PI;

use openrcad_foundation::{Ax2, Ax3, Vec as FVec};
use openrcad_geom::{Circle, ConicalSurface, Curve, GeomSurface};
use openrcad_topo::{Edge, Face, Shell, Solid, Vertex, Wire};

use crate::common::{arc_edges, plane_at};

/// Build a (truncated) cone along the main direction of `axis`, with base radius
/// `r1` at `axis.location()` and top radius `r2` at height `height`. Set
/// `r2 = 0` for a sharp apex.
pub fn make_cone(axis: &Ax2, r1: f64, r2: f64, height: f64) -> Solid {
    assert!(
        r1 >= 0.0 && r2 >= 0.0,
        "make_cone: radii must be non-negative"
    );
    assert!(r1 > 0.0, "make_cone: base radius r1 must be positive");
    assert!(height > 0.0, "make_cone: height must be positive");

    let frame: Ax3 = (*axis).into();
    let base = frame.location();
    let zdir = frame.direction();
    let xref = frame.x_direction();
    let top = base + FVec::from_dir(zdir) * height;
    let base_frame = Ax3::new_axes(base, zdir, xref);

    // r(v) = r1 + v·tan(semi_angle); slope = (r2 − r1)/height.
    let semi_angle = ((r2 - r1) / height).atan();
    let lateral = GeomSurface::cone(ConicalSurface::new(base_frame, r1, semi_angle));

    let thirds = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0, 2.0 * PI];
    let seam_params = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0];
    let base_circle = Circle::new(base_frame, r1);
    let base_arcs = arc_edges(base_circle, &thirds);

    let bottom = Face::new(
        Some(plane_at(base, zdir.reversed())),
        Wire::from_edges(base_arcs.clone()),
    );
    let mut faces = vec![bottom];

    if r2 > openrcad_foundation::tolerance::CONFUSION {
        // Truncated: a top cap plus three tapered wall quads.
        let top_frame = Ax3::new_axes(top, zdir, xref);
        let top_circle = Circle::new(top_frame, r2);
        let top_arcs = arc_edges(top_circle, &thirds);
        let seams: Vec<Edge> = seam_params
            .iter()
            .map(|&u| Edge::between_points(base_circle.point(u), top_circle.point(u)))
            .collect();

        faces.push(Face::new(
            Some(plane_at(top, zdir)),
            Wire::from_edges(top_arcs.clone()),
        ));
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
    } else {
        // Sharp apex: three triangular wall faces meeting at the tip.
        let apex = top;
        let seams: Vec<Edge> = seam_params
            .iter()
            .map(|&u| {
                Edge::new(
                    None,
                    0.0,
                    0.0,
                    Vertex::new(base_circle.point(u)),
                    Vertex::new(apex),
                )
            })
            .collect();
        for i in 0..3 {
            let next = (i + 1) % 3;
            let wire = Wire::from_edges([
                base_arcs[i].clone(),
                seams[next].clone(),
                seams[i].reversed(),
            ]);
            faces.push(Face::new(Some(lateral.clone()), wire));
        }
    }

    Solid::new(Shell::from_faces(faces))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir, Pnt};

    #[test]
    fn truncated_cone_counts_and_euler() {
        let s = make_cone(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 1.5, 4.0);
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.edge_count(), 9);
        assert_eq!(s.face_count(), 5);
        let chi = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(chi, 2);
    }

    #[test]
    fn apex_cone_counts_and_euler() {
        let s = make_cone(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 0.0, 4.0);
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 6);
        assert_eq!(s.face_count(), 4);
        let chi = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(chi, 2);
    }

    #[test]
    fn cone_round_trips_through_serde() {
        let s = make_cone(&Ax2::new(Pnt::origin(), Dir::dz()), 3.0, 0.0, 4.0);
        let json = serde_json::to_string(&s).unwrap();
        let back: Solid = serde_json::from_str(&json).unwrap();
        assert_eq!(back.vertex_count(), 4);
        assert_eq!(back.face_count(), 4);
    }
}
