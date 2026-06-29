//! Fillet a selected planar edge whose endpoint runs into a tangent curved wall.
//!
//! This reproduces an extruded-sketch topology: a straight sketch segment is
//! tangent to a circular sketch arc, both swept into a prism. Filleting the top
//! copy of the straight segment must run out cleanly into the swept arc's
//! analytic cylinder instead of capping the blend with an off-wall circular arc.

use core::f64::consts::FRAC_PI_2;
use std::collections::HashMap;

use openrcad_algo::{fillet_edges, prism};
use openrcad_foundation::{Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_mesh::tessellate;
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};

type Key = (i64, i64, i64);

fn tangent_d_profile_body() -> Solid {
    let circle = Circle::new(
        Ax3::new_axes(Pnt::new(5.0, 5.0, 0.0), Dir::dz(), Dir::dx()),
        5.0,
    );
    let arc = Edge::new(
        Some(GeomCurve::circle(circle)),
        -FRAC_PI_2,
        FRAC_PI_2,
        Vertex::new(circle.point(-FRAC_PI_2)),
        Vertex::new(circle.point(FRAC_PI_2)),
    );
    let face = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(5.0, 0.0, 0.0)),
            arc,
            Edge::between_points(Pnt::new(5.0, 10.0, 0.0), Pnt::new(0.0, 10.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 10.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]),
    );
    prism(&face, GeomVec::new(0.0, 0.0, 5.0)).expect("D profile should extrude")
}

fn selected_tangent_runout_edge() -> Edge {
    Edge::between_points(Pnt::new(0.0, 0.0, 5.0), Pnt::new(5.0, 0.0, 5.0))
}

fn cracks(s: &Solid) -> usize {
    let mesh = tessellate(s, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> Key {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&c| c == 1).count()
}

fn curved_wall_boundary_stays_on_cylinder(s: &Solid) -> bool {
    let faces = s.shell().faces();
    let Some(wall) = faces.iter().find(|f| {
        matches!(
            f.surface(),
            Some(GeomSurface::Cylinder(c))
                if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                    && (c.radius() - 5.0).abs() < 1.0e-3
        )
    }) else {
        return false;
    };
    let Some(wire) = wall.outer_wire() else {
        return false;
    };
    for edge in wire.edges() {
        let Some(curve) = edge.curve() else {
            return false;
        };
        for k in 0..=12 {
            let t = edge.first() + (edge.last() - edge.first()) * k as f64 / 12.0;
            let p = curve.point(t);
            let radial = ((p.x() - 5.0).powi(2) + (p.y() - 5.0).powi(2)).sqrt();
            if (radial - 5.0).abs() > 5.0e-3 {
                return false;
            }
        }
    }
    true
}

#[test]
fn fillet_planar_edge_runs_out_into_tangent_curved_wall() {
    let body = tangent_d_profile_body();
    assert!(
        body.is_watertight(),
        "precondition: body must be watertight"
    );
    assert!(
        body.health_report().is_healthy(),
        "precondition: body must be healthy: {:?}",
        body.health_report().errors
    );

    let selected = selected_tangent_runout_edge();
    for r in [0.5_f64, 1.0, 1.5] {
        let rounded = fillet_edges(&body, std::slice::from_ref(&selected), r)
            .unwrap_or_else(|e| panic!("r={r}: tangent curved-wall runout must solve: {e:?}"));

        assert!(rounded.is_watertight(), "r={r}: result must be watertight");
        assert!(
            rounded.health_report().is_healthy(),
            "r={r}: result must be healthy: {:?}",
            rounded.health_report().errors
        );
        assert!(
            rounded.shell().faces().iter().any(|f| {
                matches!(f.surface(), Some(GeomSurface::Cylinder(c)) if (c.radius() - r).abs() < 1.0e-3)
            }),
            "r={r}: selected-edge blend cylinder must be present"
        );
        assert!(
            curved_wall_boundary_stays_on_cylinder(&rounded),
            "r={r}: tangent curved wall must remain an analytic cylinder"
        );
        assert_eq!(
            cracks(&rounded),
            0,
            "r={r}: result must tessellate crack-free"
        );
    }
}
