//! Fillets on a triangle prism fused with a cylinder boss straddling its
//! slanted face (the "line-drawn triangle + boss" GUI model):
//!
//! 1. The vertical seam edge where the boss wall meets the slanted planar face
//!    (a concave plane–cylinder crease).
//! 2. The prism's top edge on the slanted face, whose adjacent top face has a
//!    circular bite from the boss (a non-simple outer loop).

use std::collections::HashMap;

use openrcad_algo::{boolean, fillet_edges, prism, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomSurface, Plane};
use openrcad_mesh::tessellate;
use openrcad_primitives::make_cylinder;
use openrcad_topo::{Edge, Face, Solid, Wire};

const PRISM_H: f64 = 20.0;
const BOSS_H: f64 = 30.0;
const R: f64 = 6.0;

/// Triangle (0,0)-(40,0)-(0,30) extruded to `PRISM_H`, fused with a radius-6
/// vertical cylinder centered at the hypotenuse midpoint (20,15).
fn prism_boss_body() -> Solid {
    let tri = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(40.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(40.0, 0.0, 0.0), Pnt::new(0.0, 30.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 30.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]),
    );
    let wedge = prism(&tri, GeomVec::new(0.0, 0.0, PRISM_H)).expect("triangle should extrude");
    let boss = make_cylinder(
        &Ax2::new(Pnt::new(20.0, 15.0, 0.0), Dir::dz()),
        R,
        BOSS_H,
    );
    let fused = boolean(&wedge, &boss, BooleanOp::Fuse);
    assert!(fused.is_watertight(), "fused body must be watertight");
    assert!(
        fused.health_report().is_healthy(),
        "fused body must be healthy: {:?}",
        fused.health_report().errors
    );
    fused
}

/// Seam points: the boss circle is centered on the hypotenuse 3x+4y=120, so
/// the wall∩plane seams sit ±R along the line direction (-0.8, 0.6).
fn seam_xy(sign: f64) -> (f64, f64) {
    (20.0 + sign * -0.8 * R, 15.0 + sign * 0.6 * R)
}

fn cracks(s: &Solid) -> usize {
    let mesh = tessellate(s, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    let open: Vec<_> = edges
        .iter()
        .filter(|(_, &c)| c == 1)
        .take(12)
        .map(|(k, _)| *k)
        .collect();
    for (a, b) in &open {
        println!(
            "crack: [{:.3},{:.3},{:.3}]-[{:.3},{:.3},{:.3}]",
            a.0 as f64 / 1e4,
            a.1 as f64 / 1e4,
            a.2 as f64 / 1e4,
            b.0 as f64 / 1e4,
            b.1 as f64 / 1e4,
            b.2 as f64 / 1e4
        );
    }
    edges.values().filter(|&&c| c == 1).count()
}

fn assert_blend_ok(body: &Solid, edge: Edge, radius: f64, what: &str) {
    let rounded = fillet_edges(body, std::slice::from_ref(&edge), radius)
        .unwrap_or_else(|e| panic!("{what}: fillet must solve: {e}"));
    assert!(rounded.is_watertight(), "{what}: result must be watertight");
    assert!(
        rounded.health_report().is_healthy(),
        "{what}: result must be healthy: {:?}",
        rounded.health_report().errors
    );
    // The fused INPUT mesh already carries a few crack edges near the seam
    // tops (a pre-existing boolean-tessellation sliver unrelated to fillets);
    // the fillet must not ADD any.
    let before = cracks(body);
    let after = cracks(&rounded);
    assert!(
        after <= before,
        "{what}: fillet must not add render-mesh cracks (before {before}, after {after})"
    );
}

#[test]
fn debug_dump_fused_faces() {
    let body = prism_boss_body();
    for (fi, face) in body.shell().faces().iter().enumerate() {
        let kind = match face.surface() {
            Some(openrcad_geom::GeomSurface::Plane(_)) => "Plane",
            Some(openrcad_geom::GeomSurface::Cylinder(_)) => "Cyl",
            _ => "Other",
        };
        println!("face {fi}: {kind} orient={:?}", face.orientation());
        for wire in face.wires() {
            let pts: Vec<String> = wire
                .edges()
                .iter()
                .map(|e| {
                    let a = e.source().point();
                    let b = e.target().point();
                    format!(
                        "[{:.1},{:.1},{:.1}]->[{:.1},{:.1},{:.1}]",
                        a.x(),
                        a.y(),
                        a.z(),
                        b.x(),
                        b.y(),
                        b.z()
                    )
                })
                .collect();
            println!("  wire: {}", pts.join(" "));
        }
    }
}

#[test]
fn fillet_boss_plane_seam_edge() {
    let body = prism_boss_body();
    let (x, y) = seam_xy(1.0); // (15.2, 18.6)
    let edge = Edge::between_points(Pnt::new(x, y, 0.0), Pnt::new(x, y, PRISM_H));
    assert_blend_ok(&body, edge, 3.0, "boss/plane seam");
}

#[test]
fn fillet_prism_top_edge_with_boss_bite() {
    let body = prism_boss_body();
    let (bx, by) = seam_xy(-1.0); // (24.8, 11.4)
    let edge = Edge::between_points(Pnt::new(40.0, 0.0, PRISM_H), Pnt::new(bx, by, PRISM_H));
    assert_blend_ok(&body, edge, 3.0, "top edge with boss bite");
}

/// The greatest distance any tessellated vertex of `body` sits OUTSIDE the
/// prism formed by extruding the base polygon `base` (at z=0, any winding) up to
/// `height`. `<= ~0` means fully contained. Each base edge's outward-normal
/// signed distance is taken (orientation fixed by pointing away from the
/// centroid), plus the z slab — the max over all is the point's escape.
fn prism_outside_distance(body: &Solid, base: &[(f64, f64)], height: f64) -> f64 {
    let mesh = tessellate(body, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let n = base.len();
    let cx = base.iter().map(|p| p.0).sum::<f64>() / n as f64;
    let cy = base.iter().map(|p| p.1).sum::<f64>() / n as f64;
    let mut worst = f64::MIN;
    for v in gpu.positions.chunks_exact(3) {
        let (x, y, z) = (v[0] as f64, v[1] as f64, v[2] as f64);
        let mut d = (-z).max(z - height);
        for i in 0..n {
            let (ax, ay) = base[i];
            let (bx, by) = base[(i + 1) % n];
            let (ex, ey) = (bx - ax, by - ay);
            let len = ex.hypot(ey).max(1e-12);
            // Edge normal, flipped to point away from the centroid (outward).
            let (mut nx, mut ny) = (ey / len, -ex / len);
            if (cx - ax) * nx + (cy - ay) * ny > 0.0 {
                nx = -nx;
                ny = -ny;
            }
            d = d.max((x - ax) * nx + (y - ay) * ny);
        }
        worst = worst.max(d);
    }
    worst
}

/// Extrude the base triangle to `height`, fillet the vertical edge at base
/// vertex `corner` by `radius`, and assert the result is watertight, healthy,
/// crack-free, and stays inside the original prism (the GUI's containment gate
/// rejects any bulge — the bug this guards is the ball center landing too close
/// to the corner at non-right wedge angles).
fn assert_prism_corner_fillet_inside(
    base: &[(f64, f64)],
    height: f64,
    corner: usize,
    radius: f64,
    what: &str,
) {
    let edges: Vec<Edge> = (0..base.len())
        .map(|i| {
            let a = base[i];
            let b = base[(i + 1) % base.len()];
            Edge::between_points(Pnt::new(a.0, a.1, 0.0), Pnt::new(b.0, b.1, 0.0))
        })
        .collect();
    let face = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges(edges),
    );
    let body =
        prism(&face, GeomVec::new(0.0, 0.0, height)).expect("triangle base should extrude");
    assert!(body.is_watertight(), "{what}: base body must be watertight");
    let (cx, cy) = base[corner];
    let edge = Edge::between_points(Pnt::new(cx, cy, 0.0), Pnt::new(cx, cy, height));
    let rounded = fillet_edges(&body, std::slice::from_ref(&edge), radius)
        .unwrap_or_else(|e| panic!("{what}: fillet must solve: {e}"));
    assert!(rounded.is_watertight(), "{what}: result must be watertight");
    assert!(
        rounded.health_report().is_healthy(),
        "{what}: result must be healthy: {:?}",
        rounded.health_report().errors
    );
    assert_eq!(cracks(&rounded), 0, "{what}: result mesh must be crack-free");
    let out = prism_outside_distance(&rounded, base, height);
    assert!(
        out <= 1.0e-3,
        "{what}: fillet result bulges {out:.4} outside the prism"
    );
}

/// A plain triangle prism with an ACUTE (~44°) corner: filleting the vertical
/// edge at the sharp corner must round it and stay inside the original body.
#[test]
fn fillet_acute_prism_corner_edge() {
    assert_prism_corner_fillet_inside(
        &[(-7.4, 0.0), (5.0, -5.0), (5.0, 5.0)],
        6.0,
        0,
        1.42,
        "acute corner",
    );
}

/// A REALLY sharp (~23°) corner with a large radius: the contact feet land far
/// from the corner (r/tan(θ/2) ≈ 26) but still on the faces — the blend must
/// stay contained like any other wedge angle.
#[test]
fn fillet_sharp_prism_corner_edge() {
    assert_prism_corner_fillet_inside(
        &[(10.0, 0.0), (-30.0, -8.0), (-30.0, 8.0)],
        6.0,
        0,
        5.26,
        "sharp corner",
    );
}

/// The mirror OBTUSE (~127°) corner: the fix (ball center at radius/sin(θ/2))
/// must hold on both sides of 90°, not just the acute side.
#[test]
fn fillet_obtuse_prism_corner_edge() {
    assert_prism_corner_fillet_inside(
        &[(-1.0, 0.0), (5.0, -12.0), (5.0, 12.0)],
        6.0,
        0,
        1.42,
        "obtuse corner",
    );
}
