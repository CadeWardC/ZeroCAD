//! A planar loop whose circular boundary wraps a reflex corner is valid even
//! though the chord of a wide arc crosses the corner's edge. Found by the
//! ZeroCAD join/cut property sweep: a cylinder boss overlapping a block corner
//! leaves exactly this overhang region, and strict validation rejected its
//! section prism as `UvLoopSelfIntersection`, failing an everyday Join.
use std::f64::consts::{PI, TAU};

use openrcad_algo::prism_operation;
use openrcad_foundation::{Ax3, Dir, Pnt, TolerancePolicy};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_topo::{Edge, Face, Vertex, Wire};

#[test]
fn arc_wrapping_a_reflex_corner_is_not_a_self_intersection() {
    // Circle (38, 6.5) r 10.5 minus the block quadrant x < 43, y > 0: the
    // overhang is bounded by y = 0, the circle below and right of the corner,
    // and x = 43.
    let circle = Circle::new(
        Ax3::new_axes(Pnt::new(38.0, 6.5, 0.0), Dir::dz(), Dir::dx()),
        10.5,
    );
    let start = (-6.5_f64).atan2(-(10.5_f64 * 10.5 - 6.5 * 6.5).sqrt()) + TAU;
    let end = (10.5_f64 * 10.5 - 5.0 * 5.0).sqrt().atan2(5.0) + TAU;
    // Split where a sketch arrangement splits it, at 3pi/2 and 2pi. The chord
    // of the middle arc, (38, -4) to (48.5, 6.5), crosses x = 43 at y = 1.
    let breaks = [start, 1.5 * PI, TAU, end];
    let arcs = breaks.windows(2).map(|pair| {
        Edge::new(
            Some(GeomCurve::circle(circle)),
            pair[0],
            pair[1],
            Vertex::new(circle.point(pair[0])),
            Vertex::new(circle.point(pair[1])),
        )
    });
    let corner = Pnt::new(43.0, 0.0, 0.0);
    let mut edges = vec![Edge::between_points(corner, circle.point(start))];
    edges.extend(arcs);
    edges.push(Edge::between_points(circle.point(end), corner));
    let profile = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges(edges),
    );

    let solid = prism_operation(&profile, openrcad_foundation::Vec::new(0.0, 0.0, 1.0))
        .expect("the overhang region is a simple loop")
        .value;
    assert!(solid.is_watertight());
    solid
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("arc chords crossing a reflex edge are not a loop self-intersection");
}

/// The arc can clear the corner by far less than one fixed-angle chord's sag
/// (found by the ZeroCAD join/cut census, chain 307): a boss whose 120° wall
/// arc passes 0.12 mm outside the block corner. One chord per 22.5° sags
/// 0.21 mm on this 11 mm arc, so the refined pass still saw a crossing; the
/// refined chords are now bounded by their sag instead.
#[test]
fn boss_arc_grazing_past_a_block_corner_fuses() {
    use openrcad_algo::{boolean_operation, BooleanOp};
    use openrcad_foundation::Ax2;
    use openrcad_primitives::{make_box_operation, make_cylinder_operation};
    let block = make_box_operation(
        &Pnt::origin(),
        51.3437823009577,
        46.331810469804395,
        12.354889773812346,
    )
    .expect("block")
    .value;
    let boss = make_cylinder_operation(
        &Ax2::new(
            Pnt::new(44.097025332946465, 37.99495370039926, 1.6778834807720107),
            Dir::dz(),
        ),
        11.138586111407127,
        12.004219510260441,
    )
    .expect("boss")
    .value;
    let fused = boolean_operation(&block, &boss, BooleanOp::Fuse)
        .expect("the boss fuses over the block corner")
        .value;
    fused
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("valid solid");
}
