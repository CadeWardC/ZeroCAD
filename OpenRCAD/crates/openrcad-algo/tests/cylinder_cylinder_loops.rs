//! Crossing cylinders meet in closed loops. Found by the ZeroCAD mixed-axis
//! join/cut census: a side tunnel grazing past the widest point of a vertical
//! hole meets its wall in one loop that turns back twice in the parameter it
//! is traced along. Traced from samples alone, both branches stopped short of
//! each turning point, the loop came back as open pieces with ~0.6 mm gaps,
//! the faces were never split, and the cut returned a valid solid holding a
//! phantom slab of material inside the hole.
use openrcad_algo::intersect::surface_surface;
use openrcad_foundation::{Ax3, Dir, Pnt};
use openrcad_geom::{Curve, CylindricalSurface, GeomCurve, GeomSurface};

fn cylinder(location: Pnt, axis: Dir, radius: f64) -> GeomSurface {
    GeomSurface::cylinder(CylindricalSurface::new(Ax3::new(location, axis), radius))
}

/// Distance from `p` to the axis line through `location` along `axis`.
fn axis_distance(p: Pnt, location: Pnt, axis: Dir) -> f64 {
    let d = p - location;
    let along = d.x() * axis.x() + d.y() * axis.y() + d.z() * axis.z();
    let radial = (
        d.x() - along * axis.x(),
        d.y() - along * axis.y(),
        d.z() - along * axis.z(),
    );
    (radial.0 * radial.0 + radial.1 * radial.1 + radial.2 * radial.2).sqrt()
}

/// Every curve must be a closed loop lying on both cylinders.
fn assert_closed_loops_on_both(
    curves: &[GeomCurve],
    first: (Pnt, Dir, f64),
    second: (Pnt, Dir, f64),
) {
    for curve in curves {
        let (t0, t1) = curve.bounds();
        assert_eq!(
            curve.point(t0),
            curve.point(t1),
            "an intersection loop must close exactly"
        );
        for i in 0..=400 {
            let p = curve.point(t0 + (t1 - t0) * i as f64 / 400.0);
            for (location, axis, radius) in [first, second] {
                // Vertices found on the loop must sit well inside the kernel's
                // 1e-5 vertex and pcurve tolerances, so the loop must too.
                let error = (axis_distance(p, location, axis) - radius).abs();
                assert!(error < 1.0e-6, "loop point {p:?} is {error} off a cylinder");
            }
        }
    }
}

#[test]
fn grazing_cylinders_meet_in_one_closed_loop() {
    let hole = (
        Pnt::new(7.437238573622317, 24.56758494682629, 0.0),
        Dir::dz(),
        8.47739536547999,
    );
    // The tunnel reaches 0.12 past the hole's widest point.
    let tunnel = (
        Pnt::new(13.601359326051771, 0.0, 4.115005235872185),
        Dir::dy(),
        2.4354769329312296,
    );
    for (s1, s2) in [(hole, tunnel), (tunnel, hole)] {
        let curves = surface_surface(
            &cylinder(s1.0, s1.1, s1.2),
            &cylinder(s2.0, s2.1, s2.2),
            1.0e-7,
        );
        assert_eq!(
            curves.len(),
            1,
            "grazing cylinders meet in exactly one loop"
        );
        assert_closed_loops_on_both(&curves, hole, tunnel);
    }
}

#[test]
fn piercing_cylinders_meet_in_two_closed_loops() {
    let hole = (Pnt::new(0.0, 0.0, 0.0), Dir::dz(), 8.0);
    let tunnel = (Pnt::new(1.0, 0.0, 4.0), Dir::dy(), 2.5);
    for (s1, s2) in [(hole, tunnel), (tunnel, hole)] {
        let curves = surface_surface(
            &cylinder(s1.0, s1.1, s1.2),
            &cylinder(s2.0, s2.1, s2.2),
            1.0e-7,
        );
        assert_eq!(
            curves.len(),
            2,
            "a tunnel through a hole wall leaves two windows"
        );
        assert_closed_loops_on_both(&curves, hole, tunnel);
    }
}
