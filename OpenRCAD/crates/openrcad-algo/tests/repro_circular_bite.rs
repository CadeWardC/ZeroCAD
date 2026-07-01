//! Cutting a cylinder whose footprint *straddles* a box wall must leave a clean
//! concave cylindrical bite — not collapse the wall into planar facets.
//!
//! This is the "vanishing circular bite": ZeroCAD builds a rect-minus-circle
//! body (and the box-cylinder fillet fallback part) by subtracting a cylinder
//! whose circle crosses one rectangle edge. A clean 2-point crosscut imprinted
//! onto the cylindrical wall used to be split *immediately*, fragmenting the
//! wall so the bite degenerated to a box. `split_tracked` now force-queues clean
//! crosscuts on cylindrical faces so the wall partitions together and
//! `merge_cocylindrical_faces` keeps it as one analytic cylinder.

use openrcad_algo::{boolean_checked, BooleanError, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::{Curve, GeomSurface};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

/// The reported sketch: a 40×30 rectangle from (0,5) and a circle at (20,8)
/// radius 14 that crosses the bottom edge — the cutter bites a concave scallop
/// out of the bottom wall.
const BITE_CX: f64 = 20.0;
const BITE_CY: f64 = 8.0;
const BITE_R: f64 = 14.0;

fn circular_bite_cut() -> Result<Solid, BooleanError> {
    let block = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    // Axis vertical (+Z), tall enough to pass fully through the 10mm-thick block.
    let cutter = make_cylinder(
        &Ax2::new(Pnt::new(BITE_CX, BITE_CY, -1.0), Dir::dz()),
        BITE_R,
        12.0,
    );
    boolean_checked(&block, &cutter, BooleanOp::Cut)
}

/// True when every cylindrical wall face of radius ≈ `BITE_R` about a vertical
/// axis has all its boundary-edge points lying on that cylinder — i.e. the bite
/// is a real analytic scallop, not facets masquerading as one.
fn bite_wall_stays_on_cylinder(s: &Solid) -> bool {
    let walls: Vec<_> = s
        .shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - BITE_R).abs() < 1.0e-3
            )
        })
        .collect();

    if walls.is_empty() {
        return false;
    }

    for wall in &walls {
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
                let radial = ((p.x() - BITE_CX).powi(2) + (p.y() - BITE_CY).powi(2)).sqrt();
                if (radial - BITE_R).abs() > 5.0e-3 {
                    return false;
                }
            }
        }
    }
    true
}

#[test]
fn straddling_cylinder_cut_is_watertight_and_healthy() {
    let bite = circular_bite_cut()
        .unwrap_or_else(|e| panic!("circular-bite cut must succeed: {e}"));
    assert!(bite.is_watertight(), "circular bite must be watertight");
    assert!(
        bite.health_report().is_healthy(),
        "circular bite must be healthy: {:?}",
        bite.health_report().errors
    );
}

#[test]
fn straddling_cylinder_cut_keeps_analytic_bite_wall() {
    let bite = circular_bite_cut()
        .unwrap_or_else(|e| panic!("circular-bite cut must succeed: {e}"));

    // The concave wall must survive as analytic radius-14 cylinder(s) about the
    // vertical cut axis. If the bite collapsed to a box there would be none.
    let bite_walls = bite
        .shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - BITE_R).abs() < 1.0e-3
            )
        })
        .count();
    assert!(
        bite_walls > 0,
        "the circular bite must keep an analytic cylindrical wall, not become a box"
    );

    assert!(
        bite_wall_stays_on_cylinder(&bite),
        "every bite-wall boundary point must lie on the radius-14 cylinder"
    );
}
