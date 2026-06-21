//! Reproduce single-edge fillet behaviour (ZeroCAD's apply_fillet path):
//! fillet_edges on one selected edge must yield a closed, healthy solid.

use openrcad_algo::{fillet_edges, RollingBallError};
use openrcad_foundation::Pnt;
use openrcad_primitives::make_box;
use openrcad_topo::Edge;

fn report(name: &str, r: &Result<openrcad_topo::Solid, RollingBallError>) {
    match r {
        Ok(s) => {
            let hr = s.health_report();
            println!(
                "{name}: OK faces={} watertight={} healthy={} errors={:?}",
                s.face_count(), s.is_watertight(), hr.is_healthy(), hr.errors
            );
        }
        Err(e) => println!("{name}: ERR {e:?}"),
    }
}

#[test]
fn fillet_one_box_edge_closed() {
    // A 10-cube; round the top-front edge (y=0, z=10), running along X.
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(10.0, 0.0, 10.0));
    let r = fillet_edges(&cube, std::slice::from_ref(&edge), 2.0);
    report("fillet_one_box_edge", &r);
    let s = r.expect("box edge fillet should succeed");
    assert!(s.is_watertight(), "filleted box must be watertight");
    assert!(s.health_report().is_healthy(), "filleted box must be healthy");
}

#[test]
fn fillet_thin_plate_edge_closed() {
    // A thin plate (40x40x2); round a top long edge running along X.
    let plate = make_box(&Pnt::origin(), 40.0, 40.0, 2.0);
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 2.0), Pnt::new(40.0, 0.0, 2.0));
    let r = fillet_edges(&plate, std::slice::from_ref(&edge), 0.5);
    report("fillet_thin_plate_edge", &r);
    let s = r.expect("thin-plate edge fillet should succeed");
    assert!(s.is_watertight(), "filleted plate must be watertight");
    assert!(s.health_report().is_healthy(), "filleted plate must be healthy");
}

#[test]
fn fillet_edge_of_union_result() {
    use openrcad_algo::{boolean, BooleanOp};
    // Body produced by a union (how ZeroCAD makes most bodies), then fillet an
    // outer edge of it.
    let a = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let b = make_box(&Pnt::new(10.0, 0.0, 0.0), 20.0, 20.0, 10.0);
    let body = boolean(&a, &b, BooleanOp::Fuse); // 30x20x10 merged box
    println!("union body: faces={} watertight={}", body.face_count(), body.is_watertight());
    // Top-front edge of the merged box: y=0, z=10, x in [0,30].
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(30.0, 0.0, 10.0));
    let r = fillet_edges(&body, std::slice::from_ref(&edge), 2.0);
    report("fillet_edge_of_union", &r);
    // The merged box must expose its full-span top edge (collinear sub-edges
    // collapsed) so endpoint selection finds it instead of SpineNotOnFace.
    let s = r.expect("filleting a boolean-result edge must succeed");
    assert!(s.is_watertight() && s.health_report().is_healthy());
}

#[test]
fn fillet_various_radii_on_plate() {
    let plate = make_box(&Pnt::origin(), 40.0, 40.0, 3.0);
    for radius in [0.5, 1.0, 1.4] {
        let edge = Edge::between_points(Pnt::new(0.0, 0.0, 3.0), Pnt::new(40.0, 0.0, 3.0));
        let r = fillet_edges(&plate, std::slice::from_ref(&edge), radius);
        report(&format!("plate r={radius}"), &r);
        let s = r.unwrap_or_else(|e| panic!("plate r={radius} should fillet: {e:?}"));
        assert!(
            s.is_watertight() && s.health_report().is_healthy(),
            "plate r={radius} must be watertight+healthy"
        );
    }
    // A radius larger than half the 3mm thickness can't fit — must be a clean
    // error, never an Ok with a degenerate (zero-length-edge) solid.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 3.0), Pnt::new(40.0, 0.0, 3.0));
    let too_big = fillet_edges(&plate, std::slice::from_ref(&edge), 2.9);
    report("plate r=2.9 (too big)", &too_big);
    match too_big {
        Err(_) => {}
        Ok(s) => assert!(
            s.is_watertight() && s.health_report().is_healthy(),
            "an accepted oversize fillet must still be valid (got degenerate solid)"
        ),
    }
}
