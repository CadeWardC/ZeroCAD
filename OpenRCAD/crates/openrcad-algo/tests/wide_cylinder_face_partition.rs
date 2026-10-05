//! A cylinder face spanning more than half a turn must still split cleanly.
//! Found by the ZeroCAD mixed-axis join/cut census: a round side tunnel whose
//! lower part opens into a flat-roofed rectangular tunnel leaves its upper wall
//! as one cylinder face about 245 degrees wide. A vertical slot across both
//! tunnels must cut that wall in two places. Face partition unwrapped the
//! wall's UV loop to within pi of one boundary vertex, which cut the face's
//! angular range in two, inverted the loop's winding, and kept the unsplit
//! wall as the only "interior" region, so the cut came back with free edges.
use openrcad_algo::{boolean::point_in_solid, boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};
use openrcad_topo::Solid;

fn make_box(corner: &Pnt, dx: f64, dy: f64, dz: f64) -> Solid {
    make_box_operation(corner, dx, dy, dz).expect("box").value
}

fn make_cylinder(axis: &Ax2, radius: f64, height: f64) -> Solid {
    make_cylinder_operation(axis, radius, height)
        .expect("cylinder")
        .value
}

#[test]
fn slot_across_a_tunnel_opening_into_a_flat_roofed_tunnel() {
    let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 50.473, 33.802, 11.645);
    // Rectangular tunnel along Y; its roof is at z = 5.343.
    let flat = make_box(
        &Pnt::new(6.653, -2.0, 1.224),
        30.766 - 6.653,
        53.802,
        5.343 - 1.224,
    );
    // Round tunnel along Y, blind at y = 19.519, dipping below that roof.
    let round = make_cylinder(
        &Ax2::new(Pnt::new(11.985, -2.0, 7.361), Dir::dy()),
        3.734,
        21.519,
    );
    // Vertical slot across both tunnels.
    let slot = make_box(
        &Pnt::new(7.151, 5.966, -2.0),
        24.162 - 7.151,
        11.623 - 5.966,
        31.645,
    );

    let mut solid = block;
    for (name, tool) in [
        ("flat tunnel", &flat),
        ("round tunnel", &round),
        ("slot", &slot),
    ] {
        solid = boolean_operation(&solid, tool, BooleanOp::Cut)
            .map(|result| result.value)
            .unwrap_or_else(|error| panic!("{name} cut failed: {error:?}"));
    }
    solid
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("the slotted part is a valid solid");

    // The slot removes everything between its walls, including the round
    // tunnel's roof; material remains beside it and in the tunnel walls.
    assert!(
        !point_in_solid(&Pnt::new(12.0, 8.8, 9.0), &solid),
        "slot is empty"
    );
    assert!(
        !point_in_solid(&Pnt::new(12.0, 3.0, 7.4), &solid),
        "tunnel is empty"
    );
    assert!(
        point_in_solid(&Pnt::new(12.0, 3.0, 11.3), &solid),
        "tunnel roof remains"
    );
    assert!(
        point_in_solid(&Pnt::new(40.0, 8.8, 9.0), &solid),
        "block beside the slot"
    );
}
