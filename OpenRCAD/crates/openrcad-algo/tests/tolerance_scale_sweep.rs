//! Property sweep guarding operation-local tolerances. This belongs beside the
//! unit-floor removal so scale regressions cannot be separated from that edit.

use openrcad_algo::{boolean_operation_with_policy, BooleanOp};
use openrcad_foundation::{Pnt, TolerancePolicy};
use openrcad_primitives::make_box_operation_with_policy;
use openrcad_topo::Solid;

fn assert_closed(label: &str, solid: &Solid) {
    assert!(solid.validate().is_ok(), "{label}: invalid topology");
    assert!(solid.is_watertight(), "{label}: non-watertight result");
    assert!(
        solid.health_report().is_healthy(),
        "{label}: unhealthy result"
    );
}

fn close_coordinate(actual: f64, expected: f64, scale: f64) -> bool {
    let arithmetic = actual.abs().max(expected.abs()) * f64::EPSILON * 64.0;
    (actual - expected).abs() <= arithmetic.max(scale * 1.0e-6).max(1.0e-12)
}

fn assert_bounds(
    label: &str,
    solid: &Solid,
    expected_lo: [f64; 3],
    expected_hi: [f64; 3],
    scale: f64,
) {
    let (lo, hi) = solid.bounding_box().corners().expect("result bounds");
    for (axis, (actual, expected)) in [lo.x(), lo.y(), lo.z()]
        .into_iter()
        .zip(expected_lo)
        .enumerate()
    {
        assert!(
            close_coordinate(actual, expected, scale),
            "{label}: lower axis {axis}: {actual} != {expected}"
        );
    }
    for (axis, (actual, expected)) in [hi.x(), hi.y(), hi.z()]
        .into_iter()
        .zip(expected_hi)
        .enumerate()
    {
        assert!(
            close_coordinate(actual, expected, scale),
            "{label}: upper axis {axis}: {actual} != {expected}"
        );
    }
}

fn exercise_case(scale: f64, aspect: f64, origin: [f64; 3]) {
    let policy = &TolerancePolicy::STANDARD;
    let [x, y, z] = origin;
    let (dx, dy, dz) = (8.0 * scale * aspect, 4.0 * scale, 2.0 * scale);
    let source = make_box_operation_with_policy(&Pnt::new(x, y, z), dx, dy, dz, policy)
        .expect("source box")
        .value;
    let tool =
        make_box_operation_with_policy(&Pnt::new(x + dx * 0.5, y, z), dx * 0.75, dy, dz, policy)
            .expect("tool box")
            .value;
    let label = format!("scale={scale:e}, aspect={aspect}, origin={origin:?}");

    let fused = boolean_operation_with_policy(&source, &tool, BooleanOp::Fuse, policy)
        .unwrap_or_else(|error| panic!("{label}: fuse failed: {error}"))
        .value;
    assert_closed(&format!("{label}: fuse"), &fused);
    assert_bounds(
        &format!("{label}: fuse"),
        &fused,
        [x, y, z],
        [x + dx * 1.25, y + dy, z + dz],
        scale,
    );

    let cut = boolean_operation_with_policy(&source, &tool, BooleanOp::Cut, policy)
        .unwrap_or_else(|error| panic!("{label}: cut failed: {error}"))
        .value;
    assert_closed(&format!("{label}: cut"), &cut);
    assert_bounds(
        &format!("{label}: cut"),
        &cut,
        [x, y, z],
        [x + dx * 0.5, y + dy, z + dz],
        scale,
    );
}

#[test]
fn booleans_hold_across_scale_aspect_and_far_origin_sweeps() {
    for scale in [1.0e-3, 1.0e-1, 1.0, 10.0, 1.0e3] {
        for aspect in [1.0, 10.0, 100.0] {
            exercise_case(scale, aspect, [0.0, 0.0, 0.0]);
        }
    }

    // Far-origin arithmetic is most exposed at the smallest scale; retain one
    // large-scale endpoint as a check that conditioning does not overcorrect.
    for scale in [1.0e-3, 1.0e3] {
        for aspect in [1.0, 10.0, 100.0] {
            exercise_case(scale, aspect, [1.0e9, -1.0e9, 5.0e8]);
        }
    }
}
