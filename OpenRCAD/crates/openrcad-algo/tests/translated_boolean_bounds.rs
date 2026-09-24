use openrcad_algo::{boolean_operation, BooleanOp};
use openrcad_foundation::Pnt;

// Analytic plane/plane intersection lines must be trimmed to the operands,
// not an arbitrary interval around the world origin.
#[test]
fn translated_overlapping_boxes_preserve_all_three_boolean_volumes() {
    for origin in [
        [0., 0., 0.],
        [100., -200., 300.],
        [-500., 600., -700.],
        [10_000., 20_000., -30_000.],
    ] {
        for size in [1., 10., 100.] {
            let a = openrcad_primitives::make_box_operation(
                &Pnt::new(origin[0], origin[1], origin[2]),
                size,
                size,
                size,
            )
            .unwrap()
            .value;
            let b = openrcad_primitives::make_box_operation(
                &Pnt::new(
                    origin[0] + size / 2.,
                    origin[1] + size / 2.,
                    origin[2] + size / 2.,
                ),
                size,
                size,
                size,
            )
            .unwrap()
            .value;
            for (op, fraction) in [
                (BooleanOp::Fuse, 1.875),
                (BooleanOp::Cut, 0.875),
                (BooleanOp::Common, 0.125),
            ] {
                let output = boolean_operation(&a, &b, op)
                    .unwrap_or_else(|e| panic!("origin={origin:?} size={size} op={op:?}: {e}"));
                assert!(output.validation.is_valid());
                assert_eq!(output.value.split_disconnected().len(), 1);
                let mesh = openrcad_mesh::tessellate_checked(&output.value, 0.001, 0.05).unwrap();
                let actual = openrcad_mesh::mass_properties(&mesh).unwrap().volume;
                let expected = fraction * size.powi(3);
                assert!(
                    (actual - expected).abs() < expected * 1e-6,
                    "origin={origin:?} size={size} op={op:?}: {actual} != {expected}"
                );
            }
        }
    }
}
