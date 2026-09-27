use openrcad::foundation::Pnt;
use zerocad_core::mock_kernel::KernelSolid;
use zerocad_core::*;

#[test]
fn slots_cross_existing_bores_at_multiple_sizes_and_positions() {
    use openrcad::foundation::{Ax2, Dir};
    use openrcad::primitives::{
        make_box_operation as make_box, make_cylinder_operation as make_cylinder,
    };
    use zerocad_core::mock_kernel::{difference, difference_bodies};

    for (scale, offset) in [(1.0, 0.0), (0.5, 37.0), (2.0, -123.0)] {
        let p = |x, y, z| Pnt::new(offset + scale * x, offset + scale * y, offset + scale * z);
        let block = make_box(&p(0.0, 0.0, 0.0), 20.0 * scale, 20.0 * scale, 10.0 * scale)
            .unwrap()
            .value;
        let drill = make_cylinder(
            &Ax2::new(p(10.0, 10.0, -1.0), Dir::dz()),
            3.0 * scale,
            12.0 * scale,
        )
        .unwrap()
        .value;
        let bored = difference(&block, &drill).expect("initial bore");
        for width in [2.0_f64, 8.0] {
            for height in [5.0_f64, 10.0] {
                let slot = make_box(
                    &p(10.0 - width / 2.0, -1.0, -1.0),
                    width * scale,
                    22.0 * scale,
                    (height + 1.0) * scale,
                )
                .unwrap()
                .value;
                let parts = difference_bodies(&bored, &slot).expect("slot through bore");
                let half = (width / 2.0).min(3.0);
                let strip = 2.0 * (half * (9.0 - half * half).sqrt() + 9.0 * (half / 3.0).asin());
                let expected =
                    (4000.0 - std::f64::consts::PI * 90.0 - height * (20.0 * width - strip))
                        * scale.powi(3);
                let mut volume = 0.0;
                for part in &parts {
                    assert!(part.is_watertight());
                    assert!(part.health_report().is_healthy());
                    part.validate_strict_with_policy(&Default::default())
                        .unwrap();
                    let mesh =
                        openrcad::mesh::tessellate_checked(part, 0.002 * scale, 0.1).unwrap();
                    volume += openrcad::mesh::mass_properties(&mesh).unwrap().volume;
                }
                assert!(
                    (volume - expected).abs() < 0.25 * scale.powi(3),
                    "scale={scale} width={width} height={height}: {volume} != {expected}"
                );
                for x in [1.0_f64, 6.5, 8.5, 10.2, 11.5, 13.5, 19.0] {
                    for y in [1.0_f64, 8.0, 10.3, 12.0, 19.0] {
                        for z in [1.0, 4.5, 5.5, 9.0] {
                            let in_bore = (x - 10.0).powi(2) + (y - 10.0).powi(2) < 9.0;
                            let in_slot = (x - 10.0).abs() < width / 2.0 && z < height;
                            assert_eq!(
                                parts
                                    .iter()
                                    .any(|solid| openrcad::algo::boolean::point_in_solid(
                                        &p(x, y, z),
                                        solid
                                    )),
                                !in_bore && !in_slot,
                                "scale={scale} width={width} height={height} at ({x},{y},{z})"
                            );
                        }
                    }
                }
            }
        }
    }
}

fn document(bytes: &[u8]) -> ParametricGraph {
    read_document_from_slice(bytes, &Default::default())
        .unwrap()
        .document
        .into_evaluator_graph()
        .clone_document()
}

fn checked(graph: &ParametricGraph) -> (KernelSolid, f64) {
    let (meshes, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(meshes.len(), 1);
    let mut bodies = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].1.len(), 1);
    let solid = bodies.remove(0).1.remove(0);
    assert!(solid.is_watertight());
    assert!(solid.health_report().is_healthy());
    solid
        .validate_strict_with_policy(&Default::default())
        .unwrap();
    let mesh = openrcad::mesh::tessellate_checked(&solid, 0.002, 0.1).unwrap();
    let volume = openrcad::mesh::mass_properties(&mesh).unwrap().volume;
    (solid, volume)
}

#[test]
fn cut_through_existing_cuts_preserves_material_and_reopens() {
    let graph = document(include_bytes!("fixtures/cut-through-error.zcad"));
    let mut before = graph.clone_document();
    before.set_feature_suppressed("extrude_37", true);
    let (source, initial_volume) = checked(&before);
    // The slot removes a 10 x 25 x 4 slab, less the existing cylindrical
    // void inside the strip |x| < 5. Integrate that circular strip exactly.
    let radius = 5.15_f64;
    let half_width = 5.0_f64;
    let circular_strip = 2.0
        * (half_width * (radius.powi(2) - half_width.powi(2)).sqrt()
            + radius.powi(2) * (half_width / radius).asin());
    let expected = initial_volume - 4.0 * (250.0 - circular_strip);
    let bytes = write_document_to_vec(
        &Document::from_graph(graph.clone_document(), Unit::Millimeter),
        &Default::default(),
        &Default::default(),
    )
    .unwrap();
    for candidate in [graph, document(&bytes)] {
        for _ in 0..2 {
            let (solid, volume) = checked(&candidate);
            assert!((volume - expected).abs() < 0.2, "{volume} != {expected}");
            // Compare solid membership against an independent source-minus-box
            // predicate, including both old bores, the slot and nearby walls.
            for x in [-15.0_f64, -5.1, -4.9, 0.2, 4.9, 5.1, 15.0] {
                for y in [1.0, 7.0, 10.0, 12.0, 16.0, 19.0, 24.0] {
                    for z in [-28.0, -21.0, 0.0, 19.6, 21.0, 23.3, 23.6, 28.0] {
                        let p = Pnt::new(x, y, z);
                        let removed = x.abs() < 5.0 && z > 15.45 && z < 23.45;
                        let expected_material =
                            openrcad::algo::boolean::point_in_solid(&p, &source) && !removed;
                        assert_eq!(
                            openrcad::algo::boolean::point_in_solid(&p, &solid),
                            expected_material,
                            "material at {p:?}"
                        );
                    }
                }
            }
        }
    }
}
