mod common;
use common::*;
use zerocad_core::*;

#[test]
fn closed_rectangular_spines_preserve_leg_cross_section_in_both_directions() {
    for (width, height) in [(20., 20.), (40., 20.), (20., 60.)] {
        for reverse in [false, true] {
            for radius in [1., 2.] {
                let mut g = ParametricGraph::new();
                let mut points = vec![(0., 0.), (width, 0.), (width, height), (0., height)];
                if reverse {
                    points.reverse();
                }
                let mut path = SketchCurves::new();
                for (a, b) in points.iter().zip(points.iter().cycle().skip(1)).take(4) {
                    path.add_line(*a, *b);
                }
                add_sketch(&mut g, "path", path);
                add_sketch_cs(
                    &mut g,
                    "profile",
                    CoordinateSystem::YZ,
                    circle_sketch((0., 0.), radius),
                );
                add_feature(
                    &mut g,
                    "sweep",
                    FeatureType::Sweep {
                        profile_sketch: "profile".into(),
                        profile_region: 0,
                        path_sketch: "path".into(),
                        mode: ExtrudeMode::NewBody,
                        target: None,
                        total_twist_deg: 0.,
                        total_twist_expr: None,
                        guide: None,
                    },
                    &["profile", "path"],
                );
                for graph in [&g, &g.clone_document()] {
                    let (bodies, warnings) = graph
                        .evaluate_bodies_with_warnings(&Default::default())
                        .unwrap();
                    assert!(
                        warnings.is_empty(),
                        "{width}x{height} r={radius} reversed={reverse}: {warnings:?}"
                    );
                    assert_eq!(bodies.len(), 1);
                    assert_meshes_valid(&bodies);
                    let expected = std::f64::consts::PI
                        * f64::from(radius).powi(2)
                        * 2.
                        * f64::from(width + height);
                    assert_close(
                        total_volume(&bodies),
                        expected,
                        1e-5,
                        0.01,
                        "miter sweep volume",
                    );
                    assert_occupancy(
                        &bodies[0].1,
                        &[
                            [width as f64 / 2., 0., 0.],
                            [width as f64 / 2., height as f64, 0.],
                            [0., height as f64 / 2., 0.],
                            [width as f64, height as f64 / 2., 0.],
                        ],
                        &[[width as f64 / 2., height as f64 / 2., 0.]],
                    );
                }
            }
        }
    }
}
