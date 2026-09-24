//! Deterministic join break tests with independent analytic material oracles.
mod common;

use common::*;
use zerocad_core::*;

fn block(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, lo: [f32; 3], hi: [f32; 3]) {
    let sketch = format!("{id}_sketch");
    add_sketch_cs(
        g,
        &sketch,
        cs.with_origin(cs.origin.add(cs.n.mul(lo[2]))),
        rect_sketch((lo[0], lo[1]), (hi[0], hi[1])),
    );
    add_extrude(g, id, &sketch, hi[2] - lo[2], ExtrudeMode::NewBody);
}

fn joined(offset: [f32; 3], scale: f32, plane: CoordinateSystem, reverse: bool) {
    let mut g = ParametricGraph::new();
    let lo = offset.map(|x| x * scale);
    let hi = lo.map(|x| x + 10.0 * scale);
    block(&mut g, "a", plane, [0.; 3], [10. * scale; 3]);
    block(&mut g, "b", plane, lo, hi);
    let sources = if reverse {
        vec!["b".into(), "a".into()]
    } else {
        vec!["a".into(), "b".into()]
    };
    g.add_feature(FeatureNode {
        id: "join".into(),
        name: "join".into(),
        feature: FeatureType::BodyJoin { sources },
    });
    g.add_dependency("a", "join");
    g.add_dependency("b", "join");
    let overlap: f64 = offset
        .iter()
        .map(|v| (10. - v.abs()).max(0.) as f64)
        .product();
    let expected = (2000. - overlap) * (scale as f64).powi(3);
    for graph in [&g, &g.clone_document()] {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(
            warnings.is_empty(),
            "offset={offset:?} scale={scale} reverse={reverse}: {warnings:?}"
        );
        assert_eq!(bodies.len(), 1);
        assert_meshes_finite(&bodies);
        let actual = total_volume(&bodies);
        assert!(
            (actual - expected).abs() <= expected * 2e-5,
            "offset={offset:?} scale={scale}: {actual} != {expected}"
        );
        let kernel = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
        assert_eq!(kernel.len(), 1);
        assert_eq!(kernel[0].1.len(), 1);
        let solid = &kernel[0].1[0];
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
        assert_eq!(solid.split_disconnected().len(), 1);
        // Samples avoid all boundaries; compare against the union of two boxes.
        for x in [-2., 2., 7., 12., 17.] {
            for y in [-2., 2., 7., 12., 17.] {
                for z in [2., 7., 12.] {
                    let local = [x, y, z];
                    let inside = local.iter().all(|v| *v > 0. && *v < 10.)
                        || local
                            .iter()
                            .zip(offset)
                            .all(|(v, o)| *v > o && *v < o + 10.);
                    let p = plane
                        .origin
                        .add(plane.u.mul(x * scale))
                        .add(plane.v.mul(y * scale))
                        .add(plane.n.mul(z * scale));
                    assert_eq!(
                        openrcad::algo::boolean::point_in_solid(
                            &openrcad::foundation::Pnt::new(p.x as f64, p.y as f64, p.z as f64),
                            solid
                        ),
                        inside,
                        "sample {local:?}, offset={offset:?}"
                    );
                }
            }
        }
    }
}

macro_rules! cases {
    ($($name:ident: $offset:expr),* $(,)?) => {$(
        mod $name {
            use super::*;
            #[test] fn xy() { joined($offset, 1., CoordinateSystem::XY, false); }
            #[test] fn reversed() { joined($offset, 1., CoordinateSystem::XY, true); }
            #[test] fn xz() { joined($offset, 1., CoordinateSystem::XZ, false); }
            #[test] fn yz() { joined($offset, 1., CoordinateSystem::YZ, false); }
            #[test] fn small() { joined($offset, 0.1, CoordinateSystem::XY, false); }
            #[test] fn large_translated() { joined($offset, 10., CoordinateSystem::XY.with_origin(Vec3::new(100., -200., 300.)), false); }
        }
    )*};
}
cases! {
    identical: [0., 0., 0.],
    partial_x: [5., 0., 0.],
    partial_y: [0., 5., 0.],
    partial_z: [0., 0., 5.],
    corner_overlap: [5., 5., 5.],
    negative_overlap: [-5., -5., -5.],
    shared_x_face: [10., 0., 0.],
    shared_y_face: [0., 10., 0.],
    shared_z_face: [0., 0., 10.],
    partial_face: [10., 5., 0.],
    thin_overlap: [9.99, 0., 0.],
}

#[test]
fn large_corner_overlap_at_origin() {
    joined([5., 5., 5.], 10., CoordinateSystem::XY, false);
}

#[test]
fn translated_corner_overlap_at_original_size() {
    joined(
        [5., 5., 5.],
        1.,
        CoordinateSystem::XY.with_origin(Vec3::new(100., -200., 300.)),
        false,
    );
}

#[test]
fn bridge_join_all_six_source_orders() {
    for sources in [
        ["a", "b", "c"],
        ["a", "c", "b"],
        ["b", "a", "c"],
        ["b", "c", "a"],
        ["c", "a", "b"],
        ["c", "b", "a"],
    ] {
        let mut g = ParametricGraph::new();
        for (id, x) in [("a", 0.), ("b", 20.), ("c", 10.)] {
            block(
                &mut g,
                id,
                CoordinateSystem::XY,
                [x, 0., 0.],
                [x + 10., 10., 10.],
            );
        }
        g.add_feature(FeatureNode {
            id: "join".into(),
            name: "join".into(),
            feature: FeatureType::BodyJoin {
                sources: sources.map(String::from).to_vec(),
            },
        });
        for id in sources {
            g.add_dependency(id, "join");
        }
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "{sources:?}: {warnings:?}");
        assert_eq!(bodies.len(), 1);
        assert!((total_volume(&bodies) - 3000.).abs() < 0.01);
    }
}

#[test]
fn disconnected_or_missing_sources_roll_back_every_body() {
    for offset in [
        [10., 10., 0.],
        [10., 10., 10.],
        [10.1, 0., 0.],
        [100., 0., 0.],
    ] {
        for missing in [false, true] {
            let mut g = ParametricGraph::new();
            block(&mut g, "a", CoordinateSystem::XY, [0.; 3], [10.; 3]);
            block(
                &mut g,
                "b",
                CoordinateSystem::XY,
                offset,
                offset.map(|v| v + 10.),
            );
            let before = g
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap()
                .0;
            let mut sources = vec!["a".into(), "b".into()];
            if missing {
                sources.push("missing".into());
            }
            g.add_feature(FeatureNode {
                id: "join".into(),
                name: "join".into(),
                feature: FeatureType::BodyJoin { sources },
            });
            g.add_dependency("a", "join");
            g.add_dependency("b", "join");
            let (after, warnings) = g
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            assert!(
                warnings.iter().any(|w| w.contains("join")),
                "{offset:?}: {warnings:?}"
            );
            assert_eq!(after.len(), before.len());
            for ((a_id, a), (b_id, b)) in before.iter().zip(&after) {
                assert_eq!(a_id, b_id);
                assert_eq!(a.vertices, b.vertices);
                assert_eq!(a.indices, b.indices);
            }
        }
    }
}
