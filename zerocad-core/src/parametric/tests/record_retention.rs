//! A Join or Cut that the exact sectional path cannot build falls back to the
//! general boolean. The body's prism record must survive that fallback, so
//! later features return to the sectional path instead of inheriting it.
use super::common::*;
use super::*;
use crate::parametric::cut::{FORCE_GENERAL_PATH, SECTIONAL_CUTS};
use openrcad::foundation::Pnt;

fn plane_at(z: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

fn circle_sketch(center: (f32, f32), radius: f32) -> SketchCurves {
    let mut curves = SketchCurves::new();
    curves.add_circle(center, radius);
    curves
}

/// Overlapping parallel joins and cuts; ids follow `extrude_{2 * step + 2}`.
fn stack() -> ParametricGraph {
    use ExtrudeMode::{Cut, Join, NewBody};
    let steps = [
        (rect_sketch((0., 0.), (60., 40.)), 0., 10., NewBody),
        (rect_sketch((40., 10.), (80., 30.)), 10., 8., Join),
        (circle_sketch((70., 20.), 12.), 0., 14., Join),
        (circle_sketch((42., 12.), 7.), -2., 30., Cut),
        (rect_sketch((5., 18.), (85., 22.)), -2., 30., Cut),
        (circle_sketch((60., 30.), 10.), -2., 8., Cut),
        (circle_sketch((36., 2.), 6.), -2., 30., Cut),
    ];
    let mut graph = ParametricGraph::new();
    for (index, (curves, z, depth, mode)) in steps.into_iter().enumerate() {
        let sketch = format!("sketch_{}", 2 * index + 1);
        add_sketch_cs(&mut graph, &sketch, plane_at(z), curves);
        add_extrude(
            &mut graph,
            &format!("extrude_{}", 2 * index + 2),
            &sketch,
            depth,
            mode,
        );
    }
    graph
}

fn evaluate(forced: &[&str]) -> (Vec<openrcad::topo::Solid>, Vec<String>) {
    FORCE_GENERAL_PATH.with(|set| {
        *set.borrow_mut() = forced.iter().map(|id| id.to_string()).collect();
    });
    SECTIONAL_CUTS.with(|cuts| cuts.borrow_mut().clear());
    let graph = stack();
    let (_, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    let parts = graph
        .evaluated_kernel_bodies(&Default::default())
        .unwrap()
        .into_iter()
        .flat_map(|(_, parts)| parts)
        .collect();
    FORCE_GENERAL_PATH.with(|set| set.borrow_mut().clear());
    (parts, SECTIONAL_CUTS.with(|cuts| cuts.borrow().clone()))
}

#[test]
fn general_path_join_and_cut_keep_later_cuts_sectional() {
    let (reference, reference_cuts) = evaluate(&[]);
    assert_eq!(
        reference_cuts,
        ["extrude_8", "extrude_10", "extrude_12", "extrude_14"],
        "every cut should be sectional when nothing is forced"
    );

    // J2 (extrude_6) and C2 (extrude_10) take the general boolean.
    let (forced, forced_cuts) = evaluate(&["extrude_6", "extrude_10"]);
    assert_eq!(
        forced_cuts,
        ["extrude_8", "extrude_12", "extrude_14"],
        "cuts after a general-path join or cut must return to the sectional path"
    );

    assert_eq!(forced.len(), reference.len());
    let policy = Default::default();
    for part in &forced {
        assert!(part.is_watertight());
        assert!(part.validate_strict_with_policy(&policy).is_ok());
    }
    // Both evaluations model the same material; the lattice offsets keep the
    // samples off the integer-aligned profile boundaries.
    for i in 0..30 {
        for j in 0..18 {
            for k in 0..8 {
                let point = Pnt::new(
                    -3.37 + 3.11 * f64::from(i),
                    -4.29 + 3.07 * f64::from(j),
                    -1.13 + 2.53 * f64::from(k),
                );
                let inside = |parts: &[openrcad::topo::Solid]| {
                    parts
                        .iter()
                        .any(|part| openrcad::algo::boolean::point_in_solid(&point, part))
                };
                assert_eq!(
                    inside(&forced),
                    inside(&reference),
                    "material differs at {point:?}"
                );
            }
        }
    }
}
