use std::collections::HashSet;
use zerocad_core::{FeatureType, ParametricGraph, SketchCurves};

fn bracket(two_cuts: bool) -> ParametricGraph {
    let mut graph = zerocad_core::read_document_from_slice(
        include_bytes!("fixtures/bracket-face-cut.zcad"),
        &Default::default(),
    )
    .unwrap()
    .document
    .into_evaluator_graph();
    // The user confirmed both intended selections are the circle. The frozen
    // save records the surrounding region for the second feature.
    for node in graph.graph.node_weights_mut() {
        if node.id == "extrude_8" {
            if let FeatureType::Extrude { region_indices, .. } = &mut node.feature {
                *region_indices = vec![1];
            }
        }
    }
    graph.commit_feature_edit("extrude_8").unwrap();
    graph.set_feature_suppressed("extrude_8", !two_cuts);
    graph
}

fn check(graph: &ParametricGraph, two_cuts: bool) {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(bodies.len(), 1);
    let area = std::f64::consts::PI * 5.3_f64.powi(2);
    // The sloping strip's horizontal thickness is linear in z; its disk
    // average is therefore its thickness at the circle center, z=22.9.
    let strip_thickness = (42.4 - 22.9) * 35.0 / 33.0 - (36.4 - 22.9) * 29.0 / 27.0;
    let expected = 15620.0 - area * (strip_thickness + if two_cuts { 7.0 } else { 0.0 });
    let measured = graph.inspect_body("extrude_2").unwrap().volume_mm3.unwrap();
    assert!(
        (measured - expected).abs() < 0.6,
        "{measured} != {expected}"
    );
    let solids = graph.evaluated_kernel_bodies(&HashSet::new()).unwrap();
    assert_eq!(solids[0].1.len(), 1);
    let solid = &solids[0].1[0];
    let inside = |x, y, z| {
        openrcad::algo::boolean::point_in_solid(&openrcad::foundation::Pnt::new(x, y, z), solid)
    };
    assert!(!inside(20.0, 10.0, 22.9), "brace hole must be open");
    assert_eq!(
        inside(2.99, 10.0, 22.9),
        !two_cuts,
        "outward cut must not scar the supporting face"
    );
    assert_eq!(inside(0.0, 10.0, 22.9), !two_cuts);
    assert!(
        inside(0.0, 17.0, 22.9),
        "material outside the circle must remain"
    );
}

#[test]
fn saved_bracket_circular_cuts_preserve_material_and_reopen() {
    for two_cuts in [false, true] {
        let mut graph = bracket(two_cuts);
        check(&graph, two_cuts);
        check(&graph, two_cuts); // warm evaluation
        graph.apply_face_reattach();
        let document = zerocad_core::Document::from_graph(graph, zerocad_core::Unit::Millimeter);
        let bytes = zerocad_core::write_document_to_vec(
            &document,
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        let reopened = zerocad_core::read_document_from_slice(&bytes, &Default::default()).unwrap();
        check(reopened.document.evaluator_graph(), two_cuts);
    }
}

#[test]
fn projected_circle_rims_do_not_create_selectable_slivers() {
    for scale in [0.1_f32, 1.0, 100.0] {
        for reversed in [false, true] {
            let radius = 5.3 * scale;
            let mut drawn = SketchCurves::new();
            drawn.add_circle((0.0, 0.0), radius);
            let mut outline = SketchCurves::new();
            outline.add_rectangle((-10.0 * scale, -10.0 * scale), (10.0 * scale, 10.0 * scale));
            for i in 0..96 {
                let point = |i: usize| {
                    let t = i as f32 * std::f32::consts::TAU / 96.0;
                    (radius * t.cos(), radius * t.sin())
                };
                let (a, b) = (point(i), point((i + 1) % 96));
                outline.add_line(if reversed { b } else { a }, if reversed { a } else { b });
            }
            drawn.extend_face_boundary(&outline);
            assert_eq!(drawn.segments.len(), 4);
            let regions = zerocad_core::detect_regions(&drawn);
            assert_eq!(regions.len(), 2);
            assert_eq!(regions.iter().filter(|r| r.contains((0.0, 0.0))).count(), 1);
            // A genuine chord or incomplete arc is not a duplicate rim.
            let mut partial = SketchCurves::new();
            partial.add_line((radius, 0.0), (-radius, 0.0));
            drawn.extend_face_boundary(&partial);
            assert_eq!(drawn.segments.len(), 5);
        }
    }
}
