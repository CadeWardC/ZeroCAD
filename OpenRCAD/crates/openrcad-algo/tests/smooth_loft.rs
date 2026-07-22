use openrcad_algo::{
    skin_smooth_section_loops_operation_with_policy, ModelingOperationError, SmoothCurveSpan,
    SmoothLoftError, SmoothSectionLoops, SmoothSpanKind,
};
use openrcad_foundation::{Pnt, ToleranceContext, TolerancePolicy};
use openrcad_geom::{BSplineCurve, Curve, GeomSurface, Surface};

fn line(first: Pnt, last: Pnt, provenance: u64) -> SmoothCurveSpan {
    SmoothCurveSpan::new(
        BSplineCurve::new(1, vec![first, last], None, vec![0.0, 1.0], vec![2, 2]),
        0.0,
        1.0,
        provenance,
        SmoothSpanKind::Line,
    )
}

fn split_quadratic_line(first: Pnt, last: Pnt, provenance: u64) -> SmoothCurveSpan {
    let direction = last - first;
    SmoothCurveSpan::new(
        BSplineCurve::new(
            2,
            [0.0, 0.25, 0.5, 0.75, 1.0]
                .into_iter()
                .map(|fraction| first + direction * fraction)
                .collect(),
            Some(vec![2.0; 5]),
            vec![0.0, 0.5, 1.0],
            vec![3, 2, 3],
        ),
        0.0,
        1.0,
        provenance,
        SmoothSpanKind::Line,
    )
}

fn weighted_line(first: Pnt, last: Pnt, provenance: u64, weight: f64) -> SmoothCurveSpan {
    SmoothCurveSpan::new(
        BSplineCurve::new(
            1,
            vec![first, last],
            Some(vec![weight, weight]),
            vec![0.0, 1.0],
            vec![2, 2],
        ),
        0.0,
        1.0,
        provenance,
        SmoothSpanKind::Line,
    )
}

fn rectangle(origin: [f64; 3], width: f64, height: f64) -> Vec<SmoothCurveSpan> {
    oriented_rectangle(origin, width, height, 0.0)
}

fn oriented_rectangle(
    origin: [f64; 3],
    width: f64,
    height: f64,
    angle_deg: f64,
) -> Vec<SmoothCurveSpan> {
    let [x, y, z] = origin;
    let angle = angle_deg.to_radians();
    let x_axis = [angle.cos(), angle.sin()];
    let y_axis = [-angle.sin(), angle.cos()];
    let point = |u: f64, v: f64| {
        Pnt::new(
            x + u * x_axis[0] + v * y_axis[0],
            y + u * x_axis[1] + v * y_axis[1],
            z,
        )
    };
    let points = [
        point(-width * 0.5, -height * 0.5),
        point(width * 0.5, -height * 0.5),
        point(width * 0.5, height * 0.5),
        point(-width * 0.5, height * 0.5),
    ];
    (0..4)
        .map(|index| line(points[index], points[(index + 1) % 4], index as u64))
        .collect()
}

fn reverse_loop(mut spans: Vec<SmoothCurveSpan>) -> Vec<SmoothCurveSpan> {
    spans.reverse();
    for span in &mut spans {
        core::mem::swap(&mut span.first, &mut span.last);
    }
    spans
}

fn sections(count: usize, scale: f64, world: [f64; 3]) -> Vec<SmoothSectionLoops> {
    (0..count)
        .map(|section| {
            let fraction = section as f64 / (count - 1) as f64;
            SmoothSectionLoops {
                outer: rectangle(
                    [
                        world[0] + scale * fraction * 0.35,
                        world[1],
                        world[2] + scale * fraction * 3.0,
                    ],
                    scale * (2.0 + fraction),
                    scale * (1.5 + 0.25 * fraction),
                ),
                holes: Vec::new(),
            }
        })
        .collect()
}

#[test]
fn smooth_loft_interpolates_two_through_five_exact_sections() {
    let policy = TolerancePolicy::STANDARD;
    for count in 2..=5 {
        let input = sections(count, 1.0, [0.0; 3]);
        let operation = skin_smooth_section_loops_operation_with_policy(&input, &policy)
            .unwrap_or_else(|error| panic!("{count}-section Smooth Loft failed: {error}"));
        assert!(operation.validation.is_valid());
        assert!(operation.value.validate_strict_with_policy(&policy).is_ok());
        assert!(operation
            .history
            .coverage_for_solid(&operation.value)
            .is_complete());

        let surfaces = operation
            .value
            .shell()
            .faces()
            .into_iter()
            .filter_map(|face| match face.surface() {
                Some(GeomSurface::BSpline(surface)) => Some(surface.clone()),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(surfaces.len(), 4, "one exact surface per source span");

        // Chord lengths are equal for this affine family, so its section
        // parameters are evenly spaced. Every source span must lie exactly on
        // its generated surface at that parameter.
        for (section_index, section) in input.iter().enumerate() {
            let v = section_index as f64 / (count - 1) as f64;
            for (span_index, span) in section.outer.iter().enumerate() {
                for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
                    let expected = span.curve.point(u);
                    let actual = surfaces[span_index].point(u, v);
                    assert!(
                        actual.distance(&expected) <= policy.pcurve_consistency,
                        "section={section_index} span={span_index} u={u}: {}",
                        actual.distance(&expected)
                    );
                }
            }
        }
    }
}

#[test]
fn smooth_loft_scale_and_far_origin_sweep_is_valid() {
    let policy = TolerancePolicy::STANDARD;
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for world in [[0.0; 3], [1.0e6, -2.0e6, 3.0e6]] {
            let input = sections(4, scale, world);
            let mut bounds = openrcad_foundation::BndBox::new();
            for span in &input[0].outer {
                bounds.add(&span.curve.point(span.first));
            }
            let context = ToleranceContext::derive(&policy, &[bounds], Some(scale), 1.0).unwrap();
            let operation = skin_smooth_section_loops_operation_with_policy(&input, &policy)
                .unwrap_or_else(|error| {
                    panic!("scale={scale} world={world:?} Smooth Loft failed: {error}")
                });
            assert!(operation.value.is_watertight_with_policy(&context.policy));
            assert!(operation
                .value
                .validate_strict_with_policy(&context.policy)
                .is_ok());
        }
    }
    for (aspect, angle, world) in [
        (1.0e-3_f64, 37.0, [0.0; 3]),
        (1.0, 90.0, [1.0e6, -2.0e6, 3.0e6]),
        (1.0e3, 0.0, [0.0; 3]),
    ] {
        let root = aspect.sqrt();
        let input = (0..4)
            .map(|section| {
                let fraction = section as f64 / 3.0;
                SmoothSectionLoops {
                    outer: oriented_rectangle(
                        [
                            world[0] + fraction * 0.25,
                            world[1],
                            world[2] + fraction * 4.0,
                        ],
                        4.0 * root,
                        3.0 / root,
                        angle,
                    ),
                    holes: vec![],
                }
            })
            .collect::<Vec<_>>();
        let operation = skin_smooth_section_loops_operation_with_policy(&input, &policy)
            .unwrap_or_else(|error| {
                panic!("aspect={aspect} angle={angle} world={world:?} failed: {error}")
            });
        assert!(operation.validation.is_valid());
        assert!(operation.value.validate_strict_with_policy(&policy).is_ok());
    }
}

#[test]
fn smooth_loft_rejects_topology_change_before_building_a_candidate() {
    let mut input = sections(3, 1.0, [0.0; 3]);
    input[1].outer.pop();
    let error = skin_smooth_section_loops_operation_with_policy(&input, &TolerancePolicy::STANDARD)
        .unwrap_err();
    assert!(matches!(
        error,
        ModelingOperationError::SmoothLoft(SmoothLoftError::OpenLoop { .. })
            | ModelingOperationError::SmoothLoft(SmoothLoftError::SpanCountMismatch { .. })
    ));
    let ModelingOperationError::SmoothLoft(error) = error else {
        unreachable!()
    };
    assert_eq!(error.diagnostic_code(), "loft.section_mismatch");
}

#[test]
fn smooth_loft_matches_reordered_holes_uniquely() {
    let at = |z: f64, reverse_holes: bool| {
        let left = reverse_loop(rectangle([-2.0, 0.0, z], 1.0, 1.0));
        let right = reverse_loop(rectangle([2.0, 0.0, z], 1.5, 0.75));
        SmoothSectionLoops {
            outer: rectangle([0.0, 0.0, z], 8.0, 5.0),
            holes: if reverse_holes {
                vec![right, left]
            } else {
                vec![left, right]
            },
        }
    };
    let operation = skin_smooth_section_loops_operation_with_policy(
        &[at(0.0, false), at(4.0, true)],
        &TolerancePolicy::STANDARD,
    )
    .expect("uniquely shaped holes must match independently of input order");
    assert!(operation.validation.is_valid());
    assert!(operation
        .value
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .is_ok());
    assert_eq!(
        operation
            .value
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::BSpline(_))))
            .count(),
        12,
        "outer plus two four-span hole skins"
    );
}

#[test]
fn smooth_loft_unifies_degree_knots_and_non_unit_weights_exactly() {
    let bottom = rectangle([0.0, 0.0, 0.0], 4.0, 3.0);
    let top_linear = rectangle([0.5, 0.0, 5.0], 6.0, 2.0);
    let top = top_linear
        .iter()
        .map(|span| {
            split_quadratic_line(
                span.curve.point(span.first),
                span.curve.point(span.last),
                span.provenance,
            )
        })
        .collect();
    let operation = skin_smooth_section_loops_operation_with_policy(
        &[
            SmoothSectionLoops {
                outer: bottom,
                holes: vec![],
            },
            SmoothSectionLoops {
                outer: top,
                holes: vec![],
            },
        ],
        &TolerancePolicy::STANDARD,
    )
    .expect("exact knot insertion and homogeneous degree elevation");
    assert!(operation.validation.is_valid());
    assert!(operation
        .value
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .is_ok());
    let lateral = operation
        .value
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::BSpline(surface)) => Some(surface.clone()),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(
        lateral.len(),
        8,
        "each source line is split at the union knot"
    );
    assert!(lateral.iter().all(|surface| surface.u_degree() == 2));
}

#[test]
fn smooth_loft_rejects_an_interpolation_with_non_positive_homogeneous_weights() {
    let input = [1.0, 100.0, 1.0, 100.0]
        .into_iter()
        .enumerate()
        .map(|(section, weight)| {
            let lines = rectangle([0.0, 0.0, section as f64], 4.0, 3.0);
            SmoothSectionLoops {
                outer: lines
                    .iter()
                    .map(|span| {
                        weighted_line(
                            span.curve.point(span.first),
                            span.curve.point(span.last),
                            span.provenance,
                            weight,
                        )
                    })
                    .collect(),
                holes: vec![],
            }
        })
        .collect::<Vec<_>>();
    let error = skin_smooth_section_loops_operation_with_policy(&input, &TolerancePolicy::STANDARD)
        .expect_err("a negative interpolated homogeneous weight must reject");
    assert!(matches!(
        error,
        ModelingOperationError::SmoothLoft(SmoothLoftError::NonPositiveWeight { .. })
    ));
}

#[test]
fn smooth_loft_rejects_coincident_and_reversed_section_order() {
    let coincident = vec![
        SmoothSectionLoops {
            outer: rectangle([0.0, 0.0, 0.0], 4.0, 3.0),
            holes: vec![],
        },
        SmoothSectionLoops {
            outer: rectangle([0.0, 0.0, 0.0], 4.0, 3.0),
            holes: vec![],
        },
    ];
    assert!(matches!(
        skin_smooth_section_loops_operation_with_policy(&coincident, &TolerancePolicy::STANDARD),
        Err(ModelingOperationError::SmoothLoft(
            SmoothLoftError::CoincidentSections { section: 1 }
        ))
    ));

    let reversed = [0.0, 3.0, 2.0]
        .into_iter()
        .map(|z| SmoothSectionLoops {
            outer: rectangle([0.0, 0.0, z], 4.0, 3.0),
            holes: vec![],
        })
        .collect::<Vec<_>>();
    assert!(matches!(
        skin_smooth_section_loops_operation_with_policy(&reversed, &TolerancePolicy::STANDARD),
        Err(ModelingOperationError::SmoothLoft(
            SmoothLoftError::ReversedSectionOrder { section: 2 }
        ))
    ));
}

#[test]
fn smooth_loft_rejects_a_twisted_self_intersecting_correspondence() {
    let input = [
        SmoothSectionLoops {
            outer: oriented_rectangle([0.0, 0.0, 0.0], 4.0, 3.0, 0.0),
            holes: vec![],
        },
        SmoothSectionLoops {
            outer: oriented_rectangle([0.0, 0.0, 4.0], 4.0, 3.0, 180.0),
            holes: vec![],
        },
    ];
    assert!(matches!(
        skin_smooth_section_loops_operation_with_policy(&input, &TolerancePolicy::STANDARD),
        Err(ModelingOperationError::SmoothLoft(
            SmoothLoftError::SelfIntersectionUnresolved { .. }
        ))
    ));
}
