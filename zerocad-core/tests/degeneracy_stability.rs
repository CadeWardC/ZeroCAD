//! Metamorphic gate for degenerate sketch configurations.
//!
//! Exact tangency and exact collinearity are measure-zero in random geometry
//! but are the *most likely* input a CAD user produces, because they snap,
//! constrain, and type round numbers. A circle of radius 0.4 inside a band
//! 0.8 tall is not a coincidence — it is the intent.
//!
//! Two bugs reached a user through this path: a circle tangent to a line tied
//! the arrangement's half-edge sort and fused two faces, and a circle tangent
//! to both edges of a band produced a cusped face the extruder silently
//! dropped. Neither was found by an example-based test, because nobody
//! enumerates degeneracies exhaustively.
//!
//! So the gate here is metamorphic rather than example-based: sweep a
//! parameter *through* the degenerate value and assert an invariant that must
//! hold at every step, including exactly on the knife edge.

use zerocad_core::sketch::{detect_regions, Circle, SketchCurves};
use zerocad_core::{CoordinateSystem, Region};

const DEPTH: f32 = 1.5;

/// Areas below this are the arrangement's own hairline bookkeeping, not faces
/// a user drew or could select meaningfully.
const SUBSTANTIAL_AREA: f32 = 1.0e-3;

/// Offsets swept through each fixture's degenerate value. The extremes are
/// comfortably non-degenerate; the middle is exactly on it.
const OFFSETS: [f32; 7] = [-1.0e-2, -1.0e-3, -1.0e-4, 0.0, 1.0e-4, 1.0e-3, 1.0e-2];

fn rectangle(curves: &mut SketchCurves, x0: f32, y0: f32, x1: f32, y1: f32) {
    curves.add_line((x0, y0), (x1, y0));
    curves.add_line((x1, y0), (x1, y1));
    curves.add_line((x1, y1), (x0, y1));
    curves.add_line((x0, y1), (x0, y0));
}

/// A band 0.8 tall with a circle of radius 0.4 centred on its left edge, so at
/// `offset == 0` the circle is exactly tangent to BOTH band edges. This is the
/// reported case, reduced.
fn tangent_circle_in_band(offset: f32) -> SketchCurves {
    let mut curves = SketchCurves::default();
    rectangle(&mut curves, -13.2, -13.4, 8.2, -6.9);
    rectangle(&mut curves, -10.2, -6.9, 8.2, -6.1);
    rectangle(&mut curves, -10.3, -6.1, 8.2, -2.9);
    curves.circles.push(Circle {
        center: (-10.2, -6.5),
        radius: 0.4 + offset,
    });
    curves
}

/// A circle resting on the shared edge of two stacked rectangles: tangent to
/// one line rather than two.
fn circle_tangent_to_shared_edge(offset: f32) -> SketchCurves {
    let mut curves = SketchCurves::default();
    rectangle(&mut curves, -3.0, -6.5, 40.0, 0.0);
    rectangle(&mut curves, 0.0, 0.0, 37.0, 0.4);
    curves.circles.push(Circle {
        center: (0.0, 0.4),
        radius: 0.4 + offset,
    });
    curves
}

/// Control: a circle that genuinely crosses the edge rather than touching it.
fn circle_crossing_an_edge(offset: f32) -> SketchCurves {
    let mut curves = SketchCurves::default();
    rectangle(&mut curves, -10.0, -5.0, 10.0, 5.0);
    curves.circles.push(Circle {
        center: (0.0, 5.0),
        radius: 2.0 + offset,
    });
    curves
}

/// The reported case, transcribed from the user's document: three stacked
/// rectangles, a circle tangent to both edges of the middle band, and two
/// stray segments left lying along the shared edges. The band's face is
/// pinched to zero width where the circle touches, so it cannot be swept.
///
/// The coordinates are the document's exact `f32` values, to nine significant
/// digits, and they have to be. Rounding them to the tidy numbers the user
/// typed (−6.9 rather than −6.900001049) moves the geometry by one micron,
/// which is enough to step off the knife edge and make every face build. That
/// sensitivity is the bug's whole character: the failure lives at one exact
/// configuration and vanishes under any perturbation.
fn reported_bug_case() -> SketchCurves {
    let mut curves = SketchCurves::default();
    curves.add_line((-13.2, -13.4), (8.2, -13.4));
    curves.add_line((8.2, -13.4), (8.2, -6.899_999_6));
    curves.add_line((8.2, -6.899_999_6), (-13.2, -6.899_999_6));
    curves.add_line((-13.2, -6.899_999_6), (-13.2, -13.4));
    curves.add_line((8.2, -6.899_999_6), (-10.2, -6.900_001));
    curves.add_line((-10.2, -6.900_001), (-10.2, -6.500_001));
    curves.add_line((-10.2, -6.900_001), (8.2, -6.900_001));
    curves.add_line((8.2, -6.900_001), (8.2, -6.100_001));
    curves.add_line((8.2, -6.100_001), (-10.2, -6.100_001));
    curves.add_line((-10.2, -6.100_001), (-10.2, -6.900_001));
    curves.add_line((8.2, -6.100_001), (-10.3, -6.100_001));
    curves.add_line((-10.3, -6.100_001), (-10.3, -2.900_000_8));
    curves.add_line((-10.3, -2.900_000_8), (8.2, -2.900_000_8));
    curves.add_line((8.2, -2.900_000_8), (8.2, -6.100_001));
    curves.circles.push(Circle {
        center: (-10.2, -6.500_001),
        radius: 0.4,
    });
    curves
}

/// Control: no curves at all, just rectangles sharing an edge.
fn stacked_rectangles(offset: f32) -> SketchCurves {
    let mut curves = SketchCurves::default();
    rectangle(&mut curves, 0.0, 0.0, 20.0, 10.0);
    rectangle(&mut curves, 2.0, 10.0 + offset, 18.0, 16.0);
    curves
}

type Fixture = (&'static str, fn(f32) -> SketchCurves);

const FIXTURES: [Fixture; 4] = [
    ("tangent circle in a band", tangent_circle_in_band),
    (
        "circle tangent to a shared edge",
        circle_tangent_to_shared_edge,
    ),
    ("circle crossing an edge", circle_crossing_an_edge),
    ("stacked rectangles", stacked_rectangles),
];

/// Fixtures whose face set must be correct at every offset, including exactly
/// on the degenerate value. `tangent_circle_in_band` earned its place here
/// when the arrangement's ±π-seam bug was fixed: before that, its four-way
/// contact vertices lost the top rectangle at exact tangency.
const STABLE_FIXTURES: [Fixture; 4] = [
    ("tangent circle in a band", tangent_circle_in_band),
    (
        "circle tangent to a shared edge",
        circle_tangent_to_shared_edge,
    ),
    ("circle crossing an edge", circle_crossing_an_edge),
    ("stacked rectangles", stacked_rectangles),
];

fn builds(region: &Region) -> bool {
    zerocad_core::mock_kernel::extruded_sketch_region_solid(
        region,
        DEPTH,
        &CoordinateSystem::XY,
        &[],
    )
    .is_some()
}

/// The invariant that matters most: a face the user can see and select must
/// either become a solid, or carry a machine-readable reason why it cannot.
/// Silence is the one outcome that is never acceptable — it puts a hole in the
/// model with nothing to explain it, which is exactly how both reported bugs
/// presented.
#[test]
fn every_substantial_region_builds_or_explains_itself() {
    for (name, build) in FIXTURES {
        for offset in OFFSETS {
            let regions = detect_regions(&build(offset));
            for (index, region) in regions.iter().enumerate() {
                if region.area <= SUBSTANTIAL_AREA {
                    continue;
                }
                assert!(
                    builds(region) || region.degeneracy().is_some(),
                    "{name} @ offset {offset:+e}: region {index} (area {:.6}) neither built \
                     nor reported a degeneracy — it would be dropped in silence",
                    region.area
                );
            }
        }
    }
}

/// Total face area must move continuously as a fixture is swept through its
/// degenerate value. A discontinuity means faces fused, vanished, or doubled
/// at the knife edge — the signature of a tie in a geometric predicate.
fn area_sweep(build: fn(f32) -> SketchCurves) -> Vec<f32> {
    OFFSETS
        .iter()
        .map(|offset| {
            detect_regions(&build(*offset))
                .iter()
                .map(|region| region.area)
                .sum()
        })
        .collect()
}

#[test]
fn total_face_area_is_continuous_through_degeneracy() {
    for (name, build) in STABLE_FIXTURES {
        let totals = area_sweep(build);
        for window in totals.windows(2) {
            let jump = (window[1] - window[0]).abs();
            assert!(
                jump < 0.5,
                "{name}: total face area jumps by {jump:.6} across the sweep {totals:?} — \
                 faces fused or vanished at a degenerate configuration"
            );
        }
    }
}

/// The reported case, pinned: at exact tangency the band's face is pinched to
/// zero width where the circle touches, and that must be reported as a cusp.
/// Written so it keeps passing if the extruder later learns to sweep a cusped
/// boundary — the requirement is "explained or built", never "silently gone".
#[test]
fn tangent_circle_in_a_band_is_diagnosable() {
    let regions = detect_regions(&tangent_circle_in_band(0.0));
    let band = regions
        .iter()
        .find(|region| (region.area - 14.4687).abs() < 0.05)
        .expect("the band face should exist at exact tangency");
    assert!(
        builds(band)
            || matches!(
                band.degeneracy(),
                Some(zerocad_core::RegionDegeneracy::Cusp { .. })
            ),
        "the cusped band must either build or report its cusp; area {:.6}, degeneracy {:?}",
        band.area,
        band.degeneracy()
    );
}

/// End-to-end pin of the reported bug, in its final resolved form.
///
/// History, because this test's assertion has inverted once and may again:
/// the reported failure was the band region silently missing from the
/// extrude. First fix: the evaluator gained a "could not be built" diagnostic
/// so the drop was at least loud, and this test asserted that diagnostic
/// appeared. Second fix went deeper — the region only failed to build because
/// span junctions carried ~1-ulp f32 quantization noise (1.4e-6 at coordinate
/// magnitude 10) that the kernel's strict 1e-6 audits rejected; welding
/// junctions at the sketch→kernel boundary made every region of this sketch
/// build. So the assertion is now the strongest one: the extrude completes
/// with NO dropped-region diagnostic. If it ever reappears, either the weld
/// regressed or a new failure mode arrived; both deserve a red test.
///
/// The diagnostic pathway itself stays guarded at region level by
/// `every_substantial_region_builds_or_explains_itself`.
#[test]
fn the_reported_sketch_extrudes_every_region_without_dropping_any() {
    use std::collections::HashSet;
    use zerocad_core::parametric::{
        EvaluationCancellation, EvaluationQuality, FeatureNode, FeatureType, ParametricGraph,
    };

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".to_string(),
        name: "Sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: reported_bug_case(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".to_string(),
        name: "Extrude".to_string(),
        feature: FeatureType::Extrude {
            depth: DEPTH,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
            region_indices: Vec::new(),
            mode: Default::default(),
            target: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");

    let output = graph
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(
                0,
                std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)),
            ),
        )
        .expect("evaluation should complete, not abort");

    let dropped: Vec<&str> = output
        .diagnostics
        .iter()
        .map(|diagnostic| diagnostic.message.as_str())
        .filter(|message| message.contains("could not be built into a solid"))
        .collect();
    assert!(
        dropped.is_empty(),
        "every region of the reported sketch should build after junction \
         welding, but the extrude dropped: {dropped:?}"
    );
    assert!(
        !output.bodies.is_empty(),
        "the extrude should produce at least one body"
    );

    // The same guarantee stated region-by-region: each substantial face of the
    // reported sketch sweeps into a solid directly.
    for (index, region) in detect_regions(&reported_bug_case()).iter().enumerate() {
        if region.area <= SUBSTANTIAL_AREA {
            continue;
        }
        assert!(
            builds(region),
            "region {index} (area {:.4}) of the reported sketch fails to build again",
            region.area
        );
    }
}

/// The detector must stay quiet on ordinary geometry. Right angles, the sharp
/// corners a user draws deliberately, and a circle crossing an edge
/// transversally are all normal — flagging them would make the diagnostic
/// worthless.
///
/// Note the bar is "does not fire on ordinary shapes", not "fires only when
/// the build fails". A boundary that *nearly* doubles back (a circle a micron
/// away from tangency) genuinely does nearly double back, and reporting that
/// is honest; it stays invisible because the evaluator only consults
/// `degeneracy()` on a face that already failed to build.
#[test]
fn ordinary_faces_report_no_degeneracy() {
    let ordinary: [(&str, SketchCurves); 3] = [
        ("stacked rectangles", stacked_rectangles(1.0e-2)),
        ("circle crossing an edge", circle_crossing_an_edge(0.0)),
        ("plain rectangle", {
            let mut curves = SketchCurves::default();
            rectangle(&mut curves, -5.0, -5.0, 5.0, 5.0);
            curves
        }),
    ];
    for (name, curves) in ordinary {
        for (index, region) in detect_regions(&curves).iter().enumerate() {
            if region.area <= SUBSTANTIAL_AREA {
                continue;
            }
            assert!(
                region.degeneracy().is_none(),
                "{name}: region {index} (area {:.4}) is ordinary but was flagged {:?}",
                region.area,
                region.degeneracy()
            );
        }
    }
}
