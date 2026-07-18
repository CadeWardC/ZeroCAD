//! Gate 0 semantic golden. Exact document/output structure is frozen, while
//! geometric scalars use one OpenRCAD-policy-derived tolerance. Raw mesh
//! buffers, tessellation order, and triangle counts are deliberately absent.

#[path = "../benches/support/benchmark_corpus.rs"]
mod corpus;

use openrcad::foundation::TolerancePolicy;
use serde::Deserialize;
use std::collections::{BTreeMap, HashSet};
use zerocad_core::ParametricGraph;

const GOLDEN_JSON: &str = include_str!("fixtures/semantic/vnext-baseline-v1.json");
const BASELINE_COMMIT: &str = "71acd92e1ee152fcd64daa5c91bedd1b8d5cb270";
type CorpusBuilder = fn() -> ParametricGraph;

#[derive(Clone, Debug, Deserialize)]
struct GoldenManifest {
    schema: u32,
    baseline_commit: String,
    comparison: ComparisonConfig,
    cases: Vec<GoldenCase>,
}

#[derive(Clone, Debug, Deserialize)]
struct ComparisonConfig {
    kernel_policy: String,
    absolute_multiplier: f64,
    relative_multiplier: f64,
}

#[derive(Clone, Debug, Deserialize)]
struct GoldenCase {
    name: String,
    feature_count: usize,
    feature_kind_counts: BTreeMap<String, usize>,
    diagnostic_count: usize,
    bodies: Vec<GoldenBody>,
}

#[derive(Clone, Debug, Deserialize)]
struct GoldenBody {
    id: String,
    parts: Vec<GoldenPart>,
    mass: GoldenMass,
}

#[derive(Clone, Debug, Deserialize)]
struct GoldenPart {
    shell_count: usize,
    face_count: usize,
    edge_count: usize,
    vertex_count: usize,
    bounds_min: [f64; 3],
    bounds_max: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
struct GoldenMass {
    volume: f64,
    surface_area: f64,
    centroid: [f64; 3],
}

#[derive(Clone, Copy, Debug)]
struct ComparisonTolerance {
    absolute: f64,
    relative: f64,
}

fn parse_manifest() -> GoldenManifest {
    serde_json::from_str(GOLDEN_JSON).expect("semantic golden manifest must be valid JSON")
}

fn comparison_tolerance(config: &ComparisonConfig) -> Result<ComparisonTolerance, String> {
    if config.kernel_policy != "openrcad-standard-v1" {
        return Err(format!(
            "unknown semantic-golden kernel policy '{}'",
            config.kernel_policy
        ));
    }
    if !config.absolute_multiplier.is_finite()
        || config.absolute_multiplier <= 0.0
        || !config.relative_multiplier.is_finite()
        || config.relative_multiplier <= 0.0
    {
        return Err("semantic-golden tolerance multipliers must be finite and positive".into());
    }
    let base = TolerancePolicy::STANDARD.classification;
    Ok(ComparisonTolerance {
        absolute: base * config.absolute_multiplier,
        relative: base * config.relative_multiplier,
    })
}

fn actual_case(name: &str, graph: ParametricGraph) -> GoldenCase {
    let hidden = HashSet::new();
    let (meshes, warnings) = graph
        .evaluate_bodies_with_warnings(&hidden)
        .unwrap_or_else(|error| panic!("{name}: evaluation failed: {error}"));
    let solids = graph
        .debug_kernel_solids(&hidden)
        .unwrap_or_else(|error| panic!("{name}: B-Rep evaluation failed: {error}"));

    let mut feature_kind_counts = BTreeMap::new();
    for feature in graph
        .graph
        .node_weights()
        .filter(|feature| feature.kind_id.as_str() != "core.origin")
    {
        *feature_kind_counts
            .entry(feature.kind_id.to_string())
            .or_default() += 1;
    }

    let bodies = solids
        .into_iter()
        .map(|(body_id, parts)| {
            let mesh = &meshes
                .iter()
                .find(|(id, _)| id == &body_id)
                .unwrap_or_else(|| panic!("{name}: missing display body for '{body_id}'"))
                .1;
            let mass = mesh
                .mass_properties()
                .unwrap_or_else(|| panic!("{name}: body '{body_id}' is not closed"));
            let mut parts = parts
                .into_iter()
                .map(|solid| {
                    let (lo, hi) = solid.bounding_box().corners().unwrap_or_else(|| {
                        panic!("{name}: body '{body_id}' has an empty B-Rep part")
                    });
                    GoldenPart {
                        shell_count: solid.shells().len(),
                        face_count: solid.face_count(),
                        edge_count: solid.edge_count(),
                        vertex_count: solid.vertex_count(),
                        bounds_min: [lo.x(), lo.y(), lo.z()],
                        bounds_max: [hi.x(), hi.y(), hi.z()],
                    }
                })
                .collect::<Vec<_>>();
            parts.sort_by(|left, right| compare_point(&left.bounds_min, &right.bounds_min));
            GoldenBody {
                id: body_id,
                parts,
                mass: GoldenMass {
                    volume: mass.volume,
                    surface_area: mass.surface_area,
                    centroid: mass.centroid,
                },
            }
        })
        .collect();

    GoldenCase {
        name: name.to_string(),
        feature_count: graph.graph.node_count().saturating_sub(1),
        feature_kind_counts,
        diagnostic_count: warnings.len(),
        bodies,
    }
}

fn compare_point(left: &[f64; 3], right: &[f64; 3]) -> std::cmp::Ordering {
    left.iter()
        .zip(right)
        .map(|(left, right)| left.total_cmp(right))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or(std::cmp::Ordering::Equal)
}

fn compare_case(
    expected: &GoldenCase,
    actual: &GoldenCase,
    tolerance: ComparisonTolerance,
) -> Result<(), String> {
    exact("case name", &expected.name, &actual.name)?;
    exact(
        "feature count",
        expected.feature_count,
        actual.feature_count,
    )?;
    exact(
        "feature kind counts",
        &expected.feature_kind_counts,
        &actual.feature_kind_counts,
    )?;
    exact(
        "diagnostic count",
        expected.diagnostic_count,
        actual.diagnostic_count,
    )?;
    exact("body count", expected.bodies.len(), actual.bodies.len())?;

    for (body_index, (expected_body, actual_body)) in
        expected.bodies.iter().zip(&actual.bodies).enumerate()
    {
        exact(
            &format!("body {body_index} id"),
            &expected_body.id,
            &actual_body.id,
        )?;
        exact(
            &format!("body '{}' part count", expected_body.id),
            expected_body.parts.len(),
            actual_body.parts.len(),
        )?;
        for (part_index, (expected_part, actual_part)) in expected_body
            .parts
            .iter()
            .zip(&actual_body.parts)
            .enumerate()
        {
            let prefix = format!("body '{}' part {part_index}", expected_body.id);
            exact(
                &format!("{prefix} shell count"),
                expected_part.shell_count,
                actual_part.shell_count,
            )?;
            exact(
                &format!("{prefix} face count"),
                expected_part.face_count,
                actual_part.face_count,
            )?;
            exact(
                &format!("{prefix} edge count"),
                expected_part.edge_count,
                actual_part.edge_count,
            )?;
            exact(
                &format!("{prefix} vertex count"),
                expected_part.vertex_count,
                actual_part.vertex_count,
            )?;
            close_point(
                &format!("{prefix} minimum bound"),
                &expected_part.bounds_min,
                &actual_part.bounds_min,
                tolerance,
            )?;
            close_point(
                &format!("{prefix} maximum bound"),
                &expected_part.bounds_max,
                &actual_part.bounds_max,
                tolerance,
            )?;
        }
        close(
            &format!("body '{}' volume", expected_body.id),
            expected_body.mass.volume,
            actual_body.mass.volume,
            tolerance,
        )?;
        close(
            &format!("body '{}' surface area", expected_body.id),
            expected_body.mass.surface_area,
            actual_body.mass.surface_area,
            tolerance,
        )?;
        close_point(
            &format!("body '{}' centroid", expected_body.id),
            &expected_body.mass.centroid,
            &actual_body.mass.centroid,
            tolerance,
        )?;
    }
    Ok(())
}

fn exact<T: PartialEq + std::fmt::Debug>(
    label: &str,
    expected: T,
    actual: T,
) -> Result<(), String> {
    if expected == actual {
        Ok(())
    } else {
        Err(format!(
            "{label} changed: expected {expected:?}, actual {actual:?}"
        ))
    }
}

fn close_point(
    label: &str,
    expected: &[f64; 3],
    actual: &[f64; 3],
    tolerance: ComparisonTolerance,
) -> Result<(), String> {
    for axis in 0..3 {
        close(
            &format!("{label} axis {axis}"),
            expected[axis],
            actual[axis],
            tolerance,
        )?;
    }
    Ok(())
}

fn close(
    label: &str,
    expected: f64,
    actual: f64,
    tolerance: ComparisonTolerance,
) -> Result<(), String> {
    let scale = expected.abs().max(actual.abs());
    let allowed = tolerance.absolute + tolerance.relative * scale;
    let delta = (expected - actual).abs();
    if expected.is_finite() && actual.is_finite() && delta <= allowed {
        Ok(())
    } else {
        Err(format!(
            "{label} changed: expected {expected}, actual {actual}, delta {delta}, allowed {allowed}"
        ))
    }
}

fn corpus_cases() -> [(&'static str, CorpusBuilder); 5] {
    [
        (corpus::SMALL_CORPUS, corpus::small_part_history),
        (
            corpus::HUNDRED_FEATURE_CORPUS,
            corpus::hundred_feature_history,
        ),
        (
            corpus::FIVE_HUNDRED_FEATURE_CORPUS,
            corpus::five_hundred_feature_history,
        ),
        (corpus::IMPORTED_STEP_CORPUS, corpus::imported_step_history),
        (
            corpus::DIFFICULT_KERNEL_CORPUS,
            corpus::difficult_kernel_history,
        ),
    ]
}

#[test]
fn committed_semantic_golden_matches_the_vnext_baseline() {
    let manifest = parse_manifest();
    assert_eq!(manifest.schema, 1);
    assert_eq!(manifest.baseline_commit, BASELINE_COMMIT);
    let tolerance = comparison_tolerance(&manifest.comparison).unwrap();
    let corpus = corpus_cases();
    assert_eq!(manifest.cases.len(), corpus.len());

    for (expected, (name, build)) in manifest.cases.iter().zip(corpus) {
        assert_eq!(expected.name, name, "semantic corpus order changed");
        let actual = actual_case(name, build());
        compare_case(expected, &actual, tolerance)
            .unwrap_or_else(|error| panic!("semantic golden '{name}' failed: {error}"));
    }
}

#[test]
fn comparator_accepts_tolerant_geometry_without_weakening_exact_semantics() {
    let manifest = parse_manifest();
    let tolerance = comparison_tolerance(&manifest.comparison).unwrap();
    let expected = &manifest.cases[0];
    let mut within = expected.clone();
    within.bodies[0].mass.volume += tolerance.absolute * 0.5;
    within.bodies[0].parts[0].bounds_max[0] += tolerance.absolute * 0.5;
    assert!(compare_case(expected, &within, tolerance).is_ok());

    let mut semantic_drift = expected.clone();
    semantic_drift.bodies[0].id.push_str("-changed");
    let error = compare_case(expected, &semantic_drift, tolerance).unwrap_err();
    assert!(error.contains("body 0 id changed"), "{error}");
}

#[test]
fn comparator_rejects_non_finite_and_out_of_policy_geometry() {
    let manifest = parse_manifest();
    let tolerance = comparison_tolerance(&manifest.comparison).unwrap();
    let expected = &manifest.cases[0];

    let mut outside = expected.clone();
    let scale = expected.bodies[0].mass.volume.abs();
    outside.bodies[0].mass.volume += 2.0 * (tolerance.absolute + tolerance.relative * scale);
    let error = compare_case(expected, &outside, tolerance).unwrap_err();
    assert!(error.contains("volume changed"), "{error}");

    let mut non_finite = expected.clone();
    non_finite.bodies[0].mass.centroid[0] = f64::NAN;
    let error = compare_case(expected, &non_finite, tolerance).unwrap_err();
    assert!(error.contains("centroid axis 0 changed"), "{error}");
}
