//! Frozen Phase 0 workloads. Keep their names, feature counts, and defining
//! dimensions stable so later architectural phases compare like with like.

#[path = "../benches/support/benchmark_corpus.rs"]
mod corpus;

use std::collections::HashSet;
use zerocad_core::ParametricGraph;

type CorpusBuilder = fn() -> ParametricGraph;

fn frozen_corpora() -> [(&'static str, CorpusBuilder); 5] {
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
fn baseline_corpus_manifest_is_stable() {
    assert_eq!(
        [
            corpus::SMALL_CORPUS,
            corpus::HUNDRED_FEATURE_CORPUS,
            corpus::FIVE_HUNDRED_FEATURE_CORPUS,
            corpus::IMPORTED_STEP_CORPUS,
            corpus::DIFFICULT_KERNEL_CORPUS,
        ],
        [
            "small_part",
            "dependent_100_feature",
            "dependent_500_feature",
            "imported_step_box",
            "difficult_through_hole",
        ]
    );
    let feature_count = |graph: zerocad_core::ParametricGraph| graph.graph.node_count() - 1;
    assert_eq!(feature_count(corpus::small_part_history()), 4);
    assert_eq!(feature_count(corpus::hundred_feature_history()), 100);
    assert_eq!(feature_count(corpus::five_hundred_feature_history()), 500);
    assert_eq!(feature_count(corpus::imported_step_history()), 1);
    assert_eq!(corpus::EXTERNAL_STEP_CORPUS, "external_nist_bracket");
    assert_eq!(feature_count(corpus::external_step_history()), 1);
    assert_eq!(feature_count(corpus::difficult_kernel_history()), 4);
}

#[test]
fn geometry_corpora_evaluate_without_warnings() {
    let hidden = HashSet::new();
    for (name, graph) in [
        (corpus::SMALL_CORPUS, corpus::small_part_history()),
        (
            corpus::IMPORTED_STEP_CORPUS,
            corpus::imported_step_history(),
        ),
        (
            corpus::DIFFICULT_KERNEL_CORPUS,
            corpus::difficult_kernel_history(),
        ),
    ] {
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&hidden)
            .unwrap_or_else(|error| panic!("{name} failed: {error}"));
        assert!(warnings.is_empty(), "{name} warnings: {warnings:?}");
        assert!(!bodies.is_empty(), "{name} produced no bodies");
        assert!(
            bodies
                .iter()
                .all(|(_, mesh)| mesh.mass_properties().is_some()),
            "{name} produced an open display mesh"
        );
    }
}

#[test]
fn fresh_baseline_evaluations_have_deterministic_body_and_triangle_counts() {
    let hidden = HashSet::new();
    for (name, build) in frozen_corpora() {
        let evaluate = || {
            let graph = build();
            let (bodies, warnings) = graph
                .evaluate_bodies_with_warnings(&hidden)
                .unwrap_or_else(|error| panic!("{name} failed: {error}"));
            assert!(warnings.is_empty(), "{name} warnings: {warnings:?}");
            bodies
                .iter()
                .map(|(body, mesh)| (body.clone(), mesh.indices.len() / 3))
                .collect::<Vec<_>>()
        };

        assert_eq!(
            evaluate(),
            evaluate(),
            "{name} body ordering and triangle counts changed across fresh evaluations"
        );
    }
}

#[test]
fn external_step_fixture_retains_its_non_openrcad_provenance() {
    let fixture = include_str!("fixtures/step/nist-bracket1-part.stp");
    assert!(fixture.contains("'ST-ACIS'"));
    assert!(fixture.contains("'bracket1-Part'"));
    assert!(!fixture.contains("OpenRCAD"));
}

#[test]
fn committed_baseline_json_matches_the_frozen_corpus_manifest() {
    let report: serde_json::Value = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../benchmarks/phase0-baseline.json"
    )))
    .expect("committed Phase 0 baseline must be valid JSON");
    assert_eq!(report["schema"], 1);

    let committed = report["corpora"]
        .as_array()
        .expect("baseline corpora must be an array");
    let expected = frozen_corpora();
    assert_eq!(committed.len(), expected.len());
    for (entry, (name, build)) in committed.iter().zip(expected) {
        assert_eq!(entry["name"], name);
        assert_eq!(
            entry["feature_count"],
            build().graph.node_count().saturating_sub(1)
        );
    }
}
