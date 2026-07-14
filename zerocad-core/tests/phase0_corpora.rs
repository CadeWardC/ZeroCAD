//! Frozen Phase 0 workloads. Keep their names, feature counts, and defining
//! dimensions stable so later architectural phases compare like with like.

#[path = "../benches/support/phase0_corpus.rs"]
mod corpus;

use std::collections::HashSet;

#[test]
fn phase0_corpus_manifest_is_stable() {
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
