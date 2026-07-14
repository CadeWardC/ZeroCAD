use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use std::collections::HashSet;
use std::time::Duration;
use zerocad_core::{read_zcad, write_zcad, ParametricGraph, Unit, ZcadDocument};

#[path = "support/phase0_corpus.rs"]
mod corpus;

fn modeling_pipeline(c: &mut Criterion) {
    let hidden = HashSet::new();

    let mut cold = c.benchmark_group("phase0_cold_rebuild");
    cold.sample_size(10)
        .measurement_time(Duration::from_secs(5));
    for (name, builder) in [
        (
            corpus::SMALL_CORPUS,
            corpus::small_part_history as fn() -> ParametricGraph,
        ),
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
    ] {
        cold.bench_function(name, |b| {
            b.iter_batched(
                builder,
                |graph| black_box(graph.evaluate_bodies_with_warnings(&hidden).unwrap()),
                BatchSize::SmallInput,
            )
        });
    }
    cold.finish();

    let warm = corpus::hundred_feature_history();
    let _ = warm.evaluate_bodies_with_warnings(&hidden).unwrap();
    c.bench_function("phase0_warm_100_feature_checkpoint_hit", |b| {
        b.iter(|| black_box(warm.evaluate_bodies_with_warnings(&hidden).unwrap()))
    });

    let bodies = warm.evaluate_bodies(&hidden).unwrap();
    let snapshot = warm.evaluation_cache_snapshot();
    let hydrated = write_zcad(&ZcadDocument {
        graph: &warm,
        thumbnail_png: None,
        mesh_cache: Some(&bodies),
        units: Unit::Millimeter,
        bbox: [0.0; 6],
        created_unix: Some(0),
        hidden_nodes: HashSet::new(),
        evaluation_cache: Some(&snapshot),
        hydrated_cache_limit: None,
    })
    .unwrap();
    c.bench_function("phase0_hydrated_open_first_eval_100_feature", |b| {
        b.iter(|| {
            let loaded = read_zcad(black_box(&hydrated)).unwrap();
            if let Some(cache) = loaded.evaluation_cache {
                loaded.graph.install_evaluation_cache(cache);
            }
            black_box(loaded.graph.evaluate_bodies(&hidden).unwrap())
        })
    });
}

criterion_group!(benches, modeling_pipeline);
criterion_main!(benches);
