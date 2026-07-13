use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use std::collections::HashSet;
use zerocad_core::{
    read_zcad, write_zcad, FeatureNode, FeatureType, ParametricGraph, Unit, ZcadDocument,
};

fn independent_body_history(count: usize) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    for i in 0..count {
        graph.add_feature(FeatureNode {
            id: format!("box_{}", i + 1),
            name: format!("Box {}", i + 1),
            feature: FeatureType::Box {
                w: 10.0 + (i % 5) as f32,
                h: 8.0 + (i % 3) as f32,
                d: 6.0 + (i % 7) as f32,
            },
        });
    }
    graph
}

fn modeling_pipeline(c: &mut Criterion) {
    let hidden = HashSet::new();

    c.bench_function("cold_50_body_history", |b| {
        b.iter_batched(
            || independent_body_history(50),
            |graph| black_box(graph.evaluate_bodies_with_warnings(&hidden).unwrap()),
            BatchSize::SmallInput,
        )
    });

    let warm = independent_body_history(50);
    let _ = warm.evaluate_bodies_with_warnings(&hidden).unwrap();
    c.bench_function("warm_50_body_checkpoint_hit", |b| {
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
        created_unix: None,
        hidden_nodes: HashSet::new(),
        evaluation_cache: Some(&snapshot),
        hydrated_cache_limit: None,
    })
    .unwrap();
    c.bench_function("hydrated_open_and_first_eval_50_body", |b| {
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
