use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use std::collections::HashSet;
use std::time::Duration;
use zerocad_core::{
    read_document_from_slice, write_document_to_vec, Document, HydrationBundle, LoadOptions,
    ParametricGraph, SaveOptions, SaveProfile, Unit,
};

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
    let document = Document::from_graph(warm.clone_document(), Unit::Millimeter);
    let hydrated = write_document_to_vec(
        &document,
        &SaveOptions {
            profile: SaveProfile::Hydrated {
                total_accelerator_budget: 128 * 1024 * 1024,
            },
        },
        &HydrationBundle {
            display_meshes: Some(bodies),
            evaluation_cache: Some(snapshot),
            ..Default::default()
        },
    )
    .unwrap();
    c.bench_function("phase0_hydrated_open_first_eval_100_feature", |b| {
        b.iter(|| {
            let loaded =
                read_document_from_slice(black_box(&hydrated), &LoadOptions::default()).unwrap();
            if let Some(cache) = loaded.accelerators.evaluation_cache {
                loaded.document.install_evaluation_cache(cache);
            }
            black_box(loaded.document.evaluate_bodies(&hidden).unwrap())
        })
    });
}

fn phase6_hotspots(c: &mut Criterion) {
    let left = openrcad::primitives::make_box_operation(
        &openrcad::foundation::Pnt::origin(),
        20.0,
        16.0,
        12.0,
    )
    .unwrap()
    .value;
    let right = openrcad::primitives::make_box_operation(
        &openrcad::foundation::Pnt::new(8.0, 5.0, 3.0),
        20.0,
        16.0,
        12.0,
    )
    .unwrap()
    .value;
    c.bench_function("phase6_boolean_common", |b| {
        b.iter(|| {
            black_box(
                openrcad::algo::boolean_operation(
                    black_box(&left),
                    black_box(&right),
                    openrcad::algo::BooleanOp::Common,
                )
                .unwrap(),
            )
        })
    });

    let cylinder = openrcad::primitives::make_cylinder_operation(
        &openrcad::foundation::Ax2::new(
            openrcad::foundation::Pnt::origin(),
            openrcad::foundation::Dir::dz(),
        ),
        12.0,
        30.0,
    )
    .unwrap()
    .value;
    c.bench_function("phase6_checked_tessellation", |b| {
        b.iter(|| {
            black_box(openrcad::mesh::tessellate_checked(
                black_box(&cylinder),
                0.005,
                0.05,
            ))
        })
    });
    let display_mesh = openrcad::mesh::tessellate_checked(&cylinder, 0.005, 0.05).unwrap();
    c.bench_function("phase6_render_buffer_preparation", |b| {
        b.iter(|| black_box(display_mesh.gpu_mesh()))
    });

    let large = Document::from_graph(corpus::five_hundred_feature_history(), Unit::Millimeter);
    let compact =
        write_document_to_vec(&large, &SaveOptions::default(), &HydrationBundle::default())
            .unwrap();
    c.bench_function("phase6_canonical_save_500_features", |b| {
        b.iter(|| {
            black_box(
                write_document_to_vec(
                    black_box(&large),
                    &SaveOptions::default(),
                    &HydrationBundle::default(),
                )
                .unwrap(),
            )
        })
    });
    c.bench_function("phase6_canonical_open_500_features", |b| {
        b.iter(|| {
            black_box(
                read_document_from_slice(black_box(&compact), &LoadOptions::default()).unwrap(),
            )
        })
    });
}

criterion_group!(benches, modeling_pipeline, phase6_hotspots);
criterion_main!(benches);
