#[path = "../benches/support/phase0_corpus.rs"]
mod corpus;

use serde::Serialize;
use std::collections::HashSet;
use std::time::Instant;
use zerocad_core::{read_zcad, write_zcad, ParametricGraph, Unit, ZcadDocument};

#[derive(Serialize)]
struct CorpusMeasurement {
    name: &'static str,
    feature_count: usize,
    body_count: usize,
    triangle_count: usize,
    cold_rebuild_ms: f64,
    warm_rebuild_ms: f64,
    compact_save_ms: f64,
    compact_open_ms: f64,
    compact_bytes: usize,
    hydrated_bytes: usize,
}

#[derive(Serialize)]
struct BaselineReport {
    schema: u32,
    profile: &'static str,
    tessellation_ms: f64,
    tessellation_triangles: usize,
    corpora: Vec<CorpusMeasurement>,
}

fn elapsed_ms(started: Instant) -> f64 {
    started.elapsed().as_secs_f64() * 1_000.0
}

fn measure_corpus(name: &'static str, graph: ParametricGraph) -> CorpusMeasurement {
    let hidden = HashSet::new();
    let feature_count = graph.graph.node_count().saturating_sub(1);

    let started = Instant::now();
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&hidden)
        .expect("cold corpus evaluation");
    let cold_rebuild_ms = elapsed_ms(started);
    assert!(warnings.is_empty(), "{name} warnings: {warnings:?}");

    let started = Instant::now();
    let warm_bodies = graph
        .evaluate_bodies(&hidden)
        .expect("warm corpus evaluation");
    let warm_rebuild_ms = elapsed_ms(started);
    let triangle_count = bodies.iter().map(|(_, mesh)| mesh.indices.len() / 3).sum();

    let started = Instant::now();
    let compact = write_zcad(&ZcadDocument {
        graph: &graph,
        thumbnail_png: None,
        mesh_cache: None,
        units: Unit::Millimeter,
        bbox: [0.0; 6],
        created_unix: Some(0),
        hidden_nodes: HashSet::new(),
        evaluation_cache: None,
        hydrated_cache_limit: None,
    })
    .expect("compact save");
    let compact_save_ms = elapsed_ms(started);

    let started = Instant::now();
    let loaded = read_zcad(&compact).expect("compact open");
    let compact_open_ms = elapsed_ms(started);
    assert_eq!(
        loaded.graph.graph.node_count().saturating_sub(1),
        feature_count
    );

    let snapshot = graph.evaluation_cache_snapshot();
    let hydrated = write_zcad(&ZcadDocument {
        graph: &graph,
        thumbnail_png: None,
        mesh_cache: Some(&warm_bodies),
        units: Unit::Millimeter,
        bbox: [0.0; 6],
        created_unix: Some(0),
        hidden_nodes: HashSet::new(),
        evaluation_cache: Some(&snapshot),
        hydrated_cache_limit: None,
    })
    .expect("hydrated save");

    CorpusMeasurement {
        name,
        feature_count,
        body_count: bodies.len(),
        triangle_count,
        cold_rebuild_ms,
        warm_rebuild_ms,
        compact_save_ms,
        compact_open_ms,
        compact_bytes: compact.len(),
        hydrated_bytes: hydrated.len(),
    }
}

fn measure_tessellation() -> (f64, usize) {
    let solid = openrcad::primitives::make_cylinder(
        &openrcad::foundation::Ax2::new(
            openrcad::foundation::Pnt::origin(),
            openrcad::foundation::Dir::dz(),
        ),
        12.0,
        30.0,
    );
    let started = Instant::now();
    let mesh = openrcad::mesh::tessellate(&solid, 0.05, 0.13);
    (elapsed_ms(started), mesh.triangles.len())
}

fn main() {
    let (tessellation_ms, tessellation_triangles) = measure_tessellation();
    let corpora = vec![
        measure_corpus(corpus::SMALL_CORPUS, corpus::small_part_history()),
        measure_corpus(
            corpus::HUNDRED_FEATURE_CORPUS,
            corpus::hundred_feature_history(),
        ),
        measure_corpus(
            corpus::FIVE_HUNDRED_FEATURE_CORPUS,
            corpus::five_hundred_feature_history(),
        ),
        measure_corpus(
            corpus::IMPORTED_STEP_CORPUS,
            corpus::imported_step_history(),
        ),
        measure_corpus(
            corpus::DIFFICULT_KERNEL_CORPUS,
            corpus::difficult_kernel_history(),
        ),
    ];
    let report = BaselineReport {
        schema: 1,
        profile: "cargo run --release -p zerocad-core --example phase0_baseline",
        tessellation_ms,
        tessellation_triangles,
        corpora,
    };
    println!(
        "{}",
        serde_json::to_string_pretty(&report).expect("serialize baseline")
    );
}
