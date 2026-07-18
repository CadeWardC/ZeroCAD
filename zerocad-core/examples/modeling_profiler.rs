#[path = "../benches/support/benchmark_corpus.rs"]
#[allow(dead_code)]
mod corpus;

use serde::Serialize;
use std::collections::HashSet;
use std::hint::black_box;
use std::time::Instant;
use zerocad_core::{FeatureType, ParametricGraph};

const WARMUP_COUNT: usize = 5;
const SAMPLE_COUNT: usize = 31;
const TRAILING_EDIT_BUDGET_MS: f64 = 50.0;
const REPRESENTATIVE_REBUILD_BUDGET_MS: f64 = 250.0;
const LAST_FEATURE: &str = "move_0100";

#[derive(Serialize)]
struct WorkloadReport {
    name: &'static str,
    corpus: &'static str,
    scope: &'static str,
    warmups: usize,
    samples: usize,
    samples_ms: Vec<f64>,
    p50_ms: f64,
    p95_ms: f64,
    budget_ms: f64,
    passed: bool,
}

#[derive(Serialize)]
struct ModelingProfileReport {
    schema: u32,
    profile: &'static str,
    percentile_method: &'static str,
    reference_hardware: &'static str,
    workloads: Vec<WorkloadReport>,
    passed: bool,
}

fn elapsed_ms(started: Instant) -> f64 {
    started.elapsed().as_secs_f64() * 1_000.0
}

fn nearest_rank(samples: &[f64], percentile: f64) -> f64 {
    assert!(!samples.is_empty(), "percentile requires samples");
    assert!((0.0..=1.0).contains(&percentile));
    let mut ordered = samples.to_vec();
    ordered.sort_by(f64::total_cmp);
    let rank = (percentile * ordered.len() as f64).ceil().max(1.0) as usize;
    ordered[rank - 1]
}

fn evaluate(graph: &ParametricGraph, hidden: &HashSet<String>) {
    let output = graph
        .evaluate_bodies_with_warnings(hidden)
        .expect("modeling profiler evaluation");
    assert!(
        output.1.is_empty(),
        "profiler corpus warnings: {:?}",
        output.1
    );
    black_box(output);
}

fn toggle_last_feature(graph: &mut ParametricGraph, alternate: bool) {
    let index = graph
        .graph
        .node_indices()
        .find(|index| graph.graph[*index].id == LAST_FEATURE)
        .expect("100-feature corpus trailing node");
    let FeatureType::BodyTransform { translation, .. } = &mut graph.graph[index].feature else {
        panic!("the 100-feature corpus must end in a body transform");
    };
    translation[0] = if alternate { 0.03 } else { 0.02 };
    graph
        .commit_feature_edit(LAST_FEATURE)
        .expect("commit trailing corpus edit");
}

fn measure_trailing_edit() -> WorkloadReport {
    let hidden = HashSet::new();
    let mut graph = corpus::hundred_feature_history();
    evaluate(&graph, &hidden);
    let mut alternate = false;
    for _ in 0..WARMUP_COUNT {
        alternate = !alternate;
        toggle_last_feature(&mut graph, alternate);
        evaluate(&graph, &hidden);
    }
    let mut samples = Vec::with_capacity(SAMPLE_COUNT);
    for _ in 0..SAMPLE_COUNT {
        alternate = !alternate;
        let started = Instant::now();
        toggle_last_feature(&mut graph, alternate);
        evaluate(&graph, &hidden);
        samples.push(elapsed_ms(started));
    }
    let p50_ms = nearest_rank(&samples, 0.50);
    let p95_ms = nearest_rank(&samples, 0.95);
    WorkloadReport {
        name: "trailing_edit",
        corpus: corpus::HUNDRED_FEATURE_CORPUS,
        scope: "toggle and commit move_0100, then rebuild and tessellate",
        warmups: WARMUP_COUNT,
        samples: SAMPLE_COUNT,
        samples_ms: samples,
        p50_ms,
        p95_ms,
        budget_ms: TRAILING_EDIT_BUDGET_MS,
        passed: p50_ms < TRAILING_EDIT_BUDGET_MS && p95_ms < TRAILING_EDIT_BUDGET_MS,
    }
}

fn fresh_rebuild(hidden: &HashSet<String>) -> f64 {
    let graph = corpus::five_hundred_feature_history();
    let started = Instant::now();
    evaluate(&graph, hidden);
    elapsed_ms(started)
}

fn measure_representative_rebuild() -> WorkloadReport {
    let hidden = HashSet::new();
    for _ in 0..WARMUP_COUNT {
        black_box(fresh_rebuild(&hidden));
    }
    let samples: Vec<_> = (0..SAMPLE_COUNT).map(|_| fresh_rebuild(&hidden)).collect();
    let p50_ms = nearest_rank(&samples, 0.50);
    let p95_ms = nearest_rank(&samples, 0.95);
    WorkloadReport {
        name: "representative_rebuild",
        corpus: corpus::FIVE_HUNDRED_FEATURE_CORPUS,
        scope: "evaluate and tessellate a newly constructed cache-free graph",
        warmups: WARMUP_COUNT,
        samples: SAMPLE_COUNT,
        samples_ms: samples,
        p50_ms,
        p95_ms,
        budget_ms: REPRESENTATIVE_REBUILD_BUDGET_MS,
        passed: p50_ms < REPRESENTATIVE_REBUILD_BUDGET_MS
            && p95_ms < REPRESENTATIVE_REBUILD_BUDGET_MS,
    }
}

fn main() {
    let workloads = vec![measure_trailing_edit(), measure_representative_rebuild()];
    let passed = workloads.iter().all(|workload| workload.passed);
    let report = ModelingProfileReport {
        schema: 1,
        profile: "cargo run --release -p zerocad-core --example modeling_profiler",
        percentile_method: "nearest-rank: sorted[ceil(p * n) - 1]",
        reference_hardware: "Lenovo 82XU; Ryzen 7 7840HS; 16 GiB; Windows 11; Rust 1.94",
        workloads,
        passed,
    };
    println!(
        "{}",
        serde_json::to_string_pretty(&report).expect("serialize modeling profile")
    );
    if !passed {
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_rank_uses_the_requested_31_sample_positions() {
        let samples: Vec<_> = (1..=31).map(f64::from).collect();
        assert_eq!(nearest_rank(&samples, 0.50), 16.0);
        assert_eq!(nearest_rank(&samples, 0.95), 30.0);
    }
}
