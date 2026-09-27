//! Reproducible local timings; use --release. No wall-clock test assertions.
use std::time::Instant;
use zerocad_core::{read_document_from_slice, sketch::resolution::SolverCache, FeatureType};

fn main() {
    for path in std::env::args().skip(1) {
        let bytes = std::fs::read(&path).unwrap();
        let started = Instant::now();
        let loaded = read_document_from_slice(&bytes, &Default::default()).unwrap();
        println!("{path}: load {:?}", started.elapsed());
        let graph = loaded.document.evaluator_graph();
        let variables = graph.variable_map();
        for node in graph.graph.node_weights() {
            if let FeatureType::Sketch {
                solver: Some(model),
                ..
            } = &node.feature
            {
                let mut cache = SolverCache::default();
                for pass in 0..2 {
                    let started = Instant::now();
                    let result = cache.resolve(model, &variables, &|| false).unwrap();
                    println!(
                        "{} pass={pass} {:?} {:?} residual={} solves={}",
                        node.id,
                        started.elapsed(),
                        result.report.outcome,
                        result.report.residual,
                        cache.solve_count()
                    );
                }
            }
        }
        for pass in 0..2 {
            let started = Instant::now();
            let result = graph
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            println!(
                "evaluation pass={pass} {:?} bodies={} warnings={:?}",
                started.elapsed(),
                result.0.len(),
                result.1
            );
        }
    }
}
