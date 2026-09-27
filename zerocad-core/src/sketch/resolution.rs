//! Bounded, disposable solver reuse. Nothing in this module is document state.
//!
//! Exact input comparisons avoid hashing/serializing a sketch on display reads.
//! Both failures and successes are reusable; entity extraction must not run the
//! conflict search again. Each evaluator thread owns its cache independently.

use super::{solve, SketchSolverModel, SolveOutcome, SolveReport};
use std::{cell::RefCell, collections::HashMap, sync::Arc};

const MAX_ENTRIES: usize = 16;
const MAX_PAYLOAD_BYTES: usize = 8 * 1024 * 1024;

#[derive(Debug)]
pub struct ResolvedSolver {
    pub model: SketchSolverModel,
    pub report: SolveReport,
}

struct Entry {
    source: SketchSolverModel,
    variables: HashMap<String, f64>,
    result: Arc<ResolvedSolver>,
    bytes: usize,
}

#[derive(Default)]
pub struct SolverCache {
    entries: Vec<Entry>,
    bytes: usize,
    solves: usize,
}

impl SolverCache {
    pub fn solve_count(&self) -> usize {
        self.solves
    }

    /// Cancellation never publishes a partial result or a cache entry.
    pub fn resolve(
        &mut self,
        model: &SketchSolverModel,
        vars: &HashMap<String, f64>,
        cancelled: &dyn Fn() -> bool,
    ) -> Option<Arc<ResolvedSolver>> {
        if cancelled() {
            return None;
        }
        if let Some(index) = self
            .entries
            .iter()
            .position(|entry| entry.source == *model && entry.variables == *vars)
        {
            let entry = self.entries.remove(index);
            let result = entry.result.clone();
            self.entries.push(entry);
            return Some(result);
        }
        self.solves += 1;
        let report = solve::solve_model_cancellable(model, vars, cancelled)?;
        let mut solved = model.clone();
        if report.outcome == SolveOutcome::Converged {
            solve::apply_solution(&mut solved, &report);
        }
        let result = Arc::new(ResolvedSolver {
            model: solved,
            report,
        });
        // Bound retained payload as well as entry count. Serialization is only
        // used on misses for accounting, never as a per-frame cache key.
        let bytes = serde_json::to_vec(&(model, vars, &result.model))
            .map(|bytes| bytes.len())
            .unwrap_or(MAX_PAYLOAD_BYTES + 1);
        if bytes <= MAX_PAYLOAD_BYTES {
            while !self.entries.is_empty()
                && (self.entries.len() >= MAX_ENTRIES || self.bytes + bytes > MAX_PAYLOAD_BYTES)
            {
                self.bytes -= self.entries.remove(0).bytes;
            }
            self.bytes += bytes;
            self.entries.push(Entry {
                source: model.clone(),
                variables: vars.clone(),
                result: result.clone(),
                bytes,
            });
        }
        Some(result)
    }
}

thread_local! {
    static CACHE: RefCell<SolverCache> = RefCell::new(SolverCache::default());
}

pub fn resolve_solver(
    model: &SketchSolverModel,
    vars: &HashMap<String, f64>,
) -> Arc<ResolvedSolver> {
    CACHE.with(|cache| cache.borrow_mut().resolve(model, vars, &|| false).unwrap())
}

/// Warm the same cache used by baking and diagnostics without losing worker
/// cancellation while a conflicting sketch is being diagnosed.
pub fn resolve_solver_cancellable(
    model: &SketchSolverModel,
    vars: &HashMap<String, f64>,
    cancelled: &dyn Fn() -> bool,
) -> Option<Arc<ResolvedSolver>> {
    CACHE.with(|cache| cache.borrow_mut().resolve(model, vars, cancelled))
}

/// Monotonic count of primary solves started on this evaluator thread, useful
/// for structural regression checks and profiling without wall-clock gates.
pub fn cached_solve_count() -> usize {
    CACHE.with(|cache| cache.borrow().solve_count())
}
