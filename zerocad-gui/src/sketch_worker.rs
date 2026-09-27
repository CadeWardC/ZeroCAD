//! One bounded solver worker. Display reads only enqueue work or reuse results.
use std::{
    collections::{HashMap, VecDeque},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Condvar, Mutex,
    },
};
use zerocad_core::sketch::{
    resolution::{ResolvedSolver, SolverCache},
    SketchSolverModel,
};

const CAPACITY: usize = 128;
const PAYLOAD_BUDGET: usize = 16 * 1024 * 1024;

struct Entry {
    id: u64,
    model: Arc<SketchSolverModel>,
    vars: Arc<HashMap<String, f64>>,
    cancelled: Arc<AtomicBool>,
    started: bool,
    result: Option<Arc<ResolvedSolver>>,
    ready: Option<Arc<ResolvedSolver>>,
    bytes: usize,
    repaint: Option<eframe::egui::Context>,
}

#[derive(Default)]
struct State {
    next: u64,
    live: Option<u64>,
    entries: VecDeque<Entry>,
    stop: bool,
}

pub(crate) struct SketchWorker {
    shared: Arc<(Mutex<State>, Condvar)>,
}

impl Default for SketchWorker {
    fn default() -> Self {
        let shared = Arc::new((Mutex::new(State::default()), Condvar::new()));
        let worker = shared.clone();
        std::thread::Builder::new()
            .name("zerocad-sketch-solver".into())
            .spawn(move || {
                let mut cache = SolverCache::default();
                loop {
                    let (id, model, vars, cancelled) = {
                        let (lock, wake) = &*worker;
                        let mut state = lock.lock().unwrap();
                        loop {
                            if state.stop {
                                return;
                            }
                            let live = state.live;
                            let index = state
                                .entries
                                .iter()
                                .position(|e| Some(e.id) == live && !e.started)
                                .or_else(|| state.entries.iter().position(|e| !e.started));
                            if let Some(index) = index {
                                let entry = &mut state.entries[index];
                                entry.started = true;
                                break (
                                    entry.id,
                                    entry.model.clone(),
                                    entry.vars.clone(),
                                    entry.cancelled.clone(),
                                );
                            }
                            state = wake.wait(state).unwrap();
                        }
                    };
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        cache.resolve(&model, &vars, &|| cancelled.load(Ordering::Acquire))
                    }))
                    .unwrap_or_else(|_| {
                        // A failed worker must not turn into an endless pending UI.
                        Some(Arc::new(ResolvedSolver {
                            model: (*model).clone(),
                            report: zerocad_core::sketch::SolveReport {
                                outcome: zerocad_core::sketch::SolveOutcome::DidNotConverge,
                                positions: Vec::new(),
                                radii: Vec::new(),
                                dof: 0,
                                conflicting: None,
                                residual: f64::INFINITY,
                            },
                        }))
                    });
                    let mut state = worker.0.lock().unwrap();
                    if let Some(entry) = state.entries.iter_mut().find(|e| e.id == id) {
                        if !cancelled.load(Ordering::Acquire)
                            && Arc::ptr_eq(&entry.cancelled, &cancelled)
                        {
                            entry.ready = result;
                            if let Some(ctx) = &entry.repaint {
                                ctx.request_repaint();
                            }
                        }
                    }
                }
            })
            .expect("start sketch solver");
        Self { shared }
    }
}

impl SketchWorker {
    /// Exact input identity includes geometry, constraint sources and variables.
    /// No serialization or solver execution occurs on a display read.
    pub(crate) fn request(
        &self,
        model: &SketchSolverModel,
        vars: &HashMap<String, f64>,
        repaint: Option<&eframe::egui::Context>,
        live: bool,
    ) -> Option<Arc<ResolvedSolver>> {
        self.request_inner(model, vars, repaint, live, false)
    }

    pub(crate) fn edit(
        &self,
        model: &SketchSolverModel,
        vars: &HashMap<String, f64>,
        repaint: Option<&eframe::egui::Context>,
    ) -> Option<Arc<ResolvedSolver>> {
        self.request_inner(model, vars, repaint, true, true)
    }

    fn request_inner(
        &self,
        model: &SketchSolverModel,
        vars: &HashMap<String, f64>,
        repaint: Option<&eframe::egui::Context>,
        live: bool,
        immediate: bool,
    ) -> Option<Arc<ResolvedSolver>> {
        let mut state = self.shared.0.lock().unwrap();
        if let Some(index) = state.entries.iter().position(|e| {
            (*e.model == *model || e.result.as_ref().is_some_and(|r| r.model == *model))
                && *e.vars == *vars
        }) {
            let mut entry = state.entries.remove(index).unwrap();
            if live {
                cancel_previous_live(&mut state, entry.id);
            }
            if repaint.is_some() {
                entry.repaint = repaint.cloned();
            }
            let result = entry.result.clone();
            state.entries.push_back(entry);
            return result;
        }
        let result = if immediate {
            let start = std::time::Instant::now();
            // Small edits stay immediate; costly work is retried by the worker.
            let report = zerocad_core::sketch::solve::solve_model_cancellable(model, vars, &|| {
                start.elapsed() >= std::time::Duration::from_millis(2)
            });
            report.map(|report| {
                let mut model = model.clone();
                if report.outcome == zerocad_core::sketch::SolveOutcome::Converged {
                    zerocad_core::sketch::solve::apply_solution(&mut model, &report);
                }
                Arc::new(ResolvedSolver { model, report })
            })
        } else {
            None
        };
        state.next += 1;
        let id = state.next;
        if live {
            cancel_previous_live(&mut state, id);
        }
        let bytes = serde_json::to_vec(&(model, vars))
            .map(|b| b.len().saturating_mul(2))
            .unwrap_or(PAYLOAD_BUDGET);
        while !state.entries.is_empty()
            && (state.entries.len() >= CAPACITY
                || state
                    .entries
                    .iter()
                    .map(|entry| entry.bytes)
                    .sum::<usize>()
                    .saturating_add(bytes)
                    > PAYLOAD_BUDGET)
        {
            if let Some(old) = state.entries.pop_front() {
                old.cancelled.store(true, Ordering::Release);
            }
        }
        state.entries.push_back(Entry {
            id,
            model: Arc::new(model.clone()),
            vars: Arc::new(vars.clone()),
            cancelled: Arc::new(AtomicBool::new(false)),
            started: result.is_some(),
            result: result.clone(),
            ready: None,
            bytes,
            repaint: repaint.cloned(),
        });
        self.shared.1.notify_one();
        result
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn curves(
        &self,
        curves: &zerocad_core::SketchCurves,
        shapes: &[zerocad_core::SketchShape],
        mods: &[zerocad_core::CornerMod],
        mirrors: &[zerocad_core::SketchMirror],
        solver: Option<&SketchSolverModel>,
        vars: &HashMap<String, f64>,
        repaint: Option<&eframe::egui::Context>,
    ) -> zerocad_core::SketchCurves {
        let resolved = solver
            .filter(|m| {
                !m.is_empty() && zerocad_core::sketch::solve::has_variable_bound_constraint(m)
            })
            .and_then(|m| self.request(m, vars, repaint, false));
        zerocad_core::sketch::effective_curves_from_resolved(
            curves,
            shapes,
            mods,
            mirrors,
            resolved.as_ref().map(|r| &r.model).or(solver),
            vars,
        )
        .0
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn ink_mask(
        &self,
        curves: &zerocad_core::SketchCurves,
        shapes: &[zerocad_core::SketchShape],
        mods: &[zerocad_core::CornerMod],
        mirrors: &[zerocad_core::SketchMirror],
        solver: Option<&SketchSolverModel>,
        vars: &HashMap<String, f64>,
        regions: &[zerocad_core::Region],
        repaint: Option<&eframe::egui::Context>,
    ) -> Vec<bool> {
        let resolved = solver
            .filter(|m| {
                !m.is_empty() && zerocad_core::sketch::solve::has_variable_bound_constraint(m)
            })
            .and_then(|m| self.request(m, vars, repaint, false));
        zerocad_core::text::sketch_region_ink_mask_resolved(
            curves,
            shapes,
            mods,
            mirrors,
            resolved.as_ref().map(|r| &r.model).or(solver),
            vars,
            regions,
        )
    }

    /// Publish completions once at the frame boundary so picking, drawing, and
    /// measurement in that frame all consume the same snapshot.
    pub(crate) fn publish(&self) {
        let mut state = self.shared.0.lock().unwrap();
        for entry in &mut state.entries {
            if let Some(result) = entry.ready.take() {
                entry.result = Some(result);
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn scheduled_count(&self) -> u64 {
        self.shared.0.lock().unwrap().next
    }

    pub(crate) fn cancel_live(&self) {
        let mut state = self.shared.0.lock().unwrap();
        cancel_previous_live(&mut state, u64::MAX);
        state.live = None;
    }
}

fn cancel_previous_live(state: &mut State, id: u64) {
    if state.live != Some(id) {
        if let Some(previous) = state.live {
            // Remove superseded work so a burst retains only its latest live
            // request. Completed pure results may remain useful for undo.
            if let Some(index) = state
                .entries
                .iter()
                .position(|e| e.id == previous && e.result.is_none())
            {
                let old = state.entries.remove(index).unwrap();
                old.cancelled.store(true, Ordering::Release);
            }
        }
        // A live edit must not wait behind an unrelated saved-sketch solve.
        // Restart that pure display request after the latest edit is finished.
        for entry in &mut state.entries {
            if id != u64::MAX
                && entry.id != id
                && entry.started
                && entry.result.is_none()
                && entry.ready.is_none()
            {
                entry.cancelled.store(true, Ordering::Release);
                entry.cancelled = Arc::new(AtomicBool::new(false));
                entry.started = false;
            }
        }
        state.live = Some(id);
    }
}

impl Drop for SketchWorker {
    fn drop(&mut self) {
        let mut state = self.shared.0.lock().unwrap();
        state.stop = true;
        for entry in &state.entries {
            entry.cancelled.store(true, Ordering::Release);
        }
        self.shared.1.notify_one();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocad_core::{
        sketch::{Constraint, EntityId, SketchPoint, SolveOutcome},
        Dimension,
    };

    fn line(length: f32) -> SketchSolverModel {
        SketchSolverModel {
            points: vec![
                SketchPoint {
                    id: EntityId(1),
                    pos: (0., 0.),
                },
                SketchPoint {
                    id: EntityId(2),
                    pos: (1., 0.),
                },
            ],
            constraints: vec![
                Constraint::Fixed {
                    id: EntityId(3),
                    p: EntityId(1),
                },
                Constraint::Distance {
                    id: EntityId(4),
                    a: EntityId(1),
                    b: EntityId(2),
                    d: Dimension::literal(length),
                },
            ],
            ..Default::default()
        }
    }

    fn complete(
        worker: &SketchWorker,
        model: &SketchSolverModel,
        live: bool,
    ) -> Arc<ResolvedSolver> {
        let start = std::time::Instant::now();
        loop {
            worker.publish();
            if let Some(result) = worker.request(model, &HashMap::new(), None, live) {
                return result;
            }
            assert!(
                start.elapsed() < std::time::Duration::from_secs(10),
                "worker did not complete"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    #[test]
    fn display_misses_schedule_and_publish_one_immutable_result() {
        let worker = SketchWorker::default();
        let model = line(10.);
        // A display miss never executes an inline solve, even for a tiny model.
        assert!(worker
            .request(&model, &HashMap::new(), None, false)
            .is_none());
        let result = complete(&worker, &model, false);
        assert_eq!(result.report.outcome, SolveOutcome::Converged);
        assert!((result.model.points[1].pos.0 - 10.).abs() < 1e-6);
        for _ in 0..100 {
            assert!(Arc::ptr_eq(
                &result,
                &worker
                    .request(&model, &HashMap::new(), None, false)
                    .unwrap()
            ));
        }
        assert_eq!(worker.shared.0.lock().unwrap().next, 1);
        assert_eq!(model.points[1].pos.0, 1., "input remains authoritative");
    }

    #[test]
    fn live_bursts_cancel_old_work_and_never_return_a_stale_result() {
        let worker = SketchWorker::default();
        for length in 2..100 {
            worker.request(&line(length as f32), &HashMap::new(), None, true);
            let state = worker.shared.0.lock().unwrap();
            assert!(state.entries.len() <= CAPACITY);
            assert!(state.entries.iter().filter(|e| e.result.is_none()).count() <= 1);
        }
        let latest = complete(&worker, &line(99.), true);
        assert!((latest.model.points[1].pos.0 - 99.).abs() < 1e-6);
        worker.cancel_live();
        // Undo can reuse an exact completed input, but never another revision.
        let undone = complete(&worker, &line(5.), true);
        assert!((undone.model.points[1].pos.0 - 5.).abs() < 1e-6);
    }

    #[test]
    fn requesting_an_existing_entry_while_cancelling_an_earlier_live_entry_is_safe() {
        let worker = SketchWorker::default();
        worker.request(&line(20.), &HashMap::new(), None, true);
        worker.request(&line(30.), &HashMap::new(), None, false);
        worker.request(&line(30.), &HashMap::new(), None, true);
        assert!((complete(&worker, &line(30.), true).model.points[1].pos.0 - 30.).abs() < 1e-6);
    }
}
