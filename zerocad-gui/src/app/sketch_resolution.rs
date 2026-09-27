use crate::*;
use zerocad_core::sketch::{SketchSolverModel, SolveReport};

impl ZeroCadApp {
    /// Read-only UI consumers never invoke the solver, even on a cache miss.
    pub(crate) fn sketch_report(
        &self,
        model: &SketchSolverModel,
        vars: &HashMap<String, f64>,
    ) -> Option<SolveReport> {
        self.sketch_worker
            .request(model, vars, self.egui_ctx.as_ref(), false)
            .map(|r| r.report.clone())
    }

    pub(crate) fn display_sketch_curves(
        &self,
        curves: &SketchCurves,
        shapes: &[SketchShape],
        mods: &[CornerMod],
        mirrors: &[zerocad_core::SketchMirror],
        solver: Option<&SketchSolverModel>,
        vars: &HashMap<String, f64>,
    ) -> SketchCurves {
        self.sketch_worker.curves(
            curves,
            shapes,
            mods,
            mirrors,
            solver,
            vars,
            self.egui_ctx.as_ref(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn display_sketch_ink_mask(
        &self,
        curves: &SketchCurves,
        shapes: &[SketchShape],
        mods: &[CornerMod],
        mirrors: &[zerocad_core::SketchMirror],
        solver: Option<&SketchSolverModel>,
        vars: &HashMap<String, f64>,
        regions: &[Region],
    ) -> Vec<bool> {
        self.sketch_worker.ink_mask(
            curves,
            shapes,
            mods,
            mirrors,
            solver,
            vars,
            regions,
            self.egui_ctx.as_ref(),
        )
    }

    pub(crate) fn poll_sketch_solve(&mut self, ctx: &egui::Context) {
        self.sketch_worker.publish();
        if !self.is_sketch_mode {
            self.cancel_pending_sketch_solve();
            return;
        }
        if self.live_solve_pending {
            if let Some(model) = self.sketch_solver_model.as_ref() {
                let vars = self.document.variable_map();
                if let Some(result) = self.sketch_worker.request(model, &vars, Some(ctx), true) {
                    self.live_solve_pending = false;
                    self.sketch_conflict_constraint = result.report.conflicting;
                    if result.report.outcome == zerocad_core::sketch::SolveOutcome::Converged {
                        self.sketch_solver_model = Some(result.model.clone());
                    }
                    self.rebuild_active_sketch_curves_throttled();
                }
            } else {
                self.live_solve_pending = false;
            }
        }
        if self.finish_sketch_pending && !self.live_solve_pending {
            self.finish_sketch_pending = false;
            self.finish_active_sketch(ctx);
        }
    }

    pub(crate) fn cancel_pending_sketch_solve(&mut self) {
        self.sketch_worker.cancel_live();
        self.live_solve_pending = false;
        self.dimension_solve_pending = false;
        self.dimension_accept_pending = false;
        self.finish_sketch_pending = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finish_waits_for_current_solve_and_leaving_sketch_discards_pending_work() {
        let loaded = zerocad_core::read_document_from_slice(
            include_bytes!("../../../zerocad-core/tests/fixtures/more-lag.zcad"),
            &Default::default(),
        )
        .unwrap();
        let mut app = ZeroCadApp::new_with_settings(Default::default());
        app.document = loaded.document;
        let ctx = egui::Context::default();
        app.egui_ctx = Some(ctx.clone());
        app.edit_sketch("sketch_4", 0.);
        app.sketch_worker = Default::default();
        app.live_solve_pending = true;
        let model = app.sketch_solver_model.clone().unwrap();
        app.sketch_worker
            .request(&model, &app.document.variable_map(), Some(&ctx), true);
        app.finish_active_sketch(&ctx);
        assert!(app.is_sketch_mode && app.finish_sketch_pending);
        let start = std::time::Instant::now();
        while app.is_sketch_mode {
            app.poll_sketch_solve(&ctx);
            assert!(start.elapsed() < std::time::Duration::from_secs(10));
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        app.edit_sketch("sketch_4", 0.);
        app.live_solve_pending = true;
        app.finish_sketch_pending = true;
        app.is_sketch_mode = false;
        let before = app.sketch_solver_model.clone();
        app.poll_sketch_solve(&ctx);
        assert!(!app.live_solve_pending && !app.finish_sketch_pending);
        assert_eq!(
            before, app.sketch_solver_model,
            "a discarded session cannot apply a late solve"
        );
    }

    #[test]
    fn saved_lag_models_redraw_pick_and_measure_without_resolving_again() {
        for bytes in [
            include_bytes!("../../../zerocad-core/tests/fixtures/lag-fix.zcad").as_slice(),
            include_bytes!("../../../zerocad-core/tests/fixtures/more-lag.zcad").as_slice(),
        ] {
            let loaded =
                zerocad_core::read_document_from_slice(bytes, &Default::default()).unwrap();
            let mut app = ZeroCadApp::new_with_settings(Default::default());
            app.document = loaded.document;
            app.gpu_render = false;
            app.onboarding_visible = false;
            let ctx = egui::Context::default();
            app.egui_ctx = Some(ctx.clone());
            let node = app
                .document
                .graph
                .node_weights()
                .find(|n| matches!(n.feature, FeatureType::Sketch { .. }))
                .unwrap();
            let id = node.id.clone();
            let FeatureType::Sketch {
                solver: Some(model),
                ..
            } = &node.feature
            else {
                panic!("missing solver")
            };
            let model = model.clone();
            let vars = app.document.variable_map();
            let start = std::time::Instant::now();
            while app.sketch_report(&model, &vars).is_none() {
                app.poll_sketch_solve(&ctx);
                assert!(start.elapsed() < std::time::Duration::from_secs(10));
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
            let scheduled = app.sketch_worker.scheduled_count();
            app.selected_edges.insert((id, 0));
            let mut samples = Vec::new();
            for index in 0..60 {
                let start = std::time::Instant::now();
                let _ = ctx.run(
                    egui::RawInput {
                        screen_rect: Some(egui::Rect::from_min_size(
                            egui::Pos2::ZERO,
                            egui::vec2(1024., 768.),
                        )),
                        events: vec![egui::Event::PointerMoved(egui::pos2(
                            400. + index as f32,
                            400.,
                        ))],
                        ..Default::default()
                    },
                    |ctx| {
                        app.poll_sketch_solve(ctx);
                        app.draw_status_bar(ctx);
                        app.draw_workspace_viewport(ctx);
                    },
                );
                samples.push(start.elapsed());
            }
            assert_eq!(
                app.sketch_worker.scheduled_count(),
                scheduled,
                "unchanged UI reads must reuse the result"
            );
            samples.sort();
            eprintln!(
                "sketch UI smoke p50={:?} p95={:?} (CPU viewport, not native presentation latency)",
                samples[30], samples[57]
            );
        }
    }
}
