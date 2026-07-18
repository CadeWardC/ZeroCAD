use crate::*;

impl ZeroCadApp {
    /// Run a keyboard-shortcut action. Single dispatch point shared by the global
    /// hotkey handler and (where relevant) menu items.
    pub(crate) fn run_shortcut(&mut self, action: ShortcutAction) {
        match action {
            ShortcutAction::NewDesign => self.new_design(),
            ShortcutAction::OpenDesign => self.open_design(),
            ShortcutAction::SaveDesign => self.open_save_dialog(),
            ShortcutAction::ExportStl => self.export_stl(),
            ShortcutAction::Undo => {
                if self.is_sketch_mode {
                    self.undo_last_sketch_action();
                } else {
                    self.undo();
                }
            }
            ShortcutAction::Redo => self.redo(),
            ShortcutAction::CopyBody => self.copy_selected_body(),
            ShortcutAction::PasteBody => self.paste_copied_body(),
            ShortcutAction::DeleteSelection => self.delete_selected_node(),
            ShortcutAction::ToggleTheme => self.dark_mode = !self.dark_mode,
            ShortcutAction::OpenSettings => self.show_preferences = true,
        }
    }

    /// Process global keyboard shortcuts, or capture a new binding when the
    /// Shortcuts settings tab is waiting for one. Called first thing each frame,
    /// before any UI, so bindings fire regardless of which panel is hovered.
    pub(crate) fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        // Rebinding capture mode: the Shortcuts tab is waiting for a key combo.
        if let Some(action) = self.capturing_shortcut {
            // Escape cancels the capture, leaving the existing binding intact.
            if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.capturing_shortcut = None;
                return;
            }
            let captured = ctx.input(|i| {
                i.events.iter().find_map(|ev| match ev {
                    egui::Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        modifiers,
                        ..
                    } => Some((*key, *modifiers)),
                    _ => None,
                })
            });
            if let Some((key, mods)) = captured {
                self.keymap
                    .set(action, shortcuts::Hotkey::from_event(key, mods));
                self.keymap.save();
                self.capturing_shortcut = None;
            }
            // Suppress normal dispatch while capturing so the captured combo does
            // not also trigger an action on this frame.
            return;
        }

        // Sketch undo owns its binding even while the inline dimension field is
        // focused (continuous Line keeps that field alive between segments).
        // Consume it here so TextEdit cannot also undo its buffer this frame.
        if self.is_sketch_mode
            && self
                .keymap
                .get(ShortcutAction::Undo)
                .is_some_and(|hotkey| hotkey.consume_if_pressed(ctx))
        {
            self.run_shortcut(ShortcutAction::Undo);
            return;
        }

        // Normal dispatch. Skip entirely while a widget holds keyboard focus, so
        // typing in a text field (dimensions, variable names, …) never fires a
        // command. At most one action runs per frame.
        if ctx.memory(|m| m.focused().is_some()) {
            return;
        }
        let mut fire = None;
        for &action in ShortcutAction::ALL {
            if let Some(hk) = self.keymap.get(action) {
                if hk.pressed(ctx) {
                    fire = Some(action);
                    break;
                }
            }
        }
        if let Some(action) = fire {
            self.run_shortcut(action);
        }
    }

    /// Rebuild the model using the committed native geometry. 3D fillets use
    /// OpenRCAD's rolling-ball builder directly, so there is no separate
    /// faceted-preview/arc-refine swap.
    pub(crate) fn reevaluate_geometry(&mut self) {
        self.document_revision = self.document_revision.wrapping_add(1);
        let snapshot = self.current_document_snapshot();
        self.recovery.note_edit(snapshot);
        self.spawn_refine_eval();
        self.status_msg = "Updating model...".to_string();
    }

    /// The ONE way to replace the displayed body meshes. Bumps `mesh_epoch`
    /// (the GPU viewport re-uploads its scene only when this changes — a
    /// forgotten bump means it silently renders a stale model) and refreshes
    /// the cached mesh stats. Never assign `self.body_meshes` directly.
    pub(crate) fn set_body_meshes(&mut self, bodies: Vec<(String, MockMesh)>) {
        self.body_meshes = std::sync::Arc::new(bodies);
        self.mesh_epoch = self.mesh_epoch.wrapping_add(1);
        self.mesh_stats = Self::mesh_totals(&self.body_meshes);
    }

    /// Apply an evaluation result to the displayed model + status line.
    pub(crate) fn apply_eval_result(
        &mut self,
        bodies: Vec<(String, MockMesh)>,
        mut warnings: Vec<String>,
    ) {
        log::debug!(
            "[evaluation] applying committed result bodies={:?} warnings={:?}",
            bodies.iter().map(|(id, _)| id.as_str()).collect::<Vec<_>>(),
            warnings
        );
        self.pending_visual = None;
        self.move_preview_bodies = None;
        self.set_body_meshes(bodies);
        // Sketch-on-face reattachment: if the evaluation re-projected a face
        // outline (the parent body changed), commit the refreshed boundary +
        // remapped extrude region indices so the displayed sketch and any
        // future picks agree with what was just built. Self-healing
        // normalization, not a user edit — no undo step. If the user is
        // mid-edit on such a sketch, refresh its live reference outline too.
        if self.document.apply_face_reattach() {
            if let Some(editing_id) = self.editing_sketch_id.clone() {
                if let Some(b) = self.document.sketch_face_boundaries.get(&editing_id) {
                    self.active_face_boundary = b.clone();
                    self.recompute_sketch_regions();
                }
            }
        }
        match self.refresh_projected_sketch_edges() {
            Ok(true) => {
                self.spawn_refine_eval();
                self.status_msg =
                    "Updating sketches that reference changed body edges...".to_string();
                return;
            }
            Ok(false) => {}
            Err(error) => warnings.push(error),
        }
        if warnings.is_empty() {
            self.error_msg = None;
            self.status_msg = "Model evaluated successfully.".to_string();
        } else {
            // Non-fatal: the model evaluated, but a boolean didn't do what the
            // user asked. Surface it instead of letting the geometry come out
            // wrong silently.
            self.status_msg = format!("Model evaluated with {} warning(s).", warnings.len());
            self.error_msg = Some(warnings.join("\n"));
        }
    }

    /// Spawn the background arc-fillet evaluation. Tagged with a generation so a
    /// later edit's job supersedes this one (stale results are dropped on
    /// arrival). The worker wakes the UI via `request_repaint` the moment it's
    /// done so the refined geometry appears without waiting for the next input.
    pub(crate) fn spawn_refine_eval(&mut self) {
        self.eval_generation = self.evaluator.submit(
            evaluation_worker::EvaluationPurpose::CommittedModel,
            self.document.clone(),
            self.hidden_nodes.clone(),
            zerocad_core::EvaluationQuality::Final,
            self.egui_ctx.clone(),
        );
        self.eval_pending = true;
        self.eval_started = Some(std::time::Instant::now());
    }

    /// Poll the background refine channel; apply the result if it's the current
    /// generation. Called once per frame.
    pub(crate) fn poll_refine_eval(&mut self) {
        while let Some(completion) = self.evaluator.try_recv() {
            if completion.generation != self.evaluator.current_generation() {
                continue;
            }

            if let evaluation_worker::EvaluationPurpose::ExtrudePreview(key) = completion.purpose {
                if self.extrude_preview_inflight == Some(key) {
                    self.extrude_preview_inflight = None;
                    match completion.result {
                        Ok(output) => {
                            if !output.warnings.is_empty() {
                                log::debug!("extrude preview warnings: {:?}", output.warnings);
                            }
                            self.extrude_preview_cache =
                                Some((key, std::sync::Arc::new(output.bodies)));
                        }
                        Err(err) => {
                            log::warn!("Background extrude preview failed: {err}");
                            self.extrude_preview_cache = None;
                        }
                    }
                }
                continue;
            }

            if let evaluation_worker::EvaluationPurpose::EdgeModPreview(key) = completion.purpose {
                if self.edge_mod_arc_inflight == Some(key) {
                    self.edge_mod_arc_inflight = None;
                    match completion.result {
                        Ok(output) if output.warnings.is_empty() => {
                            let bodies = std::sync::Arc::new(output.bodies);
                            self.remember_edge_mod_arc(key, bodies.clone(), &output.warnings);
                            self.edge_mod_arc_cache = Some((key, bodies, output.warnings));
                            self.edge_mod_arc_failed = None;
                        }
                        Ok(output) => {
                            self.edge_mod_arc_cache = None;
                            let msg = output
                                .warnings
                                .first()
                                .cloned()
                                .unwrap_or_else(|| "the blend could not be computed".into());
                            self.status_msg = format!(
                                "This size does not work here: {msg} Try another size or cancel."
                            );
                            self.edge_mod_arc_failed = Some(key);
                        }
                        Err(err) => {
                            self.edge_mod_arc_cache = None;
                            self.status_msg = format!(
                                "This size does not work here: {err} Try another size or cancel."
                            );
                            self.edge_mod_arc_failed = Some(key);
                        }
                    }
                }
                continue;
            }

            if completion.generation != self.eval_generation {
                continue;
            }
            self.eval_pending = false;
            self.eval_started = None;
            match completion.result {
                Ok(output) => {
                    log::debug!(
                        "model eval: total={:?} build={:?} tess={:?}",
                        output.timings.total,
                        output.timings.build,
                        output.timings.tessellation
                    );
                    self.unresolved_features = output
                        .statuses
                        .iter()
                        .filter_map(|s| s.reason().map(|r| (s.feature_id.clone(), r.to_string())))
                        .collect();
                    self.document
                        .install_evaluation_cache(output.cache_snapshot);
                    self.document
                        .apply_face_reattach_updates(output.face_reattach);
                    self.apply_eval_result(output.bodies, output.warnings);
                }
                Err(err) => {
                    self.error_msg = Some(err.to_string());
                    self.status_msg = "Error: Model evaluation failed.".to_string();
                }
            }
        }
    }
}
