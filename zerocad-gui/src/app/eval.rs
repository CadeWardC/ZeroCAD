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
            ShortcutAction::Undo => self.undo(),
            ShortcutAction::Redo => self.redo(),
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

        // Normal dispatch. Skip entirely while a widget holds keyboard focus, so
        // typing in a text field (dimensions, variable names, …) never fires a
        // command. At most one action runs per frame.
        if ctx.memory(|m| m.focus().is_some()) {
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
        // Native fillets are final geometry, not a faceted draft.
        match self.graph.evaluate_bodies_with_status(&self.hidden_nodes) {
            Ok((bodies, warnings, statuses)) => {
                // Record which features failed to resolve so the history tree can
                // flag them (⚠) rather than only a global warning count.
                self.unresolved_features = statuses
                    .into_iter()
                    .filter_map(|s| s.reason().map(|r| (s.feature_id.clone(), r.to_string())))
                    .collect();
                self.apply_eval_result(bodies, warnings);
            }
            Err(err) => {
                self.error_msg = Some(err);
                self.status_msg = "Error: Model evaluation failed.".to_string();
                return;
            }
        }
        // Refine to the smooth single-face arc fillet off the UI thread. Skipped
        // when there's no fillet, since then the draft already *is* the final.
        if self.graph.has_arc_fillet(&self.hidden_nodes) {
            self.spawn_refine_eval();
            self.status_msg = "Smoothing fillet…".to_string();
        } else {
            self.eval_pending = false;
            self.eval_rx = None;
        }
    }

    /// Apply an evaluation result to the displayed model + status line.
    pub(crate) fn apply_eval_result(
        &mut self,
        bodies: Vec<(String, MockMesh)>,
        warnings: Vec<String>,
    ) {
        self.pending_visual = None;
        self.body_meshes = bodies;
        self.mesh_stats = Self::mesh_totals(&self.body_meshes);
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
        self.eval_gen += 1;
        let gen = self.eval_gen;
        let graph = self.graph.clone();
        let hidden = self.hidden_nodes.clone();
        let ctx = self.egui_ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        self.eval_rx = Some(rx);
        self.eval_pending = true;
        std::thread::spawn(move || {
            let result = graph.evaluate_bodies_with_warnings(&hidden);
            let _ = tx.send((gen, result));
            if let Some(ctx) = ctx {
                ctx.request_repaint();
            }
        });
    }

    /// Poll the background refine channel; apply the result if it's the current
    /// generation. Called once per frame.
    pub(crate) fn poll_refine_eval(&mut self) {
        let Some(rx) = self.eval_rx.as_ref() else {
            return;
        };
        match rx.try_recv() {
            Ok((gen, result)) => {
                self.eval_rx = None;
                self.eval_pending = false;
                // Drop a superseded job's result; a newer one is (or will be) in
                // flight and owns the display.
                if gen != self.eval_gen {
                    return;
                }
                match result {
                    Ok((bodies, warnings)) => self.apply_eval_result(bodies, warnings),
                    // A failed refine leaves the faceted draft on screen — still a
                    // valid model — rather than blanking it.
                    Err(err) => log::warn!("Background refine failed: {err}"),
                }
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.eval_rx = None;
                self.eval_pending = false;
            }
        }
    }

    /// Synchronous rebuild that waits for the **final** arc geometry. Use only
    /// where the result must be current before the next line runs (export, save
    /// thumbnail); interactive edits use [`reevaluate_geometry`] so they don't
    /// stall.
    pub(crate) fn reevaluate_geometry_blocking(&mut self) {
        // A pending background job is now obsolete — bump the generation so its
        // late result is discarded in favour of this authoritative one.
        self.eval_gen += 1;
        self.eval_rx = None;
        self.eval_pending = false;
        self.pending_visual = None;
        match self.graph.evaluate_bodies_with_warnings(&self.hidden_nodes) {
            Ok((bodies, warnings)) => self.apply_eval_result(bodies, warnings),
            Err(err) => {
                self.error_msg = Some(err);
                self.status_msg = "Error: Model evaluation failed.".to_string();
            }
        }
    }
}
