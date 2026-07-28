use crate::*;
use zerocad_core::SharedEvaluatedScene;

/// The immutable evaluated scene paired with the revision consumed by the GPU.
///
/// The inner fields stay private to this module so replacing the scene cannot
/// accidentally bypass epoch invalidation. Read-only viewport consumers use
/// the `Deref<EvaluatedScene>` implementation.
pub(crate) struct ViewportSceneState {
    scene: SharedEvaluatedScene,
    epoch: u64,
}

impl ViewportSceneState {
    pub(crate) fn new(scene: SharedEvaluatedScene) -> Self {
        Self { scene, epoch: 0 }
    }

    fn replace(&mut self, scene: SharedEvaluatedScene) {
        self.scene = scene;
        self.epoch = self.epoch.wrapping_add(1);
    }

    pub(crate) fn shared(&self) -> SharedEvaluatedScene {
        self.scene.clone()
    }

    pub(crate) fn epoch(&self) -> u64 {
        self.epoch
    }
}

impl std::ops::Deref for ViewportSceneState {
    type Target = EvaluatedScene;

    fn deref(&self) -> &Self::Target {
        &self.scene
    }
}

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

    /// The one mutation seam for the evaluated viewport scene. Geometry and
    /// placement changes both pass through here, so the GPU epoch and cached
    /// scene statistics cannot be forgotten.
    pub(crate) fn replace_evaluated_scene(&mut self, scene: SharedEvaluatedScene) {
        self.scene_stats = scene.stats();
        self.evaluated_scene.replace(scene);
    }

    /// The ONE way to replace the displayed part meshes. The evaluator/cache
    /// body vector and identity-placement scene share one geometry allocation.
    /// Never assign `self.body_meshes` directly.
    pub(crate) fn set_body_meshes(&mut self, bodies: Vec<(String, MockMesh)>) {
        let bodies = std::sync::Arc::new(bodies);
        let scene = std::sync::Arc::new(EvaluatedScene::from_part_bodies(bodies.clone()));
        self.body_meshes = bodies;
        self.replace_evaluated_scene(scene);
    }

    pub(crate) fn mirror_join_outcome(
        &self,
        feature_id: &str,
        source_body_id: &str,
    ) -> MirrorJoinOutcome {
        classify_mirror_join_outcome(
            self.eval_pending,
            self.unresolved_features.get(feature_id).cloned(),
            self.body_meshes.iter().any(|(id, _)| id == feature_id),
            self.body_meshes.iter().any(|(id, _)| id == source_body_id),
        )
    }

    fn finish_pending_mirror_join_feedback(&mut self) {
        let Some(pending) = self.pending_mirror_join_feedback.clone() else {
            return;
        };
        let had_warnings = self.error_msg.is_some();
        match self.mirror_join_outcome(&pending.feature_id, &pending.source_body_id) {
            MirrorJoinOutcome::Evaluating => {}
            MirrorJoinOutcome::Joined => {
                self.status_msg = if had_warnings {
                    "Mirror joined into the source body; the model also has warnings.".to_string()
                } else {
                    "Mirror joined into the source body.".to_string()
                };
                self.pending_mirror_join_feedback = None;
            }
            MirrorJoinOutcome::Separate => {
                self.status_msg = if had_warnings {
                    "Mirror created as a separate body; Join could not connect. See warning."
                        .to_string()
                } else {
                    "Mirror created as a separate body; Join could not connect.".to_string()
                };
                self.pending_mirror_join_feedback = None;
            }
            MirrorJoinOutcome::Unresolved(_) => {
                self.status_msg = "Mirror remains unresolved; see the reported issue.".to_string();
                self.pending_mirror_join_feedback = None;
            }
        }
    }

    /// Apply an evaluation result to the displayed model + status line.
    pub(crate) fn apply_eval_result(
        &mut self,
        bodies: Vec<(String, MockMesh)>,
        mut warnings: Vec<String>,
        face_reattached: bool,
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
        if face_reattached {
            if let Some(editing_id) = self.editing_sketch_id.clone() {
                if let Some(b) = self
                    .document
                    .sketch_face_boundaries
                    .get(editing_id.as_str())
                {
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
                            let warnings = output.rendered_warnings();
                            if !warnings.is_empty() {
                                log::debug!("extrude preview diagnostics: {warnings:?}");
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
                        Ok(output) if !output.has_diagnostic_warnings() => {
                            let warnings = output.rendered_warnings();
                            let bodies = std::sync::Arc::new(output.bodies);
                            self.remember_edge_mod_arc(key, bodies.clone(), &warnings);
                            self.edge_mod_arc_cache = Some((key, bodies, warnings));
                            self.edge_mod_arc_failed = None;
                        }
                        Ok(output) => {
                            self.edge_mod_arc_cache = None;
                            let msg = output
                                .diagnostics
                                .iter()
                                .find(|diagnostic| {
                                    diagnostic.severity != zerocad_core::DiagnosticSeverity::Info
                                })
                                .map(|diagnostic| diagnostic.rendered_message().to_string())
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
                        .filter_map(|s| {
                            s.reason()
                                .map(|r| (s.feature_id.to_string(), r.to_string()))
                        })
                        .collect();
                    self.datum_values = output.datums.clone();
                    let warnings = output.rendered_warnings();
                    self.document
                        .install_evaluation_cache(output.cache_snapshot);
                    self.document
                        .queue_legacy_reference_backfills(output.legacy_reference_backfills);
                    let face_reattached = self
                        .document
                        .apply_face_reattach_updates(output.face_reattach);
                    self.apply_eval_result(output.bodies, warnings, face_reattached);
                    self.finish_pending_mirror_join_feedback();
                }
                Err(err) => {
                    self.error_msg = Some(err.to_string());
                    self.status_msg = if self.pending_mirror_join_feedback.take().is_some() {
                        "Error: Mirror evaluation failed.".to_string()
                    } else {
                        "Error: Model evaluation failed.".to_string()
                    };
                }
            }
        }
    }
}

fn classify_mirror_join_outcome(
    eval_pending: bool,
    unresolved_reason: Option<String>,
    has_feature_body: bool,
    has_source_body: bool,
) -> MirrorJoinOutcome {
    if eval_pending {
        MirrorJoinOutcome::Evaluating
    } else if let Some(reason) = unresolved_reason {
        MirrorJoinOutcome::Unresolved(reason)
    } else if has_feature_body {
        MirrorJoinOutcome::Separate
    } else if has_source_body {
        MirrorJoinOutcome::Joined
    } else {
        MirrorJoinOutcome::Unresolved("neither joined nor separate output is available".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocad_core::{SceneInstance, ScenePlacement};

    #[test]
    fn every_scene_replacement_advances_the_gpu_epoch() {
        let mut app = ZeroCadApp::new();
        let initial_epoch = app.evaluated_scene.epoch();

        app.set_body_meshes(vec![("box".to_string(), MockMesh::make_box(2.0, 3.0, 4.0))]);
        assert_eq!(app.evaluated_scene.epoch(), initial_epoch.wrapping_add(1));
        assert_eq!(app.scene_stats, app.evaluated_scene.stats());
        assert_eq!(app.scene_stats.geometry_pool_count, 1);
        assert_eq!(app.scene_stats.referenced_geometry_count, 1);
        assert_eq!(app.scene_stats.instance_count, 1);
        assert_eq!(
            app.scene_stats.instance_expanded_triangles,
            app.scene_stats.geometry_pool_triangles
        );

        let placement = ScenePlacement::from_rotation_translation(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [10.0, 0.0, 0.0],
        )
        .unwrap();
        let placed = std::sync::Arc::new(
            EvaluatedScene::from_instances(
                app.body_meshes.clone(),
                vec![SceneInstance::new("placed-box", 0, placement)],
            )
            .unwrap(),
        );
        let before_placement = app.evaluated_scene.epoch();
        app.replace_evaluated_scene(placed.clone());

        assert_eq!(
            app.evaluated_scene.epoch(),
            before_placement.wrapping_add(1)
        );
        assert!(std::sync::Arc::ptr_eq(
            &app.evaluated_scene.shared(),
            &placed
        ));
        assert_eq!(
            app.evaluated_scene.world_bounds(),
            Some(([10.0, 0.0, 0.0], [12.0, 3.0, 4.0]))
        );
        assert_eq!(app.scene_stats, placed.stats());
    }

    #[test]
    fn mirror_join_outcome_tracks_evaluating_joined_separate_and_unresolved() {
        assert_eq!(
            classify_mirror_join_outcome(true, None, true, true),
            MirrorJoinOutcome::Evaluating
        );
        assert_eq!(
            classify_mirror_join_outcome(false, None, false, true),
            MirrorJoinOutcome::Joined
        );
        assert_eq!(
            classify_mirror_join_outcome(false, None, true, true),
            MirrorJoinOutcome::Separate
        );
        assert_eq!(
            classify_mirror_join_outcome(false, Some("missing plane".to_string()), false, true),
            MirrorJoinOutcome::Unresolved("missing plane".to_string())
        );
    }
}
