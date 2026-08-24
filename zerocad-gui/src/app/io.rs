use crate::*;

const SAVE_DIALOG_BACKDROP_ID: &str = "save_dialog_backdrop";
const SAVE_DIALOG_WINDOW_ID: &str = "save_dialog_window";
const SAVE_DIALOG_CONTENT_WIDTH: f32 = 520.0;
const SAVE_DIALOG_DESTINATION_WIDTH: f32 = SAVE_DIALOG_CONTENT_WIDTH - 24.0;
const UNSAVED_CHANGES_BACKDROP_ID: &str = "unsaved_changes_backdrop";
const UNSAVED_CHANGES_WINDOW_ID: &str = "unsaved_changes_window";

fn save_project_title_error(title: &str) -> Option<&'static str> {
    let trimmed = title.trim();
    if trimmed.is_empty() {
        return Some("Enter a project name.");
    }
    if trimmed != title {
        return Some("The project name cannot begin or end with a space.");
    }
    if matches!(trimmed, "." | "..") {
        return Some("Choose a more descriptive project name.");
    }
    if trimmed.ends_with('.') {
        return Some("The project name cannot end with a period.");
    }
    if trimmed
        .chars()
        .any(|character| character.is_control() || r#"<>:"/\|?*"#.contains(character))
    {
        return Some("The project name contains a character that cannot be used in a file name.");
    }
    None
}

fn save_dialog_should_close(
    backdrop_clicked: bool,
    escape_pressed: bool,
    window_open: bool,
) -> bool {
    backdrop_clicked || escape_pressed || !window_open
}

fn unsaved_changes_should_cancel(
    phase: ProjectTransitionPhase,
    backdrop_clicked: bool,
    escape_pressed: bool,
    window_open: bool,
) -> bool {
    phase == ProjectTransitionPhase::Confirm && (backdrop_clicked || escape_pressed || !window_open)
}

/// One undo/redo entry: the parametric history plus the visibility set. The
/// visibility set has to travel with the graph — an extrude auto-hides its
/// sketch, so undoing the extrude must also reveal the sketch again.
impl ZeroCadApp {
    pub(crate) fn current_document_snapshot(&self) -> Document {
        let mut document = self.document.clone_authoritative();
        document.state.units = self.current_unit;
        document.state.created_unix = self.doc_created_unix;
        document.state.visibility.clear();
        for hidden in &self.hidden_nodes {
            document.set_visible(hidden.clone(), false);
        }
        document
    }

    pub(crate) fn current_project_snapshot(&self) -> ProjectDocument {
        match self.project_kind {
            ProjectKind::Part => ProjectDocument::Part(self.current_document_snapshot()),
            ProjectKind::Assembly => {
                ProjectDocument::Assembly(self.assembly_document.clone_authoritative())
            }
        }
    }

    fn snapshot(&self) -> UndoSnapshot {
        UndoSnapshot {
            project: self.current_project_snapshot(),
        }
    }

    /// Restore a snapshot: swap in the graph (rebuilding its skipped id→index
    /// map so later features can still resolve their parents) and the
    /// visibility set, then clear all selection/op state that may reference
    /// nodes that no longer exist.
    fn restore_snapshot(&mut self, snap: UndoSnapshot) {
        match snap.project {
            ProjectDocument::Part(document) => {
                self.project_kind = ProjectKind::Part;
                self.current_unit = document.state.units;
                self.doc_created_unix = document.state.created_unix;
                self.hidden_nodes = document.hidden_entities();
                self.document = document;
                self.document.rebuild_node_map();
            }
            ProjectDocument::Assembly(document) => {
                self.project_kind = ProjectKind::Assembly;
                self.current_unit = document.presentation.units;
                self.doc_created_unix = document.created_unix;
                self.assembly_document = document;
                self.selected_assembly_occurrence = None;
                self.assembly_rename_occurrence = None;
                self.assembly_interactive_target = None;
                self.assembly_gizmo_drag = None;
                self.selected_assembly_mate = None;
                self.hydrate_assembly_definitions();
            }
        }
        self.selected_node_id = None;
        self.feature_properties_dialog = None;
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_sketch_points.clear();
        self.selected_body.clear();
        self.extrude_op = None;
        self.extrude_profile_pick_active = false;
        self.edge_mod_op = None;
        self.move_op = None;
        self.combine_op = None;
        self.split_body_op = None;
        self.scale_body_op = None;
        self.move_preview_bodies = None;
        self.pending_visual = None;
        if self.project_kind == ProjectKind::Part {
            self.reevaluate_geometry();
        } else {
            self.document_revision = self.document_revision.wrapping_add(1);
            self.recovery.note_edit(self.current_project_snapshot());
        }
    }

    pub(crate) fn has_unsaved_changes(&self) -> bool {
        self.document_revision != self.saved_document_revision
    }

    fn request_project_transition(&mut self, action: ProjectTransition) {
        if self.pending_project_transition.is_some() {
            return;
        }
        if self.has_unsaved_changes() {
            self.pending_project_transition = Some(PendingProjectTransition {
                action,
                phase: ProjectTransitionPhase::Confirm,
            });
        } else {
            self.perform_project_transition(action);
        }
    }

    fn perform_project_transition(&mut self, action: ProjectTransition) {
        match action {
            ProjectTransition::NewPart => self.new_design_now(),
            ProjectTransition::NewAssembly => self.new_assembly_now(),
            ProjectTransition::Open(path) => self.load_design_from_now(path),
            ProjectTransition::RecoverAutosave => self.recover_latest_autosave_now(),
            ProjectTransition::Exit => {
                self.allow_window_close = true;
                if let Some(ctx) = self.egui_ctx.as_ref() {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        }
    }

    pub(crate) fn request_window_close(&mut self) {
        self.request_project_transition(ProjectTransition::Exit);
    }

    /// Recalculates the geometry after a parametric history change (skipping
    /// hidden bodies).
    /// Snapshot the current `ParametricGraph` onto the undo stack (capped at 50)
    /// and clear the redo stack. Call before any destructive graph mutation.
    pub(crate) fn push_undo(&mut self) {
        let snap = self.snapshot();
        if self.undo_stack.len() >= 50 {
            self.undo_stack.remove(0);
        }
        self.undo_stack.push(snap);
        self.redo_stack.clear();
    }

    /// Restore the previous graph snapshot (Ctrl+Z).
    pub(crate) fn undo(&mut self) {
        if let Some(snap) = self.undo_stack.pop() {
            self.redo_stack.push(self.snapshot());
            self.restore_snapshot(snap);
            self.status_msg = "Undo.".to_string();
        } else {
            self.status_msg = "Nothing to undo.".to_string();
        }
    }

    /// Reapply the previously undone change (Ctrl+Y / Ctrl+Shift+Z).
    pub(crate) fn redo(&mut self) {
        if let Some(snap) = self.redo_stack.pop() {
            if self.undo_stack.len() >= 50 {
                self.undo_stack.remove(0);
            }
            self.undo_stack.push(self.snapshot());
            self.restore_snapshot(snap);
            self.status_msg = "Redo.".to_string();
        } else {
            self.status_msg = "Nothing to redo.".to_string();
        }
    }

    /// Replace the model with a fresh empty design (undoable).
    pub(crate) fn new_design(&mut self) {
        self.request_project_transition(ProjectTransition::NewPart);
    }

    fn new_design_now(&mut self) {
        log::info!("Creating new empty model.");
        if self.project_kind == ProjectKind::Part {
            self.push_undo();
        } else {
            // Part snapshots cannot be restored into an assembly workspace.
            self.undo_stack.clear();
            self.redo_stack.clear();
        }
        self.project_kind = ProjectKind::Part;
        self.reset_to_empty_project();
        self.document_revision = 0;
        self.saved_document_revision = 0;
        self.status_msg = "New blank design created.".to_string();
    }

    /// Enter a new, empty assembly workspace.
    ///
    /// Assembly persistence and occurrence editing intentionally remain behind
    /// their later milestones. This route is nevertheless real: part tools are
    /// unavailable and the renderer-facing scene is reset at the project
    /// boundary, ready for assembly instances to be attached.
    pub(crate) fn new_assembly(&mut self) {
        self.request_project_transition(ProjectTransition::NewAssembly);
    }

    fn new_assembly_now(&mut self) {
        log::info!("Creating new empty assembly.");
        self.undo_stack.clear();
        self.redo_stack.clear();
        self.project_kind = ProjectKind::Assembly;
        self.reset_to_empty_project();
        // This binary understands AssemblyRecipeV2. New workspaces therefore
        // author V2 explicitly even before the first mate is added.
        self.assembly_document.mates = Some(Default::default());
        self.document_revision = 0;
        self.saved_document_revision = 0;
        self.status_msg = "New blank assembly created.".to_string();
    }

    fn begin_project_replacement(&mut self) {
        self.workspace_generation = self.workspace_generation.wrapping_add(1);
        // A queued save has not captured its document yet, so carrying it into
        // the next project would save the wrong document. A dispatched save
        // already owns its snapshot and may finish safely in the background.
        if self
            .pending_save
            .as_ref()
            .is_some_and(|save| !save.dispatched)
        {
            self.pending_save = None;
        }

        // A late result from the part evaluator must never populate the new
        // workspace. `cancel` advances the worker generation immediately.
        self.evaluator.cancel();
        self.eval_generation = self.evaluator.current_generation();
        self.eval_pending = false;
        self.eval_started = None;
    }

    fn reset_to_empty_project(&mut self) {
        self.begin_project_replacement();

        self.document = Document::new();
        self.assembly_document = AssemblyDocument::new();
        self.assembly_definition_geometry.clear();
        self.assembly_scene_entities.clear();
        self.assembly_unresolved_definitions.clear();
        self.selected_assembly_occurrence = None;
        self.assembly_transform_edit_active = false;
        self.assembly_interactive_target = None;
        self.assembly_gizmo_drag = None;
        self.assembly_rename_occurrence = None;
        self.selected_assembly_mate = None;
        self.assembly_mate_statuses.clear();
        self.doc_created_unix = None;
        self.current_document_path = None;
        self.set_body_meshes(Vec::new());
        self.datum_values.clear();
        self.unresolved_features.clear();
        self.hidden_nodes.clear();
        self.error_msg = None;
        self.selected_node_id = None;
        self.feature_properties_dialog = None;
        self.pending_visual = None;
        self.pending_mirror_join_feedback = None;
        self.pending_extrude_visibility = None;
        self.reset_sketch_state();
        self.is_sketch_mode = false;
        self.is_plane_selection_mode = false;
        self.active_tool = None;
        self.active_sketch_face_ref = None;
        self.active_sketch_datum_ref = None;
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_sketch_points.clear();
        self.selected_body.clear();
        self.body_clipboard = None;
        self.move_op = None;
        self.combine_op = None;
        self.split_body_op = None;
        self.scale_body_op = None;
        self.move_preview_bodies = None;
        self.extrude_op = None;
        self.extrude_profile_pick_active = false;
        self.revolve_op = None;
        self.pattern_op = None;
        self.hole_op = None;
        self.shell_op = None;
        self.thread_op = None;
        self.sweep_op = None;
        self.draft_op = None;
        self.edge_mod_op = None;
        self.extrude_preview_cache = None;
        self.extrude_preview_mesh_cache = None;
        self.extrude_ghost_base = None;
        self.extrude_preview_inflight = None;
        self.extrude_preview_settle = None;
        self.edge_mod_preview_cache = None;
        self.edge_mod_preview_mesh_cache = None;
        self.edge_mod_arc_cache = None;
        self.edge_mod_arc_lru.clear();
        self.edge_mod_arc_inflight = None;
        self.edge_mod_arc_failed = None;
        self.extrude_depth_dragging = false;
        self.extrude_dim_pos = None;
        self.edge_mod_dim_pos = None;
        self.edge_mod_handle = None;
        self.inspection_dialog = None;
        self.section_view = None;
        self.parameters_dialog = None;
        self.save_dialog = None;
    }

    /// Open the in-app save dialog. The dialog presents a project title, format
    /// dropdown, recent folders, and a browse button.
    pub(crate) fn open_save_dialog(&mut self) {
        // Default directory: parent of last saved/opened project, else the
        // user's home / documents folder.
        let default_dir = self
            .recent_files
            .entries
            .first()
            .and_then(|e| e.path.parent().map(|p| p.to_path_buf()))
            .or_else(|| {
                std::env::var_os("USERPROFILE")
                    .or_else(|| std::env::var_os("HOME"))
                    .map(PathBuf::from)
            })
            .unwrap_or_else(|| PathBuf::from("."));

        // Default title: the stem of the most-recent project, or "Untitled".
        let default_title = self
            .current_document_path
            .as_ref()
            .and_then(|path| path.file_stem())
            .map(|stem| stem.to_string_lossy().into_owned())
            .unwrap_or_else(|| match self.project_kind {
                ProjectKind::Part => "Untitled".to_string(),
                ProjectKind::Assembly => "Untitled Assembly".to_string(),
            });

        self.save_dialog = Some(SaveDialogState {
            project_title: default_title,
            save_format: SaveFormat::ZcadLightweight,
            save_dir: default_dir,
        });
    }

    /// Execute the save using the current save-dialog parameters.
    pub(crate) fn do_save(&mut self) {
        let Some(current_state) = self.save_dialog.as_ref() else {
            return;
        };
        if let Some(error) = save_project_title_error(&current_state.project_title) {
            self.status_msg = error.to_string();
            return;
        }

        let mut state = match self.save_dialog.take() {
            Some(s) => s,
            None => return,
        };
        state.project_title = state.project_title.trim().to_string();

        let ext = state.save_format.extension();
        let file_name = format!("{}.{ext}", state.project_title);
        let path = state.save_dir.join(&file_name);
        self.pending_save = Some(PendingSave {
            path,
            profile: state.save_format.profile(self.hydrated_cache_mb),
            started: std::time::Instant::now(),
            dispatched: false,
            revision: None,
            workspace_generation: self.workspace_generation,
            project_kind: self.project_kind,
        });
        if let Some(pending) = self.pending_project_transition.as_mut() {
            if pending.phase == ProjectTransitionPhase::SaveDialog {
                pending.phase = ProjectTransitionPhase::Saving;
            }
        }
        self.status_msg = if self.eval_pending {
            "Save queued — waiting for the current model update…".to_string()
        } else {
            "Saving design…".to_string()
        };
        self.poll_document_worker();
    }

    pub(crate) fn poll_document_worker(&mut self) {
        let should_dispatch = self.pending_save.as_ref().is_some_and(|save| {
            !save.dispatched
                && (!self.eval_pending
                    || matches!(save.profile, zerocad_core::SaveProfile::Compact))
        });
        if should_dispatch {
            if self.project_kind == ProjectKind::Part
                && self.document.apply_legacy_reference_migrations()
            {
                self.document_revision = self.document_revision.wrapping_add(1);
                log::info!("Committed unique legacy reference backfills during explicit save");
            }
            let created_unix = *self.doc_created_unix.get_or_insert_with(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs()
            });
            let save = self.pending_save.as_mut().expect("pending save vanished");
            let document = match self.project_kind {
                ProjectKind::Part => {
                    let mut document = self.document.clone();
                    document.state.units = self.current_unit;
                    document.state.created_unix = Some(created_unix);
                    document.state.visibility.clear();
                    for hidden in &self.hidden_nodes {
                        document.set_visible(hidden.clone(), false);
                    }
                    ProjectDocument::Part(document)
                }
                ProjectKind::Assembly => {
                    let mut document = self.assembly_document.clone_authoritative();
                    document.presentation.units = self.current_unit;
                    document.created_unix = Some(created_unix);
                    ProjectDocument::Assembly(document)
                }
            };
            self.document_worker.submit(document_worker::SaveRequest {
                path: save.path.clone(),
                document,
                scene: self.evaluated_scene.shared(),
                profile: save.profile,
                cache: self.document.evaluation_cache_snapshot(),
            });
            save.dispatched = true;
            save.revision = Some(self.document_revision);
            self.status_msg = "Saving design…".to_string();
        }
        if let Some(done) = self.document_worker.try_recv() {
            let completed_save = self.pending_save.take();
            let saved_revision = completed_save.as_ref().and_then(|save| save.revision);
            let belongs_to_active_workspace = completed_save
                .as_ref()
                .is_some_and(|save| save.workspace_generation == self.workspace_generation);
            let mut transition_to_resume = None;
            match done.result {
                Ok(()) => {
                    let how = if matches!(done.profile, zerocad_core::SaveProfile::Compact) {
                        " (compact)"
                    } else {
                        " (hydrated)"
                    };
                    let completed_kind = completed_save
                        .as_ref()
                        .map(|save| save.project_kind)
                        .unwrap_or(self.project_kind);
                    self.recent_files.record(&done.path, completed_kind);
                    if belongs_to_active_workspace {
                        self.status_msg = format!("Design saved to {}{how}", done.path.display());
                        self.current_document_path = Some(done.path.clone());
                        if saved_revision == Some(self.document_revision) {
                            self.saved_document_revision = self.document_revision;
                            let snapshot = self.current_project_snapshot();
                            self.recovery.mark_saved(&snapshot);
                            if self
                                .pending_project_transition
                                .as_ref()
                                .is_some_and(|pending| {
                                    pending.phase == ProjectTransitionPhase::Saving
                                })
                            {
                                transition_to_resume = self
                                    .pending_project_transition
                                    .take()
                                    .map(|pending| pending.action);
                            }
                        } else if let Some(pending) = self.pending_project_transition.as_mut() {
                            if pending.phase == ProjectTransitionPhase::Saving {
                                pending.phase = ProjectTransitionPhase::Confirm;
                                self.status_msg =
                                    "The project changed while saving; review unsaved changes again."
                                        .to_string();
                            }
                        }
                    }
                    self.defer_onboarding_texture_eviction(&done.path);
                }
                Err(error) if belongs_to_active_workspace => {
                    self.status_msg = format!("Save failed: {error}");
                    if let Some(pending) = self.pending_project_transition.as_mut() {
                        if pending.phase == ProjectTransitionPhase::Saving {
                            pending.phase = ProjectTransitionPhase::Confirm;
                        }
                    }
                }
                Err(error) => log::warn!("Background save for prior project failed: {error}"),
            }
            if let Some(action) = transition_to_resume {
                self.perform_project_transition(action);
            }
        }
        let completions = {
            let mut queue = self
                .export_completions
                .lock()
                .expect("export queue poisoned");
            std::mem::take(&mut *queue)
        };
        if let Some(done) = completions.into_iter().last() {
            self.status_msg = done.message.clone();
            self.error_msg = done.error.then_some(done.message);
        }
    }

    pub(crate) fn recover_latest_autosave(&mut self) {
        self.request_project_transition(ProjectTransition::RecoverAutosave);
    }

    fn recover_latest_autosave_now(&mut self) {
        match self.recovery.load_latest() {
            Ok(ProjectDocument::Part(document)) => {
                self.begin_project_replacement();
                if self.project_kind == ProjectKind::Part {
                    self.push_undo();
                } else {
                    self.undo_stack.clear();
                    self.redo_stack.clear();
                }
                self.project_kind = ProjectKind::Part;
                self.current_unit = document.state.units;
                self.doc_created_unix = document.state.created_unix;
                self.hidden_nodes = document.hidden_entities();
                self.document = document;
                self.document.rebuild_node_map();
                self.selected_node_id = None;
                self.feature_properties_dialog = None;
                self.selected_faces.clear();
                self.selected_edges.clear();
                self.selected_sketch_points.clear();
                self.selected_body.clear();
                self.reevaluate_geometry();
                self.document_revision = 1;
                self.saved_document_revision = 0;
                self.status_msg =
                    "Recovered the latest crash-safe autosave. Save it to keep it permanently."
                        .to_string();
            }
            Ok(ProjectDocument::Assembly(document)) => {
                self.new_assembly_now();
                self.current_unit = document.presentation.units;
                self.doc_created_unix = document.created_unix;
                self.assembly_document = document;
                self.document_revision = 1;
                self.saved_document_revision = 0;
                self.status_msg =
                    "Recovered the latest assembly autosave. Save it to keep it permanently."
                        .to_string();
            }
            Err(error) => {
                self.status_msg = format!("Recovery failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
            }
        }
    }

    /// Render the in-app save dialog as a centered modal overlay.
    pub(crate) fn show_save_dialog(&mut self, ctx: &egui::Context) {
        if self.save_dialog.is_none() {
            return;
        }

        let pal = self.pal();
        let recent_folders = self.recent_files.recent_folders();
        let window_title = match self.project_kind {
            ProjectKind::Part => "Save design",
            ProjectKind::Assembly => "Save assembly",
        };
        let backdrop_id = egui::Id::new(SAVE_DIALOG_BACKDROP_ID);
        let window_id = egui::Id::new(SAVE_DIALOG_WINDOW_ID);
        let backdrop_layer = egui::LayerId::new(egui::Order::Foreground, backdrop_id);
        let window_layer = egui::LayerId::new(egui::Order::Foreground, window_id);

        // Treat the backdrop and window as one modal stack. Moving the backdrop
        // to the top dims every floating workspace panel; marking the window as
        // its sublayer keeps it above the clickable backdrop even after the
        // backdrop itself is clicked.
        ctx.set_sublayer(backdrop_layer, window_layer);
        ctx.move_to_top(backdrop_layer);
        let backdrop_response = egui::Area::new(backdrop_id)
            .order(egui::Order::Foreground)
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                ui.painter()
                    .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(145));
                ui.allocate_rect(screen, egui::Sense::click()).clicked()
            });
        let backdrop_clicked = backdrop_response.response.clicked() || backdrop_response.inner;

        let escape_pressed =
            ctx.input_mut(|input| input.consume_key(egui::Modifiers::NONE, egui::Key::Escape));

        let mut close = false;
        let mut do_save = false;
        let mut window_open = true;

        egui::Window::new(window_title)
            .id(window_id)
            .order(egui::Order::Foreground)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut window_open)
            .collapsible(false)
            .resizable(false)
            .movable(false)
            .default_width(SAVE_DIALOG_CONTENT_WIDTH)
            .min_width(SAVE_DIALOG_CONTENT_WIDTH)
            .max_width(SAVE_DIALOG_CONTENT_WIDTH)
            .frame(
                egui::Frame::window(&ctx.style())
                    .fill(pal.surface)
                    .stroke(egui::Stroke::new(1.0, pal.border))
                    .rounding(12.0)
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 10.0),
                        blur: 28.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(48),
                    })
                    .inner_margin(egui::Margin::symmetric(20.0, 18.0)),
            )
            .show(ctx, |ui| {
                // Keep every full-width control tied to this fixed content
                // column. Deriving their widths from `available_width()` can
                // create an egui feedback loop where a framed child makes the
                // auto-sized window a little wider on every repaint.
                ui.set_width(SAVE_DIALOG_CONTENT_WIDTH);
                ui.spacing_mut().item_spacing = egui::vec2(8.0, 8.0);
                let state = self.save_dialog.as_mut().unwrap();

                ui.horizontal(|ui| {
                    let (icon_rect, _) =
                        ui.allocate_exact_size(egui::vec2(42.0, 42.0), egui::Sense::hover());
                    ui.painter().rect_filled(icon_rect, 10.0, pal.accent_soft);
                    icons::Icon::Save.draw(
                        ui.painter(),
                        icon_rect.shrink2(egui::vec2(11.0, 11.0)),
                        pal.accent,
                    );
                    ui.vertical(|ui| {
                        ui.label(
                            egui::RichText::new("Save an editable ZeroCAD project")
                                .strong()
                                .size(16.0)
                                .color(pal.text_strong),
                        );
                        ui.label(
                            egui::RichText::new(
                                "Choose a project name, file type, and destination folder.",
                            )
                            .size(12.0)
                            .color(pal.text_muted),
                        );
                    });
                });

                ui.add_space(4.0);
                ui.separator();
                ui.add_space(4.0);

                ui.label(
                    egui::RichText::new("PROJECT NAME")
                        .strong()
                        .size(10.5)
                        .color(pal.text_muted),
                );
                ui.add_sized(
                    [SAVE_DIALOG_CONTENT_WIDTH, 34.0],
                    egui::TextEdit::singleline(&mut state.project_title)
                        .hint_text("e.g. Motor Bracket"),
                );
                let title_error = save_project_title_error(&state.project_title);
                if let Some(error) = title_error {
                    ui.label(egui::RichText::new(error).size(11.0).color(pal.danger));
                } else {
                    ui.label(
                        egui::RichText::new("The file extension is added automatically.")
                            .size(11.0)
                            .color(pal.text_faint),
                    );
                }

                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new("FILE TYPE")
                        .strong()
                        .size(10.5)
                        .color(pal.text_muted),
                );
                egui::ComboBox::from_id_salt("save_format")
                    .width(SAVE_DIALOG_CONTENT_WIDTH)
                    .selected_text(state.save_format.label())
                    .show_ui(ui, |ui: &mut egui::Ui| {
                        ui.selectable_value(
                            &mut state.save_format,
                            SaveFormat::ZcadLightweight,
                            SaveFormat::ZcadLightweight.label(),
                        );
                        ui.selectable_value(
                            &mut state.save_format,
                            SaveFormat::ZcadFull,
                            SaveFormat::ZcadFull.label(),
                        );
                    });
                let format_description = match state.save_format {
                    SaveFormat::ZcadLightweight => {
                        "Recommended for everyday work — compact, portable, and fully editable."
                    }
                    SaveFormat::ZcadFull => {
                        "Includes rebuild caches for faster reopening, with a larger file size."
                    }
                };
                ui.label(
                    egui::RichText::new(format_description)
                        .size(11.0)
                        .color(pal.text_faint),
                );

                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new("LOCATION")
                        .strong()
                        .size(10.5)
                        .color(pal.text_muted),
                );
                ui.horizontal(|ui| {
                    let mut display = state.save_dir.display().to_string();
                    let browse_width = 112.0;
                    let path_width =
                        (SAVE_DIALOG_CONTENT_WIDTH - browse_width - ui.spacing().item_spacing.x)
                            .max(220.0);
                    ui.add_sized(
                        [path_width, 34.0],
                        egui::TextEdit::singleline(&mut display).interactive(false),
                    );
                    if ui
                        .add_sized([browse_width, 34.0], egui::Button::new("Choose folder…"))
                        .clicked()
                    {
                        if let Some(dir) = rfd::FileDialog::new()
                            .set_title("Choose save folder")
                            .set_directory(&state.save_dir)
                            .pick_folder()
                        {
                            state.save_dir = dir;
                        }
                    }
                });

                if !recent_folders.is_empty() {
                    ui.add_space(2.0);
                    ui.label(
                        egui::RichText::new("RECENT FOLDERS")
                            .strong()
                            .size(10.5)
                            .color(pal.text_muted),
                    );
                    for folder in recent_folders.iter().take(3) {
                        let label = folder.display().to_string();
                        let selected = *folder == state.save_dir;
                        let button = egui::Button::new(
                            egui::RichText::new(&label).size(11.5).color(if selected {
                                pal.accent
                            } else {
                                pal.text_body
                            }),
                        )
                        .fill(if selected {
                            pal.accent_soft
                        } else {
                            pal.surface_subtle
                        })
                        .stroke(egui::Stroke::new(
                            1.0,
                            if selected { pal.accent } else { pal.border },
                        ));
                        if ui
                            .add_sized([SAVE_DIALOG_CONTENT_WIDTH, 30.0], button)
                            .on_hover_text(&label)
                            .clicked()
                        {
                            state.save_dir = folder.clone();
                        }
                    }
                }

                let full_path = state.save_dir.join(format!(
                    "{}.{}",
                    state.project_title,
                    state.save_format.extension()
                ));
                let full_path_label = full_path.display().to_string();
                ui.add_space(4.0);
                egui::Frame::none()
                    .fill(pal.surface_subtle)
                    .stroke(egui::Stroke::new(1.0, pal.border))
                    .rounding(8.0)
                    .inner_margin(egui::Margin::symmetric(12.0, 10.0))
                    .show(ui, |ui| {
                        ui.set_width(SAVE_DIALOG_DESTINATION_WIDTH);
                        ui.label(
                            egui::RichText::new("DESTINATION")
                                .strong()
                                .size(9.5)
                                .color(pal.text_muted),
                        );
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(&full_path_label)
                                    .monospace()
                                    .size(11.0)
                                    .color(pal.text_body),
                            )
                            .truncate(),
                        )
                        .on_hover_text(&full_path_label);
                    });

                ui.add_space(6.0);
                ui.separator();
                ui.add_space(4.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let primary_text = if self.dark_mode {
                        egui::Color32::from_rgb(15, 23, 42)
                    } else {
                        egui::Color32::WHITE
                    };
                    let save_button =
                        egui::Button::new(egui::RichText::new("Save").strong().color(primary_text))
                            .fill(pal.accent)
                            .stroke(egui::Stroke::NONE);
                    if ui
                        .add_enabled_ui(title_error.is_none(), |ui| {
                            ui.add_sized([104.0, 34.0], save_button)
                        })
                        .inner
                        .clicked()
                    {
                        do_save = true;
                    }
                    if ui
                        .add_sized([88.0, 34.0], egui::Button::new("Cancel"))
                        .clicked()
                    {
                        close = true;
                    }
                });
            });

        // Reassert after both areas have been registered this frame. This is
        // what prevents a click on the backdrop from promoting it over the
        // window on the following frame.
        ctx.set_sublayer(backdrop_layer, window_layer);
        close |= save_dialog_should_close(backdrop_clicked, escape_pressed, window_open);

        if do_save {
            self.do_save();
        } else if close {
            self.save_dialog = None;
        }
    }

    /// Record `path` in the recent-projects list and (re)bake a thumbnail of the
    /// currently-evaluated bodies for the onboarding screen. Called after a
    /// successful save/open, when `body_meshes` reflects `path`'s model.
    pub(crate) fn remember_project(&mut self, path: &Path) {
        self.recent_files.record(path, self.project_kind);
        if !self.evaluated_scene.is_empty() {
            let (w, h, rgba) = thumbnail::render_thumbnail(&self.evaluated_scene, 256);
            settings::save_thumb(path, w, h, &rgba);
        }
        // Reload this thumbnail on a later frame. It may already have been
        // painted this frame when the project was opened from its Recent card.
        self.defer_onboarding_texture_eviction(path);
    }

    /// Mark a stale onboarding thumbnail for release at the start of the next
    /// frame. Dropping the last handle in the frame that painted it can make
    /// egui-wgpu destroy the texture before wgpu submits that frame.
    fn defer_onboarding_texture_eviction(&mut self, path: &Path) {
        self.pending_onboarding_texture_evictions
            .insert(path.to_path_buf());
    }

    pub(crate) fn flush_onboarding_texture_evictions(&mut self) {
        for path in self.pending_onboarding_texture_evictions.drain() {
            self.onboarding_textures.remove(&path);
        }
    }

    /// Fetch (uploading once, then caching) the egui texture for a project's
    /// cached thumbnail, or `None` if there's no `.thumb` for it yet.
    pub(crate) fn thumb_texture(
        &mut self,
        ctx: &egui::Context,
        path: &Path,
    ) -> Option<egui::TextureHandle> {
        if let Some(t) = self.onboarding_textures.get(path) {
            return Some(t.clone());
        }
        let (w, h, rgba) = settings::load_thumb(path)?;
        let image = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
        let tex = ctx.load_texture(
            format!("thumb_{}", path.display()),
            image,
            egui::TextureOptions::LINEAR,
        );
        self.onboarding_textures
            .insert(path.to_path_buf(), tex.clone());
        Some(tex)
    }

    /// Dedicated start page shown before a document is opened. Unlike the old
    /// modal onboarding card this is a real application page: workspace panels
    /// are not constructed behind it and no modeling input can leak through.
    pub(crate) fn draw_start_page(&mut self, ctx: &egui::Context) {
        let pal = self.pal();
        let recents: Vec<(PathBuf, String, Option<ProjectKind>)> = self
            .recent_files
            .entries
            .iter()
            .take(4)
            .map(|entry| {
                let name = entry
                    .path
                    .file_stem()
                    .map(|value| value.to_string_lossy().into_owned())
                    .unwrap_or_else(|| entry.path.to_string_lossy().into_owned());
                (entry.path.clone(), name, entry.project_kind)
            })
            .collect();
        let textures: Vec<Option<egui::TextureHandle>> = recents
            .iter()
            .map(|(path, _, _)| self.thumb_texture(ctx, path))
            .collect();

        let mut create_part = false;
        let mut create_assembly = false;
        let mut open_project = false;
        let mut open_recent = None;

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(pal.surface_subtle))
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let available_width = ui.available_width();
                    let content_width = (available_width - 64.0).max(320.0).min(1080.0);
                    let left_pad = ((available_width - content_width) * 0.5).max(0.0);
                    ui.horizontal(|ui| {
                        ui.add_space(left_pad);
                        ui.allocate_ui_with_layout(
                            egui::vec2(content_width, 0.0),
                            egui::Layout::top_down(egui::Align::Min),
                            |ui| {
                                // `allocate_ui_with_layout` inherits the parent's
                                // clip bounds, so explicitly constrain the child.
                                // Without this, multi-column rows can size against
                                // the scroll area's virtual width and run offscreen.
                                ui.set_width(content_width);
                                ui.add_space(42.0);
                                ui.horizontal(|ui| {
                                    ui.spacing_mut().item_spacing.x = 1.0;
                                    ui.label(
                                        egui::RichText::new("Welcome to ")
                                            .size(30.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.label(
                                        egui::RichText::new("Zero")
                                            .strong()
                                            .size(30.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.label(
                                        egui::RichText::new("CAD")
                                            .strong()
                                            .size(30.0)
                                            .color(pal.accent),
                                    );
                                });
                                ui.label(
                                    egui::RichText::new("Design, model, and build with precision.")
                                        .size(14.0)
                                        .color(pal.text_muted),
                                );
                                ui.add_space(24.0);
                                ui.label(
                                    egui::RichText::new("Start a Project")
                                        .strong()
                                        .size(15.0)
                                        .color(pal.text_strong),
                                );
                                ui.add_space(8.0);

                                ui.columns(4, |columns| {
                                    create_part = Self::project_type_card(
                                        &mut columns[0],
                                        &pal,
                                        icons::Icon::Cube,
                                        "Part",
                                        "Create a new 3D model.",
                                        true,
                                        false,
                                    )
                                    .clicked();
                                    create_assembly = Self::project_type_card(
                                        &mut columns[1],
                                        &pal,
                                        icons::Icon::Assembly,
                                        "Assembly",
                                        "Combine parts and sub-assemblies.",
                                        true,
                                        false,
                                    )
                                    .clicked();
                                    Self::project_type_card(
                                        &mut columns[2],
                                        &pal,
                                        icons::Icon::Sheet,
                                        "Sheet / Sketch",
                                        "Create 2D sketches and drawings.",
                                        false,
                                        true,
                                    );
                                    open_project = Self::project_type_card(
                                        &mut columns[3],
                                        &pal,
                                        icons::Icon::Folder,
                                        "Open Project",
                                        "Open an existing ZeroCAD file.",
                                        true,
                                        false,
                                    )
                                    .clicked();
                                });

                                ui.add_space(20.0);
                                ui.columns(2, |columns| {
                                    Self::start_panel(
                                        &mut columns[0],
                                        &pal,
                                        "Recent Projects",
                                        |ui| {
                                            if recents.is_empty() {
                                                ui.add_space(24.0);
                                                ui.label(
                                                    egui::RichText::new("No recent projects yet.")
                                                        .size(13.0)
                                                        .color(pal.text_muted),
                                                );
                                                ui.label(
                                                egui::RichText::new(
                                                    "Saved and opened projects will appear here.",
                                                )
                                                .size(12.0)
                                                .color(pal.text_faint),
                                            );
                                                ui.add_space(24.0);
                                            } else {
                                                for ((path, name, kind), texture) in
                                                    recents.iter().zip(textures.iter())
                                                {
                                                    if Self::recent_project_row(
                                                        ui,
                                                        &pal,
                                                        path,
                                                        name,
                                                        *kind,
                                                        texture.as_ref(),
                                                    )
                                                    .clicked()
                                                    {
                                                        open_recent = Some(path.clone());
                                                    }
                                                }
                                            }
                                        },
                                    );

                                    Self::start_panel(&mut columns[1], &pal, "Get Started", |ui| {
                                        for (icon, title, description) in [
                                            (
                                                icons::Icon::Help,
                                                "Learn the Basics",
                                                "Step-by-step tutorials and guides.",
                                            ),
                                            (
                                                icons::Icon::Settings,
                                                "Keyboard Shortcuts",
                                                "View and customize shortcuts.",
                                            ),
                                            (
                                                icons::Icon::Cube,
                                                "Example Projects",
                                                "Explore sample models and projects.",
                                            ),
                                        ] {
                                            Self::coming_soon_row(
                                                ui,
                                                &pal,
                                                icon,
                                                title,
                                                description,
                                            );
                                        }
                                    });
                                });

                                ui.add_space(18.0);
                                ui.horizontal_centered(|ui| {
                                    ui.checkbox(
                                        &mut self.show_onboarding,
                                        "Show this page on startup",
                                    );
                                });
                                ui.add_space(24.0);
                            },
                        );
                    });
                });
            });

        if create_part {
            self.new_design();
            self.onboarding_visible = false;
        } else if create_assembly {
            self.new_assembly();
            self.onboarding_visible = false;
        } else if open_project {
            if let Some(path) = rfd::FileDialog::new()
                .set_title("Open ZeroCAD Design")
                .add_filter("ZeroCAD Design", &["zcad", "zcadh"])
                .pick_file()
            {
                self.load_design_from(path.clone());
                if self.current_document_path.as_ref() == Some(&path) {
                    self.onboarding_visible = false;
                }
            }
        } else if let Some(path) = open_recent {
            self.load_design_from(path.clone());
            if self.current_document_path.as_ref() == Some(&path) {
                self.onboarding_visible = false;
            }
        }
    }

    fn project_type_card(
        ui: &mut egui::Ui,
        pal: &Palette,
        icon: icons::Icon,
        title: &str,
        description: &str,
        enabled: bool,
        coming_soon: bool,
    ) -> egui::Response {
        let size = egui::vec2(ui.available_width(), 100.0);
        let sense = if enabled {
            egui::Sense::click()
        } else {
            egui::Sense::hover()
        };
        let (rect, response) = ui.allocate_exact_size(size, sense);
        let hovered = enabled && response.hovered();
        let border = if hovered || (enabled && title == "Part") {
            pal.accent
        } else {
            pal.border
        };
        ui.painter().rect(
            rect,
            7.0,
            if hovered {
                pal.accent_soft
            } else {
                pal.surface
            },
            egui::Stroke::new(1.0, border),
        );
        let content = rect.shrink2(egui::vec2(15.0, 13.0));
        let icon_rect = egui::Rect::from_min_size(content.min, egui::vec2(28.0, 28.0));
        let foreground = if enabled {
            pal.text_strong
        } else {
            pal.text_faint
        };
        icon.draw(
            ui.painter(),
            icon_rect,
            if enabled { pal.accent } else { pal.text_faint },
        );
        ui.painter().text(
            egui::pos2(icon_rect.right() + 12.0, content.top() + 2.0),
            egui::Align2::LEFT_TOP,
            title,
            egui::FontId::proportional(13.5),
            foreground,
        );
        ui.painter().text(
            egui::pos2(icon_rect.right() + 12.0, content.top() + 27.0),
            egui::Align2::LEFT_TOP,
            description,
            egui::FontId::proportional(11.0),
            if enabled {
                pal.text_muted
            } else {
                pal.text_faint
            },
        );
        if coming_soon {
            let badge = egui::Rect::from_min_size(
                egui::pos2(content.right() - 82.0, content.bottom() - 23.0),
                egui::vec2(82.0, 20.0),
            );
            ui.painter().rect(
                badge,
                4.0,
                pal.surface_subtle,
                egui::Stroke::new(1.0, pal.border),
            );
            ui.painter().text(
                badge.center(),
                egui::Align2::CENTER_CENTER,
                "COMING SOON",
                egui::FontId::proportional(9.5),
                pal.text_faint,
            );
        }
        response
    }

    fn start_panel(
        ui: &mut egui::Ui,
        pal: &Palette,
        title: &str,
        content: impl FnOnce(&mut egui::Ui),
    ) {
        egui::Frame::none()
            .fill(pal.surface)
            .stroke(egui::Stroke::new(1.0, pal.border))
            .rounding(8.0)
            .inner_margin(egui::Margin::same(18.0))
            .show(ui, |ui| {
                ui.set_min_height(310.0);
                ui.label(
                    egui::RichText::new(title)
                        .strong()
                        .size(15.0)
                        .color(pal.text_strong),
                );
                ui.add_space(10.0);
                content(ui);
            });
    }

    fn recent_project_row(
        ui: &mut egui::Ui,
        pal: &Palette,
        path: &Path,
        name: &str,
        project_kind: Option<ProjectKind>,
        texture: Option<&egui::TextureHandle>,
    ) -> egui::Response {
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 66.0), egui::Sense::click());
        if response.hovered() {
            ui.painter().rect_filled(rect, 5.0, pal.accent_soft);
        }
        ui.painter().line_segment(
            [rect.left_bottom(), rect.right_bottom()],
            egui::Stroke::new(1.0, pal.border),
        );
        let image_rect = egui::Rect::from_min_size(
            rect.left_top() + egui::vec2(4.0, 6.0),
            egui::vec2(58.0, 52.0),
        );
        if let Some(texture) = texture {
            ui.painter().image(
                texture.id(),
                image_rect,
                egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        } else {
            ui.painter().rect(
                image_rect,
                5.0,
                pal.surface_subtle,
                egui::Stroke::new(1.0, pal.border),
            );
            let icon = match project_kind {
                Some(ProjectKind::Assembly) => icons::Icon::Assembly,
                _ => icons::Icon::Cube,
            };
            icon.draw(ui.painter(), image_rect.shrink(15.0), pal.text_faint);
        }
        let text_left = image_rect.right() + 14.0;
        ui.painter().text(
            egui::pos2(text_left, rect.top() + 12.0),
            egui::Align2::LEFT_TOP,
            name,
            egui::FontId::proportional(13.0),
            pal.text_strong,
        );
        ui.painter().text(
            egui::pos2(text_left, rect.top() + 36.0),
            egui::Align2::LEFT_TOP,
            path.to_string_lossy(),
            egui::FontId::proportional(10.5),
            pal.text_faint,
        );
        if let Some(kind) = project_kind {
            ui.painter().text(
                egui::pos2(rect.right() - 8.0, rect.top() + 12.0),
                egui::Align2::RIGHT_TOP,
                match kind {
                    ProjectKind::Part => "PART",
                    ProjectKind::Assembly => "ASSEMBLY",
                },
                egui::FontId::proportional(9.5),
                pal.text_muted,
            );
        }
        response
    }

    fn coming_soon_row(
        ui: &mut egui::Ui,
        pal: &Palette,
        icon: icons::Icon,
        title: &str,
        description: &str,
    ) {
        let (rect, _) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 80.0), egui::Sense::hover());
        ui.painter().line_segment(
            [rect.left_bottom(), rect.right_bottom()],
            egui::Stroke::new(1.0, pal.border),
        );
        let icon_rect = egui::Rect::from_min_size(
            rect.left_top() + egui::vec2(4.0, 17.0),
            egui::vec2(28.0, 28.0),
        );
        icon.draw(ui.painter(), icon_rect, pal.text_faint);
        ui.painter().text(
            egui::pos2(icon_rect.right() + 12.0, rect.top() + 12.0),
            egui::Align2::LEFT_TOP,
            title,
            egui::FontId::proportional(13.0),
            pal.text_muted,
        );
        ui.painter().text(
            egui::pos2(icon_rect.right() + 12.0, rect.top() + 36.0),
            egui::Align2::LEFT_TOP,
            description,
            egui::FontId::proportional(10.5),
            pal.text_faint,
        );
        let badge = egui::Rect::from_min_size(
            egui::pos2(rect.right() - 82.0, rect.top() + 26.0),
            egui::vec2(78.0, 20.0),
        );
        ui.painter().rect(
            badge,
            4.0,
            pal.surface_subtle,
            egui::Stroke::new(1.0, pal.border),
        );
        ui.painter().text(
            badge.center(),
            egui::Align2::CENTER_CENTER,
            "COMING SOON",
            egui::FontId::proportional(9.0),
            pal.text_faint,
        );
    }

    /// The centered Welcome modal: New / Open / Recent. Drawn over a dimmed,
    /// click-swallowing backdrop so the workspace beneath is inert. A no-op
    /// unless `onboarding_visible`. Esc, the Close button, or choosing any action
    /// dismisses it (without touching the persisted "show on startup" preference,
    /// which the footer checkbox edits separately).
    #[allow(dead_code)] // Kept temporarily for compatibility with older UI entry points.
    pub(crate) fn draw_onboarding(&mut self, ctx: &egui::Context) {
        if !self.onboarding_visible {
            return;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.onboarding_visible = false;
            return;
        }

        let pal = self.pal();

        // Dim + swallow input to the workspace behind the card (Middle sits above
        // the Background panels but below the Foreground card). Clicking the
        // backdrop — anywhere outside the card, since the card sits on top and
        // consumes clicks over its own rect — dismisses onboarding so the user
        // drops straight into the (blank) workspace and can start modeling.
        let backdrop_clicked = egui::Area::new(egui::Id::new("onboarding_dim"))
            .order(egui::Order::Middle)
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                let resp = ui.allocate_rect(screen, egui::Sense::click_and_drag());
                ui.painter()
                    .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(120));
                resp.clicked()
            })
            .inner;

        // Top 5 recents, snapshotted (path + display name) so the draw closure
        // borrows locals, not `self.recent_files`. Textures are pre-loaded for
        // the same reason (uploading mutably borrows `self`).
        let recents: Vec<(PathBuf, String)> = self
            .recent_files
            .entries
            .iter()
            .take(5)
            .map(|e| {
                let name = e
                    .path
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| e.path.to_string_lossy().into_owned());
                (e.path.clone(), name)
            })
            .collect();
        let textures: Vec<Option<egui::TextureHandle>> = recents
            .iter()
            .map(|(p, _)| self.thumb_texture(ctx, p))
            .collect();

        let mut do_new = false;
        let mut do_open = false;
        let mut open_recent: Option<PathBuf> = None;
        let mut close = false;

        // The window defaults to Order::Middle and is registered after the dim
        // Area (also Middle), so it draws on top of the dim — and both sit above
        // the Background-order workspace panels.
        egui::Window::new("onboarding_window")
            .title_bar(false)
            .collapsible(false)
            .resizable(false)
            .movable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .fixed_size(egui::vec2(560.0, 430.0))
            .frame(egui::Frame::window(&ctx.style()).inner_margin(egui::Margin::same(22.0)))
            .show(ctx, |ui| {
                // Brand title.
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 1.0;
                    ui.label(
                        egui::RichText::new("Welcome to ")
                            .size(22.0)
                            .color(pal.text_strong),
                    );
                    ui.label(
                        egui::RichText::new("Zero")
                            .strong()
                            .size(22.0)
                            .color(pal.text_strong),
                    );
                    ui.label(
                        egui::RichText::new("CAD")
                            .strong()
                            .size(22.0)
                            .color(egui::Color32::from_rgb(37, 99, 235)),
                    );
                });
                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new(
                        "Start a new project, open one, or pick up where you left off.",
                    )
                    .size(13.0)
                    .color(pal.text_muted),
                );
                ui.add_space(16.0);

                // New / Open.
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 10.0;
                    if icons::Icon::New
                        .labeled_button(
                            ui,
                            "New Project",
                            egui::Color32::from_rgb(37, 99, 235),
                            egui::Color32::from_rgb(29, 78, 216),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        )
                        .clicked()
                    {
                        do_new = true;
                    }
                    if icons::Icon::Folder
                        .labeled_button(
                            ui,
                            "Open Project",
                            egui::Color32::from_rgb(241, 245, 249),
                            egui::Color32::from_rgb(226, 232, 240),
                            pal.text_body,
                            egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                        )
                        .clicked()
                    {
                        do_open = true;
                    }
                });

                ui.add_space(16.0);
                ui.separator();
                ui.add_space(8.0);
                ui.label(
                    egui::RichText::new("Recent")
                        .strong()
                        .size(14.0)
                        .color(pal.text_strong),
                );
                ui.add_space(8.0);

                if recents.is_empty() {
                    ui.add_space(20.0);
                    ui.vertical_centered(|ui| {
                        ui.label(
                            egui::RichText::new("No recent projects yet.")
                                .size(13.0)
                                .color(pal.text_muted),
                        );
                        ui.label(
                            egui::RichText::new("Saved and opened projects will appear here.")
                                .size(12.0)
                                .color(pal.text_muted),
                        );
                    });
                } else {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 10.0;
                        for ((path, name), tex) in recents.iter().zip(textures.iter()) {
                            if Self::recent_card(ui, &pal, name, tex.as_ref())
                                .on_hover_text(path.to_string_lossy())
                                .clicked()
                            {
                                open_recent = Some(path.clone());
                            }
                        }
                    });
                }

                ui.add_space(16.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_onboarding, "Show on startup");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add(egui::Button::new("Close").min_size(egui::vec2(80.0, 28.0)))
                            .clicked()
                        {
                            close = true;
                        }
                    });
                });
            });

        // Apply deferred actions (outside the borrow of the draw closure).
        if do_new {
            self.new_design();
            self.onboarding_visible = false;
        } else if do_open {
            self.open_design();
            self.onboarding_visible = false;
        } else if let Some(path) = open_recent {
            self.load_design_from(path);
            self.onboarding_visible = false;
        } else if close || backdrop_clicked {
            // Close button, or a click anywhere off the card: dismiss and let the
            // user model in the current (blank-on-startup) workspace.
            self.onboarding_visible = false;
        }
    }

    /// One clickable Recent card: thumbnail (or placeholder) above the project
    /// name, with a hover highlight. Returns its click response.
    #[allow(dead_code)] // Paired with the retained legacy onboarding renderer above.
    pub(crate) fn recent_card(
        ui: &mut egui::Ui,
        pal: &Palette,
        name: &str,
        tex: Option<&egui::TextureHandle>,
    ) -> egui::Response {
        const CARD: egui::Vec2 = egui::vec2(96.0, 120.0);
        const IMG: f32 = 84.0;
        let (rect, resp) = ui.allocate_exact_size(CARD, egui::Sense::click());
        if !ui.is_rect_visible(rect) {
            return resp;
        }
        let painter = ui.painter();
        let hovered = resp.hovered();
        painter.rect(
            rect,
            6.0,
            if hovered {
                egui::Color32::from_rgb(226, 232, 240)
            } else {
                egui::Color32::TRANSPARENT
            },
            if hovered {
                egui::Stroke::new(1.0, egui::Color32::from_rgb(37, 99, 235))
            } else {
                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225))
            },
        );
        let img_rect = egui::Rect::from_min_size(
            egui::pos2(rect.center().x - IMG * 0.5, rect.top() + 6.0),
            egui::vec2(IMG, IMG),
        );
        match tex {
            Some(t) => {
                painter.image(
                    t.id(),
                    img_rect,
                    egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                    egui::Color32::WHITE,
                );
            }
            None => {
                painter.rect_filled(img_rect, 4.0, egui::Color32::from_rgb(238, 241, 245));
                let icon = egui::Rect::from_center_size(img_rect.center(), egui::vec2(28.0, 28.0));
                icons::Icon::Sketch.draw(painter, icon, pal.text_muted);
            }
        }
        // Project name, truncated to fit.
        let label: String = if name.chars().count() > 13 {
            format!("{}…", name.chars().take(12).collect::<String>())
        } else {
            name.to_string()
        };
        painter.text(
            egui::pos2(rect.center().x, img_rect.bottom() + 8.0),
            egui::Align2::CENTER_TOP,
            label,
            egui::FontId::proportional(12.0),
            pal.text_body,
        );
        resp
    }

    /// Prompt for a `.zcad` file and load it, replacing the current model.
    pub(crate) fn open_design(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Open ZeroCAD Design")
            .add_filter("ZeroCAD Design", &["zcad", "zcadh"])
            .pick_file()
        else {
            return;
        };
        self.load_design_from(path);
    }

    /// Load the `.zcad` file at `path`, replacing the current model (undoable).
    /// Selection / preview state is reset to match the new graph. Shared by the
    /// Open dialog and the onboarding Recent list.
    pub(crate) fn load_design_from(&mut self, path: PathBuf) {
        self.request_project_transition(ProjectTransition::Open(path));
    }

    pub(crate) fn show_unsaved_changes_dialog(&mut self, ctx: &egui::Context) {
        let Some(pending) = self.pending_project_transition.as_ref() else {
            return;
        };
        if pending.phase == ProjectTransitionPhase::SaveDialog {
            return;
        }

        let phase = pending.phase;
        let destination = match &pending.action {
            ProjectTransition::NewPart => "create a new part".to_owned(),
            ProjectTransition::NewAssembly => "create a new assembly".to_owned(),
            ProjectTransition::Open(path) => format!("open {}", path.display()),
            ProjectTransition::RecoverAutosave => "recover the autosave".to_owned(),
            ProjectTransition::Exit => "exit ZeroCAD".to_owned(),
        };

        let backdrop_id = egui::Id::new(UNSAVED_CHANGES_BACKDROP_ID);
        let window_id = egui::Id::new(UNSAVED_CHANGES_WINDOW_ID);
        let backdrop_layer = egui::LayerId::new(egui::Order::Foreground, backdrop_id);
        let window_layer = egui::LayerId::new(egui::Order::Foreground, window_id);

        // Keep the confirmation above its backdrop even after the backdrop is
        // clicked. Without an explicit layer relationship, egui promotes the
        // backdrop and can leave the dialog dimmed and unreachable.
        ctx.set_sublayer(backdrop_layer, window_layer);
        ctx.move_to_top(backdrop_layer);
        let backdrop_response = egui::Area::new(backdrop_id)
            .order(egui::Order::Foreground)
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                ui.painter()
                    .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(140));
                ui.allocate_rect(screen, egui::Sense::click()).clicked()
            });
        let backdrop_clicked = backdrop_response.response.clicked() || backdrop_response.inner;
        let escape_pressed = phase == ProjectTransitionPhase::Confirm
            && ctx.input_mut(|input| input.consume_key(egui::Modifiers::NONE, egui::Key::Escape));

        let mut save = false;
        let mut discard = false;
        let mut cancel = false;
        let mut window_open = true;
        let window = egui::Window::new("Unsaved changes")
            .id(window_id)
            .order(egui::Order::Foreground)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .collapsible(false)
            .resizable(false)
            .min_width(420.0);
        let window = if phase == ProjectTransitionPhase::Confirm {
            window.open(&mut window_open)
        } else {
            window
        };
        window.show(ctx, |ui| {
            if phase == ProjectTransitionPhase::Saving {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Saving the current project…");
                });
                return;
            }
            ui.label(format!(
                "Save changes to the current project before you {destination}?"
            ));
            ui.add_space(12.0);
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    save = true;
                }
                if ui.button("Discard").clicked() {
                    discard = true;
                }
                if ui.button("Cancel").clicked() {
                    cancel = true;
                }
            });
        });

        // Reassert after both areas are registered so a backdrop interaction
        // cannot reorder it above the confirmation on the following frame.
        ctx.set_sublayer(backdrop_layer, window_layer);
        cancel |=
            unsaved_changes_should_cancel(phase, backdrop_clicked, escape_pressed, window_open);

        if save {
            self.open_save_dialog();
            if self.save_dialog.is_some() {
                if let Some(pending) = self.pending_project_transition.as_mut() {
                    pending.phase = ProjectTransitionPhase::SaveDialog;
                }
            }
        } else if discard {
            if let Some(pending) = self.pending_project_transition.take() {
                self.perform_project_transition(pending.action);
            }
        } else if cancel {
            self.pending_project_transition = None;
        }
    }

    pub(crate) fn reconcile_transition_save_dialog(&mut self) {
        let save_dialog_was_cancelled = self
            .pending_project_transition
            .as_ref()
            .is_some_and(|pending| pending.phase == ProjectTransitionPhase::SaveDialog)
            && self.save_dialog.is_none()
            && self.pending_save.is_none();
        if save_dialog_was_cancelled {
            if let Some(pending) = self.pending_project_transition.as_mut() {
                pending.phase = ProjectTransitionPhase::Confirm;
            }
        }
    }

    fn load_design_from_now(&mut self, path: PathBuf) {
        let loaded = match zerocad_core::read_project_document_file(
            &path,
            &zerocad_core::LoadOptions::default(),
        ) {
            Ok(l) => l,
            Err(e) => {
                self.status_msg = format!("Load failed: {e}");
                return;
            }
        };
        let zerocad_core::LoadedProjectDocument {
            document,
            accelerators,
            diagnostics,
            ..
        } = loaded;
        let document = match document {
            ProjectDocument::Part(document) => document,
            ProjectDocument::Assembly(document) => {
                self.load_assembly_from_now(path, document, accelerators, diagnostics);
                return;
            }
        };

        self.begin_project_replacement();

        if self.project_kind == ProjectKind::Part {
            self.push_undo();
        } else {
            self.undo_stack.clear();
            self.redo_stack.clear();
        }
        self.project_kind = ProjectKind::Part;
        self.hidden_nodes = document.hidden_entities();
        self.current_unit = document.state.units;
        self.doc_created_unix = document.state.created_unix;
        self.document = document;
        self.current_document_path = Some(path.clone());
        // Continue the user-facing feature id sequence after the largest loaded
        // suffix. Dependencies and semantic timelines determine evaluation.
        self.reseed_id_counter_from_graph();
        self.selected_node_id = None;
        self.feature_properties_dialog = None;
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_sketch_points.clear();
        self.selected_body.clear();
        self.extrude_op = None;
        self.extrude_profile_pick_active = false;
        self.edge_mod_op = None;
        self.body_clipboard = None;
        self.move_op = None;
        self.combine_op = None;
        self.move_preview_bodies = None;

        // Show the embedded geometry cache immediately (instant open). It's only
        // present when fresh (its hash matched the loaded graph), so it's safe to
        // display; `reevaluate_geometry` then swaps in freshly-computed bodies.
        let had_mesh_cache = accelerators.display_meshes.is_some();
        if let Some(cache) = accelerators.display_meshes {
            self.set_body_meshes(cache);
        }
        let had_evaluation_cache = accelerators.evaluation_cache.is_some();
        if let Some(cache) = accelerators.evaluation_cache {
            self.document.install_evaluation_cache(cache);
        }
        // Seed the onboarding thumbnail cache from the file's embedded preview so
        // a `.zcad` from another machine shows its real thumbnail even if it has
        // no geometry to re-render (e.g. evaluation fails).
        if let Some(png) = &accelerators.small_preview_png {
            if let Some((w, h, rgba)) = thumbnail::decode_png(png) {
                settings::save_thumb(&path, w, h, &rgba);
                self.defer_onboarding_texture_eviction(&path);
            }
        }

        // Regenerate from the recipe (authoritative). On failure, the cached
        // bodies above remain on screen so the model is never lost.
        self.pending_visual = None;
        if !(had_mesh_cache && had_evaluation_cache) {
            self.reevaluate_geometry();
        }
        self.status_msg = if diagnostics.is_empty() {
            format!("Design loaded from {}", path.display())
        } else {
            format!(
                "Design loaded from {} with {} recoverable warning(s)",
                path.display(),
                diagnostics.len()
            )
        };
        self.remember_project(&path);
        let snapshot = self.current_project_snapshot();
        self.recovery.mark_saved(&snapshot);
        self.document_revision = 0;
        self.saved_document_revision = 0;
        self.onboarding_visible = false;
    }

    fn load_assembly_from_now(
        &mut self,
        path: PathBuf,
        document: AssemblyDocument,
        accelerators: zerocad_core::HydrationBundle,
        diagnostics: Vec<zerocad_core::LoadDiagnostic>,
    ) {
        self.new_assembly_now();
        self.current_unit = document.presentation.units;
        self.doc_created_unix = document.created_unix;
        self.assembly_document = document;
        self.current_document_path = Some(path.clone());
        self.hydrate_assembly_definitions();
        if let Some(png) = accelerators.small_preview_png {
            if let Some((w, h, rgba)) = thumbnail::decode_png(&png) {
                settings::save_thumb(&path, w, h, &rgba);
                self.defer_onboarding_texture_eviction(&path);
            }
        }
        self.status_msg = if diagnostics.is_empty() {
            format!("Assembly loaded from {}", path.display())
        } else {
            format!(
                "Assembly loaded from {} with {} recoverable warning(s)",
                path.display(),
                diagnostics.len()
            )
        };
        self.remember_project(&path);
        let snapshot = self.current_project_snapshot();
        self.recovery.mark_saved(&snapshot);
        self.document_revision = 0;
        self.saved_document_revision = 0;
        self.onboarding_visible = false;
    }

    /// Prompt for a path and write all current bodies as one binary STL mesh.
    /// STL is a triangle soup (no history/units), so this is export-only — the
    /// editable document stays the `.zcad` JSON.
    pub(crate) fn export_stl(&mut self) {
        if self.body_meshes.is_empty() {
            self.status_msg = "Nothing to export — the model has no solid bodies.".to_string();
            return;
        }
        // Export the final arc geometry, not a faceted draft mid-refine.
        if self.eval_pending {
            self.status_msg = "Export waits for the current model update to finish.".to_string();
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export STL")
            .add_filter("STL mesh", &["stl"])
            .save_file()
        else {
            return;
        };
        let bodies = self.body_meshes.clone();
        let completions = self.export_completions.clone();
        let repaint = self.egui_ctx.clone();
        self.status_msg = "Exporting STL…".to_string();
        std::thread::spawn(move || {
            let bytes = zerocad_core::meshes_to_binary_stl(bodies.iter().map(|(_, m)| m));
            let tris = bytes.len().saturating_sub(84) / 50;
            let (message, error) = match std::fs::write(&path, bytes) {
                Ok(()) => (
                    format!("Exported {tris} triangles to {}", path.display()),
                    false,
                ),
                Err(error) => (format!("STL export failed: {error}"), true),
            };
            completions
                .lock()
                .expect("export queue poisoned")
                .push(ExportCompletion { message, error });
            if let Some(ctx) = repaint {
                ctx.request_repaint();
            }
        });
    }

    /// Prompt for a path and write all current bodies as a 3MF package (one
    /// object per body, so multi-part designs slice as separate parts). Like
    /// STL this is export-only; the editable document stays the `.zcad`.
    pub(crate) fn export_3mf(&mut self) {
        if self.body_meshes.is_empty() {
            self.status_msg = "Nothing to export — the model has no solid bodies.".to_string();
            return;
        }
        if self.eval_pending {
            self.status_msg = "Export waits for the current model update to finish.".to_string();
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export 3MF")
            .add_filter("3MF model", &["3mf"])
            .save_file()
        else {
            return;
        };
        // Use each body's display name (falling back to its node id) so the
        // slicer's object list reads like the feature tree.
        let names: Vec<(String, usize)> = self
            .body_meshes
            .iter()
            .enumerate()
            .map(|(i, (id, _))| {
                let name = self
                    .document
                    .graph
                    .node_indices()
                    .find(|&n| self.document.graph[n].id == *id)
                    .map(|n| self.document.graph[n].name.clone())
                    .unwrap_or_else(|| id.clone());
                (name, i)
            })
            .collect();
        let bodies = self.body_meshes.clone();
        let body_count = bodies.len();
        let completions = self.export_completions.clone();
        let repaint = self.egui_ctx.clone();
        self.status_msg = "Exporting 3MF…".to_string();
        std::thread::spawn(move || {
            let bytes = zerocad_core::meshes_to_3mf(
                names.iter().map(|(name, i)| (name.as_str(), &bodies[*i].1)),
            );
            let (message, error) = match std::fs::write(&path, bytes) {
                Ok(()) => (
                    format!("Exported {body_count} bodies to {}", path.display()),
                    false,
                ),
                Err(error) => (format!("3MF export failed: {error}"), true),
            };
            completions
                .lock()
                .expect("export queue poisoned")
                .push(ExportCompletion { message, error });
            if let Some(ctx) = repaint {
                ctx.request_repaint();
            }
        });
    }

    /// Prompt for a path and write the current bodies as named objects in one
    /// Wavefront OBJ mesh. Like STL and 3MF, OBJ is a lossy export and does not
    /// replace the editable `.zcad` document.
    pub(crate) fn export_obj(&mut self) {
        if self.body_meshes.is_empty() {
            self.status_msg = "Nothing to export — the model has no solid bodies.".to_string();
            return;
        }
        if self.eval_pending {
            self.status_msg = "Export waits for the current model update to finish.".to_string();
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export OBJ")
            .add_filter("Wavefront OBJ", &["obj"])
            .save_file()
        else {
            return;
        };
        let names: Vec<(String, usize)> = self
            .body_meshes
            .iter()
            .enumerate()
            .map(|(i, (id, _))| {
                let name = self
                    .document
                    .graph
                    .node_indices()
                    .find(|&n| self.document.graph[n].id == *id)
                    .map(|n| self.document.graph[n].name.clone())
                    .unwrap_or_else(|| id.clone());
                (name, i)
            })
            .collect();
        let bodies = self.body_meshes.clone();
        let body_count = bodies.len();
        let completions = self.export_completions.clone();
        let repaint = self.egui_ctx.clone();
        self.status_msg = "Exporting OBJ…".to_string();
        std::thread::spawn(move || {
            let bytes = zerocad_core::meshes_to_obj(
                names.iter().map(|(name, i)| (name.as_str(), &bodies[*i].1)),
            );
            let (message, error) = match std::fs::write(&path, bytes) {
                Ok(()) => (
                    format!("Exported {body_count} bodies to {}", path.display()),
                    false,
                ),
                Err(error) => (format!("OBJ export failed: {error}"), true),
            };
            completions
                .lock()
                .expect("export queue poisoned")
                .push(ExportCompletion { message, error });
            if let Some(ctx) = repaint {
                ctx.request_repaint();
            }
        });
    }

    /// Prompt for a STEP file and add it to the design as an Import feature
    /// (undoable). The file's text is embedded in the feature so the `.zcad`
    /// stays self-contained; evaluation parses it into a body like any other
    /// feature node.
    pub(crate) fn import_step(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Import STEP")
            .add_filter("STEP model", &["step", "stp"])
            .pick_file()
        else {
            return;
        };
        let step_data = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                self.status_msg = format!("STEP import failed: {e}");
                return;
            }
        };
        let label = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("imported")
            .to_string();
        self.push_undo();
        let id = format!("import_{}", self.next_id());
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: label.clone(),
            feature: FeatureType::Import { step_data, label },
        });
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = format!("Imported {}", path.display());
    }

    /// Import an ASCII or binary STL as an explicitly mesh-only history
    /// feature. Validation diagnostics are produced by the evaluator and the
    /// original bytes remain content-addressed inside the `.zcad` document.
    pub(crate) fn import_stl(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Import STL")
            .add_filter("STL mesh", &["stl"])
            .pick_file()
        else {
            return;
        };
        let stl_data = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) => {
                self.status_msg = format!("STL import failed: {error}");
                return;
            }
        };
        let label = path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("imported mesh")
            .to_string();
        if let Err(error) = self.commit_stl_import(stl_data, label) {
            self.status_msg = format!("STL import failed: {error}");
            return;
        }
        self.status_msg = format!("Imported mesh {}", path.display());
    }

    fn commit_stl_import(&mut self, stl_data: Vec<u8>, label: String) -> Result<String, String> {
        let _validation =
            zerocad_core::read_stl_mesh(&stl_data).map_err(|error| error.to_string())?;
        self.push_undo();
        let id = format!("stl_import_{}", self.next_id());
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: label.clone(),
            feature: FeatureType::ImportStl { stl_data, label },
        });
        self.selected_node_id = Some(id.clone());
        self.reevaluate_geometry();
        Ok(id)
    }

    /// Delete the currently selected browser node (sketch/body/variable set), if
    /// any (undoable). Mirrors the per-row delete button in the document browser.
    pub(crate) fn delete_selected_node(&mut self) {
        let Some(del_id) = self.selected_node_id.clone() else {
            self.status_msg = "Nothing selected to delete.".to_string();
            return;
        };
        if self.delete_node_by_id(&del_id) {
            self.status_msg = "Deleted selection.".to_string();
        }
    }

    /// Delete a node by id (undoable): removes it from the graph (via
    /// `remove_feature`, which keeps the id→index map consistent), clears any
    /// selection state that referenced it, and reveals a sketch that was
    /// hidden only because the deleted feature consumed it — deleting a body
    /// should hand the user back its source sketch.
    pub(crate) fn delete_node_by_id(&mut self, del_id: &str) -> bool {
        let exists = self
            .document
            .graph
            .node_indices()
            .any(|idx| self.document.graph[idx].id == del_id);
        if !exists {
            return false;
        }
        self.push_undo();
        let reveal = self.document.sole_sketch_parents(del_id);
        self.document.remove_feature(del_id);
        for sketch_id in reveal {
            self.hidden_nodes.remove(&sketch_id);
        }
        if self.selected_node_id.as_deref() == Some(del_id) {
            self.selected_node_id = None;
        }
        if self.feature_properties_dialog.as_deref() == Some(del_id) {
            self.feature_properties_dialog = None;
        }
        self.selected_faces.retain(|(sid, _)| sid != del_id);
        self.selected_edges.retain(|(sid, _)| sid != del_id);
        self.selected_sketch_points
            .retain(|(sketch_id, _)| sketch_id != del_id);
        self.selected_body.retain(|(nid, _)| nid != del_id);
        self.hidden_nodes.remove(del_id);
        self.reevaluate_geometry();
        true
    }
}

#[cfg(test)]
mod save_dialog_tests {
    use super::*;

    #[test]
    fn project_title_validation_prevents_invalid_file_names() {
        assert_eq!(save_project_title_error(""), Some("Enter a project name."));
        assert!(save_project_title_error(" trailing ").is_some());
        assert!(save_project_title_error("bad/name").is_some());
        assert!(save_project_title_error("bad.").is_some());
        assert_eq!(save_project_title_error("Motor Bracket 01"), None);
    }

    #[test]
    fn every_standard_modal_dismissal_closes_the_save_dialog() {
        assert!(save_dialog_should_close(true, false, true));
        assert!(save_dialog_should_close(false, true, true));
        assert!(save_dialog_should_close(false, false, false));
        assert!(!save_dialog_should_close(false, false, true));
    }

    #[test]
    fn clicking_the_modal_backdrop_cancels_the_live_dialog() {
        let mut app = ZeroCadApp::new();
        app.open_save_dialog();
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(900.0, 700.0));

        let mut first_frame = egui::RawInput::default();
        first_frame.screen_rect = Some(screen);
        let _ = ctx.run(first_frame, |ctx| app.show_save_dialog(ctx));

        let mut press = egui::RawInput::default();
        press.screen_rect = Some(screen);
        press.events.extend([
            egui::Event::PointerMoved(egui::pos2(8.0, 8.0)),
            egui::Event::PointerButton {
                pos: egui::pos2(8.0, 8.0),
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        let _ = ctx.run(press, |ctx| app.show_save_dialog(ctx));

        let mut release = egui::RawInput::default();
        release.screen_rect = Some(screen);
        release.events.extend([
            egui::Event::PointerMoved(egui::pos2(8.0, 8.0)),
            egui::Event::PointerButton {
                pos: egui::pos2(8.0, 8.0),
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        let _ = ctx.run(release, |ctx| app.show_save_dialog(ctx));

        assert!(app.save_dialog.is_none());
    }

    #[test]
    fn pointer_movement_does_not_expand_the_save_dialog() {
        let mut app = ZeroCadApp::new();
        app.open_save_dialog();
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1100.0, 850.0));

        let render_at = |ctx: &egui::Context, app: &mut ZeroCadApp, pointer: egui::Pos2| {
            let mut input = egui::RawInput::default();
            input.screen_rect = Some(screen);
            input.events.push(egui::Event::PointerMoved(pointer));
            let _ = ctx.run(input, |ctx| app.show_save_dialog(ctx));
            ctx.memory(|memory| {
                memory
                    .area_rect(egui::Id::new(SAVE_DIALOG_WINDOW_ID))
                    .expect("save dialog should have a persisted area")
                    .width()
            })
        };

        // Allow the auto-sized window to settle before checking subsequent
        // pointer-triggered frames.
        let _ = render_at(&ctx, &mut app, egui::pos2(20.0, 20.0));
        let expected_width = render_at(&ctx, &mut app, egui::pos2(40.0, 40.0));

        for pointer in [
            egui::pos2(100.0, 120.0),
            egui::pos2(300.0, 220.0),
            egui::pos2(550.0, 425.0),
            egui::pos2(800.0, 650.0),
            egui::pos2(1050.0, 800.0),
        ] {
            let width = render_at(&ctx, &mut app, pointer);
            assert!(
                (width - expected_width).abs() < 0.01,
                "save dialog grew from {expected_width} to {width} after pointer movement"
            );
        }
    }
}

#[cfg(test)]
mod unsaved_changes_dialog_tests {
    use super::*;

    #[test]
    fn confirmation_dismissals_cancel_but_an_active_save_cannot_be_dismissed() {
        assert!(unsaved_changes_should_cancel(
            ProjectTransitionPhase::Confirm,
            true,
            false,
            true,
        ));
        assert!(unsaved_changes_should_cancel(
            ProjectTransitionPhase::Confirm,
            false,
            true,
            true,
        ));
        assert!(unsaved_changes_should_cancel(
            ProjectTransitionPhase::Confirm,
            false,
            false,
            false,
        ));
        assert!(!unsaved_changes_should_cancel(
            ProjectTransitionPhase::Saving,
            true,
            true,
            false,
        ));
    }

    #[test]
    fn clicking_the_backdrop_cancels_the_live_exit_confirmation() {
        let mut app = ZeroCadApp::new();
        app.document_revision = 1;
        app.saved_document_revision = 0;
        app.request_window_close();
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(900.0, 700.0));

        let mut first_frame = egui::RawInput::default();
        first_frame.screen_rect = Some(screen);
        let _ = ctx.run(first_frame, |ctx| app.show_unsaved_changes_dialog(ctx));

        let mut press = egui::RawInput::default();
        press.screen_rect = Some(screen);
        press.events.extend([
            egui::Event::PointerMoved(egui::pos2(8.0, 8.0)),
            egui::Event::PointerButton {
                pos: egui::pos2(8.0, 8.0),
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        let _ = ctx.run(press, |ctx| app.show_unsaved_changes_dialog(ctx));

        let mut release = egui::RawInput::default();
        release.screen_rect = Some(screen);
        release.events.extend([
            egui::Event::PointerMoved(egui::pos2(8.0, 8.0)),
            egui::Event::PointerButton {
                pos: egui::pos2(8.0, 8.0),
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        let _ = ctx.run(release, |ctx| app.show_unsaved_changes_dialog(ctx));

        assert!(app.pending_project_transition.is_none());
        assert!(!app.allow_window_close);
    }
}

#[cfg(test)]
mod workspace_routing_tests {
    use super::*;

    #[test]
    fn new_assembly_enters_an_isolated_empty_workspace() {
        let mut app = ZeroCadApp::new();
        app.document.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 1.0,
                h: 2.0,
                d: 3.0,
            },
        });
        app.current_document_path = Some(PathBuf::from("C:/parts/source.zcad"));
        app.hidden_nodes.insert("box_1".to_string());
        app.selected_node_id = Some("box_1".to_string());
        app.is_sketch_mode = true;

        app.new_assembly();

        assert_eq!(app.project_kind, ProjectKind::Assembly);
        assert_eq!(app.project_title(), "Untitled Assembly");
        assert_eq!(app.document.graph.node_count(), 1, "empty origin only");
        assert!(app.body_meshes.is_empty());
        assert!(app.evaluated_scene.instances().is_empty());
        assert!(app.current_document_path.is_none());
        assert!(app.hidden_nodes.is_empty());
        assert!(app.selected_node_id.is_none());
        assert!(!app.is_sketch_mode);
        assert!(!app.is_plane_selection_mode);
        assert!(app.undo_stack.is_empty());
        assert!(app.redo_stack.is_empty());
    }

    #[test]
    fn dirty_project_transition_waits_for_an_explicit_decision() {
        let mut app = ZeroCadApp::new();
        app.document_revision = 7;
        app.saved_document_revision = 6;

        app.new_assembly();

        assert_eq!(app.project_kind, ProjectKind::Part);
        assert!(matches!(
            app.pending_project_transition,
            Some(PendingProjectTransition {
                action: ProjectTransition::NewAssembly,
                phase: ProjectTransitionPhase::Confirm,
            })
        ));
    }

    #[test]
    fn discard_executes_exactly_the_guarded_transition() {
        let mut app = ZeroCadApp::new();
        app.document_revision = 1;
        app.saved_document_revision = 0;
        app.new_assembly();
        let pending = app.pending_project_transition.take().unwrap();

        app.perform_project_transition(pending.action);

        assert_eq!(app.project_kind, ProjectKind::Assembly);
        assert!(!app.has_unsaved_changes());
        assert!(app.pending_project_transition.is_none());
    }

    #[test]
    fn cancelling_guarded_transition_preserves_the_workspace() {
        let mut app = ZeroCadApp::new();
        app.document_revision = 3;
        app.saved_document_revision = 2;
        app.new_assembly();

        app.pending_project_transition = None;

        assert_eq!(app.project_kind, ProjectKind::Part);
        assert_eq!(app.document_revision, 3);
        assert!(app.has_unsaved_changes());
    }

    #[test]
    fn returning_to_part_does_not_restore_assembly_through_part_undo() {
        let mut app = ZeroCadApp::new();
        app.new_assembly();

        app.new_design();

        assert_eq!(app.project_kind, ProjectKind::Part);
        assert_eq!(app.project_title(), "Untitled Project");
        assert!(app.undo_stack.is_empty());
        assert!(app.redo_stack.is_empty());
    }

    #[test]
    fn project_switch_cancels_a_save_that_has_not_captured_its_document() {
        let mut app = ZeroCadApp::new();
        app.pending_save = Some(PendingSave {
            path: PathBuf::from("queued.zcad"),
            project_kind: ProjectKind::Part,
            profile: zerocad_core::SaveProfile::Hydrated {
                total_accelerator_budget: 1024,
            },
            started: std::time::Instant::now(),
            dispatched: false,
            revision: None,
            workspace_generation: app.workspace_generation,
        });

        app.new_assembly();

        assert!(app.pending_save.is_none());
    }

    #[test]
    fn assembly_uses_the_project_save_dialog() {
        let mut app = ZeroCadApp::new();
        app.new_assembly();

        app.open_save_dialog();

        assert!(app.save_dialog.is_some());
        assert_eq!(
            app.save_dialog.as_ref().unwrap().project_title,
            "Untitled Assembly"
        );
    }
}

#[cfg(test)]
mod phase5_stl_tests {
    use super::*;

    const TRIANGLE: &[u8] = br#"solid triangle
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
endsolid triangle
"#;

    #[test]
    fn validated_stl_import_is_undoable_and_redoable() {
        let mut app = ZeroCadApp::new();
        let id = app
            .commit_stl_import(TRIANGLE.to_vec(), "triangle.stl".into())
            .expect("GUI STL import");
        assert_eq!(app.selected_node_id.as_deref(), Some(id.as_str()));
        assert!(app.document.graph.node_weights().any(|node| {
            matches!(
                &node.feature,
                FeatureType::ImportStl { stl_data, .. } if stl_data == TRIANGLE
            )
        }));
        app.undo();
        assert!(!app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::ImportStl { .. })));
        app.redo();
        assert!(app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::ImportStl { .. })));
    }

    #[test]
    fn malformed_stl_does_not_mutate_the_document() {
        let mut app = ZeroCadApp::new();
        assert!(app
            .commit_stl_import(b"not an STL".to_vec(), "bad.stl".into())
            .is_err());
        assert_eq!(app.document.graph.node_count(), 1, "origin only");
        assert!(app.undo_stack.is_empty());
    }

    #[test]
    fn phase7_repeated_undo_redo_preserves_document_semantics() {
        const STACK_DEPTH: usize = 8;
        let stress_iterations = std::env::var("ZEROCAD_LONG_STRESS_ITERATIONS")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .unwrap_or(12)
            .clamp(1, 1000);
        let mut app = ZeroCadApp::new();
        for index in 0..STACK_DEPTH {
            app.push_undo();
            app.document.add_feature(FeatureNode {
                id: format!("phase7_undo_box_{index}"),
                name: format!("Undo Box {index}"),
                feature: FeatureType::Box {
                    w: 1.0 + index as f32,
                    h: 2.0,
                    d: 3.0,
                },
            });
        }
        let final_count = app.document.graph.node_count();
        app.document.validate_semantic_contracts().unwrap();
        for _ in 0..STACK_DEPTH {
            app.undo();
            app.document.validate_semantic_contracts().unwrap();
        }
        assert_eq!(app.document.graph.node_count(), 1, "origin only after undo");
        for _ in 0..STACK_DEPTH {
            app.redo();
            app.document.validate_semantic_contracts().unwrap();
        }
        assert_eq!(app.document.graph.node_count(), final_count);

        // Exercise repeated state transitions without growing the snapshot
        // stack with the configured iteration count. This keeps a long run's
        // memory bounded while still validating both directions each cycle.
        for iteration in 0..stress_iterations {
            app.undo();
            app.document.validate_semantic_contracts().unwrap();
            assert_eq!(
                app.document.graph.node_count(),
                final_count - 1,
                "undo cycle {iteration}"
            );
            app.redo();
            app.document.validate_semantic_contracts().unwrap();
            assert_eq!(
                app.document.graph.node_count(),
                final_count,
                "redo cycle {iteration}"
            );
        }
    }
}
