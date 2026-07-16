use crate::*;

/// One undo/redo entry: the parametric history plus the visibility set. The
/// visibility set has to travel with the graph — an extrude auto-hides its
/// sketch, so undoing the extrude must also reveal the sketch again.
impl ZeroCadApp {
    fn snapshot(&self) -> UndoSnapshot {
        let mut document = self.document.clone();
        document.state.units = self.current_unit;
        document.state.created_unix = self.doc_created_unix;
        document.state.visibility.clear();
        for hidden in &self.hidden_nodes {
            document.set_visible(hidden.clone(), false);
        }
        UndoSnapshot { document }
    }

    /// Restore a snapshot: swap in the graph (rebuilding its skipped id→index
    /// map so later features can still resolve their parents) and the
    /// visibility set, then clear all selection/op state that may reference
    /// nodes that no longer exist.
    fn restore_snapshot(&mut self, snap: UndoSnapshot) {
        self.current_unit = snap.document.state.units;
        self.doc_created_unix = snap.document.state.created_unix;
        self.hidden_nodes = snap.document.hidden_entities();
        self.document = snap.document;
        self.document.rebuild_node_map();
        self.selected_node_id = None;
        self.selected_faces.clear();
        self.selected_body.clear();
        self.extrude_op = None;
        self.edge_mod_op = None;
        self.move_op = None;
        self.combine_op = None;
        self.split_body_op = None;
        self.scale_body_op = None;
        self.move_preview_bodies = None;
        self.pending_visual = None;
        self.reevaluate_geometry();
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
        log::info!("Creating new empty model.");
        self.push_undo();
        self.document = Document::new();
        self.doc_created_unix = None;
        self.set_body_meshes(Vec::new());
        self.selected_node_id = None;
        self.pending_visual = None;
        self.reset_sketch_state();
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_body.clear();
        self.body_clipboard = None;
        self.move_op = None;
        self.combine_op = None;
        self.move_preview_bodies = None;
        self.status_msg = "New blank design created.".to_string();
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
            .recent_files
            .entries
            .first()
            .and_then(|e| e.path.file_stem().map(|s| s.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "Untitled".to_string());

        self.save_dialog = Some(SaveDialogState {
            project_title: default_title,
            save_format: SaveFormat::ZcadLightweight,
            save_dir: default_dir,
        });
    }

    /// Execute the save using the current save-dialog parameters.
    pub(crate) fn do_save(&mut self) {
        let state = match self.save_dialog.take() {
            Some(s) => s,
            None => return,
        };

        let ext = state.save_format.extension();
        let file_name = format!("{}.{ext}", state.project_title);
        let path = state.save_dir.join(&file_name);
        self.pending_save = Some(PendingSave {
            path,
            profile: state.save_format.profile(self.hydrated_cache_mb),
            started: std::time::Instant::now(),
            dispatched: false,
        });
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
            let created_unix = *self.doc_created_unix.get_or_insert_with(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs()
            });
            let save = self.pending_save.as_mut().expect("pending save vanished");
            let mut document = self.document.clone();
            document.state.units = self.current_unit;
            document.state.created_unix = Some(created_unix);
            document.state.visibility.clear();
            for hidden in &self.hidden_nodes {
                document.set_visible(hidden.clone(), false);
            }
            self.document_worker.submit(document_worker::SaveRequest {
                path: save.path.clone(),
                document,
                bodies: self.body_meshes.clone(),
                profile: save.profile,
                cache: self.document.evaluation_cache_snapshot(),
            });
            save.dispatched = true;
            self.status_msg = "Saving design…".to_string();
        }
        if let Some(done) = self.document_worker.try_recv() {
            self.pending_save = None;
            match done.result {
                Ok(()) => {
                    let how = if matches!(done.profile, zerocad_core::SaveProfile::Compact) {
                        " (compact)"
                    } else {
                        " (hydrated)"
                    };
                    self.status_msg = format!("Design saved to {}{how}", done.path.display());
                    self.recent_files.record(&done.path);
                    self.defer_onboarding_texture_eviction(&done.path);
                }
                Err(error) => self.status_msg = format!("Save failed: {error}"),
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

    /// Render the in-app save dialog as a centered modal overlay.
    pub(crate) fn show_save_dialog(&mut self, ctx: &egui::Context) {
        let is_open = self.save_dialog.is_some();
        if !is_open {
            return;
        }

        // Semi-transparent backdrop.
        egui::Area::new(egui::Id::new("save_dialog_backdrop"))
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                ui.painter()
                    .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(120));
                // Consume clicks on the backdrop so they don't fall through.
                ui.allocate_rect(screen, egui::Sense::click());
            });

        let mut close = false;
        let mut do_save = false;

        egui::Window::new("Save Design")
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .collapsible(false)
            .resizable(false)
            .min_width(420.0)
            .show(ctx, |ui| {
                let state = self.save_dialog.as_mut().unwrap();

                ui.add_space(4.0);

                // --- Project Title ---
                ui.horizontal(|ui| {
                    ui.label("Project Title:");
                    ui.text_edit_singleline(&mut state.project_title);
                });

                ui.add_space(6.0);

                // --- File Format ---
                ui.horizontal(|ui| {
                    ui.label("File Format:");
                    egui::ComboBox::from_id_salt("save_format")
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
                });

                ui.add_space(6.0);

                // --- Save Location ---
                ui.horizontal(|ui| {
                    ui.label("Save to:");
                    let display = state.save_dir.display().to_string();
                    ui.add(
                        egui::TextEdit::singleline(&mut display.clone())
                            .desired_width(260.0)
                            .interactive(false),
                    );
                    if ui.button("Browse…").clicked() {
                        if let Some(dir) = rfd::FileDialog::new()
                            .set_title("Choose save folder")
                            .set_directory(&state.save_dir)
                            .pick_folder()
                        {
                            state.save_dir = dir;
                        }
                    }
                });

                ui.add_space(6.0);

                // --- Recent Folders ---
                let folders = self.recent_files.recent_folders();
                if !folders.is_empty() {
                    ui.label("Recent Folders:");
                    let state = self.save_dialog.as_mut().unwrap();
                    egui::ScrollArea::vertical()
                        .max_height(100.0)
                        .show(ui, |ui| {
                            for folder in &folders {
                                let label = folder.display().to_string();
                                let selected = *folder == state.save_dir;
                                if ui.selectable_label(selected, &label).clicked() {
                                    state.save_dir = folder.clone();
                                }
                            }
                        });
                    ui.add_space(6.0);
                }

                // --- Full path preview ---
                let state = self.save_dialog.as_ref().unwrap();
                let full_path = state.save_dir.join(format!(
                    "{}.{}",
                    state.project_title,
                    state.save_format.extension()
                ));
                ui.horizontal(|ui| {
                    ui.label("File:");
                    ui.monospace(full_path.display().to_string());
                });

                ui.add_space(8.0);

                // --- Buttons ---
                ui.horizontal(|ui| {
                    if ui.button("Save").clicked() {
                        do_save = true;
                    }
                    if ui.button("Cancel").clicked() {
                        close = true;
                    }
                });
            });

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
        self.recent_files.record(path);
        if !self.body_meshes.is_empty() {
            let (w, h, rgba) = thumbnail::render_thumbnail(&self.body_meshes, 256);
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

    /// The centered Welcome modal: New / Open / Recent. Drawn over a dimmed,
    /// click-swallowing backdrop so the workspace beneath is inert. A no-op
    /// unless `onboarding_visible`. Esc, the Close button, or choosing any action
    /// dismisses it (without touching the persisted "show on startup" preference,
    /// which the footer checkbox edits separately).
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
        let loaded =
            match zerocad_core::read_document_file(&path, &zerocad_core::LoadOptions::default()) {
                Ok(l) => l,
                Err(e) => {
                    self.status_msg = format!("Load failed: {e}");
                    return;
                }
            };

        self.push_undo();
        self.hidden_nodes = loaded.document.hidden_entities();
        self.current_unit = loaded.document.state.units;
        self.doc_created_unix = loaded.document.state.created_unix;
        self.document = loaded.document;
        // Continue the user-facing feature id sequence after the largest loaded
        // suffix. Dependencies and semantic timelines determine evaluation.
        self.reseed_id_counter_from_graph();
        self.selected_node_id = None;
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_body.clear();
        self.extrude_op = None;
        self.edge_mod_op = None;
        self.body_clipboard = None;
        self.move_op = None;
        self.combine_op = None;
        self.move_preview_bodies = None;

        // Show the embedded geometry cache immediately (instant open). It's only
        // present when fresh (its hash matched the loaded graph), so it's safe to
        // display; `reevaluate_geometry` then swaps in freshly-computed bodies.
        let had_mesh_cache = loaded.accelerators.display_meshes.is_some();
        if let Some(cache) = loaded.accelerators.display_meshes {
            self.set_body_meshes(cache);
        }
        let had_evaluation_cache = loaded.accelerators.evaluation_cache.is_some();
        if let Some(cache) = loaded.accelerators.evaluation_cache {
            self.document.install_evaluation_cache(cache);
        }
        // Seed the onboarding thumbnail cache from the file's embedded preview so
        // a `.zcad` from another machine shows its real thumbnail even if it has
        // no geometry to re-render (e.g. evaluation fails).
        if let Some(png) = &loaded.accelerators.small_preview_png {
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
        self.status_msg = if loaded.diagnostics.is_empty() {
            format!("Design loaded from {}", path.display())
        } else {
            format!(
                "Design loaded from {} with {} recoverable warning(s)",
                path.display(),
                loaded.diagnostics.len()
            )
        };
        self.remember_project(&path);
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
        self.selected_faces.retain(|(sid, _)| sid != del_id);
        self.selected_edges.retain(|(sid, _)| sid != del_id);
        self.selected_body.retain(|(nid, _)| nid != del_id);
        self.hidden_nodes.remove(del_id);
        self.reevaluate_geometry();
        true
    }
}
