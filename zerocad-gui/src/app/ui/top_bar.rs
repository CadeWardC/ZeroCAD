use crate::*;

impl ZeroCadApp {
    pub(crate) fn project_title(&self) -> String {
        self.current_document_path
            .as_ref()
            .and_then(|path| path.file_stem())
            .map(|name| name.to_string_lossy().into_owned())
            .filter(|name| !name.is_empty())
            .unwrap_or_else(|| match self.project_kind {
                ProjectKind::Part => "Untitled Project".to_string(),
                ProjectKind::Assembly => "Untitled Assembly".to_string(),
            })
    }

    pub(crate) fn draw_top_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("global_header")
            .exact_height(54.0)
            .frame(
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .inner_margin(egui::Margin::symmetric(24.0, 8.0)),
            )
            .show(ctx, |ui| {
                ui.columns(2, |columns| {
                    let (left, right) = columns.split_at_mut(1);
                    left[0].horizontal_centered(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(10.0, 0.0);
                        self.draw_wordmark(ui);
                        self.draw_top_bar_file_menu(ui, ctx);

                        if !self.onboarding_visible {
                            ui.separator();

                            if icons::Icon::Save
                                .icon_button(
                                    ui,
                                    egui::Color32::TRANSPARENT,
                                    self.pal().accent_soft,
                                    self.pal().text_body,
                                )
                                .on_hover_text("Save project (Ctrl+S)")
                                .clicked()
                            {
                                self.open_save_dialog();
                            }

                            ui.label(
                                egui::RichText::new(self.project_title())
                                    .strong()
                                    .size(13.0)
                                    .color(self.pal().text_strong),
                            );

                            if self.project_kind == ProjectKind::Part {
                                let undo = icons::Icon::Undo
                                    .icon_button(
                                        ui,
                                        egui::Color32::TRANSPARENT,
                                        self.pal().accent_soft,
                                        if self.undo_stack.is_empty() {
                                            self.pal().text_faint
                                        } else {
                                            self.pal().text_body
                                        },
                                    )
                                    .on_hover_text("Undo (Ctrl+Z)");
                                if undo.clicked() && !self.undo_stack.is_empty() {
                                    self.undo();
                                }
                                let redo = icons::Icon::Redo
                                    .icon_button(
                                        ui,
                                        egui::Color32::TRANSPARENT,
                                        self.pal().accent_soft,
                                        if self.redo_stack.is_empty() {
                                            self.pal().text_faint
                                        } else {
                                            self.pal().text_body
                                        },
                                    )
                                    .on_hover_text("Redo (Ctrl+Y)");
                                if redo.clicked() && !self.redo_stack.is_empty() {
                                    self.redo();
                                }
                            }
                        }
                    });

                    right[0].with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if icons::Icon::Settings
                            .icon_button(
                                ui,
                                egui::Color32::TRANSPARENT,
                                self.pal().accent_soft,
                                self.pal().text_body,
                            )
                            .on_hover_text("Settings")
                            .clicked()
                        {
                            self.show_preferences = true;
                        }
                        if icons::Icon::Help
                            .icon_button(
                                ui,
                                egui::Color32::TRANSPARENT,
                                self.pal().accent_soft,
                                self.pal().text_body,
                            )
                            .on_hover_text("About ZeroCAD")
                            .clicked()
                        {
                            self.show_about = true;
                        }

                        if !self.onboarding_visible {
                            ui.separator();
                            if self.project_kind == ProjectKind::Assembly {
                                self.mode_tab(ui, "Assembly", icons::Icon::Assembly, true);
                            } else {
                                if self
                                    .mode_tab(ui, "Inspect", icons::Icon::Inspect, false)
                                    .clicked()
                                {
                                    self.open_inspection_dialog();
                                }
                                let sketch_active =
                                    self.is_sketch_mode || self.is_plane_selection_mode;
                                let sketch_tab =
                                    self.mode_tab(ui, "Sketch", icons::Icon::Sketch, sketch_active);
                                if sketch_tab.clicked() && !sketch_active {
                                    self.begin_sketch_from_header(ui.input(|input| input.time));
                                }
                                let model_tab =
                                    self.mode_tab(ui, "Model", icons::Icon::Cube, !sketch_active);
                                if model_tab
                                    .on_hover_text(if sketch_active {
                                        "Finish the active sketch and return to Model"
                                    } else {
                                        "Model workspace"
                                    })
                                    .clicked()
                                    && sketch_active
                                {
                                    self.finish_active_sketch(ctx);
                                }
                            }
                        }
                    });
                });
            });

        if !self.onboarding_visible {
            egui::TopBottomPanel::top("context_toolbar")
                .exact_height(54.0)
                .frame(
                    egui::Frame::none()
                        .fill(self.pal().surface)
                        .stroke(egui::Stroke::new(1.0, self.pal().border))
                        .inner_margin(egui::Margin::symmetric(24.0, 8.0)),
                )
                .show(ctx, |ui| {
                    ui.horizontal_centered(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(8.0, 0.0);
                        if self.project_kind == ProjectKind::Assembly {
                            ui.label(
                                egui::RichText::new("ASSEMBLY WORKSPACE")
                                    .strong()
                                    .size(11.0)
                                    .color(self.pal().accent),
                            );
                            ui.separator();
                            ui.label(
                                egui::RichText::new(format!(
                                    "{} components",
                                    self.assembly_document.occurrences.len()
                                ))
                                .size(11.5)
                                .color(self.pal().text_muted),
                            );
                            if ui.button("Insert Part…").clicked() {
                                self.insert_part_into_assembly();
                            }
                        } else {
                            let active_sketching =
                                self.is_sketch_mode || self.is_plane_selection_mode;
                            self.draw_top_bar_modeling_commands(ui, ctx, active_sketching);
                            self.draw_sketch_tool_strip(ui);
                        }
                    });
                });
        }
    }

    fn draw_wordmark(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(1.0, 0.0);
            ui.label(
                egui::RichText::new("Zero")
                    .strong()
                    .size(19.0)
                    .color(self.pal().text_strong),
            );
            ui.label(
                egui::RichText::new("CAD")
                    .strong()
                    .size(19.0)
                    .color(egui::Color32::from_rgb(37, 99, 235)),
            );
        });
    }

    fn mode_tab(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        icon: icons::Icon,
        active: bool,
    ) -> egui::Response {
        icon.labeled_button(
            ui,
            label,
            if active {
                self.pal().accent_soft
            } else {
                egui::Color32::TRANSPARENT
            },
            self.pal().accent_soft,
            if active {
                self.pal().accent
            } else {
                self.pal().text_body
            },
            if active {
                egui::Stroke::new(1.0, self.pal().accent)
            } else {
                egui::Stroke::NONE
            },
        )
    }

    fn begin_sketch_from_header(&mut self, now: f64) {
        let face_sel = if self.selected_body.len() == 1 {
            self.selected_body
                .iter()
                .next()
                .and_then(|(node, pick)| match pick {
                    BodyPick::Face(face) => Some((node.clone(), *face)),
                    _ => None,
                })
        } else {
            None
        };
        let cs_and_ref = face_sel.and_then(|(node, face)| {
            self.face_cs(&node, face)
                .map(|cs| (cs, self.face_ref(&node, face), node, face))
        });
        if let Some((cs, face_ref, node, face)) = cs_and_ref {
            let boundary = self.face_boundary_curves(&node, face, &cs);
            self.active_sketch_on_face = true;
            self.active_sketch_face_ref = face_ref;
            self.begin_sketch_on(cs, now);
            self.active_face_boundary = boundary;
            self.recompute_sketch_regions();
            self.status_msg =
                "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
        } else {
            self.active_sketch_on_face = false;
            self.active_sketch_face_ref = None;
            self.is_plane_selection_mode = true;
            self.is_sketch_mode = false;
            self.reset_sketch_state();
            self.status_msg =
                "Select an origin plane, datum plane, or planar face to begin sketching."
                    .to_string();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn project_title_uses_file_stem_or_untitled() {
        let mut app = ZeroCadApp::new();
        assert_eq!(app.project_title(), "Untitled Project");
        app.current_document_path = Some(PathBuf::from("C:/parts/motor_mount.zcad"));
        assert_eq!(app.project_title(), "motor_mount");
        app.new_assembly();
        assert_eq!(app.project_title(), "Untitled Assembly");
    }
}
