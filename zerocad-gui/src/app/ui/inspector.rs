use crate::*;

pub(super) const WORKSPACE_INSPECTOR_CONTENT_WIDTH: f32 = 246.0;
pub(super) const WORKSPACE_INSPECTOR_VIEWPORT_RESERVATION: f32 =
    WORKSPACE_INSPECTOR_CONTENT_WIDTH + 42.0;
const WORKSPACE_INSPECTOR_EDGE_PADDING: f32 = 16.0;

impl ZeroCadApp {
    pub(crate) fn inspector_has_content(&self) -> bool {
        if self.is_sketch_mode {
            return false;
        }
        self.extrude_op.is_some()
            || self.is_plane_selection_mode
            || self.other_active_operation_name().is_some()
            || self.has_viewport_selection()
    }

    fn has_viewport_selection(&self) -> bool {
        !self.selected_body.is_empty()
            || !self.selected_faces.is_empty()
            || !self.selected_edges.is_empty()
    }

    pub(crate) fn draw_inspector(&mut self, ctx: &egui::Context, viewport: egui::Rect) {
        let content_width = WORKSPACE_INSPECTOR_CONTENT_WIDTH;
        let overlay_height = (viewport.height() - 32.0).clamp(280.0, 680.0);
        egui::Area::new(egui::Id::new("workspace_inspector_overlay"))
            .order(egui::Order::Foreground)
            .pivot(egui::Align2::RIGHT_TOP)
            .fixed_pos(egui::pos2(
                viewport.right() - WORKSPACE_INSPECTOR_EDGE_PADDING,
                viewport.top() + WORKSPACE_INSPECTOR_EDGE_PADDING,
            ))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .rounding(8.0)
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 3.0),
                        blur: 12.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(24),
                    })
                    .inner_margin(egui::Margin::symmetric(14.0, 10.0))
                    .show(ui, |ui| {
                        ui.set_width(content_width);
                        ui.set_max_height(overlay_height);
                        ui.add_space(9.0);
                        let title = self.inspector_title();
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(title)
                                    .strong()
                                    .size(12.0)
                                    .color(self.pal().text_strong),
                            );
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    icons::Icon::Inspect.draw(
                                        ui.painter(),
                                        egui::Rect::from_min_size(
                                            ui.cursor().min,
                                            egui::vec2(17.0, 17.0),
                                        ),
                                        self.pal().text_muted,
                                    );
                                    ui.add_space(17.0);
                                },
                            );
                        });
                        ui.add_space(5.0);
                        ui.separator();

                        if self.extrude_op.is_some() {
                            self.draw_extrude_inspector(ui);
                        } else if self.is_plane_selection_mode {
                            self.draw_plane_selection_inspector(ui);
                        } else if let Some(operation) = self.other_active_operation_name() {
                            ui.add_space(12.0);
                            Self::inspector_section(ui, &self.pal(), "Active operation", |ui| {
                                ui.label(
                                    egui::RichText::new(operation)
                                        .strong()
                                        .size(14.0)
                                        .color(self.pal().accent),
                                );
                                ui.add_space(4.0);
                                ui.label(
                            egui::RichText::new(
                                "Adjust the live operation controls and preview in the viewport.",
                            )
                            .size(11.5)
                            .color(self.pal().text_muted),
                        );
                            });
                        } else if self.has_viewport_selection() {
                            ui.add_space(12.0);
                            Self::inspector_section(ui, &self.pal(), "Selection", |ui| {
                                if !self.selected_body.is_empty() {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{} solid element{} selected",
                                            self.selected_body.len(),
                                            if self.selected_body.len() == 1 {
                                                ""
                                            } else {
                                                "s"
                                            }
                                        ))
                                        .strong()
                                        .size(13.0)
                                        .color(self.pal().text_strong),
                                    );
                                }
                                if !self.selected_faces.is_empty() {
                                    ui.label(format!(
                                        "{} sketch profile{} selected",
                                        self.selected_faces.len(),
                                        if self.selected_faces.len() == 1 {
                                            ""
                                        } else {
                                            "s"
                                        }
                                    ));
                                }
                                if !self.selected_edges.is_empty() {
                                    ui.label(format!(
                                        "{} sketch edge{} selected",
                                        self.selected_edges.len(),
                                        if self.selected_edges.len() == 1 {
                                            ""
                                        } else {
                                            "s"
                                        }
                                    ));
                                }
                            });
                        }
                    });
            });
    }

    fn inspector_title(&self) -> &'static str {
        if self.extrude_op.is_some() {
            "EXTRUDE"
        } else if self.is_plane_selection_mode {
            "SKETCH PLANE"
        } else if let Some(name) = self.other_active_operation_name() {
            name
        } else {
            "INSPECTOR"
        }
    }

    fn other_active_operation_name(&self) -> Option<&'static str> {
        if self.revolve_op.is_some() {
            Some("REVOLVE")
        } else if self.sweep_op.is_some() {
            Some("SWEEP")
        } else if self.edge_mod_op.is_some() {
            Some("EDGE MODIFICATION")
        } else if self.hole_op.is_some() {
            Some("HOLE")
        } else if self.thread_op.is_some() {
            Some("THREAD")
        } else if self.shell_op.is_some() {
            Some("SHELL")
        } else if self.draft_op.is_some() {
            Some("DRAFT")
        } else if self.pattern_op.is_some() {
            Some("PATTERN")
        } else if self.move_op.is_some() {
            Some("MOVE")
        } else if self.combine_op.is_some() {
            Some("COMBINE")
        } else if self.split_body_op.is_some() {
            Some("SPLIT BODY")
        } else if self.scale_body_op.is_some() {
            Some("SCALE BODY")
        } else {
            None
        }
    }

    fn draw_extrude_inspector(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        let mut commit = false;
        let mut cancel = false;
        egui::ScrollArea::vertical()
            .id_salt("extrude_inspector_scroll")
            .show(ui, |ui| {
                ui.add_space(12.0);
                if let Some(op) = self.extrude_op.as_mut() {
                    let faces: usize = op.targets.iter().map(|target| target.indices.len()).sum();
                    Self::inspector_section(ui, &pal, "Profiles", |ui| {
                        egui::Frame::none()
                            .fill(pal.accent_soft)
                            .stroke(egui::Stroke::new(1.0, pal.accent))
                            .rounding(6.0)
                            .inner_margin(egui::Margin::symmetric(10.0, 8.0))
                            .show(ui, |ui| {
                                ui.label(
                                    egui::RichText::new(format!("{faces} selected"))
                                        .color(pal.accent)
                                        .size(12.0),
                                );
                            });
                    });

                    Self::inspector_section(ui, &pal, "Distance", |ui| {
                        let mut changed = ui
                            .add(
                                egui::DragValue::new(&mut op.depth)
                                    .speed(0.5)
                                    .suffix(self.current_unit.suffix()),
                            )
                            .changed();
                        if ui.button("Flip direction").clicked() {
                            op.depth = -op.depth;
                            changed = true;
                        }
                        if changed {
                            op.depth_text = format!("{:.2}", op.depth);
                        }
                        ui.label(
                            egui::RichText::new("Drag the viewport handle to push or pull.")
                                .size(10.5)
                                .color(pal.text_faint),
                        );
                    });

                    Self::inspector_section(ui, &pal, "Operation", |ui| {
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 3.0;
                            for (mode, label) in [
                                (ExtrudeMode::NewBody, "New Body"),
                                (ExtrudeMode::Join, "Join"),
                                (ExtrudeMode::Cut, "Cut"),
                            ] {
                                let selected = op.mode == mode;
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new(label).size(11.0).color(
                                                if selected {
                                                    egui::Color32::WHITE
                                                } else {
                                                    pal.text_body
                                                },
                                            ),
                                        )
                                        .fill(if selected {
                                            pal.accent
                                        } else {
                                            pal.surface_subtle
                                        })
                                        .stroke(egui::Stroke::new(
                                            1.0,
                                            if selected { pal.accent } else { pal.border },
                                        ))
                                        .rounding(5.0),
                                    )
                                    .clicked()
                                {
                                    op.mode = mode;
                                }
                            }
                        });
                    });
                }

                ui.add_space(14.0);
                ui.separator();
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("Create")
                                    .strong()
                                    .color(egui::Color32::WHITE),
                            )
                            .fill(pal.accent)
                            .stroke(egui::Stroke::NONE)
                            .rounding(6.0)
                            .min_size(egui::vec2(104.0, 34.0)),
                        )
                        .clicked()
                    {
                        commit = true;
                    }
                    if ui
                        .add(egui::Button::new("Cancel").min_size(egui::vec2(90.0, 34.0)))
                        .clicked()
                    {
                        cancel = true;
                    }
                });
                ui.label(
                    egui::RichText::new("Enter = Create  •  Esc = Cancel")
                        .size(10.0)
                        .color(pal.text_faint),
                );
            });
        if commit {
            self.commit_extrude_op();
        } else if cancel {
            self.cancel_extrude_op();
        }
    }

    fn draw_plane_selection_inspector(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        ui.add_space(12.0);
        Self::inspector_section(ui, &pal, "Choose a plane", |ui| {
            ui.label(
                egui::RichText::new(
                    "Select an origin plane, datum plane, or planar body face in the viewport.",
                )
                .size(12.0)
                .color(pal.text_muted),
            );
        });
    }

    fn inspector_section(
        ui: &mut egui::Ui,
        pal: &Palette,
        title: &str,
        content: impl FnOnce(&mut egui::Ui),
    ) {
        ui.add_space(4.0);
        ui.label(
            egui::RichText::new(title)
                .strong()
                .size(12.0)
                .color(pal.text_strong),
        );
        ui.add_space(5.0);
        content(ui);
        ui.add_space(10.0);
        ui.separator();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sketch_mode_suppresses_the_workspace_inspector() {
        let mut app = ZeroCadApp::new();
        app.is_sketch_mode = true;
        app.selected_node_id = Some("selected_feature".to_string());

        assert!(!app.inspector_has_content());
    }

    #[test]
    fn selected_feature_does_not_open_the_workspace_inspector() {
        let mut app = ZeroCadApp::new();
        app.selected_node_id = Some("selected_feature".to_string());

        assert!(!app.inspector_has_content());
    }
}
