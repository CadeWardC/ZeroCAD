use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_status_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("status_bar")
            .exact_height(34.0)
            .frame(
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .inner_margin(egui::Margin::symmetric(22.0, 5.0)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing = egui::vec2(8.0, 0.0);
                    ui.label(
                        egui::RichText::new("Ready")
                            .size(11.0)
                            .color(self.pal().text_body),
                    );
                    ui.separator();

                    let delayed_busy = self.eval_started.is_some_and(|started| {
                        started.elapsed() >= std::time::Duration::from_millis(150)
                    }) || self.pending_save.as_ref().is_some_and(|save| {
                        save.started.elapsed() >= std::time::Duration::from_millis(150)
                    });
                    if delayed_busy {
                        ui.spinner();
                    } else {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(15.0, 15.0), egui::Sense::hover());
                        icons::Icon::Help.draw(ui.painter(), rect, self.pal().text_muted);
                    }

                    let message = self.error_msg.as_deref().unwrap_or(&self.status_msg);
                    ui.label(egui::RichText::new(message).size(11.0).color(
                        if self.error_msg.is_some() {
                            self.pal().danger
                        } else {
                            self.pal().text_body
                        },
                    ));

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.checkbox(&mut self.snap_enabled, "Snap")
                            .on_hover_text("Enable sketch snapping");
                        ui.separator();
                        if ui
                            .selectable_label(self.grid_visible, "Grid: 5 mm")
                            .on_hover_text("Show or hide the reference grid")
                            .clicked()
                        {
                            self.grid_visible = !self.grid_visible;
                        }
                        ui.separator();
                        ui.scope(|ui| {
                            // Preserve comfortable text/arrow padding in the
                            // compact status bar and carry it into the popup.
                            ui.spacing_mut().button_padding = egui::vec2(10.0, 3.0);
                            egui::ComboBox::from_id_salt("status_unit_select")
                                .width(138.0)
                                .selected_text(match self.current_unit {
                                    Unit::Millimeter => "Millimeters (mm)",
                                    Unit::Inch => "Inches (in)",
                                    Unit::Meter => "Meters (m)",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut self.current_unit,
                                        Unit::Millimeter,
                                        "Millimeters (mm)",
                                    );
                                    ui.selectable_value(
                                        &mut self.current_unit,
                                        Unit::Inch,
                                        "Inches (in)",
                                    );
                                    ui.selectable_value(
                                        &mut self.current_unit,
                                        Unit::Meter,
                                        "Meters (m)",
                                    );
                                });
                        });
                        if self.is_sketch_mode {
                            ui.separator();
                            let status = self
                                .sketch_solver_model
                                .as_ref()
                                .map(|model| {
                                    let vars = self.document.variable_map();
                                    let report = zerocad_core::sketch::solve_model(model, &vars);
                                    if report.dof == 0 {
                                        "Fully defined".to_string()
                                    } else {
                                        format!("{} DOF", report.dof)
                                    }
                                })
                                .unwrap_or_else(|| "Under defined".to_string());
                            ui.label(
                                egui::RichText::new(status)
                                    .size(10.5)
                                    .color(self.pal().text_muted),
                            );
                        }
                    });
                });
            });
    }
}
