use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_extrude_panel(&mut self, ctx: &egui::Context) {
        // RIGHT PANEL: Extrude tool window, shown alongside the inline distance
        // box (the floating "n.nn mm" field, drawn after the viewport in
        // `show_extrude_dialog`). Both edit the same `op.depth`.
        if self.extrude_op.is_some() {
            egui::SidePanel::right("extrude_tool_window")
                .resizable(false)
                .default_width(220.0)
                .show(ctx, |ui| {
                    ui.add_space(8.0);
                    ui.heading("⬆️ Extrude");
                    ui.separator();

                    let unit_suffix = self.current_unit.suffix();
                    let mut commit = false;
                    let mut cancel = false;
                    if let Some(op) = self.extrude_op.as_mut() {
                        let faces: usize = op.targets.iter().map(|t| t.indices.len()).sum();
                        ui.label(format!("Faces: {}", faces));
                        ui.add_space(6.0);

                        ui.label("Distance");
                        let mut changed = ui
                            .add(
                                egui::DragValue::new(&mut op.depth)
                                    .speed(0.5)
                                    .suffix(unit_suffix),
                            )
                            .changed();
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut op.depth, -150.0..=150.0)
                                    .suffix(unit_suffix),
                            )
                            .changed();

                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            if ui
                                .button("Flip")
                                .on_hover_text("Reverse direction")
                                .clicked()
                            {
                                op.depth = -op.depth;
                                changed = true;
                            }
                            ui.weak("· drag in view to push/pull");
                        });

                        // Keep the inline box text in sync with slider/drag edits.
                        if changed {
                            op.depth_text = format!("{:.2}", op.depth);
                        }

                        ui.add_space(10.0);
                        ui.label("Operation");
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                            for (mode, label) in [
                                (ExtrudeMode::NewBody, "New Body"),
                                (ExtrudeMode::Join, "Join"),
                                (ExtrudeMode::Cut, "Cut"),
                            ] {
                                let selected = op.mode == mode;
                                let (fill, text) = if selected {
                                    (egui::Color32::from_rgb(0, 120, 215), egui::Color32::WHITE)
                                } else {
                                    (
                                        egui::Color32::from_rgb(238, 241, 245),
                                        egui::Color32::from_rgb(70, 75, 82),
                                    )
                                };
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new(label).color(text).size(11.5),
                                        )
                                        .fill(fill)
                                        .rounding(3.0)
                                        .min_size(egui::vec2(58.0, 22.0)),
                                    )
                                    .clicked()
                                {
                                    op.mode = mode;
                                }
                            }
                        });
                    }

                    ui.add_space(12.0);
                    ui.separator();
                    ui.horizontal(|ui| {
                        let ok_btn = icons::Icon::Check.labeled_button(
                            ui,
                            "OK",
                            egui::Color32::from_rgb(16, 185, 129),
                            egui::Color32::from_rgb(5, 150, 105),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        );
                        if ok_btn.clicked() {
                            commit = true;
                        }
                        if ui
                            .add(egui::Button::new("Cancel").min_size(egui::vec2(70.0, 28.0)))
                            .clicked()
                        {
                            cancel = true;
                        }
                    });
                    ui.add_space(4.0);
                    ui.weak("Enter = OK · Esc = Cancel");

                    if commit {
                        self.commit_extrude_op();
                    } else if cancel {
                        self.cancel_extrude_op();
                    }
                });
        }
    }
}
