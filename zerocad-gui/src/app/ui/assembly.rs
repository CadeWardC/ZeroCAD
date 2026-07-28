use crate::*;

impl ZeroCadApp {
    /// Draw the assembly route without constructing any part-only viewport UI.
    ///
    /// The empty state is deliberately its own workspace boundary. Milestones
    /// that add occurrences can replace the center content with the shared
    /// evaluated-scene viewport without first untangling modeling commands,
    /// feature history, or part selection from this route.
    pub(crate) fn draw_assembly_workspace(&mut self, ctx: &egui::Context) {
        let pal = self.pal();
        egui::CentralPanel::default()
            .frame(
                egui::Frame::none()
                    .fill(pal.surface_subtle)
                    .inner_margin(egui::Margin::same(12.0)),
            )
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    egui::Frame::none()
                        .fill(pal.surface)
                        .stroke(egui::Stroke::new(1.0, pal.border))
                        .rounding(8.0)
                        .inner_margin(egui::Margin::symmetric(14.0, 12.0))
                        .show(ui, |ui| {
                            ui.set_width(250.0);
                            ui.set_min_height((ui.available_height() - 2.0).max(280.0));
                            ui.label(
                                egui::RichText::new("ASSEMBLY")
                                    .strong()
                                    .size(12.0)
                                    .color(pal.text_strong),
                            );
                            ui.add_space(8.0);
                            ui.separator();
                            ui.add_space(10.0);
                            ui.horizontal(|ui| {
                                let (icon_rect, _) = ui.allocate_exact_size(
                                    egui::vec2(22.0, 22.0),
                                    egui::Sense::hover(),
                                );
                                icons::Icon::Assembly.draw(ui.painter(), icon_rect, pal.accent);
                                ui.label(
                                    egui::RichText::new("Untitled Assembly")
                                        .strong()
                                        .size(13.0)
                                        .color(pal.text_body),
                                );
                            });
                            ui.add_space(16.0);
                            ui.label(
                                egui::RichText::new("COMPONENTS")
                                    .strong()
                                    .size(10.5)
                                    .color(pal.text_muted),
                            );
                            ui.add_space(7.0);
                            ui.label(
                                egui::RichText::new("No components")
                                    .size(12.0)
                                    .color(pal.text_faint),
                            );
                        });

                    ui.add_space(12.0);

                    let size = ui.available_size() - egui::vec2(0.0, 2.0);
                    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
                    let painter = ui.painter_at(rect);
                    painter.rect(rect, 8.0, pal.surface, egui::Stroke::new(1.0, pal.border));

                    let icon_rect = egui::Rect::from_center_size(
                        rect.center() - egui::vec2(0.0, 58.0),
                        egui::vec2(52.0, 52.0),
                    );
                    painter.rect_filled(icon_rect.expand(16.0), 18.0, pal.accent_soft);
                    icons::Icon::Assembly.draw(&painter, icon_rect, pal.accent);
                    painter.text(
                        rect.center() + egui::vec2(0.0, 17.0),
                        egui::Align2::CENTER_CENTER,
                        "Empty assembly",
                        egui::FontId::proportional(20.0),
                        pal.text_strong,
                    );
                    painter.text(
                        rect.center() + egui::vec2(0.0, 48.0),
                        egui::Align2::CENTER_CENTER,
                        "The assembly workspace is active and ready for component insertion.",
                        egui::FontId::proportional(12.5),
                        pal.text_muted,
                    );
                    painter.text(
                        rect.center() + egui::vec2(0.0, 70.0),
                        egui::Align2::CENTER_CENTER,
                        "Part insertion and placement tools are the next assembly step.",
                        egui::FontId::proportional(11.5),
                        pal.text_faint,
                    );
                });
            });
    }
}
