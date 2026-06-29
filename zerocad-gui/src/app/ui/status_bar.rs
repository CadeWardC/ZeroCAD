use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_status_bar(&mut self, ctx: &egui::Context) {
        // BOTTOM PANEL: Status Bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

                // Show an elegant status icon based on contents
                let status_icon = if self.status_msg.starts_with("Error") {
                    "❌"
                } else if self.status_msg.starts_with("Click")
                    || self.status_msg.starts_with("Hover")
                    || self.status_msg.starts_with("Select")
                {
                    "🖱️"
                } else if self.status_msg.starts_with("Toggled") {
                    "📐"
                } else {
                    "ℹ️"
                };

                ui.label(egui::RichText::new(status_icon).size(11.0));
                ui.label(
                    egui::RichText::new(&self.status_msg)
                        .size(11.5)
                        .color(self.pal().text_body), // Slate-600
                );

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(ref err) = self.error_msg {
                        ui.label(
                            egui::RichText::new(format!("⚠️ {}", err))
                                .size(11.5)
                                .color(egui::Color32::from_rgb(220, 38, 38)) // Red-600
                                .strong(),
                        );
                    } else {
                        let (verts, tris) = self.mesh_stats;
                        ui.label(
                            egui::RichText::new(format!(
                                "Vertices: {}  |  Triangles: {}",
                                verts, tris
                            ))
                            .size(11.0)
                            .color(self.pal().text_faint), // Slate-400
                        );
                    }
                });
            });
        });
    }
}
