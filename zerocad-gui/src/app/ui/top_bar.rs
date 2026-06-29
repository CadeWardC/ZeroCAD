use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_top_bar(&mut self, ctx: &egui::Context) {
        // TOP PANEL: Operations Toolbar
        egui::TopBottomPanel::top("operations_toolbar")
            .exact_height(48.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.style_mut().spacing.button_padding = egui::vec2(12.0, 7.0);
                    ui.style_mut().spacing.item_spacing = egui::vec2(10.0, 10.0);

                    // Premium branded logo
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(1.0, 0.0);
                        ui.label(
                            egui::RichText::new("Zero")
                                .strong()
                                .size(18.0)
                                .color(self.pal().text_strong),
                        );
                        ui.label(
                            egui::RichText::new("CAD")
                                .strong()
                                .size(18.0)
                                .color(egui::Color32::from_rgb(37, 99, 235)),
                        );
                    });

                    self.draw_top_bar_file_menu(ui, ctx);

                    ui.separator();

                    let active_sketching = self.is_sketch_mode || self.is_plane_selection_mode;
                    self.draw_top_bar_modeling_commands(ui, ctx, active_sketching);
                    self.draw_sketch_tool_strip(ui);
                });
            });
    }
}
