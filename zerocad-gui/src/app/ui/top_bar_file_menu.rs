use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_top_bar_file_menu(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        // File Button Tab dropdown menu
        let file_btn_id = ui.make_persistent_id("file_menu_dropdown");
        let file_btn = icons::Icon::Folder.labeled_button(
            ui,
            "File",
            egui::Color32::from_rgb(241, 245, 249), // Clean slate grey
            egui::Color32::from_rgb(226, 232, 240), // Hover
            self.pal().text_body,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
        );
        if file_btn.clicked() {
            ui.memory_mut(|mem| mem.toggle_popup(file_btn_id));
        }
        egui::popup_below_widget::<()>(ui, file_btn_id, &file_btn, |ui| {
            ui.set_min_width(180.0);
            ui.style_mut().spacing.button_padding = egui::vec2(16.0, 6.0);

            // Shortcut hint for a menu action, taken from the live keymap.
            let hint = |app: &ZeroCadApp, action: ShortcutAction| {
                app.keymap
                    .get(action)
                    .map(|h| h.label())
                    .unwrap_or_default()
            };

            if icons::Icon::New
                .menu_button_hint(ui, "New Design", &hint(self, ShortcutAction::NewDesign))
                .clicked()
            {
                ui.memory_mut(|mem| mem.close_popup());
                self.new_design();
            }

            if icons::Icon::Save
                .menu_button_hint(ui, "Save Design", &hint(self, ShortcutAction::SaveDesign))
                .clicked()
            {
                ui.memory_mut(|mem| mem.close_popup());
                self.open_save_dialog();
            }

            if icons::Icon::Download
                .menu_button_hint(ui, "Open Design", &hint(self, ShortcutAction::OpenDesign))
                .clicked()
            {
                ui.memory_mut(|mem| mem.close_popup());
                self.open_design();
            }

            ui.separator();

            if icons::Icon::Download
                .menu_button_hint(ui, "Export STL", &hint(self, ShortcutAction::ExportStl))
                .clicked()
            {
                ui.memory_mut(|mem| mem.close_popup());
                self.export_stl();
            }

            ui.separator();

            if icons::Icon::Settings
                .menu_button_hint(ui, "Settings", &hint(self, ShortcutAction::OpenSettings))
                .clicked()
            {
                ui.memory_mut(|mem| mem.close_popup());
                log::info!("Opening Settings window.");
                self.show_preferences = true;
            }

            ui.separator();

            if icons::Icon::Exit.menu_button(ui, "Exit ZeroCAD").clicked() {
                ui.memory_mut(|mem| mem.close_popup());
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        });
    }
}
