use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_top_bar_file_menu(&mut self, ui: &mut egui::Ui, _ctx: &egui::Context) {
        // Compact application menu; the surrounding global header carries the
        // project title and quick actions.
        let file_btn_id = ui.make_persistent_id("file_menu_dropdown");
        let file_btn = icons::Icon::Menu.icon_button(
            ui,
            egui::Color32::TRANSPARENT,
            self.pal().accent_soft,
            self.pal().text_body,
        );
        let file_btn = file_btn.on_hover_text("Main menu");
        if file_btn.clicked() {
            ui.memory_mut(|mem| mem.toggle_popup(file_btn_id));
        }
        egui::popup_below_widget::<()>(
            ui,
            file_btn_id,
            &file_btn,
            egui::PopupCloseBehavior::CloseOnClickOutside,
            |ui| {
                ui.set_min_width(180.0);
                ui.style_mut().spacing.button_padding = egui::vec2(16.0, 6.0);

                // Shortcut hint for a menu action, taken from the live keymap.
                let hint = |app: &ZeroCadApp, action: ShortcutAction| {
                    app.keymap
                        .get(action)
                        .map(|h| h.label())
                        .unwrap_or_default()
                };

                if icons::Icon::Home.menu_button(ui, "Start Page").clicked() {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.onboarding_visible = true;
                }

                ui.separator();

                if icons::Icon::New
                    .menu_button_hint(ui, "New Part", &hint(self, ShortcutAction::NewDesign))
                    .clicked()
                {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.new_design();
                }

                if icons::Icon::Assembly
                    .menu_button(ui, "New Assembly")
                    .clicked()
                {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.new_assembly();
                }

                let save_label = match self.project_kind {
                    ProjectKind::Part => "Save Design",
                    ProjectKind::Assembly => "Save Assembly",
                };
                if icons::Icon::Save
                    .menu_button_hint(ui, save_label, &hint(self, ShortcutAction::SaveDesign))
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

                if self.project_kind == ProjectKind::Part {
                    ui.separator();

                    if icons::Icon::Download
                        .menu_button(ui, "Import STEP")
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.import_step();
                    }

                    if icons::Icon::Download
                        .menu_button(ui, "Import STL")
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.import_stl();
                    }

                    if icons::Icon::Download
                        .menu_button(ui, "Import DXF to Sketch")
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.import_dxf();
                    }

                    if icons::Icon::Download
                        .menu_button_hint(ui, "Export STL", &hint(self, ShortcutAction::ExportStl))
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.export_stl();
                    }

                    if icons::Icon::Download
                        .menu_button(ui, "Export 3MF")
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.export_3mf();
                    }

                    ui.separator();

                    if icons::Icon::Settings
                        .menu_button(ui, "Parameters")
                        .clicked()
                    {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.open_parameters_dialog();
                    }

                    if icons::Icon::EyeOpen.menu_button(ui, "Inspect").clicked() {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.open_inspection_dialog();
                    }
                }

                if icons::Icon::Settings
                    .menu_button_hint(ui, "Settings", &hint(self, ShortcutAction::OpenSettings))
                    .clicked()
                {
                    ui.memory_mut(|mem| mem.close_popup());
                    log::info!("Opening Settings window.");
                    self.show_preferences = true;
                }

                ui.separator();

                let mut recover_clicked = false;
                ui.add_enabled_ui(self.recovery.has_recovery(), |ui| {
                    recover_clicked = icons::Icon::Download
                        .menu_button(ui, "Recover Autosave")
                        .on_hover_text("Restore the latest crash-safe local recovery document")
                        .clicked();
                });
                if recover_clicked {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.recover_latest_autosave();
                }

                if icons::Icon::Download.menu_button(ui, "Export Bug Report…").on_hover_text(
                    "Create an offline ZIP containing this document, build identity, diagnostics, and session log",
                ).clicked() {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.export_bug_report();
                }

                if icons::Icon::Settings
                    .menu_button(ui, "About ZeroCAD")
                    .clicked()
                {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.show_about = true;
                }

                ui.separator();

                if icons::Icon::Exit.menu_button(ui, "Exit ZeroCAD").clicked() {
                    ui.memory_mut(|mem| mem.close_popup());
                    self.request_window_close();
                }
            },
        );
    }
}
