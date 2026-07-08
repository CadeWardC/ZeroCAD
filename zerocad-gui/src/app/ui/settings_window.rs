use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_settings_window(&mut self, ctx: &egui::Context) {
        // SETTINGS WINDOW (floating, modal-style; tab rail left, content right)
        if self.show_preferences {
            let mut open = self.show_preferences;
            let strong = self.pal().text_strong;
            egui::Window::new("Settings")
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .default_width(460.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::hover());
                        icons::Icon::Settings.draw(ui.painter(), rect, strong);
                        ui.label(egui::RichText::new("Settings").strong().size(14.0));
                    });
                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(8.0);

                    // Fixed-height row so the vertical separator below doesn't
                    // stretch to fill the whole (auto-sized) window.
                    ui.allocate_ui_with_layout(
                        egui::vec2(444.0, 270.0),
                        egui::Layout::left_to_right(egui::Align::Min),
                        |ui| {
                            // Left rail: tab list.
                            ui.allocate_ui_with_layout(
                                egui::vec2(110.0, 270.0),
                                egui::Layout::top_down_justified(egui::Align::Min),
                                |ui| {
                                    for &tab in SettingsTab::ALL {
                                        ui.selectable_value(
                                            &mut self.settings_tab,
                                            tab,
                                            tab.label(),
                                        );
                                    }
                                },
                            );

                            ui.separator();

                            // Right pane: content for the selected tab.
                            ui.vertical(|ui| {
                                match self.settings_tab {
                                    SettingsTab::General => {
                                        // --- Units ---
                                        ui.label(
                                            egui::RichText::new("Default measurement unit")
                                                .strong(),
                                        );
                                        ui.add_space(2.0);
                                        egui::ComboBox::from_id_salt("pref_unit_select")
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

                                        ui.add_space(12.0);

                                        // --- Onboarding ---
                                        ui.checkbox(&mut self.show_onboarding, "Onboarding Screen");

                                        ui.add_space(12.0);

                                        // --- Renderer ---
                                        ui.label(egui::RichText::new("Viewport").strong());
                                        ui.add_space(2.0);
                                        ui.checkbox(
                                            &mut self.gpu_render,
                                            "GPU-accelerated 3D rendering",
                                        );
                                        ui.weak(
                                            "Off falls back to the CPU software renderer (slower, \
                                             for comparison/troubleshooting).",
                                        );
                                        if self.gpu_render && !self.gpu.is_available() {
                                            ui.colored_label(
                                                egui::Color32::from_rgb(200, 140, 40),
                                                "GPU renderer isn't available on this system — \
                                                 the CPU renderer is being used.",
                                            );
                                        }

                                        ui.add_space(6.0);
                                        egui::ComboBox::from_label("Anti-aliasing")
                                            .selected_text(self.msaa_level.label())
                                            .show_ui(ui, |ui| {
                                                for level in settings::MsaaLevel::ALL {
                                                    ui.selectable_value(
                                                        &mut self.msaa_level,
                                                        level,
                                                        level.label(),
                                                    );
                                                }
                                            });
                                        ui.weak(
                                            "Smooths jagged silhouettes and wireframe edges. \
                                             Applies immediately; 8× falls back to 4× where \
                                             unsupported.",
                                        );

                                        ui.add_space(6.0);
                                        egui::ComboBox::from_label("Graphics backend")
                                            .selected_text(self.graphics_backend.label())
                                            .show_ui(ui, |ui| {
                                                for b in settings::GraphicsBackend::ALL {
                                                    ui.selectable_value(
                                                        &mut self.graphics_backend,
                                                        b,
                                                        b.label(),
                                                    );
                                                }
                                            });
                                        ui.weak(
                                            "Which GPU API the viewport uses. Takes effect after \
                                             restarting ZeroCAD; if the chosen backend can't \
                                             start, the app falls back to OpenGL with the CPU \
                                             renderer.",
                                        );
                                        if let Some(info) = self.gpu.adapter_info() {
                                            ui.weak(format!("Currently rendering on: {info}"));
                                        }
                                    }
                                    SettingsTab::Shortcuts => {
                                        ui.label(
                                            egui::RichText::new("Keyboard shortcuts").strong(),
                                        );
                                        ui.add_space(2.0);
                                        ui.weak(
                                        "Click a shortcut, then press the new combo. Esc cancels.",
                                    );
                                        ui.add_space(8.0);

                                        // Deferred mutations so the keymap isn't borrowed
                                        // mutably while the rows read it.
                                        let mut toggle_capture: Option<ShortcutAction> = None;
                                        let mut clear_action: Option<ShortcutAction> = None;

                                        egui::ScrollArea::vertical().max_height(190.0).show(
                                            ui,
                                            |ui| {
                                                for &action in ShortcutAction::ALL {
                                                    ui.horizontal(|ui| {
                                                        ui.add_sized(
                                                            [150.0, 24.0],
                                                            egui::Label::new(action.label()),
                                                        );
                                                        let capturing =
                                                            self.capturing_shortcut == Some(action);
                                                        let text = if capturing {
                                                            "Press a key…".to_string()
                                                        } else {
                                                            self.keymap
                                                                .get(action)
                                                                .map(|h| h.label())
                                                                .unwrap_or_else(|| {
                                                                    "Unbound".to_string()
                                                                })
                                                        };
                                                        let mut btn = egui::Button::new(text)
                                                            .min_size(egui::vec2(120.0, 24.0));
                                                        if capturing {
                                                            btn =
                                                                btn.fill(egui::Color32::from_rgb(
                                                                    0, 120, 215,
                                                                ));
                                                        }
                                                        if ui.add(btn).clicked() {
                                                            toggle_capture = Some(action);
                                                        }
                                                        if ui
                                                            .small_button("✕")
                                                            .on_hover_text("Unbind")
                                                            .clicked()
                                                        {
                                                            clear_action = Some(action);
                                                        }
                                                    });
                                                    ui.add_space(2.0);
                                                }
                                            },
                                        );

                                        ui.add_space(10.0);
                                        if ui.button("Reset to defaults").clicked() {
                                            self.keymap.reset_to_defaults();
                                            self.keymap.save();
                                            self.capturing_shortcut = None;
                                        }

                                        // Apply the deferred row actions.
                                        if let Some(action) = toggle_capture {
                                            // Clicking the row already capturing cancels it.
                                            self.capturing_shortcut =
                                                if self.capturing_shortcut == Some(action) {
                                                    None
                                                } else {
                                                    Some(action)
                                                };
                                        }
                                        if let Some(action) = clear_action {
                                            self.keymap.unbind(action);
                                            self.keymap.save();
                                            if self.capturing_shortcut == Some(action) {
                                                self.capturing_shortcut = None;
                                            }
                                        }
                                    }
                                }
                            });
                        },
                    );

                    ui.add_space(12.0);
                    ui.separator();
                    ui.add_space(8.0);

                    ui.horizontal(|ui| {
                        if ui
                            .add(egui::Button::new("Done").min_size(egui::vec2(80.0, 28.0)))
                            .clicked()
                        {
                            self.show_preferences = false;
                        }
                        ui.weak("Changes apply immediately.");
                    });
                    ui.add_space(4.0);
                });
            // Respect the window's close (✕) button as well as the Done button.
            if !open {
                self.show_preferences = false;
            }
        }
    }
}
