use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_sketch_tool_strip(&mut self, ui: &mut egui::Ui) {
        if self.is_sketch_mode {
            ui.separator();

            // Premium control tabs with custom vector graphics for Sketch Tools
            ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

                        // Helper to draw a beautiful tool button
                        let pal = self.pal();
                        let draw_tool_btn = |ui: &mut egui::Ui, is_active: bool, text: &str, icon: Option<icons::Icon>| -> egui::Response {
                            let button_color = if is_active {
                                egui::Color32::from_rgb(219, 234, 254) // Active light blue
                            } else {
                                egui::Color32::from_rgb(241, 245, 249) // Slate grey
                            };
                            let text_color = if is_active {
                                egui::Color32::from_rgb(29, 78, 216) // Solid active blue
                            } else {
                                pal.text_body // Muted slate text
                            };
                            let stroke = if is_active {
                                egui::Stroke::new(1.5, egui::Color32::from_rgb(59, 130, 246))
                            } else {
                                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225))
                            };

                            let desired_size = egui::vec2(95.0, 28.0);
                            let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click());

                            if ui.is_rect_visible(rect) {
                                let bg_fill = if response.hovered() {
                                    if is_active { egui::Color32::from_rgb(191, 219, 254) } else { egui::Color32::from_rgb(226, 232, 240) }
                                } else {
                                    button_color
                                };

                                ui.painter().rect(rect, 4.0, bg_fill, stroke);

                                // Position and paint icon
                                let icon_rect = egui::Rect::from_min_size(rect.left_top() + egui::vec2(8.0, 6.0), egui::vec2(16.0, 16.0));
                                if let Some(ic) = icon {
                                    ic.draw(ui.painter(), icon_rect, text_color);
                                } else {
                                    // Custom pointer polygon for "Select" tool
                                    let p = icon_rect.left_top();
                                    let points = vec![
                                        p + egui::vec2(3.0, 2.0),
                                        p + egui::vec2(3.0, 13.0),
                                        p + egui::vec2(6.0, 10.0),
                                        p + egui::vec2(10.0, 10.0),
                                    ];
                                    ui.painter().add(egui::Shape::convex_polygon(points, text_color, egui::Stroke::new(1.0, text_color)));
                                }

                                // Position and paint text
                                let text_pos = rect.left_top() + egui::vec2(28.0, 6.0);
                                ui.painter().text(
                                    text_pos,
                                    egui::Align2::LEFT_TOP,
                                    text,
                                    egui::FontId::proportional(12.0),
                                    text_color
                                );
                            }

                            response
                        };

                        // No explicit "Select" button: pressing Esc returns to the
                        // neutral Select state (`active_tool = None`), which lets the
                        // user pick body faces/edges/vertices without leaving the
                        // sketch — see the global Escape handler below.

                        // Line Tool (single mode, no flyout).
                        {
                            let is_active = self.active_tool == Some(SketchTool::Line);
                            let btn = draw_tool_btn(ui, is_active, "Line", Some(icons::Icon::Line));
                            if btn
                                .on_hover_text(
                                    "Draw connected lines — chain segments and click the start to close a loop into a face (L)",
                                )
                                .clicked()
                            {
                                self.active_tool = Some(SketchTool::Line);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                log::info!("Switched to Line tool");
                            }
                        }

                        // Keep the primary profile tools together at the start
                        // of the strip: Line, Rectangle, Circle, then Polygon.
                        for (family, key, hover) in [
                            (
                                ToolFamily::Rectangle,
                                "Rectangle",
                                "Rectangle (R) — click again or right-click for modes",
                            ),
                            (
                                ToolFamily::Circle,
                                "Circle",
                                "Circle (C) — click again or right-click for ellipse / 3-point modes",
                            ),
                            (
                                ToolFamily::Polygon,
                                "Polygon",
                                "Regular polygon — set the side count, click the center, then set the hidden guide-circle diameter. Click again or right-click for inscribed / circumscribed",
                            ),
                        ] {
                            let active = self.active_tool.is_some_and(|tool| tool.family() == family);
                            let icon = if active {
                                self.active_tool.unwrap().icon()
                            } else {
                                family.default_mode().icon()
                            };
                            let btn =
                                draw_tool_btn(ui, active, key, Some(icon)).on_hover_text(hover);
                            let popup_id = ui.make_persistent_id(("tool_flyout", key));

                            if btn.clicked() {
                                if active {
                                    ui.memory_mut(|memory| memory.toggle_popup(popup_id));
                                } else {
                                    self.active_tool = Some(family.default_mode());
                                    self.cancel_in_progress_shape();
                                    self.line_chain_start = None;
                                    self.clear_pending_corners();
                                    ui.memory_mut(|memory| memory.close_popup());
                                }
                            }
                            if btn.secondary_clicked() {
                                ui.memory_mut(|memory| memory.open_popup(popup_id));
                            }

                            egui::popup_below_widget(
                                ui,
                                popup_id,
                                &btn,
                                egui::PopupCloseBehavior::CloseOnClickOutside,
                                |ui| {
                                    ui.set_min_width(180.0);
                                    for &mode in family.modes() {
                                        let selected = self.active_tool == Some(mode);
                                        let prefix = if selected { "● " } else { "   " };
                                        let row = mode
                                            .icon()
                                            .menu_button(ui, &format!("{prefix}{}", mode.label()));
                                        if row.clicked() {
                                            self.active_tool = Some(mode);
                                            self.cancel_in_progress_shape();
                                            self.line_chain_start = None;
                                            self.clear_pending_corners();
                                            ui.memory_mut(|memory| memory.close_popup());
                                            log::info!("Switched to {:?}", mode);
                                        }
                                    }
                                },
                            );
                        }

                        // Keep polygon configuration beside the Polygon button.
                        if self
                            .active_tool
                            .is_some_and(|tool| tool.family() == ToolFamily::Polygon)
                        {
                            ui.label(
                                egui::RichText::new("Sides:")
                                    .size(12.0)
                                    .color(self.pal().text_body),
                            );
                            ui.add(
                                egui::DragValue::new(&mut self.polygon_sides)
                                    .range(3..=64)
                                    .speed(0.1),
                            );
                        }

                        // Mirror Tool (single mode, no flyout). Click two points
                        // to reflect the whole sketch across that axis.
                        {
                            let is_active = self.active_tool == Some(SketchTool::Mirror);
                            let btn = draw_tool_btn(ui, is_active, "Mirror", Some(icons::Icon::Mirror));
                            if btn
                                .on_hover_text(
                                    "Mirror the whole sketch — click two points to set the reflection axis",
                                )
                                .clicked()
                            {
                                self.active_tool = Some(SketchTool::Mirror);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                log::info!("Switched to Mirror tool");
                            }
                        }

                        // Native center-to-center slot. Two clicks set the cap
                        // centers; the third sets the full width.
                        {
                            let is_active = self.active_tool == Some(SketchTool::Slot);
                            let btn = draw_tool_btn(ui, is_active, "Slot", Some(icons::Icon::Slot));
                            if btn
                                .on_hover_text(
                                    "Center-to-center slot — click both end centers, then set the width",
                                )
                                .clicked()
                            {
                                self.active_tool = Some(SketchTool::Slot);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                log::info!("Switched to Slot tool");
                            }
                        }

                        // Shaped text: opens a dialog (string, font, size,
                        // placement) and commits baked glyph outlines as one
                        // Text shape. Not a click-driven draw tool.
                        {
                            let btn = draw_tool_btn(
                                ui,
                                self.text_dialog.is_some(),
                                "Text",
                                None,
                            );
                            if btn
                                .on_hover_text(
                                    "Add text — shaped with an installed font, baked into the sketch",
                                )
                                .clicked()
                            {
                                self.open_text_dialog();
                                log::info!("Opened the sketch Text dialog");
                            }
                        }

                        // Associative analytic offset. Source selection happens
                        // in the viewport; a click away from the selected chain
                        // supplies the creation-side seed and distance.
                        {
                            let is_active = self.active_tool == Some(SketchTool::Offset);
                            let btn =
                                draw_tool_btn(ui, is_active, "Offset", Some(icons::Icon::Offset));
                            if btn
                                .on_hover_text(
                                    "Offset connected lines/arcs or a circle — select sources, then click the desired side",
                                )
                                .clicked()
                            {
                                self.ensure_active_solver_model();
                                self.active_tool = Some(SketchTool::Offset);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                self.status_msg =
                                    "Click an offset source. Shift-click to extend the chain, then click the desired side."
                                        .to_string();
                                log::info!("Switched to Offset tool");
                            }
                        }

                        // Parametric curve Trim. Hover computes an immutable
                        // removable-span plan and the click commits that same plan.
                        {
                            let is_active = self.active_tool == Some(SketchTool::Trim);
                            let btn =
                                draw_tool_btn(ui, is_active, "Trim", Some(icons::Icon::Trim));
                            if btn
                                .on_hover_text(
                                    "Trim a line, arc, circle, ellipse, or spline — hover the removable span, then click",
                                )
                                .clicked()
                            {
                                self.ensure_active_solver_model();
                                self.active_tool = Some(SketchTool::Trim);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                self.status_msg =
                                    "Hover a curve to preview the exact removable span."
                                        .to_string();
                                log::info!("Switched to Trim tool");
                            }
                        }

                        // Unified driving-dimension command. Geometry clicks
                        // select the target; a click in empty space places the
                        // inferred length, distance, radius/diameter, or angle.
                        {
                            let is_active = self.active_tool == Some(SketchTool::Dimension);
                            let btn = draw_tool_btn(
                                ui,
                                is_active,
                                "Dimension",
                                Some(icons::Icon::Dimension),
                            );
                            if btn
                                .on_hover_text(
                                    "Dimension — select geometry, click to place, then enter a value, expression, or variable",
                                )
                                .clicked()
                            {
                                self.ensure_active_solver_model();
                                self.active_tool = Some(SketchTool::Dimension);
                                self.cancel_in_progress_shape();
                                self.line_chain_start = None;
                                self.clear_pending_corners();
                                self.sketch_selected_ids.clear();
                                self.status_msg =
                                    "Select one line, one circle/arc, two points, or two lines."
                                        .to_string();
                                log::info!("Switched to Dimension tool");
                            }
                        }

                        let has_projectable_edge = self
                            .selected_body
                            .iter()
                            .any(|(_, pick)| matches!(pick, BodyPick::Edge(_)));
                        let project = ui
                            .add_enabled(
                                has_projectable_edge,
                                egui::Button::new("Project Edge"),
                            )
                            .on_hover_text(
                                "Project selected body edges into this sketch as construction geometry",
                            );
                        if project.clicked() {
                            self.project_selected_edges_to_sketch();
                        }

                        // Spline follows the editing tools; the corner tool is
                        // rendered after every other sketch action below.
                        for (family, key, hover) in [
                            (
                                ToolFamily::Spline,
                                "Spline",
                                "Control-point / fit-point spline — click points, then double-click or press Enter to finish",
                            ),
                        ] {
                            let active = self.active_tool.map_or(false, |t| t.family() == family);
                            // The button shows the icon of the active sub-mode so
                            // the user sees which variant is armed.
                            let icon = if active {
                                self.active_tool.unwrap().icon()
                            } else {
                                family.default_mode().icon()
                            };
                            // Fixed label per button, except the corner button shows
                            // the armed kind (Fillet vs Chamfer) so it's visible which
                            // is set — the two labels are short enough to fit.
                            let label = if family == ToolFamily::Corner && active {
                                self.active_tool.unwrap().label()
                            } else {
                                key
                            };
                            let btn = draw_tool_btn(ui, active, label, Some(icon))
                                .on_hover_text(hover);
                            let popup_id = ui.make_persistent_id(("tool_flyout", key));

                            if btn.clicked() {
                                if active {
                                    // Re-clicking the armed tool opens the flyout.
                                    ui.memory_mut(|m| m.toggle_popup(popup_id));
                                } else {
                                    self.active_tool = Some(family.default_mode());
                                    self.cancel_in_progress_shape();
                                    self.line_chain_start = None;
                                    self.clear_pending_corners();
                                    ui.memory_mut(|m| m.close_popup());
                                }
                            }
                            if btn.secondary_clicked() {
                                ui.memory_mut(|m| m.open_popup(popup_id));
                            }

                            egui::popup_below_widget(
                                ui,
                                popup_id,
                                &btn,
                                egui::PopupCloseBehavior::CloseOnClickOutside,
                                |ui| {
                                ui.set_min_width(180.0);
                                for &mode in family.modes() {
                                    let selected = self.active_tool == Some(mode);
                                    let prefix = if selected { "● " } else { "   " };
                                    let row = mode.icon().menu_button(
                                        ui,
                                        &format!("{}{}", prefix, mode.label()),
                                    );
                                    if row.clicked() {
                                        self.active_tool = Some(mode);
                                        self.cancel_in_progress_shape();
                                        self.line_chain_start = None;
                                        self.clear_pending_corners();
                                        ui.memory_mut(|m| m.close_popup());
                                        log::info!("Switched to {:?}", mode);
                                    }
                                }
                            });
                        }

                        if self.active_tool == Some(SketchTool::Offset) {
                            ui.label(
                                egui::RichText::new("D:")
                                    .size(12.0)
                                    .color(self.pal().text_body),
                            );
                            ui.add(
                                egui::TextEdit::singleline(&mut self.offset_distance_text)
                                    .desired_width(58.0)
                                    .hint_text("click"),
                            )
                            .on_hover_text(
                                "Optional distance or expression; leave empty to use the placement click",
                            );
                            let can_dissolve = self
                                .sketch_solver_model
                                .as_ref()
                                .is_some_and(|model| !model.offsets.is_empty());
                            if ui
                                .add_enabled(can_dissolve, egui::Button::new("Dissolve Offset"))
                                .on_hover_text(
                                    "Convert the most recent associative offset into editable entities",
                                )
                                .clicked()
                            {
                                self.dissolve_last_sketch_offset();
                            }
                        }

                        let can_pattern = self.sketch_solver_model.is_some()
                            && !self.sketch_selected_ids.is_empty();
                        let mut create_linear_pattern = false;
                        let mut create_circular_pattern = false;
                        if can_pattern {
                            ui.menu_button("Sketch Pattern", |ui| {
                                ui.set_min_width(245.0);
                                ui.label("X/Y: direction (Linear) or center (Circular)");
                                ui.horizontal(|ui| {
                                    ui.label("X");
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut self.sketch_pattern_x_text,
                                        )
                                        .desired_width(65.0),
                                    );
                                    ui.label("Y");
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut self.sketch_pattern_y_text,
                                        )
                                        .desired_width(65.0),
                                    );
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Spacing");
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut self.sketch_pattern_spacing_text,
                                        )
                                        .desired_width(65.0),
                                    );
                                    ui.label("Count");
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut self.sketch_pattern_count_text,
                                        )
                                        .desired_width(45.0),
                                    );
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Total angle");
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut self.sketch_pattern_angle_text,
                                        )
                                        .desired_width(65.0),
                                    );
                                    ui.label("°");
                                });
                                ui.separator();
                                if ui.button("Create Linear Pattern").clicked() {
                                    create_linear_pattern = true;
                                    ui.close_menu();
                                }
                                if ui.button("Create Circular Pattern").clicked() {
                                    create_circular_pattern = true;
                                    ui.close_menu();
                                }
                            });
                        }
                        let can_dissolve_pattern = self
                            .sketch_solver_model
                            .as_ref()
                            .is_some_and(|model| !model.patterns.is_empty());
                        if ui
                            .add_enabled(
                                can_dissolve_pattern,
                                egui::Button::new("Dissolve Pattern"),
                            )
                            .on_hover_text(
                                "Convert the most recent associative pattern into editable entities",
                            )
                            .clicked()
                        {
                            self.dissolve_last_sketch_pattern();
                        }
                        if create_linear_pattern {
                            self.create_sketch_pattern(false);
                        }
                        if create_circular_pattern {
                            self.create_sketch_pattern(true);
                        }

                        // Fillet/Chamfer is intentionally the final sketch tool.
                        // Re-click or right-click to switch between the two modes.
                        {
                            let family = ToolFamily::Corner;
                            let active =
                                self.active_tool.is_some_and(|tool| tool.family() == family);
                            let icon = if active {
                                self.active_tool.unwrap().icon()
                            } else {
                                family.default_mode().icon()
                            };
                            let label = if active {
                                self.active_tool.unwrap().label()
                            } else {
                                "Fillet"
                            };
                            let btn = draw_tool_btn(ui, active, label, Some(icon)).on_hover_text(
                                "Fillet / Chamfer — set the radius, then click a corner. Click again or right-click to switch kind",
                            );
                            let popup_id =
                                ui.make_persistent_id(("tool_flyout", "Fillet"));

                            if btn.clicked() {
                                if active {
                                    ui.memory_mut(|memory| memory.toggle_popup(popup_id));
                                } else {
                                    self.active_tool = Some(family.default_mode());
                                    self.cancel_in_progress_shape();
                                    self.line_chain_start = None;
                                    self.clear_pending_corners();
                                    ui.memory_mut(|memory| memory.close_popup());
                                }
                            }
                            if btn.secondary_clicked() {
                                ui.memory_mut(|memory| memory.open_popup(popup_id));
                            }

                            egui::popup_below_widget(
                                ui,
                                popup_id,
                                &btn,
                                egui::PopupCloseBehavior::CloseOnClickOutside,
                                |ui| {
                                    ui.set_min_width(180.0);
                                    for &mode in family.modes() {
                                        let selected = self.active_tool == Some(mode);
                                        let prefix = if selected { "● " } else { "   " };
                                        let row = mode
                                            .icon()
                                            .menu_button(ui, &format!("{prefix}{}", mode.label()));
                                        if row.clicked() {
                                            self.active_tool = Some(mode);
                                            self.cancel_in_progress_shape();
                                            self.line_chain_start = None;
                                            self.clear_pending_corners();
                                            ui.memory_mut(|memory| memory.close_popup());
                                            log::info!("Switched to {:?}", mode);
                                        }
                                    }
                                },
                            );
                        }

                        // Radius/distance input for the active corner tool, with
                        // a unit suffix. Editing it re-previews the staged corners
                        // live. An OK button (and Enter) commits the pending set.
                        if let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) {
                            let label = match kind {
                                CornerKind::Fillet => "R:",
                                CornerKind::Chamfer => "D:",
                            };
                            ui.label(egui::RichText::new(label).size(12.0).color(self.pal().text_body));
                            let changed = ui
                                .add(
                                    egui::TextEdit::singleline(&mut self.corner_radius_text)
                                        .desired_width(46.0)
                                        .hint_text("5"),
                                )
                                .changed();
                            let unit_suffix = match self.current_unit {
                                Unit::Millimeter => "mm",
                                Unit::Inch => "in",
                                Unit::Meter => "m",
                            };
                            ui.label(
                                egui::RichText::new(unit_suffix)
                                    .size(11.0)
                                    .color(self.pal().text_faint),
                            );
                            // Live: changing the radius re-previews the staged corners.
                            if changed && !self.pending_corners.is_empty() {
                                self.rebuild_active_sketch_curves();
                            }
                            if !self.pending_corners.is_empty() {
                                let ok = ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(format!(
                                            "✓ OK ({})",
                                            self.pending_corners.len()
                                        ))
                                        .size(12.0)
                                        .color(egui::Color32::WHITE),
                                    )
                                    .fill(egui::Color32::from_rgb(34, 139, 84))
                                    .rounding(egui::Rounding::same(4.0)),
                                );
                                if ok.on_hover_text("Apply the staged corners (Enter)").clicked() {
                                    self.commit_pending_corners();
                                }
                            }
                        }
                    });
        }
    }
}
