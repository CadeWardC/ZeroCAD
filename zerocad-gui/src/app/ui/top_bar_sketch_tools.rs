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
                            if btn.on_hover_text("Draw individual line segments (L)").clicked() {
                                self.active_tool = Some(SketchTool::Line);
                                self.cancel_in_progress_shape();
                                self.clear_pending_corners();
                                log::info!("Switched to Line tool");
                            }
                        }

                        // Rectangle, Circle and the corner tool each expose a mode
                        // flyout: click the active button again (or right-click it)
                        // to choose corner/center/3-point, ellipse, or Fillet ↔
                        // Chamfer. The corner button is a single button (like the 3D
                        // edge fillet/chamfer) whose flyout switches the two kinds.
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
                                ToolFamily::Corner,
                                "Fillet",
                                "Fillet / Chamfer — set the radius, then click a corner. Click again or right-click to switch kind",
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
                                    self.clear_pending_corners();
                                    ui.memory_mut(|m| m.close_popup());
                                }
                            }
                            if btn.secondary_clicked() {
                                ui.memory_mut(|m| m.open_popup(popup_id));
                            }

                            egui::popup_below_widget(ui, popup_id, &btn, |ui| {
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
                                        self.clear_pending_corners();
                                        ui.memory_mut(|m| m.close_popup());
                                        log::info!("Switched to {:?}", mode);
                                    }
                                }
                            });
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

            // Curve statistics, Undo / Clear Sketch row
            let curve_count = self.sketch_curves.segments.len() + self.sketch_curves.circles.len();
            if curve_count > 0 {
                ui.separator();
                ui.label(
                    egui::RichText::new(format!(
                        "Curves: {} · Faces: {}",
                        curve_count,
                        self.detected_regions.len()
                    ))
                    .color(self.pal().text_body)
                    .size(12.0),
                );

                let undo_btn = ui.add(
                    egui::Button::new(egui::RichText::new("↩ Undo").size(12.0))
                        .fill(egui::Color32::from_rgb(241, 245, 249))
                        .rounding(egui::Rounding::same(4.0)),
                );
                if undo_btn.on_hover_text("Undo last drawn shape").clicked() {
                    // One undo removes a whole shape (not a single
                    // segment), then the live curves are rebuilt.
                    self.sketch_shapes.pop();
                    self.rebuild_active_sketch_curves();
                }

                let reset_btn = icons::Icon::Trash.labeled_button(
                    ui,
                    "Clear",
                    egui::Color32::from_rgb(254, 242, 242),
                    egui::Color32::from_rgb(254, 226, 226),
                    egui::Color32::from_rgb(185, 28, 28),
                    egui::Stroke::NONE,
                );
                if reset_btn
                    .on_hover_text("Clear all curves in current sketch")
                    .clicked()
                {
                    self.reset_sketch_state();
                }
            }
        }
    }
}
