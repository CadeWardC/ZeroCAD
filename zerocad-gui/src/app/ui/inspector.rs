use crate::*;
use zerocad_core::sketch::{Constraint, SolveOutcome};

pub(super) const WORKSPACE_INSPECTOR_CONTENT_WIDTH: f32 = 246.0;
pub(super) const WORKSPACE_INSPECTOR_VIEWPORT_RESERVATION: f32 =
    WORKSPACE_INSPECTOR_CONTENT_WIDTH + 42.0;
const WORKSPACE_INSPECTOR_EDGE_PADDING: f32 = 16.0;

impl ZeroCadApp {
    pub(crate) fn inspector_has_content(&self) -> bool {
        self.extrude_op.is_some()
            || self.is_sketch_mode
            || self.is_plane_selection_mode
            || self.other_active_operation_name().is_some()
            || self.selected_node_id.is_some()
            || self.has_viewport_selection()
    }

    fn has_viewport_selection(&self) -> bool {
        !self.selected_body.is_empty()
            || !self.selected_faces.is_empty()
            || !self.selected_edges.is_empty()
    }

    pub(crate) fn draw_inspector(&mut self, ctx: &egui::Context, viewport: egui::Rect) {
        let content_width = WORKSPACE_INSPECTOR_CONTENT_WIDTH;
        let overlay_height = (viewport.height() - 32.0).clamp(280.0, 680.0);
        egui::Area::new(egui::Id::new("workspace_inspector_overlay"))
            .order(egui::Order::Foreground)
            .pivot(egui::Align2::RIGHT_TOP)
            .fixed_pos(egui::pos2(
                viewport.right() - WORKSPACE_INSPECTOR_EDGE_PADDING,
                viewport.top() + WORKSPACE_INSPECTOR_EDGE_PADDING,
            ))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .rounding(8.0)
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 3.0),
                        blur: 12.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(24),
                    })
                    .inner_margin(egui::Margin::symmetric(14.0, 10.0))
                    .show(ui, |ui| {
                        ui.set_width(content_width);
                        ui.set_max_height(overlay_height);
                        ui.add_space(9.0);
                        let title = self.inspector_title();
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(title)
                                    .strong()
                                    .size(12.0)
                                    .color(self.pal().text_strong),
                            );
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    icons::Icon::Inspect.draw(
                                        ui.painter(),
                                        egui::Rect::from_min_size(
                                            ui.cursor().min,
                                            egui::vec2(17.0, 17.0),
                                        ),
                                        self.pal().text_muted,
                                    );
                                    ui.add_space(17.0);
                                },
                            );
                        });
                        ui.add_space(5.0);
                        ui.separator();

                        if self.extrude_op.is_some() {
                            self.draw_extrude_inspector(ui);
                        } else if self.is_sketch_mode || self.is_plane_selection_mode {
                            self.draw_sketch_inspector(ui);
                        } else if let Some(operation) = self.other_active_operation_name() {
                            ui.add_space(12.0);
                            Self::inspector_section(ui, &self.pal(), "Active operation", |ui| {
                                ui.label(
                                    egui::RichText::new(operation)
                                        .strong()
                                        .size(14.0)
                                        .color(self.pal().accent),
                                );
                                ui.add_space(4.0);
                                ui.label(
                            egui::RichText::new(
                                "Adjust the live operation controls and preview in the viewport.",
                            )
                            .size(11.5)
                            .color(self.pal().text_muted),
                        );
                            });
                        } else if self.has_viewport_selection() {
                            ui.add_space(12.0);
                            Self::inspector_section(ui, &self.pal(), "Selection", |ui| {
                                if !self.selected_body.is_empty() {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{} solid element{} selected",
                                            self.selected_body.len(),
                                            if self.selected_body.len() == 1 {
                                                ""
                                            } else {
                                                "s"
                                            }
                                        ))
                                        .strong()
                                        .size(13.0)
                                        .color(self.pal().text_strong),
                                    );
                                }
                                if !self.selected_faces.is_empty() {
                                    ui.label(format!(
                                        "{} sketch profile{} selected",
                                        self.selected_faces.len(),
                                        if self.selected_faces.len() == 1 {
                                            ""
                                        } else {
                                            "s"
                                        }
                                    ));
                                }
                                if !self.selected_edges.is_empty() {
                                    ui.label(format!(
                                        "{} sketch edge{} selected",
                                        self.selected_edges.len(),
                                        if self.selected_edges.len() == 1 {
                                            ""
                                        } else {
                                            "s"
                                        }
                                    ));
                                }
                            });
                        } else if self.selected_node_id.is_some() {
                            egui::ScrollArea::vertical()
                                .id_salt("feature_properties_scroll")
                                .auto_shrink([false, false])
                                .show(ui, |ui| self.draw_selected_feature_properties(ui));
                        }
                    });
            });
    }

    fn inspector_title(&self) -> &'static str {
        if self.extrude_op.is_some() {
            "EXTRUDE"
        } else if self.is_sketch_mode || self.is_plane_selection_mode {
            "SKETCH"
        } else if let Some(name) = self.other_active_operation_name() {
            name
        } else {
            "INSPECTOR"
        }
    }

    fn other_active_operation_name(&self) -> Option<&'static str> {
        if self.revolve_op.is_some() {
            Some("REVOLVE")
        } else if self.sweep_op.is_some() {
            Some("SWEEP")
        } else if self.edge_mod_op.is_some() {
            Some("EDGE MODIFICATION")
        } else if self.hole_op.is_some() {
            Some("HOLE")
        } else if self.thread_op.is_some() {
            Some("THREAD")
        } else if self.shell_op.is_some() {
            Some("SHELL")
        } else if self.draft_op.is_some() {
            Some("DRAFT")
        } else if self.pattern_op.is_some() {
            Some("PATTERN")
        } else if self.move_op.is_some() {
            Some("MOVE")
        } else if self.combine_op.is_some() {
            Some("COMBINE")
        } else if self.split_body_op.is_some() {
            Some("SPLIT BODY")
        } else if self.scale_body_op.is_some() {
            Some("SCALE BODY")
        } else {
            None
        }
    }

    fn draw_extrude_inspector(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        let mut commit = false;
        let mut cancel = false;
        egui::ScrollArea::vertical()
            .id_salt("extrude_inspector_scroll")
            .show(ui, |ui| {
                ui.add_space(12.0);
                if let Some(op) = self.extrude_op.as_mut() {
                    let faces: usize = op.targets.iter().map(|target| target.indices.len()).sum();
                    Self::inspector_section(ui, &pal, "Profiles", |ui| {
                        egui::Frame::none()
                            .fill(pal.accent_soft)
                            .stroke(egui::Stroke::new(1.0, pal.accent))
                            .rounding(6.0)
                            .inner_margin(egui::Margin::symmetric(10.0, 8.0))
                            .show(ui, |ui| {
                                ui.label(
                                    egui::RichText::new(format!("{faces} selected"))
                                        .color(pal.accent)
                                        .size(12.0),
                                );
                            });
                    });

                    Self::inspector_section(ui, &pal, "Distance", |ui| {
                        let mut changed = ui
                            .add(
                                egui::DragValue::new(&mut op.depth)
                                    .speed(0.5)
                                    .suffix(self.current_unit.suffix()),
                            )
                            .changed();
                        if ui.button("Flip direction").clicked() {
                            op.depth = -op.depth;
                            changed = true;
                        }
                        if changed {
                            op.depth_text = format!("{:.2}", op.depth);
                        }
                        ui.label(
                            egui::RichText::new("Drag the viewport handle to push or pull.")
                                .size(10.5)
                                .color(pal.text_faint),
                        );
                    });

                    Self::inspector_section(ui, &pal, "Operation", |ui| {
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 3.0;
                            for (mode, label) in [
                                (ExtrudeMode::NewBody, "New Body"),
                                (ExtrudeMode::Join, "Join"),
                                (ExtrudeMode::Cut, "Cut"),
                            ] {
                                let selected = op.mode == mode;
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new(label).size(11.0).color(
                                                if selected {
                                                    egui::Color32::WHITE
                                                } else {
                                                    pal.text_body
                                                },
                                            ),
                                        )
                                        .fill(if selected {
                                            pal.accent
                                        } else {
                                            pal.surface_subtle
                                        })
                                        .stroke(egui::Stroke::new(
                                            1.0,
                                            if selected { pal.accent } else { pal.border },
                                        ))
                                        .rounding(5.0),
                                    )
                                    .clicked()
                                {
                                    op.mode = mode;
                                }
                            }
                        });
                    });
                }

                ui.add_space(14.0);
                ui.separator();
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("Create")
                                    .strong()
                                    .color(egui::Color32::WHITE),
                            )
                            .fill(pal.accent)
                            .stroke(egui::Stroke::NONE)
                            .rounding(6.0)
                            .min_size(egui::vec2(104.0, 34.0)),
                        )
                        .clicked()
                    {
                        commit = true;
                    }
                    if ui
                        .add(egui::Button::new("Cancel").min_size(egui::vec2(90.0, 34.0)))
                        .clicked()
                    {
                        cancel = true;
                    }
                });
                ui.label(
                    egui::RichText::new("Enter = Create  •  Esc = Cancel")
                        .size(10.0)
                        .color(pal.text_faint),
                );
            });
        if commit {
            self.commit_extrude_op();
        } else if cancel {
            self.cancel_extrude_op();
        }
    }

    fn draw_sketch_inspector(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        if self.is_plane_selection_mode {
            ui.add_space(12.0);
            Self::inspector_section(ui, &pal, "Choose a plane", |ui| {
                ui.label(
                    egui::RichText::new(
                        "Select an origin plane, datum plane, or planar body face in the viewport.",
                    )
                    .size(12.0)
                    .color(pal.text_muted),
                );
            });
            return;
        }

        let area: f32 = self.detected_regions.iter().map(|region| region.area).sum();
        let perimeter: f32 = self
            .detected_regions
            .iter()
            .map(|region| {
                polygon_perimeter(&region.boundary)
                    + region
                        .holes
                        .iter()
                        .map(|hole| polygon_perimeter(hole))
                        .sum::<f32>()
            })
            .sum();
        let vars = self.document.variable_map();
        let solve = self
            .sketch_solver_model
            .as_ref()
            .map(|model| zerocad_core::sketch::solve_model(model, &vars));

        egui::ScrollArea::vertical()
            .id_salt("sketch_inspector_scroll")
            .show(ui, |ui| {
                ui.add_space(12.0);
                Self::inspector_section(ui, &pal, "Profile", |ui| {
                    let closed = self.detected_regions.len();
                    ui.horizontal(|ui| {
                        ui.colored_label(pal.success, "●");
                        ui.label(format!(
                            "{closed} closed profile{}",
                            if closed == 1 { "" } else { "s" }
                        ));
                    });
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Area").color(pal.text_muted));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(format!("{area:.2} mm²"));
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Perimeter").color(pal.text_muted));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(format!("{perimeter:.2} mm"));
                        });
                    });
                });

                Self::inspector_section(ui, &pal, "Constraints", |ui| {
                    if let Some(report) = &solve {
                        let (label, color) = match report.outcome {
                            SolveOutcome::Conflicting => {
                                ("Over-constrained".to_string(), pal.danger)
                            }
                            SolveOutcome::DidNotConverge => {
                                ("Did not converge".to_string(), pal.danger)
                            }
                            SolveOutcome::Converged if report.dof == 0 => {
                                ("Fully defined".to_string(), pal.success)
                            }
                            SolveOutcome::Converged => {
                                (format!("{} DOF remaining", report.dof), pal.accent)
                            }
                        };
                        ui.colored_label(color, label);
                    }
                    if let Some(model) = &self.sketch_solver_model {
                        for constraint in model.constraints.iter().take(9) {
                            ui.label(
                                egui::RichText::new(constraint_label(constraint))
                                    .size(11.5)
                                    .color(pal.text_body),
                            );
                        }
                        if model.constraints.len() > 9 {
                            ui.label(
                                egui::RichText::new(format!(
                                    "+ {} more",
                                    model.constraints.len() - 9
                                ))
                                .size(10.5)
                                .color(pal.accent),
                            );
                        }
                    } else {
                        ui.label(
                            egui::RichText::new("No solver constraints yet.").color(pal.text_faint),
                        );
                    }
                });

                Self::inspector_section(ui, &pal, "Snapping", |ui| {
                    ui.checkbox(&mut self.snap_enabled, "Enable snapping");
                    ui.checkbox(&mut self.grid_visible, "Show grid");
                    ui.label(
                        egui::RichText::new(
                            "Hold Ctrl while drawing to temporarily bypass snapping.",
                        )
                        .size(10.5)
                        .color(pal.text_faint),
                    );
                });
            });
    }

    fn inspector_section(
        ui: &mut egui::Ui,
        pal: &Palette,
        title: &str,
        content: impl FnOnce(&mut egui::Ui),
    ) {
        ui.add_space(4.0);
        ui.label(
            egui::RichText::new(title)
                .strong()
                .size(12.0)
                .color(pal.text_strong),
        );
        ui.add_space(5.0);
        content(ui);
        ui.add_space(10.0);
        ui.separator();
    }
}

fn polygon_perimeter(points: &[(f32, f32)]) -> f32 {
    if points.len() < 2 {
        return 0.0;
    }
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt())
        .sum()
}

fn constraint_label(constraint: &Constraint) -> &'static str {
    match constraint {
        Constraint::Coincident { .. } => "Coincident",
        Constraint::Horizontal { .. } => "Horizontal",
        Constraint::Vertical { .. } => "Vertical",
        Constraint::Distance { .. } => "Distance",
        Constraint::Radius { .. } => "Radius",
        Constraint::Parallel { .. } => "Parallel",
        Constraint::Perpendicular { .. } => "Perpendicular",
        Constraint::Tangent { .. } => "Tangent",
        Constraint::Equal { .. } => "Equal",
        Constraint::Fixed { .. } => "Fixed",
        Constraint::DistanceX { .. } => "Horizontal distance",
        Constraint::DistanceY { .. } => "Vertical distance",
        Constraint::Angle { .. } => "Angle",
        Constraint::Concentric { .. } => "Concentric",
        Constraint::Midpoint { .. } => "Midpoint",
        Constraint::PointOnObject { .. } => "Point on object",
        Constraint::Collinear { .. } => "Collinear",
        Constraint::Symmetric { .. } => "Symmetric",
        Constraint::Diameter { .. } => "Diameter",
        Constraint::SplineTangent { .. } => "Spline tangent",
        Constraint::SplineCurvature { .. } => "Spline curvature",
        Constraint::LineDistance { .. } => "Line distance",
    }
}

#[cfg(test)]
mod tests {
    use super::polygon_perimeter;

    #[test]
    fn perimeter_closes_polygon() {
        assert_eq!(
            polygon_perimeter(&[(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]),
            12.0
        );
    }
}
