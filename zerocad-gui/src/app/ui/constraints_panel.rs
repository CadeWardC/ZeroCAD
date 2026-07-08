//! The Edit Sketch constraints panel: a floating window with the constraint
//! palette (buttons enabled by the current point/entity selection), the live
//! constraint list with per-row delete, and the DOF / conflict status line.
//!
//! Shown only while an Edit Sketch session is active (a solver model exists).
//! Every mutation re-solves the live model immediately (`solve_live_sketch`),
//! so the sketch follows each constraint the moment it is added or removed;
//! nothing touches the document until Finish Sketch commits in place.

use crate::*;
use zerocad_core::sketch::{Constraint, EntityId, SketchEntity, SolveOutcome};

/// What the current selection can be constrained as.
struct SelectionShape {
    points: Vec<EntityId>,
    lines: Vec<EntityId>,
    circles: Vec<EntityId>,
}

impl ZeroCadApp {
    pub(crate) fn show_constraints_panel(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.sketch_solver_model.is_none() {
            return;
        }
        let vars = self.graph.variable_map();
        // Solve once per frame for the status line + conflict highlight (the
        // model is tiny; the solve is microseconds).
        let report = self
            .sketch_solver_model
            .as_ref()
            .map(|m| zerocad_core::sketch::solve_model(m, &vars));

        let sel = self.classify_selection();
        let mut add: Option<Constraint> = None;
        let mut delete: Option<EntityId> = None;
        // Local id counter (written back below) so the palette closures don't
        // hold a mutable borrow of `self` across the `&self` helpers.
        let mut next_id = self.sketch_next_entity_id;

        egui::Window::new("Constraints")
            .id(egui::Id::new("sketch-constraints-panel"))
            .default_pos(egui::pos2(16.0, 90.0))
            .resizable(false)
            .show(ctx, |ui| {
                // --- Status line -------------------------------------------
                if let Some(report) = &report {
                    let (text, color) = match report.outcome {
                        SolveOutcome::Conflicting => (
                            match report.conflicting {
                                Some(id) => format!("⚠ Over-constrained (conflict: c{})", id.0),
                                None => "⚠ Over-constrained".to_string(),
                            },
                            egui::Color32::from_rgb(220, 38, 38),
                        ),
                        SolveOutcome::DidNotConverge => (
                            "⚠ Did not converge".to_string(),
                            egui::Color32::from_rgb(220, 38, 38),
                        ),
                        SolveOutcome::Converged if report.dof == 0 => (
                            "● Fully constrained".to_string(),
                            egui::Color32::from_rgb(22, 163, 74),
                        ),
                        SolveOutcome::Converged => (
                            format!("◐ {} DOF remaining", report.dof),
                            egui::Color32::from_rgb(37, 99, 235),
                        ),
                    };
                    ui.label(egui::RichText::new(text).size(12.0).color(color));
                    ui.separator();
                }

                // --- Palette (enabled by selection shape) ------------------
                ui.label(
                    egui::RichText::new(if self.sketch_selected_ids.is_empty() {
                        "Select points/lines/circles in the sketch, then apply:".to_string()
                    } else {
                        format!(
                            "Selection: {} point(s), {} line(s), {} circle(s)",
                            sel.points.len(),
                            sel.lines.len(),
                            sel.circles.len()
                        )
                    })
                    .size(11.0)
                    .weak(),
                );
                ui.add_space(4.0);
                let mut alloc = || {
                    let id = EntityId(next_id);
                    next_id += 1;
                    id
                };

                ui.horizontal_wrapped(|ui| {
                    let only = |n_p: usize, n_l: usize, n_c: usize| {
                        sel.points.len() == n_p
                            && sel.lines.len() == n_l
                            && sel.circles.len() == n_c
                    };
                    let btn = |ui: &mut egui::Ui, label: &str, hover: &str, on: bool| {
                        ui.add_enabled(on, egui::Button::new(label))
                            .on_hover_text(hover)
                            .clicked()
                    };
                    if btn(ui, "— H", "Horizontal (1 line)", only(0, 1, 0)) {
                        add = Some(Constraint::Horizontal {
                            id: alloc(),
                            line: sel.lines[0],
                        });
                    }
                    if btn(ui, "| V", "Vertical (1 line)", only(0, 1, 0)) {
                        add = Some(Constraint::Vertical {
                            id: alloc(),
                            line: sel.lines[0],
                        });
                    }
                    if btn(
                        ui,
                        "◎ Coincident",
                        "Merge two points (2 points)",
                        only(2, 0, 0),
                    ) {
                        add = Some(Constraint::Coincident {
                            id: alloc(),
                            a: sel.points[0],
                            b: sel.points[1],
                        });
                    }
                    if btn(
                        ui,
                        "↔ Distance",
                        "Driving distance (2 points)",
                        only(2, 0, 0),
                    ) {
                        // Seed the dimension with the CURRENT distance so adding
                        // it never moves geometry; edit the value afterwards.
                        let d = self.current_point_distance(sel.points[0], sel.points[1]);
                        add = Some(Constraint::Distance {
                            id: alloc(),
                            a: sel.points[0],
                            b: sel.points[1],
                            d: Dimension::literal(d),
                        });
                    }
                    if btn(ui, "∥ Parallel", "Parallel (2 lines)", only(0, 2, 0)) {
                        add = Some(Constraint::Parallel {
                            id: alloc(),
                            a: sel.lines[0],
                            b: sel.lines[1],
                        });
                    }
                    if btn(
                        ui,
                        "⟂ Perpendicular",
                        "Perpendicular (2 lines)",
                        only(0, 2, 0),
                    ) {
                        add = Some(Constraint::Perpendicular {
                            id: alloc(),
                            a: sel.lines[0],
                            b: sel.lines[1],
                        });
                    }
                    if btn(
                        ui,
                        "⌒ Tangent",
                        "Tangent (1 line + 1 circle)",
                        only(0, 1, 1),
                    ) {
                        add = Some(Constraint::Tangent {
                            id: alloc(),
                            line: sel.lines[0],
                            circle: sel.circles[0],
                        });
                    }
                    let equal_ok = only(0, 2, 0) || only(0, 0, 2);
                    if btn(
                        ui,
                        "= Equal",
                        "Equal length/radius (2 lines or 2 circles)",
                        equal_ok,
                    ) {
                        let (a, b) = if sel.lines.len() == 2 {
                            (sel.lines[0], sel.lines[1])
                        } else {
                            (sel.circles[0], sel.circles[1])
                        };
                        add = Some(Constraint::Equal { id: alloc(), a, b });
                    }
                    if btn(ui, "⌀ Radius", "Driving radius (1 circle)", only(0, 0, 1)) {
                        let r = self.current_circle_radius(sel.circles[0]);
                        add = Some(Constraint::Radius {
                            id: alloc(),
                            circle: sel.circles[0],
                            r: Dimension::literal(r),
                        });
                    }
                    if btn(
                        ui,
                        "⚓ Fix",
                        "Anchor a point in place (1 point)",
                        only(1, 0, 0),
                    ) {
                        add = Some(Constraint::Fixed {
                            id: alloc(),
                            p: sel.points[0],
                        });
                    }
                });

                // --- Constraint list with delete ----------------------------
                ui.separator();
                let conflict = report.as_ref().and_then(|r| r.conflicting);
                let constraints: Vec<Constraint> = self
                    .sketch_solver_model
                    .as_ref()
                    .map(|m| m.constraints.clone())
                    .unwrap_or_default();
                if constraints.is_empty() {
                    ui.label(egui::RichText::new("No constraints yet.").size(11.0).weak());
                }
                egui::ScrollArea::vertical()
                    .max_height(180.0)
                    .show(ui, |ui| {
                        for c in &constraints {
                            let id = c.id();
                            let is_conflict = conflict == Some(id);
                            ui.horizontal(|ui| {
                                if ui
                                    .small_button("✕")
                                    .on_hover_text("Delete constraint")
                                    .clicked()
                                {
                                    delete = Some(id);
                                }
                                let mut text = egui::RichText::new(constraint_label(c)).size(11.0);
                                if is_conflict {
                                    text =
                                        text.color(egui::Color32::from_rgb(220, 38, 38)).strong();
                                } else if self.sketch_selected_constraint == Some(id) {
                                    text = text.strong();
                                }
                                if ui
                                    .add(egui::Label::new(text).sense(egui::Sense::click()))
                                    .clicked()
                                {
                                    self.sketch_selected_constraint = Some(id);
                                }
                            });
                        }
                    });
            });

        self.sketch_next_entity_id = next_id;
        if let Some(constraint) = add {
            if let Some(model) = &mut self.sketch_solver_model {
                model.constraints.push(constraint);
            }
            self.solve_live_sketch();
            self.status_msg = "Constraint added.".to_string();
        }
        if let Some(id) = delete {
            if let Some(model) = &mut self.sketch_solver_model {
                model.constraints.retain(|c| c.id() != id);
            }
            if self.sketch_selected_constraint == Some(id) {
                self.sketch_selected_constraint = None;
            }
            self.solve_live_sketch();
            self.status_msg = "Constraint deleted.".to_string();
        }
    }

    /// Split the current selection into points / lines / circles-arcs.
    fn classify_selection(&self) -> SelectionShape {
        let mut sel = SelectionShape {
            points: Vec::new(),
            lines: Vec::new(),
            circles: Vec::new(),
        };
        let Some(model) = &self.sketch_solver_model else {
            return sel;
        };
        for &id in &self.sketch_selected_ids {
            if model.points.iter().any(|p| p.id == id) {
                sel.points.push(id);
            } else if let Some(e) = model.entities.iter().find(|e| e.id() == id) {
                match e {
                    SketchEntity::Line { .. } => sel.lines.push(id),
                    SketchEntity::Circle { .. } | SketchEntity::Arc { .. } => sel.circles.push(id),
                }
            }
        }
        sel
    }

    fn current_point_distance(&self, a: EntityId, b: EntityId) -> f32 {
        let Some(model) = &self.sketch_solver_model else {
            return 0.0;
        };
        match (model.point(a), model.point(b)) {
            (Some(pa), Some(pb)) => {
                (((pb.pos.0 - pa.pos.0).powi(2) + (pb.pos.1 - pa.pos.1).powi(2)).sqrt()) as f32
            }
            _ => 0.0,
        }
    }

    fn current_circle_radius(&self, id: EntityId) -> f32 {
        self.sketch_solver_model
            .as_ref()
            .and_then(|m| {
                m.entities.iter().find_map(|e| match e {
                    SketchEntity::Circle {
                        id: eid, radius, ..
                    }
                    | SketchEntity::Arc {
                        id: eid, radius, ..
                    } if *eid == id => Some(*radius as f32),
                    _ => None,
                })
            })
            .unwrap_or(0.0)
    }
}

impl ZeroCadApp {
    /// Paint each constraint's glyph at its anchor (red for the reported
    /// conflict), plus highlight rings on the selected points/entities. Pure
    /// painter overlay — clicking/deleting happens in the constraints panel.
    pub(crate) fn draw_constraint_badges(
        &self,
        painter: &egui::Painter,
        model: &zerocad_core::sketch::SketchSolverModel,
        conflict: Option<EntityId>,
        to_screen: &dyn Fn((f64, f64)) -> egui::Pos2,
    ) {
        use std::collections::HashMap;
        let pos = |id: EntityId| model.point(id).map(|p| p.pos);
        let entity_anchor = |id: EntityId| -> Option<(f64, f64)> {
            model.entities.iter().find_map(|e| match e {
                SketchEntity::Line {
                    id: eid, p0, p1, ..
                } if *eid == id => {
                    let a = pos(*p0)?;
                    let b = pos(*p1)?;
                    Some(((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5))
                }
                SketchEntity::Circle {
                    id: eid,
                    center,
                    radius,
                    ..
                }
                | SketchEntity::Arc {
                    id: eid,
                    center,
                    radius,
                    ..
                } if *eid == id => {
                    let c = pos(*center)?;
                    Some((c.0 + radius * 0.7, c.1 + radius * 0.7))
                }
                _ => None,
            })
        };

        // Selection rings first, under the badges.
        for &id in &self.sketch_selected_ids {
            let anchor = pos(id).or_else(|| entity_anchor(id));
            if let Some(a) = anchor {
                painter.circle_stroke(
                    to_screen(a),
                    7.0,
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(37, 99, 235)),
                );
            }
        }

        // One glyph per constraint; stacked badges at a shared anchor step
        // sideways so overlapping constraints stay individually readable.
        let mut stacked: HashMap<(i64, i64), usize> = HashMap::new();
        for c in &model.constraints {
            let (glyph, anchor) = match c {
                Constraint::Coincident { a, .. } => ("◎", pos(*a)),
                Constraint::Horizontal { line, .. } => ("H", entity_anchor(*line)),
                Constraint::Vertical { line, .. } => ("V", entity_anchor(*line)),
                Constraint::Distance { a, b, .. } => (
                    "↔",
                    match (pos(*a), pos(*b)) {
                        (Some(pa), Some(pb)) => Some(((pa.0 + pb.0) * 0.5, (pa.1 + pb.1) * 0.5)),
                        _ => None,
                    },
                ),
                Constraint::Radius { circle, .. } => ("⌀", entity_anchor(*circle)),
                Constraint::Parallel { a, .. } => ("∥", entity_anchor(*a)),
                Constraint::Perpendicular { a, .. } => ("⟂", entity_anchor(*a)),
                Constraint::Tangent { line, .. } => ("⌒", entity_anchor(*line)),
                Constraint::Equal { a, .. } => ("=", entity_anchor(*a)),
                Constraint::Fixed { p, .. } => ("⚓", pos(*p)),
            };
            let Some(anchor) = anchor else { continue };
            let base = to_screen(anchor);
            let key = ((base.x / 4.0) as i64, (base.y / 4.0) as i64);
            let n = stacked.entry(key).or_insert(0);
            let at = egui::pos2(base.x + 10.0 + (*n as f32) * 13.0, base.y - 10.0);
            *n += 1;

            let is_conflict = conflict == Some(c.id());
            let is_selected = self.sketch_selected_constraint == Some(c.id());
            let (fill, fg) = if is_conflict {
                (egui::Color32::from_rgb(220, 38, 38), egui::Color32::WHITE)
            } else if is_selected {
                (egui::Color32::from_rgb(37, 99, 235), egui::Color32::WHITE)
            } else {
                (
                    egui::Color32::from_rgba_unmultiplied(255, 255, 255, 220),
                    egui::Color32::from_rgb(51, 65, 85),
                )
            };
            painter.circle_filled(at, 7.0, fill);
            painter.circle_stroke(
                at,
                7.0,
                egui::Stroke::new(1.0, egui::Color32::from_rgb(148, 163, 184)),
            );
            painter.text(
                at,
                egui::Align2::CENTER_CENTER,
                glyph,
                egui::FontId::proportional(10.0),
                fg,
            );
        }
    }
}

/// Short human label for one constraint (the list row text).
pub(crate) fn constraint_label(c: &Constraint) -> String {
    match c {
        Constraint::Coincident { id, a, b } => format!("c{} ◎ Coincident p{} p{}", id.0, a.0, b.0),
        Constraint::Horizontal { id, line } => format!("c{} — Horizontal e{}", id.0, line.0),
        Constraint::Vertical { id, line } => format!("c{} | Vertical e{}", id.0, line.0),
        Constraint::Distance { id, a, b, d } => match &d.expr {
            Some(e) => format!("c{} ↔ Distance p{} p{} = {}", id.0, a.0, b.0, e),
            None => format!("c{} ↔ Distance p{} p{} = {:.3}", id.0, a.0, b.0, d.value),
        },
        Constraint::Radius { id, circle, r } => match &r.expr {
            Some(e) => format!("c{} ⌀ Radius e{} = {}", id.0, circle.0, e),
            None => format!("c{} ⌀ Radius e{} = {:.3}", id.0, circle.0, r.value),
        },
        Constraint::Parallel { id, a, b } => format!("c{} ∥ Parallel e{} e{}", id.0, a.0, b.0),
        Constraint::Perpendicular { id, a, b } => {
            format!("c{} ⟂ Perpendicular e{} e{}", id.0, a.0, b.0)
        }
        Constraint::Tangent { id, line, circle } => {
            format!("c{} ⌒ Tangent e{} e{}", id.0, line.0, circle.0)
        }
        Constraint::Equal { id, a, b } => format!("c{} = Equal e{} e{}", id.0, a.0, b.0),
        Constraint::Fixed { id, p } => format!("c{} ⚓ Fixed p{}", id.0, p.0),
    }
}
