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
    splines: Vec<EntityId>,
}

#[derive(Debug, Clone, PartialEq)]
struct SplineEdit {
    id: EntityId,
    kind: zerocad_core::SplineKind,
    point_count: usize,
    degree: u8,
    closed: bool,
    periodic: bool,
    continuity: zerocad_core::SplineContinuity,
    trim: Option<(f32, f32)>,
}

impl ZeroCadApp {
    pub(crate) fn show_constraints_panel(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.sketch_solver_model.is_none() {
            return;
        }
        let vars = self.document.variable_map();
        // Solve once per frame for the status line + conflict highlight (the
        // model is tiny; the solve is microseconds).
        let report = self
            .sketch_solver_model
            .as_ref()
            .map(|m| zerocad_core::sketch::solve_model(m, &vars));

        let sel = self.classify_selection();
        let mut spline_edit = (sel.splines.len() == 1)
            .then(|| {
                self.sketch_solver_model
                    .as_ref()?
                    .entities
                    .iter()
                    .find_map(|entity| match entity {
                        SketchEntity::Spline {
                            id,
                            points,
                            kind,
                            degree,
                            closed,
                            periodic,
                            continuity,
                            trim,
                            ..
                        } if *id == sel.splines[0] => Some(SplineEdit {
                            id: *id,
                            kind: *kind,
                            point_count: points.len(),
                            degree: *degree,
                            closed: *closed,
                            periodic: *periodic,
                            continuity: *continuity,
                            trim: *trim,
                        }),
                        _ => None,
                    })
            })
            .flatten();
        let original_spline_edit = spline_edit.clone();
        let selected_entities: Vec<EntityId> = sel
            .lines
            .iter()
            .chain(&sel.circles)
            .chain(&sel.splines)
            .copied()
            .collect();
        let mut construction_value = (!selected_entities.is_empty())
            .then(|| {
                self.sketch_solver_model.as_ref().is_some_and(|model| {
                    selected_entities
                        .iter()
                        .all(|id| model.construction.contains(id))
                })
            })
            .unwrap_or(false);
        let selected_has_projection = self.sketch_solver_model.as_ref().is_some_and(|model| {
            selected_entities
                .iter()
                .any(|id| model.is_projected_entity(*id))
        });
        let original_construction_value = construction_value;
        let mut add: Option<Constraint> = None;
        let mut delete: Option<EntityId> = None;
        let mut dimension_edit: Option<(EntityId, Dimension)> = None;
        let mut dimension_mode_edit: Option<(EntityId, bool, f32)> = None;
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
                            "Selection: {} point(s), {} line(s), {} circle(s), {} spline(s)",
                            sel.points.len(),
                            sel.lines.len(),
                            sel.circles.len(),
                            sel.splines.len()
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
                            && sel.splines.is_empty()
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
                    if btn(ui, "↦ ΔX", "Horizontal distance (2 points)", only(2, 0, 0)) {
                        add = Some(Constraint::DistanceX {
                            id: alloc(),
                            a: sel.points[0],
                            b: sel.points[1],
                            d: Dimension::literal(
                                self.current_point_delta(sel.points[0], sel.points[1]).0,
                            ),
                        });
                    }
                    if btn(ui, "↥ ΔY", "Vertical distance (2 points)", only(2, 0, 0)) {
                        add = Some(Constraint::DistanceY {
                            id: alloc(),
                            a: sel.points[0],
                            b: sel.points[1],
                            d: Dimension::literal(
                                self.current_point_delta(sel.points[0], sel.points[1]).1,
                            ),
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
                    if btn(ui, "∠ Angle", "Driving angle (2 lines)", only(0, 2, 0)) {
                        add = Some(Constraint::Angle {
                            id: alloc(),
                            a: sel.lines[0],
                            b: sel.lines[1],
                            angle_deg: Dimension::literal(
                                self.current_line_angle(sel.lines[0], sel.lines[1]),
                            ),
                        });
                    }
                    if btn(ui, "⫽ Collinear", "Collinear (2 lines)", only(0, 2, 0)) {
                        add = Some(Constraint::Collinear {
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
                    let spline_tangent_ok = sel.points.is_empty()
                        && sel.lines.len() == 1
                        && sel.circles.is_empty()
                        && sel.splines.len() == 1;
                    for (label, at_start) in [("⌁ Spline tangent S", true), ("⌁ Spline tangent E", false)] {
                        if btn(ui, label, "Align a line to a native spline endpoint derivative", spline_tangent_ok) {
                            add = Some(Constraint::SplineTangent {
                                id: alloc(),
                                spline: sel.splines[0],
                                line: sel.lines[0],
                                at_start,
                            });
                        }
                    }
                    let spline_only = sel.points.is_empty()
                        && sel.lines.is_empty()
                        && sel.circles.is_empty()
                        && sel.splines.len() == 1;
                    for (label, at_start) in [("◠ Curvature S", true), ("◠ Curvature E", false)] {
                        if btn(ui, label, "Approximate endpoint curvature from the first three spline handles (v1; not exact NURBS curvature)", spline_only) {
                            add = Some(Constraint::SplineCurvature {
                                id: alloc(),
                                spline: sel.splines[0],
                                at_start,
                                radius: Dimension::literal(self.current_spline_radius(sel.splines[0], at_start)),
                            });
                        }
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
                    if btn(ui, "⌀ Diameter", "Driving diameter (1 circle)", only(0, 0, 1)) {
                        add = Some(Constraint::Diameter {
                            id: alloc(),
                            circle: sel.circles[0],
                            d: Dimension::literal(self.current_circle_radius(sel.circles[0]) * 2.0),
                        });
                    }
                    if btn(ui, "◉ Concentric", "Concentric (2 circles)", only(0, 0, 2)) {
                        add = Some(Constraint::Concentric {
                            id: alloc(),
                            a: sel.circles[0],
                            b: sel.circles[1],
                        });
                    }
                    if btn(ui, "M Midpoint", "Point at line midpoint", only(1, 1, 0)) {
                        add = Some(Constraint::Midpoint {
                            id: alloc(),
                            point: sel.points[0],
                            line: sel.lines[0],
                        });
                    }
                    if btn(
                        ui,
                        "⊙ Point on",
                        "Point on line or circle",
                        only(1, 1, 0) || only(1, 0, 1),
                    ) {
                        add = Some(Constraint::PointOnObject {
                            id: alloc(),
                            point: sel.points[0],
                            object: sel
                                .lines
                                .first()
                                .or_else(|| sel.circles.first())
                                .copied()
                                .unwrap(),
                        });
                    }
                    if btn(
                        ui,
                        "S Symmetric",
                        "Two points symmetric about one line",
                        only(2, 1, 0),
                    ) {
                        add = Some(Constraint::Symmetric {
                            id: alloc(),
                            a: sel.points[0],
                            b: sel.points[1],
                            axis: sel.lines[0],
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

                if !selected_entities.is_empty() {
                    ui.add_enabled(
                        !selected_has_projection,
                        egui::Checkbox::new(
                            &mut construction_value,
                            "Construction / reference geometry",
                        ),
                    )
                        .on_hover_text(
                            if selected_has_projection {
                                "Projected edges are associative reference geometry and cannot become profile edges."
                            } else {
                                "Construction entities remain editable and constrained but do not create profile regions."
                            },
                        );
                }

                if let Some(spline) = &mut spline_edit {
                    ui.separator();
                    ui.label(egui::RichText::new("Spline").strong());
                    ui.label(match spline.kind {
                        zerocad_core::SplineKind::ControlPoint => "Control-point B-spline",
                        zerocad_core::SplineKind::FitPoint => "Fit-point spline",
                    });
                    if spline.kind == zerocad_core::SplineKind::ControlPoint {
                        ui.horizontal(|ui| {
                            ui.label("Degree");
                            ui.add(egui::DragValue::new(&mut spline.degree).range(
                                1..=spline.point_count.saturating_sub(1).max(1) as u8,
                            ));
                        });
                    } else {
                        ui.horizontal(|ui| {
                            ui.label("Continuity");
                            for (continuity, label) in [
                                (zerocad_core::SplineContinuity::Position, "C0"),
                                (zerocad_core::SplineContinuity::Tangent, "C1"),
                                (zerocad_core::SplineContinuity::Curvature, "C2"),
                            ] {
                                ui.selectable_value(&mut spline.continuity, continuity, label);
                            }
                        });
                    }
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut spline.closed, "Closed");
                        ui.add_enabled_ui(spline.closed, |ui| {
                            ui.checkbox(&mut spline.periodic, "Periodic");
                        });
                    });
                    let mut trimmed = spline.trim.is_some();
                    if ui.checkbox(&mut trimmed, "Trim parameter range").changed() {
                        spline.trim = trimmed.then_some((0.1, 0.9));
                    }
                    if let Some((start, end)) = &mut spline.trim {
                        ui.horizontal(|ui| {
                            ui.label("Start");
                            ui.add(egui::DragValue::new(start).range(0.0..=0.999).speed(0.01));
                            ui.label("End");
                            ui.add(egui::DragValue::new(end).range(0.001..=1.0).speed(0.01));
                        });
                        if *end <= *start {
                            *end = (*start + 0.001).min(1.0);
                        }
                    }
                }

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
                            let is_driven = self
                                .sketch_solver_model
                                .as_ref()
                                .is_some_and(|model| model.is_driven_dimension(id));
                            ui.horizontal(|ui| {
                                if ui
                                    .small_button("✕")
                                    .on_hover_text("Delete constraint")
                                    .clicked()
                                {
                                    delete = Some(id);
                                }
                                let label = if is_driven {
                                    format!("[ref] {}", constraint_label(c))
                                } else {
                                    constraint_label(c)
                                };
                                let mut text = egui::RichText::new(label).size(11.0);
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

                if let Some(selected) = self.sketch_selected_constraint {
                    if let Some((constraint, (label, mut dimension))) = constraints
                        .iter()
                        .find(|constraint| constraint.id() == selected)
                        .and_then(|constraint| {
                            constraint_dimension(constraint).map(|dimension| (constraint, dimension))
                        })
                    {
                        ui.separator();
                        let mut is_driven = self
                            .sketch_solver_model
                            .as_ref()
                            .is_some_and(|model| model.is_driven_dimension(selected));
                        let was_driven = is_driven;
                        ui.label(
                            egui::RichText::new(if is_driven {
                                "Reference dimension"
                            } else {
                                "Driving dimension"
                            })
                            .strong(),
                        );
                        if ui
                            .checkbox(&mut is_driven, "Reference (driven)")
                            .on_hover_text(
                                "Reference dimensions report the current geometry and do not constrain it.",
                            )
                            .changed()
                        {
                            dimension_mode_edit = Some((
                                selected,
                                is_driven,
                                self.measured_constraint_value(constraint),
                            ));
                        }
                        let original = dimension.clone();
                        if was_driven {
                            dimension.value = self.measured_constraint_value(constraint);
                            dimension.expr = None;
                        }
                        ui.add_enabled_ui(!was_driven, |ui| {
                            ui.horizontal(|ui| {
                                ui.label(label);
                                ui.add(
                                    egui::DragValue::new(&mut dimension.value)
                                        .speed(0.1)
                                        .max_decimals(6),
                                );
                                let mut expression = dimension.expr.clone().unwrap_or_default();
                                if ui
                                    .add(
                                        egui::TextEdit::singleline(&mut expression)
                                            .desired_width(150.0)
                                            .hint_text("parameter expression"),
                                    )
                                    .changed()
                                {
                                    dimension.expr = (!expression.trim().is_empty())
                                        .then(|| expression.trim().to_string());
                                }
                            });
                        });
                        if !was_driven && dimension != original {
                            dimension_edit = Some((selected, dimension));
                        }
                    }
                }
            });

        if spline_edit != original_spline_edit {
            let mut changed = false;
            if let (Some(edit), Some(model)) = (spline_edit, &mut self.sketch_solver_model) {
                if let Some(SketchEntity::Spline {
                    degree,
                    closed,
                    periodic,
                    continuity,
                    trim,
                    ..
                }) = model
                    .entities
                    .iter_mut()
                    .find(|entity| entity.id() == edit.id)
                {
                    *degree = edit
                        .degree
                        .min(edit.point_count.saturating_sub(1).max(1) as u8);
                    *closed = edit.closed;
                    *periodic = edit.periodic && edit.closed;
                    *continuity = edit.continuity;
                    *trim = edit.trim;
                    changed = true;
                }
            }
            if changed {
                self.rebuild_active_sketch_curves();
                self.status_msg = "Spline settings updated.".to_string();
            }
        }

        if construction_value != original_construction_value {
            if let Some(model) = &mut self.sketch_solver_model {
                model
                    .construction
                    .retain(|id| !selected_entities.contains(id));
                if construction_value {
                    model.construction.extend(selected_entities.iter().copied());
                    model.construction.sort();
                    model.construction.dedup();
                }
            }
            self.rebuild_active_sketch_curves();
            self.status_msg = if construction_value {
                "Selection converted to construction geometry.".to_string()
            } else {
                "Selection converted to profile geometry.".to_string()
            };
        }

        self.sketch_next_entity_id = next_id;
        if let Some((id, dimension)) = dimension_edit {
            if let Some(constraint) = self
                .sketch_solver_model
                .as_mut()
                .and_then(|model| model.constraints.iter_mut().find(|item| item.id() == id))
            {
                set_constraint_dimension(constraint, dimension);
                self.solve_live_sketch();
                self.status_msg = "Driving dimension updated.".to_string();
            }
        }
        if let Some((id, driven, measured)) = dimension_mode_edit {
            if let Some(model) = &mut self.sketch_solver_model {
                model.driven_dimensions.retain(|candidate| *candidate != id);
                if driven {
                    model.driven_dimensions.push(id);
                    model.driven_dimensions.sort();
                    model.driven_dimensions.dedup();
                } else if let Some(constraint) = model
                    .constraints
                    .iter_mut()
                    .find(|constraint| constraint.id() == id)
                {
                    set_constraint_dimension(constraint, Dimension::literal(measured));
                }
            }
            self.solve_live_sketch();
            self.status_msg = if driven {
                "Dimension converted to reference measurement.".to_string()
            } else {
                "Dimension converted to driving constraint.".to_string()
            };
        }
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
                model.driven_dimensions.retain(|candidate| *candidate != id);
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
            splines: Vec::new(),
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
                    // An associative projected ellipse exposes its center point,
                    // but radius/concentric constraints do not apply to its two
                    // independent analytic axes.
                    SketchEntity::Ellipse { .. } => {}
                    SketchEntity::Spline { .. } => sel.splines.push(id),
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

    fn current_point_delta(&self, a: EntityId, b: EntityId) -> (f32, f32) {
        let Some(model) = &self.sketch_solver_model else {
            return (0.0, 0.0);
        };
        match (model.point(a), model.point(b)) {
            (Some(pa), Some(pb)) => ((pb.pos.0 - pa.pos.0) as f32, (pb.pos.1 - pa.pos.1) as f32),
            _ => (0.0, 0.0),
        }
    }

    fn current_line_angle(&self, a: EntityId, b: EntityId) -> f32 {
        let Some(model) = &self.sketch_solver_model else {
            return 0.0;
        };
        let direction = |id| {
            let entity = model.entities.iter().find(|entity| entity.id() == id)?;
            let SketchEntity::Line { p0, p1, .. } = entity else {
                return None;
            };
            let p0 = model.point(*p0)?.pos;
            let p1 = model.point(*p1)?.pos;
            Some((p1.0 - p0.0, p1.1 - p0.1))
        };
        let Some(da) = direction(a) else {
            return 0.0;
        };
        let Some(db) = direction(b) else {
            return 0.0;
        };
        let cross = da.0 * db.1 - da.1 * db.0;
        let dot = da.0 * db.0 + da.1 * db.1;
        cross.atan2(dot).abs().to_degrees() as f32
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

    fn current_spline_radius(&self, id: EntityId, at_start: bool) -> f32 {
        let Some(model) = &self.sketch_solver_model else {
            return 10.0;
        };
        let Some(points) = model.entities.iter().find_map(|entity| match entity {
            SketchEntity::Spline {
                id: entity_id,
                points,
                ..
            } if *entity_id == id && points.len() >= 3 => Some(points),
            _ => None,
        }) else {
            return 10.0;
        };
        let ids = if at_start {
            [points[0], points[1], points[2]]
        } else {
            let last = points.len() - 1;
            [points[last], points[last - 1], points[last - 2]]
        };
        let (Some(first), Some(second), Some(third)) = (
            model.point(ids[0]).map(|point| point.pos),
            model.point(ids[1]).map(|point| point.pos),
            model.point(ids[2]).map(|point| point.pos),
        ) else {
            return 10.0;
        };
        let tangent = [second.0 - first.0, second.1 - first.1];
        let second_derivative = [
            third.0 - 2.0 * second.0 + first.0,
            third.1 - 2.0 * second.1 + first.1,
        ];
        let length_squared = tangent[0] * tangent[0] + tangent[1] * tangent[1];
        let cross = (tangent[0] * second_derivative[1] - tangent[1] * second_derivative[0]).abs();
        if cross <= 1.0e-12 || length_squared <= 1.0e-12 {
            1.0e6
        } else {
            (length_squared.powf(1.5) / cross) as f32
        }
    }

    fn measured_constraint_value(&self, constraint: &Constraint) -> f32 {
        match constraint {
            Constraint::Distance { a, b, .. } => self.current_point_distance(*a, *b),
            Constraint::DistanceX { a, b, .. } => self.current_point_delta(*a, *b).0,
            Constraint::DistanceY { a, b, .. } => self.current_point_delta(*a, *b).1,
            Constraint::Radius { circle, .. } => self.current_circle_radius(*circle),
            Constraint::Diameter { circle, .. } => self.current_circle_radius(*circle) * 2.0,
            Constraint::Angle { a, b, .. } => self.current_line_angle(*a, *b),
            Constraint::SplineCurvature {
                spline, at_start, ..
            } => self.current_spline_radius(*spline, *at_start),
            _ => 0.0,
        }
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
                SketchEntity::Spline {
                    id: entity_id,
                    points,
                    ..
                } if *entity_id == id => points.first().and_then(|point| pos(*point)),
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
                Constraint::DistanceX { a, b, .. } | Constraint::DistanceY { a, b, .. } => (
                    "↦",
                    match (pos(*a), pos(*b)) {
                        (Some(pa), Some(pb)) => Some(((pa.0 + pb.0) * 0.5, (pa.1 + pb.1) * 0.5)),
                        _ => None,
                    },
                ),
                Constraint::Angle { a, .. } => ("∠", entity_anchor(*a)),
                Constraint::Concentric { a, .. } => ("◉", entity_anchor(*a)),
                Constraint::Midpoint { point, .. } => ("M", pos(*point)),
                Constraint::PointOnObject { point, .. } => ("⊙", pos(*point)),
                Constraint::Collinear { a, .. } => ("⫽", entity_anchor(*a)),
                Constraint::Symmetric { a, .. } => ("S", pos(*a)),
                Constraint::Diameter { circle, .. } => ("⌀", entity_anchor(*circle)),
                Constraint::SplineTangent { spline, .. } => ("⌁", entity_anchor(*spline)),
                Constraint::SplineCurvature { spline, .. } => ("◠", entity_anchor(*spline)),
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
fn constraint_dimension(constraint: &Constraint) -> Option<(&'static str, Dimension)> {
    match constraint {
        Constraint::Distance { d, .. } => Some(("Distance", d.clone())),
        Constraint::DistanceX { d, .. } => Some(("Horizontal distance", d.clone())),
        Constraint::DistanceY { d, .. } => Some(("Vertical distance", d.clone())),
        Constraint::Radius { r, .. } => Some(("Radius", r.clone())),
        Constraint::Diameter { d, .. } => Some(("Diameter", d.clone())),
        Constraint::Angle { angle_deg, .. } => Some(("Angle (degrees)", angle_deg.clone())),
        Constraint::SplineCurvature { radius, .. } => {
            Some(("Approx. spline curvature radius", radius.clone()))
        }
        _ => None,
    }
}

fn set_constraint_dimension(constraint: &mut Constraint, dimension: Dimension) {
    match constraint {
        Constraint::Distance { d, .. }
        | Constraint::DistanceX { d, .. }
        | Constraint::DistanceY { d, .. }
        | Constraint::Diameter { d, .. } => *d = dimension,
        Constraint::Radius { r, .. } => *r = dimension,
        Constraint::Angle { angle_deg, .. } => *angle_deg = dimension,
        Constraint::SplineCurvature { radius, .. } => *radius = dimension,
        _ => {}
    }
}

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
        Constraint::DistanceX { id, a, b, d } => {
            format!(
                "c{} ↦ Horizontal distance p{} p{} = {}",
                id.0,
                a.0,
                b.0,
                d.expr
                    .as_deref()
                    .map_or_else(|| format!("{:.3}", d.value), str::to_string)
            )
        }
        Constraint::DistanceY { id, a, b, d } => {
            format!(
                "c{} ↥ Vertical distance p{} p{} = {}",
                id.0,
                a.0,
                b.0,
                d.expr
                    .as_deref()
                    .map_or_else(|| format!("{:.3}", d.value), str::to_string)
            )
        }
        Constraint::Angle {
            id,
            a,
            b,
            angle_deg,
        } => format!(
            "c{} ∠ Angle e{} e{} = {}°",
            id.0,
            a.0,
            b.0,
            angle_deg
                .expr
                .as_deref()
                .map_or_else(|| format!("{:.3}", angle_deg.value), str::to_string)
        ),
        Constraint::Concentric { id, a, b } => {
            format!("c{} ◉ Concentric e{} e{}", id.0, a.0, b.0)
        }
        Constraint::Midpoint { id, point, line } => {
            format!("c{} M Midpoint p{} e{}", id.0, point.0, line.0)
        }
        Constraint::PointOnObject { id, point, object } => {
            format!("c{} ⊙ Point-on-object p{} e{}", id.0, point.0, object.0)
        }
        Constraint::Collinear { id, a, b } => {
            format!("c{} ⫽ Collinear e{} e{}", id.0, a.0, b.0)
        }
        Constraint::Symmetric { id, a, b, axis } => {
            format!("c{} S Symmetric p{} p{} about e{}", id.0, a.0, b.0, axis.0)
        }
        Constraint::Diameter { id, circle, d } => format!(
            "c{} ⌀ Diameter e{} = {}",
            id.0,
            circle.0,
            d.expr
                .as_deref()
                .map_or_else(|| format!("{:.3}", d.value), str::to_string)
        ),
        Constraint::SplineTangent {
            id,
            spline,
            line,
            at_start,
        } => format!(
            "c{} ⌁ Spline tangent {} e{} to e{}",
            id.0,
            if *at_start { "start" } else { "end" },
            spline.0,
            line.0
        ),
        Constraint::SplineCurvature {
            id,
            spline,
            at_start,
            radius,
        } => format!(
            "c{} ◠ Spline curvature {} e{} = {}",
            id.0,
            if *at_start { "start" } else { "end" },
            spline.0,
            radius
                .expr
                .as_deref()
                .map_or_else(|| format!("{:.3}", radius.value), str::to_string)
        ),
    }
}
