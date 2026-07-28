use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_status_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("status_bar")
            .exact_height(34.0)
            .frame(
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .inner_margin(egui::Margin::symmetric(22.0, 5.0)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing = egui::vec2(8.0, 0.0);
                    ui.label(
                        egui::RichText::new("Ready")
                            .size(11.0)
                            .color(self.pal().text_body),
                    );
                    ui.separator();

                    let delayed_busy = self.eval_started.is_some_and(|started| {
                        started.elapsed() >= std::time::Duration::from_millis(150)
                    }) || self.pending_save.as_ref().is_some_and(|save| {
                        save.started.elapsed() >= std::time::Duration::from_millis(150)
                    });
                    if delayed_busy {
                        ui.spinner();
                    } else {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(15.0, 15.0), egui::Sense::hover());
                        icons::Icon::Help.draw(ui.painter(), rect, self.pal().text_muted);
                    }

                    let message = self.error_msg.as_deref().unwrap_or(&self.status_msg);
                    ui.label(egui::RichText::new(message).size(11.0).color(
                        if self.error_msg.is_some() {
                            self.pal().danger
                        } else {
                            self.pal().text_body
                        },
                    ));

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if self.project_kind == ProjectKind::Assembly {
                            ui.label(
                                egui::RichText::new(match self.current_unit {
                                    Unit::Millimeter => "Millimeters (mm)",
                                    Unit::Inch => "Inches (in)",
                                    Unit::Meter => "Meters (m)",
                                })
                                .size(10.5)
                                .color(self.pal().text_muted),
                            );
                            ui.separator();
                            ui.label(
                                egui::RichText::new("Assembly")
                                    .size(10.5)
                                    .color(self.pal().text_muted),
                            );
                        } else {
                            ui.checkbox(&mut self.snap_enabled, "Snap")
                                .on_hover_text("Enable sketch snapping");
                            ui.separator();
                            ui.scope(|ui| {
                                // Preserve comfortable text/arrow padding in the
                                // compact status bar and carry it into the popup.
                                ui.spacing_mut().button_padding = egui::vec2(10.0, 3.0);
                                egui::ComboBox::from_id_salt("status_unit_select")
                                    .width(138.0)
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
                            });
                            if self.is_sketch_mode && self.active_tool.is_none() {
                                if let Some(status) = self.sketch_selection_status() {
                                    ui.separator();
                                    ui.label(
                                        egui::RichText::new(status)
                                            .size(10.5)
                                            .color(self.pal().text_muted),
                                    );
                                }
                            }
                        }
                    });
                });
            });
    }

    fn sketch_selection_status(&self) -> Option<String> {
        if !self.selected_region_indices.is_empty() {
            let regions: Vec<_> = self
                .selected_region_indices
                .iter()
                .filter_map(|index| self.detected_regions.get(*index))
                .collect();
            if regions.is_empty() {
                return None;
            }
            let area_mm2: f64 = regions.iter().map(|region| region.area as f64).sum();
            let bounds = regions
                .iter()
                .flat_map(|region| region.boundary.iter().copied())
                .fold(None, |bounds, point| {
                    Some(expand_sketch_bounds(
                        bounds,
                        (point.0 as f64, point.1 as f64),
                    ))
                });
            let area = format_area(self.current_unit, area_mm2);
            if let Some((min_x, min_y, max_x, max_y)) = bounds {
                let width = format_length(self.current_unit, max_x - min_x);
                let height = format_length(self.current_unit, max_y - min_y);
                let label = if regions.len() == 1 {
                    "Face".to_string()
                } else {
                    format!("{} faces", regions.len())
                };
                return Some(format!(
                    "{label} — Area: {area} · Dimensions: {width} × {height}"
                ));
            }
            return Some(format!("Face — Area: {area}"));
        }

        let model = self.sketch_solver_model.as_ref()?;
        if self.sketch_selected_ids.len() == 1 {
            let id = self.sketch_selected_ids[0];
            if let Some(point) = model.point(id) {
                return Some(format!(
                    "Point — X: {} · Y: {}",
                    format_length(self.current_unit, point.pos.0),
                    format_length(self.current_unit, point.pos.1),
                ));
            }
            let entity = model.entities.iter().find(|entity| entity.id() == id)?;
            return sketch_entity_status(self.current_unit, model, entity);
        }

        if self.sketch_selected_ids.len() > 1 {
            let mut total_length = 0.0;
            let mut entity_count = 0usize;
            let mut point_count = 0usize;
            for id in &self.sketch_selected_ids {
                if model.point(*id).is_some() {
                    point_count += 1;
                } else if let Some(entity) = model.entities.iter().find(|entity| entity.id() == *id)
                {
                    entity_count += 1;
                    total_length += sketch_entity_length(model, entity).unwrap_or(0.0);
                }
            }
            if entity_count > 0 && point_count == 0 {
                return Some(format!(
                    "{entity_count} edges — Total length: {}",
                    format_length(self.current_unit, total_length),
                ));
            }
            return Some(format!(
                "{} sketch elements selected",
                entity_count + point_count
            ));
        }

        None
    }
}

fn format_length(unit: Unit, value_mm: f64) -> String {
    let value = unit.from_base(value_mm);
    let precision = match unit {
        Unit::Millimeter => 2,
        Unit::Inch => 3,
        Unit::Meter => 4,
    };
    format!("{value:.precision$} {}", unit.suffix())
}

fn format_area(unit: Unit, value_mm2: f64) -> String {
    let multiplier = unit.to_base_multiplier();
    let value = value_mm2 / (multiplier * multiplier);
    let precision = match unit {
        Unit::Millimeter => 2,
        Unit::Inch => 3,
        Unit::Meter => 6,
    };
    format!("{value:.precision$} {}²", unit.suffix())
}

fn expand_sketch_bounds(
    bounds: Option<(f64, f64, f64, f64)>,
    point: (f64, f64),
) -> (f64, f64, f64, f64) {
    match bounds {
        Some((min_x, min_y, max_x, max_y)) => (
            min_x.min(point.0),
            min_y.min(point.1),
            max_x.max(point.0),
            max_y.max(point.1),
        ),
        None => (point.0, point.1, point.0, point.1),
    }
}

fn sketch_entity_status(
    unit: Unit,
    model: &zerocad_core::sketch::SketchSolverModel,
    entity: &zerocad_core::sketch::SketchEntity,
) -> Option<String> {
    use zerocad_core::sketch::SketchEntity;

    let length = format_length(unit, sketch_entity_length(model, entity)?);
    match entity {
        SketchEntity::Circle { radius, .. } => Some(format!(
            "Circle — Diameter: {} · Length: {length}",
            format_length(unit, radius * 2.0),
        )),
        SketchEntity::Arc { radius, .. } => Some(format!(
            "Arc — Radius: {} · Length: {length}",
            format_length(unit, *radius),
        )),
        SketchEntity::Ellipse {
            major_axis,
            minor_axis,
            ..
        } => {
            let major = major_axis[0].hypot(major_axis[1]) * 2.0;
            let minor = minor_axis[0].hypot(minor_axis[1]) * 2.0;
            Some(format!(
                "Ellipse — Dimensions: {} × {} · Length: {length}",
                format_length(unit, major),
                format_length(unit, minor),
            ))
        }
        SketchEntity::Line { .. } => Some(format!("Edge — Length: {length}")),
        SketchEntity::Spline { .. } => Some(format!("Spline — Length: {length}")),
    }
}

fn sketch_entity_length(
    model: &zerocad_core::sketch::SketchSolverModel,
    entity: &zerocad_core::sketch::SketchEntity,
) -> Option<f64> {
    use zerocad_core::sketch::SketchEntity;

    let point = |id| model.point(id).map(|point| point.pos);
    match entity {
        SketchEntity::Line { p0, p1, .. } => {
            let (a, b) = (point(*p0)?, point(*p1)?);
            Some((b.0 - a.0).hypot(b.1 - a.1))
        }
        SketchEntity::Circle { radius, .. } => Some(std::f64::consts::TAU * radius.abs()),
        SketchEntity::Arc {
            center,
            start,
            end,
            radius,
            clockwise,
            ..
        } => {
            let center = point(*center)?;
            let start = point(*start)?;
            let end = point(*end)?;
            let start_angle = (start.1 - center.1).atan2(start.0 - center.0);
            let end_angle = (end.1 - center.1).atan2(end.0 - center.0);
            let sweep = if *clockwise {
                (start_angle - end_angle).rem_euclid(std::f64::consts::TAU)
            } else {
                (end_angle - start_angle).rem_euclid(std::f64::consts::TAU)
            };
            Some(radius.abs() * sweep)
        }
        SketchEntity::Ellipse {
            major_axis,
            minor_axis,
            start_parameter,
            end_parameter,
            closed,
            ..
        } => {
            let sweep = if *closed {
                std::f64::consts::TAU
            } else {
                end_parameter - start_parameter
            };
            let sample = |parameter: f64| {
                let (sin, cos) = parameter.sin_cos();
                (
                    major_axis[0] * cos + minor_axis[0] * sin,
                    major_axis[1] * cos + minor_axis[1] * sin,
                )
            };
            let mut length = 0.0;
            let mut previous = sample(*start_parameter);
            for index in 1..=128 {
                let current = sample(*start_parameter + sweep * index as f64 / 128.0);
                length += (current.0 - previous.0).hypot(current.1 - previous.1);
                previous = current;
            }
            Some(length)
        }
        SketchEntity::Spline {
            points,
            kind,
            degree,
            knots,
            weights,
            closed,
            periodic,
            continuity,
            trim,
            ..
        } => {
            let points: Option<Vec<(f32, f32)>> = points
                .iter()
                .map(|id| point(*id).map(|point| (point.0 as f32, point.1 as f32)))
                .collect();
            let spline = zerocad_core::Spline {
                kind: *kind,
                points: points?,
                degree: *degree,
                knots: knots.clone(),
                weights: weights.clone(),
                closed: *closed,
                periodic: *periodic,
                continuity: *continuity,
                trim: *trim,
            };
            Some(
                spline
                    .sampled_points(0.01)
                    .windows(2)
                    .map(|pair| {
                        let dx = (pair[1].0 - pair[0].0) as f64;
                        let dy = (pair[1].1 - pair[0].1) as f64;
                        dx.hypot(dy)
                    })
                    .sum(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocad_core::sketch::{EntityId, SketchEntity, SketchPoint, SketchSolverModel};

    #[test]
    fn face_status_reports_area_and_bounding_dimensions() {
        let mut app = ZeroCadApp::new();
        app.current_unit = Unit::Millimeter;
        app.detected_regions = vec![zerocad_core::Region {
            boundary: vec![(1.0, 2.0), (5.0, 2.0), (5.0, 5.0), (1.0, 5.0)],
            holes: Vec::new(),
            area: 12.0,
            analytic: None,
        }];
        app.selected_region_indices.insert(0);

        assert_eq!(
            app.sketch_selection_status().as_deref(),
            Some("Face — Area: 12.00 mm² · Dimensions: 4.00 mm × 3.00 mm")
        );
    }

    #[test]
    fn line_measurement_uses_live_solver_points() {
        let line = SketchEntity::Line {
            id: EntityId(2),
            p0: EntityId(0),
            p1: EntityId(1),
            derived_from: None,
        };
        let model = SketchSolverModel {
            points: vec![
                SketchPoint {
                    id: EntityId(0),
                    pos: (0.0, 0.0),
                },
                SketchPoint {
                    id: EntityId(1),
                    pos: (3.0, 4.0),
                },
            ],
            entities: vec![line.clone()],
            ..Default::default()
        };

        assert_eq!(sketch_entity_length(&model, &line), Some(5.0));
        assert_eq!(
            sketch_entity_status(Unit::Millimeter, &model, &line).as_deref(),
            Some("Edge — Length: 5.00 mm")
        );
    }
}
