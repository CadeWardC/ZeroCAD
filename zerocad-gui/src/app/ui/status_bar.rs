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
                let selection_status = self.bottom_selection_status();
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
                            if let Some(status) = selection_status {
                                ui.separator();
                                ui.label(
                                    egui::RichText::new(status)
                                        .size(10.5)
                                        .color(self.pal().text_muted),
                                );
                            }
                        }
                    });
                });
            });
    }

    fn bottom_selection_status(&self) -> Option<String> {
        if self.is_sketch_mode {
            return self
                .active_tool
                .is_none()
                .then(|| self.sketch_selection_status())
                .flatten();
        }
        self.selected_sketch_profiles_status()
            .or_else(|| self.modeling_selection_status())
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
            let mut measurements = SketchProfileMeasurements::default();
            for region in regions {
                measurements.add_region(region);
            }
            let label = if measurements.count == 1 {
                "Face".to_string()
            } else {
                format!("{} faces", measurements.count)
            };
            return Some(measurements.format(self.current_unit, label, true));
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

    fn selected_sketch_profiles_status(&self) -> Option<String> {
        if self.selected_faces.is_empty() {
            return None;
        }

        let variables = self.document.variable_map();
        let mut measurements = SketchProfileMeasurements::default();
        let mut measured_sketch_id: Option<String> = None;
        let mut spans_multiple_sketches = false;

        for idx in self.document.graph.node_indices() {
            let node = &self.document.graph[idx];
            let selected_indices: Vec<_> = self
                .selected_faces
                .iter()
                .filter_map(|(sketch_id, region_index)| {
                    (sketch_id == &node.id).then_some(*region_index)
                })
                .collect();
            if selected_indices.is_empty() {
                continue;
            }

            let FeatureType::Sketch {
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver,
                ..
            } = &node.feature
            else {
                continue;
            };
            if measured_sketch_id
                .as_ref()
                .is_some_and(|sketch_id| sketch_id != &node.id)
            {
                spans_multiple_sketches = true;
            } else if measured_sketch_id.is_none() {
                measured_sketch_id = Some(node.id.clone());
            }

            let mut region_curves = zerocad_core::effective_curves_solved(
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver.as_ref(),
                &variables,
            );
            if let Some(boundary) = self.document.sketch_face_boundaries.get(node.id.as_str()) {
                region_curves.extend_curves(boundary);
            }
            let regions = detect_regions(&region_curves);
            for region_index in selected_indices {
                if let Some(region) = regions.get(region_index) {
                    measurements.add_region(region);
                }
            }
        }

        if measurements.count == 0 {
            return None;
        }
        let label = if measurements.count == 1 {
            "1 sketch profile selected".to_string()
        } else {
            format!("{} sketch profiles selected", measurements.count)
        };
        Some(measurements.format(self.current_unit, label, !spans_multiple_sketches))
    }

    fn modeling_selection_status(&self) -> Option<String> {
        let selections = if self.selected_edges.is_empty() && self.selected_sketch_points.is_empty()
        {
            self.selected_body_measurements()
        } else {
            self.selected_sketch_measurements()
        };
        match selections.as_slice() {
            [MeasuredSelection::Edge(edge)] => {
                if let Some(circle) = edge.circle.as_ref().filter(|_| edge.closed) {
                    Some(format!(
                        "Circle edge — Diameter: {}",
                        format_length(self.current_unit, circle.radius * 2.0)
                    ))
                } else {
                    Some(format!(
                        "Edge — Length: {}",
                        format_length(self.current_unit, edge.length_mm)
                    ))
                }
            }
            [MeasuredSelection::Point(point)] => Some(format!(
                "Point — X: {} · Y: {} · Z: {}",
                format_length(self.current_unit, point[0]),
                format_length(self.current_unit, point[1]),
                format_length(self.current_unit, point[2]),
            )),
            [MeasuredSelection::Edge(first), MeasuredSelection::Edge(second)] => {
                if let Some(angle) = shared_endpoint_angle(first, second) {
                    Some(format!("2 edges selected — Angle: {angle:.2}°"))
                } else {
                    Some(format!(
                        "2 edges selected — Shortest distance: {}",
                        format_length(self.current_unit, shortest_edge_distance(first, second))
                    ))
                }
            }
            [MeasuredSelection::Edge(edge), MeasuredSelection::Point(point)]
            | [MeasuredSelection::Point(point), MeasuredSelection::Edge(edge)] => Some(format!(
                "Edge and point selected — Shortest distance: {}",
                format_length(self.current_unit, edge_point_distance(edge, *point))
            )),
            [MeasuredSelection::Point(first), MeasuredSelection::Point(second)] => Some(format!(
                "2 points selected — Distance: {}",
                format_length(self.current_unit, distance3(*first, *second))
            )),
            selections if selections.len() > 2 => {
                let edge_count = selections
                    .iter()
                    .filter(|selection| matches!(selection, MeasuredSelection::Edge(_)))
                    .count();
                let point_count = selections.len() - edge_count;
                if point_count == 0 {
                    let total_length = selections
                        .iter()
                        .filter_map(|selection| match selection {
                            MeasuredSelection::Edge(edge) => Some(edge.length_mm),
                            MeasuredSelection::Point(_) => None,
                        })
                        .sum();
                    Some(format!(
                        "{edge_count} edges selected — Total length: {}",
                        format_length(self.current_unit, total_length)
                    ))
                } else {
                    Some(format!("{} measurable elements selected", selections.len()))
                }
            }
            _ => None,
        }
    }

    fn selected_sketch_measurements(&self) -> Vec<MeasuredSelection> {
        let mut selected: Vec<_> = self.selected_edges.iter().cloned().collect();
        selected.sort();
        let variables = self.document.variable_map();
        let mut measurements = Vec::with_capacity(selected.len());

        for (sketch_id, edge_index) in selected {
            let Some(node) = self
                .document
                .graph
                .node_weights()
                .find(|node| node.id == sketch_id)
            else {
                continue;
            };
            let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver,
                ..
            } = &node.feature
            else {
                continue;
            };
            let curves = zerocad_core::effective_curves_solved(
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver.as_ref(),
                &variables,
            );
            let segment_count = curves.segments.len();
            let circle_count = curves.circles.len();
            let edge = if let Some(segment) = curves.segments.get(edge_index) {
                let points = vec![
                    sketch_world_point(cs, segment.a),
                    sketch_world_point(cs, segment.b),
                ];
                Some(MeasuredEdge::from_open_points(points))
            } else if let Some(circle) = edge_index
                .checked_sub(segment_count)
                .and_then(|index| curves.circles.get(index))
            {
                let center = sketch_world_point(cs, circle.center);
                let normal = [cs.n.x as f64, cs.n.y as f64, cs.n.z as f64];
                let points = sample_sketch_circle(cs, circle.center, circle.radius, 192);
                Some(MeasuredEdge {
                    points,
                    length_mm: std::f64::consts::TAU * circle.radius.abs() as f64,
                    circle: Some(MeasuredCircle {
                        center,
                        normal,
                        radius: circle.radius.abs() as f64,
                    }),
                    closed: true,
                })
            } else {
                edge_index
                    .checked_sub(segment_count + circle_count)
                    .and_then(|index| curves.splines.get(index))
                    .map(|spline| {
                        let points = spline
                            .sampled_points(0.01)
                            .into_iter()
                            .map(|point| sketch_world_point(cs, point))
                            .collect();
                        MeasuredEdge::from_points(points)
                    })
            };
            if let Some(edge) = edge.filter(|edge| edge.points.len() >= 2) {
                measurements.push(MeasuredSelection::Edge(edge));
            }
        }

        let mut selected_points: Vec<_> = self.selected_sketch_points.iter().cloned().collect();
        selected_points.sort();
        for (sketch_id, point_index) in selected_points {
            let Some(node) = self
                .document
                .graph
                .node_weights()
                .find(|node| node.id == sketch_id)
            else {
                continue;
            };
            let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver,
                ..
            } = &node.feature
            else {
                continue;
            };
            let curves = zerocad_core::effective_curves_solved(
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver.as_ref(),
                &variables,
            );
            if let Some(point) = crate::geom2d::selectable_sketch_points(&curves).get(point_index) {
                measurements.push(MeasuredSelection::Point(sketch_world_point(cs, *point)));
            }
        }
        measurements
    }

    fn selected_body_measurements(&self) -> Vec<MeasuredSelection> {
        let mut selected: Vec<_> = self.selected_body.iter().cloned().collect();
        selected.sort_by(|left, right| {
            left.0
                .cmp(&right.0)
                .then_with(|| body_pick_sort_key(left.1).cmp(&body_pick_sort_key(right.1)))
        });
        let mut measurements = Vec::with_capacity(selected.len());
        for (body_id, pick) in selected {
            match pick {
                BodyPick::Edge(group) => {
                    if let Some(edge) = self.measured_body_edge(&body_id, group) {
                        measurements.push(MeasuredSelection::Edge(edge));
                    }
                }
                BodyPick::Vertex(index) => {
                    let Some((instance, mesh)) = self.evaluated_scene.find(&body_id) else {
                        continue;
                    };
                    let offset = index as usize * 3;
                    if let Some(position) = mesh.edge_vertices.get(offset..offset + 3) {
                        let point = instance.placement().transform_point([
                            position[0],
                            position[1],
                            position[2],
                        ]);
                        measurements.push(MeasuredSelection::Point(point.map(f64::from)));
                    }
                }
                BodyPick::Face(_) | BodyPick::Whole => {}
            }
        }
        measurements
    }

    fn measured_body_edge(&self, body_id: &str, group: u32) -> Option<MeasuredEdge> {
        let edge_ref = self.edge_ref_from(body_id, group)?;
        if edge_ref.curve.is_some() {
            return Some(MeasuredEdge::from_edge_ref(&edge_ref));
        }

        let (instance, mesh) = self.evaluated_scene.find(body_id)?;
        let points: Vec<_> = ordered_edge_group_points(mesh, group)
            .into_iter()
            .map(|point| instance.placement().transform_point(point).map(f64::from))
            .collect();
        if points.len() >= 2 {
            Some(MeasuredEdge::from_points(points))
        } else {
            Some(MeasuredEdge::from_edge_ref(&edge_ref))
        }
    }
}

enum MeasuredSelection {
    Edge(MeasuredEdge),
    Point([f64; 3]),
}

struct MeasuredEdge {
    points: Vec<[f64; 3]>,
    length_mm: f64,
    circle: Option<MeasuredCircle>,
    closed: bool,
}

struct MeasuredCircle {
    center: [f64; 3],
    normal: [f64; 3],
    radius: f64,
}

impl MeasuredEdge {
    fn from_open_points(points: Vec<[f64; 3]>) -> Self {
        let length_mm = polyline_length(&points);
        Self {
            points,
            length_mm,
            circle: None,
            closed: false,
        }
    }

    fn from_points(points: Vec<[f64; 3]>) -> Self {
        let closed = points
            .first()
            .zip(points.last())
            .is_some_and(|(first, last)| distance3(*first, *last) <= 1.0e-6);
        let length_mm = polyline_length(&points);
        Self {
            points,
            length_mm,
            circle: None,
            closed,
        }
    }

    fn from_edge_ref(edge: &EdgeRef) -> Self {
        match &edge.curve {
            Some(EdgeCurveHint::Circle {
                center,
                axis,
                x_dir,
                radius,
                start,
                end,
                closed,
            }) => {
                let center = center.map(f64::from);
                let normal = normalize3(axis.map(f64::from));
                let radius = radius.abs() as f64;
                let sweep = if *closed {
                    std::f64::consts::TAU
                } else {
                    f64::from(*end - *start)
                };
                Self {
                    points: sample_circle_3d(
                        center,
                        normal,
                        normalize3(x_dir.map(f64::from)),
                        radius,
                        f64::from(*start),
                        sweep,
                        192,
                    ),
                    length_mm: radius * sweep.abs(),
                    circle: Some(MeasuredCircle {
                        center,
                        normal,
                        radius,
                    }),
                    closed: *closed,
                }
            }
            None | Some(EdgeCurveHint::Line) => {
                Self::from_open_points(vec![edge.p0.map(f64::from), edge.p1.map(f64::from)])
            }
        }
    }

    fn endpoint_rays(&self) -> Vec<([f64; 3], [f64; 3])> {
        if self.closed || self.points.len() < 2 {
            return Vec::new();
        }
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        vec![
            (first, normalize3(sub3(self.points[1], first))),
            (
                last,
                normalize3(sub3(self.points[self.points.len() - 2], last)),
            ),
        ]
    }
}

fn body_pick_sort_key(pick: BodyPick) -> (u8, u32) {
    match pick {
        BodyPick::Edge(index) => (0, index),
        BodyPick::Vertex(index) => (1, index),
        BodyPick::Face(index) => (2, index),
        BodyPick::Whole => (3, 0),
    }
}

fn ordered_edge_group_points(mesh: &MockMesh, group: u32) -> Vec<[f32; 3]> {
    let segment_count = mesh.edge_indices.len() / 2;
    let segment_indices: Vec<_> = if mesh.edge_groups.is_empty() {
        ((group as usize) < segment_count)
            .then_some(group as usize)
            .into_iter()
            .collect()
    } else {
        mesh.edge_groups
            .iter()
            .take(segment_count)
            .enumerate()
            .filter_map(|(index, edge_group)| (*edge_group == group).then_some(index))
            .collect()
    };
    let vertex = |index: u32| -> Option<[f32; 3]> {
        let offset = index as usize * 3;
        let coordinates = mesh.edge_vertices.get(offset..offset + 3)?;
        Some([coordinates[0], coordinates[1], coordinates[2]])
    };
    let segments: Vec<_> = segment_indices
        .into_iter()
        .filter_map(|index| {
            let endpoints = mesh.edge_indices.get(index * 2..index * 2 + 2)?;
            Some((vertex(endpoints[0])?, vertex(endpoints[1])?))
        })
        .collect();
    let Some(first_segment) = segments.first() else {
        return Vec::new();
    };

    let tolerance = 1.0e-4_f64;
    let matches = |first: [f32; 3], second: [f32; 3]| {
        distance3(first.map(f64::from), second.map(f64::from)) <= tolerance
    };
    let endpoint_use_count = |point: [f32; 3]| {
        segments
            .iter()
            .flat_map(|(start, end)| [*start, *end])
            .filter(|candidate| matches(point, *candidate))
            .count()
    };
    let start = segments
        .iter()
        .flat_map(|(first, second)| [*first, *second])
        .find(|point| endpoint_use_count(*point) == 1)
        .unwrap_or(first_segment.0);

    let mut used = vec![false; segments.len()];
    let mut points = vec![start];
    while let Some(last) = points.last().copied() {
        let Some((index, next)) =
            segments
                .iter()
                .enumerate()
                .find_map(|(index, (first, second))| {
                    if used[index] {
                        None
                    } else if matches(last, *first) {
                        Some((index, *second))
                    } else if matches(last, *second) {
                        Some((index, *first))
                    } else {
                        None
                    }
                })
        else {
            break;
        };
        used[index] = true;
        points.push(next);
    }
    points
}

fn sketch_world_point(cs: &CoordinateSystem, point: (f32, f32)) -> [f64; 3] {
    let point = cs.unproject(point.0, point.1);
    [point.x as f64, point.y as f64, point.z as f64]
}

fn sample_sketch_circle(
    cs: &CoordinateSystem,
    center: (f32, f32),
    radius: f32,
    segments: usize,
) -> Vec<[f64; 3]> {
    (0..=segments)
        .map(|index| {
            let angle = std::f32::consts::TAU * index as f32 / segments as f32;
            sketch_world_point(
                cs,
                (
                    center.0 + radius * angle.cos(),
                    center.1 + radius * angle.sin(),
                ),
            )
        })
        .collect()
}

fn sample_circle_3d(
    center: [f64; 3],
    normal: [f64; 3],
    x_dir: [f64; 3],
    radius: f64,
    start: f64,
    sweep: f64,
    segments: usize,
) -> Vec<[f64; 3]> {
    let y_dir = normalize3(cross3(normal, x_dir));
    (0..=segments)
        .map(|index| {
            let angle = start + sweep * index as f64 / segments as f64;
            add3(
                center,
                scale3(
                    add3(scale3(x_dir, angle.cos()), scale3(y_dir, angle.sin())),
                    radius,
                ),
            )
        })
        .collect()
}

fn shared_endpoint_angle(first: &MeasuredEdge, second: &MeasuredEdge) -> Option<f64> {
    let tolerance = 1.0e-4 * first.length_mm.max(second.length_mm).max(1.0);
    for (first_point, first_direction) in first.endpoint_rays() {
        for (second_point, second_direction) in second.endpoint_rays() {
            if distance3(first_point, second_point) <= tolerance {
                let cosine = dot3(first_direction, second_direction).clamp(-1.0, 1.0);
                return Some(cosine.acos().to_degrees());
            }
        }
    }
    None
}

fn shortest_edge_distance(first: &MeasuredEdge, second: &MeasuredEdge) -> f64 {
    if first.closed && second.closed {
        let (Some(first_circle), Some(second_circle)) = (&first.circle, &second.circle) else {
            return polyline_distance(&first.points, &second.points);
        };
        if let Some(distance) = parallel_circle_distance(first_circle, second_circle) {
            return distance;
        }
    }
    polyline_distance(&first.points, &second.points)
}

fn parallel_circle_distance(first: &MeasuredCircle, second: &MeasuredCircle) -> Option<f64> {
    let first_normal = normalize3(first.normal);
    let second_normal = normalize3(second.normal);
    if dot3(first_normal, second_normal).abs() < 1.0 - 1.0e-6 {
        return None;
    }
    let center_delta = sub3(second.center, first.center);
    let axial = dot3(center_delta, first_normal).abs();
    let planar_delta = sub3(
        center_delta,
        scale3(first_normal, dot3(center_delta, first_normal)),
    );
    let center_distance = magnitude3(planar_delta);
    let radius_sum = first.radius + second.radius;
    let radius_difference = (first.radius - second.radius).abs();
    let planar_gap = if center_distance > radius_sum {
        center_distance - radius_sum
    } else if center_distance < radius_difference {
        radius_difference - center_distance
    } else {
        0.0
    };
    Some(planar_gap.hypot(axial))
}

fn edge_point_distance(edge: &MeasuredEdge, point: [f64; 3]) -> f64 {
    if edge.closed {
        if let Some(circle) = &edge.circle {
            let normal = normalize3(circle.normal);
            let center_delta = sub3(point, circle.center);
            let axial = dot3(center_delta, normal);
            let radial = magnitude3(sub3(center_delta, scale3(normal, axial)));
            return (radial - circle.radius).hypot(axial);
        }
    }
    edge.points
        .windows(2)
        .map(|segment| point_segment_distance(point, segment[0], segment[1]))
        .fold(f64::INFINITY, f64::min)
}

fn polyline_distance(first: &[[f64; 3]], second: &[[f64; 3]]) -> f64 {
    first
        .windows(2)
        .flat_map(|first_segment| {
            second.windows(2).map(move |second_segment| {
                segment_segment_distance(
                    first_segment[0],
                    first_segment[1],
                    second_segment[0],
                    second_segment[1],
                )
            })
        })
        .fold(f64::INFINITY, f64::min)
}

fn polyline_length(points: &[[f64; 3]]) -> f64 {
    points
        .windows(2)
        .map(|segment| distance3(segment[0], segment[1]))
        .sum()
}

fn point_segment_distance(point: [f64; 3], start: [f64; 3], end: [f64; 3]) -> f64 {
    let segment = sub3(end, start);
    let length_squared = dot3(segment, segment);
    if length_squared <= f64::EPSILON {
        return distance3(point, start);
    }
    let parameter = (dot3(sub3(point, start), segment) / length_squared).clamp(0.0, 1.0);
    distance3(point, add3(start, scale3(segment, parameter)))
}

fn segment_segment_distance(
    first_start: [f64; 3],
    first_end: [f64; 3],
    second_start: [f64; 3],
    second_end: [f64; 3],
) -> f64 {
    let first_direction = sub3(first_end, first_start);
    let second_direction = sub3(second_end, second_start);
    let offset = sub3(first_start, second_start);
    let first_length_squared = dot3(first_direction, first_direction);
    let second_length_squared = dot3(second_direction, second_direction);
    let second_projection = dot3(second_direction, offset);
    let epsilon = 1.0e-12;

    let (mut first_parameter, mut second_parameter);
    if first_length_squared <= epsilon && second_length_squared <= epsilon {
        return distance3(first_start, second_start);
    } else if first_length_squared <= epsilon {
        first_parameter = 0.0;
        second_parameter = (second_projection / second_length_squared).clamp(0.0, 1.0);
    } else {
        let first_projection = dot3(first_direction, offset);
        if second_length_squared <= epsilon {
            second_parameter = 0.0;
            first_parameter = (-first_projection / first_length_squared).clamp(0.0, 1.0);
        } else {
            let directions_dot = dot3(first_direction, second_direction);
            let denominator =
                first_length_squared * second_length_squared - directions_dot * directions_dot;
            first_parameter = if denominator.abs() > epsilon {
                ((directions_dot * second_projection - first_projection * second_length_squared)
                    / denominator)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            second_parameter =
                (directions_dot * first_parameter + second_projection) / second_length_squared;
            if second_parameter < 0.0 {
                second_parameter = 0.0;
                first_parameter = (-first_projection / first_length_squared).clamp(0.0, 1.0);
            } else if second_parameter > 1.0 {
                second_parameter = 1.0;
                first_parameter =
                    ((directions_dot - first_projection) / first_length_squared).clamp(0.0, 1.0);
            }
        }
    }

    let first_point = add3(first_start, scale3(first_direction, first_parameter));
    let second_point = add3(second_start, scale3(second_direction, second_parameter));
    distance3(first_point, second_point)
}

fn add3(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[0] + second[0],
        first[1] + second[1],
        first[2] + second[2],
    ]
}

fn sub3(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2],
    ]
}

fn scale3(vector: [f64; 3], factor: f64) -> [f64; 3] {
    [vector[0] * factor, vector[1] * factor, vector[2] * factor]
}

fn dot3(first: [f64; 3], second: [f64; 3]) -> f64 {
    first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
}

fn cross3(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

fn magnitude3(vector: [f64; 3]) -> f64 {
    dot3(vector, vector).sqrt()
}

fn normalize3(vector: [f64; 3]) -> [f64; 3] {
    let magnitude = magnitude3(vector);
    if magnitude <= f64::EPSILON {
        [0.0; 3]
    } else {
        scale3(vector, magnitude.recip())
    }
}

fn distance3(first: [f64; 3], second: [f64; 3]) -> f64 {
    magnitude3(sub3(first, second))
}

#[derive(Default)]
struct SketchProfileMeasurements {
    count: usize,
    area_mm2: f64,
    perimeter_mm: f64,
    bounds: Option<(f64, f64, f64, f64)>,
}

impl SketchProfileMeasurements {
    fn add_region(&mut self, region: &Region) {
        self.count += 1;
        self.area_mm2 += region.area as f64;
        self.perimeter_mm += closed_loop_length(&region.boundary);
        self.perimeter_mm += region
            .holes
            .iter()
            .map(|hole| closed_loop_length(hole))
            .sum::<f64>();
        for point in &region.boundary {
            self.bounds = Some(expand_sketch_bounds(
                self.bounds,
                (point.0 as f64, point.1 as f64),
            ));
        }
    }

    fn format(&self, unit: Unit, label: String, include_dimensions: bool) -> String {
        let mut status = format!("{label} — Area: {}", format_area(unit, self.area_mm2));
        if include_dimensions {
            if let Some((min_x, min_y, max_x, max_y)) = self.bounds {
                status.push_str(&format!(
                    " · Dimensions: {} × {}",
                    format_length(unit, max_x - min_x),
                    format_length(unit, max_y - min_y),
                ));
            }
        }
        status.push_str(&format!(
            " · Perimeter: {}",
            format_length(unit, self.perimeter_mm)
        ));
        status
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

fn closed_loop_length(points: &[(f32, f32)]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(start, end)| {
            let dx = (end.0 - start.0) as f64;
            let dy = (end.1 - start.1) as f64;
            dx.hypot(dy)
        })
        .sum()
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

    fn app_with_finished_sketch(curves: SketchCurves) -> ZeroCadApp {
        let mut app = ZeroCadApp::new();
        app.current_unit = Unit::Millimeter;
        app.document.add_feature(FeatureNode {
            id: "sketch_1".to_string(),
            name: "Sketch_1".to_string(),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XY,
                curves,
                shapes: Vec::new(),
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        app
    }

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
            Some("Face — Area: 12.00 mm² · Dimensions: 4.00 mm × 3.00 mm · Perimeter: 14.00 mm")
        );
    }

    #[test]
    fn finished_profile_status_reports_selection_and_measurements() {
        let mut curves = SketchCurves::new();
        curves.segments = vec![
            LineSegment {
                a: (1.0, 2.0),
                b: (5.0, 2.0),
            },
            LineSegment {
                a: (5.0, 2.0),
                b: (5.0, 5.0),
            },
            LineSegment {
                a: (5.0, 5.0),
                b: (1.0, 5.0),
            },
            LineSegment {
                a: (1.0, 5.0),
                b: (1.0, 2.0),
            },
        ];
        let mut app = app_with_finished_sketch(curves);
        app.selected_faces.insert(("sketch_1".to_string(), 0));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some(
                "1 sketch profile selected — Area: 12.00 mm² · Dimensions: 4.00 mm × 3.00 mm · Perimeter: 14.00 mm"
            )
        );
    }

    #[test]
    fn finished_line_edge_status_reports_length() {
        let mut curves = SketchCurves::new();
        curves.segments.push(LineSegment {
            a: (0.0, 0.0),
            b: (3.0, 4.0),
        });
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Edge — Length: 5.00 mm")
        );
    }

    #[test]
    fn finished_sketch_point_status_reports_world_coordinates() {
        let mut curves = SketchCurves::new();
        curves.add_line((2.0, 3.0), (7.0, 3.0));
        let mut app = app_with_finished_sketch(curves);
        app.selected_sketch_points
            .insert(("sketch_1".to_string(), 0));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Point — X: 2.00 mm · Y: 3.00 mm · Z: 0.00 mm")
        );
    }

    #[test]
    fn finished_sketch_edge_and_point_report_shortest_distance() {
        let mut curves = SketchCurves::new();
        curves.add_line((0.0, 0.0), (10.0, 0.0));
        curves.add_line((5.0, 3.0), (5.0, 4.0));
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));
        app.selected_sketch_points
            .insert(("sketch_1".to_string(), 2));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Edge and point selected — Shortest distance: 3.00 mm")
        );
    }

    #[test]
    fn finished_circle_edge_status_reports_diameter() {
        let mut curves = SketchCurves::new();
        curves.add_circle((2.0, 3.0), 5.0);
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Circle edge — Diameter: 10.00 mm")
        );
    }

    #[test]
    fn open_circular_body_edge_reports_arc_length() {
        let mut app = ZeroCadApp::new();
        app.current_unit = Unit::Millimeter;
        let mut mesh = MockMesh::empty();
        mesh.edge_vertices = vec![2.0, 0.0, 0.0, 0.0, 2.0, 0.0];
        mesh.edge_indices = vec![0, 1];
        mesh.edge_groups = vec![0];
        mesh.edge_refs.push(zerocad_core::mock_kernel::MeshEdgeRef {
            group: 0,
            p0: [2.0, 0.0, 0.0],
            p1: [0.0, 2.0, 0.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: Some(EdgeCurveHint::Circle {
                center: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 2.0,
                start: 0.0,
                end: std::f32::consts::FRAC_PI_2,
                closed: false,
            }),
            topology: None,
        });
        app.set_body_meshes(vec![("body_1".to_string(), mesh)]);
        app.selected_body
            .insert(("body_1".to_string(), BodyPick::Edge(0)));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Edge — Length: 3.14 mm")
        );
    }

    #[test]
    fn non_analytic_body_edge_uses_its_curve_length_instead_of_its_chord() {
        let mut app = ZeroCadApp::new();
        app.current_unit = Unit::Millimeter;
        let mut mesh = MockMesh::empty();
        mesh.edge_vertices = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        mesh.edge_indices = vec![0, 1, 1, 2];
        mesh.edge_groups = vec![7, 7];
        mesh.edge_refs.push(zerocad_core::mock_kernel::MeshEdgeRef {
            group: 7,
            p0: [0.0, 0.0, 0.0],
            p1: [1.0, 1.0, 0.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: None,
            topology: None,
        });
        app.set_body_meshes(vec![("body_1".to_string(), mesh)]);
        app.selected_body
            .insert(("body_1".to_string(), BodyPick::Edge(7)));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Edge — Length: 2.00 mm")
        );
    }

    #[test]
    fn shared_endpoint_edges_report_the_angle() {
        let mut curves = SketchCurves::new();
        curves.segments = vec![
            LineSegment {
                a: (0.0, 0.0),
                b: (3.0, 0.0),
            },
            LineSegment {
                a: (0.0, 0.0),
                b: (0.0, 4.0),
            },
        ];
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));
        app.selected_edges.insert(("sketch_1".to_string(), 1));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("2 edges selected — Angle: 90.00°")
        );
    }

    #[test]
    fn separate_parallel_edges_report_shortest_distance() {
        let mut curves = SketchCurves::new();
        curves.segments = vec![
            LineSegment {
                a: (0.0, 0.0),
                b: (5.0, 0.0),
            },
            LineSegment {
                a: (0.0, 4.0),
                b: (5.0, 4.0),
            },
        ];
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));
        app.selected_edges.insert(("sketch_1".to_string(), 1));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("2 edges selected — Shortest distance: 4.00 mm")
        );
    }

    #[test]
    fn two_circle_edges_report_exact_shortest_distance() {
        let mut curves = SketchCurves::new();
        curves.add_circle((0.0, 0.0), 5.0);
        curves.add_circle((30.0, 0.0), 7.0);
        let mut app = app_with_finished_sketch(curves);
        app.selected_edges.insert(("sketch_1".to_string(), 0));
        app.selected_edges.insert(("sketch_1".to_string(), 1));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("2 edges selected — Shortest distance: 18.00 mm")
        );
    }

    #[test]
    fn body_edge_and_point_report_shortest_distance() {
        let mut app = ZeroCadApp::new();
        app.current_unit = Unit::Millimeter;
        let mut mesh = MockMesh::empty();
        mesh.edge_vertices = vec![0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 5.0, 3.0, 0.0];
        mesh.edge_indices = vec![0, 1];
        mesh.edge_groups = vec![0];
        mesh.edge_refs.push(zerocad_core::mock_kernel::MeshEdgeRef {
            group: 0,
            p0: [0.0, 0.0, 0.0],
            p1: [10.0, 0.0, 0.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: Some(EdgeCurveHint::Line),
            topology: None,
        });
        app.set_body_meshes(vec![("body_1".to_string(), mesh)]);
        app.selected_body
            .insert(("body_1".to_string(), BodyPick::Edge(0)));
        app.selected_body
            .insert(("body_1".to_string(), BodyPick::Vertex(2)));

        assert_eq!(
            app.bottom_selection_status().as_deref(),
            Some("Edge and point selected — Shortest distance: 3.00 mm")
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
