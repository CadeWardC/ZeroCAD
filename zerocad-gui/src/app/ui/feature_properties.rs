use crate::*;
use zerocad_core::{DatumAxisDef, DatumPlaneDef, DatumPointDef, LoftSurfaceMode};

/// Edit the persisted source of an expression-backed numeric property and show
/// its current evaluated value without replacing the source text.
fn expression_editor(
    ui: &mut egui::Ui,
    expression: &mut Option<String>,
    vars: &std::collections::HashMap<String, f64>,
    hint: &str,
    suffix: &str,
    modified: &mut bool,
) {
    ui.add_space(3.0);
    ui.horizontal(|ui| {
        ui.label("=");
        let mut source = expression.clone().unwrap_or_default();
        if ui
            .add(
                egui::TextEdit::singleline(&mut source)
                    .hint_text(hint)
                    .desired_width(150.0),
            )
            .changed()
        {
            let source = source.trim();
            *expression = (!source.is_empty()).then(|| source.to_string());
            *modified = true;
        }
    });
    if let Some(source) = expression.as_ref() {
        let result = match zerocad_core::expr::eval(source, vars) {
            Ok(value) => format!("→ {value:.2} {suffix}"),
            Err(error) => format!("→ {error}"),
        };
        ui.label(egui::RichText::new(result).size(11.0).weak());
    }
}

fn variable_value_source(variable: &Variable) -> String {
    variable
        .expression
        .clone()
        .unwrap_or_else(|| variable.value.to_string())
}

fn apply_variable_value_source(variable: &mut Variable, source: &str) {
    let source = source.trim();
    if source.is_empty() {
        variable.expression = None;
    } else if let Ok(value) = source.parse::<f64>() {
        if value.is_finite() {
            variable.value = value;
            variable.expression = None;
        } else {
            variable.expression = Some(source.to_string());
        }
    } else {
        variable.expression = Some(source.to_string());
    }
}

#[derive(Debug, Clone)]
struct DrivingParameterRow {
    feature_id: String,
    name: String,
    value: f64,
    unit: Unit,
    expression: Option<String>,
}

#[derive(Debug, Clone)]
struct DrivingParameterEdit {
    feature_id: String,
    name: String,
    value: f64,
    unit: Unit,
}

fn driving_parameter_rows(
    document: &zerocad_core::ParametricGraph,
    feature_id: &str,
    resolved: &std::collections::HashMap<String, f64>,
) -> Vec<DrivingParameterRow> {
    let references = document.feature_variable_references(feature_id);
    let mut definitions = std::collections::HashMap::new();
    for idx in document.graph.node_indices() {
        let node = &document.graph[idx];
        if let FeatureType::VariableSet { variables } = &node.feature {
            for variable in variables {
                let name = variable.name.trim();
                if references
                    .binary_search_by(|reference| reference.as_str().cmp(name))
                    .is_ok()
                {
                    let value_in_base = resolved
                        .get(name)
                        .copied()
                        .unwrap_or_else(|| variable.value_in_base());
                    definitions.insert(
                        name.to_string(),
                        DrivingParameterRow {
                            feature_id: node.id.clone(),
                            name: name.to_string(),
                            value: variable.unit.from_base(value_in_base),
                            unit: variable.unit,
                            expression: variable.expression.clone(),
                        },
                    );
                }
            }
        }
    }
    references
        .into_iter()
        .filter_map(|name| definitions.remove(&name))
        .collect()
}

fn apply_driving_parameter_edit(
    document: &mut zerocad_core::ParametricGraph,
    edit: &DrivingParameterEdit,
) -> Result<(), String> {
    if !edit.value.is_finite() {
        return Err(format!(
            "parameter '{}' requires a finite measurement",
            edit.name
        ));
    }
    let idx = document
        .graph
        .node_indices()
        .find(|idx| document.graph[*idx].id == edit.feature_id)
        .ok_or_else(|| format!("parameter container '{}' no longer exists", edit.feature_id))?;
    let FeatureType::VariableSet { variables } = &mut document.graph[idx].feature else {
        return Err(format!(
            "feature '{}' is not a parameter container",
            edit.feature_id
        ));
    };
    let variable = variables
        .iter_mut()
        .find(|variable| variable.name.trim() == edit.name)
        .ok_or_else(|| format!("parameter '{}' no longer exists", edit.name))?;
    variable.value = edit.value;
    // An explicit measurement is authoritative. This mirrors dragging an
    // expression-backed feature property, which converts it back to a literal.
    variable.expression = None;
    document.commit_feature_edit(&edit.feature_id)
}

fn driving_parameters_editor(
    ui: &mut egui::Ui,
    selected_feature_id: &str,
    rows: &[DrivingParameterRow],
    edits: &mut Vec<DrivingParameterEdit>,
    edit_started: &mut bool,
) {
    if rows.is_empty() {
        return;
    }
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(8.0);
    ui.label(
        egui::RichText::new("Driving parameters")
            .strong()
            .size(12.0),
    );
    ui.label(
        egui::RichText::new("Enter a measurement to rebuild the part immediately.")
            .small()
            .weak(),
    );
    ui.add_space(5.0);
    for row in rows {
        let editor_id = egui::Id::new((
            "inline_driving_parameter",
            selected_feature_id,
            row.feature_id.as_str(),
            row.name.as_str(),
        ));
        let was_focused = ui.memory(|memory| memory.has_focus(editor_id));
        let mut source = if was_focused {
            ui.ctx()
                .data(|data| data.get_temp::<String>(editor_id))
                .unwrap_or_else(|| row.value.to_string())
        } else {
            row.value.to_string()
        };
        let mut parse_error = false;
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(&row.name).monospace().size(11.5));
            let mut response = ui.add(
                egui::TextEdit::singleline(&mut source)
                    .id(editor_id)
                    .desired_width(86.0)
                    .hint_text("measurement"),
            );
            response = if let Some(expression) = &row.expression {
                response.on_hover_text(format!(
                    "Currently computed by '{expression}'. Entering a measurement replaces that \
                     formula with a literal value."
                ))
            } else {
                response.on_hover_text(
                    "Changing this value rebuilds every dependent feature immediately.",
                )
            };
            if response.gained_focus() {
                *edit_started = true;
            }
            if response.changed() {
                match source.trim().parse::<f64>() {
                    Ok(value) if value.is_finite() => edits.push(DrivingParameterEdit {
                        feature_id: row.feature_id.clone(),
                        name: row.name.clone(),
                        value,
                        unit: row.unit,
                    }),
                    _ => parse_error = !source.trim().is_empty(),
                }
            }
            if response.has_focus() {
                ui.ctx()
                    .data_mut(|data| data.insert_temp(editor_id, source.clone()));
            } else {
                ui.ctx().data_mut(|data| data.remove::<String>(editor_id));
            }
            ui.label(egui::RichText::new(row.unit.suffix()).small().weak());
        });
        if parse_error {
            ui.colored_label(
                egui::Color32::from_rgb(185, 28, 28),
                egui::RichText::new("Enter a number.").small(),
            );
        } else if let Some(expression) = &row.expression {
            ui.label(
                egui::RichText::new(format!("Currently: = {expression}"))
                    .small()
                    .weak(),
            );
        }
        ui.add_space(3.0);
    }
}

impl ZeroCadApp {
    pub(crate) fn open_feature_properties(&mut self, feature_id: &str) {
        if self
            .document
            .graph
            .node_weights()
            .any(|node| node.id == feature_id)
        {
            self.selected_node_id = Some(feature_id.to_owned());
            self.feature_properties_dialog = Some(feature_id.to_owned());
        }
    }

    pub(crate) fn show_feature_properties_window(&mut self, ctx: &egui::Context) {
        let Some(feature_id) = self.feature_properties_dialog.clone() else {
            return;
        };
        let Some(feature_name) = self
            .document
            .graph
            .node_weights()
            .find(|node| node.id == feature_id)
            .map(|node| node.name.clone())
        else {
            self.feature_properties_dialog = None;
            return;
        };

        let mut open = true;
        egui::Window::new(format!("Properties — {feature_name}"))
            .id(egui::Id::new("feature_properties_dialog"))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_width(360.0)
            .default_height(540.0)
            .min_width(300.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("feature_properties_dialog_scroll")
                    .auto_shrink([false, false])
                    .max_height(560.0)
                    .show(ui, |ui| {
                        self.draw_selected_feature_properties(ui, Some(&feature_id))
                    });
            });

        if !open || self.is_sketch_mode || self.extrude_op.is_some() {
            self.feature_properties_dialog = None;
        }
    }

    fn draw_selected_feature_properties(&mut self, ui: &mut egui::Ui, selected_id: Option<&str>) {
        ui.spacing_mut().slider_width = 96.0;
        ui.add_space(4.0);

        // Render dynamic controls for the feature requested by the Properties
        // window. The target stays stable even if the tree selection changes.
        if let Some(selected_id) = selected_id {
            let mut node_idx = None;
            for idx in self.document.graph.node_indices() {
                if self.document.graph[idx].id == selected_id {
                    node_idx = Some(idx);
                    break;
                }
            }

            if let Some(idx) = node_idx {
                // Deferred action: extruding needs `&mut self`, but
                // `node` holds a mutable borrow of the graph below.
                let mut extrude_request: Option<String> = None;
                let mut edit_sketch_request: Option<String> = None;
                let selected_feature_id = self.document.graph[idx].id.clone();
                let feature_suppressed = self.document.is_feature_suppressed(&selected_feature_id);
                let mut suppression_request: Option<bool> = None;
                let mut modified = false;

                // Capture palette + unit + the variable map before
                // borrowing the graph mutably (so the Extrude panel can
                // show what an expression-driven depth resolves to).
                let pal = self.pal();
                let current_unit = self.current_unit;
                let variable_resolution = self.document.resolve_variables();
                let var_map = variable_resolution.values.clone();
                let driving_parameters = if matches!(
                    self.document.graph[idx].feature,
                    FeatureType::VariableSet { .. }
                ) {
                    Vec::new()
                } else {
                    driving_parameter_rows(&self.document, &selected_feature_id, &var_map)
                };
                let mut driving_parameter_edits = Vec::new();
                let mut driving_parameter_edit_started = false;
                let mirror_join_outcome = match &self.document.graph[idx].feature {
                    FeatureType::Pattern {
                        source,
                        kind: zerocad_core::PatternKind::Mirror { join: true, .. },
                    } => Some(self.mirror_join_outcome(&selected_feature_id, source)),
                    _ => None,
                };
                let current_body_center = self
                    .body_meshes
                    .iter()
                    .find(|(body_id, _)| body_id == &selected_feature_id)
                    .and_then(|(_, mesh)| zerocad_core::parametric::body_bounds_center(mesh));
                // Sketch-on-face reference outline, captured before the
                // mutable node borrow below (used by the Sketch panel).
                let face_boundary = self
                    .document
                    .sketch_face_boundaries
                    .get(self.document.graph[idx].id.as_str())
                    .cloned();
                let node = &mut self.document.graph[idx];

                // Render inside a semantic inspector card so the same hierarchy
                // remains legible in both light and dark themes.
                egui::Frame::none()
                    .fill(pal.surface)
                    .rounding(8.0)
                    .stroke(egui::Stroke::new(1.0, pal.border))
                    .inner_margin(12.0)
                    .show(ui, |ui| {
                        ui.vertical(|ui| {
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("ID:").weak().size(11.5));
                                ui.label(
                                    egui::RichText::new(&node.id)
                                        .strong()
                                        .size(12.0)
                                        .color(pal.text_body),
                                );
                            });
                            ui.add_space(6.0);
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("Label:").size(12.0));
                                ui.text_edit_singleline(&mut node.name);
                            });
                            if node.id != "origin" {
                                ui.add_space(4.0);
                                let mut participates_in_rebuild = !feature_suppressed;
                                if ui
                                    .checkbox(
                                        &mut participates_in_rebuild,
                                        "Participates in rebuild",
                                    )
                                    .on_hover_text(
                                        "Suppressed features stay in the timeline but do not evaluate",
                                    )
                                    .changed()
                                {
                                    suppression_request = Some(!participates_in_rebuild);
                                }
                            }
                            ui.add_space(8.0);
                            ui.separator();
                            ui.add_space(8.0);

                            match &mut node.feature {
                                FeatureType::Origin => {
                                    ui.label(
                                        egui::RichText::new(
                                            "📍 Base Origin coordinate planes space (0,0,0).",
                                        )
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::Box { w, h, d } => {
                                    ui.label(
                                        egui::RichText::new("Dimensions:")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(6.0);
                                    egui::Grid::new("box_grid")
                                        .spacing(egui::vec2(10.0, 10.0))
                                        .show(ui, |ui| {
                                            ui.label(egui::RichText::new("Width").size(12.0));
                                            let w_resp = ui.add(
                                                egui::Slider::new(w, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if w_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Height").size(12.0));
                                            let h_resp = ui.add(
                                                egui::Slider::new(h, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if h_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Depth").size(12.0));
                                            let d_resp = ui.add(
                                                egui::Slider::new(d, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if d_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();
                                        });
                                }
                                FeatureType::Import { step_data, label } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "📦 Imported STEP body \"{}\" ({:.1} KB embedded).",
                                            label,
                                            step_data.len() as f64 / 1024.0
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "The STEP file's contents are embedded in this \
                                             design, so it opens anywhere without the original \
                                             file.",
                                        )
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::DatumPlane { def } => match def {
                                    DatumPlaneDef::Offset { distance, .. } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Offset").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(distance)
                                                        .speed(0.5)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    DatumPlaneDef::Angle { angle_deg, .. } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Angle").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(angle_deg)
                                                        .speed(1.0)
                                                        .suffix("°"),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    DatumPlaneDef::ThreePoints { a, b, c } => {
                                        for (label, p) in [("A", a), ("B", b), ("C", c)] {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new(label).size(12.0));
                                                for coord in p.iter_mut() {
                                                    if ui
                                                        .add(egui::DragValue::new(coord).speed(0.5))
                                                        .changed()
                                                    {
                                                        modified = true;
                                                    }
                                                }
                                            });
                                        }
                                    }
                                    DatumPlaneDef::MidPlane { .. } => {
                                        ui.label(
                                            egui::RichText::new("Mid-plane between two planes.")
                                                .size(11.5)
                                                .color(pal.text_muted),
                                            );
                                    }
                                    DatumPlaneDef::PlanarFace { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative plane from a named planar face.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::DatumAxis { def } => match def {
                                    DatumAxisDef::TwoPoints { a, b } => {
                                        for (label, p) in [("From", a), ("To", b)] {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new(label).size(12.0));
                                                for coord in p.iter_mut() {
                                                    if ui
                                                        .add(egui::DragValue::new(coord).speed(0.5))
                                                        .changed()
                                                    {
                                                        modified = true;
                                                    }
                                                }
                                            });
                                        }
                                    }
                                    DatumAxisDef::PlaneIntersection { .. } => {
                                        ui.label(
                                            egui::RichText::new("Intersection line of two planes.")
                                                .size(11.5)
                                                .color(pal.text_muted),
                                            );
                                    }
                                    DatumAxisDef::Edge { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis from a named edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumAxisDef::CylindricalOrConicalFace { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis from an analytic cylindrical or conical face.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumAxisDef::TwoVertices { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis through two named vertices.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::DatumPoint { def } => match def {
                                    DatumPointDef::Coords { p } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Position").size(12.0));
                                            for coord in p.iter_mut() {
                                                if ui
                                                    .add(egui::DragValue::new(coord).speed(0.5))
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            }
                                        });
                                    }
                                    DatumPointDef::Vertex { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative point from a named vertex.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::Midpoint { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative midpoint between two named vertices.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::EdgeMidpoint { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative midpoint of a named edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::CircleCenter { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative center of a named circular edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::Revolve {
                                    angle_deg,
                                    angle_expr,
                                    mode,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Mode: {:?}", mode))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Angle").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(angle_deg)
                                                    .speed(1.0)
                                                    .range(0.1..=360.0)
                                                    .suffix("°"),
                                            )
                                            .changed()
                                        {
                                            *angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        angle_expr,
                                        &var_map,
                                        "expression, e.g. 360/2",
                                        "°",
                                        &mut modified,
                                    );
                                }
                                FeatureType::Loft {
                                    sections,
                                    surface_mode,
                                    mode,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Loft through {} sections ({:?}).",
                                            sections.len(),
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    let previous = *surface_mode;
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Surface").size(12.0));
                                        egui::ComboBox::from_id_salt("loft_surface_mode")
                                            .selected_text(format!("{surface_mode:?}"))
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(
                                                    surface_mode,
                                                    LoftSurfaceMode::Ruled,
                                                    "Ruled",
                                                );
                                                ui.selectable_value(
                                                    surface_mode,
                                                    LoftSurfaceMode::Smooth,
                                                    "Smooth",
                                                );
                                            });
                                    });
                                    modified |= *surface_mode != previous;
                                }
                                FeatureType::Sweep {
                                    path_sketch,
                                    guide,
                                    mode,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Sweep along '{path_sketch}' ({:?}).",
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    if let Some(guide) = guide {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "Guide '{}' Â· profile anchor {:.0}%",
                                                guide.sketch,
                                                guide.profile_parameter * 100.0
                                            ))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                        ui.horizontal(|ui| {
                                            ui.label(
                                                egui::RichText::new("Anchor position").size(12.0),
                                            );
                                            if ui
                                                .add(
                                                    egui::DragValue::new(
                                                        &mut guide.profile_parameter,
                                                    )
                                                    .range(0.0..=1.0)
                                                    .speed(0.01),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                }
                                FeatureType::Shell {
                                    thickness,
                                    thickness_expr,
                                    ..
                                } => {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Thickness").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(thickness)
                                                    .speed(0.1)
                                                    .range(0.1..=100.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *thickness_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        thickness_expr,
                                        &var_map,
                                        "expression, e.g. 5/2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                }
                                FeatureType::Hole {
                                    position,
                                    diameter,
                                    diameter_expr,
                                    depth,
                                    kind,
                                    standard,
                                    manufacturing,
                                    ..
                                } => {
                                    if let Some(reference) = standard.as_ref() {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{} {} · {} v{}",
                                                reference.family.label(),
                                                reference.designation,
                                                reference.table,
                                                reference.library_version
                                            ))
                                            .strong()
                                            .size(11.5)
                                            .color(pal.text_strong),
                                        );
                                        if let Some(class) = &reference.class {
                                            ui.label(
                                                egui::RichText::new(format!("Class {class}"))
                                                    .small()
                                                    .color(pal.text_muted),
                                            );
                                        }
                                    }
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Diameter").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(diameter)
                                                    .speed(0.2)
                                                    .range(0.1..=500.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *diameter_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        diameter_expr,
                                        &var_map,
                                        "expression, e.g. 43/2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                    if let Some(d) = depth {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Depth").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(d)
                                                        .speed(0.2)
                                                        .range(0.1..=1000.0)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    if let Some(metadata) = manufacturing.as_mut() {
                                        if metadata.cosmetic_thread {
                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "Tapped-hole metadata: {} · class {} · pitch {} mm (cosmetic)",
                                                    metadata
                                                        .thread_designation
                                                        .as_deref()
                                                        .unwrap_or("custom"),
                                                    metadata.thread_class.as_deref().unwrap_or("—"),
                                                    metadata
                                                        .tap_pitch_mm
                                                        .map_or_else(|| "—".to_string(), |pitch| pitch.to_string())
                                                ))
                                                .small()
                                                .color(pal.text_muted),
                                            );
                                        }
                                        if let Some(angle) = &mut metadata.drill_point_angle_deg {
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Drill-point angle")
                                                        .size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(angle)
                                                            .speed(0.5)
                                                            .range(1.0..=179.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                    }
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Position").size(12.0));
                                        for coord in position.iter_mut() {
                                            if ui
                                                .add(egui::DragValue::new(coord).speed(0.5))
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        }
                                    });
                                    if let Some(reference) = standard.as_mut() {
                                        let (head_diameter, head_depth, head_angle) = match kind {
                                            zerocad_core::HoleKind::Simple => (None, None, None),
                                            zerocad_core::HoleKind::Counterbore {
                                                diameter,
                                                depth,
                                            } => {
                                                (Some(*diameter), Some(*depth), None)
                                            }
                                            zerocad_core::HoleKind::Countersink {
                                                diameter,
                                                angle_deg,
                                            } => (Some(*diameter), None, Some(*angle_deg)),
                                        };
                                        if let zerocad_core::ResolvedStandardGeometry::Hole {
                                            bore_diameter_mm,
                                            head_diameter_mm,
                                            head_depth_mm,
                                            head_angle_deg,
                                            ..
                                        } = &mut reference.resolved
                                        {
                                            *bore_diameter_mm = *diameter;
                                            *head_diameter_mm = head_diameter;
                                            *head_depth_mm = head_depth;
                                            *head_angle_deg = head_angle;
                                        }
                                    }
                                }
                                FeatureType::Pattern { source, kind } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source: {source}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    match kind {
                                        zerocad_core::PatternKind::Linear {
                                            spacing,
                                            spacing_expr,
                                            count,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Spacing").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(spacing)
                                                            .speed(0.5)
                                                            .suffix(current_unit.suffix()),
                                                    )
                                                    .changed()
                                                {
                                                    *spacing_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                spacing_expr,
                                                &var_map,
                                                "expression, e.g. pitch*2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(2..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::PatternKind::Circular {
                                            count,
                                            total_angle_deg,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(2..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Total angle").size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(total_angle_deg)
                                                            .speed(1.0)
                                                            .range(1.0..=360.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::PatternKind::Mirror {
                                            offset,
                                            offset_expr,
                                            join,
                                            ..
                                        } => {
                                            ui.label(
                                                egui::RichText::new("Mirrored copy of the source.")
                                                    .size(11.5)
                                                    .color(pal.text_muted),
                                            );
                                            if *join {
                                                let (text, color) = match mirror_join_outcome
                                                    .as_ref()
                                                    .expect("Mirror+Join outcome is precomputed")
                                                {
                                                    MirrorJoinOutcome::Evaluating => (
                                                        "Outcome: Evaluating join".to_string(),
                                                        pal.text_muted,
                                                    ),
                                                    MirrorJoinOutcome::Joined => (
                                                        "Outcome: Joined into source body".to_string(),
                                                        egui::Color32::from_rgb(22, 101, 52),
                                                    ),
                                                    MirrorJoinOutcome::Separate => (
                                                        "Outcome: Separate body (Join could not connect)"
                                                            .to_string(),
                                                        egui::Color32::from_rgb(180, 83, 9),
                                                    ),
                                                    MirrorJoinOutcome::Unresolved(reason) => (
                                                        format!("Outcome: Unresolved — {reason}"),
                                                        egui::Color32::from_rgb(185, 28, 28),
                                                    ),
                                                };
                                                ui.label(
                                                    egui::RichText::new(text)
                                                        .strong()
                                                        .size(11.5)
                                                        .color(color),
                                                );
                                            }
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Offset").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(offset)
                                                            .speed(0.2)
                                                            .suffix(current_unit.suffix()),
                                                    )
                                                    .changed()
                                                {
                                                    *offset_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                offset_expr,
                                                &var_map,
                                                "expression, e.g. width/2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
                                        }
                                    }
                                }
                                FeatureType::FeaturePattern {
                                    target,
                                    source_feature,
                                    kind,
                                    compute_mode,
                                    extent_policy,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Source feature: {source_feature}  •  Target: {target}"
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Compute: {compute_mode:?}  •  Extent: {extent_policy:?}"
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    match kind {
                                        zerocad_core::parametric::FeaturePatternKind::Linear {
                                            spacing,
                                            spacing_expr,
                                            count,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Spacing").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(spacing)
                                                            .speed(0.5)
                                                            .suffix(current_unit.suffix()),
                                                    )
                                                    .changed()
                                                {
                                                    *spacing_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                spacing_expr,
                                                &var_map,
                                                "expression, e.g. pitch*2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(0..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::parametric::FeaturePatternKind::Circular {
                                            total_angle_deg,
                                            total_angle_expr,
                                            count,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Total angle").size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(total_angle_deg)
                                                            .speed(1.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    *total_angle_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                total_angle_expr,
                                                &var_map,
                                                "expression, e.g. full_angle",
                                                "°",
                                                &mut modified,
                                            );
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(0..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                    }
                                }
                                FeatureType::Draft {
                                    target,
                                    faces,
                                    neutral,
                                    angle_deg,
                                    angle_expr,
                                    flip_pull,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Target: {target}  •  Selected side faces: {}",
                                            faces.len()
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    let neutral_label = match neutral {
                                        zerocad_core::parametric::DraftNeutral::Face(face) => face
                                            .topology
                                            .as_ref()
                                            .and_then(|topology| topology.face_id.as_deref())
                                            .map_or_else(
                                                || "legacy face reference".to_string(),
                                                ToString::to_string,
                                            ),
                                        zerocad_core::parametric::DraftNeutral::Datum(id) => {
                                            format!("datum {id}")
                                        }
                                    };
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Neutral reference: {neutral_label}"
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Draft angle").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(angle_deg)
                                                    .speed(0.25)
                                                    .range(-88.9..=88.9)
                                                    .suffix("°"),
                                            )
                                            .changed()
                                        {
                                            *angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        angle_expr,
                                        &var_map,
                                        "expression, e.g. taper_angle",
                                        "°",
                                        &mut modified,
                                    );
                                    if ui
                                        .checkbox(flip_pull, "Reverse pull direction")
                                        .changed()
                                    {
                                        modified = true;
                                    }
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "Draft v1 supports straight planar side faces on retained prismatic bodies.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::Cylinder { r, h } => {
                                    ui.label(
                                        egui::RichText::new("Dimensions:")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(6.0);
                                    egui::Grid::new("cyl_grid")
                                        .spacing(egui::vec2(10.0, 10.0))
                                        .show(ui, |ui| {
                                            ui.label(egui::RichText::new("Radius").size(12.0));
                                            let r_resp = ui.add(
                                                egui::Slider::new(r, 2.0..=80.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if r_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Height").size(12.0));
                                            let h_resp = ui.add(
                                                egui::Slider::new(h, 5.0..=200.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if h_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();
                                        });
                                }
                                FeatureType::Sketch {
                                    cs,
                                    curves,
                                    shapes,
                                    corner_mods,
                                    mirrors,
                                    solver,
                                    ..
                                } => {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Plane:").size(12.0));
                                        ui.label(
                                            egui::RichText::new(Self::cs_label(cs))
                                                .strong()
                                                .size(12.0)
                                                .color(egui::Color32::from_rgb(37, 99, 235)),
                                        );
                                    });
                                    ui.add_space(4.0);
                                    // Resolve against the current variables so the counts
                                    // (and any extrude below) reflect variable-driven dims.
                                    let mut eff = zerocad_core::effective_curves_solved(
                                        curves,
                                        shapes,
                                        corner_mods,
                                        mirrors,
                                        solver.as_ref(),
                                        &var_map,
                                    );
                                    // Sketch-on-face: the projected boundary joins
                                    // region detection, so the face count here
                                    // matches the viewport and the evaluator.
                                    if let Some(b) = face_boundary.as_ref() {
                                        eff.extend_curves(b);
                                    }
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Curves: {} segments, {} circles",
                                            eff.segments.len(),
                                            eff.circles.len()
                                        ))
                                        .size(11.5)
                                        .weak(),
                                    );
                                    let regions = detect_regions(&eff);
                                    ui.label(
                                        egui::RichText::new(format!("Faces: {}", regions.len()))
                                            .size(11.5)
                                            .weak(),
                                    );
                                    // Surface stored expression dimensions so their editable
                                    // source remains visible after evaluation.
                                    let bound = sketch_variable_dims(shapes);
                                    if !bound.is_empty() {
                                        ui.add_space(2.0);
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "🔗 Expression dims: {}",
                                                bound.join(", ")
                                            ))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(37, 99, 235)),
                                        );
                                    }
                                    ui.add_space(8.0);

                                    let has_faces = !regions.is_empty();
                                    let extrude_btn = icons::Icon::Extrude.labeled_button(
                                        ui,
                                        "Extrude whole Sketch",
                                        if has_faces {
                                            egui::Color32::from_rgb(37, 99, 235)
                                        } else {
                                            egui::Color32::from_rgb(241, 245, 249)
                                        },
                                        if has_faces {
                                            egui::Color32::from_rgb(29, 78, 216)
                                        } else {
                                            egui::Color32::from_rgb(241, 245, 249)
                                        },
                                        if has_faces {
                                            egui::Color32::WHITE
                                        } else {
                                            pal.text_faint
                                        },
                                        if has_faces {
                                            egui::Stroke::NONE
                                        } else {
                                            egui::Stroke::new(
                                                1.0,
                                                egui::Color32::from_rgb(226, 232, 240),
                                            )
                                        },
                                    );
                                    if has_faces && extrude_btn.clicked() {
                                        extrude_request = Some(node.id.clone());
                                    }
                                    ui.add_space(4.0);
                                    if ui
                                        .button(egui::RichText::new("✏ Edit Sketch").size(12.0))
                                        .on_hover_text(
                                            "Re-open this sketch: drag points, geometry \
                                             re-solves against its constraints; Finish \
                                             commits in place (downstream features follow).",
                                        )
                                        .clicked()
                                    {
                                        edit_sketch_request = Some(node.id.clone());
                                    }
                                    // Constraint status: DOF / fully constrained /
                                    // conflict, from the solver model when present.
                                    if let Some(model) = solver.as_ref().filter(|m| !m.is_empty()) {
                                        let report =
                                            zerocad_core::sketch::solve_model(model, &var_map);
                                        let (text, color) = match report.outcome {
                                            zerocad_core::sketch::SolveOutcome::Conflicting => (
                                                match report.conflicting {
                                                    Some(id) => format!(
                                                        "⚠ Over-constrained (conflict: {})",
                                                        id.0
                                                    ),
                                                    None => "⚠ Over-constrained".to_string(),
                                                },
                                                egui::Color32::from_rgb(220, 38, 38),
                                            ),
                                            zerocad_core::sketch::SolveOutcome::DidNotConverge => (
                                                "⚠ Constraints did not converge".to_string(),
                                                egui::Color32::from_rgb(220, 38, 38),
                                            ),
                                            zerocad_core::sketch::SolveOutcome::Converged => {
                                                if report.dof == 0 {
                                                    (
                                                        "● Fully constrained".to_string(),
                                                        egui::Color32::from_rgb(22, 163, 74),
                                                    )
                                                } else {
                                                    (
                                                        format!("◐ {} DOF remaining", report.dof),
                                                        egui::Color32::from_rgb(37, 99, 235),
                                                    )
                                                }
                                            }
                                        };
                                        ui.add_space(2.0);
                                        ui.label(egui::RichText::new(text).size(11.0).color(color));
                                    }
                                }
                                FeatureType::Extrude {
                                    depth,
                                    region_indices,
                                    mode,
                                    depth_expr,
                                    draft_angle_deg,
                                    draft_angle_expr,
                                    ..
                                } => {
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(
                                            egui::RichText::new("Extrusion Depth:").size(12.0),
                                        );
                                        let d_resp = ui.add(
                                            egui::Slider::new(depth, 1.0..=150.0)
                                                .suffix(current_unit.suffix()),
                                        );
                                        if d_resp.changed() {
                                            // Dragging sets a literal depth — drop the binding.
                                            *depth_expr = None;
                                            modified = true;
                                        }
                                    });
                                    // Variable/expression binding: a depth like `width / 2`
                                    // re-evaluates whenever the variable changes. Empty clears it.
                                    ui.add_space(4.0);
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buf = depth_expr.clone().unwrap_or_default();
                                        let r = ui.add(
                                            egui::TextEdit::singleline(&mut buf)
                                                .hint_text("expression, e.g. width / 2")
                                                .desired_width(150.0),
                                        );
                                        if r.changed() {
                                            let t = buf.trim();
                                            *depth_expr = if t.is_empty() {
                                                None
                                            } else {
                                                Some(t.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(e) = depth_expr.as_ref() {
                                        let txt = match zerocad_core::expr::eval(e, &var_map) {
                                            Ok(v) => {
                                                format!("→ {:.2} {}", v, current_unit.suffix())
                                            }
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(txt).size(11.0).weak());
                                    }
                                    ui.add_space(6.0);
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(egui::RichText::new("Draft Angle:").size(12.0));
                                        let response = ui.add(
                                            egui::Slider::new(draft_angle_deg, -88.9..=88.9)
                                                .suffix("°"),
                                        );
                                        if response.changed() {
                                            *draft_angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buffer =
                                            draft_angle_expr.clone().unwrap_or_default();
                                        let response = ui.add(
                                            egui::TextEdit::singleline(&mut buffer)
                                                .hint_text("angle expression")
                                                .desired_width(150.0),
                                        );
                                        if response.changed() {
                                            let value = buffer.trim();
                                            *draft_angle_expr = if value.is_empty() {
                                                None
                                            } else {
                                                Some(value.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(expression) = draft_angle_expr.as_ref() {
                                        let text = match zerocad_core::expr::eval(
                                            expression,
                                            &var_map,
                                        ) {
                                            Ok(value) if value.abs() < 89.0 => {
                                                format!("→ {value:.3}°")
                                            }
                                            Ok(value) => format!(
                                                "→ {value:.3}° (invalid: must be between -89° and 89°)"
                                            ),
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(text).size(11.0).weak());
                                    }
                                    ui.add_space(6.0);
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(egui::RichText::new("Operation:").size(12.0));
                                        for (m, label) in [
                                            (ExtrudeMode::NewBody, "New Body"),
                                            (ExtrudeMode::Join, "Join"),
                                            (ExtrudeMode::Cut, "Cut"),
                                        ] {
                                            if ui.selectable_label(*mode == m, label).clicked()
                                                && *mode != m
                                            {
                                                *mode = m;
                                                modified = true;
                                            }
                                        }
                                    });
                                    ui.add_space(6.0);
                                    if region_indices.is_empty() {
                                        ui.label(
                                            egui::RichText::new("Regions: all detected")
                                                .size(11.5)
                                                .weak(),
                                        );
                                    } else {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "Regions: {:?}",
                                                region_indices
                                            ))
                                            .size(11.5)
                                            .weak(),
                                        );
                                    }
                                }
                                FeatureType::EdgeMod {
                                    dist,
                                    dist_expr,
                                    kind,
                                    ..
                                } => {
                                    let noun = match kind {
                                        CornerKind::Fillet => "Fillet radius:",
                                        CornerKind::Chamfer => "Chamfer distance:",
                                    };
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new(noun).size(12.0));
                                        let d_resp = ui.add(
                                            egui::Slider::new(dist, 0.2..=40.0)
                                                .suffix(current_unit.suffix()),
                                        );
                                        if d_resp.changed() {
                                            *dist_expr = None; // a literal drag drops the binding
                                            modified = true;
                                        }
                                    });
                                    // Variable/expression binding, mirroring the extrude depth.
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buf = dist_expr.clone().unwrap_or_default();
                                        let r = ui.add(
                                            egui::TextEdit::singleline(&mut buf)
                                                .hint_text("expression, e.g. fillet_r")
                                                .desired_width(150.0),
                                        );
                                        if r.changed() {
                                            let t = buf.trim();
                                            *dist_expr = if t.is_empty() {
                                                None
                                            } else {
                                                Some(t.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(e) = dist_expr.as_ref() {
                                        let txt = match zerocad_core::expr::eval(e, &var_map) {
                                            Ok(v) => {
                                                format!("→ {:.2} {}", v, current_unit.suffix())
                                            }
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(txt).size(11.0).weak());
                                    }
                                    ui.add_space(6.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Type:").size(12.0));
                                        for (k, label) in [
                                            (CornerKind::Fillet, "Fillet"),
                                            (CornerKind::Chamfer, "Chamfer"),
                                        ] {
                                            if ui.selectable_label(*kind == k, label).clicked()
                                                && *kind != k
                                            {
                                                *kind = k;
                                                modified = true;
                                            }
                                        }
                                    });
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "Edge captured in 3D; edits re-cut the body.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::BodyTransform {
                                    source,
                                    translation,
                                    copy,
                                } => {
                                    ui.label(
                                        egui::RichText::new(if *copy {
                                            format!("Copy of {source}")
                                        } else {
                                            format!("Moved from {source}")
                                        })
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new(label).size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut translation[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                }
                                FeatureType::BodyJoin { sources } => {
                                    ui.label(
                                        egui::RichText::new("Joined bodies")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(4.0);
                                    for source in sources {
                                        ui.label(
                                            egui::RichText::new(source.as_str())
                                                .size(11.5)
                                                .color(pal.text_muted),
                                        );
                                    }
                                }
                                FeatureType::BodyCut {
                                    target,
                                    tool,
                                    keep_tool,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Target: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!("Cutting body: {tool}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    if ui.checkbox(keep_tool, "Keep cutting body").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::BodyIntersect {
                                    target,
                                    tool,
                                    keep_tool,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Target: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!("Intersecting body: {tool}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    if ui.checkbox(keep_tool, "Keep tool body").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::BodySplit {
                                    target, plane, face, ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(if face.is_some() {
                                            "Reference: planar body face".to_string()
                                        } else {
                                            format!("Reference: {plane:?}")
                                        })
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new("Outputs: negative and positive side")
                                            .size(10.5)
                                            .color(pal.text_faint),
                                    );
                                }
                                FeatureType::BodyScale {
                                    source,
                                    factor,
                                    factor_expr,
                                    center,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source: {source}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Factor");
                                        if let Some(expression) = factor_expr {
                                            if ui.text_edit_singleline(expression).changed() {
                                                modified = true;
                                            }
                                        } else if ui
                                            .add(egui::DragValue::new(factor).speed(0.05))
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.label("Pivot");
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(label);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut center[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    if let Some(body_center) = current_body_center {
                                        if ui.button("Reset pivot to current center").clicked() {
                                            *center = body_center;
                                            modified = true;
                                        }
                                    }
                                }
                                FeatureType::FaceOffset {
                                    target,
                                    distance,
                                    distance_expr,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Signed distance");
                                        if ui
                                            .add(
                                                egui::DragValue::new(distance)
                                                    .speed(0.25)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *distance_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        distance_expr,
                                        &var_map,
                                        "expression, e.g. wall+2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Positive adds material; negative removes it.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceMove {
                                    target,
                                    translation,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label("Normal translation");
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(label);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut translation[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    ui.label(
                                        egui::RichText::new(
                                            "Tangential movement is rejected in Phase 5.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceDelete { target, .. } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Heals an internal cylindrical face (hole removal).",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceThicken {
                                    target,
                                    thickness,
                                    thickness_expr,
                                    reverse,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Thickness");
                                        if ui
                                            .add(
                                                egui::DragValue::new(thickness)
                                                    .speed(0.25)
                                                    .range(0.001..=1.0e6)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *thickness_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        thickness_expr,
                                        &var_map,
                                        "expression, e.g. sheet_thickness",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                    if ui.checkbox(reverse, "Reverse direction").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::ImportStl { stl_data, label } => {
                                    ui.label(
                                        egui::RichText::new("STL mesh body")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{} · {} bytes · mesh-only",
                                            label,
                                            stl_data.len()
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Supports move, scale, measure, section, and mesh export; B-Rep booleans are disabled.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::Thread {
                                    internal,
                                    pitch,
                                    depth,
                                    angle_deg,
                                    right_handed,
                                    designation,
                                    standard,
                                    ..
                                } => {
                                    let original_internal = *internal;
                                    if let Some(reference) = standard.as_ref() {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{} · {} v{} · class {}",
                                                reference.table,
                                                reference.library_id,
                                                reference.library_version,
                                                reference.class.as_deref().unwrap_or("—")
                                            ))
                                            .small()
                                            .color(pal.text_muted),
                                        );
                                    }
                                    if !designation.is_empty() {
                                        ui.label(
                                            egui::RichText::new(designation.as_str())
                                                .strong()
                                                .size(12.0)
                                                .color(pal.text_strong),
                                        );
                                        ui.add_space(4.0);
                                    }
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Pitch").size(12.0));
                                        if ui
                                            .add(
                                                egui::Slider::new(pitch, 0.2..=6.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Depth").size(12.0));
                                        if ui
                                            .add(
                                                egui::Slider::new(depth, 0.05..=3.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        if ui.selectable_label(!*internal, "External").clicked()
                                            && *internal
                                        {
                                            *internal = false;
                                            modified = true;
                                        }
                                        if ui.selectable_label(*internal, "Internal").clicked()
                                            && !*internal
                                        {
                                            *internal = true;
                                            modified = true;
                                        }
                                        ui.separator();
                                        if ui.selectable_label(*right_handed, "RH").clicked()
                                            && !*right_handed
                                        {
                                            *right_handed = true;
                                            modified = true;
                                        }
                                        if ui.selectable_label(!*right_handed, "LH").clicked()
                                            && *right_handed
                                        {
                                            *right_handed = false;
                                            modified = true;
                                        }
                                    });
                                    ui.add_space(4.0);
                                    if let Some(reference) = standard.as_mut() {
                                        let thread_standard = match reference.family {
                                            zerocad_core::StandardsFamily::Iso => Some(
                                                zerocad_core::parametric::ThreadStandard::MetricCoarse,
                                            ),
                                            zerocad_core::StandardsFamily::Ansi
                                                if reference.table.contains("UNF") =>
                                            {
                                                Some(
                                                    zerocad_core::parametric::ThreadStandard::UnifiedFine,
                                                )
                                            }
                                            zerocad_core::StandardsFamily::Ansi => Some(
                                                zerocad_core::parametric::ThreadStandard::UnifiedCoarse,
                                            ),
                                        };
                                        if let Some(thread_standard) = thread_standard {
                                            let classes = zerocad_core::parametric::thread_classes(
                                                thread_standard,
                                                *internal,
                                            );
                                            if original_internal != *internal
                                                || reference.class.as_deref().is_none_or(|class| {
                                                    !classes.contains(&class)
                                                })
                                            {
                                                reference.class = classes
                                                    .get(1)
                                                    .copied()
                                                    .map(str::to_string);
                                            }
                                            let mut selected =
                                                reference.class.clone().unwrap_or_default();
                                            ui.horizontal(|ui| {
                                                ui.label("Class");
                                                egui::ComboBox::from_id_salt(
                                                    "thread_property_class",
                                                )
                                                .selected_text(&selected)
                                                .show_ui(ui, |ui| {
                                                    for class in classes {
                                                        ui.selectable_value(
                                                            &mut selected,
                                                            (*class).to_string(),
                                                            *class,
                                                        );
                                                    }
                                                });
                                            });
                                            if reference.class.as_deref()
                                                != Some(selected.as_str())
                                            {
                                                reference.class = Some(selected);
                                                modified = true;
                                            }
                                        }
                                        if let zerocad_core::ResolvedStandardGeometry::Thread {
                                            pitch_mm,
                                            radial_depth_mm,
                                            angle_deg: resolved_angle,
                                            ..
                                        } = &mut reference.resolved
                                        {
                                            *pitch_mm = *pitch;
                                            *radial_depth_mm = *depth;
                                            *resolved_angle = *angle_deg;
                                        }
                                    }
                                    ui.label(
                                        egui::RichText::new(
                                            "Cut into the selected cylindrical face. A failed \
                                             helical boolean leaves the body intact (cosmetic).",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::VariableSet { variables } => {
                                    // Section header: "Variables" + count.
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("Variables")
                                                .strong()
                                                .size(12.0)
                                                .color(pal.text_strong),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!("({})", variables.len()))
                                                .size(11.5)
                                                .color(pal.text_faint),
                                        );
                                    });
                                    ui.add_space(8.0);

                                    if variables.is_empty() {
                                        egui::Frame::none()
                                            .fill(egui::Color32::from_rgb(248, 250, 252))
                                            .rounding(6.0)
                                            .inner_margin(10.0)
                                            .show(ui, |ui| {
                                                ui.label(
                                                    egui::RichText::new(
                                                        "No variables yet. Add one below.",
                                                    )
                                                    .size(11.5)
                                                    .color(pal.text_muted),
                                                );
                                            });
                                    }

                                    // Each variable is its own soft card: a full-width
                                    // name field on top, then a value-or-expression
                                    // field, unit, and delete action.
                                    let mut remove_idx: Option<usize> = None;
                                    for (i, var) in variables.iter_mut().enumerate() {
                                        egui::Frame::none()
                                            .fill(egui::Color32::from_rgb(248, 250, 252)) // slate-50
                                            .rounding(6.0)
                                            .stroke(egui::Stroke::new(
                                                1.0,
                                                egui::Color32::from_rgb(226, 232, 240),
                                            ))
                                            .inner_margin(8.0)
                                            .show(ui, |ui| {
                                                if ui
                                                    .add(
                                                        egui::TextEdit::singleline(&mut var.name)
                                                            .desired_width(f32::INFINITY)
                                                            .hint_text("name")
                                                            .font(egui::FontId::proportional(12.5)),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                                ui.add_space(6.0);
                                                ui.horizontal_wrapped(|ui| {
                                                    let editor_id = egui::Id::new((
                                                        "variable_value_expression",
                                                        &selected_feature_id,
                                                        i,
                                                    ));
                                                    let was_focused = ui
                                                        .memory(|memory| memory.has_focus(editor_id));
                                                    let mut source = if was_focused {
                                                        ui.ctx()
                                                            .data(|data| {
                                                                data.get_temp::<String>(editor_id)
                                                            })
                                                            .unwrap_or_else(|| {
                                                                variable_value_source(var)
                                                            })
                                                    } else {
                                                        variable_value_source(var)
                                                    };
                                                    let response = ui.add(
                                                        egui::TextEdit::singleline(&mut source)
                                                            .id(editor_id)
                                                            .desired_width(104.0)
                                                            .hint_text("value or expression")
                                                            .font(egui::FontId::monospace(11.5)),
                                                    )
                                                    .on_hover_text(
                                                        "Enter a number or an expression using \
                                                         other variables, e.g. blade_width / 2",
                                                    );
                                                    if response.changed() {
                                                        apply_variable_value_source(var, &source);
                                                        modified = true;
                                                    }
                                                    if response.has_focus() {
                                                        ui.ctx().data_mut(|data| {
                                                            data.insert_temp(editor_id, source);
                                                        });
                                                    } else {
                                                        ui.ctx().data_mut(|data| {
                                                            data.remove::<String>(editor_id);
                                                        });
                                                    }
                                                    let old_unit = var.unit;
                                                    egui::ComboBox::from_id_salt((
                                                        "var_unit",
                                                        &selected_feature_id,
                                                        i,
                                                    ))
                                                        .selected_text(var.unit.suffix())
                                                        .width(50.0)
                                                        .show_ui(ui, |ui| {
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Millimeter,
                                                                "mm",
                                                            );
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Inch,
                                                                "in",
                                                            );
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Meter,
                                                                "m",
                                                            );
                                                        });
                                                    if var.unit != old_unit {
                                                        modified = true;
                                                    }
                                                    ui.with_layout(
                                                        egui::Layout::right_to_left(
                                                            egui::Align::Center,
                                                        ),
                                                        |ui| {
                                                            let del = icons::Icon::Trash
                                                                .icon_button(
                                                                    ui,
                                                                    egui::Color32::TRANSPARENT,
                                                                    egui::Color32::from_rgb(
                                                                        254, 226, 226,
                                                                    ),
                                                                    egui::Color32::from_rgb(
                                                                        185, 28, 28,
                                                                    ),
                                                                );
                                                            if del
                                                                .on_hover_text("Delete variable")
                                                                .clicked()
                                                            {
                                                                remove_idx = Some(i);
                                                                modified = true;
                                                            }
                                                        },
                                                    );
                                                });
                                                if let Some(diagnostic) = variable_resolution
                                                    .diagnostics
                                                    .iter()
                                                    .find(|diagnostic| {
                                                        diagnostic.name == var.name.trim()
                                                    })
                                                {
                                                    ui.label(
                                                        egui::RichText::new(&diagnostic.message)
                                                            .size(10.5)
                                                            .color(pal.danger),
                                                    );
                                                } else if var.expression.is_some() {
                                                    if let Some(value) =
                                                        var_map.get(var.name.trim())
                                                    {
                                                        let display_value =
                                                            var.unit.from_base(*value);
                                                        ui.label(
                                                            egui::RichText::new(format!(
                                                                "Resolved: {display_value:.4} {}",
                                                                var.unit.suffix()
                                                            ))
                                                            .size(10.5)
                                                            .color(pal.text_muted),
                                                        );
                                                    }
                                                }
                                            });
                                        ui.add_space(6.0);
                                    }
                                    if let Some(i) = remove_idx {
                                        variables.remove(i);
                                    }

                                    ui.add_space(2.0);
                                    let add = icons::Icon::Sketch.labeled_button(
                                        ui,
                                        "Add Variable",
                                        egui::Color32::from_rgb(37, 99, 235),
                                        egui::Color32::from_rgb(29, 78, 216),
                                        egui::Color32::WHITE,
                                        egui::Stroke::NONE,
                                    );
                                    if add.clicked() {
                                        let n = variables.len() + 1;
                                        variables
                                            .push(Variable::new(format!("var{}", n), current_unit));
                                        modified = true;
                                    }
                                }
                            }
                            driving_parameters_editor(
                                ui,
                                &selected_feature_id,
                                &driving_parameters,
                                &mut driving_parameter_edits,
                                &mut driving_parameter_edit_started,
                            );
                        });
                    });

                if driving_parameter_edit_started {
                    self.push_undo();
                }
                let mut needs_reevaluation = false;
                for edit in driving_parameter_edits {
                    match apply_driving_parameter_edit(&mut self.document, &edit) {
                        Ok(()) => {
                            needs_reevaluation = true;
                            self.status_msg = format!(
                                "Updated parameter '{}' to {} {}.",
                                edit.name,
                                edit.value,
                                edit.unit.suffix()
                            );
                        }
                        Err(error) => {
                            self.status_msg =
                                format!("Could not update parameter '{}': {error}", edit.name);
                        }
                    }
                }

                if let Some(suppressed) = suppression_request {
                    self.push_undo();
                    if self
                        .document
                        .set_feature_suppressed(&selected_feature_id, suppressed)
                    {
                        self.status_msg = if suppressed {
                            format!("Suppressed feature '{selected_feature_id}'.")
                        } else {
                            format!("Resumed feature '{selected_feature_id}'.")
                        };
                        modified = true;
                    }
                }

                if modified {
                    match self.document.commit_feature_edit(&selected_feature_id) {
                        Ok(()) => needs_reevaluation = true,
                        Err(error) => {
                            log::error!(
                                "Failed to synchronize feature '{}': {error}",
                                selected_feature_id
                            );
                            self.status_msg = format!(
                                "Feature '{}' has inconsistent document relationships: {error}",
                                selected_feature_id
                            );
                        }
                    }
                }
                if needs_reevaluation {
                    self.reevaluate_geometry();
                }

                if let Some(sketch_id) = extrude_request {
                    self.begin_extrude_whole_sketch(&sketch_id);
                }
                if let Some(sketch_id) = edit_sketch_request {
                    let now = ui.input(|i| i.time);
                    self.edit_sketch(&sketch_id, now);
                }

                // Measure: physical properties of the selected body's mesh.
                // Computed on demand from the already-tessellated buffers, so
                // it's exact for what's on screen.
                let measured = self
                    .body_meshes
                    .iter()
                    .find(|(id, _)| id == &selected_feature_id)
                    .and_then(|(_, mesh)| mesh.mass_properties());
                if let Some(mp) = measured {
                    ui.add_space(10.0);
                    egui::Frame::none()
                        .fill(pal.surface)
                        .rounding(8.0)
                        .stroke(egui::Stroke::new(1.0, pal.border))
                        .inner_margin(12.0)
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new("Measure")
                                    .strong()
                                    .size(12.5)
                                    .color(self.pal().text_strong),
                            );
                            ui.add_space(4.0);
                            let muted = self.pal().text_muted;
                            let row = |ui: &mut egui::Ui, k: &str, v: String| {
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new(k).size(11.5).color(muted));
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.label(egui::RichText::new(v).monospace().size(11.5));
                                        },
                                    );
                                });
                            };
                            row(ui, "Volume", format!("{:.3} mm³", mp.volume));
                            row(ui, "Surface area", format!("{:.3} mm²", mp.surface_area));
                            row(
                                ui,
                                "Centroid",
                                format!(
                                    "({:.2}, {:.2}, {:.2})",
                                    mp.centroid[0], mp.centroid[1], mp.centroid[2]
                                ),
                            );
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new("Density (g/cm³)")
                                        .size(11.5)
                                        .color(self.pal().text_muted),
                                );
                                ui.add(
                                    egui::DragValue::new(&mut self.measure_density)
                                        .speed(0.05)
                                        .range(0.0..=30.0),
                                );
                            });
                            // mm³ → cm³ is ÷1000; g/cm³ × cm³ = grams.
                            let grams = mp.volume / 1000.0 * self.measure_density as f64;
                            row(ui, "Mass", format!("{grams:.2} g"));
                        });
                }
            }
        } else {
            // Render a clean fallback banner
            egui::Frame::none()
                .fill(egui::Color32::from_rgb(248, 250, 252))
                .stroke(egui::Stroke::new(
                    1.0,
                    egui::Color32::from_rgb(226, 232, 240),
                ))
                .rounding(6.0)
                .inner_margin(12.0)
                .show(ui, |ui| {
                    ui.centered_and_justified(|ui| {
                        ui.label(
                            egui::RichText::new(
                                "ℹ️ Select a feature from the tree to edit properties.",
                            )
                            .weak()
                            .size(12.0),
                        );
                    });
                });
        }
    }
}

#[cfg(test)]
mod variable_value_editor_tests {
    use super::*;

    #[test]
    fn properties_window_targets_the_requested_feature() {
        let mut app = ZeroCadApp::new();

        app.open_feature_properties("origin");

        assert_eq!(app.selected_node_id.as_deref(), Some("origin"));
        assert_eq!(app.feature_properties_dialog.as_deref(), Some("origin"));
    }

    #[test]
    fn literal_input_updates_value_and_clears_expression() {
        let mut variable = Variable {
            name: "width".to_string(),
            value: 10.0,
            unit: Unit::Millimeter,
            expression: Some("base * 2".to_string()),
        };

        apply_variable_value_source(&mut variable, "42.8");

        assert_eq!(variable.value, 42.8);
        assert_eq!(variable.expression, None);
        assert_eq!(variable_value_source(&variable), "42.8");
    }

    #[test]
    fn expression_input_preserves_fallback_and_becomes_editor_source() {
        let mut variable = Variable {
            name: "width".to_string(),
            value: 10.0,
            unit: Unit::Millimeter,
            expression: None,
        };

        apply_variable_value_source(&mut variable, " blade_length / 2 ");

        assert_eq!(variable.value, 10.0);
        assert_eq!(variable.expression.as_deref(), Some("blade_length / 2"));
        assert_eq!(variable_value_source(&variable), "blade_length / 2");
    }

    #[test]
    fn inline_measurement_updates_the_driver_and_clears_its_formula() {
        let mut document = zerocad_core::ParametricGraph::new();
        document.add_feature(FeatureNode {
            id: "variables_1".to_string(),
            name: "Parameters".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![Variable {
                    name: "width".to_string(),
                    value: 1.0,
                    unit: Unit::Inch,
                    expression: Some("base_width * 2".to_string()),
                }],
            },
        });
        apply_driving_parameter_edit(
            &mut document,
            &DrivingParameterEdit {
                feature_id: "variables_1".to_string(),
                name: "width".to_string(),
                value: 2.5,
                unit: Unit::Inch,
            },
        )
        .unwrap();

        let variable = document
            .graph
            .node_weights()
            .find_map(|node| match &node.feature {
                FeatureType::VariableSet { variables } => variables.first(),
                _ => None,
            })
            .unwrap();
        assert_eq!(variable.value, 2.5);
        assert_eq!(variable.unit, Unit::Inch);
        assert_eq!(variable.expression, None);
        assert!((document.variable_map()["width"] - 63.5).abs() < 1.0e-9);
    }

    #[test]
    fn inspector_rows_follow_the_selected_features_references() {
        let mut document = zerocad_core::ParametricGraph::new();
        document.add_feature(FeatureNode {
            id: "variables_1".to_string(),
            name: "Parameters".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![
                    Variable {
                        name: "width".to_string(),
                        value: 25.0,
                        unit: Unit::Millimeter,
                        expression: None,
                    },
                    Variable {
                        name: "unused".to_string(),
                        value: 99.0,
                        unit: Unit::Millimeter,
                        expression: None,
                    },
                ],
            },
        });
        document.add_feature(FeatureNode {
            id: "extrude_2".to_string(),
            name: "Extrude".to_string(),
            feature: FeatureType::Extrude {
                depth: 25.0,
                region_indices: vec![],
                mode: ExtrudeMode::NewBody,
                target: None,
                depth_expr: Some("width / 2".to_string()),
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });

        let resolved = document.variable_map();
        let rows = driving_parameter_rows(&document, "extrude_2", &resolved);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].name, "width");
        assert_eq!(rows[0].value, 25.0);
    }
}
