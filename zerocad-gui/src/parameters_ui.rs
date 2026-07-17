use crate::*;

#[derive(Debug, Clone)]
struct ParameterDraft {
    feature_id: Option<String>,
    original_name: String,
    name: String,
    value: f64,
    unit: Unit,
    expression: String,
}

#[derive(Debug, Clone)]
pub(crate) struct ParametersDialog {
    rows: Vec<ParameterDraft>,
    default_feature: Option<String>,
    error: Option<String>,
}

impl ParametersDialog {
    fn from_document(document: &zerocad_core::ParametricGraph) -> Self {
        let mut rows = Vec::new();
        let mut default_feature = None;
        for index in document.graph.node_indices() {
            let node = &document.graph[index];
            if let FeatureType::VariableSet { variables } = &node.feature {
                default_feature.get_or_insert_with(|| node.id.clone());
                for variable in variables {
                    rows.push(ParameterDraft {
                        feature_id: Some(node.id.clone()),
                        original_name: variable.name.clone(),
                        name: variable.name.clone(),
                        value: variable.value,
                        unit: variable.unit,
                        expression: variable.expression.clone().unwrap_or_default(),
                    });
                }
            }
        }
        Self {
            rows,
            default_feature,
            error: None,
        }
    }
}

impl ZeroCadApp {
    pub(crate) fn open_parameters_dialog(&mut self) {
        self.parameters_dialog = Some(ParametersDialog::from_document(&self.document));
    }

    pub(crate) fn show_parameters_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.parameters_dialog.take() else {
            return;
        };
        let current = self.document.resolve_variables();
        let mut apply = false;
        let mut close = false;
        let mut remove = None;
        egui::Window::new("Parameters")
            .id(egui::Id::new("parameters-table"))
            .default_size(egui::vec2(760.0, 360.0))
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "Named values are stored with units. Expressions resolve in millimeters and may depend on other parameters.",
                    )
                    .small()
                    .weak(),
                );
                ui.add_space(6.0);
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        egui::Grid::new("parameters-grid")
                            .num_columns(7)
                            .striped(true)
                            .spacing([8.0, 6.0])
                            .show(ui, |ui| {
                                ui.strong("Name");
                                ui.strong("Value");
                                ui.strong("Unit");
                                ui.strong("Expression");
                                ui.strong("Dependencies");
                                ui.strong("Resolved");
                                ui.label("");
                                ui.end_row();

                                for (index, row) in dialog.rows.iter_mut().enumerate() {
                                    ui.add(
                                        egui::TextEdit::singleline(&mut row.name)
                                            .desired_width(100.0),
                                    );
                                    ui.add(
                                        egui::DragValue::new(&mut row.value)
                                            .speed(0.1)
                                            .max_decimals(6),
                                    );
                                    egui::ComboBox::from_id_salt(("parameter-unit", index))
                                        .selected_text(row.unit.suffix())
                                        .show_ui(ui, |ui| {
                                            for unit in [Unit::Millimeter, Unit::Inch, Unit::Meter] {
                                                ui.selectable_value(
                                                    &mut row.unit,
                                                    unit,
                                                    unit.suffix(),
                                                );
                                            }
                                        });
                                    ui.add(
                                        egui::TextEdit::singleline(&mut row.expression)
                                            .desired_width(170.0)
                                            .hint_text("optional, e.g. width / 2"),
                                    );
                                    let dependencies = zerocad_core::expr::identifiers(
                                        &row.expression,
                                    )
                                    .map(|names| names.join(", "))
                                    .unwrap_or_else(|_| "invalid".to_string());
                                    ui.label(egui::RichText::new(dependencies).small());
                                    let resolved = current
                                        .values
                                        .get(&row.original_name)
                                        .or_else(|| current.values.get(&row.name))
                                        .map(|value| format!("{value:.6} mm"))
                                        .unwrap_or_else(|| "—".to_string());
                                    ui.label(resolved);
                                    if ui.small_button("Remove").clicked() {
                                        remove = Some(index);
                                    }
                                    ui.end_row();
                                }
                            });
                    });
                if let Some(index) = remove {
                    dialog.rows.remove(index);
                }
                if let Some(error) = &dialog.error {
                    ui.colored_label(egui::Color32::from_rgb(220, 38, 38), error);
                }
                for diagnostic in &current.diagnostics {
                    ui.colored_label(
                        egui::Color32::from_rgb(180, 83, 9),
                        format!("{}: {}", diagnostic.name, diagnostic.message),
                    );
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("+ Add parameter").clicked() {
                        let suffix = dialog.rows.len() + 1;
                        dialog.rows.push(ParameterDraft {
                            feature_id: dialog.default_feature.clone(),
                            original_name: String::new(),
                            name: format!("parameter_{suffix}"),
                            value: 0.0,
                            unit: self.current_unit,
                            expression: String::new(),
                        });
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Cancel").clicked() {
                            close = true;
                        }
                        if ui.button("Apply").clicked() {
                            apply = true;
                        }
                    });
                });
            });

        if apply {
            match self.apply_parameter_drafts(&dialog) {
                Ok(()) => close = true,
                Err(error) => dialog.error = Some(error),
            }
        }
        if !close {
            self.parameters_dialog = Some(dialog);
        }
    }

    fn apply_parameter_drafts(&mut self, dialog: &ParametersDialog) -> Result<(), String> {
        let mut names = std::collections::HashSet::new();
        for row in &dialog.rows {
            let name = row.name.trim();
            if !zerocad_core::expr::is_valid_identifier(name) {
                return Err(format!("'{name}' is not a valid parameter name"));
            }
            if !names.insert(name.to_string()) {
                return Err(format!("parameter name '{name}' is duplicated"));
            }
            if !row.value.is_finite() {
                return Err(format!(
                    "parameter '{name}' has a non-finite fallback value"
                ));
            }
        }

        let mut candidate = self.document.clone();
        let renames: Vec<(String, String)> = dialog
            .rows
            .iter()
            .filter(|row| !row.original_name.is_empty() && row.original_name != row.name.trim())
            .map(|row| (row.original_name.clone(), row.name.trim().to_string()))
            .collect();
        let mut temporary = Vec::new();
        for (index, (old, _)) in renames.iter().enumerate() {
            let mut name = format!("__zcad_parameter_rename_{index}");
            while candidate.variable_map().contains_key(&name) {
                name.push('_');
            }
            candidate.rename_variable(old, &name)?;
            temporary.push(name);
        }
        for ((_, new), temporary) in renames.iter().zip(&temporary) {
            candidate.rename_variable(temporary, new)?;
        }

        for node in candidate.graph.node_weights_mut() {
            if let FeatureType::VariableSet { variables } = &mut node.feature {
                variables.clear();
            }
        }
        let needs_new_set = dialog.rows.iter().any(|row| row.feature_id.is_none());
        let proposed_id = format!("variables_{}", self.id_counter);
        if needs_new_set {
            candidate.add_feature(FeatureNode {
                id: proposed_id.clone(),
                name: "Parameters".to_string(),
                feature: FeatureType::VariableSet {
                    variables: Vec::new(),
                },
            });
        }
        for row in &dialog.rows {
            let feature_id = row.feature_id.as_deref().unwrap_or(&proposed_id);
            let node = candidate
                .graph
                .node_weights_mut()
                .find(|node| node.id == feature_id)
                .ok_or_else(|| format!("parameter container '{feature_id}' no longer exists"))?;
            let FeatureType::VariableSet { variables } = &mut node.feature else {
                return Err(format!(
                    "feature '{feature_id}' is not a parameter container"
                ));
            };
            variables.push(Variable {
                name: row.name.trim().to_string(),
                value: row.value,
                unit: row.unit,
                expression: (!row.expression.trim().is_empty())
                    .then(|| row.expression.trim().to_string()),
            });
        }
        let resolution = candidate.resolve_variables();
        if !resolution.diagnostics.is_empty() {
            return Err(resolution
                .diagnostics
                .iter()
                .map(|diagnostic| format!("{}: {}", diagnostic.name, diagnostic.message))
                .collect::<Vec<_>>()
                .join("; "));
        }
        self.push_undo();
        self.document = candidate;
        if needs_new_set {
            self.id_counter += 1;
        }
        self.reevaluate_geometry();
        self.status_msg = format!("Updated {} parameter(s).", dialog.rows.len());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dialog_apply_rejects_cycles_atomically() {
        let mut app = ZeroCadApp::new();
        let before =
            serde_json::to_vec(&(app.document.evaluator_graph(), &app.document.state)).unwrap();
        let dialog = ParametersDialog {
            rows: vec![
                ParameterDraft {
                    feature_id: None,
                    original_name: String::new(),
                    name: "a".to_string(),
                    value: 1.0,
                    unit: Unit::Millimeter,
                    expression: "b".to_string(),
                },
                ParameterDraft {
                    feature_id: None,
                    original_name: String::new(),
                    name: "b".to_string(),
                    value: 1.0,
                    unit: Unit::Millimeter,
                    expression: "a".to_string(),
                },
            ],
            default_feature: None,
            error: None,
        };
        assert!(app.apply_parameter_drafts(&dialog).is_err());
        assert_eq!(
            serde_json::to_vec(&(app.document.evaluator_graph(), &app.document.state,)).unwrap(),
            before
        );
    }
}
