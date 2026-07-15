use crate::*;

type BrowserRow = (String, String);
type BodyBrowserGroup = (String, String, Vec<BrowserRow>, Vec<BrowserRow>);

impl ZeroCadApp {
    pub(crate) fn draw_feature_tree(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("history_sidebar")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.label(
                    egui::RichText::new("Document Browser")
                        .font(egui::FontId::proportional(14.0))
                        .strong()
                        .color(self.pal().text_strong),
                );
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                let mut origin = None;
                let mut sketches = Vec::new();
                let mut datums = Vec::new();
                let mut variable_sets = Vec::new();
                let mut names = std::collections::BTreeMap::new();
                for node in self.graph.graph.node_weights() {
                    names.insert(node.id.clone(), node.name.clone());
                    let entry = (node.id.clone(), node.name.clone());
                    match node.feature {
                        FeatureType::Origin => origin = Some(entry),
                        FeatureType::Sketch { .. } => sketches.push(entry),
                        FeatureType::DatumPlane { .. }
                        | FeatureType::DatumAxis { .. }
                        | FeatureType::DatumPoint { .. } => datums.push(entry),
                        FeatureType::VariableSet { .. } => variable_sets.push(entry),
                        _ => {}
                    }
                }

                // The semantic body records are authoritative. Each group owns
                // one ordered feature timeline; runtime split outputs are merely
                // display children and never reconstruct ownership from id text.
                let body_groups: Vec<BodyBrowserGroup> = self
                    .graph
                    .semantics
                    .bodies
                    .values()
                    .map(|body| {
                        let timeline = body
                            .timeline
                            .iter()
                            .filter_map(|id| {
                                names
                                    .get(id.as_str())
                                    .map(|name| (id.to_string(), name.clone()))
                            })
                            .collect();
                        let mut outputs: Vec<BrowserRow> = self
                            .body_meshes
                            .iter()
                            .filter(|(id, _)| {
                                id != body.id.as_str()
                                    && zerocad_core::body_output_owner_id(id) == body.id.as_str()
                            })
                            .map(|(id, _)| (id.clone(), body_output_label(&body.name, id)))
                            .collect();
                        outputs.sort_by_key(|(id, _)| zerocad_core::body_output_index(id));
                        (body.id.to_string(), body.name.clone(), timeline, outputs)
                    })
                    .collect();

                let mut id_to_delete = None;
                let mut id_to_toggle = None;
                let mut id_to_suppress = None;
                let mut id_to_move = None;
                let mut id_to_add_var = None;
                let mut id_to_edit_sketch = None;
                let mut create_var_set = false;

                macro_rules! capture_action {
                    ($action:expr, $id:expr) => {
                        match $action {
                            RowAction::Delete => id_to_delete = Some($id.to_string()),
                            RowAction::ToggleVisibility => id_to_toggle = Some($id.to_string()),
                            RowAction::ToggleSuppression => id_to_suppress = Some($id.to_string()),
                            RowAction::MoveUp => id_to_move = Some(($id.to_string(), -1)),
                            RowAction::MoveDown => id_to_move = Some(($id.to_string(), 1)),
                            RowAction::AddVariable => id_to_add_var = Some($id.to_string()),
                            RowAction::EditSketch => id_to_edit_sketch = Some($id.to_string()),
                            RowAction::None => {}
                        }
                    };
                }

                egui::ScrollArea::vertical()
                    .id_salt("tree_scroll")
                    .max_height(360.0)
                    .show(ui, |ui| {
                        if let Some((id, name)) = &origin {
                            capture_action!(
                                self.feature_tree_row(ui, id, name, false, false, false, false),
                                id
                            );
                        }

                        egui::CollapsingHeader::new(section_label("Sketches", sketches.len()))
                            .default_open(true)
                            .show(ui, |ui| {
                                if sketches.is_empty() {
                                    ui.weak("No sketches yet — use Draw Sketch.");
                                }
                                for (id, name) in &sketches {
                                    capture_action!(
                                        self.feature_tree_row(
                                            ui,
                                            id,
                                            name,
                                            self.hidden_nodes.contains(id),
                                            false,
                                            true,
                                            false,
                                        ),
                                        id
                                    );
                                }
                            });

                        egui::CollapsingHeader::new(section_label("Bodies", body_groups.len()))
                            .default_open(true)
                            .show(ui, |ui| {
                                if body_groups.is_empty() {
                                    ui.weak("No bodies yet — add a primitive or extrude a sketch.");
                                }
                                for (body_id, body_name, timeline, outputs) in &body_groups {
                                    egui::CollapsingHeader::new(
                                        egui::RichText::new(body_name).strong(),
                                    )
                                    .id_salt(("semantic_body", body_id))
                                    .default_open(true)
                                    .show(ui, |ui| {
                                        for (id, name) in timeline {
                                            capture_action!(
                                                self.feature_tree_row(
                                                    ui,
                                                    id,
                                                    name,
                                                    self.hidden_nodes.contains(id),
                                                    false,
                                                    false,
                                                    true,
                                                ),
                                                id
                                            );
                                        }
                                        for (id, name) in outputs {
                                            capture_action!(
                                                self.feature_tree_row(
                                                    ui,
                                                    id,
                                                    name,
                                                    self.hidden_nodes.contains(body_id),
                                                    false,
                                                    false,
                                                    false,
                                                ),
                                                body_id
                                            );
                                        }
                                    });
                                }
                            });

                        if !datums.is_empty() {
                            egui::CollapsingHeader::new(section_label("Datums", datums.len()))
                                .default_open(true)
                                .show(ui, |ui| {
                                    for (id, name) in &datums {
                                        capture_action!(
                                            self.feature_tree_row(
                                                ui,
                                                id,
                                                name,
                                                self.hidden_nodes.contains(id),
                                                false,
                                                false,
                                                false,
                                            ),
                                            id
                                        );
                                    }
                                });
                        }

                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.label(section_label("Variable Sets", variable_sets.len()));
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui
                                        .small_button("+")
                                        .on_hover_text("New variable set")
                                        .clicked()
                                    {
                                        create_var_set = true;
                                    }
                                },
                            );
                        });
                        if variable_sets.is_empty() {
                            ui.weak("No variable sets yet — click + to add one.");
                        }
                        for (id, name) in &variable_sets {
                            capture_action!(
                                self.feature_tree_row(ui, id, name, false, true, false, false),
                                id
                            );
                        }
                    });

                if let Some(toggle_id) = id_to_toggle {
                    self.push_undo();
                    if !self.hidden_nodes.remove(&toggle_id) {
                        self.hidden_nodes.insert(toggle_id);
                    }
                    self.reevaluate_geometry();
                }
                if let Some(feature_id) = id_to_suppress {
                    let suppressed = self.graph.is_feature_suppressed(&feature_id);
                    self.push_undo();
                    if self.graph.set_feature_suppressed(&feature_id, !suppressed) {
                        self.status_msg = if suppressed {
                            format!("Resumed feature '{feature_id}'.")
                        } else {
                            format!("Suppressed feature '{feature_id}'.")
                        };
                        self.reevaluate_geometry();
                    }
                }
                if let Some((feature_id, offset)) = id_to_move {
                    self.push_undo();
                    if self.graph.move_feature_in_timeline(&feature_id, offset) {
                        self.selected_node_id = Some(feature_id.clone());
                        self.status_msg =
                            format!("Moved feature '{feature_id}' in its body timeline.");
                        self.reevaluate_geometry();
                    }
                }
                if let Some(del_id) = id_to_delete {
                    self.delete_node_by_id(&del_id);
                }
                if let Some(edit_id) = id_to_edit_sketch {
                    self.edit_sketch(&edit_id, ctx.input(|input| input.time));
                }
                if create_var_set {
                    self.push_undo();
                    let id = format!("varset_{}", self.next_id());
                    self.graph.add_feature(FeatureNode {
                        id: id.clone(),
                        name: self.next_variable_set_name(),
                        feature: FeatureType::VariableSet {
                            variables: Vec::new(),
                        },
                    });
                    self.selected_node_id = Some(id);
                }
                if let Some(set_id) = id_to_add_var {
                    self.push_undo();
                    let unit = self.current_unit;
                    for idx in self.graph.graph.node_indices() {
                        if self.graph.graph[idx].id == set_id {
                            if let FeatureType::VariableSet { variables } =
                                &mut self.graph.graph[idx].feature
                            {
                                variables.push(Variable::new(
                                    format!("var{}", variables.len() + 1),
                                    unit,
                                ));
                            }
                            break;
                        }
                    }
                    self.selected_node_id = Some(set_id);
                }

                self.draw_selected_feature_properties(ui);
            });
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn feature_tree_row(
        &mut self,
        ui: &mut egui::Ui,
        id: &str,
        name: &str,
        hidden: bool,
        is_var_set: bool,
        is_sketch: bool,
        allow_reorder: bool,
    ) -> RowAction {
        let mut action = RowAction::None;
        let owner_id = zerocad_core::body_output_owner_id(id).to_string();
        let suppressed = self.graph.is_feature_suppressed(&owner_id);
        let is_live_body = self.body_meshes.iter().any(|(body_id, _)| body_id == id);
        let unresolved = self.unresolved_features.get(&owner_id).cloned();

        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);
            if let Some(reason) = &unresolved {
                ui.label(egui::RichText::new("⚠").color(egui::Color32::from_rgb(220, 38, 38)))
                    .on_hover_text(format!("Unresolved: {reason}"));
            }
            if id != "origin" {
                let icon = if hidden {
                    icons::Icon::EyeClosed
                } else {
                    icons::Icon::EyeOpen
                };
                if icon
                    .icon_button(
                        ui,
                        egui::Color32::TRANSPARENT,
                        egui::Color32::from_rgb(226, 232, 240),
                        if hidden {
                            egui::Color32::from_rgb(148, 163, 184)
                        } else {
                            self.pal().text_body
                        },
                    )
                    .on_hover_text(if hidden {
                        "Show in 3D View"
                    } else {
                        "Hide in 3D View"
                    })
                    .clicked()
                {
                    action = RowAction::ToggleVisibility;
                }
            } else {
                ui.add_space(24.0);
            }

            let selected = if is_live_body {
                self.selected_body
                    .contains(&(id.to_string(), BodyPick::Whole))
                    || (self.selected_body.is_empty()
                        && zerocad_core::body_output_index(id) == 0
                        && self.selected_node_id.as_deref() == Some(owner_id.as_str()))
            } else {
                self.selected_node_id.as_deref() == Some(owner_id.as_str())
            };

            if self.renaming_node.as_deref() == Some(id) {
                let response = ui.add(
                    egui::TextEdit::singleline(&mut self.rename_buffer)
                        .desired_width(f32::INFINITY),
                );
                if self.rename_focus_pending {
                    response.request_focus();
                    self.rename_focus_pending = false;
                }
                if ui.input(|input| input.key_pressed(egui::Key::Escape)) {
                    self.renaming_node = None;
                } else if response.lost_focus() {
                    let new_name = self.rename_buffer.trim();
                    if !new_name.is_empty() {
                        if let Some(node) = self
                            .graph
                            .graph
                            .node_weights_mut()
                            .find(|node| node.id == owner_id)
                        {
                            node.name = new_name.to_owned();
                        }
                        if let Some(body) = self
                            .graph
                            .semantics
                            .bodies
                            .get_mut(&zerocad_core::BodyId::from(owner_id.as_str()))
                        {
                            body.name = new_name.to_owned();
                        }
                    }
                    self.renaming_node = None;
                }
                return;
            }

            let color = if selected {
                egui::Color32::from_rgb(29, 78, 216)
            } else if hidden || suppressed {
                self.pal().text_faint
            } else {
                self.pal().text_strong
            };
            let mut text = egui::RichText::new(name).color(color).size(13.0);
            if selected {
                text = text.strong();
            }
            if suppressed {
                text = text.strikethrough();
            }
            let response = ui.selectable_label(selected, text);
            if response.double_clicked() {
                self.renaming_node = Some(id.to_owned());
                self.rename_buffer = name.to_owned();
                self.rename_focus_pending = true;
            } else if response.clicked() {
                self.selected_node_id = Some(owner_id.clone());
                if is_live_body {
                    self.selected_faces.clear();
                    self.selected_edges.clear();
                    self.selected_body.clear();
                    self.selected_body.insert((id.to_owned(), BodyPick::Whole));
                }
            }

            response.context_menu(|ui| {
                if ui.button("Rename").clicked() {
                    self.renaming_node = Some(id.to_owned());
                    self.rename_buffer = name.to_owned();
                    self.rename_focus_pending = true;
                    ui.close_menu();
                }
                if allow_reorder {
                    let feature_id = zerocad_core::FeatureId::from(owner_id.as_str());
                    if let Some(body) = self
                        .graph
                        .semantics
                        .bodies
                        .values()
                        .find(|body| body.timeline.iter().any(|member| member == &feature_id))
                    {
                        let position = body
                            .timeline
                            .iter()
                            .position(|member| member == &feature_id)
                            .unwrap();
                        if ui
                            .add_enabled(position > 0, egui::Button::new("Move Up"))
                            .clicked()
                        {
                            action = RowAction::MoveUp;
                            ui.close_menu();
                        }
                        if ui
                            .add_enabled(
                                position + 1 < body.timeline.len(),
                                egui::Button::new("Move Down"),
                            )
                            .clicked()
                        {
                            action = RowAction::MoveDown;
                            ui.close_menu();
                        }
                    }
                }
                if id != "origin"
                    && ui
                        .button(if suppressed {
                            "Resume Feature"
                        } else {
                            "Suppress Feature"
                        })
                        .clicked()
                {
                    action = RowAction::ToggleSuppression;
                    ui.close_menu();
                }
                if is_sketch && ui.button("Edit Sketch").clicked() {
                    action = RowAction::EditSketch;
                    ui.close_menu();
                }
                if is_var_set && ui.button("Add Variable").clicked() {
                    action = RowAction::AddVariable;
                    ui.close_menu();
                }
                if id != "origin" && ui.button("Delete Feature").clicked() {
                    action = RowAction::Delete;
                    ui.close_menu();
                }
            });
        });
        action
    }
}

fn section_label(name: &str, count: usize) -> egui::RichText {
    egui::RichText::new(format!("{name} ({count})"))
        .font(egui::FontId::proportional(12.5))
        .strong()
}
