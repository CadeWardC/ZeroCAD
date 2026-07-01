use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_feature_tree(&mut self, ctx: &egui::Context) {
        // LEFT PANEL: History Tree & Feature Properties
        egui::SidePanel::left("history_sidebar")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |ui| {
                ui.vertical(|ui| {
                    ui.add_space(8.0);
                    ui.label(
                        egui::RichText::new("Document Browser")
                            .font(egui::FontId::proportional(14.0))
                            .strong()
                            .color(self.pal().text_strong), // Slate-900
                    );
                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(6.0);

                    // Bucket features into Sketches (2D objects) and Bodies
                    // (solids: boxes, cylinders, extrudes). The Origin is shown
                    // on its own so it can't be mistaken for either group.
                    let mut origin: Option<(String, String)> = None;
                    let mut sketches: Vec<(String, String)> = Vec::new();
                    let mut bodies: Vec<(String, String)> = Vec::new();
                    let mut operations: Vec<(String, String)> = Vec::new();
                    let mut variable_sets: Vec<(String, String)> = Vec::new();
                    for idx in self.graph.graph.node_indices() {
                        let node = &self.graph.graph[idx];
                        let entry = (node.id.clone(), node.name.clone());
                        match node.feature {
                            FeatureType::Origin => origin = Some(entry),
                            FeatureType::Sketch { .. } => sketches.push(entry),
                            FeatureType::Box { .. }
                            | FeatureType::Cylinder { .. }
                            | FeatureType::Extrude {
                                mode: ExtrudeMode::NewBody,
                                ..
                            } => bodies.push(entry),
                            FeatureType::Extrude { .. } | FeatureType::EdgeMod { .. } => {
                                operations.push(entry)
                            }
                            FeatureType::VariableSet { .. } => variable_sets.push(entry),
                        }
                    }

                    let mut id_to_delete: Option<String> = None;
                    let mut id_to_toggle: Option<String> = None;
                    let mut id_to_add_var: Option<String> = None;
                    let mut create_var_set = false;

                    egui::ScrollArea::vertical()
                        .id_source("tree_scroll")
                        .max_height(300.0)
                        .show(ui, |ui| {
                            if let Some((id, name)) = &origin {
                                let hidden = self.hidden_nodes.contains(id);
                                match self.feature_tree_row(ui, id, name, hidden, false) {
                                    RowAction::Delete => id_to_delete = Some(id.clone()),
                                    RowAction::ToggleVisibility => id_to_toggle = Some(id.clone()),
                                    RowAction::None => {}
                                    RowAction::AddVariable => {}
                                }
                            }

                            egui::CollapsingHeader::new(
                                egui::RichText::new(format!("Sketches ({})", sketches.len()))
                                    .font(egui::FontId::proportional(12.5))
                                    .strong()
                                    .color(self.pal().text_body), // Slate-600
                            )
                            .default_open(true)
                            .show(ui, |ui| {
                                if sketches.is_empty() {
                                    ui.weak("No sketches yet — use Draw Sketch.");
                                }
                                for (id, name) in &sketches {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => {
                                            id_to_toggle = Some(id.clone())
                                        }
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                            egui::CollapsingHeader::new(
                                egui::RichText::new(format!("Bodies ({})", bodies.len()))
                                    .font(egui::FontId::proportional(12.5))
                                    .strong()
                                    .color(self.pal().text_body), // Slate-600
                            )
                            .default_open(true)
                            .show(ui, |ui| {
                                if bodies.is_empty() {
                                    ui.weak("No bodies yet — add a primitive or Extrude a sketch.");
                                }
                                for (id, name) in &bodies {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => {
                                            id_to_toggle = Some(id.clone())
                                        }
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                            egui::CollapsingHeader::new(
                                egui::RichText::new(format!("Operations ({})", operations.len()))
                                    .font(egui::FontId::proportional(12.5))
                                    .strong()
                                    .color(self.pal().text_body), // Slate-600
                            )
                            .default_open(true)
                            .show(ui, |ui| {
                                if operations.is_empty() {
                                    ui.weak("No body operations yet.");
                                }
                                for (id, name) in &operations {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => {
                                            id_to_toggle = Some(id.clone())
                                        }
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                            // Variable Sets: a section title with a "+" on the right
                            // to create a new set. Each set lives below as a row whose
                            // right-click menu can add variables.
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new(format!(
                                        "Variable Sets ({})",
                                        variable_sets.len()
                                    ))
                                    .font(egui::FontId::proportional(12.5))
                                    .strong()
                                    .color(self.pal().text_body), // Slate-600
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui
                                            .add(
                                                egui::Button::new(
                                                    egui::RichText::new("➕").size(12.0),
                                                )
                                                .small(),
                                            )
                                            .on_hover_text("New variable set")
                                            .clicked()
                                        {
                                            create_var_set = true;
                                        }
                                    },
                                );
                            });
                            if variable_sets.is_empty() {
                                ui.weak("No variable sets yet — click ➕ to add one.");
                            }
                            for (id, name) in &variable_sets {
                                match self.feature_tree_row(ui, id, name, false, true) {
                                    RowAction::Delete => id_to_delete = Some(id.clone()),
                                    RowAction::AddVariable => id_to_add_var = Some(id.clone()),
                                    RowAction::ToggleVisibility => {}
                                    RowAction::None => {}
                                }
                            }
                        });

                    if let Some(toggle_id) = id_to_toggle {
                        if !self.hidden_nodes.remove(&toggle_id) {
                            self.hidden_nodes.insert(toggle_id);
                        }
                        // Bodies are baked into the mesh, so re-evaluate to reflect
                        // the change; sketches just toggle in the draw pass.
                        self.reevaluate_geometry();
                    }

                    if let Some(del_id) = id_to_delete {
                        // Resolve the id to its current graph index, then remove.
                        let mut target = None;
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == del_id {
                                target = Some(idx);
                                break;
                            }
                        }
                        if let Some(idx) = target {
                            self.push_undo();
                            self.graph.graph.remove_node(idx);
                            if self.selected_node_id.as_deref() == Some(del_id.as_str()) {
                                self.selected_node_id = None;
                            }
                            self.selected_faces.retain(|(sid, _)| sid != &del_id);
                            self.selected_edges.retain(|(sid, _)| sid != &del_id);
                            self.selected_body.retain(|(nid, _)| nid != &del_id);
                            self.hidden_nodes.remove(&del_id);
                            self.reevaluate_geometry();
                        }
                    }

                    // Create a new, empty variable set and select it so the user
                    // can rename it (Label field) and start adding variables.
                    if create_var_set {
                        let id = format!("varset_{}", self.next_id());
                        let name = self.next_variable_set_name();
                        self.graph.add_feature(FeatureNode {
                            id: id.clone(),
                            name,
                            feature: FeatureType::VariableSet {
                                variables: Vec::new(),
                            },
                        });
                        self.selected_node_id = Some(id);
                    }

                    // Append a fresh variable to the targeted set (from a row's
                    // right-click "Add Variable").
                    if let Some(set_id) = id_to_add_var {
                        let unit = self.current_unit;
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == set_id {
                                if let FeatureType::VariableSet { variables } =
                                    &mut self.graph.graph[idx].feature
                                {
                                    let n = variables.len() + 1;
                                    variables.push(Variable::new(format!("var{}", n), unit));
                                }
                                break;
                            }
                        }
                        self.selected_node_id = Some(set_id);
                    }

                    self.draw_selected_feature_properties(ui);
                });
            });
    }

    /// Render one row in the feature tree. Returns what the user did (selecting
    /// is handled inline). `hidden` controls the eye icon.
    pub(crate) fn feature_tree_row(
        &mut self,
        ui: &mut egui::Ui,
        id: &str,
        name: &str,
        hidden: bool,
        is_var_set: bool,
    ) -> RowAction {
        let mut action = RowAction::None;
        // A feature whose reference/boolean didn't resolve on the last rebuild is
        // flagged here so it's visible in the tree, not just a global warning.
        let unresolved = self.unresolved_features.get(id).cloned();
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

            if let Some(reason) = &unresolved {
                ui.label(
                    egui::RichText::new("⚠")
                        .color(egui::Color32::from_rgb(220, 38, 38))
                        .size(13.0),
                )
                .on_hover_text(format!("Unresolved: {reason}"));
            }

            if id != "origin" {
                let eye_color = if hidden {
                    egui::Color32::from_rgb(148, 163, 184) // muted slate for hidden
                } else {
                    self.pal().text_body
                };
                let icon = if hidden {
                    icons::Icon::EyeClosed
                } else {
                    icons::Icon::EyeOpen
                };
                let eye_btn = icon.icon_button(
                    ui,
                    egui::Color32::TRANSPARENT,
                    egui::Color32::from_rgb(226, 232, 240),
                    eye_color,
                );
                if eye_btn
                    .on_hover_text(if hidden {
                        "Show node in 3D View"
                    } else {
                        "Hide node from 3D View"
                    })
                    .clicked()
                {
                    action = RowAction::ToggleVisibility;
                }
            } else {
                // Spacer to align with eye button
                ui.add_space(24.0);
            }

            let is_selected = self.selected_node_id.as_deref() == Some(id);

            // Inline rename: a text field replaces the label for the node being
            // renamed. Commits on Enter / click-away, cancels on Escape.
            if self.renaming_node.as_deref() == Some(id) {
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.rename_buffer)
                        .desired_width(f32::INFINITY)
                        .font(egui::FontId::proportional(13.0)),
                );
                if self.rename_focus_pending {
                    resp.request_focus();
                    self.rename_focus_pending = false;
                }
                let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
                if escaped {
                    self.renaming_node = None;
                } else if resp.lost_focus() {
                    let new_name = self.rename_buffer.trim().to_string();
                    if !new_name.is_empty() {
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == id {
                                self.graph.graph[idx].name = new_name.clone();
                                break;
                            }
                        }
                    }
                    self.renaming_node = None;
                }
                return; // skip the normal label + context menu this frame
            }

            let label_color = if is_selected {
                egui::Color32::from_rgb(29, 78, 216) // deep blue-700
            } else if hidden {
                self.pal().text_faint // muted slate-400
            } else {
                self.pal().text_strong // dark slate-800
            };

            let rich_text = egui::RichText::new(name).color(label_color).size(13.0);

            let rich_text = if is_selected {
                rich_text.strong()
            } else {
                rich_text
            };

            let response = ui.selectable_label(is_selected, rich_text);
            if response.double_clicked() {
                // Double-click starts an inline rename.
                self.renaming_node = Some(id.to_string());
                self.rename_buffer = name.to_string();
                self.rename_focus_pending = true;
            } else if response.clicked() {
                self.selected_node_id = Some(id.to_string());
                log::info!("Selected browser node: {}", id);
            }

            response.context_menu(|ui| {
                let rename_btn = icons::Icon::Sketch.labeled_button(
                    ui,
                    "Rename",
                    egui::Color32::from_rgb(248, 250, 252),
                    egui::Color32::from_rgb(241, 245, 249),
                    self.pal().text_body,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)),
                );
                if rename_btn.clicked() {
                    self.renaming_node = Some(id.to_string());
                    self.rename_buffer = name.to_string();
                    self.rename_focus_pending = true;
                    ui.close_menu();
                }
                if is_var_set {
                    let add_btn = icons::Icon::Sketch.labeled_button(
                        ui,
                        "Add Variable",
                        egui::Color32::from_rgb(239, 246, 255),
                        egui::Color32::from_rgb(219, 234, 254),
                        egui::Color32::from_rgb(29, 78, 216),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(191, 219, 254)),
                    );
                    if add_btn.clicked() {
                        action = RowAction::AddVariable;
                        ui.close_menu();
                        log::info!("Requested add variable to set: {}", id);
                    }
                }
                if id != "origin" {
                    let del_btn = icons::Icon::Trash.labeled_button(
                        ui,
                        "Delete Feature",
                        egui::Color32::from_rgb(254, 242, 242),
                        egui::Color32::from_rgb(254, 226, 226),
                        egui::Color32::from_rgb(185, 28, 28),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(252, 165, 165)),
                    );
                    if del_btn.clicked() {
                        action = RowAction::Delete;
                        ui.close_menu();
                        log::info!("Requested delete of browser node: {}", id);
                    }
                }
            });
        });
        action
    }
}
