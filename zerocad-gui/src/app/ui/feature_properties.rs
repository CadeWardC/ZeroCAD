use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_selected_feature_properties(&mut self, ui: &mut egui::Ui) {
        ui.add_space(15.0);
        ui.label(
            egui::RichText::new("Properties")
                .font(egui::FontId::proportional(14.0))
                .strong()
                .color(self.pal().text_strong), // Slate-900
        );
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(8.0);

        // Render dynamic sliders based on selected node's feature type
        if let Some(ref selected_id) = self.selected_node_id {
            let mut node_idx = None;
            for idx in self.graph.graph.node_indices() {
                if self.graph.graph[idx].id == *selected_id {
                    node_idx = Some(idx);
                    break;
                }
            }

            if let Some(idx) = node_idx {
                // Deferred action: extruding needs `&mut self`, but
                // `node` holds a mutable borrow of the graph below.
                let mut extrude_request: Option<String> = None;
                let mut modified = false;

                // Capture palette + unit + the variable map before
                // borrowing the graph mutably (so the Extrude panel can
                // show what an expression-driven depth resolves to).
                let pal = self.pal();
                let current_unit = self.current_unit;
                let var_map = self.graph.variable_map();
                let node = &mut self.graph.graph[idx];

                // Render inside a highly visual white inspector card
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(8.0)
                    .stroke(egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgb(226, 232, 240),
                    ))
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
                                    let eff = zerocad_core::effective_curves(
                                        curves,
                                        shapes,
                                        corner_mods,
                                        &var_map,
                                    );
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
                                    // Surface any variable-bound dimensions so the user knows
                                    // the sketch is parametric (editing is done by redrawing).
                                    let bound = sketch_variable_dims(shapes);
                                    if !bound.is_empty() {
                                        ui.add_space(2.0);
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "🔗 Variable dims: {}",
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
                                }
                                FeatureType::Extrude {
                                    depth,
                                    region_indices,
                                    mode,
                                    depth_expr,
                                } => {
                                    ui.horizontal(|ui| {
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
                                    ui.horizontal(|ui| {
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
                                    ui.horizontal(|ui| {
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
                                    // name field on top, then value + unit + delete.
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
                                                ui.add(
                                                    egui::TextEdit::singleline(&mut var.name)
                                                        .desired_width(f32::INFINITY)
                                                        .hint_text("name")
                                                        .font(egui::FontId::proportional(12.5)),
                                                );
                                                ui.add_space(6.0);
                                                ui.horizontal(|ui| {
                                                    ui.add(
                                                        egui::DragValue::new(&mut var.value)
                                                            .speed(0.1)
                                                            .min_decimals(0)
                                                            .max_decimals(3),
                                                    );
                                                    egui::ComboBox::from_id_source(("var_unit", i))
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
                                                            }
                                                        },
                                                    );
                                                });
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
                                    }
                                }
                            }
                        });
                    });

                if modified {
                    self.reevaluate_geometry();
                }

                if let Some(sketch_id) = extrude_request {
                    self.begin_extrude_whole_sketch(&sketch_id);
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
