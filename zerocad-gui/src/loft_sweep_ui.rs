//! Loft & Sweep tool UIs.
//!
//! Loft: select two or more sketch faces (across sketches), then commit — the
//! sections are ordered by sketch creation order. Sweep: select one profile
//! face, then pick the path sketch from a dropdown.

use crate::*;
use zerocad_core::LoftSurfaceMode;

/// Creation order key from a node id like `sketch_12` (the trailing number).
/// Ids without a trailing number sort first.
fn creation_key_of(id: &str) -> usize {
    id.rsplit('_')
        .next()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0)
}

/// State of the in-progress sweep tool (profile chosen; picking the path).
#[derive(Debug, Clone)]
pub(crate) struct SweepOp {
    pub(crate) profile_sketch: String,
    pub(crate) profile_region: usize,
    pub(crate) path_sketch: Option<String>,
    pub(crate) guide_sketch: Option<String>,
    pub(crate) profile_entity: Option<zerocad_core::sketch::EntityId>,
    pub(crate) profile_parameter: f32,
    pub(crate) mode: ExtrudeMode,
    pub(crate) total_twist_text: String,
}

impl ZeroCadApp {
    /// The ordered `(sketch_id, region_index)` sections a loft would use:
    /// every selected sketch face, grouped and ordered by sketch creation key.
    /// `None` unless at least two distinct sketches are represented.
    pub(crate) fn loft_sections(&self) -> Option<Vec<(String, usize)>> {
        let mut by_sketch: std::collections::BTreeMap<usize, Vec<(String, usize)>> =
            std::collections::BTreeMap::new();
        for (sid, ri) in &self.selected_faces {
            let key = creation_key_of(sid);
            by_sketch.entry(key).or_default().push((sid.clone(), *ri));
        }
        if by_sketch.len() < 2 {
            return None;
        }
        // One section per sketch (the first selected region on it), in
        // creation order.
        let sections: Vec<(String, usize)> = by_sketch
            .into_values()
            .filter_map(|mut v| {
                v.sort_by_key(|(_, ri)| *ri);
                v.into_iter().next()
            })
            .collect();
        (sections.len() >= 2).then_some(sections)
    }

    pub(crate) fn commit_loft(&mut self, sections: Vec<(String, usize)>) {
        self.push_undo();
        let n = self.next_id();
        let id = format!("loft_{n}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Loft {n}"),
            feature: FeatureType::Loft {
                sections: sections.clone(),
                surface_mode: LoftSurfaceMode::Ruled,
                mode: ExtrudeMode::NewBody,
                target: None,
            },
        });
        for (sketch_id, _) in &sections {
            self.document.add_dependency(sketch_id, &id);
            self.hidden_nodes.insert(sketch_id.clone());
        }
        self.selected_faces.clear();
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = "Loft created.".to_string();
    }

    /// The single selected profile face for a sweep, if exactly one is chosen.
    pub(crate) fn sweep_profile_candidate(&self) -> Option<(String, usize)> {
        (self.selected_faces.len() == 1)
            .then(|| self.selected_faces.iter().next().cloned())
            .flatten()
    }

    pub(crate) fn begin_sweep(&mut self, profile_sketch: String, profile_region: usize) {
        if self.sweep_op.is_some() {
            return;
        }
        self.sweep_op = Some(SweepOp {
            profile_sketch,
            profile_region,
            path_sketch: None,
            guide_sketch: None,
            profile_entity: None,
            profile_parameter: 0.0,
            mode: ExtrudeMode::NewBody,
            total_twist_text: "0".to_string(),
        });
        self.status_msg = "Sweep: choose the path sketch, then OK.".to_string();
    }

    /// The floating Sweep dialog (drawn every frame while an op is active).
    pub(crate) fn show_sweep_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.sweep_op.clone() else {
            return;
        };
        // Candidate path sketches: every sketch except the profile's.
        let sketches: Vec<(String, String)> = self
            .document
            .graph
            .node_indices()
            .filter_map(|i| {
                let node = &self.document.graph[i];
                (matches!(node.feature, FeatureType::Sketch { .. }) && node.id != op.profile_sketch)
                    .then(|| (node.id.clone(), node.name.clone()))
            })
            .collect();
        let profile_entities: Vec<zerocad_core::sketch::EntityId> = self
            .document
            .graph
            .node_indices()
            .find_map(|index| {
                let node = &self.document.graph[index];
                if node.id != op.profile_sketch {
                    return None;
                }
                let FeatureType::Sketch {
                    shapes,
                    entity_ids,
                    solver,
                    ..
                } = &node.feature
                else {
                    return None;
                };
                let mut ids = zerocad_core::sketch::effective_shape_ids(shapes.len(), entity_ids);
                if let Some(model) = solver {
                    ids.extend(
                        model
                            .entities
                            .iter()
                            .filter(|entity| !model.construction.contains(&entity.id()))
                            .map(zerocad_core::sketch::SketchEntity::id),
                    );
                }
                ids.sort_by_key(|entity| entity.0);
                ids.dedup();
                Some(ids)
            })
            .unwrap_or_default();

        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        let variables = self.visible_variable_map();
        egui::Area::new(egui::Id::new("sweep_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(240.0);
                    ui.label(
                        egui::RichText::new("Sweep")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        ui.label("Path sketch");
                        let selected = op_new
                            .path_sketch
                            .as_ref()
                            .and_then(|id| sketches.iter().find(|(sid, _)| sid == id))
                            .map(|(_, name)| name.clone())
                            .unwrap_or_else(|| "— pick —".to_string());
                        egui::ComboBox::from_id_salt("sweep_path")
                            .selected_text(selected)
                            .show_ui(ui, |ui| {
                                for (sid, name) in &sketches {
                                    if op_new.guide_sketch.as_deref() != Some(sid) {
                                        ui.selectable_value(
                                            &mut op_new.path_sketch,
                                            Some(sid.clone()),
                                            name,
                                        );
                                    }
                                }
                            });
                    });
                    if sketches.is_empty() {
                        ui.label(
                            egui::RichText::new("Draw a second sketch for the path first.")
                                .size(11.0)
                                .color(self.pal().text_faint),
                        );
                    }
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("Guide sketch");
                        let selected = op_new
                            .guide_sketch
                            .as_ref()
                            .and_then(|id| sketches.iter().find(|(sid, _)| sid == id))
                            .map(|(_, name)| name.clone())
                            .unwrap_or_else(|| "None".to_string());
                        egui::ComboBox::from_id_salt("sweep_guide")
                            .selected_text(selected)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut op_new.guide_sketch, None, "None");
                                for (sid, name) in &sketches {
                                    if op_new.path_sketch.as_deref() != Some(sid) {
                                        ui.selectable_value(
                                            &mut op_new.guide_sketch,
                                            Some(sid.clone()),
                                            name,
                                        );
                                    }
                                }
                            });
                    });
                    if op_new.guide_sketch.is_some() {
                        ui.horizontal(|ui| {
                            ui.label("Profile anchor");
                            let selected = op_new
                                .profile_entity
                                .and_then(|entity| {
                                    profile_entities
                                        .iter()
                                        .position(|candidate| *candidate == entity)
                                })
                                .map(|index| format!("Profile curve {}", index + 1))
                                .unwrap_or_else(|| "â€” pick â€”".to_string());
                            egui::ComboBox::from_id_salt("sweep_profile_anchor")
                                .selected_text(selected)
                                .show_ui(ui, |ui| {
                                    for (index, entity) in profile_entities.iter().enumerate() {
                                        ui.selectable_value(
                                            &mut op_new.profile_entity,
                                            Some(*entity),
                                            format!("Profile curve {}", index + 1),
                                        );
                                    }
                                });
                            ui.add(
                                egui::DragValue::new(&mut op_new.profile_parameter)
                                    .range(0.0..=1.0)
                                    .speed(0.01),
                            );
                        });
                        ui.label(
                            egui::RichText::new(
                                "The anchor follows the guide; explicit twist is disabled.",
                            )
                            .size(11.0)
                            .color(self.pal().text_faint),
                        );
                    }
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("Total twist");
                        ui.add_enabled(
                            op_new.guide_sketch.is_none(),
                            egui::TextEdit::singleline(&mut op_new.total_twist_text)
                                .desired_width(70.0),
                        );
                        ui.label("°");
                        crate::expr::evaluation_hint(ui, &op_new.total_twist_text, &variables, "°");
                    });
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        for (m, label) in [
                            (ExtrudeMode::NewBody, "New Body"),
                            (ExtrudeMode::Join, "Join"),
                            (ExtrudeMode::Cut, "Cut"),
                        ] {
                            if ui.selectable_label(op_new.mode == m, label).clicked() {
                                op_new.mode = m;
                            }
                        }
                    });
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        let can_ok = op_new.path_sketch.is_some()
                            && (op_new.guide_sketch.is_none() || op_new.profile_entity.is_some());
                        if ui.add_enabled(can_ok, egui::Button::new("OK")).clicked() {
                            commit = true;
                        }
                        if ui.button("Cancel").clicked() {
                            cancel = true;
                        }
                    });
                });
            });

        if cancel {
            self.sweep_op = None;
            self.status_msg = "Sweep cancelled.".to_string();
            return;
        }
        self.sweep_op = Some(op_new.clone());
        if commit {
            self.commit_sweep_op(op_new);
        }
    }

    fn commit_sweep_op(&mut self, op: SweepOp) {
        let Some(path_sketch) = op.path_sketch.clone() else {
            return;
        };
        let Some(total_twist_deg) = self.eval_dim(&op.total_twist_text) else {
            self.status_msg = "Sweep twist must be a number or valid expression.".to_string();
            return;
        };
        if !total_twist_deg.is_finite() {
            self.status_msg = "Sweep twist must be finite.".to_string();
            return;
        }
        if op.guide_sketch.is_some() && total_twist_deg != 0.0 {
            self.status_msg = "A guided Sweep cannot also use explicit twist.".to_string();
            return;
        }
        let guide = match (op.guide_sketch.clone(), op.profile_entity) {
            (Some(sketch), Some(profile_entity)) => Some(zerocad_core::SweepGuide {
                sketch,
                profile_entity,
                profile_parameter: op.profile_parameter,
            }),
            (Some(_), None) => {
                self.status_msg = "Choose a durable profile anchor for the guide.".to_string();
                return;
            }
            (None, _) => None,
        };
        // Cut/Join from a face-attached profile targets that body.
        let target = if matches!(op.mode, ExtrudeMode::Cut | ExtrudeMode::Join) {
            self.document
                .sketch_face_refs
                .get(op.profile_sketch.as_str())
                .and_then(|fref| fref.topology.as_ref())
                .and_then(|t| t.body_id.clone())
        } else {
            None
        };
        self.push_undo();
        let n = self.next_id();
        let id = format!("sweep_{n}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Sweep {n}"),
            feature: FeatureType::Sweep {
                profile_sketch: op.profile_sketch.clone(),
                profile_region: op.profile_region,
                path_sketch: path_sketch.clone(),
                mode: op.mode,
                target,
                total_twist_deg,
                total_twist_expr: zerocad_core::expr::preserves_source(&op.total_twist_text)
                    .then(|| op.total_twist_text.trim().to_string()),
                guide: guide.clone(),
            },
        });
        self.document.add_dependency(&op.profile_sketch, &id);
        self.document.add_dependency(&path_sketch, &id);
        if let Some(guide) = &guide {
            self.document.add_dependency(&guide.sketch, &id);
        }
        self.hidden_nodes.insert(op.profile_sketch.clone());
        self.selected_faces.clear();
        self.selected_node_id = Some(id);
        self.sweep_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Sweep created.".to_string();
    }
}
