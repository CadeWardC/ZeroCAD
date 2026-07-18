//! Two-body Combine UI: strict Join, Cut, or Intersect.

use crate::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CombineMode {
    Join,
    Cut,
    Intersect,
}

#[derive(Debug, Clone)]
pub(crate) struct CombineOp {
    pub(crate) bodies: [String; 2],
    pub(crate) mode: CombineMode,
    pub(crate) target_index: usize,
    pub(crate) keep_tool: bool,
}

impl ZeroCadApp {
    /// Exactly two whole-body selections in stable display order. Any mixed
    /// face/edge/vertex selection makes Combine inapplicable.
    pub(crate) fn selected_bodies_to_combine(&self) -> Option<[String; 2]> {
        if self.selected_body.len() != 2
            || self
                .selected_body
                .iter()
                .any(|(_, pick)| !matches!(pick, BodyPick::Whole))
        {
            return None;
        }
        let selected: Vec<String> = self
            .body_meshes
            .iter()
            .filter(|(id, _)| self.selected_body.contains(&(id.clone(), BodyPick::Whole)))
            .map(|(id, _)| id.clone())
            .collect();
        (selected.len() == 2).then(|| [selected[0].clone(), selected[1].clone()])
    }

    pub(crate) fn begin_combine(&mut self, bodies: [String; 2]) {
        if self.combine_op.is_some()
            || bodies
                .iter()
                .any(|body| !self.body_meshes.iter().any(|(id, _)| id == body))
        {
            return;
        }
        self.combine_op = Some(CombineOp {
            bodies,
            mode: CombineMode::Join,
            target_index: 0,
            keep_tool: false,
        });
        self.status_msg = "Combine: choose Join or Cut, then click OK.".to_string();
    }

    fn combine_body_label(&self, id: &str) -> String {
        let owner_id = self.document.body_producer_feature_id(id).unwrap_or(id);
        let mut owned_outputs: Vec<&str> = self
            .body_meshes
            .iter()
            .filter_map(|(candidate, _)| {
                (self.document.body_producer_feature_id(candidate) == Some(owner_id))
                    .then_some(candidate.as_str())
            })
            .collect();
        owned_outputs.sort_by_key(|candidate| usize::from(*candidate != owner_id));
        let output_index = owned_outputs
            .iter()
            .position(|candidate| *candidate == id)
            .unwrap_or(0);
        self.document
            .graph
            .node_indices()
            .find_map(|index| {
                let node = &self.document.graph[index];
                (node.id == owner_id)
                    .then(|| format!("{} ({id})", body_output_label(&node.name, output_index)))
            })
            .unwrap_or_else(|| id.to_string())
    }

    pub(crate) fn show_combine_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.combine_op.clone() else {
            return;
        };
        let labels = [
            self.combine_body_label(&op.bodies[0]),
            self.combine_body_label(&op.bodies[1]),
        ];
        let mut next = op;
        let mut commit = false;
        let mut cancel = false;
        egui::Area::new(egui::Id::new("combine_bodies_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(300.0);
                    ui.label(
                        egui::RichText::new("Combine Bodies")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut next.mode, CombineMode::Join, "Join");
                        ui.selectable_value(&mut next.mode, CombineMode::Cut, "Cut");
                        ui.selectable_value(&mut next.mode, CombineMode::Intersect, "Intersect");
                    });
                    ui.separator();
                    match next.mode {
                        CombineMode::Join => {
                            ui.label("Both bodies are fused into one.");
                            ui.label(
                                egui::RichText::new("They must touch or overlap.")
                                    .size(11.0)
                                    .color(self.pal().text_muted),
                            );
                            ui.add_space(4.0);
                            ui.label(&labels[0]);
                            ui.label(&labels[1]);
                        }
                        CombineMode::Cut | CombineMode::Intersect => {
                            let tool_index = 1 - next.target_index;
                            ui.label(format!("Target: {}", labels[next.target_index]));
                            ui.label(format!(
                                "{}: {}",
                                if next.mode == CombineMode::Cut {
                                    "Cutting body"
                                } else {
                                    "Intersecting body"
                                },
                                labels[tool_index]
                            ));
                            if ui.button("Swap target and tool body").clicked() {
                                next.target_index = tool_index;
                            }
                            ui.checkbox(
                                &mut next.keep_tool,
                                if next.mode == CombineMode::Cut {
                                    "Keep cutting body after cut"
                                } else {
                                    "Keep tool body after intersect"
                                },
                            );
                            ui.label(
                                egui::RichText::new(if next.mode == CombineMode::Cut {
                                    "The bodies must overlap."
                                } else {
                                    "Only their positive overlapping volume is kept."
                                })
                                .size(11.0)
                                .color(self.pal().text_muted),
                            );
                        }
                    }
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked() {
                            commit = true;
                        }
                        if ui.button("Cancel").clicked() {
                            cancel = true;
                        }
                    });
                });
            });

        if cancel {
            self.combine_op = None;
            self.status_msg = "Combine cancelled.".to_string();
        } else if commit {
            self.commit_combine(next);
        } else {
            self.combine_op = Some(next);
        }
    }

    fn commit_combine(&mut self, op: CombineOp) {
        self.push_undo();
        let n = self.next_id();
        let (id, name, feature) = match op.mode {
            CombineMode::Join => (
                format!("join_bodies_{n}"),
                self.next_body_join_name(),
                FeatureType::BodyJoin {
                    sources: op.bodies.to_vec(),
                },
            ),
            CombineMode::Cut => {
                let target = op.bodies[op.target_index].clone();
                let tool = op.bodies[1 - op.target_index].clone();
                (
                    format!("cut_bodies_{n}"),
                    self.next_body_cut_name(),
                    FeatureType::BodyCut {
                        target,
                        tool,
                        keep_tool: op.keep_tool,
                    },
                )
            }
            CombineMode::Intersect => {
                let target = op.bodies[op.target_index].clone();
                let tool = op.bodies[1 - op.target_index].clone();
                (
                    format!("intersect_bodies_{n}"),
                    self.next_body_intersect_name(),
                    FeatureType::BodyIntersect {
                        target,
                        tool,
                        keep_tool: op.keep_tool,
                    },
                )
            }
        };
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature,
        });
        for source in &op.bodies {
            self.document.add_dependency(source, &id);
        }
        self.combine_op = None;
        self.selected_body.clear();
        self.selected_body.insert((id.clone(), BodyPick::Whole));
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = match op.mode {
            CombineMode::Join => "Join created. Bodies must touch or overlap.",
            CombineMode::Cut => "Cut created. Bodies must overlap.",
            CombineMode::Intersect => "Intersect created. Only shared volume is kept.",
        }
        .to_string();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_whole_body_selections_enable_combine_in_display_order() {
        let mut app = ZeroCadApp::new();
        app.set_body_meshes(vec![
            ("body_a".to_string(), MockMesh::empty()),
            ("body_b".to_string(), MockMesh::empty()),
        ]);
        app.selected_body
            .insert(("body_b".to_string(), BodyPick::Whole));
        app.selected_body
            .insert(("body_a".to_string(), BodyPick::Whole));

        assert_eq!(
            app.selected_bodies_to_combine(),
            Some(["body_a".to_string(), "body_b".to_string()])
        );
    }

    #[test]
    fn cut_commit_preserves_target_tool_order_and_keep_option() {
        let mut app = ZeroCadApp::new();
        for (id, name) in [("body_a", "A"), ("body_b", "B")] {
            app.document.add_feature(FeatureNode {
                id: id.to_string(),
                name: name.to_string(),
                feature: FeatureType::Box {
                    w: 10.0,
                    h: 10.0,
                    d: 10.0,
                },
            });
        }
        app.commit_combine(CombineOp {
            bodies: ["body_a".to_string(), "body_b".to_string()],
            mode: CombineMode::Cut,
            target_index: 1,
            keep_tool: true,
        });

        let feature = app
            .document
            .graph
            .node_indices()
            .find_map(|index| match &app.document.graph[index].feature {
                FeatureType::BodyCut {
                    target,
                    tool,
                    keep_tool,
                } => Some((target, tool, keep_tool)),
                _ => None,
            })
            .expect("BodyCut feature");
        assert_eq!(feature.0, "body_b");
        assert_eq!(feature.1, "body_a");
        assert!(*feature.2);
    }

    #[test]
    fn intersect_commit_preserves_target_tool_order_and_keep_option() {
        let mut app = ZeroCadApp::new();
        for (id, name) in [("body_a", "A"), ("body_b", "B")] {
            app.document.add_feature(FeatureNode {
                id: id.to_string(),
                name: name.to_string(),
                feature: FeatureType::Box {
                    w: 10.0,
                    h: 10.0,
                    d: 10.0,
                },
            });
        }
        app.commit_combine(CombineOp {
            bodies: ["body_a".to_string(), "body_b".to_string()],
            mode: CombineMode::Intersect,
            target_index: 1,
            keep_tool: true,
        });
        let feature = app
            .document
            .graph
            .node_indices()
            .find_map(|index| match &app.document.graph[index].feature {
                FeatureType::BodyIntersect {
                    target,
                    tool,
                    keep_tool,
                } => Some((target, tool, keep_tool)),
                _ => None,
            })
            .expect("BodyIntersect feature");
        assert_eq!(feature.0, "body_b");
        assert_eq!(feature.1, "body_a");
        assert!(*feature.2);

        app.undo();
        assert!(!app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodyIntersect { .. })));
        app.redo();
        assert!(app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodyIntersect { .. })));
    }
}
