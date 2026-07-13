//! Shell tool UI: select the face(s) to remove on a body → set wall thickness
//! → commit a `FeatureType::Shell` node hollowing that body.

use crate::*;

/// State of the in-progress shell tool.
#[derive(Debug, Clone)]
pub(crate) struct ShellOp {
    pub(crate) target: String,
    pub(crate) open_faces: Vec<zerocad_core::parametric::FaceRef>,
    pub(crate) thickness_text: String,
}

impl ZeroCadApp {
    /// The body + faces the shell tool would use: one or more selected faces,
    /// all on the SAME body.
    pub(crate) fn shell_candidate(&self) -> Option<(String, Vec<u32>)> {
        let mut body: Option<String> = None;
        let mut fids: Vec<u32> = Vec::new();
        for (id, pick) in &self.selected_body {
            if let BodyPick::Face(f) = pick {
                match &body {
                    Some(b) if b != id => return None, // faces span two bodies
                    _ => body = Some(id.clone()),
                }
                fids.push(*f);
            }
        }
        body.filter(|_| !fids.is_empty()).map(|b| (b, fids))
    }

    pub(crate) fn begin_shell(&mut self, target: String, fids: Vec<u32>) {
        if self.shell_op.is_some() {
            return;
        }
        let open_faces: Vec<_> = fids
            .iter()
            .filter_map(|&fid| self.face_ref(&target, fid))
            .collect();
        if open_faces.is_empty() {
            self.status_msg = "Couldn't resolve the selected faces for shelling.".to_string();
            return;
        }
        self.shell_op = Some(ShellOp {
            target,
            open_faces,
            thickness_text: "2".to_string(),
        });
        self.status_msg = "Shell: set the wall thickness, then OK.".to_string();
    }

    /// The floating Shell dialog (drawn every frame while an op is active).
    pub(crate) fn show_shell_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.shell_op.clone() else {
            return;
        };
        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        let var_map = self.visible_variable_map();
        egui::Area::new(egui::Id::new("shell_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(220.0);
                    ui.label(
                        egui::RichText::new(format!(
                            "Shell ({} open face{})",
                            op_new.open_faces.len(),
                            if op_new.open_faces.len() == 1 {
                                ""
                            } else {
                                "s"
                            }
                        ))
                        .strong()
                        .size(13.5)
                        .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        ui.label("Wall thickness");
                        ui.add(
                            egui::TextEdit::singleline(&mut op_new.thickness_text)
                                .desired_width(50.0),
                        );
                        ui.label("mm");
                        crate::expr::evaluation_hint(ui, &op_new.thickness_text, &var_map, "mm");
                    });
                    ui.add_space(6.0);
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
            self.shell_op = None;
            self.status_msg = "Shell cancelled.".to_string();
            return;
        }
        self.shell_op = Some(op_new.clone());
        if commit {
            self.commit_shell_op(op_new);
        }
    }

    fn commit_shell_op(&mut self, op: ShellOp) {
        let Some(thickness) = self.eval_dim(&op.thickness_text) else {
            self.status_msg = "Shell thickness must be a number or valid expression.".to_string();
            return;
        };
        if thickness <= 0.0 {
            self.status_msg = "Shell thickness must be positive.".to_string();
            return;
        }
        self.push_undo();
        let n = self.next_id();
        let id = format!("shell_{n}");
        self.graph.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Shell {n}"),
            feature: FeatureType::Shell {
                target: op.target.clone(),
                thickness,
                thickness_expr: zerocad_core::expr::preserves_source(&op.thickness_text)
                    .then(|| op.thickness_text.trim().to_string()),
                open_faces: op.open_faces.clone(),
            },
        });
        self.graph.add_dependency(&op.target, &id);
        self.selected_body.clear();
        self.selected_node_id = Some(id);
        self.shell_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Shell created.".to_string();
    }
}
