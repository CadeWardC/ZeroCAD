//! Revolve tool UI: select sketch region(s) → pick an axis + angle + mode in a
//! floating dialog → commit a `FeatureType::Revolve` node.
//!
//! Deliberately leaner than the extrude tool (no drag-to-set or live boolean
//! preview yet): the revolve solid appears on commit via the normal
//! re-evaluation. The dialog mirrors `show_extrude_dialog`'s look.

use crate::*;
use zerocad_core::AxisBase;

/// Which axis the in-progress revolve spins about.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RevolveAxisChoice {
    X,
    Y,
    Z,
    /// A datum axis node (id, display name).
    Datum(String, String),
}

impl RevolveAxisChoice {
    pub(crate) fn label(&self) -> String {
        match self {
            Self::X => "X axis".to_string(),
            Self::Y => "Y axis".to_string(),
            Self::Z => "Z axis".to_string(),
            Self::Datum(_, name) => name.clone(),
        }
    }
    pub(crate) fn to_axis_base(&self) -> AxisBase {
        match self {
            Self::X => AxisBase::X,
            Self::Y => AxisBase::Y,
            Self::Z => AxisBase::Z,
            Self::Datum(id, _) => AxisBase::Datum(id.clone()),
        }
    }
}

/// State of the in-progress revolve tool.
#[derive(Debug, Clone)]
pub(crate) struct RevolveOp {
    pub(crate) sketch_id: String,
    pub(crate) region_indices: Vec<usize>,
    pub(crate) axis: RevolveAxisChoice,
    pub(crate) angle_text: String,
    pub(crate) mode: ExtrudeMode,
}

impl ZeroCadApp {
    /// Start a revolve from the currently selected sketch faces (one sketch).
    pub(crate) fn begin_revolve_from_selection(&mut self) {
        if self.revolve_op.is_some() {
            return;
        }
        if self.selected_faces.is_empty() {
            self.status_msg =
                "Select one or more sketch faces in the viewport first, then click Revolve."
                    .to_string();
            return;
        }
        let mut by_sketch: std::collections::HashMap<String, Vec<usize>> =
            std::collections::HashMap::new();
        for (sid, ri) in self.selected_faces.iter().cloned() {
            by_sketch.entry(sid).or_default().push(ri);
        }
        if by_sketch.len() > 1 {
            self.status_msg = "Revolve works on faces of ONE sketch at a time.".to_string();
            return;
        }
        let (sketch_id, mut region_indices) = by_sketch.into_iter().next().unwrap();
        region_indices.sort();
        region_indices.dedup();
        self.revolve_op = Some(RevolveOp {
            sketch_id,
            region_indices,
            axis: RevolveAxisChoice::Y,
            angle_text: "360".to_string(),
            mode: ExtrudeMode::NewBody,
        });
        self.status_msg =
            "Revolve: pick the axis and angle, then OK. The axis must lie in the sketch plane."
                .to_string();
    }

    /// The floating Revolve dialog (drawn every frame while an op is active).
    pub(crate) fn show_revolve_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.revolve_op.clone() else {
            return;
        };
        // Datum axes currently in the document, offered alongside X/Y/Z.
        let datum_axes: Vec<(String, String)> = self
            .resolved_datum_axes_points()
            .into_iter()
            .filter_map(|(id, name, v)| match v {
                zerocad_core::DatumValue::Axis { .. } => Some((id, name)),
                _ => None,
            })
            .collect();

        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        let var_map = self.visible_variable_map();
        egui::Area::new(egui::Id::new("revolve_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(240.0);
                    ui.label(
                        egui::RichText::new("Revolve")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        ui.label("Axis");
                        egui::ComboBox::from_id_salt("revolve_axis")
                            .selected_text(op_new.axis.label())
                            .show_ui(ui, |ui| {
                                for choice in [
                                    RevolveAxisChoice::X,
                                    RevolveAxisChoice::Y,
                                    RevolveAxisChoice::Z,
                                ] {
                                    let label = choice.label();
                                    ui.selectable_value(&mut op_new.axis, choice, label);
                                }
                                for (id, name) in &datum_axes {
                                    let choice = RevolveAxisChoice::Datum(id.clone(), name.clone());
                                    let label = choice.label();
                                    ui.selectable_value(&mut op_new.axis, choice, label);
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Angle");
                        ui.add(
                            egui::TextEdit::singleline(&mut op_new.angle_text).desired_width(60.0),
                        );
                        ui.label("°");
                        crate::expr::evaluation_hint(ui, &op_new.angle_text, &var_map, "°");
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
            self.revolve_op = None;
            self.status_msg = "Revolve cancelled.".to_string();
            return;
        }
        self.revolve_op = Some(op_new.clone());
        if commit {
            self.commit_revolve_op(op_new);
        }
    }

    fn commit_revolve_op(&mut self, op: RevolveOp) {
        let Some(angle) = self.eval_dim(&op.angle_text) else {
            self.status_msg = "Revolve angle must be a number or valid expression.".to_string();
            return;
        };
        if !(angle > 0.0 && angle <= 360.0) {
            self.status_msg = "Revolve angle must be in (0, 360].".to_string();
            return;
        }
        self.push_undo();
        let n = self.next_id();
        let id = format!("revolve_{n}");
        let name = format!("Revolve {n}");
        // Cut/Join from a face-attached sketch targets that body, like extrude.
        let target = if matches!(op.mode, ExtrudeMode::Cut | ExtrudeMode::Join) {
            self.document
                .sketch_face_refs
                .get(&op.sketch_id)
                .and_then(|fref| fref.topology.as_ref())
                .and_then(|t| t.body_id.clone())
        } else {
            None
        };
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature: FeatureType::Revolve {
                axis: op.axis.to_axis_base(),
                angle_deg: angle,
                angle_expr: zerocad_core::expr::preserves_source(&op.angle_text)
                    .then(|| op.angle_text.trim().to_string()),
                region_indices: op.region_indices.clone(),
                mode: op.mode,
                target,
            },
        });
        self.document.add_dependency(&op.sketch_id, &id);
        if let RevolveAxisChoice::Datum(datum_id, _) = &op.axis {
            self.document.add_dependency(datum_id, &id);
        }
        // The revolve consumes its sketch, like an extrude commit.
        self.hidden_nodes.insert(op.sketch_id.clone());
        self.selected_faces.clear();
        self.selected_node_id = Some(id);
        self.revolve_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Revolve created.".to_string();
    }
}
