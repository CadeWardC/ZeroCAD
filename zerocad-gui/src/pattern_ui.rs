//! Pattern/Mirror tool UI: select a body → choose linear / circular / mirror
//! parameters in a floating dialog → commit a `FeatureType::Pattern` node.

use crate::revolve_ui::RevolveAxisChoice;
use crate::*;
use zerocad_core::{PatternKind, PlaneBase};

/// Which mirror plane the in-progress pattern uses.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum MirrorPlaneChoice {
    XY,
    XZ,
    YZ,
    Datum(String, String),
}

impl MirrorPlaneChoice {
    fn label(&self) -> String {
        match self {
            Self::XY => "XY plane".to_string(),
            Self::XZ => "XZ plane (ground)".to_string(),
            Self::YZ => "YZ plane".to_string(),
            Self::Datum(_, name) => name.clone(),
        }
    }
    fn to_plane_base(&self) -> PlaneBase {
        match self {
            Self::XY => PlaneBase::XY,
            Self::XZ => PlaneBase::XZ,
            Self::YZ => PlaneBase::YZ,
            Self::Datum(id, _) => PlaneBase::Datum(id.clone()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum PatternKindChoice {
    Linear,
    Circular,
    Mirror,
}

/// State of the in-progress pattern tool.
#[derive(Debug, Clone)]
pub(crate) struct PatternOp {
    pub(crate) source: String,
    pub(crate) kind: PatternKindChoice,
    pub(crate) axis: RevolveAxisChoice,
    pub(crate) plane: MirrorPlaneChoice,
    pub(crate) spacing_text: String,
    pub(crate) count_text: String,
    pub(crate) angle_text: String,
}

impl ZeroCadApp {
    /// True when `id` names a node that produces a body (a valid pattern source).
    fn node_is_body(&self, id: &str) -> bool {
        self.graph.graph.node_indices().any(|i| {
            let n = &self.graph.graph[i];
            n.id == id
                && matches!(
                    n.feature,
                    FeatureType::Box { .. }
                        | FeatureType::Cylinder { .. }
                        | FeatureType::Import { .. }
                        | FeatureType::Pattern { .. }
                        | FeatureType::Extrude {
                            mode: ExtrudeMode::NewBody,
                            ..
                        }
                        | FeatureType::Revolve {
                            mode: ExtrudeMode::NewBody,
                            ..
                        }
                )
        })
    }

    /// The body node the pattern tool would replicate: a whole-body viewport
    /// selection first, else the feature-tree selection.
    pub(crate) fn pattern_source_candidate(&self) -> Option<String> {
        let from_viewport = self.selected_body.iter().find_map(|(id, pick)| {
            matches!(pick, BodyPick::Whole).then(|| id.clone())
        });
        from_viewport
            .or_else(|| self.selected_node_id.clone())
            .filter(|id| self.node_is_body(id))
    }

    pub(crate) fn begin_pattern(&mut self, source: String) {
        if self.pattern_op.is_some() {
            return;
        }
        self.pattern_op = Some(PatternOp {
            source,
            kind: PatternKindChoice::Linear,
            axis: RevolveAxisChoice::X,
            plane: MirrorPlaneChoice::YZ,
            spacing_text: "20".to_string(),
            count_text: "3".to_string(),
            angle_text: "360".to_string(),
        });
        self.status_msg = "Pattern: choose the kind and parameters, then OK.".to_string();
    }

    /// The floating Pattern dialog (drawn every frame while an op is active).
    pub(crate) fn show_pattern_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.pattern_op.clone() else {
            return;
        };
        let datum_axes: Vec<(String, String)> = self
            .resolved_datum_axes_points()
            .into_iter()
            .filter_map(|(id, name, v)| match v {
                zerocad_core::DatumValue::Axis { .. } => Some((id, name)),
                _ => None,
            })
            .collect();
        let datum_planes: Vec<(String, String)> = self
            .resolved_datum_planes()
            .into_iter()
            .map(|(id, name, _)| (id, name))
            .collect();

        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        egui::Area::new(egui::Id::new("pattern_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(250.0);
                    ui.label(
                        egui::RichText::new(format!("Pattern of {}", op_new.source))
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        for (k, label) in [
                            (PatternKindChoice::Linear, "Linear"),
                            (PatternKindChoice::Circular, "Circular"),
                            (PatternKindChoice::Mirror, "Mirror"),
                        ] {
                            if ui.selectable_label(op_new.kind == k, label).clicked() {
                                op_new.kind = k;
                            }
                        }
                    });
                    ui.add_space(4.0);
                    match op_new.kind {
                        PatternKindChoice::Linear => {
                            ui.horizontal(|ui| {
                                ui.label("Direction");
                                axis_combo(ui, &mut op_new.axis, &datum_axes, "pattern_dir");
                            });
                            ui.horizontal(|ui| {
                                ui.label("Spacing");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.spacing_text)
                                        .desired_width(60.0),
                                );
                                ui.label("mm");
                            });
                            ui.horizontal(|ui| {
                                ui.label("Count");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.count_text)
                                        .desired_width(40.0),
                                );
                            });
                        }
                        PatternKindChoice::Circular => {
                            ui.horizontal(|ui| {
                                ui.label("Axis");
                                axis_combo(ui, &mut op_new.axis, &datum_axes, "pattern_axis");
                            });
                            ui.horizontal(|ui| {
                                ui.label("Count");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.count_text)
                                        .desired_width(40.0),
                                );
                            });
                            ui.horizontal(|ui| {
                                ui.label("Total angle");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.angle_text)
                                        .desired_width(50.0),
                                );
                                ui.label("°");
                            });
                        }
                        PatternKindChoice::Mirror => {
                            ui.horizontal(|ui| {
                                ui.label("Plane");
                                egui::ComboBox::from_id_source("pattern_mirror_plane")
                                    .selected_text(op_new.plane.label())
                                    .show_ui(ui, |ui| {
                                        for choice in [
                                            MirrorPlaneChoice::XY,
                                            MirrorPlaneChoice::XZ,
                                            MirrorPlaneChoice::YZ,
                                        ] {
                                            let label = choice.label();
                                            ui.selectable_value(&mut op_new.plane, choice, label);
                                        }
                                        for (id, name) in &datum_planes {
                                            let choice = MirrorPlaneChoice::Datum(
                                                id.clone(),
                                                name.clone(),
                                            );
                                            let label = choice.label();
                                            ui.selectable_value(&mut op_new.plane, choice, label);
                                        }
                                    });
                            });
                        }
                    }
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
            self.pattern_op = None;
            self.status_msg = "Pattern cancelled.".to_string();
            return;
        }
        self.pattern_op = Some(op_new.clone());
        if commit {
            self.commit_pattern_op(op_new);
        }
    }

    fn commit_pattern_op(&mut self, op: PatternOp) {
        let count: u32 = op.count_text.trim().parse().unwrap_or(0);
        let kind = match op.kind {
            PatternKindChoice::Linear => {
                let spacing: f32 = op.spacing_text.trim().parse().unwrap_or(0.0);
                if count < 2 || spacing.abs() < 1e-6 {
                    self.status_msg =
                        "Linear pattern needs count ≥ 2 and a non-zero spacing.".to_string();
                    return;
                }
                PatternKind::Linear {
                    dir: op.axis.to_axis_base(),
                    spacing,
                    spacing_expr: None,
                    count,
                }
            }
            PatternKindChoice::Circular => {
                let angle: f32 = op.angle_text.trim().parse().unwrap_or(360.0);
                if count < 2 || !(angle > 0.0 && angle <= 360.0) {
                    self.status_msg =
                        "Circular pattern needs count ≥ 2 and an angle in (0, 360].".to_string();
                    return;
                }
                PatternKind::Circular {
                    axis: op.axis.to_axis_base(),
                    count,
                    total_angle_deg: angle,
                }
            }
            PatternKindChoice::Mirror => PatternKind::Mirror {
                plane: op.plane.to_plane_base(),
            },
        };
        self.push_undo();
        let n = self.next_id();
        let id = format!("pattern_{n}");
        self.graph.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Pattern {n}"),
            feature: FeatureType::Pattern {
                source: op.source.clone(),
                kind,
            },
        });
        self.graph.add_dependency(&op.source, &id);
        if let RevolveAxisChoice::Datum(datum_id, _) = &op.axis {
            self.graph.add_dependency(datum_id, &id);
        }
        if let MirrorPlaneChoice::Datum(datum_id, _) = &op.plane {
            self.graph.add_dependency(datum_id, &id);
        }
        self.selected_node_id = Some(id);
        self.pattern_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Pattern created.".to_string();
    }
}

fn axis_combo(
    ui: &mut egui::Ui,
    axis: &mut RevolveAxisChoice,
    datum_axes: &[(String, String)],
    id: &str,
) {
    egui::ComboBox::from_id_source(id)
        .selected_text(axis.label())
        .show_ui(ui, |ui| {
            for choice in [
                RevolveAxisChoice::X,
                RevolveAxisChoice::Y,
                RevolveAxisChoice::Z,
            ] {
                let label = choice.label();
                ui.selectable_value(axis, choice, label);
            }
            for (id, name) in datum_axes {
                let choice = RevolveAxisChoice::Datum(id.clone(), name.clone());
                let label = choice.label();
                ui.selectable_value(axis, choice, label);
            }
        });
}
