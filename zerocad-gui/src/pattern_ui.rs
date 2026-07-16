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
    Face(zerocad_core::parametric::FaceRef, String),
}

impl MirrorPlaneChoice {
    fn label(&self) -> String {
        match self {
            Self::XY => "XY plane".to_string(),
            Self::XZ => "XZ plane (ground)".to_string(),
            Self::YZ => "YZ plane".to_string(),
            Self::Datum(_, name) => name.clone(),
            Self::Face(_, label) => label.clone(),
        }
    }
    fn to_plane_base(&self) -> PlaneBase {
        match self {
            Self::XY => PlaneBase::XY,
            Self::XZ => PlaneBase::XZ,
            Self::YZ => PlaneBase::YZ,
            Self::Datum(id, _) => PlaneBase::Datum(id.clone()),
            // Face-picked mirrors resolve from their durable FaceRef. The base
            // value is only a backward-compatible fallback if that face is lost.
            Self::Face(_, _) => PlaneBase::XY,
        }
    }

    fn face_ref(&self) -> Option<zerocad_core::parametric::FaceRef> {
        match self {
            Self::Face(face, _) => Some(face.clone()),
            _ => None,
        }
    }
}

/// Reflect a display mesh across a world-space plane. This is intentionally a
/// mesh transform (not a kernel evaluation): it gives frame-immediate Mirror
/// feedback while the committed feature still uses the exact B-Rep transform.
fn reflect_preview_mesh(
    mesh: &MockMesh,
    origin: zerocad_core::Vec3,
    normal: zerocad_core::Vec3,
) -> MockMesh {
    let mut out = mesh.clone();
    let n = normal.normalize();
    let reflect_point = |p: &mut [f32]| {
        let d = (p[0] - origin.x) * n.x + (p[1] - origin.y) * n.y + (p[2] - origin.z) * n.z;
        p[0] -= 2.0 * d * n.x;
        p[1] -= 2.0 * d * n.y;
        p[2] -= 2.0 * d * n.z;
    };
    let reflect_vector = |v: &mut [f32]| {
        let d = v[0] * n.x + v[1] * n.y + v[2] * n.z;
        v[0] -= 2.0 * d * n.x;
        v[1] -= 2.0 * d * n.y;
        v[2] -= 2.0 * d * n.z;
    };
    for vertex in out.vertices.chunks_exact_mut(6) {
        reflect_point(&mut vertex[..3]);
        reflect_vector(&mut vertex[3..6]);
    }
    // Reflection reverses handedness; restore outward triangle winding.
    for triangle in out.indices.chunks_exact_mut(3) {
        triangle.swap(1, 2);
    }
    for point in out.edge_vertices.chunks_exact_mut(3) {
        reflect_point(point);
    }
    for normals in out.edge_face_normals.chunks_exact_mut(6) {
        reflect_vector(&mut normals[..3]);
        reflect_vector(&mut normals[3..6]);
    }
    for edge in &mut out.edge_refs {
        reflect_point(&mut edge.p0);
        reflect_point(&mut edge.p1);
        reflect_vector(&mut edge.n1);
        reflect_vector(&mut edge.n2);
    }
    for face in &mut out.face_refs {
        reflect_point(&mut face.centroid);
        reflect_vector(&mut face.normal);
    }
    out
}

fn translate_preview_mesh(mesh: &mut MockMesh, offset: zerocad_core::Vec3) {
    let translate = |point: &mut [f32]| {
        point[0] += offset.x;
        point[1] += offset.y;
        point[2] += offset.z;
    };
    for vertex in mesh.vertices.chunks_exact_mut(6) {
        translate(&mut vertex[..3]);
    }
    for point in mesh.edge_vertices.chunks_exact_mut(3) {
        translate(point);
    }
    for edge in &mut mesh.edge_refs {
        translate(&mut edge.p0);
        translate(&mut edge.p1);
    }
    for face in &mut mesh.face_refs {
        translate(&mut face.centroid);
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
    pub(crate) plane: Option<MirrorPlaneChoice>,
    /// The viewport is showing selectable origin/datum planes and planar faces.
    pub(crate) pick_plane: bool,
    pub(crate) mirror_offset_enabled: bool,
    pub(crate) mirror_offset_text: String,
    pub(crate) mirror_join: bool,
    pub(crate) spacing_text: String,
    pub(crate) count_text: String,
    pub(crate) angle_text: String,
}

impl ZeroCadApp {
    /// Lightweight live body ghost for a Mirror op whose reflection plane has
    /// been picked. Returns `None` while the picker is still waiting for input.
    pub(crate) fn mirror_preview_mesh(&self) -> Option<MockMesh> {
        let op = self.pattern_op.as_ref()?;
        if op.kind != PatternKindChoice::Mirror || op.pick_plane {
            return None;
        }
        let plane = op.plane.as_ref()?;
        let (origin, normal) = match plane {
            MirrorPlaneChoice::XY => (zerocad_core::Vec3::ZERO, zerocad_core::Vec3::Z),
            MirrorPlaneChoice::XZ => (zerocad_core::Vec3::ZERO, zerocad_core::Vec3::Y),
            MirrorPlaneChoice::YZ => (zerocad_core::Vec3::ZERO, zerocad_core::Vec3::X),
            MirrorPlaneChoice::Datum(id, _) => {
                let (_, _, cs) = self
                    .resolved_datum_planes()
                    .into_iter()
                    .find(|(datum_id, _, _)| datum_id == id)?;
                (cs.origin, cs.n)
            }
            MirrorPlaneChoice::Face(face, _) => (
                zerocad_core::Vec3::new(face.centroid[0], face.centroid[1], face.centroid[2]),
                zerocad_core::Vec3::new(face.normal[0], face.normal[1], face.normal[2]),
            ),
        };
        let (_, mesh) = self
            .body_meshes
            .iter()
            .find(|(body_id, _)| body_id == &op.source)?;
        let mut preview = reflect_preview_mesh(mesh, origin, normal);
        if op.mirror_offset_enabled {
            let distance = self.eval_dim(&op.mirror_offset_text).unwrap_or(0.0);
            translate_preview_mesh(&mut preview, normal.normalize().mul(distance));
        }
        Some(preview)
    }

    /// True when `id` names a node that produces a body (a valid pattern source).
    fn node_is_body(&self, id: &str) -> bool {
        self.document.graph.node_indices().any(|i| {
            let n = &self.document.graph[i];
            n.id == id
                && matches!(
                    n.feature,
                    FeatureType::Box { .. }
                        | FeatureType::Cylinder { .. }
                        | FeatureType::Import { .. }
                        | FeatureType::Pattern { .. }
                        | FeatureType::BodyJoin { .. }
                        | FeatureType::BodyCut { .. }
                        | FeatureType::BodyIntersect { .. }
                        | FeatureType::BodySplit { .. }
                        | FeatureType::BodyScale { .. }
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
        let from_viewport = self
            .selected_body
            .iter()
            .find_map(|(id, pick)| matches!(pick, BodyPick::Whole).then(|| id.clone()));
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
            plane: None,
            pick_plane: false,
            mirror_offset_enabled: false,
            mirror_offset_text: "0".to_string(),
            mirror_join: false,
            spacing_text: "20".to_string(),
            count_text: "3".to_string(),
            angle_text: "360".to_string(),
        });
        self.status_msg = "Pattern: choose the kind and parameters, then OK.".to_string();
    }

    /// Start the standalone 3D Mirror command directly in viewport plane-pick
    /// mode. It deliberately does not share Pattern's kind selector.
    pub(crate) fn begin_mirror(&mut self, source: String) {
        if self.pattern_op.is_some() {
            return;
        }
        self.pattern_op = Some(PatternOp {
            source,
            kind: PatternKindChoice::Mirror,
            axis: RevolveAxisChoice::X,
            plane: None,
            pick_plane: true,
            mirror_offset_enabled: false,
            mirror_offset_text: "0".to_string(),
            mirror_join: false,
            spacing_text: "20".to_string(),
            count_text: "3".to_string(),
            angle_text: "360".to_string(),
        });
        self.status_msg = "Mirror: pick an origin/datum plane or any planar body face.".to_string();
    }

    pub(crate) fn mirror_plane_pick_active(&self) -> bool {
        self.pattern_op
            .as_ref()
            .is_some_and(|op| op.kind == PatternKindChoice::Mirror && op.pick_plane)
    }

    pub(crate) fn plane_pick_active(&self) -> bool {
        self.is_plane_selection_mode || self.mirror_plane_pick_active()
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
        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        let var_map = self.visible_variable_map();
        egui::Area::new(egui::Id::new("pattern_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(250.0);
                    let title = if op_new.kind == PatternKindChoice::Mirror {
                        format!("Mirror {}", op_new.source)
                    } else {
                        format!("Pattern of {}", op_new.source)
                    };
                    ui.label(
                        egui::RichText::new(title)
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    if op_new.kind != PatternKindChoice::Mirror {
                        ui.horizontal(|ui| {
                            for (k, label) in [
                                (PatternKindChoice::Linear, "Linear"),
                                (PatternKindChoice::Circular, "Circular"),
                            ] {
                                if ui.selectable_label(op_new.kind == k, label).clicked() {
                                    op_new.kind = k;
                                }
                            }
                        });
                    }
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
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.spacing_text,
                                    &var_map,
                                    "mm",
                                );
                            });
                            ui.horizontal(|ui| {
                                ui.label("Count");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.count_text)
                                        .desired_width(40.0),
                                );
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.count_text,
                                    &var_map,
                                    "",
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
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.count_text,
                                    &var_map,
                                    "",
                                );
                            });
                            ui.horizontal(|ui| {
                                ui.label("Total angle");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.angle_text)
                                        .desired_width(50.0),
                                );
                                ui.label("°");
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.angle_text,
                                    &var_map,
                                    "°",
                                );
                            });
                        }
                        PatternKindChoice::Mirror => {
                            let label = op_new
                                .plane
                                .as_ref()
                                .map(MirrorPlaneChoice::label)
                                .unwrap_or_else(|| "No plane selected".to_string());
                            ui.label(format!("Mirror plane: {label}"));
                            let pick_label = if op_new.pick_plane {
                                "Picking plane…"
                            } else {
                                "Pick mirror plane"
                            };
                            if ui.button(pick_label).clicked() {
                                op_new.pick_plane = true;
                                self.status_msg = "Mirror: pick an origin/datum plane or a planar body face in the viewport.".to_string();
                            }
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut op_new.mirror_offset_enabled, "Offset");
                                if op_new.mirror_offset_enabled {
                                    ui.add(
                                        egui::TextEdit::singleline(
                                            &mut op_new.mirror_offset_text,
                                        )
                                        .desired_width(70.0),
                                    );
                                    ui.label("mm");
                                    crate::expr::evaluation_hint(
                                        ui,
                                        &op_new.mirror_offset_text,
                                        &var_map,
                                        "mm",
                                    );
                                }
                            });
                            ui.checkbox(
                                &mut op_new.mirror_join,
                                "Join when the mirrored body connects",
                            );
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
        let count_value = self.eval_dim(&op.count_text).unwrap_or(0.0);
        let count = count_value.round() as u32;
        if (count_value - count as f32).abs() > 1.0e-5 {
            self.status_msg = "Pattern count must evaluate to a whole number.".to_string();
            return;
        }
        let kind = match op.kind {
            PatternKindChoice::Linear => {
                let spacing = self.eval_dim(&op.spacing_text).unwrap_or(0.0);
                if count < 2 || spacing.abs() < 1e-6 {
                    self.status_msg =
                        "Linear pattern needs count ≥ 2 and a non-zero spacing.".to_string();
                    return;
                }
                PatternKind::Linear {
                    dir: op.axis.to_axis_base(),
                    spacing,
                    spacing_expr: zerocad_core::expr::preserves_source(&op.spacing_text)
                        .then(|| op.spacing_text.trim().to_string()),
                    count,
                }
            }
            PatternKindChoice::Circular => {
                let angle = self.eval_dim(&op.angle_text).unwrap_or(360.0);
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
            PatternKindChoice::Mirror => {
                let Some(plane) = op.plane.as_ref() else {
                    self.status_msg = "Pick a mirror plane or planar face first.".to_string();
                    return;
                };
                let offset = if op.mirror_offset_enabled {
                    let Some(value) = self.eval_dim(&op.mirror_offset_text) else {
                        self.status_msg =
                            "Mirror offset must be a number or valid expression.".to_string();
                        return;
                    };
                    value
                } else {
                    0.0
                };
                let offset_expr = (op.mirror_offset_enabled
                    && zerocad_core::expr::preserves_source(&op.mirror_offset_text))
                .then(|| op.mirror_offset_text.trim().to_string());
                PatternKind::Mirror {
                    plane: plane.to_plane_base(),
                    face: plane.face_ref(),
                    offset,
                    offset_expr,
                    join: op.mirror_join,
                }
            }
        };
        self.push_undo();
        let n = self.next_id();
        let id = format!("pattern_{n}");
        let mirror_joins = matches!(kind, PatternKind::Mirror { join: true, .. });
        let name = match op.kind {
            PatternKindChoice::Mirror if mirror_joins => format!("Mirror {n}"),
            PatternKindChoice::Mirror => self.next_body_name(),
            _ => format!("Pattern {n}"),
        };
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature: FeatureType::Pattern {
                source: op.source.clone(),
                kind,
            },
        });
        self.document.add_dependency(&op.source, &id);
        if let RevolveAxisChoice::Datum(datum_id, _) = &op.axis {
            self.document.add_dependency(datum_id, &id);
        }
        if let Some(MirrorPlaneChoice::Datum(datum_id, _)) = &op.plane {
            self.document.add_dependency(datum_id, &id);
        }
        if let Some(MirrorPlaneChoice::Face(face, _)) = &op.plane {
            if let Some(body_id) = face
                .topology
                .as_ref()
                .and_then(|topology| topology.body_id.as_deref())
            {
                self.document.add_dependency(body_id, &id);
            }
        }
        self.selected_node_id = Some(id);
        self.pattern_op = None;
        self.reevaluate_geometry();
        self.status_msg = if mirror_joins {
            "Mirrored copy joined into the source body.".to_string()
        } else if op.kind == PatternKindChoice::Mirror {
            "Mirrored body created.".to_string()
        } else {
            "Pattern created.".to_string()
        };
    }
}

fn axis_combo(
    ui: &mut egui::Ui,
    axis: &mut RevolveAxisChoice,
    datum_axes: &[(String, String)],
    id: &str,
) {
    egui::ComboBox::from_id_salt(id)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirror_preview_reflects_positions_normals_and_winding() {
        let mesh = MockMesh {
            vertices: vec![
                1.0, 0.0, 0.0, 1.0, 0.0, 0.0, // v0
                1.0, 1.0, 0.0, 1.0, 0.0, 0.0, // v1
                1.0, 0.0, 1.0, 1.0, 0.0, 0.0, // v2
            ],
            indices: vec![0, 1, 2],
            edge_vertices: vec![1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
            edge_indices: vec![0, 1],
            edge_face_normals: vec![1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            face_ids: vec![0],
            edge_groups: vec![0],
            edge_refs: Vec::new(),
            face_refs: Vec::new(),
        };

        let mirrored = reflect_preview_mesh(&mesh, zerocad_core::Vec3::ZERO, zerocad_core::Vec3::X);
        assert_eq!(mirrored.indices, vec![0, 2, 1]);
        for vertex in mirrored.vertices.chunks_exact(6) {
            assert!((vertex[0] + 1.0).abs() < 1.0e-6);
            assert!((vertex[3] + 1.0).abs() < 1.0e-6);
        }
        assert!((mirrored.edge_vertices[0] + 1.0).abs() < 1.0e-6);
        assert!((mirrored.edge_face_normals[0] + 1.0).abs() < 1.0e-6);
    }
}
