use crate::*;

#[derive(Debug, Clone)]
pub(crate) struct InspectionDialog {
    minimum_volume_mm3: f64,
    body_measurements: Vec<zerocad_core::BodyInspection>,
    entity_measurements: Vec<String>,
    interferences: Vec<zerocad_core::InterferencePair>,
    error: Option<String>,
    check_all_visible: bool,
}

impl Default for InspectionDialog {
    fn default() -> Self {
        Self {
            minimum_volume_mm3: 1.0e-6,
            body_measurements: Vec::new(),
            entity_measurements: Vec::new(),
            interferences: Vec::new(),
            error: None,
            check_all_visible: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SectionAxis {
    X,
    Y,
    Z,
}

impl SectionAxis {
    fn normal(self) -> [f32; 3] {
        match self {
            Self::X => [1.0, 0.0, 0.0],
            Self::Y => [0.0, 1.0, 0.0],
            Self::Z => [0.0, 0.0, 1.0],
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct SectionView {
    pub(crate) origin: [f32; 3],
    pub(crate) normal: [f32; 3],
    pub(crate) offset: f32,
    pub(crate) keep_positive: bool,
    pub(crate) capped: bool,
}

impl Default for SectionView {
    fn default() -> Self {
        Self {
            origin: [0.0; 3],
            normal: SectionAxis::Z.normal(),
            offset: 0.0,
            keep_positive: false,
            capped: true,
        }
    }
}

impl SectionView {
    pub(crate) fn plane(&self) -> ([f32; 3], [f32; 3]) {
        let length = (self.normal[0] * self.normal[0]
            + self.normal[1] * self.normal[1]
            + self.normal[2] * self.normal[2])
            .sqrt();
        let normal = if length > 1.0e-8 {
            [
                self.normal[0] / length,
                self.normal[1] / length,
                self.normal[2] / length,
            ]
        } else {
            SectionAxis::Z.normal()
        };
        (
            [
                self.origin[0] + normal[0] * self.offset,
                self.origin[1] + normal[1] * self.offset,
                self.origin[2] + normal[2] * self.offset,
            ],
            normal,
        )
    }
}

impl ZeroCadApp {
    pub(crate) fn open_inspection_dialog(&mut self) {
        self.inspection_dialog.get_or_insert_with(Default::default);
    }

    pub(crate) fn show_inspection_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.inspection_dialog.take() else {
            return;
        };
        let mut close = false;
        let mut measure = false;
        let mut interference = false;
        let mut selected_pair = None;
        let selected_face_plane = self.selected_body.iter().find_map(|(body, pick)| {
            let BodyPick::Face(face_id) = pick else {
                return None;
            };
            self.face_is_planar(body, *face_id)
                .then(|| self.face_ref(body, *face_id))
                .flatten()
                .map(|face| (face.centroid, face.normal))
        });
        let datum_planes = self.resolved_datum_planes();
        egui::Window::new("Inspect")
            .id(egui::Id::new("inspection-tools"))
            .default_size(egui::vec2(430.0, 520.0))
            .resizable(true)
            .show(ctx, |ui| {
                ui.heading("Measure");
                ui.label(
                    egui::RichText::new(
                        "Select bodies, faces, edges, or points in the viewport. Body properties are recomputed from strict kernel geometry, not the display mesh.",
                    )
                    .small()
                    .weak(),
                );
                if ui.button("Measure selection").clicked() {
                    measure = true;
                }
                ui.horizontal(|ui| {
                    ui.label("Density");
                    ui.add(
                        egui::DragValue::new(&mut self.measure_density)
                            .speed(0.05)
                            .range(0.0..=100.0)
                            .suffix(" g/cm³"),
                    );
                });
                for body in &dialog.body_measurements {
                    egui::CollapsingHeader::new(&body.body_id)
                        .default_open(true)
                        .show(ui, |ui| {
                            ui.monospace(format!("Volume: {:.6} mm³", body.volume_mm3));
                            ui.monospace(format!(
                                "Surface area: {:.6} mm²",
                                body.surface_area_mm2
                            ));
                            ui.monospace(format!(
                                "Mass: {:.6} g at {:.6} g/cm³",
                                body.mass_g, body.density_g_cm3
                            ));
                            ui.monospace(format!(
                                "Centroid: ({:.6}, {:.6}, {:.6}) mm",
                                body.centroid[0], body.centroid[1], body.centroid[2]
                            ));
                            ui.monospace(format!(
                                "Bounds: ({:.3}, {:.3}, {:.3}) – ({:.3}, {:.3}, {:.3}) mm",
                                body.bounds_min[0],
                                body.bounds_min[1],
                                body.bounds_min[2],
                                body.bounds_max[0],
                                body.bounds_max[1],
                                body.bounds_max[2]
                            ));
                        });
                }
                for measurement in &dialog.entity_measurements {
                    ui.monospace(measurement);
                }

                ui.separator();
                ui.heading("Section view");
                let mut enabled = self.section_view.is_some();
                if ui.checkbox(&mut enabled, "Enable visual section").changed() {
                    self.section_view = enabled.then(SectionView::default);
                }
                if let Some(section) = &mut self.section_view {
                    ui.label(
                        egui::RichText::new(
                            "Section mode uses the precise CPU clipping path; GPU acceleration resumes when disabled.",
                        )
                        .small()
                        .weak(),
                    );
                    ui.horizontal(|ui| {
                        ui.label("Origin plane");
                        for (axis, label) in [
                            (SectionAxis::X, "YZ"),
                            (SectionAxis::Y, "XZ"),
                            (SectionAxis::Z, "XY"),
                        ] {
                            if ui.button(label).clicked() {
                                section.origin = [0.0; 3];
                                section.normal = axis.normal();
                                section.offset = 0.0;
                            }
                        }
                    });
                    if ui
                        .add_enabled(
                            selected_face_plane.is_some(),
                            egui::Button::new("Use selected planar face"),
                        )
                        .clicked()
                    {
                        if let Some((origin, normal)) = selected_face_plane {
                            section.origin = origin;
                            section.normal = normal;
                            section.offset = 0.0;
                        }
                    }
                    if !datum_planes.is_empty() {
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Datum");
                            for (_, name, cs) in &datum_planes {
                                if ui.small_button(name).clicked() {
                                    section.origin = [cs.origin.x, cs.origin.y, cs.origin.z];
                                    section.normal = [cs.n.x, cs.n.y, cs.n.z];
                                    section.offset = 0.0;
                                }
                            }
                        });
                    }
                    ui.label(egui::RichText::new("Custom plane").small().strong());
                    ui.horizontal(|ui| {
                        ui.label("Origin");
                        for value in &mut section.origin {
                            ui.add(egui::DragValue::new(value).speed(0.25));
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Normal");
                        for value in &mut section.normal {
                            ui.add(egui::DragValue::new(value).speed(0.02));
                        }
                    });
                    ui.add(
                        egui::Slider::new(&mut section.offset, -1_000.0..=1_000.0)
                            .text("Offset (mm)"),
                    );
                    ui.checkbox(&mut section.keep_positive, "Keep positive side");
                    ui.checkbox(&mut section.capped, "Show section cap");
                }

                ui.separator();
                ui.heading("Interference");
                ui.checkbox(&mut dialog.check_all_visible, "Check all visible bodies");
                ui.horizontal(|ui| {
                    ui.label("Minimum volume");
                    ui.add(
                        egui::DragValue::new(&mut dialog.minimum_volume_mm3)
                            .speed(1.0e-4)
                            .range(0.0..=f64::MAX)
                            .suffix(" mm³"),
                    );
                });
                if ui.button("Run exact Common check").clicked() {
                    interference = true;
                }
                if dialog.interferences.is_empty() {
                    ui.label(egui::RichText::new("No positive-volume interference results.").weak());
                }
                for (index, report) in dialog.interferences.iter().enumerate() {
                    if ui
                        .selectable_label(
                            false,
                            format!(
                                "{} ↔ {}: {:.6} mm³ at ({:.3}, {:.3}, {:.3})",
                                report.first_body,
                                report.second_body,
                                report.volume_mm3,
                                report.centroid[0],
                                report.centroid[1],
                                report.centroid[2]
                            ),
                        )
                        .clicked()
                    {
                        selected_pair = Some(index);
                    }
                }
                if let Some(error) = &dialog.error {
                    ui.colored_label(egui::Color32::from_rgb(220, 38, 38), error);
                }
                ui.separator();
                if ui.button("Close").clicked() {
                    close = true;
                }
            });

        if measure {
            match self.measure_current_selection() {
                Ok((bodies, entities)) => {
                    dialog.body_measurements = bodies;
                    dialog.entity_measurements = entities;
                    dialog.error = None;
                }
                Err(error) => dialog.error = Some(error),
            }
        }
        if interference {
            let ids = if dialog.check_all_visible {
                self.body_meshes
                    .iter()
                    .filter(|(id, _)| !self.hidden_nodes.contains(id))
                    .map(|(id, _)| id.clone())
                    .collect::<Vec<_>>()
            } else {
                self.selected_body_ids()
            };
            match self
                .document
                .inspect_interference(&ids, dialog.minimum_volume_mm3)
            {
                Ok(results) => {
                    dialog.interferences = results;
                    dialog.error = None;
                }
                Err(error) => dialog.error = Some(error),
            }
        }
        if let Some(index) = selected_pair {
            if let Some(report) = dialog.interferences.get(index) {
                self.selected_body.clear();
                self.selected_body
                    .insert((report.first_body.clone(), BodyPick::Whole));
                self.selected_body
                    .insert((report.second_body.clone(), BodyPick::Whole));
            }
        }
        if !close {
            self.inspection_dialog = Some(dialog);
        }
    }

    fn selected_body_ids(&self) -> Vec<String> {
        let mut ids: Vec<String> = self
            .selected_body
            .iter()
            .map(|(id, _)| id.clone())
            .collect();
        if let Some(id) = &self.selected_node_id {
            if self.body_meshes.iter().any(|(body, _)| body == id) {
                ids.push(id.clone());
            }
        }
        ids.sort();
        ids.dedup();
        ids
    }

    fn measure_current_selection(
        &self,
    ) -> Result<(Vec<zerocad_core::BodyInspection>, Vec<String>), String> {
        let ids = self.selected_body_ids();
        if ids.is_empty() {
            return Err("Select at least one body or body element to measure.".to_string());
        }
        let mut bodies = Vec::new();
        for id in &ids {
            bodies.push(
                self.document
                    .inspect_body_with_density(id, f64::from(self.measure_density))?,
            );
        }
        let mut entities = Vec::new();
        let mut points = Vec::new();
        let mut edges = Vec::new();
        for (body_id, pick) in &self.selected_body {
            let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| id == body_id) else {
                continue;
            };
            match pick {
                BodyPick::Vertex(index) => {
                    let offset = *index as usize * 3;
                    if let Some(position) = mesh.edge_vertices.get(offset..offset + 3) {
                        let point = [position[0], position[1], position[2]];
                        points.push(point);
                        entities.push(format!(
                            "Point {}: ({:.6}, {:.6}, {:.6}) mm",
                            index, point[0], point[1], point[2]
                        ));
                    }
                }
                BodyPick::Edge(group) => {
                    if let Some(edge) = Self::edge_ref_from_mesh(body_id, mesh, *group) {
                        let vector = [
                            edge.p1[0] - edge.p0[0],
                            edge.p1[1] - edge.p0[1],
                            edge.p1[2] - edge.p0[2],
                        ];
                        edges.push(vector);
                        let chord =
                            (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
                                .sqrt();
                        match edge.curve {
                            Some(EdgeCurveHint::Circle {
                                radius,
                                start,
                                end,
                                closed,
                                ..
                            }) => {
                                let sweep = if closed {
                                    std::f32::consts::TAU
                                } else {
                                    (end - start).abs()
                                };
                                entities.push(format!(
                                    "Edge {group}: radius {radius:.6} mm, diameter {:.6} mm, arc length {:.6} mm",
                                    radius * 2.0,
                                    radius * sweep
                                ));
                            }
                            _ => entities.push(format!("Edge {group}: length {chord:.6} mm")),
                        }
                    }
                }
                BodyPick::Face(face_id) => {
                    if let Some(face) = mesh.face_refs.iter().find(|face| face.face_id == *face_id)
                    {
                        entities.push(format!(
                            "Face {face_id}: centroid ({:.6}, {:.6}, {:.6}), normal ({:.6}, {:.6}, {:.6})",
                            face.centroid[0], face.centroid[1], face.centroid[2],
                            face.normal[0], face.normal[1], face.normal[2]
                        ));
                    }
                    let area = self.document.inspect_face(body_id, *face_id)?.area_mm2;
                    entities.push(format!("Face {face_id}: area {area:.6} mm²"));
                }
                BodyPick::Whole => {}
            }
        }
        if points.len() == 2 {
            let distance = ((points[1][0] - points[0][0]).powi(2)
                + (points[1][1] - points[0][1]).powi(2)
                + (points[1][2] - points[0][2]).powi(2))
            .sqrt();
            entities.push(format!("Point distance: {distance:.6} mm"));
        }
        if edges.len() == 2 {
            let length = |vector: [f32; 3]| {
                (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
            };
            let denominator = length(edges[0]) * length(edges[1]);
            if denominator > 1.0e-8 {
                let cosine = ((edges[0][0] * edges[1][0]
                    + edges[0][1] * edges[1][1]
                    + edges[0][2] * edges[1][2])
                    / denominator)
                    .clamp(-1.0, 1.0);
                entities.push(format!("Edge angle: {:.6}°", cosine.acos().to_degrees()));
            }
        }
        Ok((bodies, entities))
    }
}
