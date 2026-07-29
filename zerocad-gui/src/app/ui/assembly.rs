use crate::*;

#[derive(Debug, Clone, Copy)]
enum MateEditAction {
    Delete,
    SetSuppressed(bool),
    FlipSense,
    SetParameter(f64),
}

#[derive(Debug, Clone, Copy)]
enum FaceMateMode {
    Coincident,
    SignedDistance,
    Angle,
}

impl ZeroCadApp {
    pub(crate) fn insert_part_into_assembly(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Insert Part into Assembly")
            .add_filter("ZeroCAD Part", &["zcad", "zcadh"])
            .pick_file()
        else {
            return;
        };
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) => {
                self.status_msg = format!("Insert failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };
        let source_name = path.file_name().and_then(|name| name.to_str());
        let prepared = match zerocad_core::prepare_part_definition(
            &bytes,
            source_name,
            &zerocad_core::LoadOptions::default(),
        ) {
            Ok(prepared) => prepared,
            Err(error) => {
                self.status_msg = format!("Insert failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };
        let geometry = prepared.display_bodies.clone();
        self.push_undo();
        let result = match zerocad_core::insert_prepared_occurrence(
            &mut self.assembly_document,
            prepared,
            zerocad_core::RigidPlacement::IDENTITY,
            false,
        ) {
            Ok(result) => result,
            Err(error) => {
                self.undo_stack.pop();
                self.status_msg = format!("Insert failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };

        self.assembly_definition_geometry
            .entry(result.definition_model_hash)
            .or_insert(geometry);
        self.selected_assembly_occurrence = Some(result.occurrence_id);
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
        self.error_msg = None;
        self.status_msg = if result.diagnostics.is_empty() {
            format!("Inserted component {}.", result.occurrence_id)
        } else {
            format!(
                "Inserted component {} with {} warning(s).",
                result.occurrence_id,
                result.diagnostics.len()
            )
        };
    }

    fn replace_assembly_occurrences(&mut self, replace_all: bool) {
        let Some(selected_id) = self.selected_assembly_occurrence else {
            return;
        };
        let Some(selected) = self
            .assembly_document
            .occurrences
            .get(&selected_id)
            .cloned()
        else {
            return;
        };
        let Some(path) = rfd::FileDialog::new()
            .set_title(if replace_all {
                "Replace All Matching Components"
            } else {
                "Replace Selected Component"
            })
            .add_filter("ZeroCAD Part", &["zcad", "zcadh"])
            .pick_file()
        else {
            return;
        };
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) => {
                self.status_msg = format!("Replace failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };
        let prepared = match zerocad_core::prepare_part_definition(
            &bytes,
            path.file_name().and_then(|name| name.to_str()),
            &zerocad_core::LoadOptions::default(),
        ) {
            Ok(prepared) => prepared,
            Err(error) => {
                self.status_msg = format!("Replace failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };
        let ids: Vec<u64> = if replace_all {
            self.assembly_document
                .occurrences
                .values()
                .filter(|occurrence| {
                    occurrence.definition_model_hash == selected.definition_model_hash
                })
                .map(|occurrence| occurrence.id)
                .collect()
        } else {
            vec![selected_id]
        };

        self.push_undo();
        let result = match zerocad_core::replace_occurrences_with_prepared(
            &mut self.assembly_document,
            &ids,
            prepared,
        ) {
            Ok(result) => result,
            Err(error) => {
                self.undo_stack.pop();
                self.status_msg = format!("Replace failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
                return;
            }
        };
        for hash in &result.pruned_definition_hashes {
            self.assembly_definition_geometry.remove(hash);
            self.assembly_unresolved_definitions.remove(hash);
        }
        self.assembly_definition_geometry
            .insert(result.definition_model_hash, result.display_bodies);
        self.assembly_unresolved_definitions
            .remove(&result.definition_model_hash);
        if self.assembly_document.mates.is_some() {
            let context = self.assembly_mate_context();
            let solution =
                zerocad_core::solve_assembly_mates_committed(&self.assembly_document, &context);
            if solution.converged {
                let _ = zerocad_core::apply_mate_solution(&mut self.assembly_document, &solution);
            }
            self.assembly_mate_statuses = solution.statuses;
        }
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
        self.error_msg = None;
        self.status_msg = if result.diagnostics.is_empty() {
            format!(
                "Replaced {} component(s).",
                result.replaced_occurrence_ids.len()
            )
        } else {
            format!(
                "Replaced {} component(s) with {} warning(s).",
                result.replaced_occurrence_ids.len(),
                result.diagnostics.len()
            )
        };
    }

    pub(crate) fn hydrate_assembly_definitions(&mut self) {
        // Reading V1 is supported, but a V2-capable application upgrades it
        // explicitly in memory so the next save cannot silently discard the
        // mate-capable schema boundary.
        self.assembly_document
            .mates
            .get_or_insert_with(Default::default);
        self.assembly_definition_geometry.clear();
        self.assembly_unresolved_definitions.clear();
        for definition in self.assembly_document.definitions.values() {
            match zerocad_core::prepare_part_definition(
                definition.compact_snapshot.as_slice(),
                definition.source_basename.as_deref(),
                &zerocad_core::LoadOptions::default(),
            ) {
                Ok(prepared) if prepared.model_hash == definition.model_hash => {
                    self.assembly_definition_geometry
                        .insert(definition.model_hash, prepared.display_bodies);
                }
                Ok(_) => {
                    self.assembly_unresolved_definitions.insert(
                        definition.model_hash,
                        "evaluated snapshot changed model identity".into(),
                    );
                }
                Err(error) => {
                    self.assembly_unresolved_definitions
                        .insert(definition.model_hash, error.to_string());
                }
            }
        }
        self.solve_loaded_assembly_mates();
        self.rebuild_assembly_scene();
    }

    fn solve_loaded_assembly_mates(&mut self) {
        if self.assembly_document.mates.is_none() {
            self.assembly_mate_statuses.clear();
            return;
        }
        let context = self.assembly_mate_context();
        let solution =
            zerocad_core::solve_assembly_mates_committed(&self.assembly_document, &context);
        self.assembly_mate_statuses = solution.statuses.clone();
        if solution.converged {
            let _ = zerocad_core::apply_mate_solution(&mut self.assembly_document, &solution);
        } else if !solution.diagnostics.is_empty() {
            self.status_msg = format!(
                "Assembly opened with mate diagnostics: {}",
                solution.diagnostics.join("; ")
            );
        }
    }

    fn assembly_mate_context(&self) -> zerocad_core::MateSolveContext {
        let mut context = zerocad_core::MateSolveContext::default();
        if let Some(mate_set) = &self.assembly_document.mates {
            for mate in mate_set.mates.values() {
                let first = self.resolve_assembly_mate_entity(&mate.first);
                let second = mate
                    .second
                    .as_ref()
                    .and_then(|entity| self.resolve_assembly_mate_entity(entity));
                if let Some(first) = first {
                    context
                        .frames
                        .insert(mate.id, zerocad_core::ResolvedMateFrames { first, second });
                }
            }
        }
        let mut world_min = [f64::INFINITY; 3];
        let mut world_max = [f64::NEG_INFINITY; 3];
        for occurrence in self.assembly_document.occurrences.values() {
            let Some(bodies) = self
                .assembly_definition_geometry
                .get(&occurrence.definition_model_hash)
            else {
                continue;
            };
            let mut local_min = [f64::INFINITY; 3];
            let mut local_max = [f64::NEG_INFINITY; 3];
            for (_, mesh) in bodies.iter() {
                for vertex in mesh.vertices.chunks_exact(6) {
                    for axis in 0..3 {
                        local_min[axis] = local_min[axis].min(vertex[axis] as f64);
                        local_max[axis] = local_max[axis].max(vertex[axis] as f64);
                    }
                }
            }
            if local_min[0].is_finite() {
                let diagonal = ((local_max[0] - local_min[0]).powi(2)
                    + (local_max[1] - local_min[1]).powi(2)
                    + (local_max[2] - local_min[2]).powi(2))
                .sqrt();
                context
                    .characteristic_lengths_mm
                    .insert(occurrence.id, (0.5 * diagonal).max(1.0));
                let placement = occurrence.resolved_placement().to_scene_placement();
                let (minimum, maximum) = placement.transform_bounds(
                    local_min.map(|value| value as f32),
                    local_max.map(|value| value as f32),
                );
                for axis in 0..3 {
                    world_min[axis] = world_min[axis].min(minimum[axis] as f64);
                    world_max[axis] = world_max[axis].max(maximum[axis] as f64);
                }
            }
        }
        context.assembly_diagonal_mm = if world_min[0].is_finite() {
            ((world_max[0] - world_min[0]).powi(2)
                + (world_max[1] - world_min[1]).powi(2)
                + (world_max[2] - world_min[2]).powi(2))
            .sqrt()
        } else {
            1.0
        };
        context
    }

    fn resolve_assembly_mate_entity(
        &self,
        entity: &zerocad_core::AssemblyEntityRef,
    ) -> Option<zerocad_core::MateFrame> {
        let occurrence = self
            .assembly_document
            .occurrences
            .get(&entity.occurrence_id)?;
        let bodies = self
            .assembly_definition_geometry
            .get(&occurrence.definition_model_hash)?;
        let mesh = &bodies
            .iter()
            .find(|(body_id, _)| body_id == &entity.local_body_id)?
            .1;
        match &entity.local_selector {
            zerocad_core::AssemblyLocalSelector::Origin => Some(zerocad_core::MateFrame {
                origin: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                radial: [1.0, 0.0, 0.0],
            }),
            zerocad_core::AssemblyLocalSelector::Face { stable_id } => {
                let face = mesh.face_refs.iter().find(|face| {
                    face.topology
                        .as_ref()
                        .and_then(|topology| topology.face_id.as_deref())
                        == Some(stable_id.as_str())
                        || stable_id == &format!("mesh-face:{}", face.face_id)
                })?;
                Some(mate_frame_from_face(face.centroid, face.normal))
            }
            zerocad_core::AssemblyLocalSelector::Edge { stable_id } => {
                let edge = mesh.edge_refs.iter().find(|edge| {
                    edge.topology
                        .as_ref()
                        .and_then(|topology| topology.edge_id.as_deref())
                        == Some(stable_id.as_str())
                        || stable_id == &format!("mesh-edge:{}", edge.group)
                })?;
                mate_frame_from_edge(edge)
            }
            zerocad_core::AssemblyLocalSelector::Axis { stable_id } => {
                let axis = match stable_id.to_ascii_lowercase().as_str() {
                    "x" | "global-x" => [1.0, 0.0, 0.0],
                    "y" | "global-y" => [0.0, 1.0, 0.0],
                    "z" | "global-z" => [0.0, 0.0, 1.0],
                    _ => return None,
                };
                Some(zerocad_core::MateFrame {
                    origin: [0.0; 3],
                    axis,
                    radial: least_aligned_radial(axis),
                })
            }
        }
    }

    pub(crate) fn rebuild_assembly_scene(&mut self) {
        let mut geometries = Vec::new();
        let mut body_indices = BTreeMap::<(zerocad_core::ModelHash, String), usize>::new();
        for (definition_hash, bodies) in &self.assembly_definition_geometry {
            for (local_body_id, mesh) in bodies.iter() {
                let index = geometries.len();
                geometries.push((
                    format!(
                        "definition-{}/{}",
                        short_hash(definition_hash),
                        local_body_id
                    ),
                    mesh.clone(),
                ));
                body_indices.insert((*definition_hash, local_body_id.clone()), index);
            }
        }

        let mut instances = Vec::new();
        let mut entity_lookup = HashMap::new();
        for occurrence in self.assembly_document.occurrences.values() {
            if self
                .assembly_document
                .presentation
                .hidden_occurrences
                .contains(&occurrence.id)
            {
                continue;
            }
            let Some(bodies) = self
                .assembly_definition_geometry
                .get(&occurrence.definition_model_hash)
            else {
                continue;
            };
            for (body_ordinal, (local_body_id, _)) in bodies.iter().enumerate() {
                if self
                    .assembly_document
                    .presentation
                    .hidden_bodies
                    .contains(&(occurrence.id, local_body_id.clone()))
                {
                    continue;
                }
                let Some(&geometry_index) =
                    body_indices.get(&(occurrence.definition_model_hash, local_body_id.clone()))
                else {
                    continue;
                };
                let entity_id = format!("assembly-entity-{}-{body_ordinal}", occurrence.id);
                entity_lookup.insert(entity_id.clone(), (occurrence.id, local_body_id.clone()));
                instances.push(zerocad_core::SceneInstance::new(
                    entity_id,
                    geometry_index,
                    occurrence.resolved_placement().to_scene_placement(),
                ));
            }
        }
        let scene = EvaluatedScene::from_instances(std::sync::Arc::new(geometries), instances)
            .expect("assembly scene identities and geometry indices are constructed together");
        self.assembly_scene_entities = entity_lookup;
        self.replace_evaluated_scene(std::sync::Arc::new(scene));
    }

    fn assembly_placement_gizmo(
        &mut self,
        painter: &egui::Painter,
        response: &egui::Response,
        rect: egui::Rect,
    ) -> bool {
        let Some(occurrence_id) = self.selected_assembly_occurrence else {
            self.assembly_gizmo_drag = None;
            return false;
        };
        let Some(occurrence) = self
            .assembly_document
            .occurrences
            .get(&occurrence_id)
            .cloned()
        else {
            self.assembly_gizmo_drag = None;
            return false;
        };
        let placement = self
            .assembly_interactive_target
            .filter(|(id, _)| id == &occurrence_id)
            .map(|(_, target)| target)
            .unwrap_or_else(|| occurrence.resolved_placement());
        let center = placement.translation().map(|value| value as f32);
        let scene_placement = placement.to_scene_placement();
        let globals = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let axes = if self.assembly_gizmo_local_space {
            globals.map(|axis| scene_placement.transform_vector(axis))
        } else {
            globals
        };
        let center_x = rect.center().x + self.camera_pan.x;
        let center_y = rect.center().y + self.camera_pan.y;
        let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
        let (sin_p, cos_p) = self.camera_pitch.sin_cos();
        let (sin_y, cos_y) = self.camera_yaw.sin_cos();
        let perspective = self.is_perspective;
        let project = |point: [f32; 3]| {
            let rx = cos_y * point[0] - sin_y * point[2];
            let rz = sin_y * point[0] + cos_y * point[2];
            let ry = cos_p * point[1] - sin_p * rz;
            let final_z = sin_p * point[1] + cos_p * rz;
            let factor = if perspective {
                let distance = crate::render::PERSP_DIST;
                distance / (distance - final_z.min(distance * 0.85))
            } else {
                1.0
            };
            egui::pos2(
                center_x + rx * view_scale * factor,
                center_y - ry * view_scale * factor,
            )
        };
        let origin = project(center);
        let length = self.camera_zoom * 0.65;
        let colors = [
            egui::Color32::from_rgb(220, 55, 55),
            egui::Color32::from_rgb(40, 175, 75),
            egui::Color32::from_rgb(55, 105, 225),
        ];
        let mut translation_handles = Vec::new();
        let mut rotation_handles = Vec::<(usize, egui::Pos2, egui::Pos2)>::new();
        match self.assembly_gizmo_mode {
            AssemblyGizmoMode::Translate => {
                for axis in 0..3 {
                    let tip_world = [
                        center[0] + axes[axis][0] * length,
                        center[1] + axes[axis][1] * length,
                        center[2] + axes[axis][2] * length,
                    ];
                    let tip = project(tip_world);
                    painter.line_segment([origin, tip], egui::Stroke::new(3.0, colors[axis]));
                    painter.circle_filled(tip, 5.0, colors[axis]);
                    painter.text(
                        tip + egui::vec2(7.0, -7.0),
                        egui::Align2::LEFT_BOTTOM,
                        ["X", "Y", "Z"][axis],
                        egui::FontId::proportional(12.0),
                        colors[axis],
                    );
                    translation_handles.push((axis, tip, tip - origin));
                }
            }
            AssemblyGizmoMode::Rotate => {
                for axis_index in 0..3 {
                    let axis = axes[axis_index].map(|value| value as f64);
                    let u = least_aligned_radial(axis).map(|value| value as f32);
                    let v = [
                        axes[axis_index][1] * u[2] - axes[axis_index][2] * u[1],
                        axes[axis_index][2] * u[0] - axes[axis_index][0] * u[2],
                        axes[axis_index][0] * u[1] - axes[axis_index][1] * u[0],
                    ];
                    let mut previous = None;
                    for sample in 0..=48 {
                        let angle = std::f32::consts::TAU * sample as f32 / 48.0;
                        let point = [
                            center[0] + length * 0.72 * (u[0] * angle.cos() + v[0] * angle.sin()),
                            center[1] + length * 0.72 * (u[1] * angle.cos() + v[1] * angle.sin()),
                            center[2] + length * 0.72 * (u[2] * angle.cos() + v[2] * angle.sin()),
                        ];
                        let screen = project(point);
                        if let Some(start) = previous {
                            painter.line_segment(
                                [start, screen],
                                egui::Stroke::new(2.5, colors[axis_index]),
                            );
                            rotation_handles.push((axis_index, start, screen));
                        }
                        previous = Some(screen);
                    }
                }
            }
        }

        let mut consumed = self.assembly_gizmo_drag.is_some();
        if response.drag_started_by(egui::PointerButton::Primary) && !occurrence.grounded {
            if let Some(pointer) = response.interact_pointer_pos() {
                let hit = match self.assembly_gizmo_mode {
                    AssemblyGizmoMode::Translate => translation_handles
                        .iter()
                        .filter_map(|(axis, tip, direction)| {
                            let distance =
                                crate::geom2d::dist_point_to_segment(pointer, origin, *tip);
                            (distance <= 10.0 && direction.length() > 1.0)
                                .then_some((*axis, distance, *direction))
                        })
                        .min_by(|first, second| first.1.total_cmp(&second.1)),
                    AssemblyGizmoMode::Rotate => rotation_handles
                        .iter()
                        .filter_map(|(axis, start, end)| {
                            let distance =
                                crate::geom2d::dist_point_to_segment(pointer, *start, *end);
                            let direction = *end - *start;
                            (distance <= 10.0 && direction.length() > 1.0)
                                .then_some((*axis, distance, direction))
                        })
                        .min_by(|first, second| first.1.total_cmp(&second.1)),
                };
                if let Some((axis, _, screen_direction)) = hit {
                    self.assembly_gizmo_drag = Some(AssemblyGizmoDrag {
                        occurrence_id,
                        axis,
                        start_pointer: pointer,
                        screen_direction: screen_direction.normalized(),
                        start_placement: placement,
                    });
                    consumed = true;
                }
            }
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            if let (Some(drag), Some(pointer)) =
                (self.assembly_gizmo_drag, response.interact_pointer_pos())
            {
                let projected_delta = (pointer - drag.start_pointer).dot(drag.screen_direction);
                let target = match self.assembly_gizmo_mode {
                    AssemblyGizmoMode::Translate => {
                        let pixels_per_world =
                            (translation_handles[drag.axis].2.length() / length).max(1.0e-4);
                        let delta = projected_delta / pixels_per_world;
                        let mut translation = drag.start_placement.translation();
                        for coordinate in 0..3 {
                            translation[coordinate] +=
                                axes[drag.axis][coordinate] as f64 * delta as f64;
                        }
                        drag.start_placement.with_translation(translation).ok()
                    }
                    AssemblyGizmoMode::Rotate => {
                        let angle = projected_delta as f64 / 60.0;
                        let axis = if self.assembly_gizmo_local_space {
                            globals[drag.axis].map(|value| value as f64)
                        } else {
                            axes[drag.axis].map(|value| value as f64)
                        };
                        drag.start_placement
                            .rotated_about_axis(axis, angle, !self.assembly_gizmo_local_space)
                            .ok()
                    }
                };
                if let Some(target) = target {
                    self.preview_assembly_occurrence_placement(drag.occurrence_id, target);
                }
                consumed = true;
            }
        }
        if response.drag_stopped_by(egui::PointerButton::Primary)
            && self.assembly_gizmo_drag.take().is_some()
        {
            self.finish_assembly_transform_edit();
            consumed = true;
        }
        consumed
    }

    fn handle_assembly_viewport_input(
        &mut self,
        response: &egui::Response,
        rect: egui::Rect,
        ctx: &egui::Context,
    ) {
        let pointer_delta = ctx.input(|input| input.pointer.delta());
        let shift = ctx.input(|input| input.modifiers.shift);
        let middle_down = ctx.input(|input| input.pointer.middle_down());
        let middle_pressed =
            ctx.input(|input| input.pointer.button_pressed(egui::PointerButton::Middle));
        if middle_pressed && response.hovered() {
            self.orbiting = true;
        }
        if !middle_down {
            self.orbiting = false;
        }
        if self.orbiting {
            if shift {
                self.camera_pan += pointer_delta;
            } else {
                (self.camera_pitch, self.camera_yaw) = super::viewport::orbit_camera_angles(
                    self.camera_pitch,
                    self.camera_yaw,
                    pointer_delta,
                );
            }
        }
        let scroll = ctx.input(|input| input.smooth_scroll_delta.y);
        self.camera_zoom = super::viewport::viewport_zoom_after_scroll(
            self.camera_zoom,
            scroll,
            response.hovered(),
        );

        let Some(click) = response
            .clicked()
            .then(|| response.interact_pointer_pos())
            .flatten()
        else {
            return;
        };
        let center_x = rect.center().x + self.camera_pan.x;
        let center_y = rect.center().y + self.camera_pan.y;
        let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
        let (sin_p, cos_p) = self.camera_pitch.sin_cos();
        let (sin_y, cos_y) = self.camera_yaw.sin_cos();
        let perspective = self.is_perspective;
        let project = |x: f32, y: f32, z: f32| {
            let rx = cos_y * x - sin_y * z;
            let rz = sin_y * x + cos_y * z;
            let ry = cos_p * y - sin_p * rz;
            let final_z = sin_p * y + cos_p * rz;
            if perspective {
                let distance = crate::render::PERSP_DIST;
                let factor = distance / (distance - final_z.min(distance * 0.85));
                (
                    center_x + rx * view_scale * factor,
                    center_y - ry * view_scale * factor,
                    final_z,
                )
            } else {
                (
                    center_x + rx * view_scale,
                    center_y - ry * view_scale,
                    final_z,
                )
            }
        };
        let gpu_face = self.gpu_pick_face(click, rect, ctx.pixels_per_point());
        if let Some((entity_id, pick)) =
            self.pick_body_element(click, &project, sin_p, cos_p, sin_y, cos_y, gpu_face)
        {
            let multi_select = ctx.input(|input| input.modifiers.shift || input.modifiers.ctrl);
            self.selected_assembly_occurrence = self
                .assembly_scene_entities
                .get(&entity_id)
                .map(|(occurrence_id, _)| *occurrence_id);
            self.select_body_hit(entity_id, pick, false, multi_select);
            if let Some(id) = self.selected_assembly_occurrence {
                self.status_msg = format!("Selected component {id}.");
            }
        } else if !ctx.input(|input| input.modifiers.shift || input.modifiers.ctrl) {
            self.selected_assembly_occurrence = None;
            self.selected_body.clear();
        }
    }

    fn align_selected_assembly_faces(&mut self) {
        let Some(moving_occurrence_id) = self.selected_assembly_occurrence else {
            self.status_msg = "Select the target face, then Shift-select the moving face.".into();
            return;
        };
        let face_selections: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(entity, pick)| match pick {
                BodyPick::Face(face_id) => Some((entity.clone(), *face_id)),
                _ => None,
            })
            .collect();
        if face_selections.len() != 2 {
            self.status_msg = "Align Faces requires exactly two selected faces.".into();
            return;
        }
        let resolve = |entity: &str, face_id: u32| {
            let (occurrence_id, local_body_id) = self.assembly_scene_entities.get(entity)?.clone();
            let occurrence = self.assembly_document.occurrences.get(&occurrence_id)?;
            let bodies = self
                .assembly_definition_geometry
                .get(&occurrence.definition_model_hash)?;
            let mesh = &bodies
                .iter()
                .find(|(body_id, _)| body_id == &local_body_id)?
                .1;
            let face = mesh.face_refs.iter().find(|face| face.face_id == face_id)?;
            Some((
                occurrence_id,
                occurrence.resolved_placement(),
                zerocad_core::AlignFaceFrame {
                    centroid: face.centroid.map(|value| value as f64),
                    normal: face.normal.map(|value| value as f64),
                    // MockMesh does not yet retain a surface parameter frame.
                    // The core command therefore uses its documented canonical
                    // least-aligned-axis fallback for the 180-degree case.
                    parametric_u: None,
                },
            ))
        };
        let Some(first) = resolve(&face_selections[0].0, face_selections[0].1) else {
            self.status_msg = "The first selected face is no longer available.".into();
            return;
        };
        let Some(second) = resolve(&face_selections[1].0, face_selections[1].1) else {
            self.status_msg = "The second selected face is no longer available.".into();
            return;
        };
        let (moving, target) = if first.0 == moving_occurrence_id {
            (first, second)
        } else if second.0 == moving_occurrence_id {
            (second, first)
        } else {
            self.status_msg = "The moving component must own one selected face.".into();
            return;
        };
        if moving.0 == target.0 {
            self.status_msg = "Choose faces on two different components.".into();
            return;
        }
        if self.assembly_document.occurrences[&moving.0].grounded {
            self.status_msg = "Unground the moving component before aligning it.".into();
            return;
        }
        let solver_controlled = self
            .assembly_document
            .mates
            .as_ref()
            .is_some_and(|mate_set| {
                mate_set.mates.values().any(|mate| {
                    !mate.suppressed
                        && (mate.first.occurrence_id == moving.0
                            || mate
                                .second
                                .as_ref()
                                .is_some_and(|entity| entity.occurrence_id == moving.0))
                })
            });
        if solver_controlled {
            let Some((first_entity, first_frame)) =
                self.selected_face_entity(&face_selections[0].0, face_selections[0].1)
            else {
                self.status_msg = "The first selected face is no longer resolvable.".into();
                return;
            };
            let Some((second_entity, second_frame)) =
                self.selected_face_entity(&face_selections[1].0, face_selections[1].1)
            else {
                self.status_msg = "The second selected face is no longer resolvable.".into();
                return;
            };
            let (target_entity, target_frame, moving_entity, moving_frame) =
                if first_entity.occurrence_id == moving.0 {
                    (second_entity, second_frame, first_entity, first_frame)
                } else {
                    (first_entity, first_frame, second_entity, second_frame)
                };
            let temporary_mate = zerocad_core::AssemblyMate {
                id: 0,
                name: String::new(),
                suppressed: false,
                first: target_entity,
                second: Some(moving_entity),
                kind: zerocad_core::AssemblyMateKind::Coincident,
                sense: zerocad_core::MateSense::AntiAligned,
            };
            let context = self.assembly_mate_context();
            self.push_undo();
            match zerocad_core::apply_ephemeral_mate_target_transactionally(
                &mut self.assembly_document,
                moving.0,
                temporary_mate,
                &context,
                zerocad_core::ResolvedMateFrames {
                    first: target_frame,
                    second: Some(moving_frame),
                },
            ) {
                Ok(solution) => {
                    self.assembly_mate_statuses = solution.statuses;
                    self.document_revision = self.document_revision.wrapping_add(1);
                    self.recovery.note_edit(self.current_project_snapshot());
                    self.rebuild_assembly_scene();
                    self.status_msg =
                        "Aligned the solver-controlled component without creating a mate.".into();
                }
                Err(error) => {
                    self.undo_stack.pop();
                    self.status_msg =
                        format!("Align Faces conflicts with the active mate system: {error}");
                }
            }
            return;
        }
        let placement = match zerocad_core::align_faces(moving.1, moving.2, target.1, target.2) {
            Ok(placement) => placement,
            Err(error) => {
                self.status_msg = format!("Align Faces failed: {error}");
                return;
            }
        };
        self.push_undo();
        let occurrence = self
            .assembly_document
            .occurrences
            .get_mut(&moving.0)
            .expect("selected moving occurrence still exists");
        occurrence.manual_placement = placement;
        occurrence.resolved_placement_override = None;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
        self.status_msg = format!("Aligned component {} to component {}.", moving.0, target.0);
    }

    fn selected_face_entity(
        &self,
        entity_id: &str,
        face_id: u32,
    ) -> Option<(zerocad_core::AssemblyEntityRef, zerocad_core::MateFrame)> {
        let (occurrence_id, local_body_id) = self.assembly_scene_entities.get(entity_id)?.clone();
        let occurrence = self.assembly_document.occurrences.get(&occurrence_id)?;
        let bodies = self
            .assembly_definition_geometry
            .get(&occurrence.definition_model_hash)?;
        let mesh = &bodies
            .iter()
            .find(|(body_id, _)| body_id == &local_body_id)?
            .1;
        let face = mesh.face_refs.iter().find(|face| face.face_id == face_id)?;
        let stable_id = face
            .topology
            .as_ref()
            .and_then(|topology| topology.face_id.clone())
            .unwrap_or_else(|| format!("mesh-face:{face_id}"));
        Some((
            zerocad_core::AssemblyEntityRef {
                occurrence_id,
                local_body_id,
                local_selector: zerocad_core::AssemblyLocalSelector::Face { stable_id },
            },
            mate_frame_from_face(face.centroid, face.normal),
        ))
    }

    fn selected_edge_entity(
        &self,
        entity_id: &str,
        edge_id: u32,
    ) -> Option<(zerocad_core::AssemblyEntityRef, zerocad_core::MateFrame)> {
        let (occurrence_id, local_body_id) = self.assembly_scene_entities.get(entity_id)?.clone();
        let occurrence = self.assembly_document.occurrences.get(&occurrence_id)?;
        let bodies = self
            .assembly_definition_geometry
            .get(&occurrence.definition_model_hash)?;
        let mesh = &bodies
            .iter()
            .find(|(body_id, _)| body_id == &local_body_id)?
            .1;
        let edge = mesh.edge_refs.iter().find(|edge| edge.group == edge_id)?;
        let stable_id = edge
            .topology
            .as_ref()
            .and_then(|topology| topology.edge_id.clone())
            .unwrap_or_else(|| format!("mesh-edge:{edge_id}"));
        Some((
            zerocad_core::AssemblyEntityRef {
                occurrence_id,
                local_body_id,
                local_selector: zerocad_core::AssemblyLocalSelector::Edge { stable_id },
            },
            mate_frame_from_edge(edge)?,
        ))
    }

    fn create_face_mate(&mut self, mode: FaceMateMode) {
        let faces: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(entity, pick)| match pick {
                BodyPick::Face(face_id) => Some((entity.clone(), *face_id)),
                _ => None,
            })
            .collect();
        if faces.len() != 2 {
            self.status_msg =
                "Select one face, then Shift-select a face on another component.".into();
            return;
        }
        let Some((first, first_frame)) = self.selected_face_entity(&faces[0].0, faces[0].1) else {
            self.status_msg = "The first face is no longer resolvable.".into();
            return;
        };
        let Some((second, second_frame)) = self.selected_face_entity(&faces[1].0, faces[1].1)
        else {
            self.status_msg = "The second face is no longer resolvable.".into();
            return;
        };
        if first.occurrence_id == second.occurrence_id {
            self.status_msg = "A mate requires entities on different components.".into();
            return;
        }
        let next_id = self
            .assembly_document
            .mates
            .as_ref()
            .map_or(1, |mates| mates.next_mate_id);
        let mut context = self.assembly_mate_context();
        context.frames.insert(
            next_id,
            zerocad_core::ResolvedMateFrames {
                first: first_frame,
                second: Some(second_frame),
            },
        );
        let first_world = world_mate_frame(
            first_frame,
            self.assembly_document.occurrences[&first.occurrence_id].resolved_placement(),
        );
        let second_world = world_mate_frame(
            second_frame,
            self.assembly_document.occurrences[&second.occurrence_id].resolved_placement(),
        );
        let anti_aligned_second_axis = second_world.axis.map(|value| -value);
        let delta = [
            second_world.origin[0] - first_world.origin[0],
            second_world.origin[1] - first_world.origin[1],
            second_world.origin[2] - first_world.origin[2],
        ];
        let current_distance = dot3(delta, first_world.axis);
        let current_angle = dot3(first_world.axis, anti_aligned_second_axis)
            .clamp(-1.0, 1.0)
            .acos();
        let (name, kind) = match mode {
            FaceMateMode::Coincident => (
                format!("Coincident {next_id}"),
                zerocad_core::AssemblyMateKind::Coincident,
            ),
            FaceMateMode::SignedDistance => (
                format!("Distance {next_id}"),
                zerocad_core::AssemblyMateKind::SignedDistance {
                    millimeters: current_distance,
                },
            ),
            FaceMateMode::Angle => (
                format!("Angle {next_id}"),
                zerocad_core::AssemblyMateKind::Angle {
                    radians: current_angle,
                },
            ),
        };
        let mate = zerocad_core::AssemblyMate {
            id: next_id,
            name,
            suppressed: false,
            first,
            second: Some(second),
            kind,
            // Selected face normals oppose when the two outward surfaces meet.
            sense: zerocad_core::MateSense::AntiAligned,
        };
        self.push_undo();
        match zerocad_core::add_mate_transactionally(&mut self.assembly_document, mate, &context) {
            Ok(solution) => {
                self.assembly_mate_statuses = solution.statuses;
                self.document_revision = self.document_revision.wrapping_add(1);
                self.recovery.note_edit(self.current_project_snapshot());
                self.rebuild_assembly_scene();
                self.status_msg = format!("Created mate {next_id}.");
            }
            Err(error) => {
                self.undo_stack.pop();
                self.status_msg = format!("Mate rejected: {error}");
            }
        }
    }

    fn create_concentric_edge_mate(&mut self) {
        let edges: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(entity, pick)| match pick {
                BodyPick::Edge(edge_id) => Some((entity.clone(), *edge_id)),
                _ => None,
            })
            .collect();
        if edges.len() != 2 {
            self.status_msg =
                "Select one circular or linear edge, then Shift-select an edge on another component."
                    .into();
            return;
        }
        let Some((first, first_frame)) = self.selected_edge_entity(&edges[0].0, edges[0].1) else {
            self.status_msg = "The first selected edge has no stable axis.".into();
            return;
        };
        let Some((second, second_frame)) = self.selected_edge_entity(&edges[1].0, edges[1].1)
        else {
            self.status_msg = "The second selected edge has no stable axis.".into();
            return;
        };
        if first.occurrence_id == second.occurrence_id {
            self.status_msg = "A mate requires entities on different components.".into();
            return;
        }
        let next_id = self
            .assembly_document
            .mates
            .as_ref()
            .map_or(1, |mates| mates.next_mate_id);
        let mut context = self.assembly_mate_context();
        context.frames.insert(
            next_id,
            zerocad_core::ResolvedMateFrames {
                first: first_frame,
                second: Some(second_frame),
            },
        );
        let mate = zerocad_core::AssemblyMate {
            id: next_id,
            name: format!("Concentric {next_id}"),
            suppressed: false,
            first,
            second: Some(second),
            kind: zerocad_core::AssemblyMateKind::Concentric,
            sense: zerocad_core::MateSense::Aligned,
        };
        self.push_undo();
        match zerocad_core::add_mate_transactionally(&mut self.assembly_document, mate, &context) {
            Ok(solution) => {
                self.assembly_mate_statuses = solution.statuses;
                self.document_revision = self.document_revision.wrapping_add(1);
                self.recovery.note_edit(self.current_project_snapshot());
                self.rebuild_assembly_scene();
                self.status_msg = format!("Created concentric mate {next_id}.");
            }
            Err(error) => {
                self.undo_stack.pop();
                self.status_msg = format!("Mate rejected: {error}");
            }
        }
    }

    fn create_fixed_mate(&mut self) {
        let Some(occurrence_id) = self.selected_assembly_occurrence else {
            self.status_msg = "Select a component to fix.".into();
            return;
        };
        let occurrence = &self.assembly_document.occurrences[&occurrence_id];
        let Some(local_body_id) = self
            .assembly_definition_geometry
            .get(&occurrence.definition_model_hash)
            .and_then(|bodies| bodies.first())
            .map(|(body_id, _)| body_id.clone())
        else {
            self.status_msg = "The selected component has no resolved body.".into();
            return;
        };
        let next_id = self
            .assembly_document
            .mates
            .as_ref()
            .map_or(1, |mates| mates.next_mate_id);
        let mate = zerocad_core::AssemblyMate {
            id: next_id,
            name: format!("Fixed {next_id}"),
            suppressed: false,
            first: zerocad_core::AssemblyEntityRef {
                occurrence_id,
                local_body_id,
                local_selector: zerocad_core::AssemblyLocalSelector::Origin,
            },
            second: None,
            kind: zerocad_core::AssemblyMateKind::Fixed,
            sense: zerocad_core::MateSense::Aligned,
        };
        let context = self.assembly_mate_context();
        self.push_undo();
        match zerocad_core::add_mate_transactionally(&mut self.assembly_document, mate, &context) {
            Ok(solution) => {
                self.assembly_mate_statuses = solution.statuses;
                self.document_revision = self.document_revision.wrapping_add(1);
                self.recovery.note_edit(self.current_project_snapshot());
                self.rebuild_assembly_scene();
                self.status_msg = format!("Fixed component {occurrence_id}.");
            }
            Err(error) => {
                self.undo_stack.pop();
                self.status_msg = format!("Mate rejected: {error}");
            }
        }
    }

    fn edit_assembly_mate(&mut self, mate_id: u64, action: MateEditAction) {
        let context = self.assembly_mate_context();
        let mut candidate = self.assembly_document.clone_authoritative();
        let Some(mate_set) = candidate.mates.as_mut() else {
            return;
        };
        match action {
            MateEditAction::Delete => {
                mate_set.mates.remove(&mate_id);
            }
            MateEditAction::SetSuppressed(suppressed) => {
                if let Some(mate) = mate_set.mates.get_mut(&mate_id) {
                    mate.suppressed = suppressed;
                }
            }
            MateEditAction::FlipSense => {
                if let Some(mate) = mate_set.mates.get_mut(&mate_id) {
                    mate.sense = match mate.sense {
                        zerocad_core::MateSense::Aligned => zerocad_core::MateSense::AntiAligned,
                        zerocad_core::MateSense::AntiAligned => zerocad_core::MateSense::Aligned,
                    };
                }
            }
            MateEditAction::SetParameter(value) => {
                if let Some(mate) = mate_set.mates.get_mut(&mate_id) {
                    match &mut mate.kind {
                        zerocad_core::AssemblyMateKind::SignedDistance { millimeters } => {
                            *millimeters = value;
                        }
                        zerocad_core::AssemblyMateKind::Angle { radians } => {
                            *radians = value.to_radians();
                        }
                        _ => return,
                    }
                }
            }
        }
        let solution = zerocad_core::solve_assembly_mates_committed(&candidate, &context);
        if !solution.converged {
            self.status_msg =
                "Mate edit was rejected because the remaining system did not solve.".into();
            return;
        }
        let _ = zerocad_core::apply_mate_solution(&mut candidate, &solution);
        self.push_undo();
        self.assembly_document = candidate;
        self.assembly_mate_statuses = solution.statuses;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn repair_assembly_mate_from_selected_faces(&mut self, mate_id: u64) {
        let faces: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(entity, pick)| match pick {
                BodyPick::Face(face_id) => Some((entity.clone(), *face_id)),
                _ => None,
            })
            .collect();
        if faces.len() != 2 {
            self.status_msg = "Repair requires two selected faces on different components.".into();
            return;
        }
        let Some((first, first_frame)) = self.selected_face_entity(&faces[0].0, faces[0].1) else {
            self.status_msg = "The first repair face is not resolvable.".into();
            return;
        };
        let Some((second, second_frame)) = self.selected_face_entity(&faces[1].0, faces[1].1)
        else {
            self.status_msg = "The second repair face is not resolvable.".into();
            return;
        };
        if first.occurrence_id == second.occurrence_id {
            self.status_msg = "Repair entities must belong to different components.".into();
            return;
        }
        let mut candidate = self.assembly_document.clone_authoritative();
        let Some(mate) = candidate
            .mates
            .as_mut()
            .and_then(|mate_set| mate_set.mates.get_mut(&mate_id))
        else {
            return;
        };
        if matches!(mate.kind, zerocad_core::AssemblyMateKind::Fixed) {
            self.status_msg = "A fixed mate has no second selector to repair.".into();
            return;
        }
        mate.first = first;
        mate.second = Some(second);
        let mut context = self.assembly_mate_context();
        context.frames.insert(
            mate_id,
            zerocad_core::ResolvedMateFrames {
                first: first_frame,
                second: Some(second_frame),
            },
        );
        let solution = zerocad_core::solve_assembly_mates_committed(&candidate, &context);
        if !solution.converged {
            self.status_msg = "The repaired selectors conflict with the active mate system.".into();
            return;
        }
        let _ = zerocad_core::apply_mate_solution(&mut candidate, &solution);
        self.push_undo();
        self.assembly_document = candidate;
        self.assembly_mate_statuses = solution.statuses;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
        self.status_msg = format!("Repaired mate {mate_id}.");
    }

    fn set_assembly_occurrence_visible(&mut self, occurrence_id: u64, visible: bool) {
        self.push_undo();
        if visible {
            self.assembly_document
                .presentation
                .hidden_occurrences
                .remove(&occurrence_id);
        } else {
            self.assembly_document
                .presentation
                .hidden_occurrences
                .insert(occurrence_id);
        }
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn set_assembly_body_visible(
        &mut self,
        occurrence_id: u64,
        local_body_id: String,
        visible: bool,
    ) {
        self.push_undo();
        let selector = (occurrence_id, local_body_id);
        if visible {
            self.assembly_document
                .presentation
                .hidden_bodies
                .remove(&selector);
        } else {
            self.assembly_document
                .presentation
                .hidden_bodies
                .insert(selector);
        }
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn set_assembly_occurrence_grounded(&mut self, occurrence_id: u64, grounded: bool) {
        let should_change = self
            .assembly_document
            .occurrences
            .get(&occurrence_id)
            .is_some_and(|occurrence| occurrence.grounded != grounded);
        if !should_change {
            return;
        }
        let mut candidate = self.assembly_document.clone_authoritative();
        let Some(occurrence) = candidate.occurrences.get_mut(&occurrence_id) else {
            return;
        };
        occurrence.grounded = grounded;
        if candidate.mates.is_some() {
            let context = self.assembly_mate_context();
            let solution = zerocad_core::solve_assembly_mates_committed(&candidate, &context);
            if !solution.converged {
                self.status_msg = "Grounding change conflicts with the active mate system.".into();
                return;
            }
            let _ = zerocad_core::apply_mate_solution(&mut candidate, &solution);
            self.assembly_mate_statuses = solution.statuses;
        }
        self.push_undo();
        self.assembly_document = candidate;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn commit_assembly_occurrence_rename(&mut self, occurrence_id: u64, name: String) {
        let name = name.trim().to_owned();
        if name.is_empty() {
            self.status_msg = "Component name cannot be empty.".into();
            return;
        }
        if self
            .assembly_document
            .occurrences
            .values()
            .any(|occurrence| {
                occurrence.id != occurrence_id && occurrence.name.eq_ignore_ascii_case(&name)
            })
        {
            self.status_msg = format!("A component named `{name}` already exists.");
            return;
        }
        let unchanged = self
            .assembly_document
            .occurrences
            .get(&occurrence_id)
            .is_none_or(|occurrence| occurrence.name == name);
        if unchanged {
            self.assembly_rename_occurrence = None;
            return;
        }
        self.push_undo();
        if let Some(occurrence) = self.assembly_document.occurrences.get_mut(&occurrence_id) {
            occurrence.name = name;
        }
        self.assembly_rename_occurrence = None;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
    }

    fn set_assembly_occurrence_transform(
        &mut self,
        occurrence_id: u64,
        translation: [f64; 3],
        euler_degrees: [f64; 3],
    ) {
        let Ok(placement) =
            zerocad_core::RigidPlacement::from_euler_xyz_degrees(translation, euler_degrees)
        else {
            self.status_msg = "Placement contains an invalid number.".into();
            return;
        };
        self.preview_assembly_occurrence_placement(occurrence_id, placement);
    }

    fn preview_assembly_occurrence_placement(
        &mut self,
        occurrence_id: u64,
        placement: zerocad_core::RigidPlacement,
    ) {
        let Some(current) = self
            .assembly_document
            .occurrences
            .get(&occurrence_id)
            .cloned()
        else {
            return;
        };
        if current.grounded {
            self.status_msg = "Unground the component before moving it.".into();
            return;
        }
        let solver_controlled = self
            .assembly_document
            .mates
            .as_ref()
            .is_some_and(|mate_set| {
                mate_set.mates.values().any(|mate| {
                    !mate.suppressed
                        && (mate.first.occurrence_id == occurrence_id
                            || mate
                                .second
                                .as_ref()
                                .is_some_and(|entity| entity.occurrence_id == occurrence_id))
                })
            });
        let current_target = self
            .assembly_interactive_target
            .filter(|(id, _)| id == &occurrence_id)
            .map(|(_, target)| target)
            .unwrap_or(current.manual_placement);
        if placement == current_target {
            return;
        }
        if !self.assembly_transform_edit_active {
            self.push_undo();
            self.assembly_transform_edit_active = true;
        }
        if solver_controlled {
            self.assembly_interactive_target = Some((occurrence_id, placement));
            let mut context = self.assembly_mate_context();
            context
                .temporary_pose_targets
                .insert(occurrence_id, placement);
            let started = std::time::Instant::now();
            let solution =
                zerocad_core::solve_assembly_mates_interactive(&self.assembly_document, &context);
            if started.elapsed() <= std::time::Duration::from_millis(4) && solution.converged {
                let _ = zerocad_core::apply_mate_solution(&mut self.assembly_document, &solution);
                self.assembly_mate_statuses = solution.statuses;
                self.status_msg = "Previewing solver-constrained placement.".into();
            } else {
                self.status_msg =
                    "Mate preview reached its frame budget; showing the last solved placement."
                        .into();
            }
            self.rebuild_assembly_scene();
            return;
        }
        let occurrence = self
            .assembly_document
            .occurrences
            .get_mut(&occurrence_id)
            .unwrap();
        occurrence.manual_placement = placement;
        occurrence.resolved_placement_override = None;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn finish_assembly_transform_edit(&mut self) {
        if !self.assembly_transform_edit_active {
            return;
        }
        self.assembly_transform_edit_active = false;
        let Some((occurrence_id, target)) = self.assembly_interactive_target.take() else {
            return;
        };
        let context = self.assembly_mate_context();
        match zerocad_core::apply_pose_target_transactionally(
            &mut self.assembly_document,
            occurrence_id,
            target,
            &context,
        ) {
            Ok(solution) => {
                self.assembly_mate_statuses = solution.statuses;
                self.document_revision = self.document_revision.wrapping_add(1);
                self.recovery.note_edit(self.current_project_snapshot());
                self.status_msg = "Moved solver-constrained component.".into();
            }
            Err(error) => {
                self.undo_stack.pop();
                self.solve_loaded_assembly_mates();
                self.status_msg = format!(
                    "Placement target conflicts with active mates; the component was restored: {error}"
                );
            }
        }
        self.rebuild_assembly_scene();
    }

    fn duplicate_selected_assembly_occurrence(&mut self) {
        let Some(source_id) = self.selected_assembly_occurrence else {
            return;
        };
        let Some(source) = self.assembly_document.occurrences.get(&source_id).cloned() else {
            return;
        };
        if self.assembly_document.occurrences.len() >= 10_000 {
            self.status_msg = "Assembly occurrence limit reached.".into();
            return;
        }
        let new_id = self.assembly_document.next_occurrence_id;
        let Some(next_id) = new_id.checked_add(1) else {
            self.status_msg = "Assembly occurrence id space is exhausted.".into();
            return;
        };
        let definition_name = self.assembly_document.definitions[&source.definition_model_hash]
            .name
            .clone();
        self.push_undo();
        self.assembly_document.occurrences.insert(
            new_id,
            zerocad_core::AssemblyOccurrence {
                id: new_id,
                definition_model_hash: source.definition_model_hash,
                name: format!("{definition_name}:{new_id}"),
                manual_placement: source.manual_placement,
                grounded: false,
                resolved_placement_override: None,
            },
        );
        self.assembly_document.next_occurrence_id = next_id;
        if self
            .assembly_document
            .presentation
            .hidden_occurrences
            .contains(&source_id)
        {
            self.assembly_document
                .presentation
                .hidden_occurrences
                .insert(new_id);
        }
        let hidden_source_bodies: Vec<String> = self
            .assembly_document
            .presentation
            .hidden_bodies
            .iter()
            .filter(|(id, _)| *id == source_id)
            .map(|(_, body_id)| body_id.clone())
            .collect();
        self.assembly_document.presentation.hidden_bodies.extend(
            hidden_source_bodies
                .into_iter()
                .map(|body_id| (new_id, body_id)),
        );
        self.selected_assembly_occurrence = Some(new_id);
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn delete_selected_assembly_occurrence(&mut self) {
        let Some(occurrence_id) = self.selected_assembly_occurrence else {
            return;
        };
        let Some(occurrence) = self
            .assembly_document
            .occurrences
            .get(&occurrence_id)
            .cloned()
        else {
            return;
        };
        self.push_undo();
        self.assembly_document.occurrences.remove(&occurrence_id);
        self.assembly_document
            .presentation
            .hidden_occurrences
            .remove(&occurrence_id);
        self.assembly_document
            .presentation
            .hidden_bodies
            .retain(|(id, _)| *id != occurrence_id);
        if let Some(mate_set) = self.assembly_document.mates.as_mut() {
            mate_set.mates.retain(|_, mate| {
                mate.first.occurrence_id != occurrence_id
                    && mate
                        .second
                        .as_ref()
                        .is_none_or(|entity| entity.occurrence_id != occurrence_id)
            });
        }
        let definition_still_used = self
            .assembly_document
            .occurrences
            .values()
            .any(|remaining| remaining.definition_model_hash == occurrence.definition_model_hash);
        if !definition_still_used {
            self.assembly_document
                .definitions
                .remove(&occurrence.definition_model_hash);
            self.assembly_definition_geometry
                .remove(&occurrence.definition_model_hash);
            self.assembly_unresolved_definitions
                .remove(&occurrence.definition_model_hash);
        }
        if self.assembly_document.mates.is_some() {
            let context = self.assembly_mate_context();
            let solution =
                zerocad_core::solve_assembly_mates_committed(&self.assembly_document, &context);
            if solution.converged {
                let _ = zerocad_core::apply_mate_solution(&mut self.assembly_document, &solution);
                self.assembly_mate_statuses = solution.statuses;
            }
        }
        // Never auto-ground another occurrence. Future solver state must not
        // move merely because a grounded component was deleted.
        self.selected_assembly_occurrence = None;
        self.document_revision = self.document_revision.wrapping_add(1);
        self.recovery.note_edit(self.current_project_snapshot());
        self.rebuild_assembly_scene();
    }

    fn export_assembly_bom_csv(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export Assembly BOM")
            .set_file_name("assembly-bom.csv")
            .add_filter("CSV", &["csv"])
            .save_file()
        else {
            return;
        };
        match std::fs::write(
            &path,
            zerocad_core::assembly_bom_csv(&self.assembly_document),
        ) {
            Ok(()) => self.status_msg = format!("BOM exported to {}", path.display()),
            Err(error) => {
                self.status_msg = format!("BOM export failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
            }
        }
    }

    fn export_assembly_mesh(&mut self, three_mf: bool) {
        let (title, extension, default_name) = if three_mf {
            ("Export Assembly as 3MF", "3mf", "assembly.3mf")
        } else {
            ("Export Assembly as STL", "stl", "assembly.stl")
        };
        let Some(path) = rfd::FileDialog::new()
            .set_title(title)
            .set_file_name(default_name)
            .add_filter(extension.to_uppercase(), &[extension])
            .save_file()
        else {
            return;
        };
        let export = if three_mf {
            zerocad_core::assembly_to_3mf(
                &self.assembly_document,
                &self.assembly_definition_geometry,
            )
        } else {
            zerocad_core::assembly_to_binary_stl(
                &self.assembly_document,
                &self.assembly_definition_geometry,
            )
        };
        let result = match export {
            Ok(bytes) => std::fs::write(&path, bytes).map_err(|error| error.to_string()),
            Err(error) => Err(error.to_string()),
        };
        match result {
            Ok(()) => {
                self.error_msg = None;
                self.status_msg = format!("Assembly exported to {}", path.display());
            }
            Err(error) => {
                self.status_msg = format!("Assembly export refused: {error}");
                self.error_msg = Some(self.status_msg.clone());
            }
        }
    }

    /// Assembly browser plus the shared evaluated-scene viewport.
    pub(crate) fn draw_assembly_workspace(&mut self, ctx: &egui::Context) {
        let pal = self.pal();
        let mut insert_clicked = false;
        let mut visibility_change = None;
        let mut body_visibility_change = None;
        let mut body_selection_change = None;
        let mut grounded_change = None;
        let mut transform_change = None;
        let mut transform_edit_finished = false;
        let mut duplicate_clicked = false;
        let mut delete_clicked = false;
        let mut replace_selected_clicked = false;
        let mut replace_all_clicked = false;
        let mut rename_clicked = false;
        let mut align_faces_clicked = false;
        let mut create_face_mate = None;
        let mut create_concentric_mate = false;
        let mut create_fixed_mate = false;
        let mut selected_mate_change = None;
        let mut mate_edit = None;
        let mut repair_mate = None;
        let mut export_bom_clicked = false;
        let mut export_3mf_clicked = false;
        let mut export_stl_clicked = false;
        egui::CentralPanel::default()
            .frame(
                egui::Frame::none()
                    .fill(pal.surface_subtle)
                    .inner_margin(egui::Margin::same(12.0)),
            )
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    egui::Frame::none()
                        .fill(pal.surface)
                        .stroke(egui::Stroke::new(1.0, pal.border))
                        .rounding(8.0)
                        .inner_margin(egui::Margin::symmetric(14.0, 12.0))
                        .show(ui, |ui| {
                            ui.set_width(270.0);
                            ui.set_min_height((ui.available_height() - 2.0).max(280.0));
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new("ASSEMBLY")
                                        .strong()
                                        .size(12.0)
                                        .color(pal.text_strong),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui.button("Insert Part…").clicked() {
                                            insert_clicked = true;
                                        }
                                    },
                                );
                            });
                            ui.add_space(8.0);
                            ui.separator();
                            ui.add_space(10.0);
                            ui.label(
                                egui::RichText::new(self.project_title())
                                    .strong()
                                    .size(13.0)
                                    .color(pal.text_body),
                            );
                            ui.add_space(16.0);
                            ui.label(
                                egui::RichText::new(format!(
                                    "DEFINITIONS ({})",
                                    self.assembly_document.definitions.len()
                                ))
                                .strong()
                                .size(10.5)
                                .color(pal.text_muted),
                            );
                            for definition in self.assembly_document.definitions.values() {
                                let quantity = self
                                    .assembly_document
                                    .occurrences
                                    .values()
                                    .filter(|occurrence| {
                                        occurrence.definition_model_hash == definition.model_hash
                                    })
                                    .count();
                                ui.horizontal(|ui| {
                                    ui.label(&definition.name);
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.label(format!("×{quantity}"));
                                        },
                                    );
                                });
                            }
                            ui.add_space(14.0);
                            ui.label(
                                egui::RichText::new(format!(
                                    "COMPONENTS ({})",
                                    self.assembly_document.occurrences.len()
                                ))
                                .strong()
                                .size(10.5)
                                .color(pal.text_muted),
                            );
                            ui.add_space(7.0);
                            if self.assembly_document.occurrences.is_empty() {
                                ui.label(
                                    egui::RichText::new("No components")
                                        .size(12.0)
                                        .color(pal.text_faint),
                                );
                            } else {
                                egui::ScrollArea::vertical().show(ui, |ui| {
                                    for occurrence in self.assembly_document.occurrences.values() {
                                        let mut visible = !self
                                            .assembly_document
                                            .presentation
                                            .hidden_occurrences
                                            .contains(&occurrence.id);
                                        let mut grounded = occurrence.grounded;
                                        let selected = self.selected_assembly_occurrence
                                            == Some(occurrence.id);
                                        ui.horizontal(|ui| {
                                            if ui.checkbox(&mut visible, "").changed() {
                                                visibility_change =
                                                    Some((occurrence.id, visible));
                                            }
                                            if ui
                                                .selectable_label(selected, &occurrence.name)
                                                .clicked()
                                            {
                                                self.selected_assembly_occurrence =
                                                    Some(occurrence.id);
                                            }
                                            if ui
                                                .checkbox(&mut grounded, "Fixed")
                                                .on_hover_text(
                                                    "Ground this component; multiple components may be fixed.",
                                                )
                                                .changed()
                                            {
                                                grounded_change =
                                                    Some((occurrence.id, grounded));
                                            }
                                        });
                                        if selected {
                                            if let Some(bodies) =
                                                self.assembly_definition_geometry.get(
                                                    &occurrence.definition_model_hash,
                                                )
                                            {
                                                ui.indent(
                                                    format!("assembly-bodies-{}", occurrence.id),
                                                    |ui| {
                                                        for (body_id, _) in bodies.iter() {
                                                            let mut body_visible = !self
                                                                .assembly_document
                                                                .presentation
                                                                .hidden_bodies
                                                                .contains(&(
                                                                    occurrence.id,
                                                                    body_id.clone(),
                                                                ));
                                                            ui.horizontal(|ui| {
                                                                if ui
                                                                    .checkbox(
                                                                        &mut body_visible,
                                                                        "",
                                                                    )
                                                                    .changed()
                                                                {
                                                                    body_visibility_change = Some((
                                                                        occurrence.id,
                                                                        body_id.clone(),
                                                                        body_visible,
                                                                    ));
                                                                }
                                                                if ui
                                                                    .selectable_label(
                                                                        false, body_id,
                                                                    )
                                                                    .clicked()
                                                                {
                                                                    body_selection_change = Some((
                                                                        occurrence.id,
                                                                        body_id.clone(),
                                                                    ));
                                                                }
                                                            });
                                                        }
                                                    },
                                                );
                                            }
                                        }
                                    }
                                });
                            }
                            if !self.assembly_unresolved_definitions.is_empty() {
                                ui.add_space(12.0);
                                ui.colored_label(
                                    pal.danger,
                                    format!(
                                        "{} unresolved definition(s)",
                                        self.assembly_unresolved_definitions.len()
                                    ),
                                );
                            }
                            ui.add_space(10.0);
                            egui::CollapsingHeader::new("Mates (V2)")
                                .default_open(true)
                                .show(ui, |ui| {
                                    if let Some(mate_set) = &self.assembly_document.mates {
                                        if mate_set.mates.is_empty() {
                                            ui.label("No mates");
                                        }
                                        for mate in mate_set.mates.values() {
                                            let mut suppressed = mate.suppressed;
                                            let selected =
                                                self.selected_assembly_mate == Some(mate.id);
                                            ui.horizontal(|ui| {
                                                if ui
                                                    .checkbox(&mut suppressed, "")
                                                    .on_hover_text("Suppress or resume this mate")
                                                    .changed()
                                                {
                                                    mate_edit = Some((
                                                        mate.id,
                                                        MateEditAction::SetSuppressed(suppressed),
                                                    ));
                                                }
                                                if ui
                                                    .selectable_label(selected, &mate.name)
                                                    .clicked()
                                                {
                                                    selected_mate_change = Some(mate.id);
                                                }
                                                let status = self
                                                    .assembly_mate_statuses
                                                    .get(&mate.id)
                                                    .copied()
                                                    .unwrap_or(if mate.suppressed {
                                                        zerocad_core::MateSolveStatus::Suppressed
                                                    } else {
                                                        zerocad_core::MateSolveStatus::Unresolved
                                                    });
                                                ui.label(format!("{status:?}"));
                                                if !matches!(
                                                    &mate.kind,
                                                    zerocad_core::AssemblyMateKind::Fixed
                                                ) && ui.small_button("Flip").clicked()
                                                {
                                                    mate_edit = Some((
                                                        mate.id,
                                                        MateEditAction::FlipSense,
                                                    ));
                                                }
                                                if ui.small_button("×").clicked() {
                                                    mate_edit =
                                                        Some((mate.id, MateEditAction::Delete));
                                                }
                                            });
                                            match &mate.kind {
                                                zerocad_core::AssemblyMateKind::SignedDistance {
                                                    millimeters,
                                                } => {
                                                    let mut value = *millimeters;
                                                    if ui
                                                        .add(
                                                            egui::DragValue::new(&mut value)
                                                                .speed(0.1)
                                                                .suffix(" mm"),
                                                        )
                                                        .changed()
                                                    {
                                                        mate_edit = Some((
                                                            mate.id,
                                                            MateEditAction::SetParameter(value),
                                                        ));
                                                    }
                                                }
                                                zerocad_core::AssemblyMateKind::Angle {
                                                    radians,
                                                } => {
                                                    let mut degrees = radians.to_degrees();
                                                    if ui
                                                        .add(
                                                            egui::DragValue::new(&mut degrees)
                                                                .speed(0.25)
                                                                .suffix("Â°"),
                                                        )
                                                        .changed()
                                                    {
                                                        mate_edit = Some((
                                                            mate.id,
                                                            MateEditAction::SetParameter(degrees),
                                                        ));
                                                    }
                                                }
                                                _ => {}
                                            }
                                            let status = self
                                                .assembly_mate_statuses
                                                .get(&mate.id)
                                                .copied()
                                                .unwrap_or(
                                                    zerocad_core::MateSolveStatus::Unresolved,
                                                );
                                            if selected
                                                && status
                                                    == zerocad_core::MateSolveStatus::Unresolved
                                                && ui
                                                    .small_button(
                                                        "Repair from selected faces",
                                                    )
                                                    .clicked()
                                            {
                                                repair_mate = Some(mate.id);
                                            }
                                        }
                                    } else {
                                        ui.label("No mates");
                                    }
                                    ui.horizontal_wrapped(|ui| {
                                        if ui.button("Fix Selected").clicked() {
                                            create_fixed_mate = true;
                                        }
                                        if ui.button("Coincident Faces").clicked() {
                                            create_face_mate = Some(FaceMateMode::Coincident);
                                        }
                                        if ui.button("Concentric Edges").clicked() {
                                            create_concentric_mate = true;
                                        }
                                        if ui.button("Distance Faces").clicked() {
                                            create_face_mate =
                                                Some(FaceMateMode::SignedDistance);
                                        }
                                        if ui.button("Angle Faces").clicked() {
                                            create_face_mate = Some(FaceMateMode::Angle);
                                        }
                                    });
                                });
                            ui.add_space(12.0);
                            egui::CollapsingHeader::new("Bill of Materials")
                                .default_open(false)
                                .show(ui, |ui| {
                                    for row in zerocad_core::assembly_bom(&self.assembly_document) {
                                        let hash: String = row.model_hash[..6]
                                            .iter()
                                            .map(|byte| format!("{byte:02x}"))
                                            .collect();
                                        ui.horizontal(|ui| {
                                            ui.label(format!("{} ×{}", row.name, row.quantity));
                                            ui.with_layout(
                                                egui::Layout::right_to_left(egui::Align::Center),
                                                |ui| {
                                                    ui.monospace(hash);
                                                },
                                            );
                                        });
                                        if let Some(source) = &row.source_basename {
                                            ui.label(
                                                egui::RichText::new(source)
                                                    .small()
                                                    .color(pal.text_faint),
                                            );
                                        }
                                    }
                                    if ui.button("Export CSV…").clicked() {
                                        export_bom_clicked = true;
                                    }
                                });
                            ui.horizontal(|ui| {
                                if ui.button("Export 3MF…").clicked() {
                                    export_3mf_clicked = true;
                                }
                                if ui.button("Export STL…").clicked() {
                                    export_stl_clicked = true;
                                }
                            });
                            if let Some(selected_id) = self.selected_assembly_occurrence {
                                if let Some(occurrence) =
                                    self.assembly_document.occurrences.get(&selected_id)
                                {
                                    ui.add_space(14.0);
                                    ui.separator();
                                    ui.add_space(8.0);
                                    ui.label(
                                        egui::RichText::new("PLACEMENT")
                                            .strong()
                                            .size(10.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.selectable_value(
                                            &mut self.assembly_gizmo_mode,
                                            AssemblyGizmoMode::Translate,
                                            "Move",
                                        );
                                        ui.selectable_value(
                                            &mut self.assembly_gizmo_mode,
                                            AssemblyGizmoMode::Rotate,
                                            "Rotate",
                                        );
                                        ui.separator();
                                        ui.selectable_value(
                                            &mut self.assembly_gizmo_local_space,
                                            false,
                                            "World",
                                        );
                                        ui.selectable_value(
                                            &mut self.assembly_gizmo_local_space,
                                            true,
                                            "Local",
                                        );
                                    });
                                    let displayed_placement = self
                                        .assembly_interactive_target
                                        .filter(|(id, _)| id == &selected_id)
                                        .map(|(_, target)| target)
                                        .unwrap_or(occurrence.manual_placement);
                                    let mut translation = displayed_placement.translation();
                                    let mut euler = displayed_placement.euler_xyz_degrees();
                                    let mut responses = Vec::new();
                                    ui.add_enabled_ui(!occurrence.grounded, |ui| {
                                        for axis in 0..3 {
                                            ui.horizontal(|ui| {
                                                ui.label(["X", "Y", "Z"][axis]);
                                                responses.push(
                                                    ui.add(
                                                        egui::DragValue::new(
                                                            &mut translation[axis],
                                                        )
                                                        .speed(0.25)
                                                        .suffix(" mm"),
                                                    ),
                                                );
                                                responses.push(
                                                    ui.add(
                                                        egui::DragValue::new(&mut euler[axis])
                                                            .speed(0.5)
                                                            .suffix("°"),
                                                    ),
                                                );
                                            });
                                        }
                                    });
                                    if responses.iter().any(egui::Response::changed) {
                                        transform_change =
                                            Some((selected_id, translation, euler));
                                    }
                                    transform_edit_finished = responses.iter().any(|response| {
                                        response.drag_stopped() || response.lost_focus()
                                    });
                                    ui.horizontal(|ui| {
                                        if ui.button("Rename").clicked() {
                                            rename_clicked = true;
                                        }
                                        if ui.button("Duplicate").clicked() {
                                            duplicate_clicked = true;
                                        }
                                        if ui.button("Delete").clicked() {
                                            delete_clicked = true;
                                        }
                                    });
                                    if ui
                                        .button("Align Selected Faces")
                                        .on_hover_text(
                                            "Select a target face, then Shift-select a face on the component to move.",
                                        )
                                        .clicked()
                                    {
                                        align_faces_clicked = true;
                                    }
                                    ui.horizontal(|ui| {
                                        if ui.button("Replace Selectedâ€¦").clicked() {
                                            replace_selected_clicked = true;
                                        }
                                        if ui.button("Replace Allâ€¦").clicked() {
                                            replace_all_clicked = true;
                                        }
                                    });
                                }
                            }
                        });

                    ui.add_space(12.0);
                    let size = ui.available_size() - egui::vec2(0.0, 2.0);
                    let (rect, response) =
                        ui.allocate_exact_size(size, egui::Sense::click_and_drag());
                    ui.painter().rect(
                        rect,
                        8.0,
                        pal.surface,
                        egui::Stroke::new(1.0, pal.border),
                    );
                    if self.evaluated_scene.is_empty() {
                        let painter = ui.painter_at(rect);
                        let icon_rect = egui::Rect::from_center_size(
                            rect.center() - egui::vec2(0.0, 52.0),
                            egui::vec2(52.0, 52.0),
                        );
                        painter.rect_filled(icon_rect.expand(16.0), 18.0, pal.accent_soft);
                        icons::Icon::Assembly.draw(&painter, icon_rect, pal.accent);
                        painter.text(
                            rect.center() + egui::vec2(0.0, 22.0),
                            egui::Align2::CENTER_CENTER,
                            "Empty assembly",
                            egui::FontId::proportional(20.0),
                            pal.text_strong,
                        );
                        painter.text(
                            rect.center() + egui::vec2(0.0, 52.0),
                            egui::Align2::CENTER_CENTER,
                            "Insert a .zcad or .zcadh part to create the first occurrence.",
                            egui::FontId::proportional(12.5),
                            pal.text_muted,
                        );
                    } else {
                        if self.gpu_render && self.gpu.is_available() {
                            self.render_gpu_scene(rect, ctx);
                        } else {
                            self.gpu_texture_id = None;
                        }
                        self.draw_viewport(
                            ui.painter_at(rect),
                            rect,
                            response.hover_pos(),
                            None,
                        );
                        let gizmo_consumed = self.assembly_placement_gizmo(
                            &ui.painter_at(rect),
                            &response,
                            rect,
                        );
                        if !gizmo_consumed {
                            self.handle_assembly_viewport_input(&response, rect, ctx);
                        }
                    }
                });
            });
        if insert_clicked {
            self.insert_part_into_assembly();
        }
        if let Some((id, visible)) = visibility_change {
            self.set_assembly_occurrence_visible(id, visible);
        }
        if let Some((occurrence_id, body_id, visible)) = body_visibility_change {
            self.set_assembly_body_visible(occurrence_id, body_id, visible);
        }
        if let Some((occurrence_id, body_id)) = body_selection_change {
            if let Some((entity_id, _)) = self
                .assembly_scene_entities
                .iter()
                .find(|(_, identity)| identity.0 == occurrence_id && identity.1 == body_id)
            {
                self.selected_body.clear();
                self.selected_body
                    .insert((entity_id.clone(), BodyPick::Whole));
                self.selected_assembly_occurrence = Some(occurrence_id);
            }
        }
        if let Some((id, grounded)) = grounded_change {
            self.set_assembly_occurrence_grounded(id, grounded);
        }
        if let Some((id, translation, euler)) = transform_change {
            self.set_assembly_occurrence_transform(id, translation, euler);
        }
        if transform_edit_finished {
            self.finish_assembly_transform_edit();
        }
        if duplicate_clicked {
            self.duplicate_selected_assembly_occurrence();
        }
        if rename_clicked {
            if let Some(id) = self.selected_assembly_occurrence {
                if let Some(occurrence) = self.assembly_document.occurrences.get(&id) {
                    self.assembly_rename_occurrence = Some((id, occurrence.name.clone()));
                }
            }
        }
        if align_faces_clicked {
            self.align_selected_assembly_faces();
        }
        if let Some(mate_id) = selected_mate_change {
            self.selected_assembly_mate = Some(mate_id);
        }
        if create_fixed_mate {
            self.create_fixed_mate();
        }
        if let Some(mode) = create_face_mate {
            self.create_face_mate(mode);
        }
        if create_concentric_mate {
            self.create_concentric_edge_mate();
        }
        if let Some((mate_id, action)) = mate_edit {
            self.edit_assembly_mate(mate_id, action);
        }
        if let Some(mate_id) = repair_mate {
            self.repair_assembly_mate_from_selected_faces(mate_id);
        }
        if delete_clicked {
            self.delete_selected_assembly_occurrence();
        }
        if replace_selected_clicked {
            self.replace_assembly_occurrences(false);
        }
        if replace_all_clicked {
            self.replace_assembly_occurrences(true);
        }
        if export_bom_clicked {
            self.export_assembly_bom_csv();
        }
        if export_3mf_clicked {
            self.export_assembly_mesh(true);
        }
        if export_stl_clicked {
            self.export_assembly_mesh(false);
        }

        let mut rename_commit = None;
        let mut rename_cancel = false;
        if let Some((id, name)) = self.assembly_rename_occurrence.as_mut() {
            egui::Window::new("Rename Component")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    let response = ui.text_edit_singleline(name);
                    response.request_focus();
                    ui.horizontal(|ui| {
                        if ui.button("Rename").clicked()
                            || (response.lost_focus()
                                && ui.input(|input| input.key_pressed(egui::Key::Enter)))
                        {
                            rename_commit = Some((*id, name.clone()));
                        }
                        if ui.button("Cancel").clicked() {
                            rename_cancel = true;
                        }
                    });
                });
        }
        if let Some((id, name)) = rename_commit {
            self.commit_assembly_occurrence_rename(id, name);
        } else if rename_cancel {
            self.assembly_rename_occurrence = None;
        }
    }
}

fn short_hash(hash: &zerocad_core::ModelHash) -> String {
    hash[..6].iter().map(|byte| format!("{byte:02x}")).collect()
}

fn mate_frame_from_face(centroid: [f32; 3], normal: [f32; 3]) -> zerocad_core::MateFrame {
    let mut axis = normal.map(|value| value as f64);
    let length = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
    if length > 1.0e-15 {
        axis = axis.map(|value| value / length);
    } else {
        axis = [0.0, 0.0, 1.0];
    }
    zerocad_core::MateFrame {
        origin: centroid.map(|value| value as f64),
        axis,
        radial: least_aligned_radial(axis),
    }
}

fn mate_frame_from_edge(
    edge: &zerocad_core::mock_kernel::MeshEdgeRef,
) -> Option<zerocad_core::MateFrame> {
    match &edge.curve {
        Some(zerocad_core::mock_kernel::EdgeCurveHint::Circle {
            center,
            axis,
            x_dir,
            ..
        }) => {
            let axis = normalize3(axis.map(|value| value as f64))?;
            let mut radial = x_dir.map(|value| value as f64);
            let projection = dot3(radial, axis);
            for coordinate in 0..3 {
                radial[coordinate] -= projection * axis[coordinate];
            }
            let radial = normalize3(radial).unwrap_or_else(|| least_aligned_radial(axis));
            Some(zerocad_core::MateFrame {
                origin: center.map(|value| value as f64),
                axis,
                radial,
            })
        }
        Some(zerocad_core::mock_kernel::EdgeCurveHint::Line) | None => {
            let direction = [
                (edge.p1[0] - edge.p0[0]) as f64,
                (edge.p1[1] - edge.p0[1]) as f64,
                (edge.p1[2] - edge.p0[2]) as f64,
            ];
            let axis = normalize3(direction)?;
            Some(zerocad_core::MateFrame {
                origin: [
                    0.5 * (edge.p0[0] + edge.p1[0]) as f64,
                    0.5 * (edge.p0[1] + edge.p1[1]) as f64,
                    0.5 * (edge.p0[2] + edge.p1[2]) as f64,
                ],
                axis,
                radial: least_aligned_radial(axis),
            })
        }
    }
}

fn world_mate_frame(
    frame: zerocad_core::MateFrame,
    placement: zerocad_core::RigidPlacement,
) -> zerocad_core::MateFrame {
    let placement = placement.to_scene_placement();
    let point = placement.transform_point(frame.origin.map(|value| value as f32));
    let axis = placement.transform_vector(frame.axis.map(|value| value as f32));
    let radial = placement.transform_vector(frame.radial.map(|value| value as f32));
    let axis = normalize3(axis.map(|value| value as f64)).unwrap_or([0.0, 0.0, 1.0]);
    let radial =
        normalize3(radial.map(|value| value as f64)).unwrap_or_else(|| least_aligned_radial(axis));
    zerocad_core::MateFrame {
        origin: point.map(|value| value as f64),
        axis,
        radial,
    }
}

fn least_aligned_radial(axis: [f64; 3]) -> [f64; 3] {
    let globals = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut chosen = globals[0];
    let mut alignment = dot3(chosen, axis).abs();
    for candidate in globals.into_iter().skip(1) {
        let candidate_alignment = dot3(candidate, axis).abs();
        if candidate_alignment < alignment {
            chosen = candidate;
            alignment = candidate_alignment;
        }
    }
    let radial = [
        chosen[1] * axis[2] - chosen[2] * axis[1],
        chosen[2] * axis[0] - chosen[0] * axis[2],
        chosen[0] * axis[1] - chosen[1] * axis[0],
    ];
    normalize3(radial).unwrap_or([1.0, 0.0, 0.0])
}

fn normalize3(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot3(vector, vector).sqrt();
    (length > 1.0e-15 && length.is_finite()).then(|| vector.map(|value| value / length))
}

fn dot3(first: [f64; 3], second: [f64; 3]) -> f64 {
    first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
}
