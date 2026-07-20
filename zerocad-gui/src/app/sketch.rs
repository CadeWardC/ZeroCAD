use crate::*;

impl ZeroCadApp {
    /// Begin one atomic live-sketch edit. Multi-entity operations call this
    /// once, immediately before their first mutation.
    pub(crate) fn push_working_sketch_undo(&mut self) {
        if self.working_sketch_undo.len() >= 50 {
            self.working_sketch_undo.remove(0);
        }
        self.working_sketch_undo.push(WorkingSketchSnapshot {
            shapes: self.sketch_shapes.clone(),
            corner_mods: self.sketch_corner_mods.clone(),
            mirrors: self.sketch_mirrors.clone(),
            solver_model: self.sketch_solver_model.clone(),
            entity_ids: self.sketch_entity_ids.clone(),
            next_entity_id: self.sketch_next_entity_id,
        });
    }

    fn restore_working_sketch_snapshot(&mut self, snapshot: WorkingSketchSnapshot) {
        self.sketch_shapes = snapshot.shapes;
        self.sketch_corner_mods = snapshot.corner_mods;
        self.sketch_mirrors = snapshot.mirrors;
        self.sketch_solver_model = snapshot.solver_model;
        self.sketch_entity_ids = snapshot.entity_ids;
        self.sketch_next_entity_id = snapshot.next_entity_id;
        self.pending_corners.clear();
        self.line_chain_start = None;
        self.sketch_selected_ids.clear();
        self.sketch_selected_constraint = None;
        self.sketch_conflict_constraint = None;
        self.sketch_trim_preview = None;
        self.cancel_in_progress_shape();
        self.rebuild_active_sketch_curves();
    }

    pub(crate) fn ensure_active_solver_model(&mut self) {
        if self.sketch_solver_model.is_some() {
            return;
        }
        let vars = self.document.variable_map();
        let ids = if self.sketch_entity_ids.len() == self.sketch_shapes.len() {
            self.sketch_entity_ids.clone()
        } else {
            zerocad_core::sketch::EntityId::sequence(self.sketch_shapes.len())
        };
        let next = self
            .sketch_next_entity_id
            .max(ids.iter().map(|id| id.0 + 1).max().unwrap_or(0));
        let (model, next) = zerocad_core::sketch::constraints::promote_shapes_to_entities(
            &self.sketch_shapes,
            &ids,
            &vars,
            next,
        );
        self.sketch_entity_ids = ids;
        self.sketch_next_entity_id = next;
        self.sketch_solver_model = Some(model);
    }

    fn selected_offset_sources(&self) -> Result<Vec<zerocad_core::sketch::EntityId>, String> {
        let model = self
            .sketch_solver_model
            .as_ref()
            .ok_or_else(|| "Offset requires editable sketch geometry.".to_string())?;
        let mut sources = Vec::new();
        for selected in &self.sketch_selected_ids {
            let Some(entity) = model
                .entities
                .iter()
                .find(|entity| entity.id() == *selected)
            else {
                continue;
            };
            match entity {
                zerocad_core::sketch::SketchEntity::Line { .. }
                | zerocad_core::sketch::SketchEntity::Circle { .. }
                | zerocad_core::sketch::SketchEntity::Arc { .. } => sources.push(*selected),
                zerocad_core::sketch::SketchEntity::Ellipse { .. } => {
                    return Err("Ellipse offsets are not supported in Offset v1.".to_string());
                }
                zerocad_core::sketch::SketchEntity::Spline { .. } => {
                    return Err("Spline offsets are not supported in Offset v1.".to_string());
                }
            }
        }
        sources.sort();
        sources.dedup();
        if sources.is_empty() {
            Err("Select one or more connected lines/arcs, or one circle.".to_string())
        } else {
            Ok(sources)
        }
    }

    fn offset_signed_distance(
        &self,
        sources: &[zerocad_core::sketch::EntityId],
        seed: (f32, f32),
    ) -> Result<f32, String> {
        use zerocad_core::sketch::SketchEntity;
        let model = self
            .sketch_solver_model
            .as_ref()
            .ok_or_else(|| "Offset requires editable sketch geometry.".to_string())?;
        let point = |id| {
            model
                .point(id)
                .map(|point| (point.pos.0 as f32, point.pos.1 as f32))
        };
        let mut candidates = Vec::new();
        for source in sources {
            let Some(entity) = model.entities.iter().find(|entity| entity.id() == *source) else {
                continue;
            };
            let signed = match entity {
                SketchEntity::Line { p0, p1, .. } => {
                    let (start, end) = (point(*p0).unwrap(), point(*p1).unwrap());
                    let dx = end.0 - start.0;
                    let dy = end.1 - start.1;
                    let length = dx.hypot(dy);
                    if length <= 1.0e-6 {
                        continue;
                    }
                    (seed.0 - start.0) * (-dy / length) + (seed.1 - start.1) * (dx / length)
                }
                SketchEntity::Circle { center, radius, .. } => {
                    let center = point(*center).unwrap();
                    *radius as f32 - (seed.0 - center.0).hypot(seed.1 - center.1)
                }
                SketchEntity::Arc {
                    center,
                    start,
                    end,
                    radius,
                    clockwise,
                    ..
                } => {
                    let center = point(*center).unwrap();
                    let start = point(*start).unwrap();
                    let end = point(*end).unwrap();
                    let traversal = if *clockwise {
                        -1.0
                    } else {
                        let first = (start.1 - center.1).atan2(start.0 - center.0);
                        let mut last = (end.1 - center.1).atan2(end.0 - center.0);
                        while last - first > std::f32::consts::PI {
                            last -= std::f32::consts::TAU;
                        }
                        while last - first < -std::f32::consts::PI {
                            last += std::f32::consts::TAU;
                        }
                        (last - first).signum()
                    };
                    (*radius as f32 - (seed.0 - center.0).hypot(seed.1 - center.1)) / traversal
                }
                SketchEntity::Ellipse { .. } | SketchEntity::Spline { .. } => continue,
            };
            candidates.push((signed.abs(), signed));
        }
        let signed = candidates
            .into_iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, signed)| signed)
            .ok_or_else(|| "The selected offset sources are unavailable.".to_string())?;
        if !signed.is_finite() || signed.abs() <= 1.0e-5 {
            Err("Click away from the source to set a nonzero offset.".to_string())
        } else {
            Ok(signed)
        }
    }

    fn offset_operation_from_seed(
        &self,
        seed: (f32, f32),
    ) -> Result<zerocad_core::sketch::SketchOffsetOperation, String> {
        let sources = self.selected_offset_sources()?;
        let placement_distance = self.offset_signed_distance(&sources, seed)?;
        let text = self.offset_distance_text.trim();
        let distance = if text.is_empty() {
            Dimension::literal(placement_distance)
        } else {
            let magnitude = self
                .eval_dim(text)
                .ok_or_else(|| format!("Invalid offset expression '{text}'."))?
                .abs();
            if !magnitude.is_finite() || magnitude <= 1.0e-5 {
                return Err("Offset distance must be greater than zero.".to_string());
            }
            let value = placement_distance.signum() * magnitude;
            let expr = zerocad_core::expr::preserves_source(text).then(|| {
                if placement_distance.is_sign_negative() {
                    format!("-({text})")
                } else {
                    text.to_string()
                }
            });
            Dimension { value, expr }
        };
        Ok(zerocad_core::sketch::SketchOffsetOperation {
            id: zerocad_core::sketch::EntityId(self.sketch_next_entity_id),
            sources,
            distance,
            creation_side_seed: seed,
        })
    }

    fn nearest_offset_entity(
        &self,
        position: (f32, f32),
        tolerance: f32,
    ) -> Option<zerocad_core::sketch::EntityId> {
        use zerocad_core::sketch::SketchEntity;
        let model = self.sketch_solver_model.as_ref()?;
        let point = |id| {
            model
                .point(id)
                .map(|point| (point.pos.0 as f32, point.pos.1 as f32))
        };
        let segment_distance = |a: (f32, f32), b: (f32, f32)| {
            let ab = (b.0 - a.0, b.1 - a.1);
            let length_squared = ab.0 * ab.0 + ab.1 * ab.1;
            if length_squared <= f32::EPSILON {
                return (position.0 - a.0).hypot(position.1 - a.1);
            }
            let t = (((position.0 - a.0) * ab.0 + (position.1 - a.1) * ab.1) / length_squared)
                .clamp(0.0, 1.0);
            (position.0 - (a.0 + ab.0 * t)).hypot(position.1 - (a.1 + ab.1 * t))
        };
        model
            .entities
            .iter()
            .filter(|entity| !model.construction.contains(&entity.id()))
            .filter_map(|entity| {
                let distance = match entity {
                    SketchEntity::Line { p0, p1, .. } => segment_distance(point(*p0)?, point(*p1)?),
                    SketchEntity::Circle { center, radius, .. }
                    | SketchEntity::Arc { center, radius, .. } => {
                        let center = point(*center)?;
                        ((position.0 - center.0).hypot(position.1 - center.1) - *radius as f32)
                            .abs()
                    }
                    SketchEntity::Ellipse { .. } | SketchEntity::Spline { .. } => return None,
                };
                (distance <= tolerance).then_some((entity.id(), distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(id, _)| id)
    }

    pub(crate) fn handle_offset_click(
        &mut self,
        position: (f32, f32),
        tolerance: f32,
        extend_selection: bool,
    ) {
        self.ensure_active_solver_model();
        if let Some(entity) = self.nearest_offset_entity(position, tolerance) {
            if !extend_selection {
                self.sketch_selected_ids.clear();
            }
            if let Some(index) = self
                .sketch_selected_ids
                .iter()
                .position(|selected| *selected == entity)
            {
                self.sketch_selected_ids.remove(index);
            } else {
                self.sketch_selected_ids.push(entity);
            }
            self.status_msg = format!(
                "{} offset source(s) selected — Shift-click more, then click on the desired side.",
                self.sketch_selected_ids.len()
            );
            return;
        }

        let operation = match self.offset_operation_from_seed(position) {
            Ok(operation) => operation,
            Err(error) => {
                self.status_msg = error;
                return;
            }
        };
        self.push_working_sketch_undo();
        self.sketch_next_entity_id += 1;
        let vars = self.document.variable_map();
        let outcome = if let Some(model) = &mut self.sketch_solver_model {
            model.offsets.push(operation.clone());
            zerocad_core::sketch::evaluate_associative_offset(model, &operation, &vars)
        } else {
            return;
        };
        self.sketch_selected_ids.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg = match outcome {
            Ok(_) => "Associative offset created. Its derived curves are read-only until Dissolve."
                .to_string(),
            Err(error) => format!("Offset {} is unresolved: {error}.", operation.id.0),
        };
    }

    pub(crate) fn preview_sketch_offset(&self, seed: (f32, f32)) -> Option<SketchCurves> {
        let operation = self.offset_operation_from_seed(seed).ok()?;
        let model = self.sketch_solver_model.as_ref()?;
        zerocad_core::sketch::evaluate_associative_offset(
            model,
            &operation,
            &self.document.variable_map(),
        )
        .ok()
    }

    pub(crate) fn dissolve_last_sketch_offset(&mut self) {
        let vars = self.document.variable_map();
        let (operation, derived) = {
            let Some(model) = self.sketch_solver_model.as_ref() else {
                self.status_msg = "This sketch has no associative offsets.".to_string();
                return;
            };
            let Some(operation) = model.offsets.last().cloned() else {
                self.status_msg = "This sketch has no associative offsets.".to_string();
                return;
            };
            let derived =
                match zerocad_core::sketch::evaluate_associative_offset(model, &operation, &vars) {
                    Ok(derived) => derived,
                    Err(error) => {
                        self.status_msg =
                            format!("Offset {} cannot dissolve: {error}.", operation.id.0);
                        return;
                    }
                };
            (operation, derived)
        };

        self.push_working_sketch_undo();
        let (addition, next) = zerocad_core::sketch::constraints::promote_shapes_to_entities(
            &[SketchShape::Raw { curves: derived }],
            &[operation.id],
            &vars,
            self.sketch_next_entity_id,
        );
        if let Some(model) = &mut self.sketch_solver_model {
            model.offsets.retain(|offset| offset.id != operation.id);
            model.points.extend(addition.points);
            model.entities.extend(addition.entities);
            model.constraints.extend(addition.constraints);
        }
        self.sketch_next_entity_id = next;
        self.rebuild_active_sketch_curves();
        self.status_msg = format!(
            "Offset {} dissolved into editable sketch entities.",
            operation.id.0
        );
    }

    pub(crate) fn create_sketch_pattern(&mut self, circular: bool) {
        self.ensure_active_solver_model();
        let vars = self.document.variable_map();
        let Some(model) = self.sketch_solver_model.as_ref() else {
            return;
        };
        let mut sources: Vec<_> = self
            .sketch_selected_ids
            .iter()
            .copied()
            .filter(|selected| model.entities.iter().any(|entity| entity.id() == *selected))
            .collect();
        sources.sort();
        sources.dedup();
        if sources.is_empty() {
            self.status_msg = "Select one or more sketch curves to pattern.".to_string();
            return;
        }
        let count_value = self
            .eval_dim(&self.sketch_pattern_count_text)
            .unwrap_or(-1.0);
        let count = count_value.round() as u32;
        if count_value < 1.0 || (count_value - count as f32).abs() > 1.0e-5 {
            self.status_msg = "Sketch Pattern count must be a positive whole number.".to_string();
            return;
        }
        let parse = |text: &str| self.eval_dim(text);
        let kind = if circular {
            let (Some(center_x), Some(center_y), Some(angle)) = (
                parse(&self.sketch_pattern_x_text),
                parse(&self.sketch_pattern_y_text),
                parse(&self.sketch_pattern_angle_text),
            ) else {
                self.status_msg = "Circular pattern center and angle must evaluate.".to_string();
                return;
            };
            zerocad_core::sketch::SketchPatternKind::Circular {
                center: [f64::from(center_x), f64::from(center_y)],
                total_angle_deg: Dimension {
                    value: angle,
                    expr: zerocad_core::expr::preserves_source(&self.sketch_pattern_angle_text)
                        .then(|| self.sketch_pattern_angle_text.trim().to_string()),
                },
                count,
            }
        } else {
            let (Some(direction_x), Some(direction_y), Some(spacing)) = (
                parse(&self.sketch_pattern_x_text),
                parse(&self.sketch_pattern_y_text),
                parse(&self.sketch_pattern_spacing_text),
            ) else {
                self.status_msg = "Linear pattern direction and spacing must evaluate.".to_string();
                return;
            };
            zerocad_core::sketch::SketchPatternKind::Linear {
                direction: [f64::from(direction_x), f64::from(direction_y)],
                spacing: Dimension {
                    value: spacing,
                    expr: zerocad_core::expr::preserves_source(&self.sketch_pattern_spacing_text)
                        .then(|| self.sketch_pattern_spacing_text.trim().to_string()),
                },
                count,
            }
        };
        let operation = zerocad_core::sketch::SketchPatternOperation {
            id: zerocad_core::sketch::EntityId(self.sketch_next_entity_id),
            sources,
            kind,
        };
        if let Err(error) =
            zerocad_core::sketch::evaluate_associative_pattern(model, &operation, &vars)
        {
            self.status_msg = format!("Sketch Pattern is unresolved: {error}.");
            return;
        }
        self.push_working_sketch_undo();
        self.sketch_next_entity_id += 1;
        if let Some(model) = &mut self.sketch_solver_model {
            model.patterns.push(operation);
        }
        self.sketch_selected_ids.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg =
            "Associative Sketch Pattern created. Generated curves are read-only until Dissolve."
                .to_string();
    }

    pub(crate) fn dissolve_last_sketch_pattern(&mut self) {
        let vars = self.document.variable_map();
        let (operation, derived) = {
            let Some(model) = self.sketch_solver_model.as_ref() else {
                self.status_msg = "This sketch has no associative patterns.".to_string();
                return;
            };
            let Some(operation) = model.patterns.last().cloned() else {
                self.status_msg = "This sketch has no associative patterns.".to_string();
                return;
            };
            let derived = match zerocad_core::sketch::dissolve_associative_pattern(
                model, &operation, &vars,
            ) {
                Ok(evaluation) => evaluation.curves,
                Err(error) => {
                    self.status_msg = format!(
                        "Sketch Pattern {} cannot dissolve: {error}.",
                        operation.id.0
                    );
                    return;
                }
            };
            (operation, derived)
        };
        self.push_working_sketch_undo();
        let (addition, next) = zerocad_core::sketch::constraints::promote_shapes_to_entities(
            &[SketchShape::Raw { curves: derived }],
            &[operation.id],
            &vars,
            self.sketch_next_entity_id,
        );
        if let Some(model) = &mut self.sketch_solver_model {
            model.patterns.retain(|pattern| pattern.id != operation.id);
            model.points.extend(addition.points);
            model.entities.extend(addition.entities);
            model.constraints.extend(addition.constraints);
        }
        self.sketch_next_entity_id = next;
        self.rebuild_active_sketch_curves();
        self.status_msg = format!(
            "Sketch Pattern {} dissolved into editable sketch entities.",
            operation.id.0
        );
    }

    /// Replace the live Trim hover plan. The plan contains both the exact
    /// removable span and the private replacement entities used by commit.
    pub(crate) fn update_trim_preview(&mut self, cursor: (f32, f32), tolerance: f32) {
        self.ensure_active_solver_model();
        self.sketch_trim_preview = self
            .sketch_solver_model
            .as_ref()
            .and_then(|model| zerocad_core::sketch::preview_trim(model, cursor, tolerance).ok());
    }

    /// Commit the immutable plan most recently produced for the cursor. One
    /// successful click creates exactly one working-sketch transaction.
    pub(crate) fn commit_trim_preview(&mut self) {
        let Some(preview) = self.sketch_trim_preview.take() else {
            self.status_msg = "Trim unresolved: hover a removable curve span.".to_string();
            return;
        };
        let target = preview.target;
        self.push_working_sketch_undo();
        let Some(model) = self.sketch_solver_model.as_mut() else {
            let _ = self.working_sketch_undo.pop();
            self.status_msg = "Trim requires editable sketch geometry.".to_string();
            return;
        };
        let outcome = zerocad_core::sketch::apply_trim_preview(
            model,
            preview,
            &mut self.sketch_next_entity_id,
        );
        self.sketch_selected_ids.clear();
        self.sketch_selected_constraint = None;
        self.rebuild_active_sketch_curves();
        let removed = outcome.constraints.removed.len();
        let ambiguous = outcome.constraints.ambiguous.len();
        self.status_msg = if removed == 0 && ambiguous == 0 {
            format!("Trimmed entity {}.", target.0)
        } else {
            format!(
                "Trimmed entity {}. Removed {} invalid constraint(s) and {} ambiguous constraint(s).",
                target.0, removed, ambiguous
            )
        };
    }

    pub(crate) fn project_selected_edges_to_sketch(&mut self) {
        let selected: Vec<(String, EdgeRef)> = self
            .selected_body
            .iter()
            .filter_map(|(body, pick)| match pick {
                BodyPick::Edge(group) => self
                    .edge_ref_from(body, *group)
                    .map(|edge| (body.clone(), edge)),
                _ => None,
            })
            .collect();
        if selected.is_empty() {
            self.status_msg = "Select one or more body edges to project.".to_string();
            return;
        }
        self.push_working_sketch_undo();
        self.ensure_active_solver_model();
        let mut added = 0usize;
        let mut errors = Vec::new();
        if let Some(model) = &mut self.sketch_solver_model {
            for (body, edge) in selected {
                match zerocad_core::sketch::append_projected_edge(
                    model,
                    body,
                    edge,
                    self.active_sketch_cs,
                    &mut self.sketch_next_entity_id,
                ) {
                    Ok(()) => added += 1,
                    Err(error) => errors.push(error),
                }
            }
        }
        if added == 0 {
            if let Some(snapshot) = self.working_sketch_undo.pop() {
                self.restore_working_sketch_snapshot(snapshot);
            }
            self.status_msg = errors
                .into_iter()
                .next()
                .unwrap_or_else(|| "The selected edges could not be projected.".to_string());
            return;
        }
        self.selected_body.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg = if errors.is_empty() {
            format!("Projected {added} edge(s) as associative construction geometry.")
        } else {
            format!(
                "Projected {added} edge(s); {} edge(s) could not be projected.",
                errors.len()
            )
        };
    }

    fn resolve_projected_edge_source(
        meshes: &[(String, MockMesh)],
        projection: &zerocad_core::sketch::ProjectedEdgeReference,
    ) -> Option<EdgeRef> {
        let (_, mesh) = meshes
            .iter()
            .find(|(body, _)| body == &projection.source_body)?;
        if let Some(edge_id) = projection
            .source
            .topology
            .as_ref()
            .and_then(|topology| topology.edge_id.as_deref())
        {
            if let Some(group) = mesh.edge_refs.iter().find_map(|edge| {
                (edge
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.edge_id.as_deref())
                    == Some(edge_id))
                .then_some(edge.group)
            }) {
                return Self::edge_ref_from_mesh(&projection.source_body, mesh, group);
            }
            // Named references are never retargeted geometrically. The caller
            // leaves the projection unresolved until that exact edge returns.
            return None;
        }

        // Genuinely unnamed legacy projection: bounded geometric fallback.
        let groups: std::collections::BTreeSet<u32> = mesh
            .edge_refs
            .iter()
            .map(|edge| edge.group)
            .chain(mesh.edge_groups.iter().copied())
            .collect();
        let distance = |a: [f32; 3], b: [f32; 3]| {
            (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
        };
        groups
            .into_iter()
            .filter_map(|group| {
                let edge = Self::edge_ref_from_mesh(&projection.source_body, mesh, group)?;
                let direct = distance(edge.p0, projection.source.p0)
                    + distance(edge.p1, projection.source.p1);
                let reverse = distance(edge.p0, projection.source.p1)
                    + distance(edge.p1, projection.source.p0);
                Some((direct.min(reverse), edge))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .and_then(|(distance, edge)| (distance <= 1.0).then_some(edge))
    }

    /// Refresh every persisted projection from the newly evaluated body
    /// meshes. Returns true when a second evaluation is needed because a
    /// projected reference moved and constrained sketch geometry may follow it.
    pub(crate) fn refresh_projected_sketch_edges(&mut self) -> Result<bool, String> {
        let meshes = self.body_meshes.clone();
        let mut updates = Vec::new();
        for node in self.document.graph.node_indices() {
            let FeatureType::Sketch {
                cs,
                solver: Some(model),
                ..
            } = &self.document.graph[node].feature
            else {
                continue;
            };
            for (projection_index, projection) in model.projected_edges.iter().enumerate() {
                let Some(edge) = Self::resolve_projected_edge_source(&meshes, projection) else {
                    continue;
                };
                if edge != projection.source {
                    updates.push((
                        node,
                        self.document.graph[node].id.clone(),
                        projection_index,
                        *cs,
                        edge,
                    ));
                }
            }
        }
        if updates.is_empty() {
            return Ok(false);
        }
        let variables = self.document.variable_map();
        let mut changed_nodes = Vec::new();
        for (node, sketch_id, projection_index, cs, edge) in updates {
            if let FeatureType::Sketch {
                solver: Some(model),
                ..
            } = &mut self.document.graph[node].feature
            {
                zerocad_core::sketch::rebuild_projected_edge(model, projection_index, edge, cs)
                    .map_err(|error| {
                        format!("Sketch '{sketch_id}' projection {projection_index}: {error}.")
                    })?;
                changed_nodes.push(node);
            }
        }
        changed_nodes.sort_by_key(|node| node.index());
        changed_nodes.dedup();
        for node in &changed_nodes {
            if let FeatureType::Sketch {
                solver: Some(model),
                ..
            } = &mut self.document.graph[*node].feature
            {
                let report = zerocad_core::sketch::solve_model(model, &variables);
                if report.outcome == zerocad_core::sketch::SolveOutcome::Converged {
                    zerocad_core::sketch::solve::apply_solution(model, &report);
                }
            }
        }
        Ok(!changed_nodes.is_empty())
    }

    /// Undo the most recently committed action in the live sketch instead of
    /// stepping the document history. Sketch edits do not enter the document
    /// undo stack until Finish Sketch, so routing Ctrl/Cmd+Z to `undo()` while
    /// this mode is active either did nothing or changed an unrelated feature.
    pub(crate) fn undo_last_sketch_action(&mut self) {
        if self.clear_pending_corners() {
            self.status_msg = "Undid the staged sketch corner.".to_string();
            return;
        }

        if let Some(snapshot) = self.working_sketch_undo.pop() {
            self.restore_working_sketch_snapshot(snapshot);
            self.status_msg = "Undid the last sketch operation.".to_string();
            return;
        }

        let Some(_shape) = self.sketch_shapes.pop() else {
            if self.sketch_mirrors.pop().is_some() || self.sketch_corner_mods.pop().is_some() {
                self.rebuild_active_sketch_curves();
                self.status_msg = "Undid the last sketch operation.".to_string();
            } else {
                self.status_msg = "Nothing to undo in this sketch.".to_string();
            }
            return;
        };

        // Edit Sketch uses the solver model as its geometry source. Remove the
        // entities promoted from the popped shape as well, plus constraints and
        // points that would otherwise keep that shape visible.
        if let Some(owner) = self.sketch_entity_ids.pop() {
            if let Some(model) = &mut self.sketch_solver_model {
                model
                    .entities
                    .retain(|entity| entity.derived_from() != Some(owner));

                let mut entity_ids = std::collections::HashSet::new();
                let mut point_ids = std::collections::HashSet::new();
                for entity in &model.entities {
                    entity_ids.insert(entity.id());
                    match entity {
                        zerocad_core::sketch::SketchEntity::Line { p0, p1, .. } => {
                            point_ids.insert(*p0);
                            point_ids.insert(*p1);
                        }
                        zerocad_core::sketch::SketchEntity::Circle { center, .. } => {
                            point_ids.insert(*center);
                        }
                        zerocad_core::sketch::SketchEntity::Arc {
                            center, start, end, ..
                        } => {
                            point_ids.insert(*center);
                            point_ids.insert(*start);
                            point_ids.insert(*end);
                        }
                        zerocad_core::sketch::SketchEntity::Ellipse { center, .. } => {
                            point_ids.insert(*center);
                        }
                        zerocad_core::sketch::SketchEntity::Spline { points, .. } => {
                            point_ids.extend(points.iter().copied());
                        }
                    }
                }
                model.points.retain(|point| point_ids.contains(&point.id));
                model.constraints.retain(|constraint| {
                    use zerocad_core::sketch::Constraint;
                    match constraint {
                        Constraint::Coincident { a, b, .. }
                        | Constraint::Distance { a, b, .. }
                        | Constraint::DistanceX { a, b, .. }
                        | Constraint::DistanceY { a, b, .. } => {
                            point_ids.contains(a) && point_ids.contains(b)
                        }
                        Constraint::Horizontal { line, .. } | Constraint::Vertical { line, .. } => {
                            entity_ids.contains(line)
                        }
                        Constraint::Radius { circle, .. } | Constraint::Diameter { circle, .. } => {
                            entity_ids.contains(circle)
                        }
                        Constraint::Parallel { a, b, .. }
                        | Constraint::Perpendicular { a, b, .. }
                        | Constraint::Equal { a, b, .. }
                        | Constraint::Angle { a, b, .. }
                        | Constraint::Concentric { a, b, .. }
                        | Constraint::Collinear { a, b, .. } => {
                            entity_ids.contains(a) && entity_ids.contains(b)
                        }
                        Constraint::Tangent { line, circle, .. } => {
                            entity_ids.contains(line) && entity_ids.contains(circle)
                        }
                        Constraint::Fixed { p, .. } => point_ids.contains(p),
                        Constraint::Midpoint { point, line, .. } => {
                            point_ids.contains(point) && entity_ids.contains(line)
                        }
                        Constraint::PointOnObject { point, object, .. } => {
                            point_ids.contains(point) && entity_ids.contains(object)
                        }
                        Constraint::Symmetric { a, b, axis, .. } => {
                            point_ids.contains(a)
                                && point_ids.contains(b)
                                && entity_ids.contains(axis)
                        }
                        Constraint::SplineTangent { spline, line, .. } => {
                            entity_ids.contains(spline) && entity_ids.contains(line)
                        }
                        Constraint::SplineCurvature { spline, .. } => entity_ids.contains(spline),
                    }
                });
            }
        }

        self.line_chain_start = None;
        self.cancel_in_progress_shape();
        self.rebuild_active_sketch_curves();
        self.status_msg = "Undid the last drawn shape.".to_string();
    }

    /// Commit an **associative** sketch Mirror across the 2-click axis `p0`→`p1`.
    /// Rather than baking a reflected copy, this stores a [`SketchMirror`] record
    /// that [`zerocad_core::effective_curves_solved`] re-applies on every rebuild
    /// — so editing (or drag-solving) the source half updates the mirror. Works
    /// the same in a fresh drawing session and an Edit Sketch session, since the
    /// record is geometry-independent.
    pub(crate) fn commit_sketch_mirror(&mut self, p0: (f32, f32), p1: (f32, f32)) {
        let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
        if (dx * dx + dy * dy).sqrt() < 1.0e-4 {
            self.status_msg = "Mirror axis too short — click two distinct points.".to_string();
            return;
        }
        if self.sketch_curves.is_empty() {
            self.status_msg = "Nothing to mirror — draw geometry first.".to_string();
            return;
        }
        self.push_working_sketch_undo();
        self.sketch_mirrors
            .push(zerocad_core::SketchMirror { a: p0, b: p1 });
        self.rebuild_active_sketch_curves();
        self.status_msg = "Mirrored sketch across the axis (associative).".to_string();
    }

    /// The reflected copy of the live sketch across the in-progress mirror axis
    /// `p0`→`cursor`, for the drawing preview. `None` if the axis is degenerate.
    pub(crate) fn mirror_preview_curves(
        &self,
        p0: (f32, f32),
        cursor: (f32, f32),
    ) -> Option<SketchCurves> {
        let (dx, dy) = (cursor.0 - p0.0, cursor.1 - p0.1);
        if (dx * dx + dy * dy).sqrt() < 1.0e-4 {
            return None;
        }
        Some(zerocad_core::reflect_curves_across(
            &self.sketch_curves,
            p0,
            cursor,
        ))
    }

    /// Re-run planar region detection on the active sketch curves.
    /// Trims selected indices that no longer correspond to a region.
    ///
    /// A sketch on a body face folds the projected face boundary in as
    /// reference curves — appended AFTER the drawn curves, the same merge
    /// order the evaluator and every committed-sketch region site use, so the
    /// region indices seen live match the ones an extrude will store.
    pub(crate) fn recompute_sketch_regions(&mut self) {
        let mut curves = self.sketch_curves.clone();
        curves.extend_curves(&self.active_face_boundary);
        self.detected_regions = detect_regions(&curves);
        let n = self.detected_regions.len();
        self.selected_region_indices.retain(|i| *i < n);
    }

    /// Reset everything related to the in-progress sketch.
    pub(crate) fn reset_sketch_state(&mut self) {
        self.sketch_curves = SketchCurves::new();
        self.active_face_boundary = SketchCurves::new();
        self.sketch_shapes.clear();
        self.sketch_corner_mods.clear();
        self.sketch_mirrors.clear();
        self.working_sketch_undo.clear();
        self.pending_corners.clear();
        self.detected_regions.clear();
        self.selected_region_indices.clear();
        self.editing_sketch_id = None;
        self.sketch_solver_model = None;
        self.sketch_entity_ids.clear();
        self.sketch_next_entity_id = 0;
        self.sketch_drag_point = None;
        self.line_chain_start = None;
        self.sketch_selected_ids.clear();
        self.sketch_selected_constraint = None;
        self.sketch_conflict_constraint = None;
        self.sketch_trim_preview = None;
        self.cancel_in_progress_shape();
    }

    /// Rebuild the live sketch geometry from the parametric `sketch_shapes` (plus
    /// any committed fillet/chamfer corner mods AND the uncommitted pending ones)
    /// against the current variables, then re-detect regions. Folding the pending
    /// corners in here is what gives the Fillet/Chamfer tool its live preview:
    /// every staged corner shows rounded/beveled at the current radius before the
    /// user commits. Called after any change to the shape list, the pending set,
    /// or the radius text.
    pub(crate) fn rebuild_active_sketch_curves(&mut self) {
        let vars = self.document.variable_map();
        let mut mods = self.sketch_corner_mods.clone();
        mods.extend(self.pending_corner_mods());
        // A solver model (Edit Sketch session) is the source of truth for the
        // live geometry; a fresh drawing session bakes from the shape list.
        self.sketch_curves = zerocad_core::effective_curves_solved(
            &SketchCurves::new(),
            &self.sketch_shapes,
            &mods,
            &self.sketch_mirrors,
            self.sketch_solver_model.as_ref(),
            &vars,
        );
        self.recompute_sketch_regions();
    }

    /// Re-solve the live sketch's constraint model in place (drag frames, a
    /// constraint edit) and rebuild the displayed curves from the result. The
    /// solve is warm-started from the current positions, so an under-constrained
    /// sketch moves minimally. On a failed solve the model keeps its current
    /// (last-valid) positions — the geometry degrades, never blanks.
    pub(crate) fn solve_live_sketch(&mut self) {
        let vars = self.document.variable_map();
        if let Some(model) = &mut self.sketch_solver_model {
            let report = zerocad_core::sketch::solve_model(model, &vars);
            if report.outcome == zerocad_core::sketch::SolveOutcome::Converged {
                zerocad_core::sketch::solve::apply_solution(model, &report);
                self.sketch_conflict_constraint = None;
            } else {
                // Cache the culprit for the red badge/list highlight; geometry
                // keeps its last-valid positions.
                self.sketch_conflict_constraint = report.conflicting;
            }
        }
        self.rebuild_active_sketch_curves();
    }

    /// Build the current radius/setback `Dimension` from the toolbar text (a
    /// number or a variable expression).
    pub(crate) fn corner_radius_dim(&self) -> Dimension {
        let text = self.corner_radius_text.clone();
        let value = self.eval_dim(&text).unwrap_or(5.0).max(0.0);
        if zerocad_core::expr::preserves_source(&text) {
            Dimension {
                value,
                expr: Some(text.trim().to_string()),
            }
        } else {
            Dimension::literal(value)
        }
    }

    /// The uncommitted corner mods: one per pending corner, all sharing the
    /// current toolbar radius and the active tool's kind. Empty unless the
    /// Fillet/Chamfer tool is armed.
    pub(crate) fn pending_corner_mods(&self) -> Vec<CornerMod> {
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return Vec::new();
        };
        if self.pending_corners.is_empty() {
            return Vec::new();
        }
        let radius = self.corner_radius_dim();
        self.pending_corners
            .iter()
            .map(|&at| CornerMod {
                at,
                radius: radius.clone(),
                kind,
            })
            .collect()
    }

    /// Stage the sketch corner nearest `at` for a fillet/chamfer. It previews
    /// immediately (live) at the current radius; nothing is committed until the
    /// user presses Enter / clicks OK. The core snaps `at` to the actual corner
    /// vertex, so clicking near a corner is enough.
    pub(crate) fn stage_corner_at(&mut self, at: (f32, f32), kind: CornerKind) {
        if self.sketch_curves.segments.is_empty() {
            self.status_msg =
                "Draw straight edges first, then fillet/chamfer a corner.".to_string();
            return;
        }
        self.pending_corners.push(at);
        self.rebuild_active_sketch_curves();
        let noun = match kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        self.status_msg = format!(
            "{} previewing {} corner(s) — adjust R, click more, then Enter / OK to apply.",
            noun,
            self.pending_corners.len()
        );
    }

    /// Commit the staged fillet/chamfer corners into the sketch, capturing the
    /// current radius on each. Rebuilds the live curves (an identity rebuild,
    /// since the geometry already previewed the same mods).
    pub(crate) fn commit_pending_corners(&mut self) {
        if self.pending_corners.is_empty() {
            return;
        }
        self.push_working_sketch_undo();
        let mods = self.pending_corner_mods();
        let n = mods.len();
        self.sketch_corner_mods.extend(mods);
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg = format!("Applied to {} corner(s).", n);
    }

    /// Drop the staged (uncommitted) fillet/chamfer corners and rebuild so the
    /// preview disappears. Returns true if anything was pending.
    pub(crate) fn clear_pending_corners(&mut self) -> bool {
        if self.pending_corners.is_empty() {
            return false;
        }
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        true
    }

    /// The sharp corner nearest `at` and its **interior bisector** (unit, in
    /// sketch coords), computed from the un-rounded geometry. Used to place and
    /// orient the 2D radius drag handle. `None` for a straight/degenerate corner.
    pub(crate) fn corner_bisector(&self, at: (f32, f32)) -> Option<((f32, f32), (f32, f32))> {
        let vars = self.document.variable_map();
        // Geometry without ANY corner mods, so the pending corner is still sharp.
        let sharp =
            zerocad_core::effective_curves(&SketchCurves::new(), &self.sketch_shapes, &[], &vars);

        // Nearest segment endpoint = the corner vertex.
        let mut best: Option<((f32, f32), f32)> = None;
        for s in &sharp.segments {
            for v in [s.a, s.b] {
                let d = (v.0 - at.0).hypot(v.1 - at.1);
                if best.map_or(true, |(_, bd)| d < bd) {
                    best = Some((v, d));
                }
            }
        }
        let (v, _) = best?;

        // Unit directions of the (up to two) segments leaving that vertex.
        let mut dirs: Vec<(f32, f32)> = Vec::new();
        for s in &sharp.segments {
            let other = if (s.a.0 - v.0).hypot(s.a.1 - v.1) < 1.0e-3 {
                Some(s.b)
            } else if (s.b.0 - v.0).hypot(s.b.1 - v.1) < 1.0e-3 {
                Some(s.a)
            } else {
                None
            };
            if let Some(o) = other {
                let (dx, dy) = (o.0 - v.0, o.1 - v.1);
                let l = dx.hypot(dy);
                if l > 1.0e-4 {
                    dirs.push((dx / l, dy / l));
                }
            }
            if dirs.len() == 2 {
                break;
            }
        }
        if dirs.len() < 2 {
            return None;
        }
        let (bx, by) = (dirs[0].0 + dirs[1].0, dirs[0].1 + dirs[1].1);
        let bl = (bx * bx + by * by).sqrt();
        if bl < 1.0e-4 {
            return None; // 180° "corner" — no bisector
        }
        Some((v, (bx / bl, by / bl)))
    }

    /// Fusion-style floating radius/setback box for the 2D Fillet/Chamfer tool,
    /// anchored on the staged corner (or the live cursor) via `corner_dim_pos`.
    /// Edits the same `corner_radius_text` the toolbar shows; while corners are
    /// staged, every keystroke re-previews them live. Variables/expressions are
    /// accepted via the shared autocomplete.
    pub(crate) fn show_corner_radius_box(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode {
            return;
        }
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return;
        };
        let Some(pos) = self.corner_dim_pos else {
            return;
        };
        let label = match kind {
            CornerKind::Fillet => "R",
            CornerKind::Chamfer => "D",
        };
        let unit_suffix = self.current_unit.suffix();
        let var_names = self.visible_variable_names();
        let var_map = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        let mut changed = false;

        egui::Area::new(egui::Id::new("corner_radius_inline"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(3.0)
                    .stroke(egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgb(170, 180, 190),
                    ))
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 2.0),
                        blur: 8.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(35),
                    })
                    .inner_margin(egui::Margin::symmetric(8.0, 5.0))
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(label)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                            ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                            ui.style_mut().visuals.widgets.inactive.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.widgets.hovered.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.selection.bg_fill =
                                egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);
                            let field_id = egui::Id::new("corner_radius_field");
                            let outcome = crate::expr::autocomplete_field(
                                ui,
                                field_id,
                                &mut self.corner_radius_text,
                                50.0,
                                true,
                                false,
                                false,
                                &var_names,
                                &mut ac,
                            );
                            if outcome.response.changed() {
                                changed = true;
                            }
                            ui.label(
                                egui::RichText::new(unit_suffix)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                            if zerocad_core::expr::preserves_source(&self.corner_radius_text) {
                                if let Ok(value) =
                                    crate::expr::eval(&self.corner_radius_text, &var_map)
                                {
                                    ui.label(
                                        egui::RichText::new(format!("= {value:.2}"))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(70, 120, 70)),
                                    );
                                }
                            }
                        });
                    });
            });
        self.autocomplete = ac;
        // Live: editing the size re-previews the staged corners.
        if changed && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }

    /// The Fusion-style drag manipulator for the 2D Fillet/Chamfer radius: a
    /// handle on the staged corner's bisector, joined to the corner by a guide
    /// line. Dragging it along that axis grows/shrinks the radius live, in sync
    /// with the size box and toolbar field. Mirrors the 3D edge handle, in the
    /// sketch plane.
    pub(crate) fn drag_corner_radius_handle(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.active_tool.and_then(|t| t.corner_kind()).is_none() {
            return;
        }
        let Some((corner, hpos, axis)) = self.corner_handle else {
            return;
        };
        let len2 = axis.length_sq();
        let r = 7.0;
        let mut dragged = false;

        egui::Area::new(egui::Id::new("corner_radius_handle"))
            .order(egui::Order::Foreground)
            .fixed_pos(hpos - egui::vec2(r, r))
            .show(ctx, |ui| {
                ui.set_clip_rect(ctx.screen_rect());
                let (_rect, resp) =
                    ui.allocate_exact_size(egui::vec2(r * 2.0, r * 2.0), egui::Sense::drag());
                let painter = ui.painter();
                let active = resp.hovered() || resp.dragged();
                let accent = if active {
                    egui::Color32::from_rgb(0, 120, 215)
                } else {
                    egui::Color32::from_rgb(255, 140, 0)
                };
                painter.line_segment([corner, hpos], egui::Stroke::new(1.5, accent));
                painter.circle_filled(hpos, r, accent);
                painter.circle_stroke(hpos, r, egui::Stroke::new(1.5, egui::Color32::WHITE));

                if resp.dragged() && len2 > 1.0e-6 {
                    let d = resp.drag_delta();
                    let delta_mm = (d.x * axis.x + d.y * axis.y) / len2;
                    let cur = self.eval_dim(&self.corner_radius_text).unwrap_or(5.0);
                    let next = (cur + delta_mm).clamp(0.1, 1000.0);
                    self.corner_radius_text = format!("{:.2}", next);
                    dragged = true;
                }
                resp.on_hover_cursor(egui::CursorIcon::ResizeHorizontal);
            });

        // Re-preview the staged corners only when the handle actually moved.
        if dragged && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }
}

#[cfg(test)]
mod working_sketch_undo_tests {
    use super::*;

    fn rectangle(width: f32) -> SketchShape {
        SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(width),
            h: Dimension::literal(5.0),
            from_center: false,
        }
    }

    #[test]
    fn multi_entity_working_sketch_transaction_undoes_atomically() {
        let mut app = ZeroCadApp::new();
        app.sketch_shapes.push(rectangle(10.0));
        app.rebuild_active_sketch_curves();
        let before = app.sketch_curves.clone();

        app.push_working_sketch_undo();
        app.sketch_shapes.push(rectangle(20.0));
        app.sketch_mirrors.push(zerocad_core::SketchMirror {
            a: (0.0, 0.0),
            b: (0.0, 1.0),
        });
        app.sketch_corner_mods.push(CornerMod {
            at: (0.0, 0.0),
            radius: Dimension::literal(1.0),
            kind: CornerKind::Chamfer,
        });
        app.rebuild_active_sketch_curves();

        app.undo_last_sketch_action();
        assert_eq!(app.sketch_shapes.len(), 1);
        assert!(app.sketch_mirrors.is_empty());
        assert!(app.sketch_corner_mods.is_empty());
        assert_eq!(app.sketch_curves, before);
    }

    #[test]
    fn working_sketch_transactions_keep_only_fifty_snapshots() {
        let mut app = ZeroCadApp::new();
        for index in 0..55 {
            app.push_working_sketch_undo();
            app.sketch_shapes.push(rectangle(index as f32 + 1.0));
        }
        assert_eq!(app.working_sketch_undo.len(), 50);
    }
}
