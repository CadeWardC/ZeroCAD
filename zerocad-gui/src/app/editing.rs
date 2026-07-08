use crate::*;

impl ZeroCadApp {
    /// Enter sketch mode on coordinate system `cs`: save the current camera,
    /// animate to look straight at the plane, switch to orthographic, and clear
    /// any in-progress sketch. `now` is the current input time (for the anim).
    pub(crate) fn begin_sketch_on(&mut self, cs: CoordinateSystem, now: f64) {
        self.pre_sketch_pitch = self.camera_pitch;
        self.pre_sketch_yaw = self.camera_yaw;
        self.pre_sketch_perspective = self.is_perspective;

        let (target_pitch, target_yaw) = Self::camera_look_at_normal(cs.n);
        self.camera_anim_active = true;
        self.camera_anim_start_pitch = self.camera_pitch;
        self.camera_anim_start_yaw = self.camera_yaw;
        self.camera_anim_target_pitch = target_pitch;
        self.camera_anim_target_yaw = target_yaw;
        self.camera_anim_start_time = now;

        self.active_sketch_cs = cs;
        self.is_plane_selection_mode = false;
        self.is_sketch_mode = true;
        self.reset_sketch_state();
        self.is_perspective = false;
        self.selected_body.clear();
    }

    /// Re-enter a committed sketch for editing: load its geometry (and its
    /// constraint model — legacy shape sketches are promoted to the entity
    /// model on entry) into the live sketch state, and remember the node id so
    /// Finish updates it **in place**. Keeping the id is what preserves
    /// `sketch_face_refs`, dependency edges, and every downstream captured
    /// reference through the edit.
    pub(crate) fn edit_sketch(&mut self, node_id: &str, now: f64) {
        let Some(idx) = self
            .graph
            .graph
            .node_indices()
            .find(|&i| self.graph.graph[i].id == node_id)
        else {
            return;
        };
        let vars = self.graph.variable_map();
        let (cs, shapes, corner_mods, on_face, entity_ids, next_id, solver) =
            match &self.graph.graph[idx].feature {
                FeatureType::Sketch {
                    cs,
                    shapes,
                    corner_mods,
                    on_face,
                    entity_ids,
                    next_entity_id,
                    solver,
                    ..
                } => (
                    *cs,
                    shapes.clone(),
                    corner_mods.clone(),
                    *on_face,
                    entity_ids.clone(),
                    *next_entity_id,
                    solver.clone(),
                ),
                _ => return,
            };

        self.begin_sketch_on(cs, now); // clears sketch state; set ours after
        self.active_sketch_on_face = on_face;
        self.active_sketch_face_ref = self.graph.sketch_face_refs.get(node_id).cloned();
        // Restore the projected face outline so it re-joins region detection
        // (and rendering/snapping) for the whole edit session.
        self.active_face_boundary = self
            .graph
            .sketch_face_boundaries
            .get(node_id)
            .cloned()
            .unwrap_or_default();
        self.editing_sketch_id = Some(node_id.to_string());
        self.sketch_shapes = shapes.clone();
        self.sketch_entity_ids =
            zerocad_core::sketch::effective_shape_ids(shapes.len(), &entity_ids);
        self.sketch_corner_mods = corner_mods;
        // Editing means constraints: promote a legacy shapes sketch to the
        // entity model (geometry-lossless, provenance preserved via
        // `derived_from`); an already-promoted sketch loads as-is.
        let (model, next) = match solver.filter(|m| !m.is_empty()) {
            Some(model) => (model, next_id),
            None => zerocad_core::sketch::constraints::promote_shapes_to_entities(
                &shapes,
                &entity_ids,
                &vars,
                next_id,
            ),
        };
        self.sketch_solver_model = Some(model);
        self.sketch_next_entity_id = next;
        self.rebuild_active_sketch_curves();
        self.status_msg = format!(
            "Editing sketch {node_id}: drag points to move constrained geometry; \
             Finish Sketch commits in place."
        );
    }

    /// Camera (pitch, yaw) that looks straight at a plane with outward normal
    /// `n` (the normal points toward the camera). Reproduces the XY/XZ/YZ locks
    /// for axis-aligned normals and works for any orientation.
    pub(crate) fn camera_look_at_normal(n: Vec3) -> (f32, f32) {
        let yaw = n.x.atan2(n.z);
        let horiz = (n.x * n.x + n.z * n.z).sqrt();
        let pitch = n.y.atan2(horiz);
        (pitch, yaw)
    }

    /// Map a screen point to (u, v) coordinates on `cs`'s plane by intersecting
    /// the click ray with the plane. Assumes the orthographic projection used
    /// while sketching, so the inverse is exact and what-you-draw lands under
    /// the cursor for ANY plane orientation.
    pub(crate) fn screen_to_sketch(
        &self,
        screen: egui::Pos2,
        rect: egui::Rect,
        cs: &CoordinateSystem,
    ) -> (f32, f32) {
        let center_x = rect.center().x + self.camera_pan.x;
        let center_y = rect.center().y + self.camera_pan.y;
        let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
        let (sp, cp) = (self.camera_pitch.sin(), self.camera_pitch.cos());
        let (sy, cy) = (self.camera_yaw.sin(), self.camera_yaw.cos());

        // Camera-space click coords; depth `d` is free along the view ray.
        let rx = (screen.x - center_x) / scale;
        let ry = (center_y - screen.y) / scale;

        // World point P(d) = A + B*d (inverse of the ortho rotation).
        let (y_a, y_b) = (cp * ry, sp);
        let (rz_a, rz_b) = (-sp * ry, cp);
        let (x_a, x_b) = (cy * rx + sy * rz_a, sy * rz_b);
        let (z_a, z_b) = (-sy * rx + cy * rz_a, cy * rz_b);

        let n = cs.n;
        let o = cs.origin;
        // Solve (A + B d - o)·n = 0 for d.
        let bn = x_b * n.x + y_b * n.y + z_b * n.z;
        let d = if bn.abs() < 1e-6 {
            0.0
        } else {
            ((o.x - x_a) * n.x + (o.y - y_a) * n.y + (o.z - z_a) * n.z) / bn
        };
        let rel = Vec3::new(
            x_a + x_b * d - o.x,
            y_a + y_b * d - o.y,
            z_a + z_b * d - o.z,
        );
        (rel.dot(cs.u), rel.dot(cs.v))
    }

    /// Snap a raw sketch-plane point, returning only the snapped position.
    /// Thin wrapper over [`snap_sketch_point_kind`] for callers that don't need
    /// to know which feature was hit (e.g. committing a click).
    pub(crate) fn snap_sketch_point(&self, raw: (f32, f32), scale: f32, shift: bool) -> (f32, f32) {
        self.snap_sketch_point_kind(raw, scale, shift).0
    }

    /// Snap a raw sketch-plane point and report *what* it snapped onto. With
    /// Shift held, returns it unchanged with no snap kind (free placement).
    /// Otherwise it prefers, in order: a nearby endpoint / circle-centre /
    /// segment-midpoint (each distinguished so the viewport can draw its glyph),
    /// then the nearest point on a segment, then a fine 0.2-unit grid. `scale`
    /// is screen-pixels-per-unit so the snap radius stays a constant on-screen
    /// distance.
    pub(crate) fn snap_sketch_point_kind(
        &self,
        raw: (f32, f32),
        scale: f32,
        shift: bool,
    ) -> ((f32, f32), Option<SnapKind>) {
        if shift {
            return (raw, None);
        }
        let tol = 9.0 / scale.max(1e-4); // ~9 px in world units
        let dist2 = |a: (f32, f32), b: (f32, f32)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);

        // 1. Snap points: endpoints, midpoints, circle centres. The closest one
        // wins, and it carries the kind so the caller draws the matching glyph.
        let mut best_pt: Option<((f32, f32), SnapKind, f32)> = None;
        let mut consider = |p: (f32, f32), kind: SnapKind| {
            let d = dist2(p, raw);
            if d < tol * tol && best_pt.map_or(true, |(_, _, bd)| d < bd) {
                best_pt = Some((p, kind, d));
            }
        };
        // Drawn segments plus the projected face boundary (sketch-on-face):
        // snapping onto the body's edges/corners is what lets a drawn profile
        // land exactly on the face outline.
        for s in self
            .sketch_curves
            .segments
            .iter()
            .chain(self.active_face_boundary.segments.iter())
        {
            consider(s.a, SnapKind::Endpoint);
            consider(s.b, SnapKind::Endpoint);
            consider(
                ((s.a.0 + s.b.0) * 0.5, (s.a.1 + s.b.1) * 0.5),
                SnapKind::Midpoint,
            );
        }
        for c in &self.sketch_curves.circles {
            consider(c.center, SnapKind::Center);
        }
        // Fillet arcs: their endpoints act as the new corners where the arc meets
        // the straight edges, and their centre is a genuine centre to snap onto.
        for a in &self.sketch_curves.arcs {
            consider(a.start, SnapKind::Endpoint);
            consider(a.end, SnapKind::Endpoint);
            consider(a.center, SnapKind::Center);
        }
        if let Some((p, kind, _)) = best_pt {
            return (p, Some(kind));
        }

        // 2. Snap to the nearest point on a segment (drawn or face boundary).
        let mut best_line: Option<((f32, f32), f32)> = None;
        for s in self
            .sketch_curves
            .segments
            .iter()
            .chain(self.active_face_boundary.segments.iter())
        {
            let proj = project_point_on_segment(raw, s.a, s.b);
            let d = dist2(proj, raw);
            if d < tol * tol && best_line.map_or(true, |(_, bd)| d < bd) {
                best_line = Some((proj, d));
            }
        }
        if let Some((p, _)) = best_line {
            return (p, Some(SnapKind::OnLine));
        }

        // 3. Fine grid snap (0.2 units).
        (
            ((raw.0 * 5.0).round() / 5.0, (raw.1 * 5.0).round() / 5.0),
            Some(SnapKind::Grid),
        )
    }

    /// Refresh the live (unlocked, untyped) dimension fields from the current
    /// cursor position so the dialog shows the value the cursor would produce.
    /// Only the 2-point tools carry inline dimensions; 3-point tools have none.
    pub(crate) fn update_dim_live(&mut self, start: (f32, f32), cursor: (f32, f32)) {
        let Some(tool) = self.active_tool else {
            return;
        };
        let Some(dim) = self.dim_input.as_mut() else {
            return;
        };
        let dx = cursor.0 - start.0;
        let dy = cursor.1 - start.1;
        let live: Vec<f32> = match tool {
            SketchTool::Rectangle => vec![dx.abs(), dy.abs()],
            SketchTool::RectangleCenter => vec![2.0 * dx.abs(), 2.0 * dy.abs()],
            SketchTool::Circle => vec![2.0 * (dx * dx + dy * dy).sqrt()],
            SketchTool::Line => vec![(dx * dx + dy * dy).sqrt(), dy.atan2(dx).to_degrees()],
            // 3-point tools draw without inline dimension fields.
            _ => vec![],
        };
        for (i, f) in dim.fields.iter_mut().enumerate() {
            if !f.locked && !f.edited {
                if let Some(v) = live.get(i) {
                    f.value = format!("{:.2}", v);
                }
            }
        }
    }

    /// Build a parametric [`Dimension`] for dimension field `i`: it captures the
    /// raw expression text when it references a variable (so the dimension
    /// follows that variable), else a plain literal. `fallback` (the
    /// cursor-derived value) is used when the field is empty or invalid.
    pub(crate) fn dim_param(&self, i: usize, fallback: f32) -> Dimension {
        let text = self
            .dim_input
            .as_ref()
            .and_then(|d| d.fields.get(i))
            .map(|f| f.value.clone());
        match text {
            Some(t) if zerocad_core::expr::references_variable(&t) => Dimension {
                value: self.eval_dim(&t).unwrap_or(fallback),
                expr: Some(t.trim().to_string()),
            },
            Some(t) => Dimension {
                value: self.eval_dim(&t).unwrap_or(fallback),
                expr: None,
            },
            None => Dimension::literal(fallback),
        }
    }

    /// Baked geometry for the point-driven tools (rotated rectangle, 3-point
    /// circle, ellipses), which have no dimension fields to bind to variables.
    pub(crate) fn raw_curves_from_points(
        &self,
        tool: SketchTool,
        p0: (f32, f32),
        p1: (f32, f32),
        last: (f32, f32),
    ) -> SketchCurves {
        let mut sc = SketchCurves::new();
        match tool {
            SketchTool::RectangleThreePoint => {
                // p0→p1 is one edge; the third point sets the perpendicular height.
                let (bx, by) = (p1.0 - p0.0, p1.1 - p0.1);
                let blen = (bx * bx + by * by).sqrt();
                if blen > 1e-4 {
                    let (ux, uy) = (bx / blen, by / blen);
                    let (px, py) = (-uy, ux); // unit perpendicular
                    let h = (last.0 - p1.0) * px + (last.1 - p1.1) * py;
                    let c2 = (p1.0 + px * h, p1.1 + py * h);
                    let c3 = (p0.0 + px * h, p0.1 + py * h);
                    sc.add_line(p0, p1);
                    sc.add_line(p1, c2);
                    sc.add_line(c2, c3);
                    sc.add_line(c3, p0);
                }
            }
            SketchTool::ThreePointCircle => {
                if let Some((c, r)) = circumcircle(p0, p1, last) {
                    sc.add_circle(c, r);
                }
            }
            SketchTool::Ellipse => {
                // p0 = center, p1 = major-axis endpoint, last = minor extent.
                let major = (p1.0 - p0.0, p1.1 - p0.1);
                let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
                if rx > 1e-4 {
                    let (pxu, pyu) = (-major.1 / rx, major.0 / rx);
                    let ry = ((last.0 - p0.0) * pxu + (last.1 - p0.1) * pyu).abs();
                    sc.add_ellipse(p0, major, ry.max(0.01));
                }
            }
            SketchTool::ThreePointEllipse => {
                // p0,p1 = major-axis diameter endpoints; last = minor extent.
                let c = ((p0.0 + p1.0) * 0.5, (p0.1 + p1.1) * 0.5);
                let major = ((p1.0 - p0.0) * 0.5, (p1.1 - p0.1) * 0.5);
                let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
                if rx > 1e-4 {
                    let (pxu, pyu) = (-major.1 / rx, major.0 / rx);
                    let ry = ((last.0 - c.0) * pxu + (last.1 - c.1) * pyu).abs();
                    sc.add_ellipse(c, major, ry.max(0.01));
                }
            }
            SketchTool::PolygonInscribed | SketchTool::PolygonCircumscribed => {
                // p0 = center, `last` = the rim point (its distance sets the size,
                // its direction the rotation). Inscribed puts a vertex under the
                // cursor (drag = circumradius); circumscribed puts an edge-flat
                // midpoint under the cursor (drag = apothem).
                let n = self.polygon_sides.clamp(3, 64) as usize;
                let (dx, dy) = (last.0 - p0.0, last.1 - p0.1);
                let drag = (dx * dx + dy * dy).sqrt();
                if drag > 1e-4 {
                    let step = std::f32::consts::TAU / n as f32;
                    let ang0 = dy.atan2(dx);
                    // Circumradius and the angle of the first vertex.
                    let (r, first) = if tool == SketchTool::PolygonCircumscribed {
                        // Cursor = apothem; a flat faces the cursor, so the first
                        // vertex sits half a step back from `ang0`.
                        (drag / (step * 0.5).cos(), ang0 - step * 0.5)
                    } else {
                        (drag, ang0)
                    };
                    let verts: Vec<(f32, f32)> = (0..n)
                        .map(|k| {
                            let a = first + step * k as f32;
                            (p0.0 + r * a.cos(), p0.1 + r * a.sin())
                        })
                        .collect();
                    for k in 0..n {
                        sc.add_line(verts[k], verts[(k + 1) % n]);
                    }
                }
            }
            _ => {}
        }
        sc
    }

    /// Build the **parametric record** for the in-progress shape from the placed
    /// points + `last`. Dimensioned 2-point tools capture their dimension
    /// expressions (so they follow variables); point-driven tools are baked into
    /// [`SketchShape::Raw`].
    pub(crate) fn shape_record_from_points(&self, last: (f32, f32)) -> Option<SketchShape> {
        let tool = self.active_tool?;
        let &p0 = self.sketch_points.first()?;
        let p1 = self.sketch_points.get(1).copied().unwrap_or(last);
        let dx = last.0 - p0.0;
        let dy = last.1 - p0.1;
        let shape = match tool {
            SketchTool::Line => SketchShape::Line {
                start: p0,
                length: self.dim_param(0, (dx * dx + dy * dy).sqrt()),
                angle_deg: self.dim_param(1, dy.atan2(dx).to_degrees()),
            },
            SketchTool::Rectangle => SketchShape::Rectangle {
                origin: p0,
                sx: if dx < 0.0 { -1.0 } else { 1.0 },
                sy: if dy < 0.0 { -1.0 } else { 1.0 },
                w: self.dim_param(0, dx.abs()),
                h: self.dim_param(1, dy.abs()),
                from_center: false,
            },
            SketchTool::RectangleCenter => SketchShape::Rectangle {
                origin: p0,
                sx: 1.0,
                sy: 1.0,
                w: self.dim_param(0, 2.0 * dx.abs()),
                h: self.dim_param(1, 2.0 * dy.abs()),
                from_center: true,
            },
            SketchTool::Circle => SketchShape::Circle {
                center: p0,
                diameter: self.dim_param(0, 2.0 * (dx * dx + dy * dy).sqrt()),
            },
            SketchTool::RectangleThreePoint
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse
            | SketchTool::PolygonInscribed
            | SketchTool::PolygonCircumscribed => SketchShape::Raw {
                curves: self.raw_curves_from_points(tool, p0, p1, last),
            },
            // Mirror reflects existing geometry across its 2-click axis; it's
            // committed via `commit_sketch_mirror`, not the shape pipeline.
            // Fillet/Chamfer modify existing corners; they don't create shapes.
            SketchTool::Mirror | SketchTool::Fillet | SketchTool::Chamfer => return None,
        };
        Some(shape)
    }

    /// The in-progress shape resolved to [`SketchCurves`] — the parametric record
    /// built against the current variables. Single source of truth for the live
    /// preview and the committed geometry, so they can never diverge.
    pub(crate) fn shape_from_points(&self, last: (f32, f32)) -> SketchCurves {
        match self.shape_record_from_points(last) {
            Some(shape) => shape.build(&self.graph.variable_map()),
            None => SketchCurves::new(),
        }
    }

    /// Commit the in-progress shape: append its parametric record to the sketch,
    /// then rebuild the live curves from the shape list. Clears the drawing state.
    ///
    /// In an Edit Sketch session (constraint model active) the new shape is
    /// promoted into the model immediately — with its structural constraints
    /// (a rectangle arrives horizontal/vertical, dimensioned, and anchored) —
    /// so it participates in drag-to-solve like everything else.
    pub(crate) fn finalize_shape(&mut self, last: (f32, f32)) {
        if self.sketch_points.is_empty() {
            return;
        }
        // Continuous-Line chaining bookkeeping (see the chaining block below).
        let seg_start = self.sketch_points.first().copied();
        let is_line = matches!(self.active_tool, Some(SketchTool::Line));
        // Closing the loop = the snapped placement landed back on the chain's
        // start. Test the SNAPPED point `last` (exact), NOT the rebuilt endpoint:
        // the inline dims are quantized to 2 decimals (update_dim_live), so the
        // rebuilt endpoint can sit ~0.05 mm off and miss the tolerance.
        let is_closing = is_line
            && self.line_chain_start.map_or(false, |cs| {
                (last.0 - cs.0).powi(2) + (last.1 - cs.1).powi(2) < 1.0e-4
            });
        // When closing, drop the quantized inline dims so the closing segment is
        // rebuilt at full precision from the snapped endpoints — otherwise its
        // endpoint lands > VERTEX_TOL (1e-3 mm) from the start and detect_regions
        // won't merge the loop into a face.
        if is_closing {
            self.dim_input = None;
        }
        let mut line_endpoint: Option<(f32, f32)> = None;
        let mut inferred_total = 0usize;
        if let Some(shape) = self.shape_record_from_points(last) {
            // Continuous-Line: resolve the committed segment — reject a
            // degenerate (zero-length) one so a stray/duplicate click can't
            // inject a zero-length line or a false loop close, and capture the
            // true endpoint (for a typed-dimension segment it differs from `last`).
            if is_line {
                if let Some(s) = shape.build(&self.graph.variable_map()).segments.last() {
                    let len2 = (s.b.0 - s.a.0).powi(2) + (s.b.1 - s.a.1).powi(2);
                    if len2 < 1.0e-8 {
                        return;
                    }
                    line_endpoint = Some(s.b);
                }
            }
            if self.sketch_solver_model.is_some() {
                let vars = self.graph.variable_map();
                let shape_id = zerocad_core::sketch::EntityId(self.sketch_next_entity_id);
                let (addition, next) =
                    zerocad_core::sketch::constraints::promote_shapes_to_entities(
                        std::slice::from_ref(&shape),
                        &[shape_id],
                        &vars,
                        self.sketch_next_entity_id + 1,
                    );
                let mut next = next;
                if let Some(model) = &mut self.sketch_solver_model {
                    // Constraint INFERENCE: the click coordinates were already
                    // snapped, so structural relationships are read off exactly.
                    // A new point landing on an existing point becomes a
                    // persisted Coincident (endpoint snap → real constraint,
                    // not a lucky coordinate), and an axis-aligned drawn line
                    // gets Horizontal/Vertical.
                    use zerocad_core::sketch::{Constraint, EntityId, SketchEntity};
                    let mut alloc = || {
                        let id = EntityId(next);
                        next += 1;
                        id
                    };
                    let mut inferred: Vec<Constraint> = Vec::new();
                    for new_p in &addition.points {
                        if let Some(existing) = model
                            .points
                            .iter()
                            .find(|p| p.pos.0 == new_p.pos.0 && p.pos.1 == new_p.pos.1)
                        {
                            inferred.push(Constraint::Coincident {
                                id: alloc(),
                                a: existing.id,
                                b: new_p.id,
                            });
                        }
                    }
                    // Rectangle promotion carries its own H/V; infer only for a
                    // bare drawn Line.
                    if matches!(shape, SketchShape::Line { .. }) {
                        for e in &addition.entities {
                            if let SketchEntity::Line { id, p0, p1, .. } = e {
                                let a = addition.points.iter().find(|p| p.id == *p0);
                                let b = addition.points.iter().find(|p| p.id == *p1);
                                if let (Some(a), Some(b)) = (a, b) {
                                    if (a.pos.1 - b.pos.1).abs() < 1.0e-9 {
                                        inferred.push(Constraint::Horizontal {
                                            id: alloc(),
                                            line: *id,
                                        });
                                    } else if (a.pos.0 - b.pos.0).abs() < 1.0e-9 {
                                        inferred.push(Constraint::Vertical {
                                            id: alloc(),
                                            line: *id,
                                        });
                                    }
                                }
                            }
                        }
                    }
                    let inferred_count = inferred.len();
                    model.points.extend(addition.points);
                    model.entities.extend(addition.entities);
                    model.constraints.extend(addition.constraints);
                    model.constraints.extend(inferred);
                    inferred_total = inferred_count;
                }
                self.sketch_entity_ids.push(shape_id);
                self.sketch_next_entity_id = next;
            }
            self.sketch_shapes.push(shape);
        }
        self.rebuild_active_sketch_curves();
        self.cancel_in_progress_shape();

        // Continuous Line: instead of a one-shot reset, chain the next segment
        // from this endpoint — or, if the endpoint lands back on the chain's
        // start, close the loop (the region/face already formed in the rebuild
        // above). Every other tool keeps the one-shot "Shape added" behavior.
        if is_line {
            if is_closing {
                // The loop just closed; the face formed in the rebuild above.
                self.line_chain_start = None;
                self.status_msg = "Loop closed — face created.".to_string();
            } else if let (Some(seg_start), Some(endpoint)) = (seg_start, line_endpoint) {
                // First segment of a chain records its start for close detection.
                self.line_chain_start.get_or_insert(seg_start);
                // Re-seed: the next click continues the chain from this endpoint,
                // and the inline Length/Angle dialog re-opens anchored at it.
                self.sketch_points = vec![endpoint];
                self.sketch_temp_start = Some(endpoint);
                self.dim_input = Some(DimInput {
                    fields: dim_fields_for(SketchTool::Line),
                    focus_request: Some(0),
                    active_field: 0,
                    select_all: true,
                });
                self.status_msg =
                    "Segment added — click the next point, or click the start to close (Esc to finish)."
                        .to_string();
            }
            return;
        }

        self.status_msg = if inferred_total > 0 {
            format!(
                "Shape added ({inferred_total} constraint(s) inferred) — click to start another."
            )
        } else {
            "Shape added — click to start another.".to_string()
        };
    }

    /// Total `(vertices, triangles)` across a body-mesh list. Vertices are 6
    /// floats (pos + normal); indices are 3 per triangle.
    pub(crate) fn mesh_totals(meshes: &[(String, MockMesh)]) -> (usize, usize) {
        meshes.iter().fold((0, 0), |(v, t), (_, m)| {
            (v + m.vertices.len() / 6, t + m.indices.len() / 3)
        })
    }

    /// Count graph features matching `pred`, then add one — the 1-based index
    /// for the next feature of that kind. Shared by the `next_*_name` helpers.
    pub(crate) fn next_feature_index(&self, pred: impl Fn(&FeatureType) -> bool) -> usize {
        self.graph
            .graph
            .node_indices()
            .filter(|&i| pred(&self.graph.graph[i].feature))
            .count()
            + 1
    }

    /// Display name for the next sketch (Sketch_1, Sketch_2, …).
    pub(crate) fn next_sketch_name(&self) -> String {
        let n = self.next_feature_index(|f| matches!(f, FeatureType::Sketch { .. }));
        format!("Sketch_{}", n)
    }

    /// Display name for the next body (Body_1, Body_2, …) across all solid types.
    pub(crate) fn next_body_name(&self) -> String {
        let n = self.next_feature_index(|f| {
            matches!(
                f,
                FeatureType::Box { .. }
                    | FeatureType::Cylinder { .. }
                    | FeatureType::Extrude {
                        mode: ExtrudeMode::NewBody,
                        ..
                    }
            )
        });
        format!("Body_{}", n)
    }

    /// Display name for an extrude that modifies existing bodies rather than
    /// owning a standalone body.
    pub(crate) fn next_operation_name(&self, mode: ExtrudeMode) -> String {
        let prefix = match mode {
            ExtrudeMode::NewBody => return self.next_body_name(),
            ExtrudeMode::Join => "Join",
            ExtrudeMode::Cut => "Cut",
        };
        let n = self.next_feature_index(
            |f| matches!(f, FeatureType::Extrude { mode: m, .. } if *m == mode),
        );
        format!("{}_{}", prefix, n)
    }

    /// Display name for the next variable set (VariableSet_1, VariableSet_2, …).
    pub(crate) fn next_variable_set_name(&self) -> String {
        let n = self.next_feature_index(|f| matches!(f, FeatureType::VariableSet { .. }));
        format!("VariableSet_{}", n)
    }
}
