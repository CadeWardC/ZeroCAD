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

    /// Snap a raw sketch-plane point. With Shift held, returns it unchanged
    /// (free placement). Otherwise it prefers, in order: a nearby endpoint /
    /// circle-centre / segment-midpoint, then the nearest point on a segment,
    /// then a fine 0.2-unit grid. `scale` is screen-pixels-per-unit so the snap
    /// radius stays a constant on-screen distance.
    pub(crate) fn snap_sketch_point(&self, raw: (f32, f32), scale: f32, shift: bool) -> (f32, f32) {
        if shift {
            return raw;
        }
        let tol = 9.0 / scale.max(1e-4); // ~9 px in world units
        let dist2 = |a: (f32, f32), b: (f32, f32)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);

        // 1. Snap points: endpoints, midpoints, circle centres.
        let mut best_pt: Option<((f32, f32), f32)> = None;
        let mut consider = |p: (f32, f32)| {
            let d = dist2(p, raw);
            if d < tol * tol && best_pt.map_or(true, |(_, bd)| d < bd) {
                best_pt = Some((p, d));
            }
        };
        for s in &self.sketch_curves.segments {
            consider(s.a);
            consider(s.b);
            consider(((s.a.0 + s.b.0) * 0.5, (s.a.1 + s.b.1) * 0.5));
        }
        for c in &self.sketch_curves.circles {
            consider(c.center);
        }
        if let Some((p, _)) = best_pt {
            return p;
        }

        // 2. Snap to the nearest point on a segment.
        let mut best_line: Option<((f32, f32), f32)> = None;
        for s in &self.sketch_curves.segments {
            let proj = project_point_on_segment(raw, s.a, s.b);
            let d = dist2(proj, raw);
            if d < tol * tol && best_line.map_or(true, |(_, bd)| d < bd) {
                best_line = Some((proj, d));
            }
        }
        if let Some((p, _)) = best_line {
            return p;
        }

        // 3. Fine grid snap (0.2 units).
        ((raw.0 * 5.0).round() / 5.0, (raw.1 * 5.0).round() / 5.0)
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
            | SketchTool::ThreePointEllipse => SketchShape::Raw {
                curves: self.raw_curves_from_points(tool, p0, p1, last),
            },
            // Fillet/Chamfer modify existing corners; they don't create shapes.
            SketchTool::Fillet | SketchTool::Chamfer => return None,
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
    pub(crate) fn finalize_shape(&mut self, last: (f32, f32)) {
        if self.sketch_points.is_empty() {
            return;
        }
        if let Some(shape) = self.shape_record_from_points(last) {
            self.sketch_shapes.push(shape);
        }
        self.rebuild_active_sketch_curves();
        self.cancel_in_progress_shape();
        self.status_msg = "Shape added — click to start another.".to_string();
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
