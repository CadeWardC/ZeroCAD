use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_workspace_viewport(&mut self, ctx: &egui::Context) {
        // CENTRAL PANEL: 3D CAD Viewport
        egui::CentralPanel::default().show(ctx, |ui| {
                    // Draw a nice border and frame around the viewport
                    egui::Frame::canvas(ui.style()).show(ui, |ui| {
                        let (rect, response) = ui.allocate_exact_size(
                            ui.available_size() - egui::vec2(0.0, 4.0),
                            egui::Sense::click() | egui::Sense::drag(),
                        );

                        let center_x = rect.center().x + self.camera_pan.x;
                        let center_y = rect.center().y + self.camera_pan.y;
                        let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);

                        let cos_p = self.camera_pitch.cos();
                        let sin_p = self.camera_pitch.sin();
                        let cos_y = self.camera_yaw.cos();
                        let sin_y = self.camera_yaw.sin();

                        // 3D coordinate projection mapping function
                        let project_3d = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
                            let rx = cos_y * x - sin_y * z;
                            let rz = sin_y * x + cos_y * z;
                            let ry = cos_p * y - sin_p * rz;
                            let final_z = sin_p * y + cos_p * rz;

                            if self.is_perspective {
                                let dist = 1200.0;
                                let factor = dist / (dist - final_z.min(dist * 0.85));
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

                        // Define coordinates of the 3 origin sheets in 3D for frame-perfect click & hover hit-tests.
                        // Quadrant origin planes meet at (0.0, 0.0, 0.0) with size 18.0 units (similar to Fusion 360).
                        let size = 18.0;

                        // XY Plane corners (positive X, positive Y)
                        let xy_c = [
                            project_3d(0.0, 0.0, 0.0),
                            project_3d(size, 0.0, 0.0),
                            project_3d(size, size, 0.0),
                            project_3d(0.0, size, 0.0),
                        ];
                        let xy_pts = [
                            egui::pos2(xy_c[0].0, xy_c[0].1),
                            egui::pos2(xy_c[1].0, xy_c[1].1),
                            egui::pos2(xy_c[2].0, xy_c[2].1),
                            egui::pos2(xy_c[3].0, xy_c[3].1),
                        ];

                        // XZ Plane corners (positive X, positive Z)
                        let xz_c = [
                            project_3d(0.0, 0.0, 0.0),
                            project_3d(size, 0.0, 0.0),
                            project_3d(size, 0.0, size),
                            project_3d(0.0, 0.0, size),
                        ];
                        let xz_pts = [
                            egui::pos2(xz_c[0].0, xz_c[0].1),
                            egui::pos2(xz_c[1].0, xz_c[1].1),
                            egui::pos2(xz_c[2].0, xz_c[2].1),
                            egui::pos2(xz_c[3].0, xz_c[3].1),
                        ];

                        // YZ Plane corners (positive Y, positive Z)
                        let yz_c = [
                            project_3d(0.0, 0.0, 0.0),
                            project_3d(0.0, size, 0.0),
                            project_3d(0.0, size, size),
                            project_3d(0.0, 0.0, size),
                        ];
                        let yz_pts = [
                            egui::pos2(yz_c[0].0, yz_c[0].1),
                            egui::pos2(yz_c[1].0, yz_c[1].1),
                            egui::pos2(yz_c[2].0, yz_c[2].1),
                            egui::pos2(yz_c[3].0, yz_c[3].1),
                        ];

                        // Perform frame-perfect hover checking immediately
                        let hover_pos = response.hover_pos();
                        self.hovered_plane = None;
                        self.hovered_sketch_face = None;
                        if self.is_plane_selection_mode {
                            if let Some(pos) = hover_pos {
                                // A planar body face under the cursor takes priority over
                                // the origin plane quads — sketch directly on the solid.
                                let face_hit = self
                                    .pick_body_element(pos, &project_3d, sin_p, cos_p, sin_y, cos_y)
                                    .and_then(|(node, pick)| match pick {
                                        BodyPick::Face(fid) if self.face_is_planar(&node, fid) => {
                                            Some((node, fid))
                                        }
                                        _ => None,
                                    });
                                if let Some((node, fid)) = face_hit {
                                    self.hovered_sketch_face = Some((node, fid));
                                } else if is_point_in_quad(pos, &xy_pts) {
                                    self.hovered_plane = Some(SketchPlane::XY);
                                } else if is_point_in_quad(pos, &xz_pts) {
                                    self.hovered_plane = Some(SketchPlane::XZ);
                                } else if is_point_in_quad(pos, &yz_pts) {
                                    self.hovered_plane = Some(SketchPlane::YZ);
                                }
                            }

                            if self.hovered_plane.is_some() || self.hovered_sketch_face.is_some() {
                                egui::show_tooltip_at_pointer(ctx, egui::Id::new("plane_select_tooltip"), |ui| {
                                    ui.style_mut().visuals.window_fill = egui::Color32::from_rgb(255, 255, 255);
                                    ui.style_mut().visuals.window_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(200, 200, 200));
                                    ui.label(
                                        egui::RichText::new("Select a plane or planar face")
                                            .color(egui::Color32::from_rgb(45, 45, 45))
                                            .size(12.0)
                                    );
                                });
                            }
                        }

                        // Viewport navigation. Button mapping:
                        //   • Middle-drag  → orbit (3D) / pan (sketch, where orbit is locked)
                        //   • Shift + drag → pan (3D)
                        //   • Left-drag    → selecting faces·edges / drawing shapes
                        //   • Shift (sketch) → suppress snapping while drawing
                        let pointer_delta = ctx.input(|i| i.pointer.delta());
                        let shift = ctx.input(|i| i.modifiers.shift);

                        // Middle-drag orbit/pan is latched rather than read from egui's
                        // per-frame `dragged_by`, which can momentarily report false
                        // mid-motion (drag-threshold / id churn) and make the orbit stall
                        // at random points. We start on a middle-press over the viewport
                        // and hold until the button is physically released.
                        let middle_down = ctx.input(|i| i.pointer.middle_down());
                        let middle_pressed =
                            ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Middle));
                        if middle_pressed && response.hovered() {
                            self.orbiting = true;
                        }
                        if !middle_down {
                            self.orbiting = false;
                        }
                        let middle_drag = self.orbiting;
                        let primary_drag = response.dragged_by(egui::PointerButton::Primary);
                        let any_drag = middle_drag || primary_drag;

                        if !self.camera_anim_active {
                            if self.is_planar_view() {
                                // Lock the camera perpendicular to the active plane,
                                // looking straight down its outward normal.
                                let (p, y) = Self::camera_look_at_normal(self.active_sketch_cs.n);
                                self.camera_pitch = p;
                                self.camera_yaw = y;
                                // Sketch mode: pan with middle-drag (camera can't orbit
                                // here). Shift is reserved for suppressing snapping while
                                // drawing, so it must NOT pan here.
                                if middle_drag {
                                    self.camera_pan += pointer_delta;
                                }
                            } else if self.extrude_op.is_some() {
                                // Push/pull with left-drag; Shift+drag pans; middle orbits.
                                if shift && any_drag {
                                    self.camera_pan += pointer_delta;
                                } else if middle_drag {
                                    // Drag right → model turns right, drag down → tilt
                                    // down (grab feel on both axes).
                                    self.camera_yaw -= pointer_delta.x * 0.008;
                                    self.camera_pitch = (self.camera_pitch + pointer_delta.y * 0.008)
                                        .clamp(-std::f32::consts::FRAC_PI_2 + 0.05, std::f32::consts::FRAC_PI_2 - 0.05);
                                } else if primary_drag {
                                    // Push/pull ALONG the extrude axis (the sketch-plane
                                    // normal) projected into screen space, so the drag
                                    // tracks the cursor for any plane orientation. The old
                                    // code drove depth straight off vertical mouse motion,
                                    // which only lined up when the normal pointed up the
                                    // screen (XY/XZ) — on YZ and tilted face planes it ran
                                    // backwards and lagged the cursor.
                                    let axis = self
                                        .extrude_op
                                        .as_ref()
                                        .and_then(|op| op.targets.first())
                                        .map(|t| t.cs.n);
                                    if let Some(n) = axis {
                                        let view_scale = rect.width().min(rect.height())
                                            / (self.camera_zoom * 5.0).max(1e-3);
                                        // World depth change for this drag, mapped onto the
                                        // screen projection of the axis. `None` when the
                                        // axis is too edge-on to track → vertical fallback.
                                        let delta = extrude_depth_delta(
                                            n,
                                            self.camera_pitch,
                                            self.camera_yaw,
                                            view_scale,
                                            pointer_delta,
                                        )
                                        .unwrap_or(-pointer_delta.y / view_scale.max(1e-6));
                                        if let Some(op) = self.extrude_op.as_mut() {
                                            op.depth = (op.depth + delta).clamp(-300.0, 300.0);
                                            // Mirror the dragged value into the inline box.
                                            op.depth_text = format!("{:.2}", op.depth);
                                            // Live ghost mode: while pushing/pulling we show
                                            // only the cheap tool volume, deferring the truck
                                            // boolean until the button is released.
                                            self.extrude_depth_dragging = true;
                                        }
                                    }
                                }
                            } else {
                                // Free 3D view: middle orbits, Shift+drag pans, left selects.
                                if shift && any_drag {
                                    self.camera_pan += pointer_delta;
                                } else if middle_drag {
                                    // Drag right → model turns right, drag down → tilt
                                    // down (grab feel on both axes).
                                    self.camera_yaw -= pointer_delta.x * 0.008;
                                    self.camera_pitch = (self.camera_pitch + pointer_delta.y * 0.008)
                                        .clamp(-std::f32::consts::FRAC_PI_2 + 0.05, std::f32::consts::FRAC_PI_2 - 0.05);
                                }
                            }
                        }

                        // End of a push/pull: once the primary button is released, drop
                        // the live-ghost flag so this frame's draw_viewport (below) runs
                        // the deferred truck boolean once and shows the real result.
                        if self.extrude_depth_dragging && !ctx.input(|i| i.pointer.primary_down()) {
                            self.extrude_depth_dragging = false;
                        }

                        // Zoom: Mouse scroll
                        let scroll_delta = ctx.input(|i| i.smooth_scroll_delta.y);
                        if scroll_delta != 0.0 {
                            self.camera_zoom = (self.camera_zoom * (1.0 + scroll_delta * 0.002)).clamp(1.0, 50.0);
                        }

                        // Plane selection click interaction
                        if self.is_plane_selection_mode && response.clicked() {
                            if let Some((node, fid)) = self.hovered_sketch_face.clone() {
                                // Sketch directly on the clicked planar body face — the
                                // same path as pre-selecting a face and pressing Draw
                                // Sketch (top_bar_commands): frame from `face_cs`, durable
                                // `face_ref` so the sketch follows the face, then begin.
                                if let Some(cs) = self.face_cs(&node, fid) {
                                    log::info!("Sketching on clicked body face {fid} of {node}.");
                                    let fref = self.face_ref(&node, fid);
                                    let now = ctx.input(|i| i.time);
                                    self.active_sketch_on_face = true;
                                    self.active_sketch_face_ref = fref;
                                    self.hovered_sketch_face = None;
                                    self.begin_sketch_on(cs, now);
                                    self.status_msg = "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
                                }
                            } else if let Some(plane) = self.hovered_plane {
                                log::info!("User selected plane sheet: {:?}", plane);

                                // Save current camera state before pivoting
                                self.pre_sketch_pitch = self.camera_pitch;
                                self.pre_sketch_yaw = self.camera_yaw;
                                self.pre_sketch_perspective = self.is_perspective;

                                // The origin plane becomes the active sketch coordinate
                                // system; animate the camera to look straight at it.
                                let cs = match plane {
                                    SketchPlane::XY => CoordinateSystem::XY,
                                    SketchPlane::XZ => CoordinateSystem::XZ,
                                    SketchPlane::YZ => CoordinateSystem::YZ,
                                };
                                let (target_pitch, target_yaw) = Self::camera_look_at_normal(cs.n);

                                log::info!("Initiating camera animation to pitch: {:.2}, yaw: {:.2}", target_pitch, target_yaw);
                                self.camera_anim_active = true;
                                self.camera_anim_start_pitch = self.camera_pitch;
                                self.camera_anim_start_yaw = self.camera_yaw;
                                self.camera_anim_target_pitch = target_pitch;
                                self.camera_anim_target_yaw = target_yaw;
                                self.camera_anim_start_time = ctx.input(|i| i.time);

                                self.active_sketch_cs = cs;
                                self.active_sketch_on_face = false;
                                self.is_plane_selection_mode = false;
                                self.is_sketch_mode = true;
                                self.reset_sketch_state();

                                // Set topographic mode: orthographic (parallel) projection
                                self.is_perspective = false;
                                self.status_msg = format!("Selected {:?}. Camera locked perpendicular. Active Tool: {:?}", plane, self.active_tool.map_or("Select".to_string(), |t| format!("{:?}", t)));
                            }
                        }

                        // Sketching interaction: Click inside viewport in Sketch Mode
                        // A left-drag with a shape tool sets the first point on press
                        // (so press-drag-release begins the shape); the shape is only
                        // finalized on the next click — never on the drag itself.
                        let begin_draw = response.drag_started_by(egui::PointerButton::Primary);
                        if self.is_sketch_mode
                            && self.active_tool.is_some()
                            && (response.clicked() || begin_draw)
                            && !self.camera_anim_active
                        {
                            if let Some(hover_pos) = response.interact_pointer_pos() {
                                let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                                // Map the click onto the active sketch plane via a ray /
                                // plane intersection — WYSIWYG on any plane orientation.
                                let raw = self.screen_to_sketch(hover_pos, rect, &self.active_sketch_cs);
                                let (sketch_x, sketch_y) = self.snap_sketch_point(raw, scale, shift);

                                let tool = self.active_tool.unwrap();
                                let pt = (sketch_x, sketch_y);

                                if let Some(kind) = tool.corner_kind() {
                                    // Fillet/Chamfer: a click STAGES the nearest corner
                                    // (live preview); it isn't committed until Enter / OK.
                                    // The user can stack several corners and tune R first.
                                    if response.clicked() {
                                        self.stage_corner_at(pt, kind);
                                    }
                                } else {
                                    let point_count = tool.point_count();
                                    if self.sketch_points.is_empty() {
                                        // First point (click or press-drag). 2-point tools open
                                        // the inline dimension dialog here; multi-point tools
                                        // (rotated rect, 3-point circle, ellipses) draw by
                                        // clicking each point with a live preview.
                                        self.sketch_points.push(pt);
                                        self.sketch_temp_start = Some(pt);
                                        if point_count == 2 {
                                            self.dim_anchor = Some(hover_pos);
                                            self.dim_input = Some(DimInput {
                                                fields: dim_fields_for(tool),
                                                focus_request: Some(0),
                                                active_field: 0,
                                                select_all: true,
                                            });
                                            self.status_msg =
                                                "First point set — move and click, or type dimensions (Tab/Enter)."
                                                    .to_string();
                                        } else {
                                            self.status_msg = format!(
                                                "Point 1 of {} set — click to place the next point.",
                                                point_count
                                            );
                                        }
                                    } else if response.clicked() {
                                        // A subsequent explicit click. Finalize once enough
                                        // points exist, otherwise record an intermediate point.
                                        if self.sketch_points.len() + 1 >= point_count {
                                            self.finalize_shape(pt);
                                        } else {
                                            self.sketch_points.push(pt);
                                            self.status_msg = format!(
                                                "Point {} of {} set — click to place the next point.",
                                                self.sketch_points.len(),
                                                point_count
                                            );
                                        }
                                    }
                                }
                            }
                        }

                        // 3D selection: click picks a body face/edge/vertex (or a finished
                        // sketch's face/edge); double-click selects the whole body/sketch.
                        // Works in normal 3D view, and while sketching when no drawing
                        // tool is armed (the Select state) — so body geometry can be
                        // selected without leaving the sketch.
                        if (response.clicked() || response.double_clicked())
                            && (!self.is_sketch_mode || self.active_tool.is_none())
                            && !self.is_plane_selection_mode
                            && self.extrude_op.is_none()
                            && self.edge_mod_op.is_none()
                            && !self.camera_anim_active
                        {
                            let is_double = response.double_clicked();
                            // Shift / Ctrl (⌘ on macOS) extend the selection: each modified
                            // click adds the picked face/edge/point to the set, or removes it
                            // if already selected, instead of replacing the whole selection.
                            let multi_select =
                                ctx.input(|i| i.modifiers.shift || i.modifiers.ctrl || i.modifiers.command);
                            if let Some(click_pos) = response.interact_pointer_pos() {
                                // Local projection that captures only Copy values (not
                                // `self`), so it doesn't extend a borrow across the
                                // mutations elsewhere in this scope.
                                let is_persp = self.is_perspective;
                                let proj = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
                                    let rx = cos_y * x - sin_y * z;
                                    let rz = sin_y * x + cos_y * z;
                                    let ry = cos_p * y - sin_p * rz;
                                    let final_z = sin_p * y + cos_p * rz;
                                    if is_persp {
                                        let dist = 1200.0;
                                        let factor = dist / (dist - final_z.min(dist * 0.85));
                                        (center_x + rx * view_scale * factor, center_y - ry * view_scale * factor, final_z)
                                    } else {
                                        (center_x + rx * view_scale, center_y - ry * view_scale, final_z)
                                    }
                                };

                                // Sketches take priority over bodies — a sketch drawn
                                // on a face sits visually on top, so clicking it should
                                // select the sketch element, not the body face behind it.
                                // We try sketch picking first and fall through to body
                                // picking only when no sketch element is under the cursor.

                                let mut best: Option<(String, usize, f32)> = None; // (sketch, region, depth)
                                let mut best_edge: Option<(String, usize, f32)> = None; // (sketch, edge, px dist)
                                const EDGE_TOL_PX: f32 = 6.0;
                                let var_map = self.graph.variable_map();
                                for idx in self.graph.graph.node_indices() {
                                    let node = &self.graph.graph[idx];
                                    if self.hidden_nodes.contains(&node.id) {
                                        continue; // can't pick a hidden sketch
                                    }
                                    if let FeatureType::Sketch { cs, curves, shapes, corner_mods, .. } = &node.feature {
                                        let cs = *cs;
                                        // Pick against the variable-resolved geometry.
                                        let eff = zerocad_core::effective_curves(curves, shapes, corner_mods, &var_map);
                                        let curves = &eff;
                                        let to_scr = |u: f32, v: f32| -> egui::Pos2 {
                                            let w = cs.unproject(u, v);
                                            let pr = proj(w.x, w.y, w.z);
                                            egui::pos2(pr.0, pr.1)
                                        };
                                        // Project a sketch loop to screen coordinates.
                                        let project_loop = |loop_pts: &[(f32, f32)]| -> Vec<(f32, f32)> {
                                            loop_pts
                                                .iter()
                                                .map(|&(u, v)| {
                                                    let s = to_scr(u, v);
                                                    (s.x, s.y)
                                                })
                                                .collect()
                                        };

                                        // Edge candidates: drawn segments, then circles.
                                        let seg_count = curves.segments.len();
                                        for (i, s) in curves.segments.iter().enumerate() {
                                            let d = dist_point_to_segment(
                                                click_pos,
                                                to_scr(s.a.0, s.a.1),
                                                to_scr(s.b.0, s.b.1),
                                            );
                                            if d < EDGE_TOL_PX
                                                && best_edge.as_ref().map_or(true, |b| d < b.2)
                                            {
                                                best_edge = Some((node.id.clone(), i, d));
                                            }
                                        }
                                        for (j, c) in curves.circles.iter().enumerate() {
                                            let mut prev: Option<egui::Pos2> = None;
                                            let mut mind = f32::INFINITY;
                                            for k in 0..=48 {
                                                let th = (k as f32 / 48.0) * std::f32::consts::TAU;
                                                let p = to_scr(
                                                    c.center.0 + c.radius * th.cos(),
                                                    c.center.1 + c.radius * th.sin(),
                                                );
                                                if let Some(pp) = prev {
                                                    mind = mind.min(dist_point_to_segment(click_pos, pp, p));
                                                }
                                                prev = Some(p);
                                            }
                                            if mind < EDGE_TOL_PX
                                                && best_edge.as_ref().map_or(true, |b| mind < b.2)
                                            {
                                                best_edge = Some((node.id.clone(), seg_count + j, mind));
                                            }
                                        }

                                        for (ri, region) in detect_regions(curves).iter().enumerate() {
                                            let screen = project_loop(&region.boundary);
                                            if screen.len() < 3 {
                                                continue;
                                            }
                                            let click = (click_pos.x, click_pos.y);
                                            // Inside the outer boundary but not in a hole.
                                            let in_outer =
                                                zerocad_core::sketch::point_in_polygon(click, &screen);
                                            let in_hole = region.holes.iter().any(|h| {
                                                let hs = project_loop(h);
                                                hs.len() >= 3
                                                    && zerocad_core::sketch::point_in_polygon(click, &hs)
                                            });
                                            if in_outer && !in_hole {
                                                // Average projected depth of the boundary,
                                                // for nearest-face selection.
                                                let depth = region
                                                    .boundary
                                                    .iter()
                                                    .map(|&(u, v)| {
                                                        let w = cs.unproject(u, v);
                                                        proj(w.x, w.y, w.z).2
                                                    })
                                                    .sum::<f32>()
                                                    / region.boundary.len().max(1) as f32;
                                                if best.as_ref().map_or(true, |b| depth > b.2) {
                                                    best = Some((node.id.clone(), ri, depth));
                                                }
                                            }
                                        }
                                    }
                                }

                                // Did we hit any sketch element?
                                let hit_sketch = best
                                    .as_ref()
                                    .map(|b| b.0.clone())
                                    .or_else(|| best_edge.as_ref().map(|b| b.0.clone()));

                                if hit_sketch.is_some() {
                                    // A sketch is under the cursor — select it, clearing
                                    // any body selection.
                                    self.selected_body.clear();

                                    // A PLAIN click selects exactly the one element under
                                    // the cursor, replacing any prior sketch selection — so
                                    // "click a face, click Extrude" pulls only that face,
                                    // never the whole sketch. Shift/Ctrl EXTENDS the
                                    // selection (toggling the clicked element), which is the
                                    // multi-face extrude workflow. Edges take priority over
                                    // faces. (A double-click no longer selects every region —
                                    // that silently turned a one-face extrude into a whole-
                                    // sketch one; the sketch property panel's "Extrude whole
                                    // Sketch" button is the explicit way to get all regions.)
                                    if !multi_select {
                                        self.selected_faces.clear();
                                        self.selected_edges.clear();
                                    }
                                    if let Some((sid, ei, _)) = best_edge {
                                        let key = (sid, ei);
                                        if multi_select && !self.selected_edges.insert(key.clone()) {
                                            self.selected_edges.remove(&key);
                                        } else {
                                            self.selected_edges.insert(key.clone());
                                        }
                                        self.status_msg = format!(
                                            "Edge {} of {} selected. Edges: {}.",
                                            key.1,
                                            key.0,
                                            self.selected_edges.len(),
                                        );
                                    } else if let Some((sid, ri, _)) = best {
                                        let key = (sid, ri);
                                        if multi_select && !self.selected_faces.insert(key.clone()) {
                                            self.selected_faces.remove(&key);
                                        } else {
                                            self.selected_faces.insert(key.clone());
                                        }
                                        self.status_msg = format!(
                                            "Face {} of {} selected. Faces: {} (click Extrude to build).",
                                            key.1,
                                            key.0,
                                            self.selected_faces.len(),
                                        );
                                    }
                                } else {
                                    // No sketch hit — try body picking instead.
                                    let body_hit =
                                        self.pick_body_element(click_pos, &proj, sin_p, cos_p, sin_y, cos_y);
                                    if let Some((node, pick)) = body_hit {
                                        // Sketch-region/edge selections are a separate
                                        // concept; a body pick always supersedes them.
                                        self.selected_faces.clear();
                                        self.selected_edges.clear();
                                        if is_double {
                                            // Double-click always selects the whole body.
                                            self.selected_body.clear();
                                            self.selected_body.insert((node.clone(), BodyPick::Whole));
                                            self.status_msg = format!("Selected whole body {}.", node);
                                        } else if multi_select {
                                            // Add to the selection, or remove it if already
                                            // selected, so multiple faces/edges/points can be
                                            // picked together (e.g. to fillet several edges).
                                            let key = (node.clone(), pick);
                                            if !self.selected_body.insert(key.clone()) {
                                                self.selected_body.remove(&key);
                                            }
                                            self.status_msg =
                                                format!("{} element(s) selected.", self.selected_body.len());
                                        } else {
                                            // Plain click replaces the selection.
                                            self.selected_body.clear();
                                            self.selected_body.insert((node.clone(), pick));
                                            self.status_msg = match pick {
                                                BodyPick::Whole => format!("Selected whole body {}.", node),
                                                BodyPick::Face(f) => {
                                                    format!("Selected face {} of {} (Draw Sketch to sketch on it).", f, node)
                                                }
                                                BodyPick::Edge(e) => format!("Selected edge {} of {}.", e, node),
                                                BodyPick::Vertex(v) => format!("Selected point {} of {}.", v, node),
                                            };
                                        }
                                    } else if !multi_select {
                                        // Nothing hit and no modifier held — clear everything
                                        // (body AND sketch face/edge selections), so an empty
                                        // click is a reliable "deselect all". With a modifier
                                        // down, keep the in-progress multi-selection intact.
                                        self.selected_body.clear();
                                        self.selected_faces.clear();
                                        self.selected_edges.clear();
                                    }
                                } // end: sketch-first picking
                            }
                        }

                        // Compute cursor snap preview coordinates
                        let current_cursor_snap = if let Some(pos) = hover_pos {
                            let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                            let raw = self.screen_to_sketch(pos, rect, &self.active_sketch_cs);
                            Some(self.snap_sketch_point(raw, scale, shift))
                        } else {
                            None
                        };

                        // Track cursor and refresh live dimension fields.
                        self.last_cursor = current_cursor_snap;
                        if let (Some(start), Some(cursor)) =
                            (self.sketch_temp_start, current_cursor_snap)
                        {
                            self.update_dim_live(start, cursor);

                            // Compute Fusion 360-style screen positions for inline dim inputs.
                            let s_scr = egui::pos2(
                                center_x + start.0 * view_scale,
                                center_y - start.1 * view_scale,
                            );
                            let c_scr = egui::pos2(
                                center_x + cursor.0 * view_scale,
                                center_y - cursor.1 * view_scale,
                            );
                            self.dim_screen_positions = match self.active_tool.unwrap_or(SketchTool::Line) {
                                SketchTool::Rectangle | SketchTool::RectangleCenter => {
                                    // Width: midpoint of the edge furthest from shape center (bottom or top)
                                    let outer_y = if c_scr.y > s_scr.y { c_scr.y } else { s_scr.y };
                                    let mid_w = egui::pos2((s_scr.x + c_scr.x) / 2.0, outer_y + 22.0);
                                    // Height: midpoint of the edge furthest from shape center (right or left)
                                    let outer_x = if c_scr.x > s_scr.x { c_scr.x } else { s_scr.x };
                                    let mid_h = egui::pos2(outer_x + 22.0, (s_scr.y + c_scr.y) / 2.0);
                                    vec![mid_w, mid_h]
                                }
                                SketchTool::Circle => {
                                    let dx = cursor.0 - start.0;
                                    let dy = cursor.1 - start.1;
                                    let r = (dx * dx + dy * dy).sqrt();
                                    let r_scr = egui::pos2(
                                        center_x + (start.0 + r) * view_scale,
                                        center_y - start.1 * view_scale,
                                    );
                                    vec![egui::pos2(r_scr.x + 18.0, r_scr.y)]
                                }
                                SketchTool::Line => {
                                    // Length: midpoint of line, offset perpendicular (upward in screen)
                                    let mid = egui::pos2(
                                        (s_scr.x + c_scr.x) / 2.0,
                                        (s_scr.y + c_scr.y) / 2.0 - 22.0,
                                    );
                                    // Angle: near start point
                                    let ang_pos = egui::pos2(s_scr.x + 40.0, s_scr.y + 22.0);
                                    vec![mid, ang_pos]
                                }
                                // 3-point tools have no inline dimensions.
                                _ => Vec::new(),
                            };
                        } else {
                            self.dim_screen_positions.clear();
                        }

                        // Draw the 3D projected CAD viewport
                        let painter = ui.painter_at(rect);
                        self.draw_viewport(painter.clone(), rect, hover_pos, current_cursor_snap);
                        painter
                    });
                });

        // Dimension dialog overlay (drawn after the viewport, on top).
        self.show_dimension_dialog(ctx);

        // Inline extrude distance box overlay (Fusion-style, mirrors the sketch
        // dimension dialog). Drawn on top of the viewport while extruding.
        self.show_extrude_dialog(ctx);

        // 3D fillet/chamfer: the drag manipulator on the edge, the inline size
        // box, and the inline 2D corner-radius box (anchored on the staged
        // corner/cursor). The handle is drawn first so the box layers over it.
        self.drag_edge_mod_handle(ctx);
        self.show_edge_mod_dialog(ctx);
        self.drag_corner_radius_handle(ctx);
        self.show_corner_radius_box(ctx);
    }
}
