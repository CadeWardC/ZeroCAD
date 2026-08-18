use crate::app::editing::snap_line_angle;
use crate::*;

#[derive(Clone, Copy, PartialEq, Eq)]
enum ViewCubeFace {
    Top,
    Bottom,
    Front,
    Back,
    Right,
    Left,
}

impl ZeroCadApp {
    pub(crate) fn draw_workspace_viewport(&mut self, ctx: &egui::Context) {
        // CENTRAL PANEL: 3D CAD Viewport
        let mut viewport_rect = None;
        egui::CentralPanel::default()
            .frame(
                egui::Frame::none()
                    .fill(self.pal().surface_subtle)
                    .inner_margin(egui::Margin::same(12.0)),
            )
            .show(ctx, |ui| {
                    // Draw a nice border and frame around the viewport
                    egui::Frame::canvas(ui.style()).show(ui, |ui| {
                        let (rect, response) = ui.allocate_exact_size(
                            ui.available_size() - egui::vec2(0.0, 4.0),
                            egui::Sense::click() | egui::Sense::drag(),
                        );
                        viewport_rect = Some(rect);

                        let context_body = self.selected_whole_body();
                        response.context_menu(|ui| {
                            if let Some(source) = context_body.clone() {
                                if ui.button("Move").clicked() {
                                    self.begin_move_body(source);
                                    ui.close_menu();
                                }
                                if ui.button("Copy").clicked() {
                                    self.copy_selected_body();
                                    ui.close_menu();
                                }
                            } else {
                                ui.label("Fully select a body for body commands.");
                            }
                            ui.separator();
                            if ui
                                .add_enabled(
                                    self.body_clipboard.is_some(),
                                    egui::Button::new("Paste"),
                                )
                                .clicked()
                            {
                                self.paste_copied_body();
                                ui.close_menu();
                            }
                        });

                        let center_x = rect.center().x + self.camera_pan.x;
                        let center_y = rect.center().y + self.camera_pan.y;
                        let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);

                        let cos_p = self.camera_pitch.cos();
                        let sin_p = self.camera_pitch.sin();
                        let cos_y = self.camera_yaw.cos();
                        let sin_y = self.camera_yaw.sin();

                        // 3D coordinate projection mapping function. Captures
                        // `is_perspective` by value so the closure borrows nothing
                        // from `self` — the GPU pick helper needs `&mut self`
                        // while this closure is alive (same pattern as render.rs).
                        let is_perspective = self.is_perspective;
                        let project_3d = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
                            let rx = cos_y * x - sin_y * z;
                            let rz = sin_y * x + cos_y * z;
                            let ry = cos_p * y - sin_p * rz;
                            let final_z = sin_p * y + cos_p * rz;

                            if is_perspective {
                                let dist = crate::render::PERSP_DIST;
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

                        // Datum plane sheets, projected for hover/click hit-tests (the
                        // same world corners the renderer draws).
                        let datum_quads: Vec<(String, CoordinateSystem, [egui::Pos2; 4])> = self
                            .resolved_datum_planes()
                            .into_iter()
                            .map(|(id, _name, cs)| {
                                let w = Self::datum_plane_world_corners(&cs);
                                let p = |c: [f32; 3]| {
                                    let pr = project_3d(c[0], c[1], c[2]);
                                    egui::pos2(pr.0, pr.1)
                                };
                                (id, cs, [p(w[0]), p(w[1]), p(w[2]), p(w[3])])
                            })
                            .collect();

                        // Perform frame-perfect hover checking immediately. Inline
                        // sketch dimensions are keyboard controls and deliberately
                        // mouse-transparent while a shape is being placed. Read the
                        // physical pointer inside the viewport in that state instead
                        // of `Response::hover_pos`: egui otherwise reports `None` as
                        // the pointer crosses the foreground TextEdit, making the
                        // rubber-band sketch disappear/flicker for a frame.
                        let hover_pos = if self.is_sketch_mode && self.dim_input.is_some() {
                            ctx.input(|i| i.pointer.hover_pos())
                                .filter(|pos| rect.contains(*pos))
                        } else {
                            response.hover_pos()
                        };
                        self.hovered_plane = None;
                        self.hovered_datum_plane = None;
                        self.hovered_sketch_face = None;
                        self.hovered_body_element = None;
                        self.hovered_sketch_element = None;
                        self.hovered_active_sketch_element = None;
                        self.hovered_active_sketch_region = None;
                        let plane_pick_active =
                            self.is_plane_selection_mode || self.mirror_plane_pick_active();
                        if plane_pick_active {
                            if let Some(pos) = hover_pos {
                                // A planar body face under the cursor takes priority over
                                // the origin plane quads — sketch directly on the solid.
                                // The GPU pick buffer answers exactly when available;
                                // otherwise the CPU triangle scan runs as before.
                                let gpu_face =
                                    self.gpu_pick_face(pos, rect, ctx.pixels_per_point());
                                let face_hit = self
                                    .pick_body_element(
                                        pos, &project_3d, sin_p, cos_p, sin_y, cos_y, gpu_face,
                                    )
                                    .and_then(|(node, pick)| match pick {
                                        BodyPick::Face(fid) if self.face_is_planar(&node, fid) => {
                                            Some((node, fid))
                                        }
                                        _ => None,
                                    });
                                let datum_hit = datum_quads
                                    .iter()
                                    .find(|(_, _, pts)| is_point_in_quad(pos, pts))
                                    .map(|(id, _, _)| id.clone());
                                if let Some((node, fid)) = face_hit {
                                    self.hovered_sketch_face = Some((node, fid));
                                } else if let Some(id) = datum_hit {
                                    self.hovered_datum_plane = Some(id);
                                } else if is_point_in_quad(pos, &xy_pts) {
                                    self.hovered_plane = Some(SketchPlane::XY);
                                } else if is_point_in_quad(pos, &xz_pts) {
                                    self.hovered_plane = Some(SketchPlane::XZ);
                                } else if is_point_in_quad(pos, &yz_pts) {
                                    self.hovered_plane = Some(SketchPlane::YZ);
                                }
                            }

                            if self.hovered_plane.is_some()
                                || self.hovered_datum_plane.is_some()
                                || self.hovered_sketch_face.is_some()
                            {
                                egui::show_tooltip_at_pointer(ctx, ui.layer_id(), egui::Id::new("plane_select_tooltip"), |ui| {
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
                        //   • Shift (line tool) → snap angle to 15° increments
                        //   • Ctrl (sketch) → suppress all snapping while drawing
                        let pointer_delta = ctx.input(|i| i.pointer.delta());
                        let shift = ctx.input(|i| i.modifiers.shift);
                        let ctrl = ctx.input(|i| i.modifiers.ctrl);

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
                                // here). Sketch modifiers are reserved for drawing, so
                                // Shift must NOT pan here.
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
                                    (self.camera_pitch, self.camera_yaw) = orbit_camera_angles(
                                        self.camera_pitch,
                                        self.camera_yaw,
                                        pointer_delta,
                                    );
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
                                    (self.camera_pitch, self.camera_yaw) = orbit_camera_angles(
                                        self.camera_pitch,
                                        self.camera_yaw,
                                        pointer_delta,
                                    );
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
                        self.camera_zoom = viewport_zoom_after_scroll(
                            self.camera_zoom,
                            scroll_delta,
                            response.hovered(),
                        );

                        // Standalone 3D Mirror plane/face selection. This shares
                        // the sketch plane sheets and planar-face hit testing but
                        // does not enter sketch mode or move the camera.
                        if self.mirror_plane_pick_active() && response.clicked() {
                            let choice = if let Some((node, fid)) =
                                self.hovered_sketch_face.clone()
                            {
                                self.face_ref(&node, fid).map(|face| {
                                    crate::pattern_ui::MirrorPlaneChoice::Face(
                                        face,
                                        format!("Face {fid} of {node}"),
                                    )
                                })
                            } else if let Some(datum_id) = self.hovered_datum_plane.clone() {
                                let name = self
                                    .resolved_datum_planes()
                                    .into_iter()
                                    .find(|(id, _, _)| *id == datum_id)
                                    .map(|(_, name, _)| name)
                                    .unwrap_or_else(|| datum_id.clone());
                                Some(crate::pattern_ui::MirrorPlaneChoice::Datum(
                                    datum_id, name,
                                ))
                            } else {
                                self.hovered_plane.map(|plane| match plane {
                                    SketchPlane::XY => crate::pattern_ui::MirrorPlaneChoice::XY,
                                    SketchPlane::XZ => crate::pattern_ui::MirrorPlaneChoice::XZ,
                                    SketchPlane::YZ => crate::pattern_ui::MirrorPlaneChoice::YZ,
                                })
                            };
                            if let Some(choice) = choice {
                                if let Some(op) = self.pattern_op.as_mut() {
                                    op.plane = Some(choice);
                                    op.pick_plane = false;
                                }
                                self.status_msg =
                                    "Mirror plane selected. Click OK to create the mirrored body."
                                        .to_string();
                            }
                        // Sketch plane selection click interaction
                        } else if self.is_plane_selection_mode && response.clicked() {
                            if let Some((node, fid)) = self.hovered_sketch_face.clone() {
                                // Sketch directly on the clicked planar body face — the
                                // same path as pre-selecting a face and pressing Draw
                                // Sketch (top_bar_commands): frame from `face_cs`, durable
                                // `face_ref` so the sketch follows the face, then begin.
                                if let Some(cs) = self.face_cs(&node, fid) {
                                    log::info!("Sketching on clicked body face {fid} of {node}.");
                                    let fref = self.face_ref(&node, fid);
                                    let boundary = self.face_boundary_curves(&node, fid, &cs);
                                    let now = ctx.input(|i| i.time);
                                    self.active_sketch_on_face = true;
                                    self.active_sketch_face_ref = fref;
                                    self.active_sketch_datum_ref = None;
                                    self.hovered_sketch_face = None;
                                    self.begin_sketch_on(cs, now);
                                    // After begin (it resets sketch state): the face
                                    // outline joins the sketch as reference geometry.
                                    self.active_face_boundary = boundary;
                                    self.recompute_sketch_regions();
                                    self.status_msg = "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
                                }
                            } else if let Some(datum_id) = self.hovered_datum_plane.clone() {
                                if let Some((_, cs, _)) =
                                    datum_quads.iter().find(|(id, _, _)| *id == datum_id)
                                {
                                    log::info!("Sketching on datum plane {datum_id}.");
                                    let now = ctx.input(|i| i.time);
                                    self.active_sketch_on_face = false;
                                    self.active_sketch_face_ref = None;
                                    self.active_sketch_datum_ref = Some(datum_id);
                                    self.hovered_datum_plane = None;
                                    self.begin_sketch_on(*cs, now);
                                    self.status_msg = "Sketching on the datum plane. Draw a profile, then Finish Sketch.".to_string();
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
                                self.active_sketch_face_ref = None;
                                self.active_sketch_datum_ref = None;
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
                            && self.active_tool == Some(SketchTool::Offset)
                            && response.clicked()
                            && !self.camera_anim_active
                        {
                            if let Some(position) = response.interact_pointer_pos() {
                                let scale =
                                    rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                                let raw =
                                    self.screen_to_sketch(position, rect, &self.active_sketch_cs);
                                let seed = self.snap_sketch_point(raw, scale, ctrl);
                                self.handle_offset_click(seed, 9.0 / scale.max(1.0e-4), shift);
                            }
                        }
                        if self.is_sketch_mode
                            && self.active_tool == Some(SketchTool::Trim)
                            && response.clicked()
                            && !self.camera_anim_active
                        {
                            if let Some(position) = response.interact_pointer_pos() {
                                let scale =
                                    rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                                let raw =
                                    self.screen_to_sketch(position, rect, &self.active_sketch_cs);
                                self.update_trim_preview(raw, 9.0 / scale.max(1.0e-4));
                                self.commit_trim_preview();
                            }
                        }
                        if self.is_sketch_mode
                            && self.active_tool == Some(SketchTool::Dimension)
                            && self.sketch_dimension_editor.is_none()
                            && response.clicked()
                            && !self.camera_anim_active
                        {
                            if let Some(position) = response.interact_pointer_pos() {
                                const DIMENSION_PICK_PX: f32 = 9.0;
                                let cs = self.active_sketch_cs;
                                let is_persp = self.is_perspective;
                                let to_screen = move |point: (f64, f64)| -> egui::Pos2 {
                                    let world = cs.unproject(point.0 as f32, point.1 as f32);
                                    let rx = cos_y * world.x - sin_y * world.z;
                                    let rz = sin_y * world.x + cos_y * world.z;
                                    let ry = cos_p * world.y - sin_p * rz;
                                    let final_z = sin_p * world.y + cos_p * rz;
                                    if is_persp {
                                        let dist = crate::render::PERSP_DIST;
                                        let factor =
                                            dist / (dist - final_z.min(dist * 0.85));
                                        egui::pos2(
                                            center_x + rx * view_scale * factor,
                                            center_y - ry * view_scale * factor,
                                        )
                                    } else {
                                        egui::pos2(
                                            center_x + rx * view_scale,
                                            center_y - ry * view_scale,
                                        )
                                    }
                                };
                                let hit = self.sketch_solver_model.as_ref().and_then(|model| {
                                    pick_solver_element(
                                        model,
                                        position,
                                        &to_screen,
                                        DIMENSION_PICK_PX,
                                    )
                                });
                                if let Some(id) = hit {
                                    self.select_for_dimension(id);
                                } else {
                                    let raw =
                                        self.screen_to_sketch(position, rect, &self.active_sketch_cs);
                                    if let Err(message) = self.place_inferred_dimension(
                                        (raw.0 as f64, raw.1 as f64),
                                        position,
                                    ) {
                                        self.status_msg = message;
                                    }
                                }
                            }
                        }
                        if self.is_sketch_mode
                            && self.active_tool.is_some()
                            && self.active_tool != Some(SketchTool::Offset)
                            && self.active_tool != Some(SketchTool::Trim)
                            && self.active_tool != Some(SketchTool::Dimension)
                            && (response.clicked() || begin_draw)
                            && !self.camera_anim_active
                        {
                            if let Some(hover_pos) = response.interact_pointer_pos() {
                                let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                                // Map the click onto the active sketch plane via a ray /
                                // plane intersection — WYSIWYG on any plane orientation.
                                let raw = self.screen_to_sketch(hover_pos, rect, &self.active_sketch_cs);
                                let snapped = self.snap_sketch_point(raw, scale, ctrl);

                                let tool = self.active_tool.unwrap();
                                let pt = if shift && !ctrl && tool == SketchTool::Line {
                                    self.sketch_points
                                        .first()
                                        .copied()
                                        .map_or(snapped, |start| snap_line_angle(start, snapped))
                                } else {
                                    snapped
                                };

                                if tool.is_spline() {
                                    if response.clicked() {
                                        let distinct = self.sketch_points.last().map_or(true, |last| {
                                            (last.0 - pt.0).hypot(last.1 - pt.1) > 1.0e-5
                                        });
                                        if distinct {
                                            self.sketch_points.push(pt);
                                            self.sketch_temp_start = self.sketch_points.first().copied();
                                        }
                                        if response.double_clicked() && self.sketch_points.len() >= 2 {
                                            self.finish_in_progress_spline();
                                        } else {
                                            self.status_msg = format!(
                                                "Spline point {} set — keep clicking; double-click or press Enter to finish.",
                                                self.sketch_points.len()
                                            );
                                        }
                                    }
                                } else if let Some(kind) = tool.corner_kind() {
                                    // Fillet/Chamfer: a click STAGES the nearest corner
                                    // (live preview); it isn't committed until Enter / OK.
                                    // The user can stack several corners and tune R first.
                                    if response.clicked() {
                                        self.stage_corner_at(pt, kind);
                                    }
                                } else {
                                    let point_count = tool.point_count();
                                    if self.sketch_points.is_empty() {
                                        // First point (click or press-drag). Dimensioned 2-point
                                        // tools open the inline dimension dialog here; the
                                        // point-drawn tools (rotated rect, 3-point circle,
                                        // ellipses, polygon, mirror) draw by clicking each point
                                        // with a live preview and no dialog.
                                        self.sketch_points.push(pt);
                                        self.sketch_temp_start = Some(pt);
                                        if point_count == 2 && !tool.is_point_drawn() {
                                            self.dim_anchor = Some(hover_pos);
                                            self.dim_input = Some(DimInput {
                                                fields: dim_fields_for(tool),
                                                focus_request: None,
                                                active_field: 0,
                                                editing_field: None,
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
                                            if tool == SketchTool::Mirror {
                                                // Mirror doesn't create a shape — its two clicks
                                                // are the reflection axis.
                                                let p0 = self.sketch_points[0];
                                                self.commit_sketch_mirror(p0, pt);
                                                self.cancel_in_progress_shape();
                                            } else {
                                                self.finalize_shape(pt);
                                            }
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

                        // Drag-to-solve: in an Edit Sketch session (solver model
                        // present) with no drawing tool armed, dragging a solver
                        // point moves it and re-solves the constraints live —
                        // an under-constrained sketch follows the cursor
                        // minimally, a dimensioned one snaps back onto its
                        // constraint set. The 2D solve is microseconds, so it
                        // runs every drag frame; the expensive downstream
                        // rebuild happens once, at Finish Sketch.
                        if self.is_sketch_mode
                            && self.active_tool.is_none()
                            && self.sketch_solver_model.is_none()
                            && (response.clicked()
                                || response.drag_started_by(egui::PointerButton::Primary))
                        {
                            self.ensure_active_solver_model();
                            self.rebuild_active_sketch_curves();
                        }

                        let mut active_sketch_hit = false;

                        if self.is_sketch_mode
                            && self.active_tool.is_none()
                            && self.sketch_solver_model.is_some()
                            && !self.camera_anim_active
                        {
                            const POINT_GRAB_PX: f32 = 9.0;
                            let cs = self.active_sketch_cs;
                            // Local projection capturing only Copy values (same
                            // pattern as the pick path below) so no `self`
                            // borrow lives across the mutations here.
                            let is_persp = self.is_perspective;
                            let to_screen = move |p: (f64, f64)| -> egui::Pos2 {
                                let w = cs.unproject(p.0 as f32, p.1 as f32);
                                let (x, y, z) = (w.x, w.y, w.z);
                                let rx = cos_y * x - sin_y * z;
                                let rz = sin_y * x + cos_y * z;
                                let ry = cos_p * y - sin_p * rz;
                                let final_z = sin_p * y + cos_p * rz;
                                if is_persp {
                                    let dist = crate::render::PERSP_DIST;
                                    let factor = dist / (dist - final_z.min(dist * 0.85));
                                    egui::pos2(
                                        center_x + rx * view_scale * factor,
                                        center_y - ry * view_scale * factor,
                                    )
                                } else {
                                    egui::pos2(center_x + rx * view_scale, center_y - ry * view_scale)
                                }
                            };
                            if !self.orbiting && self.sketch_drag_point.is_none() {
                                if let Some(pos) = hover_pos {
                                    self.hovered_active_sketch_element = self
                                        .sketch_solver_model
                                        .as_ref()
                                        .and_then(|model| {
                                            pick_solver_element(
                                                model,
                                                pos,
                                                &to_screen,
                                                POINT_GRAB_PX,
                                            )
                                        });
                                    if self.hovered_active_sketch_element.is_none() {
                                        let local = self.screen_to_sketch(
                                            pos,
                                            rect,
                                            &self.active_sketch_cs,
                                        );
                                        self.hovered_active_sketch_region = self
                                            .detected_regions
                                            .iter()
                                            .enumerate()
                                            .filter(|(_, region)| region.contains(local))
                                            .min_by(|(_, first), (_, second)| {
                                                first
                                                    .area
                                                    .partial_cmp(&second.area)
                                                    .unwrap_or(std::cmp::Ordering::Equal)
                                            })
                                            .map(|(index, _)| index);
                                    }
                                }
                            }
                            if response.drag_started_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    self.sketch_drag_point = self
                                        .sketch_solver_model
                                        .as_ref()
                                        .and_then(|model| {
                                            model
                                                .points
                                                .iter()
                                                .filter(|point| {
                                                    !model.is_projected_point(point.id)
                                                })
                                                .map(|p| (p.id, to_screen(p.pos).distance(pos)))
                                                .filter(|(_, d)| *d <= POINT_GRAB_PX)
                                                .min_by(|a, b| {
                                                    a.1.partial_cmp(&b.1)
                                                        .unwrap_or(std::cmp::Ordering::Equal)
                                                })
                                                .map(|(id, _)| id)
                                        });
                                }
                            }
                            if let Some(drag_id) = self.sketch_drag_point {
                                if response.dragged_by(egui::PointerButton::Primary) {
                                    if let Some(pos) = response.interact_pointer_pos() {
                                        let (u, v) = self.screen_to_sketch(pos, rect, &cs);
                                        if let Some(model) = &mut self.sketch_solver_model {
                                            if let Some(p) =
                                                model.points.iter_mut().find(|p| p.id == drag_id)
                                            {
                                                p.pos = (u as f64, v as f64);
                                            }
                                        }
                                        self.solve_live_sketch();
                                    }
                                }
                                if response.drag_stopped_by(egui::PointerButton::Primary) {
                                    self.sketch_drag_point = None;
                                    self.solve_live_sketch();
                                    self.status_msg =
                                        "Point moved — Finish Sketch commits the edit.".to_string();
                                }
                            }
                            // Click-selection of solver points/entities: drives
                            // which constraint-palette buttons are applicable.
                            // Plain click replaces the selection; Shift extends.
                            if response.clicked() && self.sketch_drag_point.is_none() {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    let entity_hit =
                                        self.sketch_solver_model.as_ref().and_then(|model| {
                                            pick_solver_element(
                                                model,
                                                pos,
                                                &to_screen,
                                                POINT_GRAB_PX,
                                            )
                                        });
                                    let region_hit = entity_hit
                                        .is_none()
                                        .then(|| {
                                            let local = self.screen_to_sketch(
                                                pos,
                                                rect,
                                                &self.active_sketch_cs,
                                            );
                                            self.detected_regions
                                                .iter()
                                                .enumerate()
                                                .filter(|(_, region)| region.contains(local))
                                                .min_by(|(_, a), (_, b)| {
                                                    a.area
                                                        .partial_cmp(&b.area)
                                                        .unwrap_or(std::cmp::Ordering::Equal)
                                                })
                                                .map(|(index, _)| index)
                                        })
                                        .flatten();
                                    active_sketch_hit =
                                        entity_hit.is_some() || region_hit.is_some();
                                    match (entity_hit, region_hit) {
                                        (Some(id), _) => {
                                            self.selected_region_indices.clear();
                                            if !shift {
                                                self.sketch_selected_ids.clear();
                                            }
                                            if let Some(k) = self
                                                .sketch_selected_ids
                                                .iter()
                                                .position(|&s| s == id)
                                            {
                                                self.sketch_selected_ids.remove(k);
                                            } else {
                                                self.sketch_selected_ids.push(id);
                                            }
                                        }
                                        (None, Some(index)) => {
                                            self.sketch_selected_ids.clear();
                                            if !shift {
                                                self.selected_region_indices.clear();
                                            }
                                            if !self.selected_region_indices.insert(index) {
                                                self.selected_region_indices.remove(&index);
                                            }
                                        }
                                        (None, None) if !shift => {
                                            self.sketch_selected_ids.clear();
                                            self.selected_region_indices.clear();
                                        }
                                        (None, None) => {}
                                    }
                                }
                            }
                        }

                        // Face picks requested by the Move dialog are consumed by
                        // that operation instead of replacing the normal selection.
                        if response.clicked()
                            && self
                                .move_op
                                .as_ref()
                                .is_some_and(|op| op.picking.is_some())
                        {
                            if let Some(click_pos) = response.interact_pointer_pos() {
                                let gpu_face = self.gpu_pick_face(
                                    click_pos,
                                    rect,
                                    ctx.pixels_per_point(),
                                );
                                if let Some((node, BodyPick::Face(face))) = self.pick_body_element(
                                    click_pos,
                                    &project_3d,
                                    sin_p,
                                    cos_p,
                                    sin_y,
                                    cos_y,
                                    gpu_face,
                                ) {
                                    self.pick_move_face(node, face);
                                }
                            }
                        }

                        // Planar-face picks requested by Split Body use the
                        // same exact body-face picker but do not disturb normal
                        // viewport selection.
                        if response.clicked()
                            && self
                                .split_body_op
                                .as_ref()
                                .is_some_and(|op| op.picking_face)
                        {
                            if let Some(click_pos) = response.interact_pointer_pos() {
                                let gpu_face = self.gpu_pick_face(
                                    click_pos,
                                    rect,
                                    ctx.pixels_per_point(),
                                );
                                if let Some((node, BodyPick::Face(face))) = self.pick_body_element(
                                    click_pos,
                                    &project_3d,
                                    sin_p,
                                    cos_p,
                                    sin_y,
                                    cos_y,
                                    gpu_face,
                                ) {
                                    self.pick_split_face(node, face);
                                }
                            }
                        }

                        // Ordinary Model-mode preselection. The same sketch
                        // picker and body priority used by a click drive this
                        // subtle hover, so the preview never advertises a
                        // different target than the next click will select.
                        let preselection_allowed = !self.is_sketch_mode
                            && !plane_pick_active
                            && self.extrude_op.is_none()
                            && self.edge_mod_op.is_none()
                            && self.move_op.is_none()
                            && self.combine_op.is_none()
                            && self.split_body_op.is_none()
                            && self.scale_body_op.is_none()
                            && !self.camera_anim_active
                            && !self.orbiting;
                        if preselection_allowed {
                            if let Some(position) = hover_pos {
                                self.hovered_sketch_element =
                                    self.pick_finished_sketch_element(position, &project_3d);
                                if self.hovered_sketch_element.is_none() {
                                    let gpu_composited =
                                        self.gpu_render && self.gpu_texture_id.is_some();
                                    let detail_hit = self.pick_body_element(
                                        position,
                                        &project_3d,
                                        sin_p,
                                        cos_p,
                                        sin_y,
                                        cos_y,
                                        if gpu_composited {
                                            crate::gpu_viewport::GpuFacePick::Hit(None)
                                        } else {
                                            crate::gpu_viewport::GpuFacePick::Unavailable
                                        },
                                    );
                                    self.hovered_body_element = detail_hit.or_else(|| {
                                        gpu_composited
                                            .then(|| self.gpu.hovered_face())
                                            .flatten()
                                            .map(|(node, face)| (node, BodyPick::Face(face)))
                                    });
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
                            && !plane_pick_active
                            && self.extrude_op.is_none()
                            && self.edge_mod_op.is_none()
                            && self.move_op.is_none()
                            && self.combine_op.is_none()
                            && self.split_body_op.is_none()
                            && self.scale_body_op.is_none()
                            && !self.camera_anim_active
                            && !active_sketch_hit
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
                                        let dist = crate::render::PERSP_DIST;
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
                                let mut best_point: Option<(String, usize, f32)> = None; // (sketch, point, px dist)
                                const EDGE_TOL_PX: f32 = 6.0;
                                const POINT_TOL_PX: f32 = 8.0;
                                let var_map = self.document.variable_map();
                                for idx in self.document.graph.node_indices() {
                                    let node = &self.document.graph[idx];
                                    if self.hidden_nodes.contains(&node.id) {
                                        continue; // can't pick a hidden sketch
                                    }
                                    if let FeatureType::Sketch { cs, curves, shapes, corner_mods, mirrors, solver, .. } = &node.feature {
                                        let cs = *cs;
                                        // Pick against the variable-resolved geometry.
                                        let eff = zerocad_core::effective_curves_solved(curves, shapes, corner_mods, mirrors, solver.as_ref(), &var_map);
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

                                        // Visible sketch points take priority over edges, just
                                        // like solid-body vertices do in the body picker.
                                        for (point_index, point) in
                                            crate::geom2d::selectable_sketch_points(curves)
                                                .into_iter()
                                                .enumerate()
                                        {
                                            let distance = click_pos.distance(to_scr(point.0, point.1));
                                            if distance < POINT_TOL_PX
                                                && best_point
                                                    .as_ref()
                                                    .is_none_or(|best| distance < best.2)
                                            {
                                                best_point =
                                                    Some((node.id.clone(), point_index, distance));
                                            }
                                        }

                                        // Edge candidates: drawn segments, circles, then splines.
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
                                        let arc_offset = seg_count + curves.circles.len();
                                        for (j, arc) in curves.arcs.iter().enumerate() {
                                            let mind = crate::geom2d::sample_arc_points(arc, 48)
                                                .windows(2)
                                                .map(|pair| {
                                                    dist_point_to_segment(
                                                        click_pos,
                                                        to_scr(pair[0].0, pair[0].1),
                                                        to_scr(pair[1].0, pair[1].1),
                                                    )
                                                })
                                                .fold(f32::INFINITY, f32::min);
                                            if mind < EDGE_TOL_PX
                                                && best_edge.as_ref().map_or(true, |b| mind < b.2)
                                            {
                                                best_edge = Some((
                                                    node.id.clone(),
                                                    arc_offset + j,
                                                    mind,
                                                ));
                                            }
                                        }
                                        let spline_offset = arc_offset + curves.arcs.len();
                                        for (j, spline) in curves.splines.iter().enumerate() {
                                            let mind = spline
                                                .sampled_points(0.01)
                                                .windows(2)
                                                .map(|pair| {
                                                    dist_point_to_segment(
                                                        click_pos,
                                                        to_scr(pair[0].0, pair[0].1),
                                                        to_scr(pair[1].0, pair[1].1),
                                                    )
                                                })
                                                .fold(f32::INFINITY, f32::min);
                                            if mind < EDGE_TOL_PX
                                                && best_edge.as_ref().map_or(true, |b| mind < b.2)
                                            {
                                                best_edge = Some((
                                                    node.id.clone(),
                                                    spline_offset + j,
                                                    mind,
                                                ));
                                            }
                                        }

                                        // Regions must include the projected face
                                        // boundary (sketch-on-face) — same merge as
                                        // eval — so a picked region index matches the
                                        // one the extrude will store. Edge picking
                                        // above stays on the drawn curves only.
                                        let mut region_curves = curves.clone();
                                        if let Some(b) = self
                                            .document
                                            .sketch_face_boundaries
                                            .get(node.id.as_str())
                                        {
                                            region_curves.extend_curves(b);
                                        }
                                        for (ri, region) in detect_regions(&region_curves).iter().enumerate() {
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
                                    .or_else(|| best_edge.as_ref().map(|b| b.0.clone()))
                                    .or_else(|| best_point.as_ref().map(|b| b.0.clone()));

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
                                        self.selected_sketch_points.clear();
                                    }
                                    if let Some((sid, point_index, _)) = best_point {
                                        let key = (sid, point_index);
                                        if multi_select
                                            && !self.selected_sketch_points.insert(key.clone())
                                        {
                                            self.selected_sketch_points.remove(&key);
                                        } else {
                                            self.selected_sketch_points.insert(key.clone());
                                        }
                                        self.status_msg = format!(
                                            "Point {} of {} selected. Points: {}.",
                                            key.1,
                                            key.0,
                                            self.selected_sketch_points.len(),
                                        );
                                    } else if let Some((sid, ei, _)) = best_edge {
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
                                        self.status_msg = if self.selected_faces.len() == 1 {
                                            "Click Extrude to build the selected sketch profile."
                                                .to_string()
                                        } else {
                                            "Click Extrude to build the selected sketch profiles."
                                                .to_string()
                                        };
                                        if self.extrude_profile_pick_active {
                                            self.begin_extrude_from_selection();
                                        }
                                    }
                                } else {
                                    // No sketch hit — try body picking instead. The face
                                    // stage uses the GPU pick buffer when available (exact
                                    // to the rendered silhouette); vertices/edges keep
                                    // their CPU proximity search and their priority.
                                    let gpu_face = self.gpu_pick_face(
                                        click_pos,
                                        rect,
                                        ctx.pixels_per_point(),
                                    );
                                    let body_hit = self.pick_body_element(
                                        click_pos, &proj, sin_p, cos_p, sin_y, cos_y, gpu_face,
                                    );
                                    if let Some((node, pick)) = body_hit {
                                        // Sketch-region/edge selections are a separate
                                        // concept; a body pick always supersedes them.
                                        self.selected_faces.clear();
                                        self.selected_edges.clear();
                                        self.selected_sketch_points.clear();
                                        self.select_body_hit(node, pick, is_double, multi_select);
                                    } else if !multi_select {
                                        // Nothing hit and no modifier held — clear everything
                                        // (body AND sketch face/edge selections), so an empty
                                        // click is a reliable "deselect all". With a modifier
                                        // down, keep the in-progress multi-selection intact.
                                        self.selected_body.clear();
                                        self.selected_faces.clear();
                                        self.selected_edges.clear();
                                        self.selected_sketch_points.clear();
                                    }
                                } // end: sketch-first picking
                            }
                        }

                        // Compute cursor snap preview coordinates, capturing what
                        // the cursor snapped onto so the viewport can draw the
                        // matching snap glyph (endpoint ring / midpoint-centre X).
                        let current_cursor_snap = if let Some(pos) = hover_pos {
                            let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                            let raw = self.screen_to_sketch(pos, rect, &self.active_sketch_cs);
                            let res = self.snap_sketch_point_kind(raw, scale, ctrl);
                            let angle_snapping = shift
                                && !ctrl
                                && self.active_tool == Some(SketchTool::Line)
                                && self.sketch_temp_start.is_some();
                            self.cursor_snap_kind = if angle_snapping { None } else { res.kind };
                            self.cursor_snap_guides = if angle_snapping {
                                Vec::new()
                            } else {
                                res.guides_used.clone()
                            };
                            // Ctrl freezes the guides and hides their dashes while
                            // all snapping is temporarily disabled.
                            if !ctrl {
                                let tol = 9.0 / scale.max(1e-4);
                                self.update_snap_guides(&res, raw, tol);
                            }
                            if self.active_tool == Some(SketchTool::Trim) {
                                self.update_trim_preview(raw, 9.0 / scale.max(1.0e-4));
                            }
                            Some(if angle_snapping {
                                snap_line_angle(self.sketch_temp_start.unwrap(), res.pos)
                            } else {
                                res.pos
                            })
                        } else {
                            self.cursor_snap_kind = None;
                            self.snap_guides.clear();
                            self.cursor_snap_guides.clear();
                            self.sketch_trim_preview = None;
                            None
                        };

                        if self.active_tool != Some(SketchTool::Trim) {
                            self.sketch_trim_preview = None;
                        }

                        // Track cursor and refresh live dimension fields.
                        self.last_cursor = current_cursor_snap;
                        if let (Some(start), Some(cursor)) =
                            (self.sketch_temp_start, current_cursor_snap)
                        {
                            self.update_dim_live(start, cursor);

                            // Anchor inputs to the same dimension-resolved geometry
                            // drawn by `draw_viewport`. Using the raw cursor here made
                            // the boxes drift far away as soon as a typed value changed
                            // the preview's true width/height.
                            let cs = self.active_sketch_cs;
                            let to_screen = |p: (f32, f32)| {
                                let w = cs.unproject(p.0, p.1);
                                let projected = project_3d(w.x, w.y, w.z);
                                egui::pos2(projected.0, projected.1)
                            };
                            let shape = self.shape_from_points(cursor);
                            self.dim_screen_positions = match self
                                .active_tool
                                .unwrap_or(SketchTool::Line)
                            {
                                SketchTool::Rectangle | SketchTool::RectangleCenter
                                    if shape.segments.len() >= 4 =>
                                {
                                    let mid = |segment: &LineSegment| {
                                        let a = to_screen(segment.a);
                                        let b = to_screen(segment.b);
                                        a + (b - a) * 0.5
                                    };
                                    let width_a = mid(&shape.segments[0]);
                                    let width_b = mid(&shape.segments[2]);
                                    let height_a = mid(&shape.segments[1]);
                                    let height_b = mid(&shape.segments[3]);
                                    let width_mid = if width_a.y >= width_b.y {
                                        width_a
                                    } else {
                                        width_b
                                    };
                                    let height_mid = if height_a.x >= height_b.x {
                                        height_a
                                    } else {
                                        height_b
                                    };
                                    let center = egui::pos2(
                                        (width_a.x + width_b.x) * 0.5,
                                        (height_a.y + height_b.y) * 0.5,
                                    );
                                    vec![
                                        offset_dimension_box(width_mid, center),
                                        offset_dimension_box(height_mid, center),
                                    ]
                                }
                                SketchTool::Circle if !shape.circles.is_empty() => {
                                    let circle = shape.circles[0];
                                    let center = to_screen(circle.center);
                                    let rim = to_screen((
                                        circle.center.0 + circle.radius,
                                        circle.center.1,
                                    ));
                                    vec![offset_dimension_box(rim, center)]
                                }
                                SketchTool::PolygonInscribed
                                | SketchTool::PolygonCircumscribed
                                    if !shape.segments.is_empty() =>
                                {
                                    let center = to_screen(start);
                                    let dx = cursor.0 - start.0;
                                    let dy = cursor.1 - start.1;
                                    let drag = (dx * dx + dy * dy).sqrt();
                                    let diameter = self
                                        .dim_param(0, 2.0 * drag)
                                        .resolve(&self.document.variable_map());
                                    let (ux, uy) = if drag > 1.0e-4 {
                                        (dx / drag, dy / drag)
                                    } else {
                                        (1.0, 0.0)
                                    };
                                    let rim = to_screen((
                                        start.0 + ux * diameter * 0.5,
                                        start.1 + uy * diameter * 0.5,
                                    ));
                                    vec![offset_dimension_box(rim, center)]
                                }
                                SketchTool::Line if !shape.segments.is_empty() => {
                                    let segment = shape.segments[0];
                                    let a = to_screen(segment.a);
                                    let b = to_screen(segment.b);
                                    let center = a + (b - a) * 0.5;
                                    let axis = b - a;
                                    let mut normal = egui::vec2(axis.y, -axis.x);
                                    if normal.y > 0.0 {
                                        normal = -normal;
                                    }
                                    let length_pos = if normal.length_sq() > 1.0e-4 {
                                        center + normal.normalized() * 14.0
                                    } else {
                                        center
                                    };
                                    vec![length_pos, a + egui::vec2(34.0, 18.0)]
                                }
                                // 3-point tools have no inline dimensions.
                                _ => Vec::new(),
                            };
                        } else {
                            self.dim_screen_positions.clear();
                        }

                        // GPU 3D scene: render the committed bodies plus any live
                        // operation preview (ghost volumes, replacement result
                        // sets) to an offscreen texture, composited inside
                        // draw_viewport. Skipped only when GPU rendering is off
                        // or the wgpu backend is unavailable.
                        if self.gpu_render
                            && self.gpu.is_available()
                        {
                            self.render_gpu_scene(rect, ctx);
                        } else {
                            self.gpu_texture_id = None;
                        }

                        // Draw the 3D projected CAD viewport
                        let painter = ui.painter_at(rect);
                        self.draw_viewport(painter.clone(), rect, hover_pos, current_cursor_snap);
                        if self.is_sketch_mode {
                            if let Some(model) = &self.sketch_solver_model {
                                let cs = self.active_sketch_cs;
                                let to_screen = |point: (f64, f64)| {
                                    let world =
                                        cs.unproject(point.0 as f32, point.1 as f32);
                                    let projected = project_3d(world.x, world.y, world.z);
                                    egui::pos2(projected.0, projected.1)
                                };
                                self.draw_constraint_badges(
                                    &painter,
                                    model,
                                    self.sketch_conflict_constraint,
                                    &to_screen,
                                );
                            }
                        }
                        let gizmo_project = |point: [f32; 3]| {
                            let p = project_3d(point[0], point[1], point[2]);
                            egui::pos2(p.0, p.1)
                        };
                        self.move_gizmo(&painter, &response, &gizmo_project);
                        painter
                    });
                });

        if let Some(rect) = viewport_rect {
            self.draw_viewport_controls(ctx, rect);
            self.draw_feature_tree(ctx, rect);
            if self.inspector_has_content() {
                self.draw_inspector(ctx, rect);
            }
        }

        // Dimension dialog overlay (drawn after the viewport, on top).
        self.show_dimension_dialog(ctx);
        self.show_sketch_dimension_editor(ctx);

        // Inline extrude distance box overlay (Fusion-style, mirrors the sketch
        // dimension dialog). Drawn on top of the viewport while extruding.
        self.show_extrude_dialog(ctx);
        self.show_revolve_dialog(ctx);
        self.show_pattern_dialog(ctx);
        self.show_hole_dialog(ctx);
        self.show_thread_dialog(ctx);
        self.show_shell_dialog(ctx);
        self.show_sweep_dialog(ctx);
        self.show_draft_dialog(ctx);

        // 3D fillet/chamfer: the drag manipulator on the edge, the inline size
        // box, and the inline 2D corner-radius box (anchored on the staged
        // corner/cursor). The handle is drawn first so the box layers over it.
        self.drag_edge_mod_handle(ctx);
        self.show_edge_mod_dialog(ctx);
        self.drag_corner_radius_handle(ctx);
        self.show_corner_radius_box(ctx);

        // Opt-in constraint palette + list for the active Edit Sketch session.
        self.show_constraints_panel(ctx);
    }

    fn draw_viewport_controls(&mut self, ctx: &egui::Context, viewport: egui::Rect) {
        if !self.is_sketch_mode {
            let inspector_offset = if self.inspector_has_content() {
                super::inspector::WORKSPACE_INSPECTOR_VIEWPORT_RESERVATION
            } else {
                0.0
            };
            egui::Area::new(egui::Id::new("view_cube"))
                .order(egui::Order::Foreground)
                .fixed_pos(egui::pos2(
                    viewport.right() - 140.0 - inspector_offset,
                    viewport.top() + 24.0,
                ))
                .show(ctx, |ui| {
                    let (rect, response) = ui.allocate_exact_size(
                        egui::vec2(112.0, 96.0),
                        egui::Sense::click_and_drag(),
                    );
                    if response.hovered() {
                        ui.output_mut(|output| {
                            output.cursor_icon = if response.dragged() {
                                egui::CursorIcon::Grabbing
                            } else {
                                egui::CursorIcon::Grab
                            };
                        });
                    }
                    // Project a real 3D cube with the same yaw/pitch transform as
                    // the viewport. The cube therefore follows every camera orbit,
                    // including drags that begin directly on the cube.
                    let center = rect.center();
                    let scale = 25.0;
                    let projected = [
                        [-1.0, -1.0, -1.0],
                        [1.0, -1.0, -1.0],
                        [1.0, 1.0, -1.0],
                        [-1.0, 1.0, -1.0],
                        [-1.0, -1.0, 1.0],
                        [1.0, -1.0, 1.0],
                        [1.0, 1.0, 1.0],
                        [-1.0, 1.0, 1.0],
                    ]
                    .map(|vertex| {
                        project_view_cube_vertex(
                            vertex,
                            self.camera_pitch,
                            self.camera_yaw,
                            center,
                            scale,
                        )
                    });
                    let projected_positions = projected.map(|(position, _)| position);
                    let mut faces: Vec<(ViewCubeFace, [egui::Pos2; 4], f32)> = [
                        (ViewCubeFace::Left, [0, 3, 7, 4]),
                        (ViewCubeFace::Right, [1, 5, 6, 2]),
                        (ViewCubeFace::Bottom, [0, 4, 5, 1]),
                        (ViewCubeFace::Top, [3, 2, 6, 7]),
                        (ViewCubeFace::Back, [0, 1, 2, 3]),
                        (ViewCubeFace::Front, [4, 7, 6, 5]),
                    ]
                    .into_iter()
                    .map(|(face, indices)| {
                        let points = indices.map(|index| projected[index].0);
                        let depth =
                            indices.iter().map(|index| projected[*index].1).sum::<f32>() / 4.0;
                        (face, points, depth)
                    })
                    .collect();
                    faces.sort_by(|left, right| {
                        left.2
                            .partial_cmp(&right.2)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });

                    // The nearest polygon under the pointer is the face that
                    // receives hover/click interaction.
                    let pointer = response.interact_pointer_pos();
                    let hovered_face = pointer.and_then(|point| {
                        faces
                            .iter()
                            .rev()
                            .find(|(_, points, _)| {
                                polygon_area(points) > 8.0 && point_in_convex_polygon(point, points)
                            })
                            .map(|(face, _, _)| *face)
                    });

                    for (face, points, _) in &faces {
                        if polygon_area(points) <= 8.0 {
                            continue;
                        }
                        let hovered = hovered_face == Some(*face);
                        let default_fill = view_cube_face_fill(*face, self.dark_mode);
                        ui.painter().add(egui::Shape::convex_polygon(
                            points.to_vec(),
                            if hovered {
                                self.pal().accent_soft
                            } else {
                                default_fill
                            },
                            egui::Stroke::new(
                                if hovered { 1.5 } else { 1.0 },
                                if hovered {
                                    self.pal().accent
                                } else {
                                    self.pal().border
                                },
                            ),
                        ));
                        draw_view_cube_face_label(
                            ui.painter(),
                            *face,
                            &view_cube_label_quad(*face, &projected_positions),
                            if hovered {
                                self.pal().accent
                            } else {
                                self.pal().text_strong
                            },
                        );
                    }

                    if response.dragged() {
                        let delta = ui.input(|input| input.pointer.delta());
                        self.camera_anim_active = false;
                        (self.camera_pitch, self.camera_yaw) =
                            orbit_camera_angles(self.camera_pitch, self.camera_yaw, delta);
                        self.is_perspective = true;
                        ctx.request_repaint();
                    } else if response.clicked() {
                        match hovered_face {
                            Some(ViewCubeFace::Top) => {
                                self.animate_to_view(ctx, std::f32::consts::FRAC_PI_2, 0.0)
                            }
                            Some(ViewCubeFace::Bottom) => {
                                self.animate_to_view(ctx, -std::f32::consts::FRAC_PI_2, 0.0)
                            }
                            Some(ViewCubeFace::Front) => self.animate_to_view(ctx, 0.0, 0.0),
                            Some(ViewCubeFace::Back) => {
                                self.animate_to_view(ctx, 0.0, std::f32::consts::PI)
                            }
                            Some(ViewCubeFace::Right) => {
                                self.animate_to_view(ctx, 0.0, std::f32::consts::FRAC_PI_2)
                            }
                            Some(ViewCubeFace::Left) => {
                                self.animate_to_view(ctx, 0.0, -std::f32::consts::FRAC_PI_2)
                            }
                            None => self.animate_to_view(ctx, 0.7, 0.7),
                        }
                    }

                    response.on_hover_text("Drag to orbit, or click a face to align the view");
                });
        }

        let controls_width = 190.0;
        egui::Area::new(egui::Id::new("viewport_navigation"))
            .order(egui::Order::Foreground)
            .fixed_pos(egui::pos2(
                viewport.center().x - controls_width * 0.5,
                viewport.bottom() - 72.0,
            ))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(self.pal().surface)
                    .stroke(egui::Stroke::new(1.0, self.pal().border))
                    .rounding(7.0)
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 2.0),
                        blur: 8.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(28),
                    })
                    .inner_margin(egui::Margin::symmetric(8.0, 6.0))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let transparent = egui::Color32::TRANSPARENT;
                            let hover = self.pal().accent_soft;
                            let color = self.pal().text_body;
                            if icons::Icon::Home
                                .icon_button(ui, transparent, hover, color)
                                .on_hover_text("Home / isometric view")
                                .clicked()
                            {
                                self.camera_pan = egui::Vec2::ZERO;
                                self.camera_zoom = 7.5;
                                self.animate_to_view(ctx, 0.7, 0.7);
                            }
                            if icons::Icon::Fit
                                .icon_button(ui, transparent, hover, color)
                                .on_hover_text("Fit all bodies")
                                .clicked()
                            {
                                self.fit_all_bodies();
                            }
                            if icons::Icon::ZoomIn
                                .icon_button(ui, transparent, hover, color)
                                .on_hover_text("Zoom in")
                                .clicked()
                            {
                                self.camera_zoom = (self.camera_zoom * 0.82).clamp(1.0, 50.0);
                            }
                            if icons::Icon::ZoomOut
                                .icon_button(ui, transparent, hover, color)
                                .on_hover_text("Zoom out")
                                .clicked()
                            {
                                self.camera_zoom = (self.camera_zoom * 1.22).clamp(1.0, 50.0);
                            }
                            let (pan_rect, pan_response) = ui
                                .allocate_exact_size(egui::vec2(24.0, 24.0), egui::Sense::hover());
                            icons::Icon::Pan.draw(ui.painter(), pan_rect.shrink(4.0), color);
                            pan_response.on_hover_text("Middle-drag to orbit; Shift-drag to pan");
                        });
                    });
            });
    }

    fn animate_to_view(&mut self, ctx: &egui::Context, pitch: f32, yaw: f32) {
        self.camera_anim_active = true;
        self.camera_anim_start_pitch = self.camera_pitch;
        self.camera_anim_start_yaw = self.camera_yaw;
        self.camera_anim_target_pitch = pitch;
        self.camera_anim_target_yaw = yaw;
        self.camera_anim_start_time = ctx.input(|input| input.time);
        self.is_perspective = true;
        ctx.request_repaint();
    }

    fn fit_all_bodies(&mut self) {
        if let Some((min, max)) = self.evaluated_scene.world_bounds() {
            let span = (0..3)
                .map(|axis| max[axis] - min[axis])
                .fold(0.0f32, f32::max)
                .max(1.0);
            self.camera_zoom = (span / 3.5).clamp(1.0, 50.0);
            self.camera_pan = egui::Vec2::ZERO;
        }
    }
}

fn point_in_convex_polygon(point: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
    let mut winding_sign = 0.0f32;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let edge = b - a;
        let offset = point - a;
        let cross = edge.x * offset.y - edge.y * offset.x;
        if cross.abs() <= 1.0e-4 {
            continue;
        }
        if winding_sign == 0.0 {
            winding_sign = cross.signum();
        } else if winding_sign * cross < 0.0 {
            return false;
        }
    }
    true
}

fn project_view_cube_vertex(
    [x, y, z]: [f32; 3],
    pitch: f32,
    yaw: f32,
    center: egui::Pos2,
    scale: f32,
) -> (egui::Pos2, f32) {
    let (sin_pitch, cos_pitch) = pitch.sin_cos();
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    let rotated_x = x * cos_yaw - z * sin_yaw;
    let rotated_z = x * sin_yaw + z * cos_yaw;
    let rotated_y = y * cos_pitch - rotated_z * sin_pitch;
    let depth = y * sin_pitch + rotated_z * cos_pitch;
    (
        center + egui::vec2(rotated_x * scale, -rotated_y * scale),
        depth,
    )
}

fn polygon_area<const N: usize>(polygon: &[egui::Pos2; N]) -> f32 {
    let twice_area = (0..N).fold(0.0, |area, index| {
        let current = polygon[index];
        let next = polygon[(index + 1) % N];
        area + current.x * next.y - next.x * current.y
    });
    twice_area.abs() * 0.5
}

fn view_cube_face_label(face: ViewCubeFace) -> &'static str {
    match face {
        ViewCubeFace::Top => "TOP",
        ViewCubeFace::Bottom => "BOTTOM",
        ViewCubeFace::Front => "FRONT",
        ViewCubeFace::Back => "BACK",
        ViewCubeFace::Right => "RIGHT",
        ViewCubeFace::Left => "LEFT",
    }
}

/// Return the four projected corners in reading order: top-left, top-right,
/// bottom-right, bottom-left when that face is viewed straight on. Keeping this
/// order independent from the hit-test polygon lets the lettering remain
/// printed onto the face instead of acting like screen-facing UI text.
fn view_cube_label_quad(face: ViewCubeFace, projected: &[egui::Pos2; 8]) -> [egui::Pos2; 4] {
    let indices = match face {
        ViewCubeFace::Top => [3, 2, 6, 7],
        ViewCubeFace::Bottom => [4, 5, 1, 0],
        ViewCubeFace::Front => [7, 6, 5, 4],
        ViewCubeFace::Back => [2, 3, 0, 1],
        ViewCubeFace::Right => [6, 2, 1, 5],
        ViewCubeFace::Left => [3, 7, 4, 0],
    };
    indices.map(|index| projected[index])
}

fn point_on_view_cube_face(quad: &[egui::Pos2; 4], u: f32, v: f32) -> egui::Pos2 {
    let top = quad[0] + (quad[1] - quad[0]) * u;
    let bottom = quad[3] + (quad[2] - quad[3]) * u;
    top + (bottom - top) * v
}

/// Lay out the normal UI typeface, then project its textured glyph mesh through
/// the face's local coordinate system. This keeps the antialiased font quality
/// while making every letter rotate and foreshorten with the cube surface.
fn draw_view_cube_face_label(
    painter: &egui::Painter,
    face: ViewCubeFace,
    quad: &[egui::Pos2; 4],
    color: egui::Color32,
) {
    if polygon_area(quad) < 70.0 {
        return;
    }

    let galley = painter.layout_no_wrap(
        view_cube_face_label(face).to_owned(),
        egui::FontId::proportional(14.0),
        color,
    );
    if galley.rect.width() <= f32::EPSILON || galley.rect.height() <= f32::EPSILON {
        return;
    }

    let font_texture_size = painter.ctx().fonts(|fonts| fonts.font_image_size());
    let uv_scale = egui::vec2(
        1.0 / font_texture_size[0] as f32,
        1.0 / font_texture_size[1] as f32,
    );
    let target_height = 0.31;
    let target_width = (galley.rect.width() / galley.rect.height() * target_height).min(0.82);
    let scale_x = target_width / galley.rect.width();
    let scale_y = (target_width * galley.rect.height() / galley.rect.width()).min(target_height)
        / galley.rect.height();
    let origin_u = 0.5 - galley.rect.width() * scale_x * 0.5;
    let origin_v = 0.5 - galley.rect.height() * scale_y * 0.5;
    let mut mesh = egui::Mesh::with_texture(egui::TextureId::default());

    for row in &galley.rows {
        let index_offset = mesh.vertices.len() as u32;
        mesh.indices.extend(
            row.visuals
                .mesh
                .indices
                .iter()
                .map(|index| index + index_offset),
        );
        mesh.vertices
            .extend(row.visuals.mesh.vertices.iter().map(|vertex| {
                let local_x = vertex.pos.x - galley.rect.left();
                let local_y = vertex.pos.y - galley.rect.top();
                egui::epaint::Vertex {
                    pos: point_on_view_cube_face(
                        quad,
                        origin_u + local_x * scale_x,
                        origin_v + local_y * scale_y,
                    ),
                    uv: (vertex.uv.to_vec2() * uv_scale).to_pos2(),
                    color,
                }
            }));
    }
    if !mesh.is_empty() {
        painter.add(egui::Shape::mesh(mesh));
    }
}

/// Apply the same unrestricted tumble to viewport and cube drags. Pitch is
/// wrapped instead of clamped, so dragging FRONT upward can continue naturally
/// over TOP and around to BACK.
pub(crate) fn orbit_camera_angles(pitch: f32, yaw: f32, pointer_delta: egui::Vec2) -> (f32, f32) {
    const SPEED: f32 = 0.008;
    (
        wrap_view_angle(pitch + pointer_delta.y * SPEED),
        wrap_view_angle(yaw - pointer_delta.x * SPEED),
    )
}

pub(crate) fn viewport_zoom_after_scroll(
    current_zoom: f32,
    scroll_delta: f32,
    viewport_hovered: bool,
) -> f32 {
    if !viewport_hovered || scroll_delta == 0.0 {
        return current_zoom;
    }
    (current_zoom * (scroll_delta * 0.002).exp()).clamp(1.0, 50.0)
}

fn wrap_view_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn view_cube_face_fill(face: ViewCubeFace, dark_mode: bool) -> egui::Color32 {
    if dark_mode {
        match face {
            ViewCubeFace::Top => egui::Color32::from_rgb(51, 65, 85),
            ViewCubeFace::Bottom => egui::Color32::from_rgb(15, 23, 42),
            ViewCubeFace::Front | ViewCubeFace::Left => egui::Color32::from_rgb(30, 41, 59),
            ViewCubeFace::Back | ViewCubeFace::Right => egui::Color32::from_rgb(24, 34, 52),
        }
    } else {
        match face {
            ViewCubeFace::Top => egui::Color32::from_rgb(239, 246, 255),
            ViewCubeFace::Bottom => egui::Color32::from_rgb(203, 213, 225),
            ViewCubeFace::Front | ViewCubeFace::Left => egui::Color32::from_rgb(248, 250, 252),
            ViewCubeFace::Back | ViewCubeFace::Right => egui::Color32::from_rgb(226, 232, 240),
        }
    }
}

/// Move a dimension box just outside its shape while keeping it attached to the
/// edge midpoint. The inline area is positioned around `(40, 10)` within the
/// box, so include that footprint before adding the visible gap; moving only
/// the anchor by a few pixels still left the widget drawn across the edge.
fn offset_dimension_box(point: egui::Pos2, center: egui::Pos2) -> egui::Pos2 {
    const ANCHOR_HALF_EXTENT: egui::Vec2 = egui::vec2(40.0, 10.0);
    const VISUAL_GAP: f32 = 8.0;
    let outward = point - center;
    if outward.length_sq() <= 1.0e-4 {
        point
    } else {
        let normal = outward.normalized();
        let footprint =
            normal.x.abs() * ANCHOR_HALF_EXTENT.x + normal.y.abs() * ANCHOR_HALF_EXTENT.y;
        point + normal * (footprint + VISUAL_GAP)
    }
}

/// Hit-test the solver model's points, then lines, then circles at `pos`
/// (screen space). Points win over entities so a shared corner is grabbable.
pub(crate) fn pick_solver_element(
    model: &zerocad_core::sketch::SketchSolverModel,
    pos: egui::Pos2,
    to_screen: &dyn Fn((f64, f64)) -> egui::Pos2,
    tol_px: f32,
) -> Option<zerocad_core::sketch::EntityId> {
    use zerocad_core::sketch::SketchEntity;
    if let Some((id, _)) = model
        .points
        .iter()
        .map(|p| (p.id, to_screen(p.pos).distance(pos)))
        .filter(|(_, d)| *d <= tol_px)
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    {
        return Some(id);
    }
    let point_pos = |id| model.point(id).map(|p| p.pos);
    let seg_dist = |a: egui::Pos2, b: egui::Pos2| -> f32 {
        let ab = b - a;
        let len2 = ab.length_sq();
        if len2 <= f32::EPSILON {
            return a.distance(pos);
        }
        let t = ((pos - a).dot(ab) / len2).clamp(0.0, 1.0);
        (a + ab * t).distance(pos)
    };
    model
        .entities
        .iter()
        .filter_map(|e| {
            let d = match e {
                SketchEntity::Line { p0, p1, .. } => {
                    seg_dist(to_screen(point_pos(*p0)?), to_screen(point_pos(*p1)?))
                }
                SketchEntity::Circle { center, radius, .. } => {
                    let c2 = point_pos(*center)?;
                    let c = to_screen(c2);
                    // Screen-space radius from a second projected sample.
                    let rim = to_screen((c2.0 + *radius, c2.1));
                    (c.distance(pos) - c.distance(rim)).abs()
                }
                SketchEntity::Arc {
                    center,
                    start,
                    end,
                    radius,
                    clockwise,
                    ..
                } => {
                    let center = point_pos(*center)?;
                    let start = point_pos(*start)?;
                    let end = point_pos(*end)?;
                    let arc = zerocad_core::sketch::Arc {
                        center: (center.0 as f32, center.1 as f32),
                        radius: *radius as f32,
                        start: (start.0 as f32, start.1 as f32),
                        end: (end.0 as f32, end.1 as f32),
                        clockwise: *clockwise,
                    };
                    crate::geom2d::sample_arc_points(&arc, 48)
                        .windows(2)
                        .map(|pair| {
                            seg_dist(
                                to_screen((pair[0].0 as f64, pair[0].1 as f64)),
                                to_screen((pair[1].0 as f64, pair[1].1 as f64)),
                            )
                        })
                        .fold(f32::INFINITY, f32::min)
                }
                SketchEntity::Ellipse {
                    center,
                    major_axis,
                    minor_axis,
                    start_parameter,
                    end_parameter,
                    closed,
                    ..
                } => {
                    let center = point_pos(*center)?;
                    let sweep = if *closed {
                        std::f64::consts::TAU
                    } else {
                        end_parameter - start_parameter
                    };
                    let point = |parameter: f64| {
                        let (sin, cos) = parameter.sin_cos();
                        to_screen((
                            center.0 + major_axis[0] * cos + minor_axis[0] * sin,
                            center.1 + major_axis[1] * cos + minor_axis[1] * sin,
                        ))
                    };
                    (0..64)
                        .map(|index| {
                            let a = *start_parameter + sweep * index as f64 / 64.0;
                            let b = *start_parameter + sweep * (index + 1) as f64 / 64.0;
                            seg_dist(point(a), point(b))
                        })
                        .fold(f32::INFINITY, f32::min)
                }
                SketchEntity::Spline {
                    points,
                    kind,
                    degree,
                    knots,
                    weights,
                    closed,
                    periodic,
                    continuity,
                    trim,
                    ..
                } => {
                    let points: Option<Vec<(f32, f32)>> = points
                        .iter()
                        .map(|id| point_pos(*id).map(|point| (point.0 as f32, point.1 as f32)))
                        .collect();
                    let spline = zerocad_core::Spline {
                        kind: *kind,
                        points: points?,
                        degree: *degree,
                        knots: knots.clone(),
                        weights: weights.clone(),
                        closed: *closed,
                        periodic: *periodic,
                        continuity: *continuity,
                        trim: *trim,
                    };
                    spline
                        .sampled_points(0.01)
                        .windows(2)
                        .map(|pair| {
                            seg_dist(
                                to_screen((pair[0].0 as f64, pair[0].1 as f64)),
                                to_screen((pair[1].0 as f64, pair[1].1 as f64)),
                            )
                        })
                        .fold(f32::INFINITY, f32::min)
                }
            };
            (d <= tol_px).then_some((e.id(), d))
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(id, _)| id)
}

#[cfg(test)]
mod dimension_anchor_tests {
    use super::{
        offset_dimension_box, orbit_camera_angles, pick_solver_element, point_in_convex_polygon,
        project_view_cube_vertex, viewport_zoom_after_scroll,
    };
    use eframe::egui;
    use zerocad_core::sketch::{EntityId, SketchEntity, SketchPoint, SketchSolverModel};

    #[test]
    fn dimension_anchor_stays_close_to_its_edge_midpoint() {
        let center = egui::pos2(100.0, 100.0);
        let bottom_mid = egui::pos2(100.0, 150.0);
        let anchored = offset_dimension_box(bottom_mid, center);
        // 10 px from the anchor to the box edge, then an 8 px visual gap.
        assert_eq!(anchored, egui::pos2(100.0, 168.0));

        let right_mid = egui::pos2(150.0, 100.0);
        let anchored = offset_dimension_box(right_mid, center);
        // The wider horizontal footprint is also kept fully off the edge.
        assert_eq!(anchored, egui::pos2(198.0, 100.0));
    }

    #[test]
    fn scroll_only_zooms_while_the_viewport_is_hovered() {
        let zoom = 7.5;
        assert_eq!(viewport_zoom_after_scroll(zoom, 120.0, false), zoom);
        assert!(viewport_zoom_after_scroll(zoom, 120.0, true) > zoom);
    }

    #[test]
    fn solver_arc_pick_uses_the_arc_span_instead_of_its_full_circle() {
        let model = SketchSolverModel {
            points: vec![
                SketchPoint {
                    id: EntityId(1),
                    pos: (0.0, 0.0),
                },
                SketchPoint {
                    id: EntityId(2),
                    pos: (10.0, 0.0),
                },
                SketchPoint {
                    id: EntityId(3),
                    pos: (0.0, 10.0),
                },
            ],
            entities: vec![SketchEntity::Arc {
                id: EntityId(4),
                center: EntityId(1),
                start: EntityId(2),
                end: EntityId(3),
                radius: 10.0,
                clockwise: false,
                derived_from: None,
            }],
            ..Default::default()
        };
        let to_screen = |point: (f64, f64)| egui::pos2(point.0 as f32, point.1 as f32);

        assert_eq!(
            pick_solver_element(&model, egui::pos2(7.1, 7.1), &to_screen, 1.5),
            Some(EntityId(4))
        );
        assert_eq!(
            pick_solver_element(&model, egui::pos2(-10.0, 0.0), &to_screen, 1.5),
            None
        );
    }

    #[test]
    fn view_cube_face_hit_testing_accepts_inside_and_rejects_outside() {
        let face = [
            egui::pos2(50.0, 10.0),
            egui::pos2(80.0, 30.0),
            egui::pos2(50.0, 50.0),
            egui::pos2(20.0, 30.0),
        ];
        assert!(point_in_convex_polygon(egui::pos2(50.0, 30.0), &face));
        assert!(!point_in_convex_polygon(egui::pos2(90.0, 30.0), &face));
    }

    #[test]
    fn view_cube_projection_tracks_camera_yaw() {
        let center = egui::pos2(100.0, 100.0);
        let (front, front_depth) =
            project_view_cube_vertex([1.0, 0.0, 0.0], 0.0, 0.0, center, 25.0);
        assert_eq!(front, egui::pos2(125.0, 100.0));
        assert!(front_depth.abs() <= f32::EPSILON);

        let (right, right_depth) = project_view_cube_vertex(
            [1.0, 0.0, 0.0],
            0.0,
            std::f32::consts::FRAC_PI_2,
            center,
            25.0,
        );
        assert!((right.x - center.x).abs() < 1.0e-4);
        assert!((right_depth - 1.0).abs() < 1.0e-4);
    }

    #[test]
    fn orbit_drag_can_continue_over_the_top_of_the_cube() {
        let nearly_top = std::f32::consts::FRAC_PI_2 - 0.01;
        let (pitch, yaw) = orbit_camera_angles(nearly_top, 0.25, egui::vec2(0.0, 10.0));
        assert!(pitch > std::f32::consts::FRAC_PI_2);
        assert!((yaw - 0.25).abs() < 1.0e-4);
    }
}
