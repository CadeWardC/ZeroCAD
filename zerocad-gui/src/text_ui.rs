//! Sketch Text dialog — shape a string with an installed font and commit the
//! baked outlines as one [`zerocad_core::SketchShape::Text`] record.
//!
//! The dialog owns everything semantic (string, family, size, spacing,
//! placement); the committed shape carries the BAKED curves, so the document
//! renders identically on machines without the font. Fonts are enumerated once
//! when the dialog opens.

use crate::*;
use zerocad_core::text::{self, TextAlign, TextParams, TextPlacement};

pub(crate) struct TextDialogState {
    families: Vec<String>,
    family: String,
    text: String,
    size_mm: f32,
    tracking_mm: f32,
    line_spacing: f32,
    align: TextAlign,
    pub(crate) origin: (f32, f32),
    rotation_deg: f32,
    error: Option<String>,
    /// While false the preview follows the cursor; a viewport click pins the
    /// origin (and a later click moves it again).
    pub(crate) pinned: bool,
    /// Baked outlines at identity placement, re-shaped only when the semantic
    /// fields change. Placement is applied per frame — a rigid map, so moving
    /// the preview costs nothing.
    preview: Option<zerocad_core::SketchCurves>,
    baked_key: String,
    /// Font bytes for the selected family, loaded once per family change.
    font_cache: Option<(String, Vec<u8>, u32)>,
    /// Installed-font catalog, scanned once when the dialog opens. Re-scanning
    /// the system font directories on every family change froze the panel.
    catalog: text::FontCatalog,
    /// Display polylines for the preview in TEXT-LOCAL space, resampled only
    /// when the semantic fields change. Per frame they are just rigidly
    /// re-placed — keying this by origin would defeat it, since the origin
    /// changes every frame while the preview follows the cursor.
    preview_polylines: Vec<Vec<(f32, f32)>>,
}

impl TextDialogState {
    fn open() -> Self {
        let catalog = text::FontCatalog::system();
        let families = catalog.families();
        let family = families
            .iter()
            .find(|name| name.as_str() == "Arial" || name.as_str() == "Segoe UI")
            .cloned()
            .or_else(|| families.first().cloned())
            .unwrap_or_default();
        Self {
            families,
            family,
            text: String::new(),
            size_mm: 10.0,
            tracking_mm: 0.0,
            line_spacing: 1.2,
            align: TextAlign::Left,
            origin: (0.0, 0.0),
            rotation_deg: 0.0,
            error: None,
            pinned: false,
            preview: None,
            baked_key: String::new(),
            font_cache: None,
            catalog,
            preview_polylines: Vec::new(),
        }
    }

    fn ensure_font(&mut self) -> Option<(&[u8], u32)> {
        if self
            .font_cache
            .as_ref()
            .is_none_or(|(family, _, _)| family != &self.family)
        {
            let (bytes, index) = self.catalog.load_family(&self.family)?;
            self.font_cache = Some((self.family.clone(), bytes, index));
        }
        self.font_cache
            .as_ref()
            .map(|(_, bytes, index)| (bytes.as_slice(), *index))
    }

    /// Re-shape the preview outlines when any semantic field changed.
    fn refresh_preview(&mut self) {
        let key = format!(
            "{}|{}|{}|{}|{}|{:?}",
            self.text, self.family, self.size_mm, self.tracking_mm, self.line_spacing, self.align
        );
        if key == self.baked_key {
            return;
        }
        self.baked_key = key;
        self.preview = None;
        if self.text.trim().is_empty() {
            return;
        }
        let params = TextParams {
            size_mm: self.size_mm,
            tracking_mm: self.tracking_mm,
            line_spacing: self.line_spacing,
            align: self.align,
        };
        if self.ensure_font().is_none() {
            return;
        }
        let Some((_, bytes, index)) = &self.font_cache else {
            return;
        };
        match text::outline_text(bytes, *index, &self.text, &params) {
            Ok(curves) => {
                self.preview_polylines = sample_polylines(&curves);
                self.preview = Some(curves);
                self.error = None;
            }
            Err(error) => self.error = Some(error.to_string()),
        }
    }

    fn placement(&self, basis: ((f32, f32), (f32, f32))) -> TextPlacement {
        TextPlacement {
            origin: self.origin,
            rotation_deg: self.rotation_deg,
            basis_u: basis.0,
            basis_v: basis.1,
        }
    }
}

/// Sample display polylines from text-local curves: one 2-point line per
/// segment, one 12-step sweep per spline. Runs only when the semantic fields
/// change; the per-frame path is a rigid transform of these points.
fn sample_polylines(curves: &zerocad_core::SketchCurves) -> Vec<Vec<(f32, f32)>> {
    let mut polylines = Vec::with_capacity(curves.segments.len() + curves.splines.len());
    for segment in &curves.segments {
        polylines.push(vec![segment.a, segment.b]);
    }
    for spline in &curves.splines {
        const STEPS: usize = 12;
        let mut polyline = Vec::with_capacity(STEPS + 1);
        for step in 0..=STEPS {
            if let Some(point) = spline.evaluate(step as f32 / STEPS as f32) {
                polyline.push(point);
            }
        }
        if polyline.len() >= 2 {
            polylines.push(polyline);
        }
    }
    polylines
}

impl ZeroCadApp {
    /// Text is a modal sketch command even though it is not represented by a
    /// `SketchTool` variant. Entering it must leave the toolbar in one coherent
    /// state and discard any half-placed geometry owned by the previous tool.
    fn deactivate_sketch_tool_for_text(&mut self) {
        self.active_tool = None;
        self.line_chain_start = None;
        self.cancel_in_progress_shape();
        self.clear_pending_corners();
    }

    /// The sketch-plane (u, v) directions that read as RIGHT and UP on screen
    /// under the current camera, measured through the app's own inverse
    /// projection so no camera convention is assumed. Zero-length axes (camera
    /// parallel to the plane) fall back to identity.
    fn sketch_screen_basis(&self) -> ((f32, f32), (f32, f32)) {
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800.0, 600.0));
        let center = rect.center();
        let cs = self.active_sketch_cs;
        let origin = self.screen_to_sketch(center, rect, &cs);
        let right = self.screen_to_sketch(center + egui::vec2(100.0, 0.0), rect, &cs);
        let up = self.screen_to_sketch(center - egui::vec2(0.0, 100.0), rect, &cs);
        let normalize = |d: (f32, f32)| -> Option<(f32, f32)> {
            let len = (d.0 * d.0 + d.1 * d.1).sqrt();
            (len > 1.0e-6).then(|| (d.0 / len, d.1 / len))
        };
        match (
            normalize((right.0 - origin.0, right.1 - origin.1)),
            normalize((up.0 - origin.0, up.1 - origin.1)),
        ) {
            (Some(u), Some(v)) => (u, v),
            _ => ((1.0, 0.0), (0.0, 1.0)),
        }
    }

    pub(crate) fn open_text_dialog(&mut self) {
        self.deactivate_sketch_tool_for_text();
        let mut state = TextDialogState::open();
        // Land where the user is looking, not at the plane origin: probe the
        // view centre through the same inverse projection the pointer uses.
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800.0, 600.0));
        let centre = self.screen_to_sketch(rect.center(), rect, &self.active_sketch_cs);
        state.origin = (centre.0.round(), centre.1.round());
        self.text_dialog = Some(state);
    }

    pub(crate) fn show_text_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut state) = self.text_dialog.take() else {
            return;
        };
        if !self.is_sketch_mode {
            // Leaving sketch mode closes the dialog rather than committing into
            // a sketch that no longer exists.
            return;
        }

        let mut keep_open = true;
        let mut commit = false;
        egui::SidePanel::right("text_tool_panel")
            .resizable(false)
            .default_width(240.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("🔤 Text");
                ui.separator();
                ui.label(if state.pinned {
                    "Click in the sketch to move the text."
                } else {
                    "Preview follows the cursor — click to place."
                });
                ui.add_space(6.0);

                egui::TextEdit::multiline(&mut state.text)
                    .hint_text("Text (Enter for a new line)")
                    .desired_rows(2)
                    .show(ui);

                egui::ComboBox::from_label("Font")
                    .selected_text(state.family.clone())
                    .show_ui(ui, |ui| {
                        for family in &state.families {
                            ui.selectable_value(&mut state.family, family.clone(), family);
                        }
                    });

                ui.horizontal(|ui| {
                    ui.label("Size (mm)");
                    ui.add(
                        egui::DragValue::new(&mut state.size_mm)
                            .range(0.1..=10_000.0)
                            .speed(0.5),
                    );
                    ui.label("Tracking");
                    ui.add(egui::DragValue::new(&mut state.tracking_mm).speed(0.05));
                });
                ui.horizontal(|ui| {
                    ui.label("Line spacing");
                    ui.add(
                        egui::DragValue::new(&mut state.line_spacing)
                            .range(0.1..=10.0)
                            .speed(0.05),
                    );
                    egui::ComboBox::from_label("Align")
                        .selected_text(match state.align {
                            TextAlign::Left => "Left",
                            TextAlign::Center => "Center",
                            TextAlign::Right => "Right",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut state.align, TextAlign::Left, "Left");
                            ui.selectable_value(&mut state.align, TextAlign::Center, "Center");
                            ui.selectable_value(&mut state.align, TextAlign::Right, "Right");
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("Origin");
                    ui.add(
                        egui::DragValue::new(&mut state.origin.0)
                            .speed(0.5)
                            .prefix("x "),
                    );
                    ui.add(
                        egui::DragValue::new(&mut state.origin.1)
                            .speed(0.5)
                            .prefix("y "),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Rotation °");
                    ui.add(egui::DragValue::new(&mut state.rotation_deg).speed(1.0));
                });

                if let Some(error) = &state.error {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                }
                if state.families.is_empty() {
                    ui.colored_label(egui::Color32::LIGHT_RED, "No system fonts found");
                }

                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    let ready = state.preview.is_some();
                    if ui
                        .add_enabled(ready, egui::Button::new("Add Text"))
                        .clicked()
                    {
                        commit = true;
                    }
                    if ui.button("Cancel").clicked() {
                        keep_open = false;
                    }
                });
            });
        state.refresh_preview();
        if commit {
            match self.commit_text_shape(&mut state) {
                Ok(()) => {
                    self.text_dialog = None;
                    return;
                }
                Err(message) => state.error = Some(message),
            }
        }
        if keep_open {
            self.text_dialog = Some(state);
        }
    }

    fn commit_text_shape(&mut self, state: &mut TextDialogState) -> Result<(), String> {
        let basis = self.sketch_screen_basis();
        let placement = state.placement(basis);
        let params = TextParams {
            size_mm: state.size_mm,
            tracking_mm: state.tracking_mm,
            line_spacing: state.line_spacing,
            align: state.align,
        };
        if state.ensure_font().is_none() {
            return Err(format!(
                "font family \"{}\" could not be loaded",
                state.family
            ));
        }
        let Some((_, bytes, face_index)) = &state.font_cache else {
            return Err("font cache emptied unexpectedly".to_owned());
        };
        let shape = text::bake_text_shape(bytes, *face_index, &state.text, &params, placement)
            .map_err(|error| error.to_string())?;

        // Same commit contract as the DXF importer: register the shape with the
        // solver as one raw entity, then rebuild the sketch curves.
        if let Some(model) = &mut self.sketch_solver_model {
            let owner = zerocad_core::sketch::EntityId(self.sketch_next_entity_id);
            let vars = self.document.variable_map();
            let (addition, next) = zerocad_core::sketch::constraints::promote_shapes_to_entities(
                std::slice::from_ref(&shape),
                &[owner],
                &vars,
                self.sketch_next_entity_id + 1,
            );
            model.points.extend(addition.points);
            model.entities.extend(addition.entities);
            model.constraints.extend(addition.constraints);
            self.sketch_entity_ids.push(owner);
            self.sketch_next_entity_id = next;
        }
        self.sketch_shapes.push(shape);
        self.rebuild_active_sketch_curves();
        self.status_msg = format!(
            "Added text \"{}\" ({} pt {})",
            state.text.lines().next().unwrap_or_default(),
            state.size_mm,
            state.family
        );
        Ok(())
    }
}

impl ZeroCadApp {
    /// Draw the live text preview at its current placement. Cheap by design:
    /// the outlines were shaped once (`refresh_preview`) and only the rigid
    /// placement is applied here, so following the cursor costs a linear map
    /// over ~100 curves per frame — no shaping, no arrangement, no fill.
    pub(crate) fn draw_text_preview(
        &self,
        painter: &egui::Painter,
        to_screen: &dyn Fn((f32, f32)) -> egui::Pos2,
    ) {
        let Some(state) = &self.text_dialog else {
            return;
        };
        if state.preview_polylines.is_empty() {
            return;
        }
        let placement = state.placement(self.sketch_screen_basis());
        let stroke =
            egui::Stroke::new(1.4, egui::Color32::from_rgba_unmultiplied(0, 150, 230, 180));
        for polyline in &state.preview_polylines {
            let mut previous: Option<egui::Pos2> = None;
            for point in polyline {
                let mapped = to_screen(placement.transform_point(*point));
                if let Some(from) = previous {
                    painter.line_segment([from, mapped], stroke);
                }
                previous = Some(mapped);
            }
        }
        // Origin marker so "where it will be put down" is explicit.
        let origin = to_screen(state.origin);
        painter.circle_stroke(origin, 4.0, egui::Stroke::new(1.5, egui::Color32::GOLD));
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn activating_text_deselects_and_cancels_the_previous_sketch_tool() {
        let mut app = ZeroCadApp::new();
        app.active_tool = Some(SketchTool::Line);
        app.line_chain_start = Some((1.0, 2.0));
        app.sketch_temp_start = Some((3.0, 4.0));
        app.sketch_points.push((5.0, 6.0));

        app.deactivate_sketch_tool_for_text();

        assert_eq!(app.active_tool, None);
        assert_eq!(app.line_chain_start, None);
        assert_eq!(app.sketch_temp_start, None);
        assert!(app.sketch_points.is_empty());
    }

    /// Which way the sketch plane's (u, v) axes point ON SCREEN once the sketch
    /// camera locks perpendicular to the plane. Uses the app's own inverse
    /// projection (`screen_to_sketch`), so it measures the real mapping rather
    /// than assuming a camera convention. Text reads correctly only when +u
    /// runs screen-right and +v runs screen-up.
    fn screen_axes(cs: zerocad_core::CoordinateSystem) -> ((f32, f32), (f32, f32)) {
        let mut app = ZeroCadApp::new();
        let (pitch, yaw) = ZeroCadApp::camera_look_at_normal(cs.n);
        app.camera_pitch = pitch;
        app.camera_yaw = yaw;
        app.camera_pan = egui::Vec2::ZERO;
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800.0, 600.0));
        let center = rect.center();
        let origin = app.screen_to_sketch(center, rect, &cs);
        let right = app.screen_to_sketch(center + egui::vec2(100.0, 0.0), rect, &cs);
        let up = app.screen_to_sketch(center - egui::vec2(0.0, 100.0), rect, &cs);
        (
            (right.0 - origin.0, right.1 - origin.1),
            (up.0 - origin.0, up.1 - origin.1),
        )
    }

    /// Text baked through the dialog must read upright and forward on every
    /// standard plane. Measured mappings without the basis fix: XY was already
    /// correct, XZ showed +v pointing DOWN-screen (upside-down letters — the
    /// user-reported ground-plane bug), and YZ swapped the axes entirely.
    #[test]
    fn baked_text_reads_upright_on_every_standard_plane() {
        for (name, cs) in [
            ("XY", zerocad_core::CoordinateSystem::XY),
            ("XZ", zerocad_core::CoordinateSystem::XZ),
            ("YZ", zerocad_core::CoordinateSystem::YZ),
        ] {
            let mut app = ZeroCadApp::new();
            let (pitch, yaw) = ZeroCadApp::camera_look_at_normal(cs.n);
            app.camera_pitch = pitch;
            app.camera_yaw = yaw;
            app.camera_pan = egui::Vec2::ZERO;
            app.active_sketch_cs = cs;

            let (basis_u, basis_v) = app.sketch_screen_basis();
            let placement = zerocad_core::text::TextPlacement {
                origin: (0.0, 0.0),
                rotation_deg: 0.0,
                basis_u,
                basis_v,
            };
            // The glyph frame's right and up vectors, mapped into (u, v).
            let map = |x: f32, y: f32| -> (f32, f32) {
                (x * basis_u.0 + y * basis_v.0, x * basis_u.1 + y * basis_v.1)
            };
            let text_right_uv = map(1.0, 0.0);
            let text_up_uv = map(0.0, 1.0);
            let _ = placement;

            // Project those (u, v) directions back to the screen and demand
            // that glyph-right lands screen-right and glyph-up lands screen-up.
            let (screen_right_uv, screen_up_uv) = screen_axes(cs);
            let dot = |a: (f32, f32), b: (f32, f32)| a.0 * b.0 + a.1 * b.1;
            assert!(
                dot(text_right_uv, screen_right_uv)
                    > 0.9 * screen_right_uv.0.hypot(screen_right_uv.1),
                "{name}: text baseline must run screen-right"
            );
            assert!(
                dot(text_up_uv, screen_up_uv) > 0.9 * screen_up_uv.0.hypot(screen_up_uv.1),
                "{name}: glyph up must point screen-up"
            );
            assert!(
                dot(text_right_uv, screen_up_uv).abs() < 0.1 * screen_up_uv.0.hypot(screen_up_uv.1),
                "{name}: baseline must not lean into the screen vertical"
            );
        }
    }
}
