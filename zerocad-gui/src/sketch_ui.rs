//! The Fusion-style inline dimension dialog shown while drawing a sketch shape,
//! plus the small value types that back it.

use eframe::egui;
use zerocad_core::Unit;

use crate::{SketchTool, ZeroCadApp};

/// One editable dimension field in the shape-creation dialog.
#[derive(Debug, Clone)]
pub(crate) struct DimField {
    pub(crate) value: String,
    /// Angle fields use degrees; every other sketch dimension follows the
    /// document's selected length unit.
    pub(crate) is_angle: bool,
    /// True once the user pressed Enter on it — its value is fixed.
    pub(crate) locked: bool,
    /// True once the user typed in it — stop overwriting it with the live value.
    pub(crate) edited: bool,
}

impl DimField {
    /// Turn a displayed dimension into an editor. Because a display-only box
    /// has no caret position, the first typed text replaces its shown value.
    fn begin_text_entry(&mut self) {
        self.value.clear();
        self.locked = false;
        self.edited = true;
    }
}

/// Fusion 360-style inline dimension inputs shown at edge midpoints during
/// shape creation. Disappears once the shape is finalized.
#[derive(Debug, Clone)]
pub(crate) struct DimInput {
    pub(crate) fields: Vec<DimField>,
    /// Field index to grab keyboard focus on the next render, if any.
    pub(crate) focus_request: Option<usize>,
    /// Which field is currently active (receives keyboard input).
    pub(crate) active_field: usize,
    /// The field currently rendered as a text editor. A selected field remains
    /// a display-only box until the user actually starts typing.
    pub(crate) editing_field: Option<usize>,
}

impl DimInput {
    /// Focus is requested only when a field first opens or Tab activates it.
    fn requests_focus(&self, field: usize) -> bool {
        self.focus_request == Some(field)
    }

    fn begin_active_text_entry(&mut self) {
        if self.fields.is_empty() || self.editing_field == Some(self.active_field) {
            return;
        }
        self.fields[self.active_field].begin_text_entry();
        self.editing_field = Some(self.active_field);
        self.focus_request = Some(self.active_field);
    }

    /// Tab cycles through every field, including completed fields. Keeping the
    /// completed value intact until typing begins makes navigation reversible.
    fn select_adjacent(&mut self, backwards: bool) {
        if self.fields.is_empty() {
            return;
        }
        self.active_field = if backwards {
            (self.active_field + self.fields.len() - 1) % self.fields.len()
        } else {
            (self.active_field + 1) % self.fields.len()
        };
        self.editing_field = None;
        self.focus_request = None;
    }

    /// Lock the active value and select the next unfinished dimension. Returns
    /// true once every field is complete.
    fn commit_active(&mut self) -> bool {
        if self.fields.is_empty() {
            return false;
        }
        let active = self.active_field;
        self.fields[active].locked = true;
        self.fields[active].edited = true;
        self.editing_field = None;
        self.focus_request = None;

        for offset in 1..=self.fields.len() {
            let next = (active + offset) % self.fields.len();
            if !self.fields[next].locked {
                self.active_field = next;
                return false;
            }
        }
        true
    }
}

/// Build the dimension fields for a tool.
pub(crate) fn dim_fields_for(tool: SketchTool) -> Vec<DimField> {
    let fields: &[bool] = match tool {
        SketchTool::Rectangle | SketchTool::RectangleCenter => &[false, false],
        SketchTool::Circle => &[false],
        SketchTool::PolygonInscribed | SketchTool::PolygonCircumscribed => &[false],
        SketchTool::Line => &[false, true],
        // 3-point tools (rotated rectangle, 3-point circle, ellipses) draw by
        // clicking points; the corner tools (fillet/chamfer) take their radius
        // from the toolbar. None use inline dimension fields.
        SketchTool::RectangleThreePoint
        | SketchTool::ThreePointCircle
        | SketchTool::Ellipse
        | SketchTool::ThreePointEllipse
        | SketchTool::Slot
        | SketchTool::Offset
        | SketchTool::Trim
        | SketchTool::ControlPointSpline
        | SketchTool::FitPointSpline
        | SketchTool::Mirror
        | SketchTool::Fillet
        | SketchTool::Chamfer => &[],
    };
    fields
        .iter()
        .map(|&is_angle| DimField {
            value: String::new(),
            is_angle,
            locked: false,
            edited: false,
        })
        .collect()
}

impl ZeroCadApp {
    /// Render Fusion 360-style inline dimension inputs at shape edge midpoints.
    /// Only shown while drawing (before the shape is finalized). Each field
    /// appears as a small box at the edge midpoint. The selected field has an
    /// orange outline but remains display-only (with no caret) until typing
    /// starts; the first typed character replaces its displayed value.
    /// Tab switches fields, Enter locks a field (finalizes when all locked),
    /// Escape cancels.
    pub(crate) fn show_dimension_dialog(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.dim_input.is_none() {
            return;
        }

        // Pulled out of `self` so the field widget can borrow the variable list
        // and the shared autocomplete state while the per-field closures hold
        // `&mut self`.
        let var_names = self.visible_variable_names();
        let var_map = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        // Set when the autocomplete swallowed Enter/Tab to accept a suggestion,
        // so that key isn't also treated as "lock field" / "next field".
        let mut suppress_keys = false;

        let field_count = self.dim_input.as_ref().map(|d| d.fields.len()).unwrap_or(0);
        let active = self.dim_input.as_ref().map(|d| d.active_field).unwrap_or(0);
        let starts_typing = ctx.input(|input| {
            input.events.iter().any(|event| {
                matches!(
                    event,
                    egui::Event::Text(text) | egui::Event::Paste(text) if !text.is_empty()
                )
            })
        });
        if starts_typing {
            if let Some(dim) = self.dim_input.as_mut() {
                dim.begin_active_text_entry();
            }
        }
        let unit_suffix = match self.current_unit {
            Unit::Millimeter => " mm",
            Unit::Inch => " in",
            Unit::Meter => " m",
        };

        // Render each field as an inline input at its screen position. Values may
        // be numbers, variables, or arithmetic expressions (evaluated later).
        for i in 0..field_count {
            let pos = self
                .dim_screen_positions
                .get(i)
                .copied()
                .unwrap_or(egui::pos2(100.0 + i as f32 * 120.0, 100.0));
            let is_active = i == active;
            let is_editing = self
                .dim_input
                .as_ref()
                .is_some_and(|dim| dim.editing_field == Some(i));
            let field_suffix = if self
                .dim_input
                .as_ref()
                .is_some_and(|dim| dim.fields[i].is_angle)
            {
                " °"
            } else {
                unit_suffix
            };

            // Read field state for rendering.
            let is_locked = self
                .dim_input
                .as_ref()
                .map(|d| d.fields[i].locked)
                .unwrap_or(false);
            let req_focus = self
                .dim_input
                .as_ref()
                .map(|d| d.requests_focus(i))
                .unwrap_or(false);

            // The orange outline marks keyboard selection without implying that
            // the display-only box is already a focused input.
            let border_color = if is_active {
                egui::Color32::from_rgb(245, 135, 25)
            } else if is_locked {
                egui::Color32::from_rgb(100, 160, 100)
            } else {
                egui::Color32::from_rgb(160, 160, 160)
            };
            let border_width = if is_active { 1.5 } else { 1.0 };
            let bg = egui::Color32::from_rgb(245, 245, 245);

            let area_id = egui::Id::new("dim_inline").with(i);
            egui::Area::new(area_id)
                .order(egui::Order::Foreground)
                .interactable(false)
                .fixed_pos(pos - egui::vec2(40.0, 10.0))
                .show(ctx, |ui| {
                    egui::Frame::none()
                        .fill(bg)
                        .rounding(3.0)
                        .stroke(egui::Stroke::new(border_width, border_color))
                        .inner_margin(egui::Margin::symmetric(4.0, 2.0))
                        .show(ui, |ui| {
                            ui.spacing_mut().item_spacing.x = 2.0;
                            ui.horizontal(|ui| {
                                if let Some(dim) = self.dim_input.as_mut() {
                                    let f = &mut dim.fields[i];
                                    ui.style_mut().visuals.extreme_bg_color = bg;
                                    ui.style_mut().visuals.widgets.inactive.bg_stroke =
                                        egui::Stroke::NONE;
                                    ui.style_mut().visuals.widgets.hovered.bg_stroke =
                                        egui::Stroke::NONE;

                                    let field_id = egui::Id::new(("sketch_dim_field", i));
                                    if is_editing {
                                        // Focus before laying out TextEdit so the event that
                                        // initiated editing is accepted on this same frame.
                                        if req_focus {
                                            ui.memory_mut(|memory| memory.request_focus(field_id));
                                        }
                                        let outcome = crate::expr::autocomplete_field(
                                            ui,
                                            field_id,
                                            &mut f.value,
                                            50.0,
                                            false,
                                            false,
                                            false,
                                            &var_names,
                                            &mut ac,
                                        );
                                        if outcome.response.changed() || outcome.accepted {
                                            f.edited = true;
                                        }
                                        if outcome.accepted_via_key {
                                            suppress_keys = true;
                                        }
                                    } else {
                                        ui.add_sized(
                                            [50.0, ui.spacing().interact_size.y],
                                            egui::Label::new(
                                                egui::RichText::new(&f.value)
                                                    .color(egui::Color32::from_rgb(30, 30, 30)),
                                            ),
                                        );
                                    }

                                    if zerocad_core::expr::preserves_source(&f.value) {
                                        if let Ok(value) = crate::expr::eval(&f.value, &var_map) {
                                            ui.label(
                                                egui::RichText::new(format!("= {value:.2}"))
                                                    .color(egui::Color32::from_rgb(70, 120, 70))
                                                    .size(11.0),
                                            );
                                        }
                                    }
                                }

                                // Unit suffix label.
                                ui.label(
                                    egui::RichText::new(field_suffix)
                                        .color(egui::Color32::from_rgb(120, 120, 120))
                                        .size(11.0),
                                );
                            });
                        });
                });
        }

        // Clear the one-shot focus request and stash autocomplete.
        if let Some(d) = self.dim_input.as_mut() {
            d.focus_request = None;
        }
        self.autocomplete = ac;

        // Commit/navigation keys are read *after* the fields render, so an
        // Enter/Tab the autocomplete consumed (to accept a suggestion) never
        // leaks into locking the field or finalizing the shape.
        let enter = ctx.input(|i| i.key_pressed(egui::Key::Enter));
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        let tab = ctx.input(|i| i.key_pressed(egui::Key::Tab));

        // Tab cycles through all fields. This intentionally includes completed
        // fields so a user can return to one and replace its value by typing.
        if tab && !suppress_keys && field_count > 0 {
            if let Some(dim) = self.dim_input.as_mut() {
                let backwards = ctx.input(|i| i.modifiers.shift);
                dim.select_adjacent(backwards);
            }
            self.autocomplete = None;
        }

        // Enter: lock the active field, advance to the next, or finalize.
        if enter && !suppress_keys {
            if let Some(dim) = self.dim_input.as_mut() {
                let all_locked = dim.commit_active();
                if all_locked {
                    let start = self.sketch_temp_start.unwrap_or((0.0, 0.0));
                    let cursor = self.last_cursor.unwrap_or((start.0 + 1.0, start.1 + 1.0));
                    self.finalize_shape(cursor);
                }
            }
        }

        if escape {
            // End a continuous-Line chain (the dialog is re-opened on every
            // re-seed, so a mid-chain Escape lands here). Committed segments are
            // kept; only the rubber-band is dropped.
            let was_chaining = self.line_chain_start.is_some();
            self.line_chain_start = None;
            self.cancel_in_progress_shape();
            self.autocomplete = None;
            self.status_msg = if was_chaining {
                "Line chain ended.".to_string()
            } else {
                "Shape cancelled.".to_string()
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{dim_fields_for, DimField, DimInput};
    use crate::{SketchTool, ZeroCadApp};
    use eframe::egui;

    fn dimension_input(focus_request: Option<usize>) -> DimInput {
        DimInput {
            fields: (0..2)
                .map(|_| DimField {
                    value: "12.34".to_string(),
                    is_angle: false,
                    locked: false,
                    edited: false,
                })
                .collect(),
            focus_request,
            active_field: 0,
            editing_field: None,
        }
    }

    #[test]
    fn selected_dimension_is_display_only_until_typing_replaces_seed() {
        let mut dim = dimension_input(Some(0));
        assert!(dim.requests_focus(0));
        assert!(!dim.requests_focus(1));
        assert_eq!(dim.editing_field, None);

        // The field remains active after the request is consumed without a
        // visible select-all state.
        dim.focus_request = None;
        assert_eq!(dim.active_field, 0);
        assert!(!dim.requests_focus(0));

        dim.begin_active_text_entry();
        assert_eq!(dim.editing_field, Some(0));
        assert!(dim.fields[0].edited);
        assert!(dim.fields[0].value.is_empty());
    }

    #[test]
    fn selected_box_only_creates_and_focuses_editor_when_typing_starts() {
        let mut app = ZeroCadApp::new();
        app.is_sketch_mode = true;
        app.dim_input = Some(dimension_input(None));
        app.dim_screen_positions = vec![egui::pos2(100.0, 100.0), egui::pos2(220.0, 100.0)];

        let ctx = egui::Context::default();
        let field_id = egui::Id::new(("sketch_dim_field", 0));
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            app.show_dimension_dialog(ctx);
        });
        assert!(!ctx.memory(|memory| memory.has_focus(field_id)));
        assert_eq!(app.dim_input.as_ref().unwrap().editing_field, None);

        let mut input = egui::RawInput::default();
        input.events.push(egui::Event::Text("7".to_string()));
        let _ = ctx.run(input, |ctx| {
            app.show_dimension_dialog(ctx);
        });

        let dim = app.dim_input.as_ref().unwrap();
        assert_eq!(dim.editing_field, Some(0));
        assert_eq!(dim.fields[0].value, "7");
        assert!(ctx.memory(|memory| memory.has_focus(field_id)));
    }

    #[test]
    fn tab_can_return_to_a_completed_dimension_for_replacement() {
        let mut dim = dimension_input(None);
        assert!(!dim.commit_active());
        assert!(dim.fields[0].locked);
        assert_eq!(dim.active_field, 1);

        dim.select_adjacent(false);
        assert_eq!(dim.active_field, 0);
        assert!(dim.fields[0].locked);
        assert_eq!(dim.editing_field, None);

        dim.begin_active_text_entry();
        assert!(!dim.fields[0].locked);
        assert_eq!(dim.editing_field, Some(0));
        assert!(dim.fields[0].value.is_empty());
    }

    #[test]
    fn line_angle_field_uses_degrees_instead_of_length_units() {
        let fields = dim_fields_for(SketchTool::Line);
        assert_eq!(fields.len(), 2);
        assert!(!fields[0].is_angle);
        assert!(fields[1].is_angle);
    }

    #[test]
    fn polygon_uses_one_length_field() {
        for tool in [
            SketchTool::PolygonInscribed,
            SketchTool::PolygonCircumscribed,
        ] {
            let fields = dim_fields_for(tool);
            assert_eq!(fields.len(), 1);
            assert!(!fields[0].is_angle);
        }
    }
}
