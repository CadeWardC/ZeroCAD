//! Dimension-field input widget: a single-line text box with variable-name
//! autocomplete (↑/↓/Enter/click to accept). The arithmetic evaluation it feeds
//! lives in `zerocad_core::expr`, so the parametric engine and the UI share one
//! grammar; this module just re-exports `eval` and adds the egui widget.

use eframe::egui;

pub(crate) use zerocad_core::expr::eval;

// ===========================================================================
// Autocomplete text field
// ===========================================================================

fn is_ident_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// Open variable-name suggestion popup. At most one is live at a time across all
/// dimension fields, keyed by the owning field's [`egui::Id`].
pub(crate) struct Autocomplete {
    owner: egui::Id,
    matches: Vec<String>,
    selected: usize,
}

/// The maximal identifier token straddling `cursor` (in char units): its
/// `start..end` char range and the prefix `[start, cursor)` used for matching.
fn token_at(chars: &[char], cursor: usize) -> (usize, usize, String) {
    let cursor = cursor.min(chars.len());
    let mut start = cursor;
    while start > 0 && is_ident_char(chars[start - 1]) {
        start -= 1;
    }
    let mut end = cursor;
    while end < chars.len() && is_ident_char(chars[end]) {
        end += 1;
    }
    let prefix: String = chars[start..cursor].iter().collect();
    (start, end, prefix)
}

/// What a [`autocomplete_field`] call did this frame, beyond the bare response.
pub(crate) struct FieldOutcome {
    pub(crate) response: egui::Response,
    /// A completion was inserted (via Enter/Tab or a click on the popup).
    pub(crate) accepted: bool,
    /// The completion was accepted with Enter/Tab — the host dialog should treat
    /// that key as "accept suggestion", not "commit/lock the field".
    pub(crate) accepted_via_key: bool,
}

/// A single-line dimension field with variable autocomplete. `center` centers
/// the text (the Fusion-style extrude box) and `width` is the field width;
/// `request_focus` force-focuses the field and (with `select_all`) selects its
/// text. The caller is expected to have set the surrounding `ui` visuals
/// already.
#[allow(clippy::too_many_arguments)]
pub(crate) fn autocomplete_field(
    ui: &mut egui::Ui,
    id: egui::Id,
    text: &mut String,
    width: f32,
    center: bool,
    request_focus: bool,
    select_all: bool,
    var_names: &[String],
    ac: &mut Option<Autocomplete>,
) -> FieldOutcome {
    // 1) Intercept navigation keys BEFORE the TextEdit sees them, but only while
    //    our popup is open — so arrows don't move the caret and Enter doesn't
    //    bubble up to the dialog as a commit.
    let popup_open = ac
        .as_ref()
        .map_or(false, |a| a.owner == id && !a.matches.is_empty());
    let mut accept_now = false;
    if popup_open {
        let (down, up, accept, esc) = ui.input_mut(|i| {
            (
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                i.consume_key(egui::Modifiers::NONE, egui::Key::Enter)
                    || i.consume_key(egui::Modifiers::NONE, egui::Key::Tab),
                i.consume_key(egui::Modifiers::NONE, egui::Key::Escape),
            )
        });
        if let Some(a) = ac.as_mut() {
            let n = a.matches.len();
            if down {
                a.selected = (a.selected + 1) % n;
            }
            if up {
                a.selected = (a.selected + n - 1) % n;
            }
        }
        accept_now = accept;
        if esc {
            *ac = None;
        }
    }

    // 2) Render the field.
    let mut edit = egui::TextEdit::singleline(text)
        .id(id)
        .desired_width(width)
        .font(egui::TextStyle::Body)
        .text_color(egui::Color32::from_rgb(30, 30, 30))
        .frame(false);
    if center {
        edit = edit.horizontal_align(egui::Align::Center);
    }
    let output = edit.show(ui);
    let response = output.response.clone();

    // Force focus (e.g. the active field), optionally selecting the whole value
    // so typing replaces the seeded number — the Fusion-style dimension boxes.
    if request_focus {
        response.request_focus();
        if select_all {
            let len = text.chars().count();
            let mut st = output.state.clone();
            st.cursor.set_char_range(Some(egui::text::CCursorRange::two(
                egui::text::CCursor::new(0),
                egui::text::CCursor::new(len),
            )));
            st.store(ui.ctx(), id);
        }
    }

    let focused = response.has_focus();

    // Current caret position (char index); fall back to end of text.
    let chars: Vec<char> = text.chars().collect();
    let cursor = output
        .state
        .cursor
        .char_range()
        .map(|r| r.primary.index)
        .unwrap_or(chars.len());

    // 3) While focused (and not accepting), recompute the suggestion list for the
    //    token under the caret.
    if focused && !accept_now {
        let (_s, _e, prefix) = token_at(&chars, cursor);
        if prefix.is_empty() {
            if ac.as_ref().map_or(false, |a| a.owner == id) {
                *ac = None;
            }
        } else {
            let pl = prefix.to_lowercase();
            let matches: Vec<String> = var_names
                .iter()
                .filter(|n| n.to_lowercase().starts_with(&pl))
                .cloned()
                .collect();
            // Hide when the only suggestion is exactly what's typed.
            let only_exact = matches.len() == 1 && matches[0] == prefix;
            if matches.is_empty() || only_exact {
                if ac.as_ref().map_or(false, |a| a.owner == id) {
                    *ac = None;
                }
            } else {
                let selected = match ac.as_ref() {
                    Some(a) if a.owner == id => a.selected.min(matches.len() - 1),
                    _ => 0,
                };
                *ac = Some(Autocomplete {
                    owner: id,
                    matches,
                    selected,
                });
            }
        }
    }

    // 4) Draw the popup (if ours) and capture a click / hover. Drawn even when the
    //    field just lost focus to the popup itself, so the click registers.
    let mut clicked: Option<usize> = None;
    let mut popup_hovered = false;
    if !accept_now {
        if let Some(a) = ac.as_ref().filter(|a| a.owner == id && !a.matches.is_empty()) {
            let anchor = response.rect.left_bottom() + egui::vec2(0.0, 3.0);
            let area = egui::Area::new(id.with("ac_popup"))
                .order(egui::Order::Tooltip)
                .fixed_pos(anchor)
                .show(ui.ctx(), |ui| {
                    egui::Frame::popup(ui.style())
                        .inner_margin(egui::Margin::symmetric(4.0, 4.0))
                        .show(ui, |ui| {
                            ui.set_min_width(response.rect.width().max(130.0));
                            ui.spacing_mut().item_spacing.y = 1.0;
                            for (i, name) in a.matches.iter().enumerate() {
                                let label = egui::RichText::new(name)
                                    .monospace()
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(30, 30, 30));
                                if ui.selectable_label(i == a.selected, label).clicked() {
                                    clicked = Some(i);
                                }
                            }
                        });
                });
            popup_hovered = area.response.hovered();
        }
    }

    // 5) Resolve an acceptance (Enter/Tab on the selection, or a click) and apply
    //    it: replace the token under the caret with the chosen name.
    let accept_index = if accept_now {
        ac.as_ref().filter(|a| a.owner == id).map(|a| a.selected)
    } else {
        clicked
    };
    let mut accepted = false;
    if let Some(idx) = accept_index {
        let chosen = ac
            .as_ref()
            .and_then(|a| a.matches.get(idx))
            .cloned();
        if let Some(name) = chosen {
            let (start, end, _p) = token_at(&chars, cursor);
            let mut new: String = chars[..start].iter().collect();
            new.push_str(&name);
            let new_cursor = start + name.chars().count();
            new.extend(chars[end..].iter());
            *text = new;
            // Restore focus + place the caret after the inserted name.
            let mut st = output.state.clone();
            st.cursor.set_char_range(Some(egui::text::CCursorRange::two(
                egui::text::CCursor::new(new_cursor),
                egui::text::CCursor::new(new_cursor),
            )));
            st.store(ui.ctx(), id);
            response.request_focus();
            *ac = None;
            accepted = true;
        }
    } else if popup_open && !focused && !popup_hovered {
        // Field lost focus and nobody is using the popup — close it.
        if ac.as_ref().map_or(false, |a| a.owner == id) {
            *ac = None;
        }
    }

    FieldOutcome {
        response,
        accepted,
        accepted_via_key: accepted && accept_now,
    }
}

#[cfg(test)]
mod tests {
    // Arithmetic evaluation itself is tested in `zerocad_core::expr`; here we
    // only cover the widget's token-boundary logic.
    use super::token_at;

    #[test]
    fn token_at_finds_identifier() {
        let chars: Vec<char> = "wi + height".chars().collect();
        // caret right after "wi"
        let (s, e, p) = token_at(&chars, 2);
        assert_eq!((s, e), (0, 2));
        assert_eq!(p, "wi");
        // caret in the middle of "height" (after "he")
        let (s, e, p) = token_at(&chars, 7);
        assert_eq!((s, e), (5, 11));
        assert_eq!(p, "he");
    }
}
