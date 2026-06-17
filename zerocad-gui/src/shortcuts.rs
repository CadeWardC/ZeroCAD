//! User-rebindable keyboard shortcuts: the bindable action set, their key
//! bindings, and JSON persistence to the OS config dir. Kept separate from the
//! app logic so the Settings "Shortcuts" tab and the global dispatcher in
//! `update()` share one source of truth.

use std::path::PathBuf;

use eframe::egui;
use serde::{Deserialize, Serialize};

/// Every command the user can bind a key to. To add one: add a variant, list it
/// in [`ShortcutAction::ALL`], give it a [`label`](ShortcutAction::label) and a
/// [`default_hotkey`](ShortcutAction::default_hotkey), and handle it in
/// `ZeroCadApp::run_shortcut`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ShortcutAction {
    NewDesign,
    OpenDesign,
    SaveDesign,
    ExportStl,
    Undo,
    Redo,
    DeleteSelection,
    ToggleTheme,
    OpenSettings,
}

impl ShortcutAction {
    /// All actions, in the order they appear in the Shortcuts settings tab.
    pub const ALL: &'static [ShortcutAction] = &[
        ShortcutAction::NewDesign,
        ShortcutAction::OpenDesign,
        ShortcutAction::SaveDesign,
        ShortcutAction::ExportStl,
        ShortcutAction::Undo,
        ShortcutAction::Redo,
        ShortcutAction::DeleteSelection,
        ShortcutAction::ToggleTheme,
        ShortcutAction::OpenSettings,
    ];

    /// Human-readable name shown in the settings list.
    pub fn label(self) -> &'static str {
        match self {
            ShortcutAction::NewDesign => "New Design",
            ShortcutAction::OpenDesign => "Open Design",
            ShortcutAction::SaveDesign => "Save Design",
            ShortcutAction::ExportStl => "Export STL",
            ShortcutAction::Undo => "Undo",
            ShortcutAction::Redo => "Redo",
            ShortcutAction::DeleteSelection => "Delete Selection",
            ShortcutAction::ToggleTheme => "Toggle Dark Mode",
            ShortcutAction::OpenSettings => "Open Settings",
        }
    }

    /// The factory-default binding, used on first run and by "Reset to defaults".
    pub fn default_hotkey(self) -> Hotkey {
        use ShortcutAction::*;
        match self {
            NewDesign => Hotkey::ctrl(egui::Key::N),
            OpenDesign => Hotkey::ctrl(egui::Key::O),
            SaveDesign => Hotkey::ctrl(egui::Key::S),
            ExportStl => Hotkey::ctrl(egui::Key::E),
            Undo => Hotkey::ctrl(egui::Key::Z),
            Redo => Hotkey::ctrl(egui::Key::Y),
            DeleteSelection => Hotkey::plain(egui::Key::Delete),
            ToggleTheme => Hotkey::ctrl(egui::Key::D),
            OpenSettings => Hotkey::ctrl(egui::Key::Comma),
        }
    }
}

/// A modifier + key combo. The key is stored by its egui `Key::name()` so the
/// binding round-trips through JSON without depending on egui's optional serde
/// feature. `ctrl` means the platform command key (Ctrl on Win/Linux, ⌘ on Mac).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hotkey {
    pub ctrl: bool,
    pub shift: bool,
    pub alt: bool,
    /// egui `Key::name()`, e.g. "S", "Delete", "Comma".
    pub key: String,
}

impl Hotkey {
    fn ctrl(key: egui::Key) -> Self {
        Hotkey {
            ctrl: true,
            shift: false,
            alt: false,
            key: key.name().to_string(),
        }
    }

    fn plain(key: egui::Key) -> Self {
        Hotkey {
            ctrl: false,
            shift: false,
            alt: false,
            key: key.name().to_string(),
        }
    }

    /// Build from a captured key event's key + modifier state.
    pub fn from_event(key: egui::Key, mods: egui::Modifiers) -> Self {
        Hotkey {
            ctrl: mods.command || mods.ctrl,
            shift: mods.shift,
            alt: mods.alt,
            key: key.name().to_string(),
        }
    }

    fn egui_key(&self) -> Option<egui::Key> {
        egui::Key::from_name(&self.key)
    }

    /// True only on the frame this exact combo is freshly pressed. Modifiers are
    /// matched exactly so Ctrl+Z does not also fire on Ctrl+Shift+Z.
    pub fn pressed(&self, ctx: &egui::Context) -> bool {
        let Some(k) = self.egui_key() else {
            return false;
        };
        ctx.input(|i| {
            if !i.key_pressed(k) {
                return false;
            }
            let m = i.modifiers;
            let cmd = m.command || m.ctrl; // platform Ctrl, or ⌘ on Mac
            cmd == self.ctrl && m.shift == self.shift && m.alt == self.alt
        })
    }

    /// Human-readable form, e.g. "Ctrl+Shift+S" or "Delete".
    pub fn label(&self) -> String {
        let mut s = String::new();
        if self.ctrl {
            s.push_str("Ctrl+");
        }
        if self.shift {
            s.push_str("Shift+");
        }
        if self.alt {
            s.push_str("Alt+");
        }
        let key_str = self
            .egui_key()
            .map(pretty_key)
            .unwrap_or_else(|| self.key.clone());
        s.push_str(&key_str);
        s
    }
}

/// Symbol form for punctuation keys so a binding reads "Ctrl+," not "Ctrl+Comma".
fn pretty_key(k: egui::Key) -> String {
    match k {
        egui::Key::Comma => ",".to_string(),
        egui::Key::Period => ".".to_string(),
        egui::Key::Semicolon => ";".to_string(),
        egui::Key::Minus => "-".to_string(),
        egui::Key::Plus => "+".to_string(),
        other => other.name().to_string(),
    }
}

/// The full set of action bindings. Stored as an ordered list (the action set is
/// tiny) so it serializes without needing string map keys.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Keymap {
    bindings: Vec<(ShortcutAction, Hotkey)>,
}

impl Default for Keymap {
    fn default() -> Self {
        Keymap {
            bindings: ShortcutAction::ALL
                .iter()
                .map(|a| (*a, a.default_hotkey()))
                .collect(),
        }
    }
}

impl Keymap {
    /// The combo currently bound to `action`, if any.
    pub fn get(&self, action: ShortcutAction) -> Option<&Hotkey> {
        self.bindings
            .iter()
            .find(|(a, _)| *a == action)
            .map(|(_, h)| h)
    }

    /// Bind `hotkey` to `action`. Any other action already using the same combo
    /// is unbound, so a combo always maps to exactly one action.
    pub fn set(&mut self, action: ShortcutAction, hotkey: Hotkey) {
        self.bindings.retain(|(a, h)| *a != action && *h != hotkey);
        self.bindings.push((action, hotkey));
    }

    /// Remove `action`'s binding entirely (the action becomes unbound).
    pub fn unbind(&mut self, action: ShortcutAction) {
        self.bindings.retain(|(a, _)| *a != action);
    }

    /// Restore the factory defaults.
    pub fn reset_to_defaults(&mut self) {
        *self = Keymap::default();
    }

    /// Fill in defaults for any action missing a binding — e.g. after loading an
    /// older config saved before that action existed. Skips a default whose
    /// combo a user has reassigned elsewhere, leaving the action unbound instead.
    fn backfill_defaults(&mut self) {
        for a in ShortcutAction::ALL {
            if self.get(*a).is_none() {
                let def = a.default_hotkey();
                if !self.bindings.iter().any(|(_, h)| *h == def) {
                    self.bindings.push((*a, def));
                }
            }
        }
    }

    /// Load from the config file, falling back to defaults on any error.
    pub fn load() -> Self {
        let mut km = config_path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<Keymap>(&s).ok())
            .unwrap_or_default();
        km.backfill_defaults();
        km
    }

    /// Persist to the config file (best-effort; errors are logged, not fatal).
    pub fn save(&self) {
        let Some(path) = config_path() else {
            return;
        };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match serde_json::to_string_pretty(self) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&path, json) {
                    log::warn!("Could not save shortcuts to {:?}: {e}", path);
                }
            }
            Err(e) => log::warn!("Could not serialize shortcuts: {e}"),
        }
    }
}

/// `%APPDATA%/ZeroCAD/shortcuts.json` on Windows; `$XDG_CONFIG_HOME` or
/// `$HOME/.config` elsewhere. `None` if no home dir can be found.
fn config_path() -> Option<PathBuf> {
    let base = std::env::var_os("APPDATA")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from))
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("ZeroCAD").join("shortcuts.json"))
}
