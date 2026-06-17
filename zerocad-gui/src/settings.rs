//! Persistent app state that lives outside the document: general preferences
//! (`settings.json`), the recent-projects list (`recent.json`), and the cached
//! Recent thumbnails (`thumbs/*.thumb`). All of it sits next to the keyboard
//! shortcuts in the OS config dir and follows the same best-effort pattern as
//! [`crate::shortcuts`] — load falls back to defaults, save logs on error and is
//! never fatal.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use zerocad_core::Unit;

/// `%APPDATA%/ZeroCAD` on Windows; `$XDG_CONFIG_HOME` / `$HOME/.config` elsewhere.
/// `None` if no home dir can be found. Mirrors `shortcuts::config_path`'s base.
fn config_dir() -> Option<PathBuf> {
    let base = std::env::var_os("APPDATA")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from))
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("ZeroCAD"))
}

fn now_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// General preferences
// ---------------------------------------------------------------------------

/// Preferences that should survive a restart. Anything the Settings window or a
/// preference toggle edits and the user expects to "stick" belongs here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AppSettings {
    /// Whether the onboarding (Welcome) screen pops up on startup.
    pub show_onboarding: bool,
    /// Dark theme on/off.
    pub dark_mode: bool,
    /// Default measurement unit.
    pub unit: Unit,
}

impl Default for AppSettings {
    fn default() -> Self {
        AppSettings {
            show_onboarding: true,
            dark_mode: false,
            unit: Unit::Millimeter,
        }
    }
}

impl AppSettings {
    fn path() -> Option<PathBuf> {
        Some(config_dir()?.join("settings.json"))
    }

    /// Load preferences, falling back to defaults on any error (missing file,
    /// older/corrupt JSON).
    pub fn load() -> Self {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<AppSettings>(&s).ok())
            .unwrap_or_default()
    }

    /// Persist preferences (best-effort; errors are logged, not fatal).
    pub fn save(&self) {
        let Some(path) = Self::path() else {
            return;
        };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match serde_json::to_string_pretty(self) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&path, json) {
                    log::warn!("Could not save settings to {:?}: {e}", path);
                }
            }
            Err(e) => log::warn!("Could not serialize settings: {e}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Recent projects
// ---------------------------------------------------------------------------

/// One entry in the recent-projects list.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecentEntry {
    pub path: PathBuf,
    /// Unix seconds of the last save/open, newest first in the list.
    pub last_opened: u64,
}

/// The recent-projects list, newest first. Capped so the file stays small; the
/// onboarding screen shows only the first few.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RecentFiles {
    pub entries: Vec<RecentEntry>,
}

/// Max entries kept on disk (the onboarding screen shows fewer).
const RECENT_CAP: usize = 10;

impl RecentFiles {
    fn path() -> Option<PathBuf> {
        Some(config_dir()?.join("recent.json"))
    }

    /// Load the list and drop any entry whose file no longer exists, so the
    /// onboarding screen never offers a dead link.
    pub fn load() -> Self {
        let mut rf = Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<RecentFiles>(&s).ok())
            .unwrap_or_default();
        rf.entries.retain(|e| e.path.exists());
        rf
    }

    /// Record `path` as the most-recently-used project: move it to the front
    /// (de-duplicated), stamp it now, and cap the list. Persists immediately.
    pub fn record(&mut self, path: &Path) {
        let path = path.to_path_buf();
        self.entries.retain(|e| e.path != path);
        self.entries.insert(
            0,
            RecentEntry {
                path,
                last_opened: now_secs(),
            },
        );
        self.entries.truncate(RECENT_CAP);
        self.save();
    }

    fn save(&self) {
        let Some(path) = Self::path() else {
            return;
        };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match serde_json::to_string_pretty(self) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&path, json) {
                    log::warn!("Could not save recent list to {:?}: {e}", path);
                }
            }
            Err(e) => log::warn!("Could not serialize recent list: {e}"),
        }
    }

    /// De-duplicated parent directories from the recent entries list (newest
    /// first), for populating the save dialog's "Recent Folders" section.
    pub fn recent_folders(&self) -> Vec<PathBuf> {
        let mut seen = std::collections::HashSet::new();
        let mut folders = Vec::new();
        for entry in &self.entries {
            if let Some(parent) = entry.path.parent() {
                let p = parent.to_path_buf();
                if seen.insert(p.clone()) {
                    folders.push(p);
                }
            }
        }
        folders
    }
}

// ---------------------------------------------------------------------------
// Thumbnail cache
// ---------------------------------------------------------------------------

/// FNV-1a over the (absolute, if resolvable) document path, used as a stable
/// cache filename. `std::hash::DefaultHasher` is *not* guaranteed stable across
/// runs, so we hash explicitly.
fn path_hash(file: &Path) -> u64 {
    let abs = std::fs::canonicalize(file).unwrap_or_else(|_| file.to_path_buf());
    let s = abs.to_string_lossy();
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Path of the cached thumbnail for `file`, or `None` if there's no config dir.
fn thumb_path(file: &Path) -> Option<PathBuf> {
    Some(
        config_dir()?
            .join("thumbs")
            .join(format!("{:016x}.thumb", path_hash(file))),
    )
}

/// Cache an RGBA thumbnail for `file`. Format is dependency-free raw bytes:
/// `[w: u32 LE][h: u32 LE][rgba bytes]`. Best-effort; errors are logged.
pub fn save_thumb(file: &Path, w: usize, h: usize, rgba: &[u8]) {
    let Some(path) = thumb_path(file) else {
        return;
    };
    if let Some(dir) = path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut bytes = Vec::with_capacity(8 + rgba.len());
    bytes.extend_from_slice(&(w as u32).to_le_bytes());
    bytes.extend_from_slice(&(h as u32).to_le_bytes());
    bytes.extend_from_slice(rgba);
    if let Err(e) = std::fs::write(&path, bytes) {
        log::warn!("Could not save thumbnail to {:?}: {e}", path);
    }
}

/// Load a cached thumbnail for `file` as `(width, height, rgba)`, or `None` if
/// absent or malformed.
pub fn load_thumb(file: &Path) -> Option<(usize, usize, Vec<u8>)> {
    let bytes = std::fs::read(thumb_path(file)?).ok()?;
    if bytes.len() < 8 {
        return None;
    }
    let w = u32::from_le_bytes(bytes[0..4].try_into().ok()?) as usize;
    let h = u32::from_le_bytes(bytes[4..8].try_into().ok()?) as usize;
    let rgba = &bytes[8..];
    if w == 0 || h == 0 || rgba.len() != w * h * 4 {
        return None;
    }
    Some((w, h, rgba.to_vec()))
}
