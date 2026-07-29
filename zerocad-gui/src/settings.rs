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
pub(crate) fn config_dir() -> Option<PathBuf> {
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

/// Which graphics backend the GPU viewport requests from wgpu. Applied when the
/// app starts (changing it needs a restart — the device/surface are created
/// once at launch). `Auto` lets wgpu pick the best available backend, which on
/// Windows normally means Vulkan, with DirectX 12 next. If the chosen backend
/// can't initialize, the app falls back to the glow (OpenGL) renderer with the
/// CPU viewport rather than failing to start.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) enum GraphicsBackend {
    #[default]
    Auto,
    Vulkan,
    /// DirectX 12 (Windows only).
    Dx12,
    /// wgpu's OpenGL backend, where the platform provides one.
    OpenGl,
}

impl GraphicsBackend {
    pub(crate) const ALL: [GraphicsBackend; 4] = [
        GraphicsBackend::Auto,
        GraphicsBackend::Vulkan,
        GraphicsBackend::Dx12,
        GraphicsBackend::OpenGl,
    ];

    pub(crate) fn label(self) -> &'static str {
        match self {
            GraphicsBackend::Auto => "Auto (recommended)",
            GraphicsBackend::Vulkan => "Vulkan",
            GraphicsBackend::Dx12 => "DirectX 12",
            GraphicsBackend::OpenGl => "OpenGL",
        }
    }
}

/// Anti-aliasing quality of the GPU viewport. Applied live (no restart) — the
/// render pipelines are rebuilt on change. 8× falls back to 4× on adapters
/// that don't support it for the surface format.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) enum MsaaLevel {
    Off,
    #[default]
    X4,
    X8,
}

impl MsaaLevel {
    pub(crate) const ALL: [MsaaLevel; 3] = [MsaaLevel::Off, MsaaLevel::X4, MsaaLevel::X8];

    pub(crate) fn label(self) -> &'static str {
        match self {
            MsaaLevel::Off => "Off",
            MsaaLevel::X4 => "4× (recommended)",
            MsaaLevel::X8 => "8×",
        }
    }

    /// The wgpu sample count this level asks for.
    pub(crate) fn samples(self) -> u32 {
        match self {
            MsaaLevel::Off => 1,
            MsaaLevel::X4 => 4,
            MsaaLevel::X8 => 8,
        }
    }
}

/// Preferences that should survive a restart. Anything the Settings window or a
/// preference toggle edits and the user expects to "stick" belongs here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct AppSettings {
    /// Whether the onboarding (Welcome) screen pops up on startup.
    pub(crate) show_onboarding: bool,
    /// Dark theme on/off.
    pub(crate) dark_mode: bool,
    /// Default measurement unit.
    pub(crate) unit: Unit,
    /// GPU-accelerated 3D viewport on/off (falls back to the CPU software
    /// renderer when off). Defaults on; `serde(default)` keeps older
    /// settings.json files loading.
    #[serde(default = "default_true")]
    pub(crate) gpu_render: bool,
    /// Requested wgpu backend for the GPU viewport (see [`GraphicsBackend`]).
    #[serde(default)]
    pub(crate) backend: GraphicsBackend,
    /// Anti-aliasing quality of the GPU viewport (see [`MsaaLevel`]).
    #[serde(default)]
    pub(crate) msaa: MsaaLevel,
    #[serde(default = "default_hydrated_cache_mb")]
    pub(crate) hydrated_cache_mb: u32,
    /// Master sketch snapping toggle. Added with a default so existing
    /// settings.json files remain compatible.
    #[serde(default = "default_true")]
    pub(crate) snap_enabled: bool,
    /// Whether reference grids are drawn in the model and sketch viewports.
    #[serde(default = "default_true")]
    pub(crate) grid_visible: bool,
    /// Whether the sketch constraint palette and inferred-constraint glyphs are
    /// visible. The solver remains active when this presentation preference is
    /// off.
    #[serde(default)]
    pub(crate) show_sketch_constraints: bool,
}

fn default_true() -> bool {
    true
}

fn default_hydrated_cache_mb() -> u32 {
    128
}

impl Default for AppSettings {
    fn default() -> Self {
        AppSettings {
            show_onboarding: true,
            dark_mode: false,
            unit: Unit::Millimeter,
            gpu_render: true,
            backend: GraphicsBackend::Auto,
            msaa: MsaaLevel::X4,
            hydrated_cache_mb: default_hydrated_cache_mb(),
            snap_enabled: true,
            grid_visible: true,
            show_sketch_constraints: false,
        }
    }
}

impl AppSettings {
    fn path() -> Option<PathBuf> {
        Some(config_dir()?.join("settings.json"))
    }

    /// Load preferences, falling back to defaults on any error (missing file,
    /// older/corrupt JSON).
    pub(crate) fn load() -> Self {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<AppSettings>(&s).ok())
            .unwrap_or_default()
    }

    /// Persist preferences (best-effort; errors are logged, not fatal).
    pub(crate) fn save(&self) {
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

#[cfg(test)]
mod tests {
    use super::AppSettings;

    #[test]
    fn sketch_constraints_are_opt_in_for_defaults_and_older_settings() {
        assert!(!AppSettings::default().show_sketch_constraints);

        let older = r#"{
            "show_onboarding": true,
            "dark_mode": false,
            "unit": "Millimeter"
        }"#;
        let loaded: AppSettings = serde_json::from_str(older).expect("older settings");
        assert!(!loaded.show_sketch_constraints);
    }
}

// ---------------------------------------------------------------------------
// Recent projects
// ---------------------------------------------------------------------------

/// One entry in the recent-projects list.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct RecentEntry {
    pub(crate) path: PathBuf,
    /// Unix seconds of the last save/open, newest first in the list.
    pub(crate) last_opened: u64,
    #[serde(default)]
    pub(crate) project_kind: Option<zerocad_core::ProjectKind>,
}

/// The recent-projects list, newest first. Capped so the file stays small; the
/// onboarding screen shows only the first few.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub(crate) struct RecentFiles {
    pub(crate) entries: Vec<RecentEntry>,
}

/// Max entries kept on disk (the onboarding screen shows fewer).
const RECENT_CAP: usize = 10;

impl RecentFiles {
    fn path() -> Option<PathBuf> {
        Some(config_dir()?.join("recent.json"))
    }

    /// Load the list and drop any entry whose file no longer exists, so the
    /// onboarding screen never offers a dead link.
    pub(crate) fn load() -> Self {
        let mut rf = Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<RecentFiles>(&s).ok())
            .unwrap_or_default();
        rf.entries.retain(|e| e.path.exists());
        rf
    }

    /// Record `path` as the most-recently-used project: move it to the front
    /// (de-duplicated), stamp it now, and cap the list. Persists immediately.
    pub(crate) fn record(&mut self, path: &Path, project_kind: zerocad_core::ProjectKind) {
        let path = path.to_path_buf();
        self.entries.retain(|e| e.path != path);
        self.entries.insert(
            0,
            RecentEntry {
                path,
                last_opened: now_secs(),
                project_kind: Some(project_kind),
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
    pub(crate) fn recent_folders(&self) -> Vec<PathBuf> {
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
pub(crate) fn save_thumb(file: &Path, w: usize, h: usize, rgba: &[u8]) {
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
pub(crate) fn load_thumb(file: &Path) -> Option<(usize, usize, Vec<u8>)> {
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
