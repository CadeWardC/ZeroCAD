//! Crash-safe, offline recovery for public-alpha and release builds.

use std::backtrace::Backtrace;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Mutex, OnceLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use zerocad_core::{
    read_project_document_file, write_project_document_file, HydrationBundle, LoadOptions,
    ProjectDocument, SaveOptions,
};

const EDIT_DEBOUNCE: Duration = Duration::from_secs(5);
const AUTOSAVE_INTERVAL: Duration = Duration::from_secs(30);
const AUTOSAVE_FILE: &str = "autosave.zcad";

static PANIC_DOCUMENT: OnceLock<Mutex<Option<ProjectDocument>>> = OnceLock::new();
static PANIC_ROOT: OnceLock<PathBuf> = OnceLock::new();
static PANIC_HOOK_INSTALLED: AtomicBool = AtomicBool::new(false);

enum RecoveryCommand {
    Save(Box<ProjectDocument>),
    Clear,
}

/// Coalescing autosave worker. It never blocks the UI and writes only compact,
/// authoritative document recipes through the same atomic writer as Save.
pub(crate) struct RecoveryManager {
    root: Option<PathBuf>,
    tx: Option<mpsc::SyncSender<RecoveryCommand>>,
    latest: Option<ProjectDocument>,
    dirty_since: Option<Instant>,
    last_autosave: Option<Instant>,
}

impl RecoveryManager {
    #[cfg(test)]
    pub(crate) fn new() -> Self {
        Self::new_in(None)
    }

    #[cfg(not(test))]
    pub(crate) fn new() -> Self {
        Self::new_in(crate::settings::config_dir().map(|dir| dir.join("recovery")))
    }

    fn disabled() -> Self {
        Self {
            root: None,
            tx: None,
            latest: None,
            dirty_since: None,
            last_autosave: None,
        }
    }

    fn new_in(root: Option<PathBuf>) -> Self {
        let Some(root) = root else {
            return Self::disabled();
        };
        if let Err(error) = std::fs::create_dir_all(&root) {
            log::warn!(
                "recovery directory unavailable at {}: {error}",
                root.display()
            );
            return Self::disabled();
        }
        let (tx, rx) = mpsc::sync_channel(1);
        let worker_root = root.clone();
        std::thread::Builder::new()
            .name("zerocad-recovery-writer".into())
            .spawn(move || {
                while let Ok(command) = rx.recv() {
                    match command {
                        RecoveryCommand::Save(document) => {
                            let path = worker_root.join(AUTOSAVE_FILE);
                            if let Err(error) = write_recovery_document(&path, &document) {
                                log::warn!("autosave failed at {}: {error}", path.display());
                            }
                        }
                        RecoveryCommand::Clear => clear_recovery_files(&worker_root),
                    }
                }
            })
            .ok();
        Self {
            root: Some(root),
            tx: Some(tx),
            latest: None,
            dirty_since: None,
            last_autosave: None,
        }
    }

    pub(crate) fn note_edit(&mut self, document: ProjectDocument) {
        update_panic_snapshot(&document);
        self.latest = Some(document);
        self.dirty_since.get_or_insert_with(Instant::now);
    }

    pub(crate) fn tick(&mut self) {
        let Some(dirty_since) = self.dirty_since else {
            return;
        };
        if dirty_since.elapsed() < EDIT_DEBOUNCE
            || self
                .last_autosave
                .is_some_and(|saved| saved.elapsed() < AUTOSAVE_INTERVAL)
        {
            return;
        }
        let (Some(tx), Some(document)) = (&self.tx, self.latest.clone()) else {
            return;
        };
        if tx
            .try_send(RecoveryCommand::Save(Box::new(document)))
            .is_ok()
        {
            self.last_autosave = Some(Instant::now());
            self.dirty_since = None;
        }
    }

    pub(crate) fn mark_saved(&mut self, document: &ProjectDocument) {
        update_panic_snapshot(document);
        self.latest = Some(document.clone());
        self.dirty_since = None;
        self.last_autosave = None;
        if let Some(tx) = &self.tx {
            let _ = tx.send(RecoveryCommand::Clear);
        }
    }

    pub(crate) fn has_recovery(&self) -> bool {
        self.root
            .as_ref()
            .is_some_and(|root| newest_recovery_path(root).is_some())
    }

    pub(crate) fn load_latest(&self) -> Result<ProjectDocument, String> {
        let root = self
            .root
            .as_ref()
            .ok_or("recovery storage is unavailable")?;
        let path = newest_recovery_path(root).ok_or("no recovery document is available")?;
        read_project_document_file(&path, &LoadOptions::default())
            .map(|loaded| loaded.document)
            .map_err(|error| format!("could not open {}: {error}", path.display()))
    }
}

fn write_recovery_document(
    path: &Path,
    document: &ProjectDocument,
) -> Result<(), zerocad_core::ZcadError> {
    write_project_document_file(
        path,
        document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
}

fn clear_recovery_files(root: &Path) {
    let Ok(entries) = std::fs::read_dir(root) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let is_recovery = path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name == AUTOSAVE_FILE || name.starts_with("panic-recovery-"));
        if is_recovery {
            let _ = std::fs::remove_file(path);
        }
    }
}

fn newest_recovery_path(root: &Path) -> Option<PathBuf> {
    std::fs::read_dir(root)
        .ok()?
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            let name = path.file_name()?.to_str()?;
            if name != AUTOSAVE_FILE && !name.starts_with("panic-recovery-") {
                return None;
            }
            let modified = entry.metadata().ok()?.modified().ok()?;
            Some((modified, path))
        })
        .max_by_key(|(modified, _)| *modified)
        .map(|(_, path)| path)
}

fn update_panic_snapshot(document: &ProjectDocument) {
    if let Ok(mut slot) = PANIC_DOCUMENT.get_or_init(|| Mutex::new(None)).lock() {
        *slot = Some(document.clone());
    }
}

/// Install an app-boundary panic hook. Caught kernel panics execute on named
/// evaluation workers and are deliberately ignored here; only a panic escaping
/// the main application thread writes a crash document.
pub(crate) fn install_panic_hook() {
    if PANIC_HOOK_INSTALLED.swap(true, Ordering::AcqRel) {
        return;
    }
    let Some(root) = crate::settings::config_dir().map(|dir| dir.join("recovery")) else {
        return;
    };
    let _ = std::fs::create_dir_all(&root);
    let _ = PANIC_ROOT.set(root);
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if std::thread::current()
            .name()
            .is_none_or(|name| name == "main")
        {
            write_panic_recovery(info);
        }
        previous(info);
    }));
}

fn write_panic_recovery(info: &std::panic::PanicHookInfo<'_>) {
    let (Some(root), Some(document_lock)) = (PANIC_ROOT.get(), PANIC_DOCUMENT.get()) else {
        return;
    };
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let base = format!("panic-recovery-{stamp}-{}", std::process::id());
    if let Ok(document) = document_lock.try_lock() {
        if let Some(document) = document.as_ref() {
            let _ = write_recovery_document(&root.join(format!("{base}.zcad")), document);
        }
    }
    let location = info
        .location()
        .map(|location| {
            format!(
                "{}:{}:{}",
                location.file(),
                location.line(),
                location.column()
            )
        })
        .unwrap_or_else(|| "unknown".to_string());
    let payload = info
        .payload()
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| info.payload().downcast_ref::<String>().map(String::as_str))
        .unwrap_or("non-string panic payload");
    let report = format!(
        "ZeroCAD {} ({})\npanic at {location}: {payload}\n\n{}\n",
        env!("CARGO_PKG_VERSION"),
        env!("ZEROCAD_GIT_HASH"),
        Backtrace::force_capture()
    );
    let _ = std::fs::write(root.join(format!("{base}.txt")), report);
}

pub(crate) fn session_log_path() -> Option<PathBuf> {
    let path = crate::settings::config_dir()?.join("session.log");
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    Some(path)
}

pub(crate) fn latest_panic_report() -> Vec<u8> {
    let Some(root) = crate::settings::config_dir().map(|dir| dir.join("recovery")) else {
        return Vec::new();
    };
    std::fs::read_dir(root)
        .ok()
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            let name = path.file_name()?.to_str()?;
            if !name.starts_with("panic-recovery-") || path.extension()?.to_str()? != "txt" {
                return None;
            }
            Some((entry.metadata().ok()?.modified().ok()?, path))
        })
        .max_by_key(|(modified, _)| *modified)
        .and_then(|(_, path)| std::fs::read(path).ok())
        .unwrap_or_default()
}

pub(crate) struct SessionLogWriter {
    file: Option<std::fs::File>,
}

impl SessionLogWriter {
    pub(crate) fn new() -> Self {
        let file = session_log_path().and_then(|path| {
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .ok()
        });
        Self { file }
    }
}

impl Write for SessionLogWriter {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        let _ = std::io::stderr().write_all(bytes);
        if let Some(file) = &mut self.file {
            file.write_all(bytes)?;
            file.flush()?;
        }
        Ok(bytes.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let _ = std::io::stderr().flush();
        if let Some(file) = &mut self.file {
            file.flush()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_round_trip_uses_canonical_document_writer() {
        let root = std::env::temp_dir().join(format!(
            "zerocad-phase7-recovery-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        std::fs::create_dir_all(&root).unwrap();
        let document = ProjectDocument::Part(zerocad_core::Document::new());
        let path = root.join(AUTOSAVE_FILE);
        write_recovery_document(&path, &document).unwrap();
        let restored = read_project_document_file(&path, &LoadOptions::default()).unwrap();
        let ProjectDocument::Part(restored) = restored.document else {
            panic!("part recovery changed project kind");
        };
        restored.validate_semantic_contracts().unwrap();
        let ProjectDocument::Part(document) = document else {
            unreachable!()
        };
        assert_eq!(restored.graph.node_count(), document.graph.node_count());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn recovery_preserves_assembly_project_kind() {
        let root = std::env::temp_dir().join(format!(
            "zerocad-assembly-recovery-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        std::fs::create_dir_all(&root).unwrap();
        let document = ProjectDocument::Assembly(zerocad_core::AssemblyDocument::new());
        let path = root.join(AUTOSAVE_FILE);
        write_recovery_document(&path, &document).unwrap();
        let restored = read_project_document_file(&path, &LoadOptions::default()).unwrap();
        assert!(matches!(restored.document, ProjectDocument::Assembly(_)));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn background_recovery_write_and_saved_cleanup_are_ordered() {
        let root = std::env::temp_dir().join(format!(
            "zerocad-phase7-recovery-worker-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let mut manager = RecoveryManager::new_in(Some(root.clone()));
        let document = ProjectDocument::Part(zerocad_core::Document::new());
        manager
            .tx
            .as_ref()
            .unwrap()
            .send(RecoveryCommand::Save(Box::new(document.clone())))
            .unwrap();
        let path = root.join(AUTOSAVE_FILE);
        for _ in 0..100 {
            if path.exists() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(path.exists(), "background autosave was not written");
        manager.mark_saved(&document);
        for _ in 0..100 {
            if !path.exists() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!path.exists(), "saved document must clear stale recovery");
        let _ = std::fs::remove_dir_all(root);
    }
}
