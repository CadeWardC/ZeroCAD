//! Fully offline, user-initiated alpha bug-report bundle export.

use crate::*;
use std::collections::BTreeMap;
use std::path::Path;

pub(crate) const APP_VERSION: &str = env!("CARGO_PKG_VERSION");
pub(crate) const GIT_HASH: &str = env!("ZEROCAD_GIT_HASH");

impl ZeroCadApp {
    pub(crate) fn export_bug_report(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export ZeroCAD Bug Report")
            .set_file_name("zerocad-bug-report.zip")
            .add_filter("ZIP archive", &["zip"])
            .save_file()
        else {
            return;
        };
        let document = self.current_document_snapshot();
        let adapter = self
            .gpu
            .adapter_info()
            .unwrap_or_else(|| "CPU renderer / adapter unavailable".to_string());
        let last_error = self.error_msg.clone().unwrap_or_default();
        let unresolved_features = self
            .unresolved_features
            .iter()
            .map(|(feature_id, reason)| (feature_id.clone(), reason.clone()))
            .collect::<BTreeMap<_, _>>();
        match write_bug_report(
            &path,
            &document,
            &adapter,
            &last_error,
            &unresolved_features,
        ) {
            Ok(()) => {
                self.status_msg = format!("Bug report exported to {}", path.display());
            }
            Err(error) => {
                self.status_msg = format!("Bug report export failed: {error}");
                self.error_msg = Some(self.status_msg.clone());
            }
        }
    }

    pub(crate) fn draw_about_window(&mut self, ctx: &egui::Context) {
        if !self.show_about {
            return;
        }
        let mut open = self.show_about;
        egui::Window::new("About ZeroCAD")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.heading("ZeroCAD Part Design");
                ui.label(format!("Version {APP_VERSION}"));
                ui.label(format!("Build {GIT_HASH}"));
                ui.label(format!(
                    "Platform {} / {}",
                    std::env::consts::OS,
                    std::env::consts::ARCH
                ));
                if let Some(adapter) = self.gpu.adapter_info() {
                    ui.label(format!("Renderer: {adapter}"));
                }
                ui.add_space(6.0);
                ui.weak("Offline by design. Bug reports are exported only when you request one.");
            });
        self.show_about = open;
    }
}

fn write_bug_report(
    path: &Path,
    document: &Document,
    adapter: &str,
    last_error: &str,
    unresolved_features: &BTreeMap<String, String>,
) -> Result<(), String> {
    let document_bytes = zerocad_core::write_document_to_vec(
        document,
        &zerocad_core::SaveOptions::default(),
        &zerocad_core::HydrationBundle::default(),
    )
    .map_err(|error| error.to_string())?;
    let manifest = build_manifest(adapter, last_error, unresolved_features)?;
    let session_log = crate::recovery::session_log_path()
        .and_then(|path| std::fs::read(path).ok())
        .unwrap_or_default();
    let panic_report = crate::recovery::latest_panic_report();
    let archive = stored_zip(&[
        ("manifest.json", manifest.as_slice()),
        ("document.zcad", document_bytes.as_slice()),
        ("session.log", session_log.as_slice()),
        ("last-panic.txt", panic_report.as_slice()),
    ])?;
    std::fs::write(path, archive).map_err(|error| error.to_string())
}

fn build_manifest(
    adapter: &str,
    last_error: &str,
    unresolved_features: &BTreeMap<String, String>,
) -> Result<Vec<u8>, String> {
    serde_json::to_vec_pretty(&serde_json::json!({
        "schema": 1,
        "app_version": APP_VERSION,
        "git_hash": GIT_HASH,
        "os": std::env::consts::OS,
        "arch": std::env::consts::ARCH,
        "adapter": adapter,
        "evaluator_diagnostics": {
            "last_error": last_error,
            "unresolved_features": unresolved_features,
        },
        "user_initiated": true,
        "network_upload": false
    }))
    .map_err(|error| error.to_string())
}

fn stored_zip(entries: &[(&str, &[u8])]) -> Result<Vec<u8>, String> {
    let mut output = Vec::new();
    let mut central = Vec::new();
    for (name, data) in entries {
        let name = name.as_bytes();
        let name_len = u16::try_from(name.len()).map_err(|_| "ZIP entry name is too long")?;
        let size = u32::try_from(data.len()).map_err(|_| "ZIP entry is too large")?;
        let offset = u32::try_from(output.len()).map_err(|_| "ZIP archive is too large")?;
        let crc = crc32(data);
        push_u32(&mut output, 0x0403_4b50);
        push_u16(&mut output, 20);
        push_u16(&mut output, 0x0800);
        push_u16(&mut output, 0);
        push_u16(&mut output, 0);
        push_u16(&mut output, 0);
        push_u32(&mut output, crc);
        push_u32(&mut output, size);
        push_u32(&mut output, size);
        push_u16(&mut output, name_len);
        push_u16(&mut output, 0);
        output.extend_from_slice(name);
        output.extend_from_slice(data);

        push_u32(&mut central, 0x0201_4b50);
        push_u16(&mut central, 20);
        push_u16(&mut central, 20);
        push_u16(&mut central, 0x0800);
        push_u16(&mut central, 0);
        push_u16(&mut central, 0);
        push_u16(&mut central, 0);
        push_u32(&mut central, crc);
        push_u32(&mut central, size);
        push_u32(&mut central, size);
        push_u16(&mut central, name_len);
        push_u16(&mut central, 0);
        push_u16(&mut central, 0);
        push_u16(&mut central, 0);
        push_u16(&mut central, 0);
        push_u32(&mut central, 0);
        push_u32(&mut central, offset);
        central.extend_from_slice(name);
    }
    let central_offset = u32::try_from(output.len()).map_err(|_| "ZIP archive is too large")?;
    let central_size = u32::try_from(central.len()).map_err(|_| "ZIP archive is too large")?;
    output.extend_from_slice(&central);
    let count = u16::try_from(entries.len()).map_err(|_| "too many ZIP entries")?;
    push_u32(&mut output, 0x0605_4b50);
    push_u16(&mut output, 0);
    push_u16(&mut output, 0);
    push_u16(&mut output, count);
    push_u16(&mut output, count);
    push_u32(&mut output, central_size);
    push_u32(&mut output, central_offset);
    push_u16(&mut output, 0);
    Ok(output)
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}

fn push_u16(output: &mut Vec<u8>, value: u16) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn push_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offline_bundle_is_a_deterministic_stored_zip() {
        let first = stored_zip(&[("manifest.json", b"{}"), ("document.zcad", b"ZCAD")]).unwrap();
        let second = stored_zip(&[("manifest.json", b"{}"), ("document.zcad", b"ZCAD")]).unwrap();
        assert_eq!(first, second);
        assert_eq!(&first[..4], b"PK\x03\x04");
        assert!(first.windows(13).any(|window| window == b"manifest.json"));
        assert!(first.windows(13).any(|window| window == b"document.zcad"));
        assert_eq!(&first[first.len() - 22..first.len() - 18], b"PK\x05\x06");
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn manifest_contains_structured_unresolved_feature_diagnostics() {
        let unresolved = BTreeMap::from([
            ("feature-b".to_string(), "missing edge".to_string()),
            ("feature-a".to_string(), "boolean failed".to_string()),
        ]);
        let bytes = build_manifest("test adapter", "last evaluator error", &unresolved).unwrap();
        let manifest: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        let diagnostics = &manifest["evaluator_diagnostics"];
        assert_eq!(diagnostics["last_error"], "last evaluator error");
        assert_eq!(
            diagnostics["unresolved_features"]["feature-a"],
            "boolean failed"
        );
        assert_eq!(
            diagnostics["unresolved_features"]["feature-b"],
            "missing edge"
        );
    }
}
