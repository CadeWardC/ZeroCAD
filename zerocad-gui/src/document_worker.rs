use crate::SharedBodyMeshes;
use std::path::PathBuf;
use std::sync::mpsc;
use zerocad_core::{Document, EvaluationCacheSnapshot, HydrationBundle, SaveOptions, SaveProfile};

pub(crate) struct SaveRequest {
    pub path: PathBuf,
    pub document: Document,
    pub bodies: SharedBodyMeshes,
    pub profile: SaveProfile,
    pub cache: EvaluationCacheSnapshot,
}

pub(crate) struct SaveCompletion {
    pub path: PathBuf,
    pub result: Result<(), String>,
    pub profile: SaveProfile,
}

pub(crate) struct DocumentWorker {
    tx: mpsc::Sender<SaveRequest>,
    rx: mpsc::Receiver<SaveCompletion>,
}

impl DocumentWorker {
    pub(crate) fn new() -> Self {
        let (tx, requests) = mpsc::channel::<SaveRequest>();
        let (completed, rx) = mpsc::channel();
        std::thread::Builder::new()
            .name("zerocad-document-writer".into())
            .spawn(move || {
                while let Ok(request) = requests.recv() {
                    let started = std::time::Instant::now();
                    let path = request.path.clone();
                    let profile = request.profile;
                    let result = save(request).map_err(|error| error.to_string());
                    let _ = completed.send(SaveCompletion {
                        path,
                        result,
                        profile,
                    });
                    log::debug!("document save worker: {:?}", started.elapsed());
                }
            })
            .expect("failed to start document worker");
        Self { tx, rx }
    }

    pub(crate) fn submit(&self, request: SaveRequest) {
        let _ = self.tx.send(request);
    }

    pub(crate) fn try_recv(&self) -> Option<SaveCompletion> {
        self.rx.try_recv().ok()
    }
}

fn save(request: SaveRequest) -> Result<(), zerocad_core::ZcadError> {
    let small_preview_png = preview_with_cap(&request.bodies, 128, 96, 32 * 1024);
    if let Some((w, h, rgba)) = (!request.bodies.is_empty())
        .then(|| crate::thumbnail::render_thumbnail(&request.bodies, 128))
    {
        crate::settings::save_thumb(&request.path, w, h, &rgba);
    }
    let large_preview_png = matches!(request.profile, SaveProfile::Hydrated { .. })
        .then(|| crate::thumbnail::render_thumbnail(&request.bodies, 256))
        .and_then(|(w, h, rgba)| crate::thumbnail::encode_png(w, h, &rgba));
    let accelerators = HydrationBundle {
        small_preview_png,
        large_preview_png,
        display_meshes: Some(request.bodies.as_ref().clone()),
        evaluation_cache: Some(request.cache),
    };
    zerocad_core::write_document_file(
        &request.path,
        &request.document,
        &SaveOptions {
            profile: request.profile,
        },
        &accelerators,
    )
}

fn preview_with_cap(
    bodies: &[(String, zerocad_core::MockMesh)],
    preferred: usize,
    fallback: usize,
    cap: usize,
) -> Option<Vec<u8>> {
    if bodies.is_empty() {
        return None;
    }
    for size in [preferred, fallback] {
        let (w, h, rgba) = crate::thumbnail::render_thumbnail(bodies, size);
        if let Some(png) = crate::thumbnail::encode_png(w, h, &rgba) {
            if png.len() <= cap {
                return Some(png);
            }
        }
    }
    None
}
