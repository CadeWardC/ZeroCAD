use crate::SharedBodyMeshes;
use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::mpsc;
use zerocad_core::{EvaluationCacheSnapshot, ParametricGraph, Unit, ZcadDocument};

pub(crate) struct SaveRequest {
    pub path: PathBuf,
    pub graph: ParametricGraph,
    pub bodies: SharedBodyMeshes,
    pub embed_hydrated: bool,
    pub units: Unit,
    pub created_unix: Option<u64>,
    pub hidden_nodes: HashSet<String>,
    pub cache: EvaluationCacheSnapshot,
    pub hydrated_cache_limit: usize,
}

pub(crate) struct SaveCompletion {
    pub path: PathBuf,
    pub result: Result<(), String>,
    pub embed_hydrated: bool,
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
                    let embed_hydrated = request.embed_hydrated;
                    let result = save(request).map_err(|error| error.to_string());
                    let _ = completed.send(SaveCompletion {
                        path,
                        result,
                        embed_hydrated,
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
    let thumbnail_png = if request.bodies.is_empty() {
        None
    } else {
        let (w, h, rgba) = crate::thumbnail::render_thumbnail(&request.bodies, 256);
        crate::settings::save_thumb(&request.path, w, h, &rgba);
        crate::thumbnail::encode_png(w, h, &rgba)
    };
    let doc = ZcadDocument {
        graph: &request.graph,
        thumbnail_png,
        mesh_cache: request.embed_hydrated.then_some(request.bodies.as_slice()),
        units: request.units,
        bbox: bodies_bbox(&request.bodies),
        created_unix: request.created_unix,
        hidden_nodes: request.hidden_nodes,
        evaluation_cache: request.embed_hydrated.then_some(&request.cache),
        hydrated_cache_limit: Some(request.hydrated_cache_limit),
    };
    zerocad_core::write_zcad_file(&request.path, &doc)
}

fn bodies_bbox(bodies: &[(String, zerocad_core::MockMesh)]) -> [f32; 6] {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for (_, mesh) in bodies {
        for vertex in mesh.vertices.chunks_exact(6) {
            for axis in 0..3 {
                min[axis] = min[axis].min(vertex[axis]);
                max[axis] = max[axis].max(vertex[axis]);
            }
        }
    }
    if min[0].is_finite() {
        [min[0], min[1], min[2], max[0], max[1], max[2]]
    } else {
        [0.0; 6]
    }
}
