use eframe::egui;
use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use zerocad_core::{
    Document, EvaluationCancellation, EvaluationError, EvaluationOutput, EvaluationQuality,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EvaluationPurpose {
    CommittedModel,
    ExtrudePreview(u64),
    EdgeModPreview(u64),
}

struct EvaluationRequest {
    generation: u64,
    purpose: EvaluationPurpose,
    document: Document,
    hidden: HashSet<String>,
    quality: EvaluationQuality,
    repaint: Option<egui::Context>,
}

pub(crate) struct EvaluationCompletion {
    pub(crate) generation: u64,
    pub(crate) purpose: EvaluationPurpose,
    pub(crate) result: Result<EvaluationOutput, EvaluationError>,
}

/// One persistent, latest-wins geometry worker for the whole application.
pub(crate) struct ModelEvaluator {
    request_tx: mpsc::Sender<EvaluationRequest>,
    completion_rx: mpsc::Receiver<EvaluationCompletion>,
    latest_generation: Arc<AtomicU64>,
}

impl ModelEvaluator {
    pub(crate) fn new() -> Self {
        let (request_tx, request_rx) = mpsc::channel::<EvaluationRequest>();
        let (completion_tx, completion_rx) = mpsc::channel::<EvaluationCompletion>();
        let latest_generation = Arc::new(AtomicU64::new(0));
        let worker_generation = latest_generation.clone();

        std::thread::Builder::new()
            .name("zerocad-model-evaluator".into())
            .spawn(move || {
                while let Ok(mut request) = request_rx.recv() {
                    // Collapse bursts before doing geometry. Only the newest
                    // request can ever become visible.
                    while let Ok(newer) = request_rx.try_recv() {
                        request = newer;
                    }
                    if worker_generation.load(Ordering::Acquire) != request.generation {
                        log::trace!(
                            "[evaluation] skipped superseded request generation={} purpose={:?}",
                            request.generation,
                            request.purpose
                        );
                        continue;
                    }

                    let started = std::time::Instant::now();
                    if request.purpose == EvaluationPurpose::CommittedModel {
                        log::debug!(
                            "[evaluation] starting generation={} purpose={:?} quality={:?} hidden_nodes={}",
                            request.generation,
                            request.purpose,
                            request.quality,
                            request.hidden.len()
                        );
                    }
                    let cancellation =
                        EvaluationCancellation::new(request.generation, worker_generation.clone());
                    let result = request.document.evaluate_request(
                        &request.hidden,
                        request.quality,
                        &cancellation,
                    );
                    if request.purpose == EvaluationPurpose::CommittedModel {
                        match &result {
                            Ok(output) => log::debug!(
                                "[evaluation] completed generation={} elapsed={:?} bodies={:?} warnings={}",
                                request.generation,
                                started.elapsed(),
                                output.bodies.iter().map(|(id, _)| id.as_str()).collect::<Vec<_>>(),
                                output
                                    .diagnostics
                                    .iter()
                                    .filter(|diagnostic| {
                                        diagnostic.severity
                                            != zerocad_core::DiagnosticSeverity::Info
                                    })
                                    .count()
                            ),
                            Err(error) => log::warn!(
                                "[evaluation] failed generation={} elapsed={:?}: {error}",
                                request.generation,
                                started.elapsed()
                            ),
                        }
                    }
                    if !matches!(result, Err(EvaluationError::Cancelled)) {
                        let _ = completion_tx.send(EvaluationCompletion {
                            generation: request.generation,
                            purpose: request.purpose,
                            result,
                        });
                    }
                    if let Some(ctx) = request.repaint {
                        ctx.request_repaint();
                    }
                }
            })
            .expect("failed to start model evaluator thread");

        Self {
            request_tx,
            completion_rx,
            latest_generation,
        }
    }

    pub(crate) fn submit(
        &self,
        purpose: EvaluationPurpose,
        document: Document,
        hidden: HashSet<String>,
        quality: EvaluationQuality,
        repaint: Option<egui::Context>,
    ) -> u64 {
        let generation = self.latest_generation.fetch_add(1, Ordering::AcqRel) + 1;
        if purpose == EvaluationPurpose::CommittedModel {
            log::debug!(
                "[evaluation] submitted generation={generation} purpose={purpose:?} quality={quality:?} hidden_nodes={}",
                hidden.len()
            );
        }
        let _ = self.request_tx.send(EvaluationRequest {
            generation,
            purpose,
            document,
            hidden,
            quality,
            repaint,
        });
        generation
    }

    pub(crate) fn cancel(&self) {
        self.latest_generation.fetch_add(1, Ordering::AcqRel);
    }

    pub(crate) fn current_generation(&self) -> u64 {
        self.latest_generation.load(Ordering::Acquire)
    }

    pub(crate) fn try_recv(&self) -> Option<EvaluationCompletion> {
        self.completion_rx.try_recv().ok()
    }
}
