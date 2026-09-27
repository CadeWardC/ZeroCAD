use eframe::egui;
use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use zerocad_core::{
    Document, EvaluationCancellation, EvaluationError, EvaluationOutput, EvaluationQuality,
};

#[cfg(not(panic = "unwind"))]
compile_error!("the model evaluator requires panic=unwind for request containment");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EvaluationPurpose {
    CommittedModel,
    ExtrudePreview(u64),
    EdgeModPreview(u64),
    #[cfg(test)]
    PanicProbe,
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
#[derive(Default)]
struct Mailbox {
    pending: Option<EvaluationRequest>,
    completion: Option<EvaluationCompletion>,
    stopped: bool,
}

pub(crate) struct ModelEvaluator {
    mailbox: Arc<(Mutex<Mailbox>, Condvar)>,
    latest_generation: Arc<AtomicU64>,
}

impl ModelEvaluator {
    pub(crate) fn new() -> Self {
        // One running request, one replaceable pending snapshot, and one
        // replaceable completion. No document/mesh backlog during a slow solve.
        let mailbox = Arc::new((Mutex::new(Mailbox::default()), Condvar::new()));
        let worker_mailbox = mailbox.clone();
        let latest_generation = Arc::new(AtomicU64::new(0));
        let worker_generation = latest_generation.clone();

        std::thread::Builder::new()
            .name("zerocad-model-evaluator".into())
            .spawn(move || {
                let mut force_cold_after_panic = false;
                loop {
                    let request = {
                        let (lock, wake) = &*worker_mailbox;
                        let mut slot = lock.lock().unwrap();
                        while slot.pending.is_none() && !slot.stopped {
                            slot = wake.wait(slot).unwrap();
                        }
                        if slot.stopped { return; }
                        slot.pending.take().unwrap()
                    };
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
                    let document = if force_cold_after_panic {
                        request.document.clone_authoritative()
                    } else {
                        request.document
                    };
                    let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        #[cfg(test)]
                        if request.purpose == EvaluationPurpose::PanicProbe {
                            panic!("injected evaluator panic");
                        }
                        document.evaluate_request(
                            &request.hidden,
                            request.quality,
                            &cancellation,
                        )
                    }));
                    let result = match caught {
                        Ok(result) => {
                            if result.is_ok() {
                                force_cold_after_panic = false;
                            }
                            result
                        }
                        Err(payload) => {
                            force_cold_after_panic = true;
                            zerocad_core::mock_kernel::reset_evaluator_thread_state();
                            let reason = payload
                                .downcast_ref::<&str>()
                                .map(|message| (*message).to_string())
                                .or_else(|| payload.downcast_ref::<String>().cloned())
                                .unwrap_or_else(|| "unknown kernel panic".to_string());
                            Err(EvaluationError::Failed(format!(
                                "model evaluation panicked and was safely discarded: {reason}"
                            )))
                        }
                    };
                    // A panicked request's graph and any newly published local
                    // Arc cache must be gone before the next request is read.
                    drop(document);
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
                        let mut slot = worker_mailbox.0.lock().unwrap();
                        if worker_generation.load(Ordering::Acquire) == request.generation && !slot.stopped {
                            slot.completion = Some(EvaluationCompletion {
                                generation: request.generation,
                                purpose: request.purpose,
                                result,
                            });
                        }
                    }
                    if let Some(ctx) = request.repaint {
                        ctx.request_repaint();
                    }
                }
            })
            .expect("failed to start model evaluator thread");

        Self {
            mailbox,
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
        let mut slot = self.mailbox.0.lock().unwrap();
        slot.completion = None;
        slot.pending = Some(EvaluationRequest {
            generation,
            purpose,
            document,
            hidden,
            quality,
            repaint,
        });
        self.mailbox.1.notify_one();
        generation
    }

    pub(crate) fn cancel(&self) {
        self.latest_generation.fetch_add(1, Ordering::AcqRel);
        let mut slot = self.mailbox.0.lock().unwrap();
        slot.pending = None;
        slot.completion = None;
    }

    pub(crate) fn current_generation(&self) -> u64 {
        self.latest_generation.load(Ordering::Acquire)
    }

    pub(crate) fn try_recv(&self) -> Option<EvaluationCompletion> {
        self.mailbox.0.lock().unwrap().completion.take()
    }
}

impl Drop for ModelEvaluator {
    fn drop(&mut self) {
        self.cancel();
        self.mailbox.0.lock().unwrap().stopped = true;
        self.mailbox.1.notify_one();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn request_bursts_keep_only_the_latest_document_and_completion() {
        let evaluator = ModelEvaluator::new();
        let mut latest = 0;
        for _ in 0..100 {
            latest = evaluator.submit(
                EvaluationPurpose::CommittedModel,
                Document::new(),
                HashSet::new(),
                EvaluationQuality::Final,
                None,
            );
            let slot = evaluator.mailbox.0.lock().unwrap();
            if let Some(request) = &slot.pending {
                assert_eq!(request.generation, latest);
            }
        }
        assert!(wait_for(&evaluator, latest).result.is_ok());
        evaluator.cancel();
        let slot = evaluator.mailbox.0.lock().unwrap();
        assert!(slot.pending.is_none() && slot.completion.is_none());
    }

    fn wait_for(evaluator: &ModelEvaluator, generation: u64) -> EvaluationCompletion {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(completion) = evaluator.try_recv() {
                if completion.generation == generation {
                    return completion;
                }
            }
            assert!(Instant::now() < deadline, "evaluation worker timed out");
            std::thread::yield_now();
        }
    }

    #[test]
    fn caught_panic_is_reported_and_next_request_succeeds_cold() {
        let evaluator = ModelEvaluator::new();
        let panic_generation = evaluator.submit(
            EvaluationPurpose::PanicProbe,
            Document::new(),
            HashSet::new(),
            EvaluationQuality::Final,
            None,
        );
        let panic_completion = wait_for(&evaluator, panic_generation);
        assert!(matches!(
            panic_completion.result,
            Err(EvaluationError::Failed(message)) if message.contains("safely discarded")
        ));

        let success_generation = evaluator.submit(
            EvaluationPurpose::CommittedModel,
            Document::new(),
            HashSet::new(),
            EvaluationQuality::Final,
            None,
        );
        let success = wait_for(&evaluator, success_generation)
            .result
            .expect("worker must service the next request");
        assert!(success.diagnostics.is_empty());
        assert!(success.trace.reused_checkpoints.is_empty());
    }
}
