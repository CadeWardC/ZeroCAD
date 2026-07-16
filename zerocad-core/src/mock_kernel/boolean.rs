use super::*;

thread_local! {
    static ACTIVE_CANCELLATION: std::cell::RefCell<Option<crate::EvaluationCancellation>> =
        const { std::cell::RefCell::new(None) };
}

pub(crate) fn with_kernel_cancellation<R>(
    cancellation: crate::EvaluationCancellation,
    f: impl FnOnce() -> R,
) -> R {
    ACTIVE_CANCELLATION.with(|slot| {
        let previous = slot.replace(Some(cancellation));
        let result = f();
        slot.replace(previous);
        result
    })
}

fn active_cancellation() -> Option<crate::EvaluationCancellation> {
    ACTIVE_CANCELLATION.with(|slot| slot.borrow().clone())
}

fn checked_boolean(a: &KernelSolid, b: &KernelSolid, op: BooleanOp) -> Result<KernelSolid, String> {
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let outcome = match active_cancellation() {
        Some(cancel) => consume_operation(
            boolean_operation_name(op),
            openrcad::algo::boolean_operation_with_policy_and_cancel(a, b, op, &policy, &cancel),
        ),
        None => consume_operation(
            boolean_operation_name(op),
            openrcad::algo::boolean_operation_with_policy(a, b, op, &policy),
        ),
    }?;
    Ok(outcome.solid)
}

fn boolean_operation_name(op: BooleanOp) -> &'static str {
    match op {
        BooleanOp::Fuse => "boolean fuse",
        BooleanOp::Cut => "boolean cut",
        BooleanOp::Common => "boolean common",
    }
}

/// Keep the recoverable-boolean call boundary explicit without changing the
/// process-wide panic hook. `boolean_checked` catches solver panics itself.
pub(crate) fn quiet_panic<R>(f: impl FnOnce() -> R) -> R {
    // `boolean_checked` already catches solver panics. Swapping the process-wide
    // panic hook here was racy once evaluations moved to background threads: an
    // unrelated panic could be silenced, or two booleans could restore hooks in
    // the wrong order. Keep recovery local and never mutate global panic state.
    f()
}

/// Boolean union (`a ∪ b`). Returns `None` if the kernel can't resolve the
/// configuration, panics, or produces a non-watertight result — callers decide
/// how to degrade. `boolean_checked` catches panics and rejects leaky output.
pub fn union(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    union_diagnostic(a, b).ok()
}

/// Boolean union that preserves the kernel's failure reason for operation-level
/// diagnostics. Interactive callers normally use [`union`]; feature evaluators
/// use this when they can identify the affected node in a useful log message.
pub(crate) fn union_diagnostic(a: &KernelSolid, b: &KernelSolid) -> Result<KernelSolid, String> {
    quiet_panic(|| checked_boolean(a, b, BooleanOp::Fuse))
}

/// Boolean difference (`a − b`): subtract `b`'s volume from `a`. Returns `None`
/// on kernel failure or non-watertight output.
pub fn difference(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    quiet_panic(|| checked_boolean(a, b, BooleanOp::Cut).ok())
}

/// Boolean union with the kernel's exact face history (see
/// [`openrcad::algo::BooleanFaceHistory`]). `obj_classes` are per-shell-face
/// owner classes of `a` — coplanar merge respects them (owner-aware merge).
pub fn union_with_history(
    a: &KernelSolid,
    b: &KernelSolid,
    obj_classes: Option<&[Option<u64>]>,
) -> Option<(KernelSolid, openrcad::algo::BooleanFaceHistory)> {
    union_with_history_diagnostic(a, b, obj_classes).ok()
}

pub(crate) fn union_with_history_diagnostic(
    a: &KernelSolid,
    b: &KernelSolid,
    obj_classes: Option<&[Option<u64>]>,
) -> Result<(KernelSolid, openrcad::algo::BooleanFaceHistory), String> {
    quiet_panic(|| {
        let policy = openrcad::foundation::TolerancePolicy::STANDARD;
        let never = openrcad::foundation::NeverCancelled;
        let active = active_cancellation();
        let cancel: &dyn openrcad::foundation::CancellationProbe =
            active.as_ref().map_or(&never, |probe| probe);
        let outcome = consume_operation(
            "boolean fuse",
            openrcad::algo::boolean_operation_with_classes_policy_and_cancel(
                a,
                b,
                BooleanOp::Fuse,
                obj_classes,
                None,
                &policy,
                cancel,
            ),
        )?;
        let history = outcome.boolean_face_history();
        Ok((outcome.solid, history))
    })
}

/// [`difference_bodies`] plus exact face history local to each connected body.
#[must_use]
pub(crate) struct DifferenceBodiesOutcome {
    pub bodies: Vec<KernelSolid>,
    pub face_history: Vec<openrcad::algo::BooleanFaceHistory>,
}

/// Connected Common results with exact face history local to each body.
#[must_use]
pub(crate) struct CommonBodiesOutcome {
    pub bodies: Vec<KernelSolid>,
    pub face_history: Vec<openrcad::algo::BooleanFaceHistory>,
}

pub(crate) enum CommonBodiesError {
    Empty,
    Failed(String),
}

pub(crate) fn common_bodies_with_history(
    a: &KernelSolid,
    b: &KernelSolid,
    obj_classes: Option<&[Option<u64>]>,
) -> Result<CommonBodiesOutcome, CommonBodiesError> {
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let never = openrcad::foundation::NeverCancelled;
    let active = active_cancellation();
    let cancel: &dyn openrcad::foundation::CancellationProbe =
        active.as_ref().map_or(&never, |probe| probe);
    let outcome = quiet_panic(|| {
        let operation = openrcad::algo::boolean_bodies_operation_with_classes_policy_and_cancel(
            a,
            b,
            BooleanOp::Common,
            obj_classes,
            None,
            &policy,
            cancel,
        );
        match operation {
            Err(openrcad::algo::BooleanError::EmptyOutput) => Err(CommonBodiesError::Empty),
            other => consume_boolean_bodies_operation("boolean common", other)
                .map_err(CommonBodiesError::Failed),
        }
    })?;
    let mut paired: Vec<_> = outcome
        .bodies
        .into_iter()
        .zip(outcome.face_history)
        .collect();
    paired.sort_by_key(|(solid, _)| part_key(solid));
    let (bodies, face_history) = paired.into_iter().unzip();
    Ok(CommonBodiesOutcome {
        bodies,
        face_history,
    })
}

pub(crate) fn difference_bodies_with_history(
    a: &KernelSolid,
    b: &KernelSolid,
    obj_classes: Option<&[Option<u64>]>,
) -> Option<DifferenceBodiesOutcome> {
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let never = openrcad::foundation::NeverCancelled;
    let active = active_cancellation();
    let cancel: &dyn openrcad::foundation::CancellationProbe =
        active.as_ref().map_or(&never, |probe| probe);
    let outcome = quiet_panic(|| {
        consume_boolean_bodies_operation(
            "boolean cut",
            openrcad::algo::boolean_bodies_operation_with_classes_policy_and_cancel(
                a,
                b,
                BooleanOp::Cut,
                obj_classes,
                None,
                &policy,
                cancel,
            ),
        )
        .ok()
    })?;
    let mut paired: Vec<_> = outcome
        .bodies
        .into_iter()
        .zip(outcome.face_history)
        .collect();
    paired.sort_by_key(|(solid, _)| part_key(solid));
    let (bodies, face_history) = paired.into_iter().unzip();
    Some(DifferenceBodiesOutcome {
        bodies,
        face_history,
    })
}

/// Boolean difference (`a − b`) that returns **one solid per connected
/// component** instead of a single shell.
///
/// A cut that *severs* `a` — e.g. a slot sliced clean through a bar, leaving two
/// separate lumps — comes back from the kernel as one watertight shell holding
/// both disjoint pieces (a valid B-Rep, but really two bodies). `difference`
/// hands that back as a single `KernelSolid`; the parametric evaluator instead
/// wants each lump as its own selectable body part. This runs the same guarded
/// cut, then splits the result into its connected components via
/// [`Solid::split_disconnected`].
///
/// Returns `None` on kernel failure or non-watertight output (the caller keeps
/// the part intact rather than dropping material). On success the vector always
/// has at least one element — an un-severed cut yields a single body.
pub fn difference_bodies(a: &KernelSolid, b: &KernelSolid) -> Option<Vec<KernelSolid>> {
    difference_bodies_with_history(a, b, None).map(|outcome| outcome.bodies)
}
