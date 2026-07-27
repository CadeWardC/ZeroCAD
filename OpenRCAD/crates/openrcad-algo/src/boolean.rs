use crate::BooleanOp;
use core::f64::consts::TAU;
use openrcad_foundation::{
    CancellationProbe, NeverCancelled, ToleranceContext, TolerancePolicy, TolerancePolicyError,
};
use openrcad_foundation::{Dir, Pnt, Trsf, Vec as GeomVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::arena::EdgeId;
use openrcad_topo::{
    BRepBuilder, Face, FaceId, HealthReport, InputTopologyRef, PcurveBuildError, Solid,
    TopologyHistory, TopologyHistoryError, TopologyRef, Wire,
};
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::bvh::Bvh;
use crate::operation::{
    Diagnostic, OperationResult, RecoveryAction, RecoveryReport, ValidationReport,
};

/// Which input face a boolean-result face came from. `usize` is the face's
/// position in the operand's `shell().faces()` — the one face key that is
/// stable across the kernel boundary (tessellation assigns mesh face ids by
/// the same enumeration).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BooleanFaceSource {
    /// Position of the source face in the object's shell.
    Object(usize),
    /// Position of the source face in the tool's shell.
    Tool(usize),
}

/// Exact face correspondence emitted by the boolean pipeline itself (OCCT
/// `Modified`/`Generated` semantics, resolved to final result faces):
/// `face_source[i]` names the input face that result face `i` (position in the
/// result's `shell().faces()`) descends from. An `Object`-sourced face is a
/// *Modified* survivor/split of that object face; in a Cut, a `Tool`-sourced
/// face is a wall *Generated* by that tool face. An input face that appears in
/// no entry was *Deleted*. `None` means the pipeline could not attribute the
/// face (the caller should fall back to geometric matching, never guess).
///
/// Unlike a post-hoc surface-signature matcher, this correspondence is derived
/// from the split bookkeeping the boolean already performs internally
/// (`obj_sub`/`tool_sub` parent→child maps), so two same-plane faces with
/// different owners resolve by their true imprint boundaries.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BooleanFaceHistory {
    pub face_source: Vec<Option<BooleanFaceSource>>,
}

/// Structured value returned by a boolean that may create several connected
/// solids.  Face history is indexed per returned body, so applications never
/// need to keep an invalid disconnected shell merely to interpret lineage.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBodies {
    pub bodies: Vec<Solid>,
    pub face_history: Vec<BooleanFaceHistory>,
}

impl BooleanFaceHistory {
    /// Source of result face at shell position `i`.
    pub fn source_of(&self, i: usize) -> Option<BooleanFaceSource> {
        self.face_source.get(i).copied().flatten()
    }

    /// Convert the legacy face correspondence into the kernel-wide topology
    /// history contract. Operand 0 is the object and operand 1 is the tool.
    pub fn topology_history(
        &self,
        op: BooleanOp,
        object_face_count: usize,
        tool_face_count: usize,
    ) -> TopologyHistory {
        let mut history = TopologyHistory::default();
        let mut descendants = [
            vec![Vec::<TopologyRef>::new(); object_face_count],
            vec![Vec::<TopologyRef>::new(); tool_face_count],
        ];

        for (result_index, source) in self.face_source.iter().enumerate() {
            let result = TopologyRef::face(result_index);
            match source {
                Some(BooleanFaceSource::Object(index)) if *index < object_face_count => {
                    descendants[0][*index].push(result);
                }
                Some(BooleanFaceSource::Tool(index)) if *index < tool_face_count => {
                    descendants[1][*index].push(result);
                }
                _ => history.generated([], result),
            }
        }

        for (operand, by_source) in descendants.iter().enumerate() {
            for (source_index, results) in by_source.iter().enumerate() {
                let source = InputTopologyRef::face(operand, source_index);
                if results.is_empty() {
                    history.deleted(source);
                } else if operand == 1 && op == BooleanOp::Cut {
                    for &result in results {
                        history.generated([source], result);
                    }
                } else if results.len() == 1 {
                    history.modified(source, results[0]);
                } else {
                    history.split(source, results.iter().copied());
                }
            }
        }

        let object_shell =
            InputTopologyRef::new(0, TopologyRef::new(openrcad_topo::TopologyKind::Shell, 0));
        let tool_shell =
            InputTopologyRef::new(1, TopologyRef::new(openrcad_topo::TopologyKind::Shell, 0));
        let result_shell = TopologyRef::new(openrcad_topo::TopologyKind::Shell, 0);
        let object_solid =
            InputTopologyRef::new(0, TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0));
        let tool_solid =
            InputTopologyRef::new(1, TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0));
        let result_solid = TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0);
        if op == BooleanOp::Cut {
            history.modified(object_shell, result_shell);
            history.deleted(tool_shell);
            history.modified(object_solid, result_solid);
            history.deleted(tool_solid);
        } else {
            history.merged([object_shell, tool_shell], result_shell);
            history.merged([object_solid, tool_solid], result_solid);
        }
        history
    }
}

/// Which operand failed preflight validation for a boolean operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanInput {
    /// The solid being operated on.
    Object,
    /// The tool solid.
    Tool,
}

/// Structured boolean failure for applications that need recoverable modeling.
#[derive(Clone, Debug, PartialEq)]
pub enum BooleanError {
    /// The supplied document tolerance policy is not usable.
    InvalidTolerancePolicy(TolerancePolicyError),
    /// A face-local pcurve could not be constructed consistently.
    PcurveBuild(PcurveBuildError),
    /// The caller superseded this operation. No partial result is returned.
    Cancelled,
    /// One of the input solids is structurally invalid or not watertight.
    InvalidInput {
        /// The invalid operand.
        input: BooleanInput,
        /// Topology health diagnostics for that operand.
        report: HealthReport,
    },
    /// The boolean implementation panicked before it could produce a result.
    Panicked,
    /// A successful geometry candidate carried malformed topology lineage.
    InvalidHistory(TopologyHistoryError),
    /// Result topology exists but the history omitted one or more entities.
    IncompleteHistory { missing: Vec<TopologyRef> },
    /// The operation produced no connected solid result.
    EmptyOutput,
    /// The resulting solid failed structural health checks.
    InvalidOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
    /// The resulting solid is structurally valid but has open/free boundary
    /// edges, so it is unsafe to treat as a closed CAD solid.
    NonWatertightOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
    /// The result is topologically watertight but contains a degenerate "sliver":
    /// two near-coincident, overlapping parallel faces forming a near-zero-
    /// thickness wall. This passes the manifold/health checks yet is a tiny
    /// geometric lie that breaks downstream operations (notably filleting), so it
    /// is rejected up front rather than cached as a bad body.
    DegenerateSliver {
        /// The wall thickness detected (distance between the coincident planes).
        thickness: f64,
    },
}

impl core::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "invalid boolean tolerance policy: {error}")
            }
            Self::PcurveBuild(error) => write!(f, "boolean pcurve construction failed: {error}"),
            Self::Cancelled => write!(f, "boolean operation cancelled"),
            Self::InvalidInput { input, report } => {
                write!(f, "invalid boolean {input:?} input: {report:?}")
            }
            Self::Panicked => write!(f, "boolean operation panicked"),
            Self::InvalidHistory(error) => write!(f, "invalid boolean topology history: {error}"),
            Self::IncompleteHistory { missing } => {
                write!(
                    f,
                    "boolean topology history omitted result entities: {missing:?}"
                )
            }
            Self::EmptyOutput => write!(f, "boolean produced no connected solid result"),
            Self::InvalidOutput { report } => {
                write!(f, "boolean produced invalid topology: {report:?}")
            }
            Self::NonWatertightOutput { report } => {
                write!(f, "boolean produced a non-watertight solid: {report:?}")
            }
            Self::DegenerateSliver { thickness } => {
                write!(
                    f,
                    "boolean produced a degenerate sliver (wall {thickness:.2e})"
                )
            }
        }
    }
}

impl std::error::Error for BooleanError {}

/// Apply `op` and reject invalid inputs, panics, or unhealthy/non-watertight
/// outputs. This is the preferred entry point for CAD applications, where a
/// failed feature should be diagnosable instead of cached as a bad body.
#[deprecated(note = "use boolean_operation; this wrapper discards operation metadata")]
pub fn boolean_checked(object: &Solid, tool: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    boolean_operation_with_policy(object, tool, op, &TolerancePolicy::STANDARD)
        .map(|result| result.value)
}

/// Checked boolean using one explicit document-wide tolerance policy for
/// intersection, reconstruction, sewing, recovery, and validation.
#[deprecated(note = "use boolean_operation_with_policy; this wrapper discards operation metadata")]
pub fn boolean_checked_with_policy(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    policy: &TolerancePolicy,
) -> Result<Solid, BooleanError> {
    boolean_operation_with_policy(object, tool, op, policy).map(|result| result.value)
}

/// Boolean operation using the shared structured result contract.
pub fn boolean_operation(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
) -> Result<OperationResult<Solid>, BooleanError> {
    boolean_operation_with_policy(object, tool, op, &TolerancePolicy::STANDARD)
}

/// Structured boolean operation under an explicit document tolerance policy.
pub fn boolean_operation_with_policy(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, BooleanError> {
    boolean_operation_with_policy_and_cancel(object, tool, op, policy, &NeverCancelled)
}

/// Cancellable canonical boolean operation under an explicit policy. No
/// partial solid or partial metadata is returned after cancellation.
pub fn boolean_operation_with_policy_and_cancel(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<OperationResult<Solid>, BooleanError> {
    boolean_operation_with_classes_policy_and_cancel(object, tool, op, None, None, policy, cancel)
}

/// Canonical boolean operation with owner-aware face merging. This is the
/// application integration boundary for durable face naming; lineage remains
/// represented by the shared [`TopologyHistory`] in the returned result.
pub fn boolean_operation_with_classes_policy_and_cancel(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    obj_classes: Option<&[Option<u64>]>,
    tool_classes: Option<&[Option<u64>]>,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<OperationResult<Solid>, BooleanError> {
    policy
        .validate()
        .map_err(BooleanError::InvalidTolerancePolicy)?;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    validate_operand(BooleanInput::Object, object, policy)?;
    validate_operand(BooleanInput::Tool, tool, policy)?;
    let tolerance_context = boolean_tolerance_context(object, tool, policy);
    let policy = &tolerance_context.policy;

    let (result, face_history, mut recovery) = catch_unwind(AssertUnwindSafe(|| {
        boolean_impl(
            object,
            tool,
            op,
            BooleanOptions {
                obj_classes,
                tool_classes,
                want_history: true,
                policy,
                cancel,
            },
        )
    }))
    .map_err(|_| BooleanError::Panicked)?
    .map_err(|_| BooleanError::Cancelled)?;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    let (value, reconstructed) = repair_boolean_output(result, policy)?;
    if reconstructed > 0 {
        recovery.actions.push(RecoveryAction::ReconstructPcurves {
            count: reconstructed,
        });
    }
    let face_history = face_history.unwrap_or_default();
    let mut history = face_history.topology_history(op, object.face_count(), tool.face_count());
    let unattributed = history.complete_unattributed_results(&value);
    history.validate().map_err(BooleanError::InvalidHistory)?;
    let coverage = history.coverage_for_solid(&value);
    if !coverage.is_complete() {
        return Err(BooleanError::IncompleteHistory {
            missing: coverage.missing_results,
        });
    }
    let mut diagnostics: Vec<_> = face_history
        .face_source
        .iter()
        .enumerate()
        .filter(|(_, source)| source.is_none())
        .map(|(index, _)| {
            Diagnostic::warning(
                "openrcad.history.unattributed-face",
                "boolean result face could not be attributed to an input face",
                Some(TopologyRef::face(index)),
            )
        })
        .collect();
    diagnostics.extend(unattributed.into_iter().map(|entity| {
        Diagnostic::info(
            "openrcad.history.conservative-generated",
            "result topology has no proven Phase 1 lineage and is explicitly marked generated",
            Some(entity),
        )
    }));
    let validation = ValidationReport::for_solid(&value, policy);
    if !validation.is_valid() {
        return Err(BooleanError::InvalidOutput {
            report: validation.health,
        });
    }

    Ok(OperationResult {
        value,
        history,
        diagnostics,
        recovery,
        validation,
    })
}

/// Canonical multi-body boolean operation under an explicit policy.
///
/// Each connected result is strictly validated and receives its own face-index
/// history. The shared operation history uses a flattened result-face index and
/// explicitly covers every lower-level result entity conservatively.
pub fn boolean_bodies_operation_with_classes_policy_and_cancel(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    obj_classes: Option<&[Option<u64>]>,
    tool_classes: Option<&[Option<u64>]>,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<OperationResult<BooleanBodies>, BooleanError> {
    policy
        .validate()
        .map_err(BooleanError::InvalidTolerancePolicy)?;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    validate_operand(BooleanInput::Object, object, policy)?;
    validate_operand(BooleanInput::Tool, tool, policy)?;
    let tolerance_context = boolean_tolerance_context(object, tool, policy);
    let policy = &tolerance_context.policy;

    let (combined, packed_history, mut recovery) = catch_unwind(AssertUnwindSafe(|| {
        boolean_impl(
            object,
            tool,
            op,
            BooleanOptions {
                obj_classes,
                tool_classes,
                want_history: true,
                policy,
                cancel,
            },
        )
    }))
    .map_err(|_| BooleanError::Panicked)?
    .map_err(|_| BooleanError::Cancelled)?;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    let (combined, reconstructed) = repair_multi_body_boolean_output(combined, policy)?;
    if reconstructed > 0 {
        recovery.actions.push(RecoveryAction::ReconstructPcurves {
            count: reconstructed,
        });
    }

    let packed_history = packed_history.unwrap_or_default();
    let bodies = combined.split_disconnected();
    if bodies.is_empty() {
        return Err(BooleanError::EmptyOutput);
    }
    let combined_faces = combined.faces();
    let mut face_history = Vec::with_capacity(bodies.len());
    for body in &bodies {
        let mut sources = Vec::with_capacity(body.face_count());
        for face in body.faces() {
            let packed_index = combined_faces
                .iter()
                .position(|candidate| candidate == &face)
                .ok_or(BooleanError::InvalidOutput {
                    report: body.health_report_with_policy(policy),
                })?;
            sources.push(packed_history.source_of(packed_index));
        }
        face_history.push(BooleanFaceHistory {
            face_source: sources,
        });
    }

    let mut history = TopologyHistory::default();
    let mut flat_face_index = 0usize;
    for per_body in &face_history {
        for source in &per_body.face_source {
            let result = TopologyRef::face(flat_face_index);
            match source {
                Some(BooleanFaceSource::Object(index)) => {
                    history.modified(InputTopologyRef::face(0, *index), result)
                }
                Some(BooleanFaceSource::Tool(index)) if op == BooleanOp::Cut => {
                    history.generated([InputTopologyRef::face(1, *index)], result)
                }
                Some(BooleanFaceSource::Tool(index)) => {
                    history.modified(InputTopologyRef::face(1, *index), result)
                }
                None => history.generated([], result),
            }
            flat_face_index += 1;
        }
    }
    // Multi-body topology has no single local arena for edge/vertex positions.
    // Account for those entities in deterministic body-major order, and record
    // every returned shell/solid explicitly.
    for kind in [
        openrcad_topo::TopologyKind::Vertex,
        openrcad_topo::TopologyKind::Edge,
        openrcad_topo::TopologyKind::Wire,
    ] {
        let mut index = 0usize;
        for body in &bodies {
            let count = match kind {
                openrcad_topo::TopologyKind::Vertex => body.vertex_count(),
                openrcad_topo::TopologyKind::Edge => body.edge_count(),
                openrcad_topo::TopologyKind::Wire => body
                    .shell()
                    .faces()
                    .iter()
                    .map(|face| face.wires().len())
                    .sum(),
                _ => unreachable!(),
            };
            for _ in 0..count {
                history.generated([], TopologyRef::new(kind, index));
                index += 1;
            }
        }
    }
    for index in 0..bodies.len() {
        history.generated(
            [InputTopologyRef::new(
                0,
                TopologyRef::new(openrcad_topo::TopologyKind::Shell, 0),
            )],
            TopologyRef::new(openrcad_topo::TopologyKind::Shell, index),
        );
        history.generated(
            [InputTopologyRef::new(
                0,
                TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0),
            )],
            TopologyRef::new(openrcad_topo::TopologyKind::Solid, index),
        );
    }

    history.validate().map_err(BooleanError::InvalidHistory)?;
    let coverage = history.coverage_for_solids(&bodies);
    if !coverage.is_complete() {
        return Err(BooleanError::IncompleteHistory {
            missing: coverage.missing_results,
        });
    }

    let mut validation = ValidationReport::for_solid(&bodies[0], policy);
    for body in bodies.iter().skip(1) {
        let report = ValidationReport::for_solid(body, policy);
        validation.health.errors.extend(report.health.errors);
        validation.health.warnings.extend(report.health.warnings);
        validation.watertight &= report.watertight;
        validation.pcurves_complete &= report.pcurves_complete;
    }
    if !validation.is_valid() {
        return Err(BooleanError::InvalidOutput {
            report: validation.health,
        });
    }
    Ok(OperationResult {
        value: BooleanBodies {
            bodies,
            face_history,
        },
        history,
        diagnostics: Vec::new(),
        recovery,
        validation,
    })
}

/// Cancellable checked boolean. Existing callers can continue to use
/// [`boolean_checked`]; interactive schedulers should use this entry point.
#[deprecated(
    note = "use boolean_operation_with_policy_and_cancel; this wrapper discards operation metadata"
)]
pub fn boolean_checked_with_cancel(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    cancel: &dyn CancellationProbe,
) -> Result<Solid, BooleanError> {
    boolean_operation_with_policy_and_cancel(object, tool, op, &TolerancePolicy::STANDARD, cancel)
        .map(|result| result.value)
}

/// Cancellable checked boolean under an explicit tolerance policy.
#[deprecated(
    note = "use boolean_operation_with_policy_and_cancel; this wrapper discards operation metadata"
)]
pub fn boolean_checked_with_policy_and_cancel(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<Solid, BooleanError> {
    policy
        .validate()
        .map_err(BooleanError::InvalidTolerancePolicy)?;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    validate_operand(BooleanInput::Object, object, policy)?;
    validate_operand(BooleanInput::Tool, tool, policy)?;
    let result = catch_unwind(AssertUnwindSafe(|| {
        boolean_impl(
            object,
            tool,
            op,
            BooleanOptions {
                obj_classes: None,
                tool_classes: None,
                want_history: false,
                policy,
                cancel,
            },
        )
    }))
    .map_err(|_| BooleanError::Panicked)?
    .map_err(|_| BooleanError::Cancelled)?
    .0;
    cancel
        .check_cancelled()
        .map_err(|_| BooleanError::Cancelled)?;
    repair_boolean_output(result, policy).map(|(solid, _)| solid)
}

/// Apply `op`, then split a result that severed the body into one solid per
/// connected component.
///
/// A cut that slices a body in two (or a fuse of two bodies that don't actually
/// touch) is returned by [`boolean`] as a single shell holding several disjoint
/// pieces — topologically valid, but really multiple bodies. This is the
/// multi-body entry point: it returns each connected component separately via
/// [`Solid::split_disconnected`]. For the common case where the result is one
/// connected body, the returned vector has a single element.
#[deprecated(
    note = "Phase 3 compatibility API for multi-body results; this wrapper discards operation metadata"
)]
#[allow(deprecated)]
pub fn boolean_bodies(object: &Solid, tool: &Solid, op: BooleanOp) -> Vec<Solid> {
    boolean_checked_bodies(object, tool, op)
        .unwrap_or_else(|error| panic!("multi-body boolean: {error}"))
}

/// Checked multi-body boolean: like [`boolean_checked`], but splits a severed
/// result into separate bodies and validates **each** one. Fails if any body is
/// unhealthy or non-watertight, so a half-formed sliver can't slip through.
#[deprecated(
    note = "Phase 3 compatibility API for multi-body results; this wrapper discards operation metadata"
)]
#[allow(deprecated)]
pub fn boolean_checked_bodies(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
) -> Result<Vec<Solid>, BooleanError> {
    boolean_checked_bodies_with_policy(object, tool, op, &TolerancePolicy::STANDARD)
}

/// Checked multi-body boolean under an explicit tolerance policy.
#[deprecated(
    note = "Phase 3 compatibility API for multi-body results; this wrapper discards operation metadata"
)]
pub fn boolean_checked_bodies_with_policy(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    policy: &TolerancePolicy,
) -> Result<Vec<Solid>, BooleanError> {
    policy
        .validate()
        .map_err(BooleanError::InvalidTolerancePolicy)?;
    validate_operand(BooleanInput::Object, object, policy)?;
    validate_operand(BooleanInput::Tool, tool, policy)?;

    let result = catch_unwind(AssertUnwindSafe(|| {
        boolean_impl(
            object,
            tool,
            op,
            BooleanOptions {
                obj_classes: None,
                tool_classes: None,
                want_history: false,
                policy,
                cancel: &NeverCancelled,
            },
        )
    }))
    .map_err(|_| BooleanError::Panicked)?
    .expect("NeverCancelled cannot cancel")
    .0;
    // A valid multi-body result is intentionally disconnected before it is
    // split, so the single-solid strict gate must be applied to each component,
    // not to the temporary packed shell.
    let (result, _) = result
        .repair_pcurves(policy)
        .map_err(BooleanError::PcurveBuild)?;
    let bodies = result.split_disconnected();
    for body in &bodies {
        validate_output(body.clone(), policy)?;
    }
    Ok(bodies)
}

/// Apply `op` between `object` and `tool`.
#[deprecated(note = "use boolean_operation; this wrapper discards metadata and panics on failure")]
pub fn boolean(object: &Solid, tool: &Solid, op: BooleanOp) -> Solid {
    boolean_operation(object, tool, op)
        .unwrap_or_else(|error| panic!("boolean: {error}"))
        .value
}

struct BooleanOptions<'a> {
    obj_classes: Option<&'a [Option<u64>]>,
    tool_classes: Option<&'a [Option<u64>]>,
    want_history: bool,
    policy: &'a TolerancePolicy,
    cancel: &'a dyn CancellationProbe,
}

fn boolean_impl(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    options: BooleanOptions<'_>,
) -> Result<(Solid, Option<BooleanFaceHistory>, RecoveryReport), openrcad_foundation::Cancelled> {
    let BooleanOptions {
        obj_classes,
        tool_classes,
        want_history,
        policy,
        cancel,
    } = options;
    cancel.check_cancelled()?;
    let tol = policy.intersection;

    // 0. Fuzzy pre-snap: nudge the tool so a near-coincident, overlapping planar
    //    face becomes exactly coincident with the object's. This collapses the
    //    near-miss / flush cut onto the boolean's clean coincident-face path,
    //    preventing the watertight "sliver" that would otherwise break a later
    //    fillet. Narrow and size-relative, so a real clearance is never snapped.
    let fuzz = policy.snap_tolerance(bbox_diag(object));
    let (snapped, snap_distance) = snap_tool_to_object(object, tool, fuzz, policy.resolution);
    let mut recovery = RecoveryReport::default();
    if let Some(distance) = snap_distance {
        recovery
            .actions
            .push(RecoveryAction::NearCoincidentSnap { distance });
    }
    let tool = &snapped;

    // 1. Build staging builders
    let mut builder_obj = BRepBuilder::from_brep((**object.brep()).clone());
    let mut builder_tool = BRepBuilder::from_brep((**tool.brep()).clone());

    // 2. Perform intersection and splitting
    let faces_obj = object.faces();
    let faces_tool = tool.faces();
    let bvh_obj = Bvh::build(&faces_obj);
    let bvh_tool = Bvh::build(&faces_tool);
    let pairs = Bvh::overlapping_pairs(&bvh_obj, &bvh_tool);

    // A. Split all boundary edges at mutual intersection points
    for &(f_obj_id, f_tool_id) in &pairs {
        cancel.check_cancelled()?;
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        for w_obj in f_obj.wires() {
            for e_obj in w_obj.edges() {
                for w_tool in f_tool.wires() {
                    for e_tool in w_tool.edges() {
                        if let (Some(c_obj), Some(c_tool)) = (e_obj.curve(), e_tool.curve()) {
                            let pts = crate::intersect::curve_curve(c_obj, c_tool, tol);
                            for pt in pts {
                                try_split_edge(
                                    &mut builder_obj,
                                    e_obj.id(),
                                    c_obj,
                                    e_obj.first(),
                                    e_obj.last(),
                                    &pt,
                                    policy,
                                );
                                try_split_edge(
                                    &mut builder_tool,
                                    e_tool.id(),
                                    c_tool,
                                    e_tool.first(),
                                    e_tool.last(),
                                    &pt,
                                    policy,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    // B. Split faces along intersection curves.
    //
    // Each original face is split only along the curves arising from the tool
    // faces it actually overlaps, and only its own descendant sub-faces are
    // re-split — never the whole builder. This keeps the work proportional to
    // the real intersections instead of exploding combinatorially across pairs.
    let mut obj_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_obj.iter().map(|f| (f.id(), vec![f.id()])).collect();
    let mut tool_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_tool.iter().map(|f| (f.id(), vec![f.id()])).collect();

    let mut splitting_edges_obj: std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    > = std::collections::HashMap::new();
    let mut splitting_edges_tool: std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    > = std::collections::HashMap::new();

    for &(f_obj_id, f_tool_id) in &pairs {
        cancel.check_cancelled()?;
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        let s_obj = f_obj.surface().unwrap();
        let s_tool = f_tool.surface().unwrap();
        if surfaces_are_same_domain(s_obj, s_tool, tol) {
            // Re-cutting a curved cavity presents the cavity wall and cutter
            // wall on the same infinite support. Their mutual boundaries do
            // not remove material; intersecting cap/blend faces provide every
            // real axial or angular split. Imprinting curved same-domain faces
            // against one another merely duplicates periodic seams.
            let recut_curved_domain = op == BooleanOp::Cut
                && !matches!(s_obj, GeomSurface::Plane(_) | GeomSurface::BSpline(_));
            if recut_curved_domain {
                continue;
            }

            // Same-domain faces: split f_obj along f_tool's boundary edges, and
            // vice versa — restricted to each face's own descendants.
            for w_tool in f_tool.wires() {
                // A coplanar circular cap (cylinder rim) sitting fully inside
                // f_obj — the cylinder-boss case. Imprint the *whole* circle once
                // so the imprint engine bores a clean hole (its closed-curve path),
                // instead of three arcs it can't close into a hole.
                if let Some(circle) = wire_full_circle(&w_tool, tol) {
                    if circle_inside_face(&circle, &f_obj) {
                        let c = GeomCurve::Circle(circle);
                        let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                        split_tracked(
                            &mut builder_obj,
                            sub,
                            &c,
                            0.0,
                            TAU,
                            &mut splitting_edges_obj,
                            tol,
                        );
                        continue;
                    }
                }
                for e_tool in w_tool.edges() {
                    if let Some(c_tool) = e_tool.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(
                            c_tool,
                            e_tool.first(),
                            e_tool.last(),
                            &f_obj,
                            tol,
                        );
                        for (first, last) in intervals {
                            let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                            split_tracked(
                                &mut builder_obj,
                                sub,
                                c_tool,
                                first,
                                last,
                                &mut splitting_edges_obj,
                                tol,
                            );
                        }
                    }
                }
            }
            for w_obj in f_obj.wires() {
                if let Some(circle) = wire_full_circle(&w_obj, tol) {
                    if circle_inside_face(&circle, &f_tool) {
                        let c = GeomCurve::Circle(circle);
                        let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                        split_tracked(
                            &mut builder_tool,
                            sub,
                            &c,
                            0.0,
                            TAU,
                            &mut splitting_edges_tool,
                            tol,
                        );
                        continue;
                    }
                }
                for e_obj in w_obj.edges() {
                    if let Some(c_obj) = e_obj.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(
                            c_obj,
                            e_obj.first(),
                            e_obj.last(),
                            &f_tool,
                            tol,
                        );
                        for (first, last) in intervals {
                            let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                            split_tracked(
                                &mut builder_tool,
                                sub,
                                c_obj,
                                first,
                                last,
                                &mut splitting_edges_tool,
                                tol,
                            );
                        }
                    }
                }
            }
        } else {
            // Intersecting surfaces: split each face along the trimmed intersection curves
            let curves = crate::intersect::surface_surface_curves(&f_obj, &f_tool, tol);
            for (curve, first, last) in curves {
                let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                split_tracked(
                    &mut builder_obj,
                    sub,
                    &curve,
                    first,
                    last,
                    &mut splitting_edges_obj,
                    tol,
                );
                let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                split_tracked(
                    &mut builder_tool,
                    sub,
                    &curve,
                    first,
                    last,
                    &mut splitting_edges_tool,
                    tol,
                );
            }
        }
    }

    // Partition faces that have accumulated splitting edges
    let run_partition = |builder: &mut BRepBuilder,
                         sub: &mut Vec<FaceId>,
                         split_map: &mut std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    >| {
        let mut new_sub = Vec::new();
        for &fid in sub.iter() {
            if let Some(mut edges) = split_map.remove(&fid) {
                if builder.brep().faces.contains_key(fid) {
                    deduplicate_splitting_edges(builder, &mut edges, policy.intersection);
                    discard_existing_boundary_splits(builder, fid, &mut edges, policy.intersection);
                    if edges.is_empty() {
                        new_sub.push(fid);
                    } else {
                        let partitioned = builder.partition_face(fid, &edges);
                        new_sub.extend(partitioned);
                    }
                } else {
                    new_sub.push(fid);
                }
            } else {
                new_sub.push(fid);
            }
        }
        *sub = new_sub;
    };

    // Partition in the original faces' deterministic order, NOT HashMap order.
    // Iterating `obj_sub.values_mut()` walks the map in Rust's per-process random
    // HashMap order, so `partition_face` allocates new FaceIds in a different
    // order each run. That cascades through `sew` (which merges in kept-face
    // order) and the merges into a topologically different — though still valid —
    // result, making downstream fillet/tessellation flaky (the same body would
    // pass or fail the ghost-material check from run to run). Walking the
    // deterministic original face lists pins the order so the boolean is
    // reproducible, which is what stable downstream edge/face identity needs.
    for f in &faces_obj {
        cancel.check_cancelled()?;
        if let Some(sub) = obj_sub.get_mut(&f.id()) {
            run_partition(&mut builder_obj, sub, &mut splitting_edges_obj);
        }
    }
    for f in &faces_tool {
        cancel.check_cancelled()?;
        if let Some(sub) = tool_sub.get_mut(&f.id()) {
            run_partition(&mut builder_tool, sub, &mut splitting_edges_tool);
        }
    }

    // 3. Classify all split faces
    let class_tol = policy.classification;
    let mut kept_faces = Vec::new();
    // Parallel to `kept_faces`: which INPUT face (by shell position) each kept
    // split face descends from — read straight from the split bookkeeping
    // (`obj_sub`/`tool_sub`), so it is exact, not matched.
    let mut kept_sources: Vec<Option<BooleanFaceSource>> = Vec::new();
    let origin_obj: std::collections::HashMap<FaceId, usize> = faces_obj
        .iter()
        .enumerate()
        .flat_map(|(idx, f)| {
            obj_sub
                .get(&f.id())
                .into_iter()
                .flatten()
                .map(move |&child| (child, idx))
        })
        .collect();
    let origin_tool: std::collections::HashMap<FaceId, usize> = faces_tool
        .iter()
        .enumerate()
        .flat_map(|(idx, f)| {
            tool_sub
                .get(&f.id())
                .into_iter()
                .flatten()
                .map(move |&child| (child, idx))
        })
        .collect();

    let brep_obj = builder_obj.build(); // seals into Arc<BRep>
    let brep_tool = builder_tool.build();
    let active_obj: std::collections::HashSet<FaceId> =
        obj_sub.values().flatten().copied().collect();
    let active_tool: std::collections::HashSet<FaceId> =
        tool_sub.values().flatten().copied().collect();

    // BVHs over the *split* faces so the coplanar pre-check tests only the
    // handful of opposite-side faces a given face actually overlaps, instead of
    // scanning all of them (O(n·m) → ≈O(n·log m)).
    let split_faces_obj: Vec<Face> = brep_obj
        .faces
        .iter()
        .filter(|(id, _)| active_obj.contains(id))
        .map(|(id, d)| Face::from_id(brep_obj.clone(), id, d.orientation))
        .collect();
    let split_faces_tool: Vec<Face> = brep_tool
        .faces
        .iter()
        .filter(|(id, _)| active_tool.contains(id))
        .map(|(id, d)| Face::from_id(brep_tool.clone(), id, d.orientation))
        .collect();
    let bvh_split_obj = Bvh::build(&split_faces_obj);
    let bvh_split_tool = Bvh::build(&split_faces_tool);

    for (f_id, f_data) in &brep_obj.faces {
        if !active_obj.contains(&f_id) {
            continue;
        }
        cancel.check_cancelled()?;
        let face = Face::from_id(brep_obj.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar_same = false;
        let mut coplanar_opposite = false;

        for ft_id in bvh_split_tool.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let ft_data = &brep_tool.faces[ft_id];
            let face_t = Face::from_id(brep_tool.clone(), ft_id, ft_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face.surface(), face_t.surface()) {
                if surfaces_are_same_domain(s_obj, s_tool, class_tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_tool, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_t) {
                        let aligned = effective_normal_at(&face, &pos)
                            .zip(effective_normal_at(&face_t, &pos))
                            .is_some_and(|(object_normal, tool_normal)| {
                                object_normal.dot(&tool_normal) > 0.0
                            });
                        if aligned {
                            coplanar_same = true;
                        } else {
                            coplanar_opposite = true;
                        }
                        break;
                    }
                }
            }
        }

        if coplanar_same {
            match op {
                BooleanOp::Fuse | BooleanOp::Common => {
                    kept_sources.push(
                        origin_obj
                            .get(&f_id)
                            .copied()
                            .map(BooleanFaceSource::Object),
                    );
                    kept_faces.push(face);
                }
                BooleanOp::Cut => {}
            }
        } else if coplanar_opposite {
            // For a cut, an oppositely-oriented coincident object face is
            // already the correctly oriented boundary of an existing cavity.
            // Re-cutting with the same tool surface keeps that face. Fuse and
            // Common discard the zero-thickness shared boundary.
            if op == BooleanOp::Cut {
                kept_sources.push(
                    origin_obj
                        .get(&f_id)
                        .copied()
                        .map(BooleanFaceSource::Object),
                );
                kept_faces.push(face);
            }
        } else {
            // Classification is a correctness boundary, including for planar
            // regions cut out by a periodic tool. A single parity ray can graze
            // one cylinder seam at ordinary part scale and keep the cap disk,
            // giving its rim three coedge uses. Use the same multi-direction
            // vote as the tool-side classification.
            let inside = is_point_inside_solid_robust(&pos, tool, &bvh_tool);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => !inside,
                BooleanOp::Common => inside,
            };
            if keep {
                kept_sources.push(
                    origin_obj
                        .get(&f_id)
                        .copied()
                        .map(BooleanFaceSource::Object),
                );
                kept_faces.push(face);
            }
        }
    }

    for (f_id, f_data) in &brep_tool.faces {
        if !active_tool.contains(&f_id) {
            continue;
        }
        cancel.check_cancelled()?;
        let face = Face::from_id(brep_tool.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar = false;
        for fo_id in bvh_split_obj.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let fo_data = &brep_obj.faces[fo_id];
            let face_o = Face::from_id(brep_obj.clone(), fo_id, fo_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face_o.surface(), face.surface()) {
                if surfaces_are_same_domain(s_obj, s_tool, class_tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_obj, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_o) {
                        coplanar = true;
                        break;
                    }
                }
            }
        }

        if coplanar {
            // Discard tool's coincident face (already handled by object's side)
        } else {
            let inside = is_point_inside_solid_robust(&pos, object, &bvh_obj);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => inside,
                BooleanOp::Common => inside,
            };
            if keep {
                kept_sources.push(origin_tool.get(&f_id).copied().map(BooleanFaceSource::Tool));
                if op == BooleanOp::Cut {
                    let reversed_face =
                        Face::from_id(brep_tool.clone(), f_id, f_data.orientation.reversed());
                    kept_faces.push(reversed_face);
                } else {
                    kept_faces.push(face);
                }
            }
        }
    }

    // 4. Sew kept faces together
    cancel.check_cancelled()?;
    let shell = crate::sew::sew_shell_with_policy(&kept_faces, policy)
        .expect("boolean policy was validated before assembly");
    let solid = Solid::new(shell);

    // 4b. Heal T-junctions: an imprint can split one face's boundary edge at a
    //     vertex without splitting the coincident edge of a perpendicular adjacent
    //     face (e.g. a boss footprint straddling a box edge), leaving the shell
    //     open. Split such edges at the stray interior vertex so the coincident
    //     edges share endpoints. A no-op (and skipped) when already watertight.
    let unhealed = solid;
    let solid = crate::merge::heal_tjunctions_with_policy(&unhealed, policy);
    if !std::sync::Arc::ptr_eq(unhealed.brep(), solid.brep()) {
        recovery.actions.push(RecoveryAction::HealTJunctions);
    }

    // History/owner plumbing. `sew`/`heal`/`merge` each re-key FaceIds, so
    // origins are not threaded through them — they are *resolved*: every stage's
    // faces are geometric subsets of the kept split faces, so sampling a point
    // on a face and asking which kept face's trimming loops contain it recovers
    // the exact origin (imprint boundaries decide coplanar ties, not
    // signatures).
    let has_classes = obj_classes.is_some() || tool_classes.is_some();
    let class_map = if has_classes {
        let origins = resolve_face_origins(&solid, &kept_faces, &kept_sources, policy);
        let mut map: std::collections::HashMap<FaceId, u64> = std::collections::HashMap::new();
        for (face, origin) in solid.faces().iter().zip(&origins) {
            // Tag the side into the class key so an object class value can
            // never collide with an equal tool class value.
            let class = match origin {
                Some(BooleanFaceSource::Object(i)) => obj_classes
                    .and_then(|c| c.get(*i).copied().flatten())
                    .map(|cl| cl << 1),
                Some(BooleanFaceSource::Tool(i)) => tool_classes
                    .and_then(|c| c.get(*i).copied().flatten())
                    .map(|cl| (cl << 1) | 1),
                None => None,
            };
            if let Some(class) = class {
                map.insert(face.id(), class);
            }
        }
        Some(map)
    } else {
        None
    };

    // 5. Merge coplanar faces split by the imprint (e.g. a union of two boxes
    //    keeps the shared face as several coplanar strips), then cocylindrical
    //    faces split by it (a corner cut whose arc crosses a `make_cylinder` rim
    //    seam leaves the concave wall as two faces). Each is a no-op fallback
    //    unless it produces a watertight, healthy, smaller solid. When owner
    //    classes are supplied, faces of different owners are never merged (the
    //    merge would erase the identity the history just preserved).
    let before_coplanar = solid;
    let before_count = before_coplanar.face_count();
    let solid = crate::merge::merge_coplanar_faces_classed_with_policy(
        &before_coplanar,
        class_map.as_ref(),
        policy,
    );
    if !std::sync::Arc::ptr_eq(before_coplanar.brep(), solid.brep()) {
        recovery.actions.push(RecoveryAction::MergeCoplanarFaces {
            removed_faces: before_count.saturating_sub(solid.face_count()),
        });
    }
    let before_cocylindrical = solid;
    let before_count = before_cocylindrical.face_count();
    let solid = crate::merge::merge_cocylindrical_faces_classed_with_policy(
        &before_cocylindrical,
        class_map.as_ref(),
        policy,
    );
    if !std::sync::Arc::ptr_eq(before_cocylindrical.brep(), solid.brep()) {
        recovery
            .actions
            .push(RecoveryAction::MergeCocylindricalFaces {
                removed_faces: before_count.saturating_sub(solid.face_count()),
            });
    }
    let solid = package_nested_cut_shells(solid, op, policy);
    let history = want_history
        .then(|| resolve_face_origins(&solid, &kept_faces, &kept_sources, policy))
        .map(|face_source| BooleanFaceHistory { face_source });
    cancel.check_cancelled()?;
    Ok((solid, history, recovery))
}

/// Convert a cut whose kept boundary has one enclosing component plus one or
/// more fully enclosed components into a single multi-shell solid. A severing
/// cut remains a packed disconnected shell so the public multi-body adapter can
/// split it into independent bodies.
fn package_nested_cut_shells(solid: Solid, op: BooleanOp, policy: &TolerancePolicy) -> Solid {
    if op != BooleanOp::Cut {
        return solid;
    }
    let mut components = solid.split_disconnected();
    if components.len() <= 1 {
        return solid;
    }

    let extent_volume = |component: &Solid| {
        component
            .bounding_box()
            .corners()
            .map(|(lo, hi)| {
                (hi.x() - lo.x()).abs() * (hi.y() - lo.y()).abs() * (hi.z() - lo.z()).abs()
            })
            .unwrap_or(0.0)
    };
    let outer_index = components
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| extent_volume(left).total_cmp(&extent_volume(right)))
        .map(|(index, _)| index)
        .unwrap_or(0);
    components.swap(0, outer_index);

    let outer = &components[0];
    let tolerance = policy.classification.max(policy.linear);
    let Some((outer_lo, outer_hi)) = outer.bounding_box().corners() else {
        return solid;
    };
    let all_enclosed = components[1..].iter().all(|component| {
        let Some((lo, hi)) = component.bounding_box().corners() else {
            return false;
        };
        let bbox_enclosed = lo.x() > outer_lo.x() + tolerance
            && lo.y() > outer_lo.y() + tolerance
            && lo.z() > outer_lo.z() + tolerance
            && hi.x() < outer_hi.x() - tolerance
            && hi.y() < outer_hi.y() - tolerance
            && hi.z() < outer_hi.z() - tolerance;
        bbox_enclosed
            && component
                .faces()
                .first()
                .is_some_and(|face| point_in_solid(&point_on_face(face), outer))
    });
    if !all_enclosed {
        return solid;
    }

    Solid::from_shells(components.into_iter().map(|component| component.shell()))
        .expect("nested cut has at least an outer shell")
}

/// For each face of `solid` (in shell order), the kept split face it descends
/// from: same supporting surface (the sample point projects onto it within
/// tolerance, outward normals aligned) *and* the sample point inside the kept
/// face's trimming loops. Containment against the true imprint boundaries is
/// what disambiguates coplanar candidates exactly.
fn resolve_face_origins(
    solid: &Solid,
    kept_faces: &[Face],
    kept_sources: &[Option<BooleanFaceSource>],
    policy: &TolerancePolicy,
) -> Vec<Option<BooleanFaceSource>> {
    let surf_tol = (policy.classification * 10.0).max(policy.pcurve_consistency);
    solid
        .faces()
        .iter()
        .map(|face| {
            face_origin_sample_points(face).into_iter().find_map(|pos| {
                let n_face = effective_normal_at(face, &pos)?;
                kept_faces
                    .iter()
                    .zip(kept_sources)
                    .find_map(|(kept, source)| {
                        let source = (*source)?;
                        let surf = kept.surface()?;
                        let (u, v) =
                            crate::intersect::search_nearest_parameter(surf, &pos, (0.0, 0.0));
                        if surf.point(u, v).distance(&pos) > surf_tol {
                            return None;
                        }
                        let n_kept = effective_normal_uv(kept, u, v)?;
                        if n_face.dot(&n_kept) < 0.5 {
                            return None;
                        }
                        if !crate::intersect::is_inside_trimming_loops(u, v, kept) {
                            return None;
                        }
                        Some(source)
                    })
            })
        })
        .collect()
}

/// Interior samples used only for history attribution after boolean face
/// healing/merging. A merged face's ordinary representative point can land
/// exactly on an old imprint seam (the centre of a face-mirrored union is the
/// common example), where neither kept half considers the point interior.
/// Blending outer pcurve midpoints toward that representative produces
/// deterministic points on both sides of the removed seam. Every candidate is
/// checked against the final face's true trimming loops before it can establish
/// lineage, so this broadens sampling without surface-signature guessing.
fn face_origin_sample_points(face: &Face) -> Vec<Pnt> {
    let primary = point_on_face(face);
    let mut samples = vec![primary];
    let (Some(outer), Some(surface)) = (face.outer_wire(), face.surface()) else {
        return samples;
    };
    let (center_u, center_v) =
        crate::intersect::search_nearest_parameter(surface, &primary, (0.0, 0.0));
    for index in 0..outer.len() {
        let Some(pcurve) = outer.pcurve(index) else {
            continue;
        };
        let boundary = pcurve.point_at_fraction(0.5);
        for interior_fraction in [0.35, 0.65] {
            let u = boundary.x() + (center_u - boundary.x()) * interior_fraction;
            let v = boundary.y() + (center_v - boundary.y()) * interior_fraction;
            if !crate::intersect::is_inside_trimming_loops(u, v, face) {
                continue;
            }
            let point = surface.point(u, v);
            if samples
                .iter()
                .all(|existing| existing.distance(&point) > 1.0e-9)
            {
                samples.push(point);
            }
        }
    }
    samples
}

/// Outward normal of `face` at the surface point nearest `pos` (dU × dV with
/// the face's orientation applied). `None` for a degenerate parameterization.
fn effective_normal_at(face: &Face, pos: &Pnt) -> Option<Dir> {
    let surf = face.surface()?;
    let (u, v) = crate::intersect::search_nearest_parameter(surf, pos, (0.0, 0.0));
    effective_normal_uv(face, u, v)
}

fn effective_normal_uv(face: &Face, u: f64, v: f64) -> Option<Dir> {
    let surf = face.surface()?;
    let (_, du, dv) = surf.d1(u, v);
    let n = du.cross(&dv);
    let n = n.normalized()?;
    let mut dir = Dir::new(n.x(), n.y(), n.z());
    if face.orientation() == openrcad_topo::Orientation::Reversed {
        dir = dir.reversed();
    }
    Some(dir)
}

fn validate_operand(
    input: BooleanInput,
    solid: &Solid,
    policy: &TolerancePolicy,
) -> Result<(), BooleanError> {
    let mut report = solid.health_report_with_policy(policy);
    if let Err(error) = solid.validate_strict_with_policy(policy) {
        let strict = openrcad_topo::HealthError::Validation(error);
        if !report.errors.contains(&strict) {
            report.errors.push(strict);
        }
    }
    if !report.is_healthy() || !solid.is_watertight_with_policy(policy) {
        return Err(BooleanError::InvalidInput { input, report });
    }
    Ok(())
}

fn validate_output(solid: Solid, policy: &TolerancePolicy) -> Result<Solid, BooleanError> {
    let mut report = solid.health_report_with_policy(policy);
    if let Err(error) = solid.validate_strict_with_policy(policy) {
        let strict = openrcad_topo::HealthError::Validation(error);
        if !report.errors.contains(&strict) {
            report.errors.push(strict);
        }
    }
    if !report.is_healthy() {
        return Err(BooleanError::InvalidOutput { report });
    }
    if !solid.is_watertight_with_policy(policy) {
        return Err(BooleanError::NonWatertightOutput { report });
    }
    let sliver_tol = policy.sliver_tolerance(bbox_diag(&solid));
    if let Some(thickness) = degenerate_sliver_thickness(&solid, sliver_tol, policy.resolution) {
        return Err(BooleanError::DegenerateSliver { thickness });
    }
    Ok(solid)
}

fn repair_boolean_output(
    solid: Solid,
    policy: &TolerancePolicy,
) -> Result<(Solid, usize), BooleanError> {
    let (solid, reconstructed) = solid
        .repair_pcurves(policy)
        .map_err(BooleanError::PcurveBuild)?;
    let solid = validate_output(solid, policy)?;
    Ok((solid, reconstructed))
}

/// Validate a compatibility result whose face history is indexed against one
/// packed shell even when the operation severed it into multiple bodies. The
/// temporary packed shell is intentionally disconnected; strict health and
/// sliver checks therefore apply to each connected result body independently.
fn repair_multi_body_boolean_output(
    solid: Solid,
    policy: &TolerancePolicy,
) -> Result<(Solid, usize), BooleanError> {
    let (solid, reconstructed) = solid
        .repair_pcurves(policy)
        .map_err(BooleanError::PcurveBuild)?;
    let bodies = solid.split_disconnected();
    if bodies.len() <= 1 {
        let solid = validate_output(solid, policy)?;
        return Ok((solid, reconstructed));
    }
    for body in bodies {
        validate_output(body, policy)?;
    }
    Ok((solid, reconstructed))
}

// ---- Fuzzy coincidence handling (snap + sliver gate) -----------------------
//
// A cut tool whose planar face stops a hair short of (or flush with) a body face
// is a near-coincidence. Left alone the boolean emits a near-zero-thickness
// "sliver": topologically watertight, but a tiny geometric lie that breaks the
// downstream fillet (the "funnel"). We handle it in two narrow, intent-preserving
// steps — both size-relative with a hard cap so a deliberate clearance larger
// than the fuzz is never touched:
//   1. a pre-boolean snap that nudges the tool to make a near-coincident,
//      overlapping face pair exactly coincident (`snap_tool_to_object`);
//   2. a post-boolean gate that rejects any residual sliver
//      (`degenerate_sliver_thickness`, wired into `validate_output`).

/// A planar face reduced to its plane frame and outer-wire vertices.
struct PlanarFaceInfo {
    normal: Dir,
    x: Dir,
    y: Dir,
    offset: f64, // normal · (point on plane)
    verts: Vec<Pnt>,
}

fn planar_faces(solid: &Solid) -> Vec<PlanarFaceInfo> {
    let mut out = Vec::new();
    for f in solid.faces() {
        if let Some(GeomSurface::Plane(p)) = f.surface() {
            let pos = p.position();
            let n = pos.direction();
            let loc = pos.location();
            let verts: Vec<Pnt> = match f.outer_wire() {
                Some(w) => w.edges().iter().map(|e| e.start().point()).collect(),
                None => continue,
            };
            if verts.is_empty() {
                continue;
            }
            out.push(PlanarFaceInfo {
                normal: n,
                x: pos.x_direction(),
                y: pos.y_direction(),
                offset: n.x() * loc.x() + n.y() * loc.y() + n.z() * loc.z(),
                verts,
            });
        }
    }
    out
}

/// Derive the operation-local numerical scale without imposing a unit floor.
fn boolean_tolerance_context(
    object: &Solid,
    tool: &Solid,
    policy: &TolerancePolicy,
) -> ToleranceContext {
    let bounds = [object.bounding_box(), tool.bounding_box()];
    let local_feature_size = bounds
        .iter()
        .filter_map(|bounds| bounds.corners())
        .flat_map(|(lo, hi)| [hi.x() - lo.x(), hi.y() - lo.y(), hi.z() - lo.z()])
        .filter(|extent| extent.is_finite() && *extent > 0.0)
        .min_by(f64::total_cmp);
    ToleranceContext::derive(policy, &bounds, local_feature_size, 1.0)
        .expect("validated policy produces a tolerance context")
}

fn bbox_diag(solid: &Solid) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for f in solid.faces() {
        if let Some(w) = f.outer_wire() {
            for e in w.edges() {
                let p = e.start().point();
                for (k, c) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
                    lo[k] = lo[k].min(c);
                    hi[k] = hi[k].max(c);
                }
            }
        }
    }
    if lo[0] > hi[0] {
        return 0.0;
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

#[inline]
fn dir_dot(a: Dir, b: Dir) -> f64 {
    a.x() * b.x() + a.y() * b.y() + a.z() * b.z()
}

/// Signed distance from `p` to face `f`'s plane.
#[inline]
fn plane_dist(f: &PlanarFaceInfo, p: Pnt) -> f64 {
    f.normal.x() * p.x() + f.normal.y() * p.y() + f.normal.z() * p.z() - f.offset
}

/// Do the two near-parallel planar faces overlap when projected onto `a`'s
/// in-plane axes? (Axis-aligned 2D bbox overlap — cheap and conservative.)
fn overlap_in_plane(a: &PlanarFaceInfo, b: &PlanarFaceInfo) -> bool {
    let proj = |verts: &[Pnt]| {
        let (mut u0, mut u1, mut v0, mut v1) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for p in verts {
            let u = a.x.x() * p.x() + a.x.y() * p.y() + a.x.z() * p.z();
            let v = a.y.x() * p.x() + a.y.y() * p.y() + a.y.z() * p.z();
            u0 = u0.min(u);
            u1 = u1.max(u);
            v0 = v0.min(v);
            v1 = v1.max(v);
        }
        (u0, u1, v0, v1)
    };
    let (au0, au1, av0, av1) = proj(&a.verts);
    let (bu0, bu1, bv0, bv1) = proj(&b.verts);
    au0 <= bu1 && bu0 <= au1 && av0 <= bv1 && bv0 <= av1
}

/// Pre-boolean fuzzy snap. If the tool sits within `fuzz` of coincidence with the
/// object on one or more compatible, overlapping planar-face pairs, translate the
/// whole tool so those pairs become exactly coincident — but only if a single
/// translation satisfies *every* detected pair. A conflicting multi-coincidence
/// (which one translation can't resolve) is left for the sliver gate.
fn snap_tool_to_object(
    object: &Solid,
    tool: &Solid,
    fuzz: f64,
    resolution: f64,
) -> (Solid, Option<f64>) {
    let obj_planes = planar_faces(object);
    let tool_planes = planar_faces(tool);
    // One required translation per near-coincident, overlapping pair.
    let mut reqs: Vec<(Dir, f64)> = Vec::new();
    for tp in &tool_planes {
        for op in &obj_planes {
            if dir_dot(tp.normal, op.normal).abs() < 0.999 {
                continue;
            }
            let d = plane_dist(op, tp.verts[0]); // tool plane's offset from obj plane
            if d == 0.0 || d.abs() >= fuzz {
                continue;
            }
            if !overlap_in_plane(op, tp) {
                continue;
            }
            reqs.push((op.normal, -d)); // close the gap: translate by op.normal·(-d)
        }
    }
    if reqs.is_empty() {
        return (tool.clone(), None);
    }
    // Compose one translation; orthogonal requirements add, a real conflict fails.
    let mut t = GeomVec::new(0.0, 0.0, 0.0);
    for &(n, s) in &reqs {
        t += GeomVec::from_dir(n) * s;
    }
    for &(n, s) in &reqs {
        let achieved = n.x() * t.x() + n.y() * t.y() + n.z() * t.z();
        if (achieved - s).abs() > fuzz * 0.5 {
            return (tool.clone(), None); // conflicting coincidences — let the gate handle it
        }
    }
    if t.magnitude() < resolution {
        return (tool.clone(), None);
    }
    let distance = t.magnitude();
    (tool.transformed(&Trsf::translation(t)), Some(distance))
}

/// Smallest wall thickness of a degenerate sliver in `solid`: two planar faces
/// that are parallel, within `tol` of coincident, and overlap in projection.
/// `None` if no sliver. (Exactly-coincident faces, `d ≈ 0`, are excluded — those
/// surface as non-manifold/non-watertight and are caught earlier.)
fn degenerate_sliver_thickness(solid: &Solid, tol: f64, resolution: f64) -> Option<f64> {
    let planes = planar_faces(solid);
    let mut found: Option<f64> = None;
    for i in 0..planes.len() {
        for j in (i + 1)..planes.len() {
            let (a, b) = (&planes[i], &planes[j]);
            if dir_dot(a.normal, b.normal).abs() < 0.999 {
                continue;
            }
            let d = plane_dist(a, b.verts[0]).abs();
            if d <= resolution || d >= tol {
                continue;
            }
            if !overlap_in_plane(a, b) {
                continue;
            }
            found = Some(found.map_or(d, |f| f.min(d)));
        }
    }
    found
}

/// Split `edge_id` at `pt`, but only when `pt` genuinely lies in the *interior*
/// of the finite edge segment. Skips off-segment crossings (the host curves are
/// infinite lines) and crossings that fall on an existing endpoint — both would
/// otherwise spawn zero-length / duplicate edges that break the sewn topology.
fn try_split_edge(
    builder: &mut BRepBuilder,
    edge_id: EdgeId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    pt: &Pnt,
    policy: &TolerancePolicy,
) {
    if !builder.brep().edges.contains_key(edge_id) {
        return;
    }
    let t = project_point_on_curve(pt, curve, first, last);
    if curve.point(t).distance(pt) > policy.pcurve_consistency {
        return; // off the finite segment
    }
    let ed = &builder.brep().edges[edge_id];
    let s_pt = builder.brep().vertices[ed.start].point;
    let e_pt = builder.brep().vertices[ed.end].point;
    if pt.distance(&s_pt) < policy.intersection || pt.distance(&e_pt) < policy.intersection {
        return; // coincides with an endpoint — no real split
    }
    let v = builder
        .brep_mut()
        .vertices
        .insert(openrcad_topo::arena::VertexData {
            point: *pt,
            tolerance: policy.linear,
        });
    builder.split_edge(edge_id, v, t);
}

/// Split every sub-face in `subfaces` along `curve`, replacing the list with the
/// resulting (possibly larger) set of descendant faces. Only these descendants
/// are ever revisited, so repeated calls across overlapping pairs stay bounded.
fn split_tracked(
    builder: &mut BRepBuilder,
    subfaces: &mut Vec<FaceId>,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    splitting_edges_map: &mut std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>>,
    tol: f64,
) {
    let mut result = Vec::with_capacity(subfaces.len());
    for &fid in subfaces.iter() {
        // A clean 2-point crosscut on a cylindrical face must be *queued* (not
        // split immediately): immediate bisection fragments the wall, so a
        // rect-minus-cylinder body collapses to a box (the vanishing circular
        // bite). Queuing lets every crosscut on the cylinder partition together
        // via the deferred `partition_face` path; `merge_cocylindrical_faces`
        // then coalesces the wall back to one analytic cylinder.
        let is_cylinder = builder
            .brep()
            .faces
            .get(fid)
            .and_then(|f| f.surface.as_ref())
            .is_some_and(|s| matches!(s, GeomSurface::Cylinder(_)));
        let force_queue_clean_crosscuts = is_cylinder
            || splitting_edges_map
                .get(&fid)
                .is_some_and(|edges| !edges.is_empty());
        let (next_faces, new_edges) = crate::imprint::imprint_curve_on_face(
            builder,
            fid,
            curve,
            first,
            last,
            force_queue_clean_crosscuts,
            tol,
        );
        if !new_edges.is_empty() {
            splitting_edges_map
                .entry(fid)
                .or_default()
                .extend(new_edges);
        }
        result.extend(next_faces);
    }
    *subfaces = result;
}

/// Remove repeated geometric split spans contributed by adjacent face pairs.
/// A periodic tool face can overlap both sides of an operand seam, causing the
/// same circle arc to be enqueued under two arena IDs. Feeding both copies to
/// the half-edge partition graph creates a zero-width two-edge region instead
/// of the intended crosscut.
fn deduplicate_splitting_edges(builder: &BRepBuilder, edges: &mut Vec<EdgeId>, tolerance: f64) {
    let mut unique = Vec::with_capacity(edges.len());
    for edge in edges.iter().copied() {
        if builder.brep().edges.contains_key(edge)
            && !unique
                .iter()
                .copied()
                .any(|other| edge_spans_match(builder, edge, other, tolerance))
        {
            unique.push(edge);
        }
    }
    *edges = unique;
}

/// A closed imprint can become an inner boundary before a second face pair
/// queues the same geometric spans for the later partition pass. Partitioning
/// along both copies gives the boundary three coedge uses. Drop queued spans
/// that the face already owns as outer or inner topology.
fn discard_existing_boundary_splits(
    builder: &BRepBuilder,
    face_id: FaceId,
    edges: &mut Vec<EdgeId>,
    tolerance: f64,
) {
    let Some(face) = builder.brep().faces.get(face_id) else {
        return;
    };
    let mut boundary_edges = Vec::new();
    for loop_id in face
        .outer_wire
        .into_iter()
        .chain(face.inner_wires.iter().copied())
    {
        if let Some(wire) = builder.brep().loops.get(loop_id) {
            boundary_edges.extend(wire.edges.iter().map(|edge| edge.id));
        }
    }
    edges.retain(|edge| {
        !boundary_edges
            .iter()
            .copied()
            .any(|boundary| edge_spans_match(builder, *edge, boundary, tolerance))
    });
}

fn edge_spans_match(builder: &BRepBuilder, left: EdgeId, right: EdgeId, tolerance: f64) -> bool {
    let Some(left) = builder.brep().edges.get(left) else {
        return false;
    };
    let Some(right) = builder.brep().edges.get(right) else {
        return false;
    };
    let tolerance = tolerance.max(1.0e-8) * 10.0;
    let sample = |edge: &openrcad_topo::arena::EdgeData, fraction: f64| {
        if let Some(curve) = &edge.curve {
            curve.point(edge.first + (edge.last - edge.first) * fraction)
        } else {
            let start = builder.brep().vertices[edge.start].point;
            let end = builder.brep().vertices[edge.end].point;
            start + (end - start) * fraction
        }
    };
    let same_direction = sample(left, 0.0).distance(&sample(right, 0.0)) <= tolerance
        && sample(left, 1.0).distance(&sample(right, 1.0)) <= tolerance;
    let opposite_direction = sample(left, 0.0).distance(&sample(right, 1.0)) <= tolerance
        && sample(left, 1.0).distance(&sample(right, 0.0)) <= tolerance;
    (same_direction || opposite_direction)
        && [0.25, 0.5, 0.75].into_iter().all(|fraction| {
            let right_fraction = if same_direction {
                fraction
            } else {
                1.0 - fraction
            };
            sample(left, fraction).distance(&sample(right, right_fraction)) <= tolerance
        })
}

pub(crate) fn project_point_on_curve(p: &Pnt, curve: &GeomCurve, t_min: f64, t_max: f64) -> f64 {
    let (t_min, t_max) = ordered_curve_bounds(t_min, t_max);
    if let GeomCurve::BSpline(spline) = curve {
        if spline.degree() == 1
            && spline.weights().is_none()
            && spline.knots().len() == spline.poles().len()
        {
            let mut best = (t_min, f64::INFINITY);
            for (index, segment) in spline.poles().windows(2).enumerate() {
                let [start, end] = segment else {
                    continue;
                };
                let delta = *end - *start;
                let length_squared = delta.dot(&delta);
                if length_squared <= 1.0e-24 {
                    continue;
                }
                let knots = spline.knots();
                let knot_start = knots[index];
                let knot_end = knots[index + 1];
                let knot_span = knot_end - knot_start;
                if knot_span.abs() <= 1.0e-24 {
                    continue;
                }
                let allowed_start = t_min.max(knot_start.min(knot_end));
                let allowed_end = t_max.min(knot_start.max(knot_end));
                if allowed_start > allowed_end {
                    continue;
                }
                let fraction_start = ((allowed_start - knot_start) / knot_span).clamp(0.0, 1.0);
                let fraction_end = ((allowed_end - knot_start) / knot_span).clamp(0.0, 1.0);
                let fraction = ((*p - *start).dot(&delta) / length_squared).clamp(
                    fraction_start.min(fraction_end),
                    fraction_start.max(fraction_end),
                );
                let candidate = *start + delta * fraction;
                let distance = candidate.distance(p);
                if distance < best.1 {
                    let parameter = knot_start + knot_span * fraction;
                    best = (parameter, distance);
                }
            }
            if best.1.is_finite() {
                return best.0;
            }
        }
    }

    let mut best_t = t_min;
    let mut min_dist = p.distance(&curve.point(t_min));
    let steps = 10;
    for i in 0..=steps {
        let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
        let dist = p.distance(&curve.point(t));
        if dist < min_dist {
            min_dist = dist;
            best_t = t;
        }
    }

    let mut t = best_t;
    for _ in 0..5 {
        let (pt, tangent) = curve.d1(t);
        let diff = pt - *p;
        let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
        t = clamp_ordered(t - dt, t_min, t_max);
    }
    t
}

fn ordered_curve_bounds(t_min: f64, t_max: f64) -> (f64, f64) {
    let t_min = if t_min.is_finite() { t_min } else { -100.0 };
    let t_max = if t_max.is_finite() { t_max } else { 100.0 };
    if t_min <= t_max {
        (t_min, t_max)
    } else {
        (t_max, t_min)
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    let (lo, hi) = if min <= max { (min, max) } else { (max, min) };
    if value.is_nan() {
        lo
    } else {
        value.max(lo).min(hi)
    }
}

pub(crate) fn point_on_face(face: &Face) -> Pnt {
    // Surface-backed faces already carry authoritative, unwrapped pcurves.
    // Averaging boundary samples in UV gives a true interior point for periodic
    // bands (notably a cylinder split by a full-circle counterbore). A 3D chord
    // centroid falls inside the cylinder and the edge-offset fallback can land
    // on the split rim, which misclassifies the engulfed band.
    if let (Some(outer), Some(surface)) = (face.outer_wire(), face.surface()) {
        let samples: Vec<_> = (0..outer.len())
            .filter_map(|index| {
                outer
                    .pcurve(index)
                    .map(|curve| curve.point_at_fraction(0.5))
            })
            .collect();
        if samples.len() == outer.len() && !samples.is_empty() {
            let (u_sum, v_sum) = samples
                .iter()
                .fold((0.0, 0.0), |(u, v), point| (u + point.x(), v + point.y()));
            let u = u_sum / samples.len() as f64;
            let v = v_sum / samples.len() as f64;
            if crate::intersect::is_inside_trimming_loops(u, v, face) {
                let point = surface.point(u, v);
                let on_boundary = face.wires().iter().any(|wire| {
                    wire.edges()
                        .iter()
                        .any(|edge| distance_point_to_edge(&point, edge) < 1e-4)
                });
                if !on_boundary {
                    return point;
                }
            }
        }
    }

    // Preferred: the centroid of the outer loop's vertices, projected onto the
    // surface. For a convex face this is a robustly *interior* point, which the
    // ray-parity classifier needs — a near-boundary sample produces ambiguous
    // (boundary-grazing) hit counts. Only used if it actually lands inside the
    // trimming loops (non-convex faces fall through to the edge-offset probe).
    if let (Some(outer), Some(surf)) = (face.outer_wire(), face.surface()) {
        let pts: Vec<Pnt> = outer.edges().iter().map(|e| e.start().point()).collect();
        if !pts.is_empty() {
            let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
            for p in &pts {
                sum += p.coord();
            }
            let centroid = Pnt::from_xyz(sum / pts.len() as f64);
            let (cu, cv) = crate::intersect::search_nearest_parameter(surf, &centroid, (0.0, 0.0));
            let on_surf = surf.point(cu, cv);
            if on_surf.distance(&centroid) < 1e-6
                && crate::intersect::is_inside_trimming_loops(cu, cv, face)
            {
                let mut on_boundary = false;
                for wire in face.wires() {
                    for edge in wire.edges() {
                        if distance_point_to_edge(&on_surf, &edge) < 1e-4 {
                            on_boundary = true;
                            break;
                        }
                    }
                    if on_boundary {
                        break;
                    }
                }
                if !on_boundary {
                    return on_surf;
                }
            }
        }
    }

    if let Some(outer) = face.outer_wire() {
        let edges = outer.edges();
        if !edges.is_empty() {
            let e = &edges[0];
            let t_mid = 0.5 * (e.first() + e.last());
            if let Some(curve) = e.curve() {
                let mid_pt = curve.point(t_mid);
                if let Some(surf) = face.surface() {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(surf, &mid_pt, (0.0, 0.0));
                    let (_, tangent) = curve.d1(t_mid);
                    let normal = match surf {
                        GeomSurface::Plane(plane) => plane.normal(),
                        _ => {
                            let (_, su, sv) = eval_d1(surf, u, v);
                            su.cross(&sv)
                                .normalized()
                                .unwrap_or(openrcad_foundation::Dir::dz())
                        }
                    };

                    let dir1 = GeomVec::from_dir(
                        tangent
                            .cross(&openrcad_foundation::Vec::from_dir(normal))
                            .normalized()
                            .unwrap(),
                    );
                    let dir2 = -dir1;

                    let test_dist = 1e-3;
                    let p1 = mid_pt + dir1 * test_dist;
                    let (u1, v1) = crate::intersect::search_nearest_parameter(surf, &p1, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u1, v1, face) {
                        return p1;
                    }

                    let p2 = mid_pt + dir2 * test_dist;
                    let (u2, v2) = crate::intersect::search_nearest_parameter(surf, &p2, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u2, v2, face) {
                        return p2;
                    }
                }
            }
        }
    }

    let vertices = face
        .wires()
        .iter()
        .flat_map(|w| w.edges())
        .map(|e| e.start().point())
        .collect::<Vec<_>>();
    if !vertices.is_empty() {
        let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
        for v in &vertices {
            sum += v.coord();
        }
        Pnt::from_xyz(sum / vertices.len() as f64)
    } else {
        Pnt::origin()
    }
}

fn point_outside_solid_bounds(p: &Pnt, bvh: &Bvh) -> bool {
    // Solid::bounding_box covers vertices only and can under-approximate curved
    // edges. The BVH root also samples boundary curves (and includes B-spline
    // poles), making it the appropriate broad-phase bound for classification.
    if let Some((lo, hi)) = bvh.nodes.first().and_then(|node| node.bounds.corners()) {
        const BOUNDS_TOL: f64 = 1.0e-7;
        p.x() < lo.x() - BOUNDS_TOL
            || p.x() > hi.x() + BOUNDS_TOL
            || p.y() < lo.y() - BOUNDS_TOL
            || p.y() > hi.y() + BOUNDS_TOL
            || p.z() < lo.z() - BOUNDS_TOL
            || p.z() > hi.z() + BOUNDS_TOL
    } else {
        false
    }
}

/// Cast one ray-parity direction, perturbing it when a hit grazes a topological
/// boundary. `None` means every permitted perturbation remained ambiguous.
fn cast_ray_parity(
    p: &Pnt,
    solid: &Solid,
    bvh: &Bvh,
    base: Dir,
    seed: u64,
    max_attempts: usize,
) -> Option<bool> {
    let mut ray_dir = base;
    let mut rng = seed;
    for _ in 0..max_attempts {
        let mut intersection_count = 0;
        let mut hit_boundary = false;
        let ray_dir_vec = GeomVec::from_dir(ray_dir);

        // Only test faces whose AABB the ray crosses (BVH prunes the rest), so
        // each cast is O(log F + hits) and only hit faces are reconstructed.
        'faces: for fid in bvh.ray_cast(p, &ray_dir_vec) {
            let orientation = solid.brep().faces[fid].orientation;
            let face = Face::from_id(solid.brep().clone(), fid, orientation);
            for hit in crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7) {
                let on_boundary = face.wires().iter().any(|wire| {
                    wire.edges()
                        .iter()
                        .any(|edge| distance_point_to_edge(&hit, edge) < 1e-5)
                });
                if on_boundary {
                    hit_boundary = true;
                    break 'faces;
                }
                intersection_count += 1;
            }
        }

        if !hit_boundary {
            return Some((intersection_count % 2) == 1);
        }

        let mut next_offset = || {
            rng = rng.wrapping_mul(1_103_515_245).wrapping_add(12_345) & 0x7fff_ffff;
            ((rng & 0xff) as f64 / 255.0) - 0.5
        };
        let offset = GeomVec::new(next_offset(), next_offset(), next_offset()) * 0.1;
        ray_dir = Dir::from_vec(&(ray_dir_vec + offset)).unwrap_or(ray_dir);
    }
    None
}

fn unchecked_ray_parity(p: &Pnt, solid: &Solid, bvh: &Bvh, ray_dir: GeomVec) -> bool {
    let mut count = 0;
    for fid in bvh.ray_cast(p, &ray_dir) {
        let orientation = solid.brep().faces[fid].orientation;
        let face = Face::from_id(solid.brep().clone(), fid, orientation);
        count += crate::intersect::ray_face_all(p, &ray_dir, &face, 1e-7).len();
    }
    (count % 2) == 1
}

fn is_point_inside_solid_robust(p: &Pnt, solid: &Solid, bvh: &Bvh) -> bool {
    if point_outside_solid_bounds(p, bvh) {
        return false;
    }

    // Parity of ray-surface crossings along one direction (`true` => odd => inside),
    // skipping any cast whose hit grazes a face boundary edge (an unreliable count).
    // Returns `None` if every perturbation of this direction kept grazing.
    //
    // A single ray can still silently mis-count without grazing — e.g. a hit that
    // exits right at a cap/wall rim is dropped by the face-containment test, turning
    // an even count odd. So the caller votes across several independent directions:
    // an occasional miss is outvoted instead of flipping the classification.
    // A spread of "generic" irrational-ish directions: a symmetric diagonal like
    // (1,1,1) skewers the corners/edges of axis-aligned boxes, defeating parity, so
    // every direction avoids integer-coordinate alignments. Voting across them makes
    // the classifier robust to a single direction's occasional rim mis-count.
    const DIRS: [(f64, f64, f64); 5] = [
        (0.182_321, 0.523_157, 0.832_511),
        (0.701_223, -0.337_419, 0.628_991),
        (-0.487_633, 0.811_077, 0.324_551),
        (0.273_194, 0.659_832, -0.700_447),
        (-0.638_915, -0.451_273, 0.623_881),
    ];

    let mut inside_votes = 0i32;
    let mut total = 0i32;
    for (i, &(x, y, z)) in DIRS.iter().enumerate() {
        let Some(dir) = Dir::from_vec(&GeomVec::new(x, y, z)) else {
            continue;
        };
        if let Some(parity) = cast_ray_parity(p, solid, bvh, dir, 42 + i as u64, 16) {
            total += 1;
            if parity {
                inside_votes += 1;
            }
        }
    }

    if total == 0 {
        // Every direction kept grazing: fall back to one no-veto parity reading so
        // we always return a definite answer.
        return unchecked_ray_parity(p, solid, bvh, GeomVec::new(0.182_321, 0.523_157, 0.832_511));
    }

    // Majority vote (ties -> inside is false, matching a strict-majority "inside").
    inside_votes * 2 > total
}

/// Whether `p` lies inside `solid` (ray-parity test).
///
/// Public robust point classifier that builds the face BVH the same way the
/// boolean engine does. Useful to classify a probe point against a body — e.g.
/// the rolling-ball fillet distinguishing a concave cut wall (material outside
/// the cylinder) from a convex prior-blend cylinder (inside).
pub fn point_in_solid(p: &Pnt, solid: &Solid) -> bool {
    let bvh = Bvh::build(&solid.faces());
    is_point_inside_solid_robust(p, solid, &bvh)
}

fn distance_point_to_edge(p: &Pnt, edge: &openrcad_topo::Edge) -> f64 {
    if let Some(curve) = edge.curve() {
        let (t_min, t_max) = ordered_curve_bounds(edge.first(), edge.last());
        let mut best_t = t_min;
        let mut min_dist = p.distance(&curve.point(t_min));

        let steps = 10;
        for i in 0..=steps {
            let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
            let dist = p.distance(&curve.point(t));
            if dist < min_dist {
                min_dist = dist;
                best_t = t;
            }
        }

        let mut t = best_t;
        for _ in 0..5 {
            let (pt, tangent) = curve.d1(t);
            let diff = pt - *p;
            let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
            t = clamp_ordered(t - dt, t_min, t_max);
        }
        p.distance(&curve.point(t))
    } else {
        p.distance(&edge.start().point())
    }
}

/// If `wire` is a closed loop whose every edge is an arc of one common circle
/// (a cylinder cap rim), return that full circle. Used to imprint the whole
/// circle as a hole instead of three un-closable arcs.
fn wire_full_circle(wire: &Wire, tol: f64) -> Option<Circle> {
    let edges = wire.edges();
    if edges.len() < 2 {
        return None;
    }
    let mut circ: Option<Circle> = None;
    for e in &edges {
        match e.curve() {
            Some(GeomCurve::Circle(c)) => match &circ {
                Some(prev) => {
                    if prev.center().distance(&c.center()) > tol
                        || (prev.radius() - c.radius()).abs() > tol
                        || !prev.axis().is_parallel(&c.axis(), 1e-6)
                    {
                        return None;
                    }
                }
                None => circ = Some(*c),
            },
            _ => return None,
        }
    }
    circ
}

/// Whether `circle` lies on `face`'s surface and strictly inside its trimming
/// loops (sampled around the rim) — i.e. the cap sits fully within `face`.
fn circle_inside_face(circle: &Circle, face: &Face) -> bool {
    let Some(surf) = face.surface() else {
        return false;
    };
    for i in 0..8 {
        let u = TAU * (i as f64) / 8.0;
        let p = circle.point(u);
        let (uu, vv) = crate::intersect::search_nearest_parameter(surf, &p, (0.0, 0.0));
        if surf.point(uu, vv).distance(&p) > 1e-6 {
            return false;
        }
        if !crate::intersect::is_inside_trimming_loops(uu, vv, face) {
            return false;
        }
    }
    true
}

fn surfaces_are_coplanar(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> bool {
    match (s1, s2) {
        (GeomSurface::Plane(p1), GeomSurface::Plane(p2)) => {
            let dot = p1.normal().dot(&p2.normal());
            if (dot.abs() - 1.0).abs() > 1e-5 {
                return false;
            }
            let dist = GeomVec::from_dir(p1.normal()).dot(&(p2.location() - p1.location()));
            dist.abs() <= tol
        }
        _ => false,
    }
}

/// Whether two analytic parameterizations describe the same infinite support
/// surface. Parameter origins and seam directions may differ; boolean trimming
/// cares about the geometric domain, not those coordinates.
pub(crate) fn surfaces_are_same_domain(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> bool {
    if surfaces_are_coplanar(s1, s2, tol) {
        return true;
    }
    let axes_same_line = |a: openrcad_foundation::Ax3, b: openrcad_foundation::Ax3| {
        let da = a.direction();
        let db = b.direction();
        da.dot(&db).abs() >= 1.0 - 1.0e-6
            && (b.location() - a.location())
                .cross(&GeomVec::from_dir(da))
                .magnitude()
                <= tol
    };
    match (s1, s2) {
        (GeomSurface::Cylinder(a), GeomSurface::Cylinder(b)) => {
            (a.radius() - b.radius()).abs() <= tol && axes_same_line(a.position(), b.position())
        }
        (GeomSurface::Cone(a), GeomSurface::Cone(b)) => {
            if (a.semi_angle() - b.semi_angle()).abs() > 1.0e-6
                || !axes_same_line(a.position(), b.position())
            {
                return false;
            }
            let slope = a.semi_angle().tan();
            if slope.abs() <= 1.0e-12 {
                return false;
            }
            let apex_a = a.position().location()
                - GeomVec::from_dir(a.position().direction()) * (a.ref_radius() / slope);
            let apex_b = b.position().location()
                - GeomVec::from_dir(b.position().direction()) * (b.ref_radius() / slope);
            apex_a.distance(&apex_b) <= tol
        }
        (GeomSurface::Sphere(a), GeomSurface::Sphere(b)) => {
            (a.radius() - b.radius()).abs() <= tol && a.center().distance(&b.center()) <= tol
        }
        (GeomSurface::Torus(a), GeomSurface::Torus(b)) => {
            (a.major_radius() - b.major_radius()).abs() <= tol
                && (a.minor_radius() - b.minor_radius()).abs() <= tol
                && a.position().location().distance(&b.position().location()) <= tol
                && a.position()
                    .direction()
                    .dot(&b.position().direction())
                    .abs()
                    >= 1.0 - 1.0e-6
        }
        _ => false,
    }
}

fn eval_d1(s: &GeomSurface, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec) {
    s.d1(u, v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;
    use openrcad_topo::Shell;

    #[test]
    fn cancelled_checked_boolean_returns_no_partial_solid() {
        let object = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let token = openrcad_foundation::CancellationToken::new();
        token.cancel();
        assert_eq!(
            boolean_checked_with_cancel(&object, &tool, BooleanOp::Fuse, &token),
            Err(BooleanError::Cancelled)
        );
    }

    #[test]
    fn explicit_standard_policy_preserves_checked_boolean_result() {
        let object = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let implicit = boolean_checked(&object, &tool, BooleanOp::Fuse).unwrap();
        let explicit = boolean_checked_with_policy(
            &object,
            &tool,
            BooleanOp::Fuse,
            &TolerancePolicy::STANDARD,
        )
        .unwrap();

        assert_eq!(implicit.face_count(), explicit.face_count());
        assert_eq!(
            implicit.bounding_box().corners(),
            explicit.bounding_box().corners()
        );
        assert!(explicit
            .health_report_with_policy(&TolerancePolicy::STANDARD)
            .is_healthy());
        assert!(explicit.is_watertight_with_policy(&TolerancePolicy::STANDARD));
    }

    #[test]
    fn checked_boolean_rejects_invalid_policy_before_modeling() {
        let object = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let invalid = TolerancePolicy {
            intersection: f64::NAN,
            ..TolerancePolicy::STANDARD
        };

        assert!(matches!(
            boolean_checked_with_policy(&object, &tool, BooleanOp::Fuse, &invalid),
            Err(BooleanError::InvalidTolerancePolicy(_))
        ));
    }

    #[test]
    fn structured_boolean_result_carries_history_recovery_and_validation() {
        let object = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let result = boolean_operation(&object, &tool, BooleanOp::Fuse).unwrap();

        assert!(result.validation.is_valid());
        assert!(result.history.validate().is_ok());
        assert_eq!(
            result
                .history
                .sources_of(TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0)),
            vec![
                InputTopologyRef::new(0, TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0)),
                InputTopologyRef::new(1, TopologyRef::new(openrcad_topo::TopologyKind::Solid, 0)),
            ]
        );
        assert!(result
            .history
            .coverage_for_solid(&result.value)
            .is_complete());
        assert!(result
            .diagnostics
            .iter()
            .all(|diagnostic| diagnostic.severity != openrcad_topo::DiagnosticSeverity::Error));
        assert!(result
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.code == "openrcad.history.conservative-generated"));
        assert!(result.recovery.was_modified());
    }

    /// Shell position of the input/result face whose plane has |normal·axis|≈1
    /// and passes through `coord` on `axis`.
    fn plane_face_pos(solid: &Solid, axis: usize, coord: f64) -> Option<usize> {
        solid.shell().faces().iter().position(|f| {
            let Some(GeomSurface::Plane(p)) = f.surface() else {
                return false;
            };
            let n = p.normal();
            if [n.x(), n.y(), n.z()][axis].abs() < 0.99 {
                return false;
            }
            let loc = p.location();
            (([loc.x(), loc.y(), loc.z()][axis]) - coord).abs() < 1e-6
        })
    }

    fn face_history_from_operation(result: &OperationResult<Solid>) -> BooleanFaceHistory {
        BooleanFaceHistory {
            face_source: (0..result.value.face_count())
                .map(|index| {
                    result
                        .history
                        .sources_of(TopologyRef::face(index))
                        .into_iter()
                        .find_map(|source| match (source.operand, source.entity.kind) {
                            (0, openrcad_topo::TopologyKind::Face) => {
                                Some(BooleanFaceSource::Object(source.entity.index))
                            }
                            (1, openrcad_topo::TopologyKind::Face) => {
                                Some(BooleanFaceSource::Tool(source.entity.index))
                            }
                            _ => None,
                        })
                })
                .collect(),
        }
    }

    #[test]
    fn history_traces_union_faces_to_their_operands() {
        // obj [0,10]^3 ∪ tool [5,15]x[0,10]x[0,10] → one [0,15] box.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let outcome = boolean_operation(&obj, &tool, BooleanOp::Fuse).expect("fuse");
        let hist = face_history_from_operation(&outcome);
        let result = outcome.value;

        assert_eq!(
            hist.face_source.len(),
            result.shell().faces().len(),
            "one history entry per result face"
        );
        let x0 = plane_face_pos(&result, 0, 0.0).expect("x=0 face");
        let x15 = plane_face_pos(&result, 0, 15.0).expect("x=15 face");
        let obj_x0 = plane_face_pos(&obj, 0, 0.0).expect("object x=0 input face");
        let tool_x15 = plane_face_pos(&tool, 0, 15.0).expect("tool x=15 input face");
        assert_eq!(
            hist.source_of(x0),
            Some(BooleanFaceSource::Object(obj_x0)),
            "x=0 face must trace to the object's own x=0 input face"
        );
        assert_eq!(
            hist.source_of(x15),
            Some(BooleanFaceSource::Tool(tool_x15)),
            "x=15 face must trace to the tool's x=15 input face"
        );
        assert!(
            hist.face_source.iter().all(|s| s.is_some()),
            "every union face should be attributed, got {:?}",
            hist.face_source
        );
    }

    #[test]
    fn history_attributes_faces_merged_across_removed_imprint_seam() {
        // A face-mirrored Join has this exact topology: the two operands share
        // x=10, and coplanar exterior pairs merge into faces whose ordinary
        // representative points lie on that removed centre seam.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let outcome = boolean_operation(&obj, &tool, BooleanOp::Fuse).expect("face-adjacent fuse");
        let hist = face_history_from_operation(&outcome);

        assert!(
            hist.face_source.iter().all(Option::is_some),
            "merged exterior faces must retain lineage from at least one contributing half: {:?}",
            hist.face_source
        );
        assert!(
            outcome
                .diagnostics
                .iter()
                .all(|diagnostic| diagnostic.code != "openrcad.history.unattributed-face"),
            "a removed imprint seam is not missing face history: {:?}",
            outcome.diagnostics
        );
    }

    #[test]
    fn history_traces_cut_pocket_walls_to_the_tool() {
        // Square pillar punched through the box in Z: 4 generated hole walls.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(3.0, 3.0, -1.0), 4.0, 4.0, 12.0);
        let outcome = boolean_operation(&obj, &tool, BooleanOp::Cut).expect("cut");
        let hist = face_history_from_operation(&outcome);
        let result = outcome.value;

        let tool_walls = hist
            .face_source
            .iter()
            .filter(|s| matches!(s, Some(BooleanFaceSource::Tool(_))))
            .count();
        assert!(
            tool_walls >= 4,
            "the 4 generated hole walls must trace to the tool, got {:?}",
            hist.face_source
        );
        // Modified: the box top survives (with a hole) and keeps its identity.
        let top = plane_face_pos(&result, 2, 10.0).expect("result keeps a z=10 top");
        let obj_top = plane_face_pos(&obj, 2, 10.0).expect("object z=10 input face");
        assert_eq!(
            hist.source_of(top),
            Some(BooleanFaceSource::Object(obj_top)),
            "the holed top face is a MODIFIED image of the object's top"
        );
        // Determinism: the correspondence is identical across rebuilds.
        let outcome2 = boolean_operation(&obj, &tool, BooleanOp::Cut).expect("repeat cut");
        let hist2 = face_history_from_operation(&outcome2);
        assert_eq!(hist.face_source, hist2.face_source);
    }

    #[test]
    fn owner_classes_block_cross_owner_coplanar_merge() {
        // Two boxes side by side, tops coplanar at z=10. Unclassed, the merge
        // unifies the tops into ONE face; with distinct owner classes on every
        // face of each operand, the two tops must stay separate faces (each
        // attributed to its own operand) — the Bidarra owner-aware-merge rule.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        let unclassed = boolean(&obj, &tool, BooleanOp::Fuse);
        let unclassed_tops = unclassed
            .shell()
            .faces()
            .iter()
            .filter(|f| match f.surface() {
                Some(GeomSurface::Plane(p)) => {
                    p.normal().z().abs() > 0.99 && (p.location().z() - 10.0).abs() < 1e-6
                }
                _ => false,
            })
            .count();
        assert_eq!(unclassed_tops, 1, "legacy merge unifies the coplanar tops");

        let obj_classes: Vec<Option<u64>> =
            (0..obj.shell().faces().len() as u64).map(Some).collect();
        let tool_classes: Vec<Option<u64>> =
            (0..tool.shell().faces().len() as u64).map(Some).collect();
        let outcome = boolean_operation_with_classes_policy_and_cancel(
            &obj,
            &tool,
            BooleanOp::Fuse,
            Some(&obj_classes),
            Some(&tool_classes),
            &TolerancePolicy::STANDARD,
            &NeverCancelled,
        )
        .expect("classed fuse");
        let hist = face_history_from_operation(&outcome);
        let classed = outcome.value;
        let classed_tops: Vec<usize> = classed
            .shell()
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| match f.surface() {
                Some(GeomSurface::Plane(p)) => (p.normal().z().abs() > 0.99
                    && (p.location().z() - 10.0).abs() < 1e-6)
                    .then_some(i),
                _ => None,
            })
            .collect();
        assert_eq!(
            classed_tops.len(),
            2,
            "owner-aware merge must keep the two differently-owned tops distinct"
        );
        let sides: Vec<_> = classed_tops.iter().map(|&i| hist.source_of(i)).collect();
        assert!(
            sides
                .iter()
                .any(|s| matches!(s, Some(BooleanFaceSource::Object(_))))
                && sides
                    .iter()
                    .any(|s| matches!(s, Some(BooleanFaceSource::Tool(_)))),
            "one top from each operand, got {sides:?}"
        );
    }

    #[test]
    fn ordered_curve_bounds_accept_reversed_and_nonfinite_limits() {
        assert_eq!(ordered_curve_bounds(2.14, 2.09), (2.09, 2.14));
        assert_eq!(ordered_curve_bounds(f64::NAN, 2.09), (-100.0, 2.09));
        assert_eq!(ordered_curve_bounds(2.14, f64::INFINITY), (2.14, 100.0));
    }

    #[test]
    fn primitives_pass_structural_validation() {
        // A well-formed box: contiguous loops, Euler–Poincaré V−E+F = 8−12+6 = 2.
        let b = make_box(&Pnt::new(-1.0, 2.0, 0.5), 2.0, 3.0, 4.0);
        b.assert_valid();
        assert_eq!(b.euler_characteristic(), 2);

        // Adversarial sliver: an extreme aspect ratio (thickness 1e-4 over a
        // 100×100 footprint) is still a topologically valid closed box. Its near
        // degeneracy is exactly the kind of input that trips fragile float
        // classification, so the structural invariants must still hold.
        let sliver = make_box(&Pnt::origin(), 100.0, 100.0, 1e-4);
        sliver.assert_valid();
        assert_eq!(sliver.euler_characteristic(), 2);
    }

    // ---- Fuzzy snap + sliver gate ------------------------------------------

    /// Box (21 × 11.5024 × 17.6) and a +Y cut cylinder whose back cap stops `gap`
    /// short of the back face (`gap > 0` = near-miss/sliver; `gap = 0` = flush).
    fn box_and_gap_cut(gap: f64) -> (Solid, Solid) {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;
        let back = 11.5024;
        let cube = make_box(&Pnt::new(-13.4, 0.0, -12.8), 21.0, back, 17.6);
        let cap_y = back - gap;
        let axis = Ax2::new_axes(Pnt::new(7.5, cap_y - 23.0, 5.0), Dir::dy(), Dir::dx());
        let cyl = make_cylinder(&axis, 7.81, 23.0);
        (cube, cyl)
    }

    /// Max vertex displacement between two structurally-identical solids.
    fn max_vertex_shift(a: &Solid, b: &Solid) -> f64 {
        let pts = |s: &Solid| -> Vec<Pnt> {
            s.shell()
                .faces()
                .iter()
                .filter_map(|f| f.outer_wire())
                .flat_map(|w| w.edges().into_iter().map(|e| e.start().point()))
                .collect()
        };
        pts(a)
            .iter()
            .zip(pts(b).iter())
            .map(|(x, y)| x.distance(y))
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn snap_fires_for_tiny_gap_but_preserves_real_clearance() {
        let (cube, _) = box_and_gap_cut(0.0);
        let fuzz = TolerancePolicy::STANDARD.snap_tolerance(bbox_diag(&cube));

        // A tiny numerical gap (sub-fuzz) snaps: the tool is nudged.
        let (cube, tool) = box_and_gap_cut(1e-7);
        let (snapped, _) =
            snap_tool_to_object(&cube, &tool, fuzz, TolerancePolicy::STANDARD.resolution);
        assert!(
            max_vertex_shift(&tool, &snapped) > 0.0,
            "a sub-fuzz near-coincidence should snap"
        );

        // A deliberate 0.1 mm clearance is far above the fuzz: the tool must NOT
        // move — the snap is not "CAD autocorrect with opinions".
        let (cube, tool) = box_and_gap_cut(0.1);
        let (snapped, _) =
            snap_tool_to_object(&cube, &tool, fuzz, TolerancePolicy::STANDARD.resolution);
        assert_eq!(
            max_vertex_shift(&tool, &snapped),
            0.0,
            "a real clearance above the fuzz must be left untouched"
        );
    }

    #[test]
    fn snap_skips_when_planes_do_not_overlap() {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;
        // A cylinder cap that is near-coplanar with the box's back face (gap 1e-7)
        // but positioned far away in-plane (centre XZ = (100,100)) — no projected
        // overlap, so it must NOT be snapped despite the near-coincident plane.
        let back = 11.5024;
        let cube = make_box(&Pnt::new(-13.4, 0.0, -12.8), 21.0, back, 17.6);
        let cap_y = back - 1e-7;
        let axis = Ax2::new_axes(Pnt::new(100.0, cap_y - 23.0, 100.0), Dir::dy(), Dir::dx());
        let tool = make_cylinder(&axis, 2.0, 23.0);
        let fuzz = TolerancePolicy::STANDARD.snap_tolerance(bbox_diag(&cube));
        let (snapped, _) =
            snap_tool_to_object(&cube, &tool, fuzz, TolerancePolicy::STANDARD.resolution);
        assert_eq!(
            max_vertex_shift(&tool, &snapped),
            0.0,
            "near-coplanar but non-overlapping faces must not snap"
        );
    }

    #[test]
    fn tiny_gap_cut_is_clean_after_snap_and_fillets() {
        let (cube, tool) = box_and_gap_cut(1e-7);
        let cut = boolean_checked(&cube, &tool, BooleanOp::Cut)
            .expect("a tiny-gap cut must snap to a clean, watertight body");
        assert!(cut.is_watertight() && cut.health_report().is_healthy());

        // The fillet that funneled in the GUI: top-back edge into the scoop.
        let edge = openrcad_topo::Edge::between_points(
            Pnt::new(-13.4, 11.5024, 4.8),
            Pnt::new(-0.31, 11.5024, 4.8),
        );
        let filleted = crate::fillet_edges(&cut, std::slice::from_ref(&edge), 3.0)
            .expect("filleting into the snapped cut must succeed");
        assert!(filleted.is_watertight() && filleted.health_report().is_healthy());
    }

    #[test]
    fn watertight_sliver_is_rejected_by_checked() {
        // A gap above the snap fuzz but a degenerate sub-sliver-tol wall: the cut
        // is watertight yet a tiny geometric lie. `boolean_checked` must reject it
        // rather than hand back a body that breaks the fillet.
        let (cube, tool) = box_and_gap_cut(2e-5);
        let res = boolean_checked(&cube, &tool, BooleanOp::Cut);
        assert!(res.is_err(), "a degenerate sliver must be rejected, got Ok");
    }

    #[test]
    fn test_boolean_intersection_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Common (Intersection)
        let common = boolean(&cube1, &cube2, BooleanOp::Common);
        // The intersection should be a 5x10x10 box
        // Bounding box of the intersection should be [5, 0, 0] to [10, 10, 10]
        let (lo, hi) = common.bounding_box().corners().unwrap();
        assert!((lo.x() - 5.0).abs() < 1e-5);
        assert!((lo.y() - 0.0).abs() < 1e-5);
        assert!((lo.z() - 0.0).abs() < 1e-5);
        assert!((hi.x() - 10.0).abs() < 1e-5);
        assert!((hi.y() - 10.0).abs() < 1e-5);
        assert!((hi.z() - 10.0).abs() < 1e-5);
        assert_eq!(common.face_count(), 6);
        assert_eq!(common.vertex_count(), 8);
        assert_eq!(common.edge_count(), 12);
        assert!(
            common.is_watertight(),
            "intersection result must be watertight"
        );
    }

    #[test]
    fn test_boolean_difference_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Cut (Difference): cube1 - cube2
        let cut = boolean(&cube1, &cube2, BooleanOp::Cut);
        // The result should be a 5x10x10 box from [0, 0, 0] to [5, 10, 10]
        let (lo, hi) = cut.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 5.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert_eq!(cut.face_count(), 6);
        assert_eq!(cut.vertex_count(), 8);
        assert_eq!(cut.edge_count(), 12);
        assert!(cut.is_watertight(), "difference result must be watertight");
    }

    /// Box minus a through-cylinder. This exercises the curved-boolean path:
    /// the planar caps are bored (closed-circle hole cutting) and the cylinder
    /// lateral wall is trimmed where it crosses the caps.
    ///
    /// This used to be a known failure: the cylinder's three lateral faces share
    /// seam edges, and when topology stored a single orientation per edge, cutting
    /// one shared seam corrupted the adjacent walls' loops, leaving them untrimmed
    /// so the result spilled past z∈[0,10]. That is fixed: orientation is now a
    /// per-use property of each co-edge in a loop ([`OrientedEdge`]), not of the
    /// shared edge, so splitting a shared seam keeps every loop tracing cleanly.
    /// The `hi.z ≈ 10` assertion below (the drill extends to z=11) locks that in.
    ///
    /// [`OrientedEdge`]: openrcad_topo::arena::OrientedEdge
    #[test]
    fn test_boolean_box_minus_cylinder_drills_hole() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        // A 10×10×10 box with a radius-2 cylinder running through it along Z,
        // centred at (5,5), extending below and above the box so it pierces both
        // the top and bottom caps.
        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // The outer envelope is unchanged (the drill only removes interior
        // material), so the bounding box is still the original cube.
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((lo.z() - 0.0).abs() < 1e-4, "lo.z={}", lo.z());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());
        assert!((hi.z() - 10.0).abs() < 1e-4, "hi.z={}", hi.z());

        // A through-hole adds the cylindrical wall faces plus bores the two caps,
        // so the result must have strictly more faces than the original 6.
        assert!(
            result.face_count() > 6,
            "expected a drilled solid with extra faces, got {}",
            result.face_count()
        );
        if !result.is_watertight() {
            println!("All faces in the result solid:");
            for (idx, face) in result.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!(
                            "    Edge: {:?} -> {:?}",
                            edge.start().point(),
                            edge.end().point()
                        );
                    }
                }
            }
        }
        assert!(result.is_watertight(), "drilled solid must be watertight");
    }

    #[test]
    fn scaled_box_minus_cylinder_drills_watertight_hole() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        // Regression dimensions minimized from ZeroCAD's bounded property test.
        // The same central through-cut worked on a 10 mm cube but left a free
        // circular boundary at ordinary part scale.
        let width: f64 = 50.453_704_715_724_64;
        let depth: f64 = 58.851_514_652_451_65;
        let height: f64 = 28.052_053_118_610_065;
        let radius = width.min(depth) * 0.254_935_325_547_562_9;
        let object = make_box(&Pnt::origin(), width, depth, height);
        let tool = make_cylinder(
            &Ax2::new(Pnt::new(width * 0.5, depth * 0.5, -1.0), Dir::dz()),
            radius,
            height + 2.0,
        );

        let result = boolean(&object, &tool, BooleanOp::Cut);
        assert!(
            result.is_watertight(),
            "scaled central through-hole must be watertight: {:?}",
            result.manifold_report()
        );
        assert!(result.health_report().is_healthy());
    }

    /// The curved-boolean engine completes without hanging (the historical
    /// blocker) and bores the box's caps. A lighter-weight companion to
    /// `test_boolean_box_minus_cylinder_drills_hole`, which additionally asserts
    /// the lateral walls are trimmed to z∈[0,10].
    #[test]
    fn test_boolean_box_minus_cylinder_runs_and_drills_caps() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // Drilling removes no outer XY material: the footprint stays [0,10]².
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());

        // The caps are bored and wall faces added, so the face count grows.
        assert!(
            result.face_count() > 6,
            "expected drilling to add faces, got {}",
            result.face_count()
        );
    }

    /// Two disjoint boxes packed into one shell split back into two solids,
    /// independent of the boolean engine. Locks in `split_disconnected`.
    #[test]
    fn split_disconnected_separates_two_packed_boxes() {
        let a = make_box(&Pnt::origin(), 5.0, 5.0, 5.0);
        let b = make_box(&Pnt::new(20.0, 0.0, 0.0), 5.0, 5.0, 5.0);
        let mut faces = a.shell().faces();
        faces.extend(b.shell().faces());
        let combined = Solid::new(Shell::from_faces(faces));
        // Combined shell: two euler-2 boxes → V−E+F = 16−24+12 = 4.
        assert_eq!(combined.euler_characteristic(), 4);

        let bodies = combined.split_disconnected();
        assert_eq!(bodies.len(), 2);
        for body in &bodies {
            assert_eq!(body.face_count(), 6);
            assert_eq!(body.euler_characteristic(), 2);
            assert!(body.is_watertight());
        }
    }

    /// A bar cut clean through the middle is severed into two separate bodies.
    /// The single-solid Phase 1 gate rejects disconnected shells, so callers use
    /// the explicit multi-body operation for this case.
    #[test]
    fn cut_severing_a_bar_yields_two_bodies() {
        // A 30×10×10 bar along X, sliced by a tool that fully spans Y and Z and
        // removes x∈[10,20], leaving x∈[0,10] and x∈[20,30].
        let bar = make_box(&Pnt::origin(), 30.0, 10.0, 10.0);
        let knife = make_box(&Pnt::new(10.0, -1.0, -1.0), 10.0, 12.0, 12.0);

        let bodies = boolean_bodies(&bar, &knife, BooleanOp::Cut);
        assert_eq!(bodies.len(), 2, "a through-cut must produce two bodies");

        for body in &bodies {
            assert!(body.is_watertight(), "each severed body must be watertight");
            assert_eq!(body.euler_characteristic(), 2);
            let (lo, hi) = body.bounding_box().corners().unwrap();
            // Each piece is a 10×10×10 cube; only its X span differs.
            assert!(
                (hi.x() - lo.x() - 10.0).abs() < 1e-4,
                "x span {}",
                hi.x() - lo.x()
            );
            assert!((hi.y() - lo.y() - 10.0).abs() < 1e-4);
            assert!((hi.z() - lo.z() - 10.0).abs() < 1e-4);
        }

        // The two pieces sit at opposite ends of the original bar.
        let mut x_los: Vec<f64> = bodies
            .iter()
            .map(|b| b.bounding_box().corners().unwrap().0.x())
            .collect();
        x_los.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(
            (x_los[0] - 0.0).abs() < 1e-4,
            "near piece at x≈0, got {}",
            x_los[0]
        );
        assert!(
            (x_los[1] - 20.0).abs() < 1e-4,
            "far piece at x≈20, got {}",
            x_los[1]
        );
    }

    #[test]
    fn structured_severing_cut_has_local_complete_history() {
        let bar = make_box(&Pnt::origin(), 30.0, 10.0, 10.0);
        let knife = make_box(&Pnt::new(10.0, -1.0, -1.0), 10.0, 12.0, 12.0);
        let outcome = boolean_bodies_operation_with_classes_policy_and_cancel(
            &bar,
            &knife,
            BooleanOp::Cut,
            None,
            None,
            &TolerancePolicy::STANDARD,
            &NeverCancelled,
        )
        .expect("structured severing cut");

        assert_eq!(outcome.value.bodies.len(), 2);
        assert_eq!(outcome.value.face_history.len(), 2);
        assert!(outcome.validation.is_valid());
        assert!(outcome.history.validate().is_ok());
        assert!(outcome
            .history
            .coverage_for_solids(&outcome.value.bodies)
            .is_complete());
        for (body, history) in outcome.value.bodies.iter().zip(&outcome.value.face_history) {
            assert!(body.is_watertight_with_policy(&TolerancePolicy::STANDARD));
            assert!(body
                .health_report_with_policy(&TolerancePolicy::STANDARD)
                .is_healthy());
            assert_eq!(history.face_source.len(), body.face_count());
            assert!(history.face_source.iter().all(Option::is_some));
        }
    }

    #[test]
    fn test_boolean_union_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Fuse (Union)
        let fuse = boolean(&cube1, &cube2, BooleanOp::Fuse);
        // The result bounding box should be [0, 0, 0] to [15, 10, 10]
        let (lo, hi) = fuse.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 15.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        if !fuse.is_watertight() {
            println!("All faces in the union solid:");
            for (idx, face) in fuse.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!(
                            "    Edge: {:?} -> {:?}",
                            edge.start().point(),
                            edge.end().point()
                        );
                    }
                }
            }
        }
        assert!(fuse.is_watertight(), "union result must be watertight");
    }
}
