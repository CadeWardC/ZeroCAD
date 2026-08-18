//! Stable semantic document contracts shared by evaluation, persistence, and UI.
//!
//! The runtime [`crate::parametric::ParametricGraph`] remains the efficient
//! petgraph-backed evaluator input.  This module owns the durable meaning that
//! must not depend on petgraph arena order or Rust enum discriminants.

use crate::parametric::{DatumAxisDef, DatumPlaneDef, DatumPointDef, ExtrudeMode, FeatureType};
use crate::{ParametricGraph, Unit};
use std::collections::BTreeMap;

macro_rules! string_id {
    ($name:ident) => {
        #[derive(
            Debug,
            Clone,
            Default,
            PartialEq,
            Eq,
            PartialOrd,
            Ord,
            Hash,
            serde::Serialize,
            serde::Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Self {
                Self(value.into())
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl From<String> for $name {
            fn from(value: String) -> Self {
                Self(value)
            }
        }

        impl From<&str> for $name {
            fn from(value: &str) -> Self {
                Self(value.to_owned())
            }
        }

        impl From<$name> for String {
            fn from(value: $name) -> Self {
                value.0
            }
        }

        impl From<&$name> for String {
            fn from(value: &$name) -> Self {
                value.0.clone()
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl AsRef<str> for $name {
            fn as_ref(&self) -> &str {
                self.as_str()
            }
        }

        impl std::borrow::Borrow<str> for $name {
            fn borrow(&self) -> &str {
                self.as_str()
            }
        }

        impl PartialEq<str> for $name {
            fn eq(&self, other: &str) -> bool {
                self.as_str() == other
            }
        }

        impl PartialEq<&str> for $name {
            fn eq(&self, other: &&str) -> bool {
                self.as_str() == *other
            }
        }

        impl PartialEq<String> for $name {
            fn eq(&self, other: &String) -> bool {
                self.as_str() == other
            }
        }

        impl PartialEq<$name> for str {
            fn eq(&self, other: &$name) -> bool {
                self == other.as_str()
            }
        }

        impl PartialEq<$name> for &str {
            fn eq(&self, other: &$name) -> bool {
                *self == other.as_str()
            }
        }

        impl PartialEq<$name> for String {
            fn eq(&self, other: &$name) -> bool {
                self.as_str() == other.as_str()
            }
        }

        impl std::ops::Deref for $name {
            type Target = str;

            fn deref(&self) -> &Self::Target {
                self.as_str()
            }
        }
    };
}

string_id!(FeatureId);
string_id!(BodyId);

/// Monotonic identity of an authoritative in-memory document state.
///
/// Revisions are deliberately not serialized. They exist only to prevent
/// background evaluation writebacks from crossing an intervening edit.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DocumentRevision(u64);

impl DocumentRevision {
    pub const INITIAL: Self = Self(0);

    pub fn get(self) -> u64 {
        self.0
    }

    pub fn next(self) -> Self {
        Self(self.0.wrapping_add(1))
    }
}
string_id!(FeatureKindId);

/// Stable ordering key inside a body's intended history.
///
/// Dependencies still decide whether evaluation is legal; this key decides the
/// order in which independent operations are intended to affect a body.
#[derive(
    Debug,
    Clone,
    Copy,
    Default,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    serde::Serialize,
    serde::Deserialize,
)]
#[serde(transparent)]
pub struct SequenceKey(pub u64);

/// Evaluation state. Visibility is intentionally absent: hiding is presentation
/// state and must never change the geometry recipe.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FeatureState {
    #[default]
    Active,
    Suppressed,
}

/// What a feature input resolves to. Direct feature dependencies remain exact
/// graph references; all model-entity inputs use the shared semantic selector.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FeatureInputTarget {
    Feature(FeatureId),
    Selection(Box<SemanticSelector>),
}

/// One explicitly named input slot. A role is stable document data (for
/// example `profile`, `target`, or `tool`) rather than an edge insertion order.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FeatureInput {
    pub role: String,
    pub target: FeatureInputTarget,
}

impl FeatureInput {
    pub fn selection(role: impl Into<String>, selector: SemanticSelector) -> Self {
        Self {
            role: role.into(),
            target: FeatureInputTarget::Selection(Box::new(selector)),
        }
    }

    pub fn feature(role: impl Into<String>, id: impl Into<FeatureId>) -> Self {
        Self {
            role: role.into(),
            target: FeatureInputTarget::Feature(id.into()),
        }
    }

    pub fn body(role: impl Into<String>, id: impl Into<BodyId>) -> Self {
        Self {
            role: role.into(),
            target: FeatureInputTarget::Selection(Box::new(SemanticSelector::body(id))),
        }
    }

    pub fn sketch(role: impl Into<String>, id: impl Into<FeatureId>) -> Self {
        Self {
            role: role.into(),
            target: FeatureInputTarget::Selection(Box::new(SemanticSelector::sketch(id))),
        }
    }

    pub fn datum(role: impl Into<String>, id: impl Into<FeatureId>) -> Self {
        Self {
            role: role.into(),
            target: FeatureInputTarget::Selection(Box::new(SemanticSelector::datum(id))),
        }
    }
}

/// A stable body and its explicit ordered feature timeline.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BodyRecord {
    pub id: BodyId,
    pub name: String,
    pub timeline: Vec<FeatureId>,
}

/// Cross-feature semantic document data. Feature contracts live directly on
/// the authoritative runtime records; only body timelines need a shared map.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DocumentSemantics {
    pub bodies: BTreeMap<BodyId, BodyRecord>,
    /// Runtime body output id -> the feature that produced it. This replaces
    /// ownership guesses based on parsing display ids.
    #[serde(default)]
    pub body_outputs: BTreeMap<String, FeatureId>,
}

/// The durable entity classes accepted by the shared selector contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SemanticEntityKind {
    Body,
    Face,
    Edge,
    Sketch,
    SketchEntity,
    Datum,
    Vertex,
}

/// Stable topology identity, independent of geometric fallback data.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SelectionTopology {
    pub body: Option<BodyId>,
    pub component_id: Option<String>,
    pub topology_version: Option<u64>,
    pub entity_id: Option<String>,
    pub adjacent_entity_ids: Vec<String>,
}

/// Operation lineage used when an exact topology name was legitimately
/// replaced. `feature` names the creating operation and `source_entity_id`
/// records the input identity when one exists.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SelectionProvenance {
    pub feature: Option<FeatureId>,
    pub source_entity_id: Option<String>,
}

/// Geometry is deliberately the final fallback, never the primary identity.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum GeometricIntent {
    Body,
    Face {
        centroid: [f32; 3],
        normal: [f32; 3],
        surface_kind: Option<String>,
    },
    Edge {
        p0: [f32; 3],
        p1: [f32; 3],
        adjacent_normals: [[f32; 3]; 2],
        curve_kind: Option<String>,
    },
    SketchEntity {
        sketch: FeatureId,
        entity_id: u32,
    },
    Sketch {
        sketch: FeatureId,
    },
    Datum {
        datum: FeatureId,
    },
    Vertex {
        point: [f32; 3],
    },
}

/// One reference contract for every selector-bearing feature.
///
/// Resolvers use [`SemanticSelector::resolution_order`] and must suspend the
/// feature when a durable identity exists but cannot be resolved. Geometry is
/// only eligible for legacy/unnamed captures, preventing silent retargeting.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SemanticSelector {
    pub kind: SemanticEntityKind,
    pub topology: SelectionTopology,
    pub provenance: SelectionProvenance,
    pub semantic_role: Option<String>,
    pub intent: GeometricIntent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectorResolutionTier {
    DurableTopology,
    OperationProvenance,
    SemanticRole,
    GeometricIntent,
}

impl SemanticSelector {
    pub fn body(body: impl Into<BodyId>) -> Self {
        let body = body.into();
        Self {
            kind: SemanticEntityKind::Body,
            topology: SelectionTopology {
                body: Some(body.clone()),
                entity_id: Some(body.to_string()),
                ..SelectionTopology::default()
            },
            provenance: SelectionProvenance::default(),
            semantic_role: None,
            intent: GeometricIntent::Body,
        }
    }

    pub fn sketch_entity(sketch: impl Into<FeatureId>, entity_id: u32) -> Self {
        let sketch = sketch.into();
        Self {
            kind: SemanticEntityKind::SketchEntity,
            topology: SelectionTopology {
                entity_id: Some(format!("{}:{entity_id}", sketch.as_str())),
                ..SelectionTopology::default()
            },
            provenance: SelectionProvenance {
                feature: Some(sketch.clone()),
                source_entity_id: None,
            },
            semantic_role: None,
            intent: GeometricIntent::SketchEntity { sketch, entity_id },
        }
    }

    pub fn sketch(sketch: impl Into<FeatureId>) -> Self {
        let sketch = sketch.into();
        Self {
            kind: SemanticEntityKind::Sketch,
            topology: SelectionTopology {
                entity_id: Some(sketch.to_string()),
                ..SelectionTopology::default()
            },
            provenance: SelectionProvenance {
                feature: Some(sketch.clone()),
                source_entity_id: None,
            },
            semantic_role: Some("sketch".to_owned()),
            intent: GeometricIntent::Sketch { sketch },
        }
    }

    pub fn datum(datum: impl Into<FeatureId>) -> Self {
        let datum = datum.into();
        Self {
            kind: SemanticEntityKind::Datum,
            topology: SelectionTopology {
                entity_id: Some(datum.to_string()),
                ..SelectionTopology::default()
            },
            provenance: SelectionProvenance {
                feature: Some(datum.clone()),
                source_entity_id: None,
            },
            semantic_role: None,
            intent: GeometricIntent::Datum { datum },
        }
    }

    pub fn from_face(face: &crate::parametric::FaceRef) -> Self {
        let topology = face.topology.as_ref();
        let entity_id = topology.and_then(|topology| topology.face_id.clone());
        Self {
            kind: SemanticEntityKind::Face,
            topology: SelectionTopology {
                body: topology
                    .and_then(|topology| topology.body_id.as_deref())
                    .map(BodyId::from),
                component_id: topology.and_then(|topology| topology.component_id.clone()),
                topology_version: topology.and_then(|topology| topology.topology_version),
                entity_id: entity_id.clone(),
                adjacent_entity_ids: Vec::new(),
            },
            provenance: SelectionProvenance {
                feature: topology
                    .and_then(|topology| topology.producer_feature_id.as_deref())
                    .map(FeatureId::from),
                source_entity_id: topology.and_then(|topology| topology.source_entity_id.clone()),
            },
            semantic_role: topology.and_then(|topology| topology.surface_kind.clone()),
            intent: GeometricIntent::Face {
                centroid: face.centroid,
                normal: face.normal,
                surface_kind: topology.and_then(|topology| topology.surface_kind.clone()),
            },
        }
    }

    pub fn from_edge(edge: &crate::parametric::EdgeRef) -> Self {
        let topology = edge.topology.as_ref();
        let entity_id = topology.and_then(|topology| topology.edge_id.clone());
        let curve_kind = topology
            .and_then(|topology| topology.curve_kind.clone())
            .or_else(|| match edge.curve.as_ref() {
                Some(crate::mock_kernel::EdgeCurveHint::Circle { .. }) => Some("circle".into()),
                Some(crate::mock_kernel::EdgeCurveHint::Line) => Some("line".into()),
                None => None,
            });
        Self {
            kind: SemanticEntityKind::Edge,
            topology: SelectionTopology {
                body: topology
                    .and_then(|topology| topology.body_id.as_deref())
                    .map(BodyId::from),
                component_id: None,
                topology_version: topology.and_then(|topology| topology.topology_version),
                entity_id: entity_id.clone(),
                adjacent_entity_ids: topology
                    .map(|topology| topology.adjacent_face_ids.clone())
                    .unwrap_or_default(),
            },
            provenance: SelectionProvenance {
                feature: topology
                    .and_then(|topology| topology.producer_feature_id.as_deref())
                    .map(FeatureId::from),
                source_entity_id: topology.and_then(|topology| topology.source_entity_id.clone()),
            },
            semantic_role: curve_kind.clone(),
            intent: GeometricIntent::Edge {
                p0: edge.p0,
                p1: edge.p1,
                adjacent_normals: [edge.n1, edge.n2],
                curve_kind,
            },
        }
    }

    pub fn from_vertex(vertex: &crate::parametric::VertexRef) -> Self {
        let topology = vertex.topology.as_ref();
        let mut incident_edge_ids = topology
            .map(|topology| topology.incident_edge_ids.clone())
            .unwrap_or_default();
        incident_edge_ids.sort();
        incident_edge_ids.dedup();
        let mut incident_face_ids = topology
            .map(|topology| topology.incident_face_ids.clone())
            .unwrap_or_default();
        incident_face_ids.sort();
        incident_face_ids.dedup();
        let entity_id = if !incident_face_ids.is_empty() {
            Some(format!("vertex:faces:{}", incident_face_ids.join("|")))
        } else {
            (!incident_edge_ids.is_empty())
                .then(|| format!("vertex:edges:{}", incident_edge_ids.join("|")))
        };
        Self {
            kind: SemanticEntityKind::Vertex,
            topology: SelectionTopology {
                body: topology
                    .and_then(|topology| topology.body_id.as_deref())
                    .map(BodyId::from),
                component_id: None,
                topology_version: topology.and_then(|topology| topology.topology_version),
                entity_id,
                adjacent_entity_ids: if incident_face_ids.is_empty() {
                    incident_edge_ids
                } else {
                    incident_face_ids
                },
            },
            provenance: SelectionProvenance {
                feature: topology
                    .and_then(|topology| topology.producer_feature_id.as_deref())
                    .map(FeatureId::from),
                source_entity_id: topology.and_then(|topology| topology.source_entity_id.clone()),
            },
            semantic_role: Some("vertex".to_owned()),
            intent: GeometricIntent::Vertex {
                point: vertex.point,
            },
        }
    }

    pub fn applies_to_body(&self, body_id: &str) -> bool {
        self.topology
            .body
            .as_ref()
            .is_none_or(|body| body.as_str() == body_id)
    }

    /// Resolution precedence shared by all entity classes. A named topology
    /// reference intentionally omits the geometric tier: named references fail
    /// loud instead of jumping to a nearby entity.
    pub fn resolution_order(&self) -> Vec<SelectorResolutionTier> {
        let mut tiers = Vec::with_capacity(4);
        if self.topology.entity_id.is_some() {
            tiers.push(SelectorResolutionTier::DurableTopology);
        }
        if self.provenance.feature.is_some() {
            tiers.push(SelectorResolutionTier::OperationProvenance);
        }
        if self.semantic_role.is_some() {
            tiers.push(SelectorResolutionTier::SemanticRole);
        }
        if self.topology.entity_id.is_none() {
            tiers.push(SelectorResolutionTier::GeometricIntent);
        }
        tiers
    }
}

/// Small authoritative document state. Visibility is presentation state;
/// suppression remains on each feature's semantic record because it changes
/// evaluation.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct DocumentState {
    pub units: Unit,
    /// Creation time is assigned once by the application and preserved on load
    /// and subsequent saves. `None` keeps deterministic library-only documents.
    pub created_unix: Option<u64>,
    /// Entity id -> visible. Missing entries mean visible.
    pub visibility: BTreeMap<String, bool>,
}

impl Default for DocumentState {
    fn default() -> Self {
        Self {
            units: Unit::Millimeter,
            created_unix: None,
            visibility: BTreeMap::new(),
        }
    }
}

/// Authoritative editable project. Runtime geometry and caches are deliberately
/// not members of this type.
#[derive(Debug, Clone)]
pub struct Document {
    runtime: ParametricGraph,
    pub state: DocumentState,
    revision: DocumentRevision,
}

impl Default for Document {
    fn default() -> Self {
        Self::new()
    }
}

impl Document {
    pub fn new() -> Self {
        Self {
            runtime: ParametricGraph::new(),
            state: DocumentState::default(),
            revision: DocumentRevision::INITIAL,
        }
    }

    pub fn from_graph(graph: ParametricGraph, units: Unit) -> Self {
        Self {
            runtime: graph,
            state: DocumentState {
                units,
                created_unix: None,
                visibility: BTreeMap::new(),
            },
            revision: DocumentRevision::INITIAL,
        }
    }

    /// Clone only persisted, authoritative document state.
    ///
    /// A normal [`Clone`] intentionally shares the immutable evaluator cache so
    /// workers can hand warm geometry between equivalent revisions cheaply.
    /// Undo, redo, autosave, and recovery snapshots must use this method: those
    /// snapshots are history, not evaluator checkpoints, and must stay bounded
    /// independently of the amount of cached geometry.
    pub fn clone_authoritative(&self) -> Self {
        Self {
            runtime: self.runtime.clone_document(),
            state: self.state.clone(),
            revision: self.revision,
        }
    }

    pub fn is_visible(&self, id: &str) -> bool {
        self.state.visibility.get(id).copied().unwrap_or(true)
    }

    pub fn set_visible(&mut self, id: impl Into<String>, visible: bool) {
        let id = id.into();
        let before = self.is_visible(&id);
        if visible {
            self.state.visibility.remove(&id);
        } else {
            self.state.visibility.insert(id, false);
        }
        if before != visible {
            self.mark_edited();
        }
    }

    pub fn hidden_entities(&self) -> std::collections::HashSet<String> {
        self.state
            .visibility
            .iter()
            .filter_map(|(id, visible)| (!*visible).then_some(id.clone()))
            .collect()
    }

    /// Borrow the evaluator projection owned by this document.
    ///
    /// Phase 3 makes `Document` the application/edit-session root while keeping
    /// `ParametricGraph` as the optimized runtime representation.  These
    /// accessors make that ownership boundary explicit for workers and
    /// persistence code without forcing UI code to know how the projection is
    /// stored.
    pub fn evaluator_graph(&self) -> &ParametricGraph {
        &self.runtime
    }

    pub fn evaluator_graph_mut(&mut self) -> &mut ParametricGraph {
        self.mark_edited();
        &mut self.runtime
    }

    pub fn into_evaluator_graph(self) -> ParametricGraph {
        self.runtime
    }

    pub fn revision(&self) -> DocumentRevision {
        self.revision
    }

    pub fn mark_edited(&mut self) -> DocumentRevision {
        self.revision = self.revision.next();
        *self.runtime.pending_legacy_backfills.borrow_mut() =
            crate::parametric::LegacyReferenceBackfills::for_revision(self.revision);
        self.revision
    }

    pub fn evaluate_request(
        &self,
        hidden: &std::collections::HashSet<String>,
        quality: crate::parametric::EvaluationQuality,
        cancellation: &crate::parametric::EvaluationCancellation,
    ) -> Result<crate::parametric::EvaluationOutput, crate::parametric::EvaluationError> {
        self.runtime
            .evaluate_request_at_revision(hidden, quality, cancellation, self.revision)
    }

    /// Commit evaluator writebacks only when they were produced from this exact
    /// authoritative state. A successful writeback becomes a new revision.
    pub fn apply_face_reattach_updates(
        &mut self,
        pending: crate::parametric::FaceReattach,
    ) -> bool {
        if pending.producing_revision() != self.revision {
            return false;
        }
        if self.runtime.apply_face_reattach_updates(pending) {
            self.revision = self.revision.next();
            self.runtime
                .pending_legacy_backfills
                .borrow_mut()
                .rebind_revision(self.revision);
            true
        } else {
            false
        }
    }

    /// Accept unique legacy matches from a background evaluation without
    /// changing persisted state. A later explicit migration/save may commit.
    pub fn queue_legacy_reference_backfills(
        &self,
        pending: crate::parametric::LegacyReferenceBackfills,
    ) -> bool {
        if pending.producing_revision() != self.revision {
            return false;
        }
        self.runtime.queue_legacy_reference_backfills(pending)
    }

    pub fn apply_legacy_reference_migrations(&mut self) -> bool {
        if self
            .runtime
            .pending_legacy_backfills
            .borrow()
            .producing_revision()
            != self.revision
        {
            return false;
        }
        if self.runtime.apply_legacy_reference_migrations() {
            self.mark_edited();
            true
        } else {
            false
        }
    }
}

impl std::ops::Deref for Document {
    type Target = ParametricGraph;

    fn deref(&self) -> &Self::Target {
        self.evaluator_graph()
    }
}

impl std::ops::DerefMut for Document {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.evaluator_graph_mut()
    }
}

fn input_target_key(target: &FeatureInputTarget) -> (&'static str, String) {
    match target {
        FeatureInputTarget::Feature(id) => ("feature", id.to_string()),
        FeatureInputTarget::Selection(selector) => (
            "selection",
            selector
                .topology
                .entity_id
                .clone()
                .or_else(|| {
                    selector
                        .provenance
                        .feature
                        .as_ref()
                        .map(ToString::to_string)
                })
                .unwrap_or_default(),
        ),
    }
}

pub(crate) fn body_for_feature(id: &str, feature: &FeatureType) -> Option<BodyId> {
    let own_body = || Some(BodyId::from(id));
    match feature {
        FeatureType::Box { .. }
        | FeatureType::Cylinder { .. }
        | FeatureType::Import { .. }
        | FeatureType::Pattern { .. }
        | FeatureType::BodyTransform { .. }
        | FeatureType::BodyJoin { .. }
        | FeatureType::BodyCut { .. }
        | FeatureType::BodyIntersect { .. }
        | FeatureType::BodySplit { .. }
        | FeatureType::BodyScale { .. }
        | FeatureType::FaceOffset { .. }
        | FeatureType::FaceMove { .. }
        | FeatureType::FaceDelete { .. }
        | FeatureType::FaceThicken { .. }
        | FeatureType::ImportStl { .. } => own_body(),
        FeatureType::Extrude { mode, target, .. }
        | FeatureType::Revolve { mode, target, .. }
        | FeatureType::Loft { mode, target, .. }
        | FeatureType::Sweep { mode, target, .. } => match mode {
            ExtrudeMode::NewBody => own_body(),
            ExtrudeMode::Join | ExtrudeMode::Cut => target.as_deref().map(BodyId::from),
        },
        FeatureType::EdgeMod { target, .. }
        | FeatureType::EdgeBlend { target, .. }
        | FeatureType::Shell { target, .. }
        | FeatureType::Hole { target, .. }
        | FeatureType::Thread { target, .. } => Some(BodyId::from(target.as_str())),
        FeatureType::FeaturePattern { target, .. } | FeatureType::Draft { target, .. } => {
            Some(BodyId::from(target.as_str()))
        }
        FeatureType::Origin
        | FeatureType::Sketch { .. }
        | FeatureType::VariableSet { .. }
        | FeatureType::DatumPlane { .. }
        | FeatureType::DatumAxis { .. }
        | FeatureType::DatumPoint { .. } => None,
    }
}

/// Reconstruct the complete semantic input contract from the runtime payload
/// and the dependency DAG. Keeping this in the semantic module gives
/// registration, strict validation, and explicit edit synchronization one
/// canonical comparison target while the Phase 2 sidecar remains in place.
pub(crate) fn feature_inputs_for_runtime(
    feature: &FeatureType,
    dependency_ids: &[String],
) -> Vec<FeatureInput> {
    let mut inputs = intrinsic_inputs(feature);
    inputs.extend(
        dependency_ids
            .iter()
            .map(|id| FeatureInput::feature("dependency", FeatureId::from(id.as_str()))),
    );
    inputs.sort_by(|a, b| {
        (a.role.as_str(), input_target_key(&a.target))
            .cmp(&(b.role.as_str(), input_target_key(&b.target)))
    });
    inputs.dedup();
    inputs
}

fn intrinsic_inputs(feature: &FeatureType) -> Vec<FeatureInput> {
    let mut inputs = match feature {
        FeatureType::EdgeMod { target, .. }
        | FeatureType::EdgeBlend { target, .. }
        | FeatureType::Shell { target, .. }
        | FeatureType::Hole { target, .. }
        | FeatureType::Thread { target, .. } => {
            vec![FeatureInput::body("target", BodyId::from(target.as_str()))]
        }
        FeatureType::Pattern { source, .. }
        | FeatureType::BodyTransform { source, .. }
        | FeatureType::BodyScale { source, .. } => {
            vec![FeatureInput::body("source", BodyId::from(source.as_str()))]
        }
        FeatureType::FeaturePattern {
            target,
            source_feature,
            ..
        } => vec![
            FeatureInput::body("target", BodyId::from(target.as_str())),
            FeatureInput::feature("source_feature", FeatureId::from(source_feature.as_str())),
        ],
        FeatureType::Draft {
            target,
            faces,
            neutral,
            ..
        } => {
            let mut inputs = vec![FeatureInput::body("target", BodyId::from(target.as_str()))];
            inputs.extend(faces.iter().map(|face| {
                FeatureInput::selection("draft_face", SemanticSelector::from_face(face))
            }));
            inputs.push(match neutral {
                crate::parametric::DraftNeutral::Face(face) => {
                    FeatureInput::selection("neutral_face", SemanticSelector::from_face(face))
                }
                crate::parametric::DraftNeutral::Datum(datum) => {
                    FeatureInput::datum("neutral_datum", FeatureId::from(datum.as_str()))
                }
            });
            inputs
        }
        FeatureType::BodyJoin { sources } => sources
            .iter()
            .map(|source| FeatureInput::body("source", BodyId::from(source.as_str())))
            .collect(),
        FeatureType::BodyCut { target, tool, .. }
        | FeatureType::BodyIntersect { target, tool, .. } => vec![
            FeatureInput::body("target", BodyId::from(target.as_str())),
            FeatureInput::body("tool", BodyId::from(tool.as_str())),
        ],
        FeatureType::BodySplit { target, face, .. } => {
            let mut inputs = vec![FeatureInput::body("target", BodyId::from(target.as_str()))];
            if let Some(face) = face {
                inputs.push(FeatureInput::selection(
                    "plane_face",
                    SemanticSelector::from_face(face),
                ));
            }
            inputs
        }
        FeatureType::FaceOffset { target, face, .. }
        | FeatureType::FaceMove { target, face, .. }
        | FeatureType::FaceDelete { target, face }
        | FeatureType::FaceThicken { target, face, .. } => vec![
            FeatureInput::body("target", BodyId::from(target.as_str())),
            FeatureInput::selection("face", SemanticSelector::from_face(face)),
        ],
        FeatureType::Loft { sections, .. } => sections
            .iter()
            .map(|(sketch, _)| FeatureInput::sketch("section", FeatureId::from(sketch.as_str())))
            .collect(),
        FeatureType::Sweep {
            profile_sketch,
            path_sketch,
            guide,
            ..
        } => {
            let mut inputs = vec![
                FeatureInput::sketch("profile", FeatureId::from(profile_sketch.as_str())),
                FeatureInput::sketch("path", FeatureId::from(path_sketch.as_str())),
            ];
            if let Some(guide) = guide {
                inputs.push(FeatureInput::sketch(
                    "guide",
                    FeatureId::from(guide.sketch.as_str()),
                ));
                inputs.push(FeatureInput::selection(
                    "profile_anchor",
                    SemanticSelector::sketch_entity(
                        FeatureId::from(profile_sketch.as_str()),
                        guide.profile_entity.0,
                    ),
                ));
            }
            inputs
        }
        FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace { face },
        } => vec![FeatureInput::selection(
            "planar_face",
            SemanticSelector::from_face(face),
        )],
        FeatureType::DatumAxis { def } => match def {
            DatumAxisDef::Edge { edge } => vec![FeatureInput::selection(
                "axis_edge",
                SemanticSelector::from_edge(edge),
            )],
            DatumAxisDef::CylindricalOrConicalFace { face } => vec![FeatureInput::selection(
                "axis_face",
                SemanticSelector::from_face(face),
            )],
            DatumAxisDef::TwoVertices { a, b } => vec![
                FeatureInput::selection("vertex_a", SemanticSelector::from_vertex(a)),
                FeatureInput::selection("vertex_b", SemanticSelector::from_vertex(b)),
            ],
            DatumAxisDef::TwoPoints { .. } | DatumAxisDef::PlaneIntersection { .. } => Vec::new(),
        },
        FeatureType::DatumPoint { def } => match def {
            DatumPointDef::Vertex { vertex } => vec![FeatureInput::selection(
                "vertex",
                SemanticSelector::from_vertex(vertex),
            )],
            DatumPointDef::Midpoint { a, b } => vec![
                FeatureInput::selection("vertex_a", SemanticSelector::from_vertex(a)),
                FeatureInput::selection("vertex_b", SemanticSelector::from_vertex(b)),
            ],
            DatumPointDef::EdgeMidpoint { edge } | DatumPointDef::CircleCenter { edge } => {
                vec![FeatureInput::selection(
                    "edge",
                    SemanticSelector::from_edge(edge),
                )]
            }
            DatumPointDef::Coords { .. } => Vec::new(),
        },
        _ => Vec::new(),
    };
    inputs.sort_by(|a, b| {
        (a.role.as_str(), input_target_key(&a.target))
            .cmp(&(b.role.as_str(), input_target_key(&b.target)))
    });
    inputs.dedup();
    inputs
}

/// One built-in feature registration. The string ID and payload version are the
/// persistence contract; UI labels may change without changing either.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeatureRegistration {
    pub kind_id: &'static str,
    /// Schema written for a newly saved feature of this kind.
    pub payload_version: u16,
    /// Every schema this build can decode, paired with its storage/decoder path.
    /// Historical schemas stay here after `payload_version` advances.
    pub payload_decoders: &'static [FeaturePayloadDecoderRegistration],
    pub display_name: &'static str,
    pub evaluator: FeatureEvaluatorKind,
    pub editor_group: FeatureEditorGroup,
}

impl FeatureRegistration {
    pub fn payload_decoder(&self, schema: u16) -> Option<FeaturePayloadDecoder> {
        self.payload_decoders
            .iter()
            .find(|registration| registration.schema == schema)
            .map(|registration| registration.decoder)
    }
}

/// Storage and decoding path for one feature-payload schema.
///
/// The enum makes the recipe reader dispatch on `(kind_id, payload_schema)`
/// before it interprets the payload envelope. Future schema-specific decoders
/// receive their own variant rather than changing the meaning of an old one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeaturePayloadDecoder {
    NumericFieldsV1,
    NumericFieldsV2,
    NumericFieldsV3,
    StepAssetV1,
    StlAssetV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeaturePayloadDecoderRegistration {
    pub schema: u16,
    pub decoder: FeaturePayloadDecoder,
}

const NUMERIC_FIELDS_V1: &[FeaturePayloadDecoderRegistration] =
    &[FeaturePayloadDecoderRegistration {
        schema: 1,
        decoder: FeaturePayloadDecoder::NumericFieldsV1,
    }];
const NUMERIC_FIELDS_V1_V2: &[FeaturePayloadDecoderRegistration] = &[
    FeaturePayloadDecoderRegistration {
        schema: 1,
        decoder: FeaturePayloadDecoder::NumericFieldsV1,
    },
    FeaturePayloadDecoderRegistration {
        schema: 2,
        decoder: FeaturePayloadDecoder::NumericFieldsV2,
    },
];
const NUMERIC_FIELDS_V1_V2_V3: &[FeaturePayloadDecoderRegistration] = &[
    FeaturePayloadDecoderRegistration {
        schema: 1,
        decoder: FeaturePayloadDecoder::NumericFieldsV1,
    },
    FeaturePayloadDecoderRegistration {
        schema: 2,
        decoder: FeaturePayloadDecoder::NumericFieldsV2,
    },
    FeaturePayloadDecoderRegistration {
        schema: 3,
        decoder: FeaturePayloadDecoder::NumericFieldsV3,
    },
];
const STEP_ASSET_V1: &[FeaturePayloadDecoderRegistration] = &[FeaturePayloadDecoderRegistration {
    schema: 1,
    decoder: FeaturePayloadDecoder::StepAssetV1,
}];
const STL_ASSET_V1: &[FeaturePayloadDecoderRegistration] = &[FeaturePayloadDecoderRegistration {
    schema: 1,
    decoder: FeaturePayloadDecoder::StlAssetV1,
}];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureEvaluatorKind {
    Infrastructure,
    Sketch,
    Datum,
    Box,
    Cylinder,
    Extrude,
    EdgeMod,
    EdgeBlend,
    Import,
    Revolve,
    Loft,
    Sweep,
    Shell,
    Hole,
    Pattern,
    FeaturePattern,
    BodyTransform,
    Thread,
    BodyJoin,
    BodyCut,
    BodyIntersect,
    BodySplit,
    BodyScale,
    FaceOffset,
    FaceMove,
    FaceDelete,
    FaceThicken,
    ImportStl,
    Draft,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureEditorGroup {
    Document,
    Sketch,
    Construct,
    Solid,
    Modify,
    Exchange,
}

pub struct FeatureRegistry;

impl FeatureRegistry {
    pub const BUILTINS: &'static [FeatureRegistration] = &[
        registration(
            "core.origin",
            "Origin",
            FeatureEvaluatorKind::Infrastructure,
            FeatureEditorGroup::Document,
        ),
        registration(
            "part.box",
            "Box",
            FeatureEvaluatorKind::Box,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.cylinder",
            "Cylinder",
            FeatureEvaluatorKind::Cylinder,
            FeatureEditorGroup::Solid,
        ),
        registration_with_payload(
            "sketch.sketch",
            3,
            NUMERIC_FIELDS_V1_V2_V3,
            "Sketch",
            FeatureEvaluatorKind::Sketch,
            FeatureEditorGroup::Sketch,
        ),
        registration_with_payload(
            "part.extrude",
            2,
            NUMERIC_FIELDS_V1_V2,
            "Extrude",
            FeatureEvaluatorKind::Extrude,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.edge_mod",
            "Fillet / Chamfer",
            FeatureEvaluatorKind::EdgeMod,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.edge_blend",
            "Edge Blend",
            FeatureEvaluatorKind::EdgeBlend,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "document.variables",
            "Variables",
            FeatureEvaluatorKind::Infrastructure,
            FeatureEditorGroup::Document,
        ),
        registration_with_payload(
            "exchange.step_import",
            1,
            STEP_ASSET_V1,
            "STEP Import",
            FeatureEvaluatorKind::Import,
            FeatureEditorGroup::Exchange,
        ),
        registration(
            "part.revolve",
            "Revolve",
            FeatureEvaluatorKind::Revolve,
            FeatureEditorGroup::Solid,
        ),
        registration_with_payload(
            "part.loft",
            2,
            NUMERIC_FIELDS_V1_V2,
            "Loft",
            FeatureEvaluatorKind::Loft,
            FeatureEditorGroup::Solid,
        ),
        registration_with_payload(
            "part.sweep",
            3,
            NUMERIC_FIELDS_V1_V2_V3,
            "Sweep",
            FeatureEvaluatorKind::Sweep,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.shell",
            "Shell",
            FeatureEvaluatorKind::Shell,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.hole",
            "Hole",
            FeatureEvaluatorKind::Hole,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.pattern",
            "Pattern",
            FeatureEvaluatorKind::Pattern,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.feature_pattern",
            "Feature Pattern",
            FeatureEvaluatorKind::FeaturePattern,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.transform",
            "Move / Copy",
            FeatureEvaluatorKind::BodyTransform,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.thread",
            "Thread",
            FeatureEvaluatorKind::Thread,
            FeatureEditorGroup::Modify,
        ),
        registration_with_payload(
            "datum.plane",
            2,
            NUMERIC_FIELDS_V1_V2,
            "Datum Plane",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration_with_payload(
            "datum.axis",
            2,
            NUMERIC_FIELDS_V1_V2,
            "Datum Axis",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration_with_payload(
            "datum.point",
            2,
            NUMERIC_FIELDS_V1_V2,
            "Datum Point",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration(
            "part.join",
            "Join",
            FeatureEvaluatorKind::BodyJoin,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.cut",
            "Cut",
            FeatureEvaluatorKind::BodyCut,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.intersect",
            "Intersect",
            FeatureEvaluatorKind::BodyIntersect,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.split",
            "Split Body",
            FeatureEvaluatorKind::BodySplit,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.scale",
            "Scale Body",
            FeatureEvaluatorKind::BodyScale,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "direct.face_offset",
            "Press / Pull Face",
            FeatureEvaluatorKind::FaceOffset,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "direct.face_move",
            "Move Face",
            FeatureEvaluatorKind::FaceMove,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "direct.face_delete",
            "Delete Face",
            FeatureEvaluatorKind::FaceDelete,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "direct.face_thicken",
            "Thicken Face",
            FeatureEvaluatorKind::FaceThicken,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.draft",
            "Draft",
            FeatureEvaluatorKind::Draft,
            FeatureEditorGroup::Modify,
        ),
        registration_with_payload(
            "exchange.stl_import",
            1,
            STL_ASSET_V1,
            "STL Import",
            FeatureEvaluatorKind::ImportStl,
            FeatureEditorGroup::Exchange,
        ),
    ];

    pub fn get(kind_id: &str) -> Option<&'static FeatureRegistration> {
        Self::BUILTINS
            .iter()
            .find(|registration| registration.kind_id == kind_id)
    }

    pub fn for_feature(feature: &FeatureType) -> &'static FeatureRegistration {
        Self::get(feature.kind_id()).expect("all built-in feature kinds must be registered")
    }

    /// Registry-owned dependency extraction. Evaluator graph edges may add
    /// scheduling constraints, but intrinsic semantic inputs always come from
    /// this one feature contract.
    pub fn dependencies(feature: &FeatureType) -> Vec<FeatureInput> {
        intrinsic_inputs(feature)
    }
}

const fn registration(
    kind_id: &'static str,
    display_name: &'static str,
    evaluator: FeatureEvaluatorKind,
    editor_group: FeatureEditorGroup,
) -> FeatureRegistration {
    registration_with_payload(
        kind_id,
        1,
        NUMERIC_FIELDS_V1,
        display_name,
        evaluator,
        editor_group,
    )
}

const fn registration_with_payload(
    kind_id: &'static str,
    payload_version: u16,
    payload_decoders: &'static [FeaturePayloadDecoderRegistration],
    display_name: &'static str,
    evaluator: FeatureEvaluatorKind,
    editor_group: FeatureEditorGroup,
) -> FeatureRegistration {
    FeatureRegistration {
        kind_id,
        payload_version,
        payload_decoders,
        display_name,
        evaluator,
        editor_group,
    }
}

impl FeatureType {
    pub fn kind_id(&self) -> &'static str {
        match self {
            FeatureType::Origin => "core.origin",
            FeatureType::Box { .. } => "part.box",
            FeatureType::Cylinder { .. } => "part.cylinder",
            FeatureType::Sketch { .. } => "sketch.sketch",
            FeatureType::Extrude { .. } => "part.extrude",
            FeatureType::EdgeMod { .. } => "part.edge_mod",
            FeatureType::EdgeBlend { .. } => "part.edge_blend",
            FeatureType::VariableSet { .. } => "document.variables",
            FeatureType::Import { .. } => "exchange.step_import",
            FeatureType::Revolve { .. } => "part.revolve",
            FeatureType::Loft { .. } => "part.loft",
            FeatureType::Sweep { .. } => "part.sweep",
            FeatureType::Shell { .. } => "part.shell",
            FeatureType::Hole { .. } => "part.hole",
            FeatureType::Pattern { .. } => "part.pattern",
            FeatureType::BodyTransform { .. } => "part.transform",
            FeatureType::Thread { .. } => "part.thread",
            FeatureType::DatumPlane { .. } => "datum.plane",
            FeatureType::DatumAxis { .. } => "datum.axis",
            FeatureType::DatumPoint { .. } => "datum.point",
            FeatureType::BodyJoin { .. } => "part.join",
            FeatureType::BodyCut { .. } => "part.cut",
            FeatureType::BodyIntersect { .. } => "part.intersect",
            FeatureType::BodySplit { .. } => "part.split",
            FeatureType::BodyScale { .. } => "part.scale",
            FeatureType::FaceOffset { .. } => "direct.face_offset",
            FeatureType::FaceMove { .. } => "direct.face_move",
            FeatureType::FaceDelete { .. } => "direct.face_delete",
            FeatureType::FaceThicken { .. } => "direct.face_thicken",
            FeatureType::Draft { .. } => "part.draft",
            FeatureType::ImportStl { .. } => "exchange.stl_import",
            FeatureType::FeaturePattern { .. } => "part.feature_pattern",
        }
    }

    pub fn payload_version(&self) -> u16 {
        FeatureRegistry::get(self.kind_id())
            .expect("all built-in feature kinds must be registered")
            .payload_version
    }
}
