//! Stable semantic document contracts shared by evaluation, persistence, and UI.
//!
//! The runtime [`crate::parametric::ParametricGraph`] remains the efficient
//! petgraph-backed evaluator input.  This module owns the durable meaning that
//! must not depend on petgraph arena order or Rust enum discriminants.

use crate::parametric::{body_output_owner_id, ExtrudeMode, FeatureType};
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

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

string_id!(FeatureId);
string_id!(BodyId);
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

/// Durable semantic metadata for one runtime feature node.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FeatureSemantics {
    pub kind_id: FeatureKindId,
    pub payload_version: u16,
    pub sequence: SequenceKey,
    pub inputs: Vec<FeatureInput>,
    pub state: FeatureState,
    pub body: Option<BodyId>,
}

/// A stable body and its explicit ordered feature timeline.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BodyRecord {
    pub id: BodyId,
    pub name: String,
    pub timeline: Vec<FeatureId>,
}

/// Semantic sidecar for the runtime evaluator graph. BTree maps keep iteration
/// deterministic for hashing and persistence.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DocumentSemantics {
    pub features: BTreeMap<FeatureId, FeatureSemantics>,
    pub bodies: BTreeMap<BodyId, BodyRecord>,
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
    pub graph: ParametricGraph,
    pub state: DocumentState,
}

impl Default for Document {
    fn default() -> Self {
        Self::new()
    }
}

impl Document {
    pub fn new() -> Self {
        Self {
            graph: ParametricGraph::new(),
            state: DocumentState::default(),
        }
    }

    pub fn from_graph(graph: ParametricGraph, units: Unit) -> Self {
        Self {
            graph,
            state: DocumentState {
                units,
                created_unix: None,
                visibility: BTreeMap::new(),
            },
        }
    }

    pub fn is_visible(&self, id: &str) -> bool {
        self.state.visibility.get(id).copied().unwrap_or(true)
    }

    pub fn set_visible(&mut self, id: impl Into<String>, visible: bool) {
        let id = id.into();
        if visible {
            self.state.visibility.remove(&id);
        } else {
            self.state.visibility.insert(id, false);
        }
    }

    pub fn hidden_entities(&self) -> std::collections::HashSet<String> {
        self.state
            .visibility
            .iter()
            .filter_map(|(id, visible)| (!*visible).then_some(id.clone()))
            .collect()
    }
}

impl DocumentSemantics {
    pub fn next_sequence(&self) -> SequenceKey {
        SequenceKey(
            self.features
                .values()
                .map(|feature| feature.sequence.0)
                .max()
                .unwrap_or(0)
                .saturating_add(1),
        )
    }

    pub(crate) fn register(
        &mut self,
        id: &str,
        name: &str,
        feature: &FeatureType,
        sequence: SequenceKey,
    ) {
        let feature_id = FeatureId::from(id);
        let body = body_for_feature(id, feature);
        let semantics = FeatureSemantics {
            kind_id: FeatureKindId::from(feature.kind_id()),
            payload_version: feature.payload_version(),
            sequence,
            inputs: FeatureRegistry::dependencies(feature),
            state: FeatureState::Active,
            body: body.clone(),
        };
        self.features.insert(feature_id.clone(), semantics);

        if let Some(body_id) = body {
            let record = self
                .bodies
                .entry(body_id.clone())
                .or_insert_with(|| BodyRecord {
                    id: body_id,
                    name: name.to_owned(),
                    timeline: Vec::new(),
                });
            if !record.timeline.contains(&feature_id) {
                record.timeline.push(feature_id);
            }
            record.timeline.sort_by_key(|member| {
                self.features
                    .get(member)
                    .map(|feature| feature.sequence)
                    .unwrap_or_default()
            });
        }
    }

    pub(crate) fn add_dependency(&mut self, parent: &str, child: &str) {
        let Some(feature) = self.features.get_mut(&FeatureId::from(child)) else {
            return;
        };
        let input = FeatureInput::feature("dependency", FeatureId::from(parent));
        if !feature.inputs.contains(&input) {
            feature.inputs.push(input);
            feature.inputs.sort_by(|a, b| {
                (a.role.as_str(), input_target_key(&a.target))
                    .cmp(&(b.role.as_str(), input_target_key(&b.target)))
            });
        }
    }

    pub(crate) fn remove(&mut self, id: &str) {
        let feature_id = FeatureId::from(id);
        self.features.remove(&feature_id);
        self.bodies.retain(|_, body| {
            body.timeline.retain(|member| member != &feature_id);
            !body.timeline.is_empty()
        });
        for feature in self.features.values_mut() {
            feature.inputs.retain(|input| match &input.target {
                FeatureInputTarget::Feature(target) => target != &feature_id,
                FeatureInputTarget::Selection(selector) => {
                    selector.provenance.feature.as_ref() != Some(&feature_id)
                }
            });
        }
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

fn body_for_feature(id: &str, feature: &FeatureType) -> Option<BodyId> {
    let own_body = || Some(BodyId::from(id));
    match feature {
        FeatureType::Box { .. }
        | FeatureType::Cylinder { .. }
        | FeatureType::Import { .. }
        | FeatureType::Pattern { .. }
        | FeatureType::BodyTransform { .. }
        | FeatureType::BodyJoin { .. } => own_body(),
        FeatureType::Extrude { mode, target, .. }
        | FeatureType::Revolve { mode, target, .. }
        | FeatureType::Loft { mode, target, .. }
        | FeatureType::Sweep { mode, target, .. } => match mode {
            ExtrudeMode::NewBody => own_body(),
            ExtrudeMode::Join | ExtrudeMode::Cut => target
                .as_deref()
                .map(body_output_owner_id)
                .map(BodyId::from),
        },
        FeatureType::EdgeMod { target, .. }
        | FeatureType::Shell { target, .. }
        | FeatureType::Hole { target, .. }
        | FeatureType::Thread { target, .. }
        | FeatureType::BodyCut { target, .. } => Some(BodyId::from(body_output_owner_id(target))),
        FeatureType::Origin
        | FeatureType::Sketch { .. }
        | FeatureType::VariableSet { .. }
        | FeatureType::DatumPlane { .. }
        | FeatureType::DatumAxis { .. }
        | FeatureType::DatumPoint { .. } => None,
    }
}

fn intrinsic_inputs(feature: &FeatureType) -> Vec<FeatureInput> {
    let mut inputs = match feature {
        FeatureType::EdgeMod { target, .. }
        | FeatureType::Shell { target, .. }
        | FeatureType::Hole { target, .. }
        | FeatureType::Thread { target, .. } => vec![FeatureInput::body(
            "target",
            BodyId::from(body_output_owner_id(target)),
        )],
        FeatureType::Pattern { source, .. } | FeatureType::BodyTransform { source, .. } => {
            vec![FeatureInput::body(
                "source",
                BodyId::from(body_output_owner_id(source)),
            )]
        }
        FeatureType::BodyJoin { sources } => sources
            .iter()
            .map(|source| FeatureInput::body("source", BodyId::from(body_output_owner_id(source))))
            .collect(),
        FeatureType::BodyCut { target, tool, .. } => vec![
            FeatureInput::body("target", BodyId::from(body_output_owner_id(target))),
            FeatureInput::body("tool", BodyId::from(body_output_owner_id(tool))),
        ],
        FeatureType::Loft { sections, .. } => sections
            .iter()
            .map(|(sketch, _)| FeatureInput::sketch("section", FeatureId::from(sketch.as_str())))
            .collect(),
        FeatureType::Sweep {
            profile_sketch,
            path_sketch,
            ..
        } => vec![
            FeatureInput::sketch("profile", FeatureId::from(profile_sketch.as_str())),
            FeatureInput::sketch("path", FeatureId::from(path_sketch.as_str())),
        ],
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
    pub payload_version: u16,
    pub display_name: &'static str,
    pub evaluator: FeatureEvaluatorKind,
    pub editor_group: FeatureEditorGroup,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureEvaluatorKind {
    Infrastructure,
    Sketch,
    Datum,
    Primitive,
    BodyOperation,
    Exchange,
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
            FeatureEvaluatorKind::Primitive,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.cylinder",
            "Cylinder",
            FeatureEvaluatorKind::Primitive,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "sketch.sketch",
            "Sketch",
            FeatureEvaluatorKind::Sketch,
            FeatureEditorGroup::Sketch,
        ),
        registration(
            "part.extrude",
            "Extrude",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.edge_mod",
            "Fillet / Chamfer",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "document.variables",
            "Variables",
            FeatureEvaluatorKind::Infrastructure,
            FeatureEditorGroup::Document,
        ),
        registration(
            "exchange.step_import",
            "STEP Import",
            FeatureEvaluatorKind::Exchange,
            FeatureEditorGroup::Exchange,
        ),
        registration(
            "part.revolve",
            "Revolve",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.loft",
            "Loft",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.sweep",
            "Sweep",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Solid,
        ),
        registration(
            "part.shell",
            "Shell",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.hole",
            "Hole",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.pattern",
            "Pattern",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.transform",
            "Move / Copy",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.thread",
            "Thread",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "datum.plane",
            "Datum Plane",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration(
            "datum.axis",
            "Datum Axis",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration(
            "datum.point",
            "Datum Point",
            FeatureEvaluatorKind::Datum,
            FeatureEditorGroup::Construct,
        ),
        registration(
            "part.join",
            "Join",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
        ),
        registration(
            "part.cut",
            "Cut",
            FeatureEvaluatorKind::BodyOperation,
            FeatureEditorGroup::Modify,
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
    FeatureRegistration {
        kind_id,
        payload_version: 1,
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
        }
    }

    pub fn payload_version(&self) -> u16 {
        FeatureRegistry::get(self.kind_id())
            .expect("all built-in feature kinds must be registered")
            .payload_version
    }
}
