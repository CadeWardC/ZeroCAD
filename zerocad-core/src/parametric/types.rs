use super::*;

/// How an extrude combines with the bodies already in the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ExtrudeMode {
    /// Create a standalone new body (the historical default).
    NewBody,
    /// Fuse (union) the extruded volume into any body it overlaps.
    Join,
    /// Subtract (difference) the extruded volume from any body in its path.
    Cut,
}

impl Default for ExtrudeMode {
    fn default() -> Self {
        ExtrudeMode::NewBody
    }
}

/// A single named, dimensioned value inside a [`FeatureType::VariableSet`].
/// `value` is expressed in `unit` (the same units offered in Settings), so the
/// UI can display it directly and convert to the base unit when needed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Variable {
    pub name: String,
    pub value: f64,
    pub unit: Unit,
}

impl Variable {
    /// A fresh variable with a placeholder name, zero value, and the given unit.
    pub fn new(name: impl Into<String>, unit: Unit) -> Self {
        Self {
            name: name.into(),
            value: 0.0,
            unit,
        }
    }

    /// The variable's value converted to the base unit (millimeters).
    pub fn value_in_base(&self) -> f64 {
        self.unit.to_base(self.value)
    }
}

/// Optional stable topology identity for a selected edge.
///
/// These fields are additive document metadata. Edge modifiers try this stable
/// identity first, then fall back to the captured world-space geometry for
/// legacy documents or genuinely changed topology.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TopologyEdgeRef {
    #[serde(default)]
    pub body_id: Option<String>,
    #[serde(default)]
    pub topology_version: Option<u64>,
    #[serde(default)]
    pub edge_id: Option<String>,
    #[serde(default)]
    pub adjacent_face_ids: Vec<String>,
    #[serde(default)]
    pub curve_kind: Option<String>,
    #[serde(default)]
    pub adjacent_surface_kinds: Vec<String>,
}

/// A solid edge captured geometrically for a 3D fillet/chamfer. The endpoints
/// and the two adjacent face normals are recorded in **world space** at
/// selection time (read straight from the body's wireframe — see
/// `MockMesh::edge_vertices` / `edge_face_normals`), which is all
/// [`crate::mock_kernel::edge_corner_cutter`] needs to orient its cutter.
///
/// The topology field lets an `EdgeMod` follow equivalent upstream dimension
/// edits. If the stable identity no longer resolves, the captured world-space
/// edge is still used as a legacy geometric fallback.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EdgeRef {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub n1: [f32; 3],
    pub n2: [f32; 3],
    #[serde(default)]
    pub curve: Option<EdgeCurveHint>,
    #[serde(default)]
    pub topology: Option<TopologyEdgeRef>,
}

/// Legacy serialized edge-mod span. The current fillet/chamfer tool no longer
/// exposes or evaluates separate full/partial modes; this remains only so older
/// `.zcad` documents with a `scope` field can still load.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum EdgeModScope {
    #[default]
    FullEdge,
    Partial {
        start_t: f32,
        end_t: f32,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum FeatureType {
    Origin,
    Box {
        w: f32,
        h: f32,
        d: f32,
    },
    Cylinder {
        r: f32,
        h: f32,
    },
    /// 2D sketch on a plane, holding the raw drawn curves. Region detection
    /// (the "faces" Fusion 360 auto-creates from intersecting shapes) is
    /// performed by the evaluator on demand.
    Sketch {
        /// The full plane the sketch lives on (origin + axes + normal), so a
        /// sketch can sit on an origin plane OR on an arbitrary body face.
        cs: crate::geometry::CoordinateSystem,
        /// Baked geometry. For parametric sketches this is a snapshot (the live
        /// geometry is rebuilt from `shapes`); for documents saved before
        /// `shapes` existed it is the authoritative geometry.
        curves: SketchCurves,
        /// Parametric construction: the shapes the sketch was drawn from, whose
        /// dimensions may reference variables. When non-empty this is the source
        /// of truth — the effective curves are rebuilt from it against the
        /// current variables (so a dimension follows its variable). Empty for
        /// legacy documents, which fall back to `curves`.
        #[serde(default)]
        shapes: Vec<crate::sketch::SketchShape>,
        /// Fillet/chamfer modifiers applied to corners after the shapes are
        /// built (see [`crate::sketch::effective_curves`]).
        #[serde(default)]
        corner_mods: Vec<crate::sketch::CornerMod>,
        /// True when the sketch was placed on an existing body face rather than
        /// an origin plane. Lets the extrude tool default to Join/Cut (combine
        /// with that body) instead of New Body. Defaults to `false` for
        /// documents saved before this field existed.
        #[serde(default)]
        on_face: bool,
    },
    /// Extrude one or more detected regions of the parent sketch by `depth`.
    /// `region_indices` selects which regions to extrude — empty means "all".
    /// `mode` decides whether the result is a new body, joined to existing
    /// bodies, or cut out of them.
    Extrude {
        depth: f32,
        region_indices: Vec<usize>,
        #[serde(default)]
        mode: ExtrudeMode,
        /// Optional expression (over the document's variables) that drives the
        /// depth. When set, it is re-evaluated against the current variables on
        /// every build, so editing a variable updates the extrude. `depth` then
        /// holds the last resolved value (a fallback when a variable is missing).
        #[serde(default)]
        depth_expr: Option<String>,
    },
    /// Round (fillet) or bevel (chamfer) one edge of an existing solid body by
    /// `dist`. Applied as a guarded boolean subtraction of an edge-aligned cutter
    /// during evaluation (see [`apply_edge_mod`]). `target` is the node id of the
    /// body it modifies; the modifier is processed after that body in creation
    /// order, like a cut extrude.
    EdgeMod {
        /// Node id of the body being modified.
        target: String,
        /// The edge to round/bevel, captured in world space.
        edge: EdgeRef,
        /// Fillet radius / chamfer setback, in base units (mm).
        dist: f32,
        /// Optional expression (over the document's variables) driving `dist`,
        /// re-evaluated every build. `dist` holds the last resolved value.
        #[serde(default)]
        dist_expr: Option<String>,
        /// Legacy field kept so older documents deserialize. New fillet/chamfer
        /// edits always use the selected edge as captured.
        #[serde(default)]
        scope: EdgeModScope,
        /// Whether to round (Fillet) or bevel (Chamfer) the edge.
        kind: crate::sketch::CornerKind,
    },
    /// A named collection of parametric variables. Carries no geometry — it's a
    /// container the user fills with dimensioned values for later reference.
    VariableSet {
        variables: Vec<Variable>,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeatureNode {
    pub id: String,
    pub name: String,
    pub feature: FeatureType,
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct ParametricGraph {
    pub graph: DiGraph<FeatureNode, ()>,
    #[serde(skip)]
    pub(crate) node_map: HashMap<String, NodeIndex>,
    /// Memoized planar-arrangement results, keyed by a content hash of a
    /// sketch's curves (see [`hash_curves`]). [`detect_regions`] is a pure,
    /// O(n²) function of the curves, so the same sketch yields the same regions
    /// every evaluation. Caching across calls keeps region detection off the
    /// hot path of extrude-drag previews, which re-evaluate the whole model on
    /// every frame while the sketches themselves never change. Skipped by serde
    /// and carried by `Clone` (so the per-frame graph clone in the preview path
    /// starts warm); it is a transparent accelerator, never persisted state.
    #[serde(skip)]
    pub(crate) region_cache: RefCell<HashMap<u64, Vec<Region>>>,
    /// Per-node geometry checkpoints from the previous evaluation, used to skip
    /// re-solving the unchanged prefix of the feature tree. Each entry holds the
    /// assembled bodies *after* one node, keyed by a cumulative content hash of
    /// every input that node's geometry depends on (see [`evaluate_bodies_inner`]).
    /// When an edit changes only a trailing node — e.g. dragging a fillet/chamfer
    /// radius — the prefix hashes still match, so the expensive upstream booleans
    /// are restored from here instead of recomputed every frame. Skipped by serde
    /// and a transparent accelerator (dropping it only costs a one-time rebuild).
    #[serde(skip)]
    pub(crate) eval_cache: RefCell<EvalCache>,
}

/// Checkpoints of [`evaluate_bodies_inner`], one per processed body node, in
/// creation order. A pure accelerator — see [`ParametricGraph::eval_cache`].
#[derive(Debug, Clone, Default)]
pub(crate) struct EvalCache {
    pub(crate) checkpoints: Vec<EvalCheckpoint>,
}

/// The assembled bodies and accumulated warnings immediately after one node was
/// applied, tagged with the cumulative hash of all geometry inputs up to it.
#[derive(Debug, Clone)]
pub(crate) struct EvalCheckpoint {
    pub(crate) key: u64,
    pub(crate) live: Vec<LiveBody>,
    pub(crate) warnings: Vec<String>,
}

#[derive(Debug, Clone)]
pub(crate) struct SketchEval {
    pub(crate) cs: CoordinateSystem,
    pub(crate) curves: SketchCurves,
    pub(crate) regions: Vec<Region>,
    pub(crate) provenance: Vec<RegionProvenance>,
    /// Full closed outlines of the drawn shapes (before region-splitting), used
    /// to combine overlapping shapes as a boolean at extrude time. Empty for
    /// legacy sketches (no `shapes`) or sketches with sketch fillets/chamfers
    /// (`corner_mods`), which fall back to the per-region extrude path.
    pub(crate) shape_loops: Vec<ShapeLoop>,
}
