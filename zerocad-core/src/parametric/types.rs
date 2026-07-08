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
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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

/// Optional stable topology identity for a selected/attached face — the face
/// analogue of [`TopologyEdgeRef`]. Faces are the primary named entity in
/// persistent topological naming (an edge is identified by the pair of faces it
/// separates), so a sketch-on-face placement or a cut/join target pins to this.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TopologyFaceRef {
    #[serde(default)]
    pub body_id: Option<String>,
    #[serde(default)]
    pub topology_version: Option<u64>,
    /// The face's durable name (its "owner"), e.g.
    /// `sketch:extrude_3:region:0:face:top`.
    #[serde(default)]
    pub face_id: Option<String>,
    #[serde(default)]
    pub surface_kind: Option<String>,
}

/// A solid face captured for reattachment: its centroid and outward normal in
/// **world space** at selection time, plus an optional stable [`TopologyFaceRef`]
/// name. A feature that targets a face (sketch-on-face, cut/join) resolves the
/// name first and falls back to the captured geometry only when unnamed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceRef {
    pub centroid: [f32; 3],
    pub normal: [f32; 3],
    #[serde(default)]
    pub topology: Option<TopologyFaceRef>,
}

/// How an edge modifier should use saved construction history.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum EdgeModReplayMode {
    /// Prefer construction replay when the target body carries replayable cut
    /// history; fall back to native edge modification for unsupported selections.
    #[default]
    Auto,
    /// Bypass construction replay. Used for native-only topology such as circular
    /// rim edges or when a future UI exposes an explicit escape hatch.
    NativeOnly,
}

/// Saved intent for reconstructing a fillet through earlier cuts.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EdgeModReplayIntent {
    #[serde(default)]
    pub mode: EdgeModReplayMode,
    #[serde(default)]
    pub pre_cut_target: Option<String>,
    #[serde(default)]
    pub replay_cut_nodes: Vec<String>,
    #[serde(default)]
    pub selected_span: Option<EdgeRef>,
}

impl Default for EdgeModReplayIntent {
    fn default() -> Self {
        Self {
            mode: EdgeModReplayMode::Auto,
            pre_cut_target: None,
            replay_cut_nodes: Vec::new(),
            selected_span: None,
        }
    }
}

impl EdgeModReplayIntent {
    pub fn auto_for(target: impl Into<String>, edge: EdgeRef) -> Self {
        Self {
            mode: EdgeModReplayMode::Auto,
            pre_cut_target: Some(target.into()),
            replay_cut_nodes: Vec::new(),
            selected_span: Some(edge),
        }
    }
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
        /// Durable id per entry of `shapes` (parallel Vec). Identity is the id,
        /// never the Vec position: deleting a shape must not re-key references
        /// downstream of its neighbours. Empty for legacy documents — they get
        /// positional backfill (`entity_ids[i] == i`) via
        /// [`crate::sketch::effective_shape_ids`], which reproduces the old
        /// grammar exactly.
        #[serde(default)]
        entity_ids: Vec<crate::sketch::EntityId>,
        /// Next unallocated [`crate::sketch::EntityId`] for this sketch.
        /// Monotonic; ids are never reused after a delete.
        #[serde(default)]
        next_entity_id: u32,
        /// Constraint-solver model (shared points + entities + constraints).
        /// `None` = legacy sketch: geometry comes from `shapes`/`curves`
        /// exactly as before this field existed. Serde-visible in full — the
        /// eval prefix cache hashes the serialized feature, so a hidden field
        /// here would reuse stale meshes after a constraint edit.
        #[serde(default)]
        solver: Option<crate::sketch::SketchSolverModel>,
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
        /// For Cut/Join: the node id of the body this boolean applies to.
        /// `None` (legacy and the default) keeps the historical behavior of
        /// hitting every AABB-overlapping body; `Some` restricts the boolean to
        /// that one body and reports Unresolved (fail-loud) when the body no
        /// longer exists instead of silently cutting whatever else is nearby.
        #[serde(default)]
        target: Option<String>,
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
        /// Construction-replay intent for fillets through earlier cuts.
        #[serde(default)]
        replay: EdgeModReplayIntent,
        /// Whether to round (Fillet) or bevel (Chamfer) the edge.
        kind: crate::sketch::CornerKind,
    },
    /// A named collection of parametric variables. Carries no geometry — it's a
    /// container the user fills with dimensioned values for later reference.
    VariableSet {
        variables: Vec<Variable>,
    },
    /// A body imported from a STEP file. The file's text is embedded so the
    /// `.zcad` document stays self-contained (no dangling path references) and
    /// the eval prefix cache hashes the actual geometry source.
    Import {
        /// The raw STEP (AP242) file contents.
        step_data: String,
        /// Display label, defaulting to the imported file's stem.
        label: String,
    },
    /// Revolve one or more detected regions of the parent sketch about an axis
    /// lying in the sketch plane. The revolve analogue of [`Self::Extrude`]:
    /// `region_indices` empty means "all", `mode` picks new-body / join / cut.
    Revolve {
        /// Revolution axis: a base axis, datum axis, or explicit segment.
        axis: AxisBase,
        /// Sweep angle in degrees, in `(0, 360]`.
        angle_deg: f32,
        /// Optional expression (over the document's variables) driving the
        /// angle; `angle_deg` holds the last resolved value.
        #[serde(default)]
        angle_expr: Option<String>,
        region_indices: Vec<usize>,
        #[serde(default)]
        mode: ExtrudeMode,
        /// For Cut/Join: the node id of the body the boolean applies to (see
        /// [`Self::Extrude::target`]).
        #[serde(default)]
        target: Option<String>,
    },
    /// Loft a solid through two or more sketch section profiles (in order).
    /// Each entry is `(sketch_node_id, region_index)`; the loft node depends on
    /// every section sketch. v1 lofts only the outer boundary of each section
    /// (holes ignored) with a ruled/planar skin.
    Loft {
        /// Ordered `(sketch_id, region_index)` sections.
        sections: Vec<(String, usize)>,
        #[serde(default)]
        mode: ExtrudeMode,
        #[serde(default)]
        target: Option<String>,
    },
    /// Sweep a profile region along a path sketch's open chain (rotation-
    /// minimizing frames — no twist). v1: the profile is placed perpendicular
    /// to the path start; the path must be a single open chain of lines/arcs.
    Sweep {
        /// The profile: `(sketch_id, region_index)`.
        profile_sketch: String,
        profile_region: usize,
        /// The path: a sketch id whose curves form one open chain.
        path_sketch: String,
        #[serde(default)]
        mode: ExtrudeMode,
        #[serde(default)]
        target: Option<String>,
    },
    /// Hollow out an existing body to a constant wall `thickness`, removing
    /// `open_faces` (at least one). Kernel support: boxes, cylinders, and any
    /// planar-faced solid with straight edges (extruded profiles); curved
    /// general shells report Unresolved rather than guessing.
    Shell {
        /// Node id of the body being hollowed.
        target: String,
        /// Wall thickness (mm), measured inward.
        thickness: f32,
        #[serde(default)]
        thickness_expr: Option<String>,
        /// The faces to remove (captured centroid+normal; resolved
        /// geometrically against the body at build time).
        open_faces: Vec<FaceRef>,
    },
    /// A drilled hole in an existing body: a cylinder cut, optionally with a
    /// counterbore or countersink, composed from the same guarded-boolean cut
    /// machinery as a Cut extrude. Placed at a point on a face, drilling along
    /// the inward face normal.
    Hole {
        /// Node id of the body being drilled.
        target: String,
        /// World-space point on the face where the hole starts.
        position: [f32; 3],
        /// Drilling direction (unit, INTO the material).
        direction: [f32; 3],
        /// Bore diameter (mm).
        diameter: f32,
        #[serde(default)]
        diameter_expr: Option<String>,
        /// Bore depth along `direction`; `None` = through-all.
        #[serde(default)]
        depth: Option<f32>,
        #[serde(default)]
        kind: HoleKind,
    },
    /// Replicate an existing BODY node's solids: linear/circular arrays and
    /// mirrors. v1 patterns whole bodies (the instances land as one new body);
    /// feature-level patterns (replicating a cut/hole across a face) build on
    /// this later.
    Pattern {
        /// Node id of the body being replicated.
        source: String,
        kind: PatternKind,
    },
    /// A reference plane (datum). Carries no geometry of its own — it resolves
    /// to a [`crate::geometry::CoordinateSystem`] during evaluation and exists
    /// so sketches (and later revolve axes, mirrors, …) can attach to a
    /// construction plane that isn't one of the three base planes.
    DatumPlane { def: DatumPlaneDef },
    /// A reference axis (datum), resolving to an origin + direction.
    DatumAxis { def: DatumAxisDef },
    /// A reference point (datum), resolving to a 3D position.
    DatumPoint { def: DatumPointDef },
}

/// A plane input to a datum definition: one of the three base planes or a
/// previously created datum plane (by node id). Datums may chain; cycles are
/// caught by the graph's toposort.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PlaneBase {
    XY,
    XZ,
    YZ,
    Datum(String),
}

/// An axis input to a datum definition.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AxisBase {
    X,
    Y,
    Z,
    /// A previously created datum axis (by node id).
    Datum(String),
    /// An explicit segment.
    TwoPoints { a: [f32; 3], b: [f32; 3] },
}

/// How a [`FeatureType::DatumPlane`] is constructed. All inputs are resolvable
/// without live body geometry (base planes, other datums, explicit points), so
/// datums evaluate in a pure pre-pass before the body loop.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DatumPlaneDef {
    /// `base` translated `distance` along its stored normal (handedness kept:
    /// the offset follows `cs.n`, never a recomputed `u × v`).
    Offset {
        base: PlaneBase,
        distance: f32,
        #[serde(default)]
        distance_expr: Option<String>,
    },
    /// `base` rotated `angle_deg` about `axis` (Rodrigues rotation of the
    /// frame's axes; the origin orbits the axis line).
    Angle {
        base: PlaneBase,
        axis: AxisBase,
        angle_deg: f32,
        #[serde(default)]
        angle_expr: Option<String>,
    },
    /// The plane through three points: origin `a`, `u` toward `b`, `v` the
    /// component of `c − a` orthogonal to `u`.
    ThreePoints {
        a: [f32; 3],
        b: [f32; 3],
        c: [f32; 3],
    },
    /// Halfway between two (parallel) planes: `a`'s axes at the midpoint of
    /// the two origins projected along `a`'s normal.
    MidPlane { a: PlaneBase, b: PlaneBase },
}

/// How a [`FeatureType::DatumAxis`] is constructed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DatumAxisDef {
    TwoPoints { a: [f32; 3], b: [f32; 3] },
    /// The intersection line of two non-parallel planes.
    PlaneIntersection { a: PlaneBase, b: PlaneBase },
}

/// How a [`FeatureType::DatumPoint`] is constructed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DatumPointDef {
    Coords { p: [f32; 3] },
}

/// The head style of a [`FeatureType::Hole`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum HoleKind {
    /// A plain cylindrical bore.
    #[default]
    Simple,
    /// A flat-bottomed enlargement at the surface (socket-head cap screws).
    Counterbore { diameter: f32, depth: f32 },
    /// A conical enlargement at the surface (flat-head screws). `angle_deg`
    /// is the full included angle (82° / 90° are the common standards).
    Countersink { diameter: f32, angle_deg: f32 },
}

/// How a [`FeatureType::Pattern`] replicates its source body.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PatternKind {
    /// `count` TOTAL instances (including the original) stepped `spacing`
    /// along `dir`.
    Linear {
        dir: AxisBase,
        spacing: f32,
        #[serde(default)]
        spacing_expr: Option<String>,
        count: u32,
    },
    /// `count` TOTAL instances equally spaced through `total_angle_deg` about
    /// `axis` (360 = a full ring; the step is angle/count so instance 0 and a
    /// would-be instance at 360° don't coincide).
    Circular {
        axis: AxisBase,
        count: u32,
        total_angle_deg: f32,
    },
    /// One mirrored copy across `plane`.
    Mirror { plane: PlaneBase },
}

/// A resolved datum: what a datum feature node evaluates to. Never serialized —
/// recomputed from the graph on every build.
#[derive(Debug, Clone, PartialEq)]
pub enum DatumValue {
    Plane(crate::geometry::CoordinateSystem),
    Axis {
        origin: crate::geometry::Vec3,
        dir: crate::geometry::Vec3,
    },
    Point(crate::geometry::Vec3),
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
    /// For a sketch placed on a body face: the durable [`FaceRef`] to that face,
    /// keyed by sketch node id. On rebuild the sketch's plane is re-derived from
    /// wherever the face now is (see `apply_extrude`), so a sketch-on-face follows
    /// the body when it changes instead of staying frozen. Persisted; empty for
    /// origin-plane sketches and legacy documents.
    #[serde(default)]
    pub sketch_face_refs: HashMap<String, FaceRef>,
    /// For a sketch placed on a datum plane: the datum node id, keyed by sketch
    /// node id. On rebuild the sketch's plane is re-derived from the datum's
    /// current resolution (so editing the datum moves everything sketched on
    /// it). Persisted; empty for origin-plane and face-attached sketches.
    #[serde(default)]
    pub sketch_datum_refs: HashMap<String, String>,
    /// For a sketch placed on a body face: that face's boundary loops (outer
    /// wire + holes), projected into the sketch's 2D plane at creation time and
    /// keyed by sketch node id. These are *reference* curves — never drawn by
    /// the user — that participate in region detection so drawn shapes
    /// intersect the face outline exactly like they intersect each other (and
    /// the bare face outline itself is an extrudable region). A snapshot: the
    /// sketch's *plane* re-derives when the body changes (`sketch_face_refs`),
    /// but the projected outline does not. Persisted; empty for origin-plane /
    /// datum sketches and legacy documents.
    #[serde(default)]
    pub sketch_face_boundaries: HashMap<String, crate::sketch::SketchCurves>,
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
    /// Face-reattachment updates queued by the last evaluation of THIS graph
    /// instance: when a face-attached sketch's parent body changed, the
    /// evaluator re-projected the face outline and remapped the affected
    /// extrudes' region indices onto the re-split regions — those refreshed
    /// values are parked here (evaluation is `&self`) for the owner to commit
    /// via [`ParametricGraph::apply_face_reattach`]. Skipped by serde; clone
    /// evals (previews, background refines) write to their own clone's queue,
    /// which dies with it.
    #[serde(skip)]
    pub(crate) pending_face_reattach: RefCell<FaceReattach>,
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

/// Refreshed sketch-on-face data queued during an evaluation — see
/// [`ParametricGraph::pending_face_reattach`].
#[derive(Debug, Clone, Default)]
pub struct FaceReattach {
    /// Sketch node id → the face outline re-projected from where the face is
    /// now (replaces the `sketch_face_boundaries` snapshot).
    pub boundaries: HashMap<String, crate::sketch::SketchCurves>,
    /// Sketch node id → the re-derived placement plane the refreshed outline
    /// was projected into. Written back to the sketch feature's saved `cs` so
    /// the GUI draws the sketch (and its outline) where the evaluator actually
    /// built from.
    pub planes: HashMap<String, crate::geometry::CoordinateSystem>,
    /// Extrude node id → its `region_indices` remapped onto the regions of the
    /// refreshed outline (an old index keeps its region by material-point
    /// containment).
    pub region_indices: HashMap<String, Vec<usize>>,
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
    /// Per-feature resolution status accumulated up to and including this node,
    /// in creation order. Cached alongside `warnings` so a reused prefix restores
    /// its statuses too — see [`FeatureStatus`].
    pub(crate) statuses: Vec<FeatureStatus>,
}

/// Whether a feature resolved its references and applied cleanly on the last
/// evaluation — or failed to, and why.
///
/// This is the structured, per-feature form of the flat warning list. It exists
/// so the GUI can flag *which* feature in the history tree is unresolved (a red
/// marker on that node) instead of only showing a global warning count, and so a
/// broken downstream reference is reported **loud and attributable** rather than
/// silently applied to the wrong entity — the core reliability contract of
/// history reattachment.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ResolutionState {
    /// The feature resolved its target(s) and applied without complaint.
    Resolved,
    /// The feature could not be applied as intended; the string is the reason
    /// (the same message surfaced in the warning list).
    Unresolved(String),
}

/// The resolution outcome of one feature during an evaluation.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FeatureStatus {
    /// The node id of the feature this status is for.
    pub feature_id: String,
    /// The feature's display name, for the tree/status UI.
    pub feature_name: String,
    /// Whether it resolved, and why not if it did not.
    pub state: ResolutionState,
}

impl FeatureStatus {
    /// Whether this feature failed to resolve/apply on the last evaluation.
    pub fn is_unresolved(&self) -> bool {
        matches!(self.state, ResolutionState::Unresolved(_))
    }

    /// The failure reason, if the feature is unresolved.
    pub fn reason(&self) -> Option<&str> {
        match &self.state {
            ResolutionState::Unresolved(r) => Some(r.as_str()),
            ResolutionState::Resolved => None,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct SketchEval {
    pub(crate) cs: CoordinateSystem,
    /// The DRAWN curves only (variable-resolved). The projected face boundary
    /// is deliberately NOT folded in here — the drawn-shape recognizers
    /// (rect-minus-circle, single-circle smooth cylinder) read this field and
    /// must see only what the user drew. `regions` below were detected on
    /// drawn ⊕ `face_boundary`.
    pub(crate) curves: SketchCurves,
    /// Sketch-on-face: the stored projected face outline that joined region
    /// detection (`graph.sketch_face_boundaries` snapshot). `None` for
    /// origin-plane / datum sketches. The extrude evaluator re-derives it from
    /// the live face and re-splits regions when the two differ.
    pub(crate) face_boundary: Option<SketchCurves>,
    pub(crate) regions: Vec<Region>,
    pub(crate) provenance: Vec<RegionProvenance>,
    /// Full closed outlines of the drawn shapes (before region-splitting), used
    /// to combine overlapping shapes as a boolean at extrude time. Empty for
    /// legacy sketches (no `shapes`) or sketches with sketch fillets/chamfers
    /// (`corner_mods`), which fall back to the per-region extrude path.
    pub(crate) shape_loops: Vec<ShapeLoop>,
    /// When the sketch's constraint model failed to re-solve against the
    /// current variables (over-constrained/conflicting), the reason. The
    /// geometry baked is the last-valid stored positions; the extrude that
    /// consumes this sketch surfaces the reason as a fail-loud warning.
    pub(crate) solve_failure: Option<String>,
}
