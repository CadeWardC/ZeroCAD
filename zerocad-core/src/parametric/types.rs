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

/// Separator used for additional solid-body outputs owned by one feature.
/// The first output keeps the feature id for backward compatibility; later
/// outputs use `feature_id::body:N` with a one-based display number.
const BODY_OUTPUT_SEPARATOR: &str = "::body:";

/// Stable runtime id for one solid-body output of a feature.
///
/// Output index zero deliberately returns `feature_id` unchanged so existing
/// documents and downstream references keep targeting the first body. Additional
/// disconnected outputs receive deterministic ids such as `extrude_5::body:2`.
pub fn body_output_id(feature_id: &str, output_index: usize) -> String {
    if output_index == 0 {
        feature_id.to_string()
    } else {
        format!("{feature_id}{BODY_OUTPUT_SEPARATOR}{}", output_index + 1)
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
    /// Optional expression evaluated in base units. `value` remains the
    /// last-valid/fallback display value in `unit`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expression: Option<String>,
}

impl Variable {
    /// A fresh variable with a placeholder name, zero value, and the given unit.
    pub fn new(name: impl Into<String>, unit: Unit) -> Self {
        Self {
            name: name.into(),
            value: 0.0,
            unit,
            expression: None,
        }
    }

    /// The variable's value converted to the base unit (millimeters).
    pub fn value_in_base(&self) -> f64 {
        self.unit.to_base(self.value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VariableDiagnostic {
    pub name: String,
    pub message: String,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct VariableResolution {
    pub values: std::collections::HashMap<String, f64>,
    pub dependencies: std::collections::BTreeMap<String, Vec<String>>,
    pub diagnostics: Vec<VariableDiagnostic>,
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
    /// Feature that produced this exact named topology entity.
    #[serde(default)]
    pub producer_feature_id: Option<String>,
    /// Previous durable entity replaced by the producing feature, when known.
    #[serde(default)]
    pub source_entity_id: Option<String>,
}

/// A solid edge captured geometrically for a 3D fillet/chamfer. The endpoints
/// and the two adjacent face normals are recorded in **world space** at
/// selection time (read straight from the body's wireframe; see
/// `MockMesh::edge_vertices` / `edge_face_normals`). The evaluator uses that
/// captured geometry when a persistent topology name cannot be resolved.
///
/// The topology field lets an `EdgeMod` follow equivalent upstream dimension
/// edits. Captured world-space geometry is a fallback only for genuinely
/// unnamed legacy references; a missing stable identity suspends the consumer.
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
    /// Stable identity of the connected solid component inside the body. This
    /// prevents a face name shared by two severed lumps from resolving against
    /// whichever lump happens to enumerate first.
    #[serde(default)]
    pub component_id: Option<String>,
    #[serde(default)]
    pub topology_version: Option<u64>,
    /// The face's durable name (its "owner"), e.g.
    /// `sketch:extrude_3:region:0:face:top`.
    #[serde(default)]
    pub face_id: Option<String>,
    #[serde(default)]
    pub surface_kind: Option<String>,
    /// Feature that produced this exact named topology entity.
    #[serde(default)]
    pub producer_feature_id: Option<String>,
    /// Previous durable entity replaced by the producing feature, when known.
    #[serde(default)]
    pub source_entity_id: Option<String>,
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
        /// Associative mirror operations, applied after the shapes and corner
        /// mods are built: each reflects the accumulated curves across its axis
        /// and appends the copy, so editing the source updates the mirror.
        /// `#[serde(default)]` so documents saved before mirrors existed load.
        #[serde(default)]
        mirrors: Vec<crate::sketch::SketchMirror>,
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
    /// every section sketch. Outer loops and deterministically-corresponded
    /// holes are skinned together; ambiguous hole matching stays unresolved.
    Loft {
        /// Ordered `(sketch_id, region_index)` sections.
        sections: Vec<(String, usize)>,
        #[serde(default)]
        mode: ExtrudeMode,
        #[serde(default)]
        target: Option<String>,
    },
    /// Sweep a profile region along a path sketch's open chain (rotation-
    /// minimizing frames — no twist). The profile, including holes, is placed
    /// perpendicular to the path start; the path must be one line/arc chain.
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
        /// Optional standards-library identity plus the resolved values used to
        /// create this hole. Evaluation always uses the numeric fields above.
        #[serde(default)]
        standard: Option<super::standards::StandardReference>,
        /// Persisted machining intent. A blind-hole drill point changes exact
        /// geometry; tap/thread fields describe cosmetic manufacturing intent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        manufacturing: Option<super::standards::HoleManufacturingMetadata>,
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
    /// Translate a finished body. `copy` leaves the source in place and emits a
    /// second body; otherwise the source is consumed and replaced by the moved
    /// result. Keeping this as a history feature makes later edits rebuild at
    /// the same placement.
    BodyTransform {
        source: String,
        translation: [f32; 3],
        #[serde(default)]
        copy: bool,
    },
    /// A modeled thread on a cylindrical face of a body: the wall is replaced by
    /// analytic helical thread bands (external = a rod OD, internal = a hole
    /// wall) — no boolean. Ends adjacent to a flat cap cut straight through the
    /// profile (Fusion-style); any other end (a chamfer cone, fillet torus, or a
    /// partial-length stop) gets a runout band fading the tooth to a plain
    /// circle plus an untouched cylinder collar, so blends on the rim survive.
    /// If the wall can't be replaced cleanly the body is left intact (cosmetic
    /// fallback) with a warning rather than hard-failing. The thread standard
    /// (metric / Unified / custom) only sets the numeric fields below.
    Thread {
        /// Node id of the body being threaded.
        target: String,
        /// The selected cylindrical face, resolved to axis + radius at build.
        face: FaceRef,
        /// Internal (tapped hole) vs external (rod). Sets the cut direction.
        internal: bool,
        /// Axial pitch — distance between crests along the axis (mm).
        pitch: f32,
        /// Radial thread depth, crest to root (mm).
        depth: f32,
        /// Included thread angle (degrees); 60 for metric and Unified.
        angle_deg: f32,
        /// Right-handed thread when true (the common case), else left-handed.
        right_handed: bool,
        /// Number of thread starts (lead = pitch × starts). 0/1 both mean single.
        #[serde(default)]
        starts: u32,
        /// Threaded length along the axis (mm); `None` = the full face length.
        #[serde(default)]
        length: Option<f32>,
        /// Anchor a partial thread at the axis-min end instead of the axis-max
        /// end. Ignored for full-length threads.
        #[serde(default)]
        flip: bool,
        /// Human designation for display (e.g. "M6×1", "¼-20 UNC", "Custom").
        #[serde(default)]
        designation: String,
        /// Optional standards-library identity plus the resolved values used to
        /// create this thread. Evaluation never re-queries the live table.
        #[serde(default)]
        standard: Option<super::standards::StandardReference>,
    },
    /// A reference plane (datum). Carries no geometry of its own — it resolves
    /// to a [`crate::geometry::CoordinateSystem`] during evaluation and exists
    /// so sketches (and later revolve axes, mirrors, …) can attach to a
    /// construction plane that isn't one of the three base planes.
    DatumPlane {
        def: DatumPlaneDef,
    },
    /// A reference axis (datum), resolving to an origin + direction.
    DatumAxis {
        def: DatumAxisDef,
    },
    /// A reference point (datum), resolving to a 3D position.
    DatumPoint {
        def: DatumPointDef,
    },
    /// Combine two or more finished bodies into one persistent body. Appended
    /// after all pre-existing variants to preserve binary `.zcad` enum tags.
    /// Connected solids are fused; disconnected inputs leave their original
    /// bodies unchanged and report the operation unresolved.
    BodyJoin {
        sources: Vec<String>,
    },
    /// Subtract one finished body (`tool`) from another (`target`). Appended
    /// after all pre-existing variants to preserve binary `.zcad` enum tags.
    /// The cutting body is consumed unless `keep_tool` is enabled.
    BodyCut {
        target: String,
        tool: String,
        #[serde(default)]
        keep_tool: bool,
    },
    /// Keep only the common positive volume of two finished bodies. The tool
    /// is consumed unless `keep_tool` is enabled.
    BodyIntersect {
        target: String,
        tool: String,
        #[serde(default)]
        keep_tool: bool,
    },
    /// Divide one body with an origin/datum plane or an associatively captured
    /// planar face. The evaluator emits two explicitly registered body outputs.
    BodySplit {
        target: String,
        plane: PlaneBase,
        #[serde(default)]
        face: Option<FaceRef>,
    },
    /// Positive uniform scale of a finished body about a persisted world-space
    /// pivot. `factor_expr` is authoritative when it resolves successfully;
    /// `factor` is the last known numeric fallback.
    BodyScale {
        source: String,
        factor: f32,
        #[serde(default)]
        factor_expr: Option<String>,
        center: [f32; 3],
    },
    /// Move a reattached planar face along its outward normal. Positive values
    /// add material (Press/Pull outward); negative values remove material.
    /// The source body is consumed only after the rebuilt solid passes strict
    /// validation.
    FaceOffset {
        target: String,
        face: FaceRef,
        distance: f32,
        #[serde(default)]
        distance_expr: Option<String>,
    },
    /// Translate a reattached planar face. Phase 5 accepts the exact direct-
    /// modeling case where the vector is parallel to the face normal; a
    /// tangential component is rejected instead of approximated.
    FaceMove {
        target: String,
        face: FaceRef,
        translation: [f32; 3],
    },
    /// Remove one face and heal the surrounding solid. Phase 5 supports
    /// internal cylindrical faces (hole deletion); other topology changes fail
    /// explicitly and leave the input untouched.
    FaceDelete {
        target: String,
        face: FaceRef,
    },
    /// Create a new solid by thickening one planar face. The source remains in
    /// the document and the new feature owns the thickened result.
    FaceThicken {
        target: String,
        face: FaceRef,
        thickness: f32,
        #[serde(default)]
        thickness_expr: Option<String>,
        #[serde(default)]
        reverse: bool,
    },
    /// A validated triangle-mesh body imported from ASCII or binary STL. The
    /// bytes are stored as a content-addressed required asset in `.zcad` files.
    /// Mesh bodies remain distinct from B-Reps and cannot enter solid booleans.
    ImportStl {
        stl_data: Vec<u8>,
        label: String,
    },
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
    TwoPoints {
        a: [f32; 3],
        b: [f32; 3],
    },
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
    TwoPoints {
        a: [f32; 3],
        b: [f32; 3],
    },
    /// The intersection line of two non-parallel planes.
    PlaneIntersection {
        a: PlaneBase,
        b: PlaneBase,
    },
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
    /// One mirrored copy across an origin/datum plane, or an associatively
    /// captured planar body face. `face` defaults to `None` for older files.
    Mirror {
        plane: PlaneBase,
        #[serde(default)]
        face: Option<FaceRef>,
        /// Translation of the reflected copy along the mirror plane normal.
        #[serde(default)]
        offset: f32,
        #[serde(default)]
        offset_expr: Option<String>,
        /// Union the source and mirrored copy when every mirrored part connects.
        #[serde(default)]
        join: bool,
    },
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

/// Authoritative feature record stored by a document.
///
/// `FeatureNode` remains the lightweight creation command used by callers, but
/// once inserted the payload and all stable semantic fields live together in
/// this one record.  The evaluator graph is derived scheduling structure; it no
/// longer needs a parallel feature sidecar that can drift from the payload.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeatureRecord {
    pub id: String,
    pub name: String,
    pub feature: FeatureType,
    pub kind_id: crate::document::FeatureKindId,
    pub payload_version: u16,
    pub sequence: crate::document::SequenceKey,
    pub inputs: Vec<crate::document::FeatureInput>,
    pub state: crate::document::FeatureState,
    pub body: Option<crate::document::BodyId>,
}

impl FeatureRecord {
    pub(crate) fn from_node(node: FeatureNode, sequence: crate::document::SequenceKey) -> Self {
        let kind_id = crate::document::FeatureKindId::from(node.feature.kind_id());
        let payload_version = node.feature.payload_version();
        let inputs = crate::document::FeatureRegistry::dependencies(&node.feature);
        let body = crate::document::body_for_feature(&node.id, &node.feature);
        Self {
            id: node.id,
            name: node.name,
            feature: node.feature,
            kind_id,
            payload_version,
            sequence,
            inputs,
            state: crate::document::FeatureState::Active,
            body,
        }
    }
}

#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct ParametricGraph {
    pub graph: DiGraph<FeatureRecord, ()>,
    /// Stable body identities and ordered timelines. Feature semantics live on
    /// each [`FeatureRecord`]; this collection owns only cross-feature bodies.
    #[serde(default)]
    pub semantics: crate::document::DocumentSemantics,
    /// For a sketch placed on a body face: the durable [`FaceRef`] to that face,
    /// keyed by sketch node id. On rebuild the sketch's plane is re-derived from
    /// wherever the face now is (see `apply_extrude`), so a sketch-on-face follows
    /// the body when it changes instead of staying frozen. Persisted; empty for
    /// origin-plane sketches and legacy documents.
    #[serde(default)]
    pub sketch_face_refs: HashMap<crate::document::FeatureId, FaceRef>,
    /// For a sketch placed on a datum plane: the datum node id, keyed by sketch
    /// node id. On rebuild the sketch's plane is re-derived from the datum's
    /// current resolution (so editing the datum moves everything sketched on
    /// it). Persisted; empty for origin-plane and face-attached sketches.
    #[serde(default)]
    pub sketch_datum_refs: HashMap<crate::document::FeatureId, crate::document::FeatureId>,
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
    pub sketch_face_boundaries: HashMap<crate::document::FeatureId, crate::sketch::SketchCurves>,
    #[serde(skip)]
    pub(crate) node_map: HashMap<crate::document::FeatureId, NodeIndex>,
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
    /// Unique legacy geometric matches discovered at their historical body
    /// state. Evaluation only queues these; explicit migration/save commits.
    #[serde(skip)]
    pub(crate) pending_legacy_backfills: RefCell<LegacyReferenceBackfills>,
    /// Per-node geometry checkpoints from the previous evaluation, used to skip
    /// re-solving the unchanged prefix of the feature tree. Each entry holds the
    /// assembled bodies *after* one node, keyed by a cumulative content hash of
    /// every input that node's geometry depends on (see [`evaluate_bodies_inner`]).
    /// When an edit changes only a trailing node — e.g. dragging a fillet/chamfer
    /// radius — the prefix hashes still match, so the expensive upstream booleans
    /// are restored from here instead of recomputed every frame. Skipped by serde
    /// and a transparent accelerator (dropping it only costs a one-time rebuild).
    #[serde(skip)]
    pub(crate) eval_cache: RefCell<std::sync::Arc<EvalCache>>,
}

#[derive(serde::Deserialize)]
struct PersistedParametricGraph {
    graph: DiGraph<FeatureRecord, ()>,
    #[serde(default)]
    semantics: crate::document::DocumentSemantics,
    #[serde(default)]
    sketch_face_refs: HashMap<crate::document::FeatureId, FaceRef>,
    #[serde(default)]
    sketch_datum_refs: HashMap<crate::document::FeatureId, crate::document::FeatureId>,
    #[serde(default)]
    sketch_face_boundaries: HashMap<crate::document::FeatureId, crate::sketch::SketchCurves>,
}

impl<'de> serde::Deserialize<'de> for ParametricGraph {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let persisted = PersistedParametricGraph::deserialize(deserializer)?;
        let mut graph = Self {
            graph: persisted.graph,
            semantics: persisted.semantics,
            sketch_face_refs: persisted.sketch_face_refs,
            sketch_datum_refs: persisted.sketch_datum_refs,
            sketch_face_boundaries: persisted.sketch_face_boundaries,
            node_map: HashMap::new(),
            region_cache: RefCell::new(HashMap::new()),
            pending_face_reattach: RefCell::new(FaceReattach::default()),
            pending_legacy_backfills: RefCell::new(LegacyReferenceBackfills::default()),
            eval_cache: RefCell::new(std::sync::Arc::new(EvalCache::default())),
        };
        graph.rebuild_node_map();
        Ok(graph)
    }
}

/// Refreshed sketch-on-face data queued during an evaluation — see
/// [`ParametricGraph::pending_face_reattach`].
#[derive(Debug, Clone)]
pub struct FaceReattach {
    producing_revision: crate::document::DocumentRevision,
    /// Sketch node id → the face outline re-projected from where the face is
    /// now (replaces the `sketch_face_boundaries` snapshot).
    pub boundaries: HashMap<crate::document::FeatureId, crate::sketch::SketchCurves>,
    /// Sketch node id → the re-derived placement plane the refreshed outline
    /// was projected into. Written back to the sketch feature's saved `cs` so
    /// the GUI draws the sketch (and its outline) where the evaluator actually
    /// built from.
    pub planes: HashMap<crate::document::FeatureId, crate::geometry::CoordinateSystem>,
    /// Extrude node id → its `region_indices` remapped onto the regions of the
    /// refreshed outline (an old index keeps its region by material-point
    /// containment).
    pub region_indices: HashMap<crate::document::FeatureId, Vec<usize>>,
}

impl Default for FaceReattach {
    fn default() -> Self {
        Self::for_revision(crate::document::DocumentRevision::INITIAL)
    }
}

impl FaceReattach {
    pub(crate) fn for_revision(revision: crate::document::DocumentRevision) -> Self {
        Self {
            producing_revision: revision,
            boundaries: HashMap::new(),
            planes: HashMap::new(),
            region_indices: HashMap::new(),
        }
    }

    pub fn producing_revision(&self) -> crate::document::DocumentRevision {
        self.producing_revision
    }
}

#[derive(Debug, Clone)]
pub struct LegacyReferenceBackfills {
    pub(crate) producing_revision: crate::document::DocumentRevision,
    pub(crate) edge_mods: HashMap<crate::document::FeatureId, EdgeRef>,
    pub(crate) sketch_faces: HashMap<crate::document::FeatureId, FaceRef>,
}

impl Default for LegacyReferenceBackfills {
    fn default() -> Self {
        Self::for_revision(crate::document::DocumentRevision::INITIAL)
    }
}

impl LegacyReferenceBackfills {
    pub(crate) fn for_revision(revision: crate::document::DocumentRevision) -> Self {
        Self {
            producing_revision: revision,
            edge_mods: HashMap::new(),
            sketch_faces: HashMap::new(),
        }
    }

    pub fn producing_revision(&self) -> crate::document::DocumentRevision {
        self.producing_revision
    }

    pub fn is_empty(&self) -> bool {
        self.edge_mods.is_empty() && self.sketch_faces.is_empty()
    }

    pub(crate) fn rebind_revision(&mut self, revision: crate::document::DocumentRevision) {
        self.producing_revision = revision;
    }
}

/// Unpersisted dependency split for one evaluator checkpoint.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct DependencyManifest {
    pub(crate) geometry_hash: u64,
    pub(crate) diagnostic_hash: u64,
}

/// Checkpoints of [`evaluate_bodies_inner`], one per processed body node, in
/// creation order. A pure accelerator — see [`ParametricGraph::eval_cache`].
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub(crate) struct EvalCache {
    pub(crate) checkpoints: Vec<Option<EvalCheckpoint>>,
}

/// The assembled bodies and accumulated warnings immediately after one node was
/// applied, tagged with the cumulative hash of all geometry inputs up to it.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct EvalCheckpoint {
    pub(crate) key: u64,
    #[serde(skip)]
    pub(crate) manifest: DependencyManifest,
    pub(crate) live: Vec<LiveBody>,
    pub(crate) warnings: Vec<String>,
    /// Per-feature resolution status accumulated up to and including this node,
    /// in creation order. Cached alongside `warnings` so a reused prefix restores
    /// its statuses too — see [`FeatureStatus`].
    pub(crate) statuses: Vec<FeatureStatus>,
    /// Time spent applying this feature when the checkpoint was built. Reused
    /// checkpoints preserve the original measurement.
    pub(crate) feature_duration: std::time::Duration,
}

/// Opaque reusable evaluator state produced by a background graph clone.
/// Installing it is always safe: every checkpoint is content-keyed and a
/// mismatch merely causes the evaluator to rebuild from an earlier prefix.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct EvaluationCacheSnapshot {
    pub(crate) cache: std::sync::Arc<EvalCache>,
}

#[derive(Debug, Clone)]
pub struct FeatureTiming {
    pub feature_id: String,
    pub duration: std::time::Duration,
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
    /// The user intentionally excluded the feature from evaluation. Unlike
    /// visibility, suppression changes geometry and participates in cache keys.
    Suppressed,
    /// The feature could not be applied as intended; the string is the reason
    /// (the same message surfaced in the warning list).
    Unresolved(String),
}

/// The resolution outcome of one feature during an evaluation.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FeatureStatus {
    /// The node id of the feature this status is for.
    pub feature_id: crate::document::FeatureId,
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
            ResolutionState::Resolved | ResolutionState::Suppressed => None,
        }
    }
}

/// Geometry/detail budget requested from the parametric evaluator.
/// Interactive evaluation keeps the same feature semantics but uses the coarse
/// tessellation budget; final evaluation is the authoritative save/export path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvaluationQuality {
    Interactive,
    Final,
}

/// Cooperative cancellation shared by the GUI scheduler and core evaluator.
/// A request is current only while `latest_generation == generation`.
#[derive(Debug, Clone)]
pub struct EvaluationCancellation {
    generation: u64,
    latest_generation: std::sync::Arc<std::sync::atomic::AtomicU64>,
}

impl EvaluationCancellation {
    pub fn new(
        generation: u64,
        latest_generation: std::sync::Arc<std::sync::atomic::AtomicU64>,
    ) -> Self {
        Self {
            generation,
            latest_generation,
        }
    }

    pub fn is_cancelled(&self) -> bool {
        self.latest_generation
            .load(std::sync::atomic::Ordering::Acquire)
            != self.generation
    }
}

impl openrcad::foundation::CancellationProbe for EvaluationCancellation {
    fn is_cancelled(&self) -> bool {
        EvaluationCancellation::is_cancelled(self)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EvaluationTimings {
    pub total: std::time::Duration,
    pub build: std::time::Duration,
    pub tessellation: std::time::Duration,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PristineMeshReuse {
    pub body_id: crate::document::BodyId,
    /// Stable only for the lifetime of the evaluation process. Tests compare
    /// identity across warm evaluations; it is never persisted or displayed.
    pub allocation_identity: usize,
}

/// Deterministic evaluator work classification. Normal tests assert this trace
/// instead of machine-dependent wall-clock durations.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EvaluationTrace {
    pub reused_checkpoints: Vec<crate::document::FeatureId>,
    pub evaluated_features: Vec<crate::document::FeatureId>,
    pub tessellated_bodies: Vec<crate::document::BodyId>,
    pub pristine_mesh_reuse: Vec<PristineMeshReuse>,
}

#[derive(Debug)]
pub struct EvaluationOutput {
    pub bodies: Vec<(String, MockMesh)>,
    pub warnings: Vec<String>,
    pub statuses: Vec<FeatureStatus>,
    pub diagnostics: Vec<EvaluationDiagnostic>,
    pub face_reattach: FaceReattach,
    pub legacy_reference_backfills: LegacyReferenceBackfills,
    pub timings: EvaluationTimings,
    pub feature_timings: Vec<FeatureTiming>,
    pub trace: EvaluationTrace,
    pub revision: crate::document::DocumentRevision,
    pub cache_snapshot: EvaluationCacheSnapshot,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticSeverity {
    Info,
    Warning,
    Error,
}

/// Stable machine-readable diagnostic identity.
///
/// Codes are deliberately separate from rendered English copy. New codes are
/// append-only; callers compare this type (and structured parameters) rather
/// than matching substrings in [`EvaluationDiagnostic::message`]. Kernel-owned
/// codes retain their names under the `kernel.` namespace.
#[derive(
    Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct DiagnosticCode(String);

impl DiagnosticCode {
    pub const REFERENCE_MISSING: &'static str = "reference.missing";
    pub const REFERENCE_AMBIGUOUS: &'static str = "reference.ambiguous";
    pub const PARAMETER_INVALID: &'static str = "parameter.invalid";
    pub const OPERATION_FAILED: &'static str = "operation.failed";
    pub const RESULT_INVALID_TOPOLOGY: &'static str = "result.invalid_topology";
    pub const FEATURE_UNRESOLVED: &'static str = "feature.unresolved";
    pub const KERNEL_RECOVERY: &'static str = "kernel.recovery";

    pub fn new(code: impl Into<String>) -> Result<Self, String> {
        let code = code.into();
        if code.is_empty()
            || !code.bytes().all(|byte| {
                byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"._-".contains(&byte)
            })
        {
            return Err(format!("invalid diagnostic code '{code}'"));
        }
        Ok(Self(code))
    }

    pub fn feature_unresolved() -> Self {
        Self(Self::FEATURE_UNRESOLVED.to_string())
    }

    pub fn reference_missing() -> Self {
        Self(Self::REFERENCE_MISSING.to_string())
    }

    pub fn reference_ambiguous() -> Self {
        Self(Self::REFERENCE_AMBIGUOUS.to_string())
    }

    pub fn parameter_invalid() -> Self {
        Self(Self::PARAMETER_INVALID.to_string())
    }

    pub fn operation_failed() -> Self {
        Self(Self::OPERATION_FAILED.to_string())
    }

    pub fn result_invalid_topology() -> Self {
        Self(Self::RESULT_INVALID_TOPOLOGY.to_string())
    }

    pub fn kernel(code: &str) -> Self {
        let normalized = code
            .chars()
            .map(|character| {
                if character.is_ascii_alphanumeric() {
                    character.to_ascii_lowercase()
                } else {
                    '_'
                }
            })
            .collect::<String>();
        Self(format!("kernel.{normalized}"))
    }

    pub fn kernel_recovery() -> Self {
        Self(Self::KERNEL_RECOVERY.to_string())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for DiagnosticCode {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

/// Canonical values accepted by diagnostic parameters. Decimal values are
/// stored as normalized strings so the diagnostic contract remains `Eq` and
/// does not inherit NaN or platform-formatting behavior.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", content = "value", rename_all = "snake_case")]
pub enum DiagnosticParameterValue {
    Text(String),
    Integer(i64),
    Unsigned(u64),
    Boolean(bool),
    Decimal(String),
}

impl From<&str> for DiagnosticParameterValue {
    fn from(value: &str) -> Self {
        Self::Text(value.to_string())
    }
}

impl From<String> for DiagnosticParameterValue {
    fn from(value: String) -> Self {
        Self::Text(value)
    }
}

impl From<bool> for DiagnosticParameterValue {
    fn from(value: bool) -> Self {
        Self::Boolean(value)
    }
}

impl From<i64> for DiagnosticParameterValue {
    fn from(value: i64) -> Self {
        Self::Integer(value)
    }
}

impl From<u64> for DiagnosticParameterValue {
    fn from(value: u64) -> Self {
        Self::Unsigned(value)
    }
}

impl From<usize> for DiagnosticParameterValue {
    fn from(value: usize) -> Self {
        Self::Unsigned(value as u64)
    }
}

/// Machine-readable counterpart to the status-bar warning text. `code` and
/// `parameters` are the behavioral contract; `message`, `operation`, and
/// `fallback` are rendered/debugging copy and may improve without changing the
/// code contract.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct EvaluationDiagnostic {
    pub feature_id: String,
    pub operation: String,
    pub code: DiagnosticCode,
    pub parameters: std::collections::BTreeMap<String, DiagnosticParameterValue>,
    /// Compatibility mirror of [`Self::code`] for older diagnostic consumers.
    /// New code must use `code`.
    pub failure_class: String,
    pub fallback: Option<String>,
    pub severity: DiagnosticSeverity,
    pub message: String,
}

impl EvaluationDiagnostic {
    pub fn new(
        feature_id: impl Into<String>,
        operation: impl Into<String>,
        code: DiagnosticCode,
        severity: DiagnosticSeverity,
        message: impl Into<String>,
    ) -> Self {
        let failure_class = code.to_string();
        Self {
            feature_id: feature_id.into(),
            operation: operation.into(),
            code,
            parameters: std::collections::BTreeMap::new(),
            failure_class,
            fallback: None,
            severity,
            message: message.into(),
        }
    }

    pub fn with_parameter(
        mut self,
        name: impl Into<String>,
        value: impl Into<DiagnosticParameterValue>,
    ) -> Self {
        self.parameters.insert(name.into(), value.into());
        self
    }

    pub fn with_fallback(mut self, fallback: impl Into<String>) -> Self {
        self.fallback = Some(fallback.into());
        self
    }

    /// Override the one-cycle compatibility classification without weakening
    /// the stable `code` contract. This is used only where the pre-typed API
    /// exposed a different spelling.
    pub fn with_failure_class(mut self, failure_class: impl Into<String>) -> Self {
        self.failure_class = failure_class.into();
        self
    }

    /// UI copy is intentionally accessed separately from the stable diagnostic
    /// identity so behavioral tests never need to parse or pin it.
    pub fn rendered_message(&self) -> &str {
        &self.message
    }
}

impl EvaluationOutput {
    /// Typed replacement for checking the compatibility `warnings` vector.
    pub fn has_diagnostic_warnings(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|diagnostic| diagnostic.severity != DiagnosticSeverity::Info)
    }

    /// Render warning/error diagnostics for compatibility UI surfaces. The
    /// diagnostic code and parameters remain authoritative; this copy is not a
    /// behavioral contract.
    pub fn rendered_warnings(&self) -> Vec<String> {
        self.diagnostics
            .iter()
            .filter(|diagnostic| diagnostic.severity != DiagnosticSeverity::Info)
            .map(|diagnostic| diagnostic.rendered_message().to_string())
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvaluationError {
    Cancelled,
    Failed(String),
}

impl std::fmt::Display for EvaluationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Cancelled => f.write_str("model evaluation was superseded"),
            Self::Failed(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for EvaluationError {}

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
