pub mod document;
pub mod expr;
mod feature_dto;
pub mod geometry;
pub mod mock_kernel;
pub mod parametric;
pub mod sketch;
pub mod stl;
pub mod units;
pub mod zcad_format;

/// How many segments a full circle is discretized into. This is the single
/// source of truth shared by sketch arrangement, ellipse faceting, cylinder
/// solids and cylinder wireframes — they must agree so a sketched circle and
/// its extruded/booleaned solid line up. Changing it here changes all of them.
pub const CIRCLE_SEGS: usize = 48;

// Re-export common structures for easy access
pub use document::{
    BodyId, BodyRecord, Document, DocumentSemantics, DocumentState, FeatureEditorGroup,
    FeatureEvaluatorKind, FeatureId, FeatureInput, FeatureInputTarget, FeatureKindId,
    FeatureRegistration, FeatureRegistry, FeatureState, GeometricIntent, SelectionProvenance,
    SelectionTopology, SelectorResolutionTier, SemanticEntityKind, SemanticSelector, SequenceKey,
};
pub use expr::eval;
pub use geometry::{CoordinateSystem, SketchPlane, Vec3};
pub use mock_kernel::MockMesh;
pub use parametric::{
    body_output_id, body_output_index, body_output_owner_id, boolean_region_plan,
    complete_selected_circles, edge_wedge_is_concave_mesh, AxisBase, BooleanRegionPlan,
    DatumAxisDef, DatumPlaneDef, DatumPointDef, DatumValue, DiagnosticSeverity, EdgeRef,
    EvaluationCacheSnapshot, EvaluationCancellation, EvaluationDiagnostic, EvaluationError,
    EvaluationOutput, EvaluationQuality, EvaluationTimings, ExtrudeMode, FeatureNode,
    FeatureTiming, FeatureType, HoleKind, ParametricGraph, PatternKind, PlaneBase, TopologyEdgeRef,
    Variable,
};
pub use sketch::{
    build_sketch_curves, detect_regions, detect_regions_with_provenance, effective_curves,
    effective_curves_solved, overlap_clusters, reflect_curves_across, shape_loops, shapes_cross,
    shapes_overlap, Circle, CornerKind, CornerMod, Dimension, LineSegment, Region,
    RegionProvenance, RegionProvenanceFragment, RegionWithProvenance, ShapeLoop, SketchCurves,
    SketchMirror, SketchShape,
};
pub use stl::{meshes_to_3mf, meshes_to_binary_stl, write_binary_stl};
pub use units::{Parameter, Unit};
#[allow(deprecated)]
pub use zcad_format::{
    read_document, read_document_file, read_document_from_slice, read_zcad, read_zcad_file,
    write_document, write_document_file, write_document_to_vec, write_zcad, write_zcad_file,
    DocumentRecipeV3, HydrationBundle, LoadDiagnostic, LoadLimits, LoadOptions, LoadedDocument,
    LoadedZcad, RecipeDependency, RecipeFeature, SaveOptions, SaveProfile, ZcadDocument, ZcadError,
    ZcadMetadata,
};
