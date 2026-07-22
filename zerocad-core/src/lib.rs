pub mod boolean_case;
pub mod document;
pub mod dxf;
pub mod expr;
mod feature_dto;
pub mod geometry;
pub mod mock_kernel;
pub mod parametric;
pub mod release;
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
pub use boolean_case::{
    AnalyticPrimitiveV1, BooleanCaseError, BooleanCaseIdentity, BooleanCaseV1, BooleanContactClass,
    BooleanOperandV1, BooleanReplayOutcome, BooleanReplaySummary, BooleanVerificationOp,
    RigidTransformV1,
};
pub use document::{
    BodyId, BodyRecord, Document, DocumentRevision, DocumentSemantics, DocumentState,
    FeatureEditorGroup, FeatureEvaluatorKind, FeatureId, FeatureInput, FeatureInputTarget,
    FeatureKindId, FeatureRegistration, FeatureRegistry, FeatureState, GeometricIntent,
    SelectionProvenance, SelectionTopology, SelectorResolutionTier, SemanticEntityKind,
    SemanticSelector, SequenceKey,
};
pub use dxf::{
    read_dxf_file, read_dxf_str, DxfDiagnostic, DxfDiagnosticSeverity, DxfError, DxfImport, DxfUnit,
};
pub use expr::eval;
pub use geometry::{CoordinateSystem, SketchPlane, Vec3};
pub use mock_kernel::MockMesh;
pub use parametric::{
    body_output_id, boolean_region_plan, complete_selected_circles, drafted_region_solid,
    edge_wedge_is_concave_mesh, AllEdgeSelector, AllEdgeSelectorError, AxisBase, BodyInspection,
    BooleanRegionPlan, DatumAxisDef, DatumPlaneDef, DatumPointDef, DatumValue, DiagnosticCode,
    DiagnosticParameterValue, DiagnosticSeverity, EdgeInspection, EdgePairInspection, EdgeRef,
    EvaluationCacheSnapshot, EvaluationCancellation, EvaluationDiagnostic, EvaluationError,
    EvaluationOutput, EvaluationQuality, EvaluationTimings, EvaluationTrace, ExtrudeMode,
    FaceInspection, FeatureNode, FeatureTiming, FeatureType, HoleApplication, HoleFit, HoleKind,
    HoleManufacturingMetadata, HoleStandardPreset, InterferencePair, LegacyReferenceBackfills,
    LoftSurfaceMode, ParametricGraph, PatternKind, PlaneBase, PristineMeshReuse,
    ResolvedStandardGeometry, StandardReference, StandardsDataPack, StandardsFamily, SweepGuide,
    TopologyEdgeRef, TopologyVertexRef, Variable, VariableDiagnostic, VariableResolution,
    VertexRef, HOLE_STANDARD_PRESETS, STANDARDS_DATA_PACKS, STANDARDS_LIBRARY_ID,
    STANDARDS_LIBRARY_VERSION,
};
pub use release::{
    validate_compatibility_exception_ledger, validate_phase7_exception_ledger,
    validate_phase7_release_evidence, AlphaEvidence, ArtifactEvidence, CompatibilityException,
    DocumentPerformanceEvidence, InteractionEvidence, Phase7ReleaseEvidence, StartupEvidence,
    ValidationEvidence, PHASE7_COMPATIBILITY_EXCEPTIONS,
};
pub use sketch::{
    build_sketch_curves, detect_regions, detect_regions_with_provenance, effective_curves,
    effective_curves_solved, overlap_clusters, reflect_curves_across, shape_loops, shapes_cross,
    shapes_overlap, Circle, CornerKind, CornerMod, Dimension, ImportedSketchMetadata, LineSegment,
    Region, RegionProvenance, RegionProvenanceFragment, RegionWithProvenance, ShapeLoop,
    SketchCurves, SketchImportFormat, SketchMirror, SketchPatternError, SketchPatternEvaluation,
    SketchPatternKind, SketchPatternOperation, SketchPatternSpanId, SketchShape, Spline,
    SplineContinuity, SplineKind,
};
pub use stl::{
    meshes_to_3mf, meshes_to_binary_stl, read_stl_mesh, write_binary_stl, StlImportError,
    StlValidationReport, ValidatedMeshBody,
};
pub use units::{Parameter, Unit};
#[allow(deprecated)]
pub use zcad_format::{
    read_document, read_document_file, read_document_from_slice, read_zcad, read_zcad_file,
    write_document, write_document_file, write_document_to_vec, write_zcad, write_zcad_file,
    DocumentRecipeV3, HydrationBundle, LoadDiagnostic, LoadLimits, LoadOptions, LoadedDocument,
    LoadedZcad, RecipeDependency, RecipeFeature, SaveOptions, SaveProfile, ZcadDocument, ZcadError,
    ZcadMetadata,
};
