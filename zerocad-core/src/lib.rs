pub mod assembly;
pub mod assembly_mates;
pub mod assembly_ops;
pub mod assembly_release;
pub mod assembly_solver;
pub mod assembly_solver_release;
pub mod boolean_case;
pub mod document;
pub mod dxf;
pub mod evaluated_scene;
pub mod expr;
mod feature_dto;
pub mod geometry;
pub mod mock_kernel;
pub mod parametric;
pub mod release;
pub mod sketch;
pub mod stl;
pub mod text;
pub mod units;
pub mod zcad_format;

/// How many segments a full circle is discretized into. This is the single
/// source of truth shared by sketch arrangement, ellipse faceting, cylinder
/// solids and cylinder wireframes — they must agree so a sketched circle and
/// its extruded/booleaned solid line up. Changing it here changes all of them.
pub const CIRCLE_SEGS: usize = 48;

// Re-export common structures for easy access
pub use assembly::{
    AssemblyDefinition, AssemblyDocument, AssemblyError, AssemblyOccurrence,
    AssemblyPresentationV1, AssetHash, ModelHash, OccurrenceId, ProjectDocument, ProjectKind,
    RigidPlacement,
};
pub use assembly_mates::{
    AssemblyEntityRef, AssemblyLocalSelector, AssemblyMate, AssemblyMateError, AssemblyMateKind,
    AssemblyMateSet, MateId, MateSense,
};
pub use assembly_ops::{
    align_faces, assembly_bom, assembly_bom_csv, insert_part_snapshot, insert_prepared_occurrence,
    prepare_part_definition, replace_occurrences_with_prepared, AlignFaceFrame, AssemblyBomRow,
    AssemblyOperationError, InsertOccurrenceResult, PreparedPartDefinition,
    ReplaceOccurrencesResult, ALIGN_FACES_DEGENERATE_CROSS_THRESHOLD,
};
pub use assembly_release::{
    validate_assembly_v1_release_evidence, validate_assembly_v1_release_evidence_for_commit,
    AssemblyV1ReleaseEvidence, ASSEMBLY_V1_DRAG_MEDIAN_BUDGET_MS, ASSEMBLY_V1_EVIDENCE_SCHEMA,
    ASSEMBLY_V1_OPEN_MEDIAN_BUDGET_MS, ASSEMBLY_V1_REFERENCE_OCCURRENCES,
    ASSEMBLY_V1_REQUIRED_SAMPLES,
};
pub use assembly_solver::{
    add_mate_transactionally, apply_ephemeral_mate_target_transactionally, apply_mate_solution,
    apply_pose_target_transactionally, solve_assembly_mates, solve_assembly_mates_committed,
    solve_assembly_mates_interactive, MateComponentEvidence, MateFrame, MateSolveContext,
    MateSolvePath, MateSolveResult, MateSolveStatus, ResolvedMateFrames,
    ANGLE_TOLERANCE_CEILING_RAD, ANGLE_TOLERANCE_RAD, COMMITTED_NONLINEAR_ITERATION_LIMIT,
    DENSE_FREE_OCCURRENCE_LIMIT, DENSE_RESIDUAL_ROW_LIMIT, INITIAL_DAMPING,
    INTERACTIVE_NONLINEAR_ITERATION_LIMIT, TRANSLATION_TOLERANCE_ABS_MM,
    TRANSLATION_TOLERANCE_CEILING_ABS_MM, TRANSLATION_TOLERANCE_CEILING_RELATIVE,
    TRANSLATION_TOLERANCE_RELATIVE,
};
pub use assembly_solver_release::{
    calibration_decision, validate_assembly_v2_release_evidence,
    validate_assembly_v2_release_evidence_for_commit, AssemblyV2ReleaseEvidence,
    MateCalibrationCaseEvidence, MateCalibrationDecision, MateStressCorpusKind, MateStressEvidence,
    ASSEMBLY_V2_EVIDENCE_SCHEMA, ASSEMBLY_V2_REQUIRED_SAMPLES, ASSEMBLY_V2_STRESS_OCCURRENCES,
};
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
pub use evaluated_scene::{
    EvaluatedScene, SceneBuildError, SceneInstance, ScenePlacement, SceneStats,
    SharedEvaluatedScene, SharedSceneGeometries,
};
pub use expr::eval;
pub use geometry::{CoordinateSystem, SketchPlane, Vec3};
pub use mock_kernel::MockMesh;
pub use parametric::{
    body_output_id, boolean_region_plan, canonicalize_edge_refs, complete_selected_circles,
    drafted_region_solid, edge_wedge_is_concave_mesh, AllEdgeSelector, AllEdgeSelectorError,
    AxisBase, BodyInspection, BooleanRegionPlan, DatumAxisDef, DatumPlaneDef, DatumPointDef,
    DatumValue, DiagnosticCode, DiagnosticParameterValue, DiagnosticSeverity, EdgeCornerMode,
    EdgeInspection, EdgePairInspection, EdgeRef, EvaluationCacheSnapshot, EvaluationCancellation,
    EvaluationDiagnostic, EvaluationError, EvaluationOutput, EvaluationQuality, EvaluationTimings,
    EvaluationTrace, ExtrudeMode, FaceInspection, FeatureNode, FeatureTiming, FeatureType,
    HoleApplication, HoleFit, HoleKind, HoleManufacturingMetadata, HoleStandardPreset,
    InterferencePair, LegacyReferenceBackfills, LoftSurfaceMode, ParametricGraph, PatternKind,
    PlaneBase, PristineMeshReuse, ResolvedStandardGeometry, StandardReference, StandardsDataPack,
    StandardsFamily, SweepGuide, TopologyEdgeRef, TopologyVertexRef, Variable, VariableDiagnostic,
    VariableResolution, VertexRef, HOLE_STANDARD_PRESETS, STANDARDS_DATA_PACKS,
    STANDARDS_LIBRARY_ID, STANDARDS_LIBRARY_VERSION,
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
    Region, RegionDegeneracy, RegionProvenance, RegionProvenanceFragment, RegionWithProvenance,
    ShapeLoop, SketchCurves, SketchImportFormat, SketchMirror, SketchPatternError,
    SketchPatternEvaluation, SketchPatternKind, SketchPatternOperation, SketchPatternSpanId,
    SketchShape, Spline, SplineContinuity, SplineKind,
};
pub use stl::{
    assembly_to_3mf, assembly_to_binary_stl, meshes_to_3mf, meshes_to_binary_stl, read_stl_mesh,
    write_binary_stl, AssemblyMeshExportError, StlImportError, StlValidationReport,
    ValidatedMeshBody,
};
pub use units::{Parameter, Unit};
#[allow(deprecated)]
pub use zcad_format::{
    read_document, read_document_file, read_document_from_slice, read_project_document,
    read_project_document_file, read_project_document_from_slice, read_zcad, read_zcad_file,
    write_document, write_document_file, write_document_to_vec, write_project_document,
    write_project_document_file, write_project_document_to_vec, write_zcad, write_zcad_file,
    AssemblyRecipeDefinitionV1, AssemblyRecipeOccurrenceV1, AssemblyRecipeV1, AssemblyRecipeV2,
    DocumentRecipeV3, HydrationBundle, LoadDiagnostic, LoadLimits, LoadOptions, LoadedDocument,
    LoadedProjectDocument, LoadedZcad, RecipeDependency, RecipeFeature, SaveOptions, SaveProfile,
    ZcadDocument, ZcadError, ZcadMetadata,
};
