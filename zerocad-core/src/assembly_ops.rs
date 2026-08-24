//! Transactional assembly definition and occurrence operations.

use crate::{
    read_project_document_from_slice, write_project_document_to_vec, AssemblyDefinition,
    AssemblyDocument, AssemblyOccurrence, HydrationBundle, LoadDiagnostic, LoadOptions, MockMesh,
    ModelHash, ProjectDocument, RigidPlacement, SaveOptions,
};
use std::collections::HashSet;
use std::fmt;
use std::sync::Arc;

pub const ALIGN_FACES_DEGENERATE_CROSS_THRESHOLD: f64 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignFaceFrame {
    pub centroid: [f64; 3],
    pub normal: [f64; 3],
    /// Moving-face parameterization direction in the face plane.
    pub parametric_u: Option<[f64; 3]>,
}

pub fn align_faces(
    moving_placement: RigidPlacement,
    moving_face: AlignFaceFrame,
    target_placement: RigidPlacement,
    target_face: AlignFaceFrame,
) -> Result<RigidPlacement, AssemblyOperationError> {
    let moving_normal = normalize(rotate(moving_placement.rotation(), moving_face.normal))
        .ok_or(AssemblyOperationError::DegenerateFaceNormal)?;
    let target_normal = normalize(rotate(target_placement.rotation(), target_face.normal))
        .ok_or(AssemblyOperationError::DegenerateFaceNormal)?;
    let desired_normal = scale(target_normal, -1.0);
    let cross_product = cross(moving_normal, desired_normal);
    let cross_magnitude = norm(cross_product);
    let dot_product = dot(moving_normal, desired_normal).clamp(-1.0, 1.0);
    let delta = if cross_magnitude >= ALIGN_FACES_DEGENERATE_CROSS_THRESHOLD {
        quaternion_from_axis_angle(
            scale(cross_product, cross_magnitude.recip()),
            cross_magnitude.atan2(dot_product),
        )
    } else if dot_product >= 0.0 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        let parameter_axis = moving_face.parametric_u.and_then(|u| {
            let world_u = rotate(moving_placement.rotation(), u);
            normalize(subtract(
                world_u,
                scale(moving_normal, dot(world_u, moving_normal)),
            ))
        });
        let axis = parameter_axis.unwrap_or_else(|| {
            let globals = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            let chosen = globals
                .into_iter()
                .min_by(|a, b| {
                    dot(*a, moving_normal)
                        .abs()
                        .total_cmp(&dot(*b, moving_normal).abs())
                })
                .unwrap();
            normalize(cross(chosen, moving_normal))
                .expect("least-aligned canonical axis must cross the normal")
        });
        // Do not obtain this from cos(PI / 2): libm returns a tiny
        // non-zero value, which would make byte-level persistence of the
        // canonical 180-degree result depend on floating-point details.
        [0.0, axis[0], axis[1], axis[2]]
    };
    let rotation = quaternion_multiply(delta, moving_placement.rotation());
    let target_centroid_world = add(
        rotate(target_placement.rotation(), target_face.centroid),
        target_placement.translation(),
    );
    let rotated_moving_centroid = rotate(rotation, moving_face.centroid);
    let translation = subtract(target_centroid_world, rotated_moving_centroid);
    RigidPlacement::new(translation, rotation)
        .map_err(|error| AssemblyOperationError::InvalidPlacement(error.to_string()))
}

fn quaternion_from_axis_angle(axis: [f64; 3], angle: f64) -> [f64; 4] {
    let (sin, cos) = (angle * 0.5).sin_cos();
    [cos, axis[0] * sin, axis[1] * sin, axis[2] * sin]
}

fn quaternion_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn rotate(quaternion: [f64; 4], vector: [f64; 3]) -> [f64; 3] {
    let q_vector = [0.0, vector[0], vector[1], vector[2]];
    let conjugate = [
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3],
    ];
    let rotated = quaternion_multiply(quaternion_multiply(quaternion, q_vector), conjugate);
    [rotated[1], rotated[2], rotated[3]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn subtract(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(vector: [f64; 3], factor: f64) -> [f64; 3] {
    [vector[0] * factor, vector[1] * factor, vector[2] * factor]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(vector: [f64; 3]) -> f64 {
    dot(vector, vector).sqrt()
}

fn normalize(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = norm(vector);
    (length.is_finite() && length > 1.0e-15).then(|| scale(vector, length.recip()))
}

#[derive(Debug, Clone)]
pub struct PreparedPartDefinition {
    pub model_hash: ModelHash,
    pub compact_snapshot: Arc<Vec<u8>>,
    pub suggested_name: String,
    pub source_basename: Option<String>,
    pub display_bodies: Arc<Vec<(String, MockMesh)>>,
    pub diagnostics: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct InsertOccurrenceResult {
    pub occurrence_id: u64,
    pub definition_model_hash: ModelHash,
    pub definition_created: bool,
    pub display_bodies: Arc<Vec<(String, MockMesh)>>,
    pub diagnostics: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct ReplaceOccurrencesResult {
    pub replaced_occurrence_ids: Vec<u64>,
    pub definition_model_hash: ModelHash,
    pub definition_created: bool,
    pub pruned_definition_hashes: Vec<ModelHash>,
    pub display_bodies: Arc<Vec<(String, MockMesh)>>,
    pub diagnostics: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssemblyBomRow {
    pub model_hash: ModelHash,
    pub name: String,
    pub source_basename: Option<String>,
    pub quantity: u64,
}

pub fn assembly_bom(document: &AssemblyDocument) -> Vec<AssemblyBomRow> {
    document
        .definitions
        .iter()
        .filter_map(|(model_hash, definition)| {
            let quantity = document
                .occurrences
                .values()
                .filter(|occurrence| occurrence.definition_model_hash == *model_hash)
                .count() as u64;
            (quantity > 0).then(|| AssemblyBomRow {
                model_hash: *model_hash,
                name: definition.name.clone(),
                source_basename: definition.source_basename.clone(),
                quantity,
            })
        })
        .collect()
}

pub fn assembly_bom_csv(document: &AssemblyDocument) -> String {
    let mut csv = "name,quantity,source,model_hash\n".to_owned();
    for row in assembly_bom(document) {
        csv.push_str(&csv_field(&row.name));
        csv.push(',');
        csv.push_str(&row.quantity.to_string());
        csv.push(',');
        csv.push_str(&csv_field(row.source_basename.as_deref().unwrap_or("")));
        csv.push(',');
        for byte in row.model_hash {
            use std::fmt::Write as _;
            let _ = write!(csv, "{byte:02x}");
        }
        csv.push('\n');
    }
    csv
}

fn csv_field(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

pub fn prepare_part_definition(
    source_bytes: &[u8],
    source_basename: Option<&str>,
    options: &LoadOptions,
) -> Result<PreparedPartDefinition, AssemblyOperationError> {
    let loaded = read_project_document_from_slice(source_bytes, options)
        .map_err(|error| AssemblyOperationError::InvalidPart(error.to_string()))?;
    let input_model_hash = loaded.metadata.model_hash;
    let mut diagnostics: Vec<String> = loaded
        .diagnostics
        .iter()
        .map(format_load_diagnostic)
        .collect();
    let ProjectDocument::Part(document) = loaded.document else {
        return Err(AssemblyOperationError::ExpectedPart);
    };

    // Definition evaluation deliberately ignores source presentation visibility.
    let (display_bodies, warnings) = document
        .evaluator_graph()
        .evaluate_bodies_with_warnings(&HashSet::new())
        .map_err(AssemblyOperationError::EvaluationFailed)?;
    diagnostics.extend(warnings);
    if !display_bodies
        .iter()
        .any(|(_, mesh)| body_is_displayable(mesh))
    {
        return Err(AssemblyOperationError::NoDisplayableFinalBody);
    }

    let compact_snapshot = write_project_document_to_vec(
        &ProjectDocument::Part(document.clone_authoritative()),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .map_err(|error| AssemblyOperationError::InvalidPart(error.to_string()))?;
    let normalized =
        read_project_document_from_slice(&compact_snapshot, options).map_err(|error| {
            AssemblyOperationError::InvalidPart(format!(
                "normalized compact snapshot failed self-check: {error}"
            ))
        })?;
    if normalized.metadata.model_hash != input_model_hash {
        return Err(AssemblyOperationError::ModelHashChangedDuringNormalization);
    }

    let source_basename = source_basename
        .and_then(|name| {
            std::path::Path::new(name)
                .file_name()
                .map(|value| value.to_string_lossy().into_owned())
        })
        .filter(|name| !name.trim().is_empty());
    let suggested_name = source_basename
        .as_deref()
        .and_then(|name| std::path::Path::new(name).file_stem())
        .map(|stem| stem.to_string_lossy().trim().to_owned())
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| "Definition".to_owned());

    Ok(PreparedPartDefinition {
        model_hash: input_model_hash,
        compact_snapshot: Arc::new(compact_snapshot),
        suggested_name,
        source_basename,
        display_bodies: Arc::new(display_bodies),
        diagnostics,
    })
}

pub fn insert_prepared_occurrence(
    assembly: &mut AssemblyDocument,
    prepared: PreparedPartDefinition,
    placement: RigidPlacement,
    grounded: bool,
) -> Result<InsertOccurrenceResult, AssemblyOperationError> {
    assembly
        .validate_structural_contracts()
        .map_err(|error| AssemblyOperationError::InvalidAssembly(error.to_string()))?;
    if assembly.occurrences.len() >= 10_000 {
        return Err(AssemblyOperationError::OccurrenceLimitReached);
    }

    let definition_created = !assembly.definitions.contains_key(&prepared.model_hash);
    if definition_created && assembly.definitions.len() >= 2_048 {
        return Err(AssemblyOperationError::DefinitionLimitReached);
    }
    let occurrence_id = assembly.next_occurrence_id;
    let next_occurrence_id = occurrence_id
        .checked_add(1)
        .ok_or(AssemblyOperationError::OccurrenceIdExhausted)?;
    if assembly.occurrences.contains_key(&occurrence_id) {
        return Err(AssemblyOperationError::OccurrenceIdCollision(occurrence_id));
    }

    let definition_name = if let Some(existing) = assembly.definitions.get(&prepared.model_hash) {
        existing.name.clone()
    } else {
        unique_definition_name(assembly, &prepared.suggested_name)
    };
    let occurrence_name = format!("{definition_name}:{occurrence_id}");
    if assembly
        .occurrences
        .values()
        .any(|existing| existing.name.eq_ignore_ascii_case(&occurrence_name))
    {
        return Err(AssemblyOperationError::OccurrenceNameCollision(
            occurrence_name,
        ));
    }

    // Everything that can fail has completed. The authoritative mutation is
    // now a short, infallible commit.
    if definition_created {
        assembly.definitions.insert(
            prepared.model_hash,
            Arc::new(AssemblyDefinition {
                model_hash: prepared.model_hash,
                name: definition_name,
                source_basename: prepared.source_basename.clone(),
                compact_snapshot: prepared.compact_snapshot.clone(),
            }),
        );
    }
    assembly.occurrences.insert(
        occurrence_id,
        AssemblyOccurrence {
            id: occurrence_id,
            definition_model_hash: prepared.model_hash,
            name: occurrence_name,
            manual_placement: placement,
            grounded,
            resolved_placement_override: None,
        },
    );
    assembly.next_occurrence_id = next_occurrence_id;

    Ok(InsertOccurrenceResult {
        occurrence_id,
        definition_model_hash: prepared.model_hash,
        definition_created,
        display_bodies: prepared.display_bodies,
        diagnostics: prepared.diagnostics,
    })
}

/// Prepare an insertion without mutating the live assembly.
pub fn prepare_occurrence_insertion(
    assembly: &AssemblyDocument,
    prepared: PreparedPartDefinition,
    placement: RigidPlacement,
    grounded: bool,
) -> Result<(AssemblyDocument, InsertOccurrenceResult), AssemblyOperationError> {
    let mut candidate = assembly.clone();
    let result = insert_prepared_occurrence(&mut candidate, prepared, placement, grounded)?;
    Ok((candidate, result))
}

pub fn insert_part_snapshot(
    assembly: &mut AssemblyDocument,
    source_bytes: &[u8],
    source_basename: Option<&str>,
    placement: RigidPlacement,
    grounded: bool,
    options: &LoadOptions,
) -> Result<InsertOccurrenceResult, AssemblyOperationError> {
    let prepared = prepare_part_definition(source_bytes, source_basename, options)?;
    insert_prepared_occurrence(assembly, prepared, placement, grounded)
}

/// Replace the definitions referenced by the selected occurrences.
///
/// Preparing and evaluating the candidate is intentionally separate. This
/// function performs every fallible check against an Arc-cheap authoritative
/// clone and returns that candidate without touching the live document.
/// Occurrence identity, authored names, placements, grounding, and visibility
/// selectors therefore survive replacement unchanged.
pub fn prepare_occurrence_replacement(
    assembly: &AssemblyDocument,
    occurrence_ids: &[u64],
    prepared: PreparedPartDefinition,
) -> Result<(AssemblyDocument, ReplaceOccurrencesResult), AssemblyOperationError> {
    assembly
        .validate_structural_contracts()
        .map_err(|error| AssemblyOperationError::InvalidAssembly(error.to_string()))?;
    if occurrence_ids.is_empty() {
        return Err(AssemblyOperationError::EmptyReplacementSelection);
    }

    let mut selected = occurrence_ids.to_vec();
    selected.sort_unstable();
    selected.dedup();
    for id in &selected {
        if !assembly.occurrences.contains_key(id) {
            return Err(AssemblyOperationError::MissingOccurrence(*id));
        }
    }

    let definition_created = !assembly.definitions.contains_key(&prepared.model_hash);
    if definition_created && assembly.definitions.len() >= 2_048 {
        // Replacement may prune a definition, but only after retargeting. Use
        // the candidate below to prove that the final state is within bounds.
        let selected_hashes: HashSet<ModelHash> = selected
            .iter()
            .map(|id| assembly.occurrences[id].definition_model_hash)
            .collect();
        let frees_definition = selected_hashes.iter().any(|hash| {
            assembly.occurrences.values().all(|occurrence| {
                occurrence.definition_model_hash != *hash
                    || selected.binary_search(&occurrence.id).is_ok()
            })
        });
        if !frees_definition {
            return Err(AssemblyOperationError::DefinitionLimitReached);
        }
    }

    let mut candidate = assembly.clone_authoritative();
    if definition_created {
        let name = unique_definition_name(&candidate, &prepared.suggested_name);
        candidate.definitions.insert(
            prepared.model_hash,
            Arc::new(AssemblyDefinition {
                model_hash: prepared.model_hash,
                name,
                source_basename: prepared.source_basename.clone(),
                compact_snapshot: prepared.compact_snapshot.clone(),
            }),
        );
    }
    for id in &selected {
        candidate
            .occurrences
            .get_mut(id)
            .expect("replacement ids were checked above")
            .definition_model_hash = prepared.model_hash;
    }

    let referenced: HashSet<ModelHash> = candidate
        .occurrences
        .values()
        .map(|occurrence| occurrence.definition_model_hash)
        .collect();
    let pruned_definition_hashes: Vec<ModelHash> = candidate
        .definitions
        .keys()
        .copied()
        .filter(|hash| !referenced.contains(hash))
        .collect();
    candidate
        .definitions
        .retain(|hash, _| referenced.contains(hash));
    candidate
        .validate_structural_contracts()
        .map_err(|error| AssemblyOperationError::InvalidAssembly(error.to_string()))?;

    let result = ReplaceOccurrencesResult {
        replaced_occurrence_ids: selected,
        definition_model_hash: prepared.model_hash,
        definition_created,
        pruned_definition_hashes,
        display_bodies: prepared.display_bodies,
        diagnostics: prepared.diagnostics,
    };
    Ok((candidate, result))
}

pub fn replace_occurrences_with_prepared(
    assembly: &mut AssemblyDocument,
    occurrence_ids: &[u64],
    prepared: PreparedPartDefinition,
) -> Result<ReplaceOccurrencesResult, AssemblyOperationError> {
    let (candidate, result) = prepare_occurrence_replacement(assembly, occurrence_ids, prepared)?;
    *assembly = candidate;
    Ok(result)
}

fn unique_definition_name(assembly: &AssemblyDocument, suggested: &str) -> String {
    if !assembly
        .definitions
        .values()
        .any(|definition| definition.name.eq_ignore_ascii_case(suggested))
    {
        return suggested.to_owned();
    }
    for ordinal in 2u64.. {
        let candidate = format!("{suggested} ({ordinal})");
        if !assembly
            .definitions
            .values()
            .any(|definition| definition.name.eq_ignore_ascii_case(&candidate))
        {
            return candidate;
        }
    }
    unreachable!("the u64 definition-name suffix space cannot be exhausted")
}

fn body_is_displayable(mesh: &MockMesh) -> bool {
    if mesh.vertices.len() < 18
        || !mesh.vertices.len().is_multiple_of(6)
        || mesh.indices.len() < 3
        || !mesh.vertices.iter().all(|value| value.is_finite())
    {
        return false;
    }
    let vertex_count = mesh.vertices.len() / 6;
    mesh.indices.chunks_exact(3).any(|triangle| {
        let [a, b, c] = [
            triangle[0] as usize,
            triangle[1] as usize,
            triangle[2] as usize,
        ];
        if a >= vertex_count || b >= vertex_count || c >= vertex_count {
            return false;
        }
        let point = |index: usize| {
            let offset = index * 6;
            [
                mesh.vertices[offset],
                mesh.vertices[offset + 1],
                mesh.vertices[offset + 2],
            ]
        };
        let (a, b, c) = (point(a), point(b), point(c));
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cross = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let area_squared = cross.iter().map(|value| value * value).sum::<f32>();
        area_squared.is_finite() && area_squared > 0.0
    })
}

fn format_load_diagnostic(diagnostic: &LoadDiagnostic) -> String {
    match diagnostic {
        LoadDiagnostic::DiscardedDisposableSection { section, reason } => {
            format!("discarded optional section {section}: {reason}")
        }
        LoadDiagnostic::ExtensionProfileMismatch { expected, actual } => {
            format!("file extension expected {expected} but content is {actual:?}")
        }
        LoadDiagnostic::PrunedStaleVisibility { entity_id } => {
            format!("discarded stale part visibility reference `{entity_id}`")
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssemblyOperationError {
    InvalidPart(String),
    ExpectedPart,
    EvaluationFailed(String),
    NoDisplayableFinalBody,
    ModelHashChangedDuringNormalization,
    InvalidAssembly(String),
    DefinitionLimitReached,
    OccurrenceLimitReached,
    OccurrenceIdExhausted,
    OccurrenceIdCollision(u64),
    OccurrenceNameCollision(String),
    EmptyReplacementSelection,
    MissingOccurrence(u64),
    DegenerateFaceNormal,
    InvalidPlacement(String),
}

impl fmt::Display for AssemblyOperationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPart(error) => write!(formatter, "invalid Part snapshot: {error}"),
            Self::ExpectedPart => formatter.write_str("assembly insertion requires a Part project"),
            Self::EvaluationFailed(error) => {
                write!(formatter, "Part evaluation failed: {error}")
            }
            Self::NoDisplayableFinalBody => {
                formatter.write_str("Part has no displayable final body")
            }
            Self::ModelHashChangedDuringNormalization => {
                formatter.write_str("Part model hash changed while producing its compact snapshot")
            }
            Self::InvalidAssembly(error) => write!(formatter, "invalid assembly: {error}"),
            Self::DefinitionLimitReached => {
                formatter.write_str("assembly definition limit has been reached")
            }
            Self::OccurrenceLimitReached => {
                formatter.write_str("assembly occurrence limit has been reached")
            }
            Self::OccurrenceIdExhausted => {
                formatter.write_str("assembly occurrence id space is exhausted")
            }
            Self::OccurrenceIdCollision(id) => {
                write!(formatter, "next occurrence id {id} is already in use")
            }
            Self::OccurrenceNameCollision(name) => {
                write!(formatter, "occurrence name `{name}` is already in use")
            }
            Self::EmptyReplacementSelection => {
                formatter.write_str("replacement requires at least one occurrence")
            }
            Self::MissingOccurrence(id) => {
                write!(formatter, "replacement occurrence {id} does not exist")
            }
            Self::DegenerateFaceNormal => {
                formatter.write_str("Align Faces requires finite non-zero face normals")
            }
            Self::InvalidPlacement(error) => {
                write!(formatter, "invalid aligned placement: {error}")
            }
        }
    }
}

impl std::error::Error for AssemblyOperationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Document, FeatureNode, FeatureType};

    fn box_part_bytes(created_unix: Option<u64>) -> Vec<u8> {
        let mut document = Document::new();
        document.state.created_unix = created_unix;
        document.evaluator_graph_mut().add_feature(FeatureNode {
            id: "box_1".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        write_project_document_to_vec(
            &ProjectDocument::Part(document),
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .unwrap()
    }

    #[test]
    fn repeated_part_insertions_share_one_definition() {
        let first = box_part_bytes(Some(1));
        let second = box_part_bytes(Some(999));
        assert_ne!(first, second, "metadata should make the snapshots differ");
        let mut assembly = AssemblyDocument::new();
        let a = insert_part_snapshot(
            &mut assembly,
            &first,
            Some("Bracket.zcad"),
            RigidPlacement::IDENTITY,
            false,
            &LoadOptions::default(),
        )
        .unwrap();
        let b = insert_part_snapshot(
            &mut assembly,
            &second,
            Some("renamed.zcadh"),
            RigidPlacement::IDENTITY,
            false,
            &LoadOptions::default(),
        )
        .unwrap();
        assert!(a.definition_created);
        assert!(!b.definition_created);
        assert_eq!(assembly.definitions.len(), 1);
        assert_eq!(assembly.occurrences.len(), 2);
        assert_eq!(assembly.occurrences[&1].name, "Bracket:1");
        assert_eq!(assembly.occurrences[&2].name, "Bracket:2");
    }

    #[test]
    fn prepared_insertion_does_not_mutate_the_live_assembly() {
        let bytes = box_part_bytes(None);
        let prepared =
            prepare_part_definition(&bytes, Some("Bracket.zcad"), &LoadOptions::default()).unwrap();
        let assembly = AssemblyDocument::new();
        let before = assembly.clone();
        let (candidate, result) =
            prepare_occurrence_insertion(&assembly, prepared, RigidPlacement::IDENTITY, false)
                .unwrap();
        assert_eq!(assembly, before);
        assert_eq!(result.occurrence_id, 1);
        assert_eq!(candidate.occurrences.len(), 1);
    }

    #[test]
    fn failed_prepare_is_transactional() {
        let mut assembly = AssemblyDocument::new();
        let before = assembly.clone();
        let error = insert_part_snapshot(
            &mut assembly,
            b"not a zcad file",
            None,
            RigidPlacement::IDENTITY,
            false,
            &LoadOptions::default(),
        )
        .unwrap_err();
        assert!(matches!(error, AssemblyOperationError::InvalidPart(_)));
        assert_eq!(assembly, before);
    }

    #[test]
    fn align_faces_uses_parametric_u_for_exact_180_degrees() {
        let result = align_faces(
            RigidPlacement::IDENTITY,
            AlignFaceFrame {
                centroid: [1.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                parametric_u: Some([1.0, 0.0, 0.0]),
            },
            RigidPlacement::IDENTITY,
            AlignFaceFrame {
                centroid: [5.0, 4.0, 3.0],
                normal: [0.0, 0.0, 1.0],
                parametric_u: None,
            },
        )
        .unwrap();
        assert_eq!(result.rotation(), [0.0, 1.0, 0.0, 0.0]);
        assert_eq!(result.translation(), [4.0, 4.0, 3.0]);
    }

    #[test]
    fn align_faces_threshold_has_deterministic_results_on_both_sides() {
        for noise in [0.5e-6, 2.0e-6] {
            let face = AlignFaceFrame {
                centroid: [0.0; 3],
                normal: [noise, 0.0, 1.0],
                parametric_u: Some([1.0, 0.0, 0.0]),
            };
            let first = align_faces(
                RigidPlacement::IDENTITY,
                face,
                RigidPlacement::IDENTITY,
                AlignFaceFrame {
                    centroid: [0.0; 3],
                    normal: [0.0, 0.0, 1.0],
                    parametric_u: None,
                },
            )
            .unwrap();
            let second = align_faces(
                RigidPlacement::IDENTITY,
                face,
                RigidPlacement::IDENTITY,
                AlignFaceFrame {
                    centroid: [0.0; 3],
                    normal: [0.0, 0.0, 1.0],
                    parametric_u: None,
                },
            )
            .unwrap();
            assert_eq!(first, second);
        }
    }

    #[test]
    fn bom_counts_hidden_occurrences_and_csv_uses_full_hash() {
        let bytes = box_part_bytes(None);
        let mut assembly = AssemblyDocument::new();
        for _ in 0..2 {
            insert_part_snapshot(
                &mut assembly,
                &bytes,
                Some("Bracket, Left.zcad"),
                RigidPlacement::IDENTITY,
                false,
                &LoadOptions::default(),
            )
            .unwrap();
        }
        assembly.presentation.hidden_occurrences.insert(2);
        let rows = assembly_bom(&assembly);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].quantity, 2);
        let csv = assembly_bom_csv(&assembly);
        assert!(csv.contains("\"Bracket, Left\""));
        assert!(csv.contains(&"00".repeat(32)) == (rows[0].model_hash == [0; 32]));
        let full_hash: String = rows[0]
            .model_hash
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        assert!(csv.contains(&full_hash));
    }

    #[test]
    fn replacement_preserves_occurrence_state_and_prunes_unused_definition() {
        let first = box_part_bytes(None);
        let mut replacement_document = Document::new();
        replacement_document
            .evaluator_graph_mut()
            .add_feature(FeatureNode {
                id: "replacement".into(),
                name: "Replacement".into(),
                feature: FeatureType::Box {
                    w: 40.0,
                    h: 20.0,
                    d: 10.0,
                },
            });
        let replacement = write_project_document_to_vec(
            &ProjectDocument::Part(replacement_document),
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .unwrap();
        let mut assembly = AssemblyDocument::new();
        insert_part_snapshot(
            &mut assembly,
            &first,
            Some("Original.zcad"),
            RigidPlacement::new([3.0, 4.0, 5.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            true,
            &LoadOptions::default(),
        )
        .unwrap();
        assembly.presentation.hidden_occurrences.insert(1);
        assembly
            .presentation
            .hidden_bodies
            .insert((1, "selector-that-may-dangle".into()));
        let before = assembly.occurrences[&1].clone();
        let prepared = prepare_part_definition(
            &replacement,
            Some("Replacement.zcad"),
            &LoadOptions::default(),
        )
        .unwrap();
        let result = replace_occurrences_with_prepared(&mut assembly, &[1], prepared).unwrap();
        let after = &assembly.occurrences[&1];
        assert_eq!(after.id, before.id);
        assert_eq!(after.name, before.name);
        assert_eq!(after.manual_placement, before.manual_placement);
        assert_eq!(after.grounded, before.grounded);
        assert!(assembly.presentation.hidden_occurrences.contains(&1));
        assert!(assembly
            .presentation
            .hidden_bodies
            .contains(&(1, "selector-that-may-dangle".into())));
        assert_eq!(assembly.definitions.len(), 1);
        assert_eq!(result.pruned_definition_hashes.len(), 1);
    }

    #[test]
    fn failed_replacement_is_transactional() {
        let first = box_part_bytes(None);
        let mut assembly = AssemblyDocument::new();
        insert_part_snapshot(
            &mut assembly,
            &first,
            Some("Original.zcad"),
            RigidPlacement::IDENTITY,
            false,
            &LoadOptions::default(),
        )
        .unwrap();
        let before = assembly.clone();
        let prepared =
            prepare_part_definition(&first, Some("Replacement.zcad"), &LoadOptions::default())
                .unwrap();
        let error = replace_occurrences_with_prepared(&mut assembly, &[999], prepared).unwrap_err();
        assert_eq!(error, AssemblyOperationError::MissingOccurrence(999));
        assert_eq!(assembly, before);
    }
}
