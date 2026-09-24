//! Final B-Rep export, independent of display meshes and GUI state.
use crate::Document;
use std::collections::HashSet;
use std::path::Path;

/// Export visible placed occurrences, sharing definition geometry. The snapshot
/// must retain derived solved placements (do not use clone_authoritative).
pub fn write_assembly_step(
    assembly: &crate::AssemblyDocument,
    path: &Path,
) -> Result<usize, String> {
    use openrcad::foundation::{Ax3, Dir, Pnt};
    assembly
        .validate_structural_contracts()
        .map_err(|e| e.to_string())?;
    if assembly.mates.as_ref().is_some_and(|mates| {
        mates.mates.values().any(|mate| {
            !mate.suppressed
                && std::iter::once(&mate.first)
                    .chain(mate.second.iter())
                    .any(|entity| {
                        let occurrence = &assembly.occurrences[&entity.occurrence_id];
                        !occurrence.grounded && occurrence.resolved_placement_override.is_none()
                    })
        })
    }) {
        return Err("Solve assembly mates before STEP export".into());
    }
    let mut definitions = Vec::new();
    let mut indices = std::collections::BTreeMap::new();
    let mut occurrences = Vec::new();
    for occurrence in assembly.occurrences.values() {
        if assembly
            .presentation
            .hidden_occurrences
            .contains(&occurrence.id)
        {
            continue;
        }
        let hidden: Vec<_> = assembly
            .presentation
            .hidden_bodies
            .iter()
            .filter(|(id, _)| *id == occurrence.id)
            .map(|(_, body)| body.clone())
            .collect();
        let key = (occurrence.definition_model_hash, hidden.clone());
        let index = if let Some(index) = indices.get(&key) {
            *index
        } else {
            let definition = &assembly.definitions[&occurrence.definition_model_hash];
            let loaded = crate::read_document_from_slice(
                &definition.compact_snapshot,
                &crate::LoadOptions::default(),
            )
            .map_err(|e| e.to_string())?;
            let bodies = loaded
                .document
                .evaluated_kernel_bodies(&hidden.into_iter().collect())?;
            let solids: Vec<_> = bodies.into_iter().flat_map(|(_, solids)| solids).collect();
            if solids.is_empty() {
                continue;
            }
            let index = definitions.len();
            definitions.push((definition.name.clone(), solids));
            indices.insert(key, index);
            index
        };
        let placement = occurrence.resolved_placement();
        let [w, x, y, z] = placement.rotation();
        let [tx, ty, tz] = placement.translation();
        let axis = Dir::new(
            2.0 * (x * z + y * w),
            2.0 * (y * z - x * w),
            1.0 - 2.0 * (x * x + y * y),
        );
        let radial = Dir::new(
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + z * w),
            2.0 * (x * z - y * w),
        );
        occurrences.push(openrcad::exchange::StepOccurrence {
            name: &occurrence.name,
            definition: index,
            placement: Ax3::new_axes(Pnt::new(tx, ty, tz), axis, radial),
        });
    }
    let refs: Vec<_> = definitions
        .iter()
        .map(|(name, solids)| (name.as_str(), solids.iter().collect()))
        .collect();
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let mut temporary = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    openrcad::exchange::write_step_assembly(&refs, &occurrences, temporary.as_file_mut())
        .map_err(|e| e.to_string())?;
    temporary.as_file().sync_all().map_err(|e| e.to_string())?;
    temporary.persist(path).map_err(|e| e.to_string())?;
    Ok(occurrences.len())
}

/// Export the supplied document snapshot. Callers capture a document revision
/// before dispatching this work so later edits cannot change the exported model.
/// Kernel coordinates are millimetres regardless of the document display unit.
pub fn write_document_step(
    document: &Document,
    hidden: &HashSet<String>,
    path: &Path,
) -> Result<usize, String> {
    let bodies = document.evaluated_kernel_bodies(hidden)?;
    let mut named = Vec::new();
    for (id, parts) in bodies {
        let name = document
            .graph
            .node_weights()
            .find(|node| node.id == id)
            .map(|node| node.name.clone())
            .unwrap_or(id);
        let count = parts.len();
        for (index, solid) in parts.into_iter().enumerate() {
            named.push((
                if count == 1 {
                    name.clone()
                } else {
                    format!("{name} {}", index + 1)
                },
                solid,
            ));
        }
    }
    let refs: Vec<_> = named
        .iter()
        .map(|(name, solid)| (name.as_str(), solid))
        .collect();
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let mut temporary = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    openrcad::exchange::write_step_bodies(&refs, temporary.as_file_mut())
        .map_err(|e| e.to_string())?;
    temporary.as_file().sync_all().map_err(|e| e.to_string())?;
    temporary.persist(path).map_err(|e| e.to_string())?;
    let _ = std::fs::File::open(parent).and_then(|directory| directory.sync_all());
    Ok(named.len())
}
