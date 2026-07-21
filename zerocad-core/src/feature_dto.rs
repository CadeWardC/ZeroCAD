//! Numeric-field v1 payload DTOs for `.zcad` recipes.
//!
//! These mappings are persistence ABI. Rust variant and field names may change;
//! the numeric slots below may not. A future field-layout change increments the
//! feature kind's payload version and adds an explicit decoder.

use crate::parametric::{
    AxisBase, DatumAxisDef, DatumPlaneDef, DatumPointDef, DraftNeutral, EdgeRef, ExtrudeMode,
    FaceRef, FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind,
    FeatureType, HoleKind, HoleManufacturingMetadata, LoftSurfaceMode, PatternKind, PlaneBase,
    StandardReference, Variable,
};
use crate::sketch::{
    CornerKind, CornerMod, EntityId, SketchMirror, SketchShape, SketchSolverModel,
};
use crate::{CoordinateSystem, SketchCurves};
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::BTreeMap;

pub(crate) type NumericFeatureFields = BTreeMap<u16, ciborium::value::Value>;

fn value<T: Serialize>(source: &T) -> ciborium::value::Value {
    let mut bytes = Vec::new();
    ciborium::into_writer(source, &mut bytes).expect("feature payload value must serialize");
    ciborium::from_reader(bytes.as_slice()).expect("serialized feature payload must decode")
}

fn put<T: Serialize>(fields: &mut NumericFeatureFields, key: u16, source: &T) {
    fields.insert(key, value(source));
}

fn take<T: DeserializeOwned>(
    fields: &mut NumericFeatureFields,
    key: u16,
    name: &str,
) -> Result<T, String> {
    let value = fields
        .remove(&key)
        .ok_or_else(|| format!("feature payload is missing field {key} ({name})"))?;
    let mut bytes = Vec::new();
    ciborium::into_writer(&value, &mut bytes).map_err(|error| error.to_string())?;
    ciborium::from_reader(bytes.as_slice())
        .map_err(|error| format!("invalid feature field {key} ({name}): {error}"))
}

fn take_if_present<T: DeserializeOwned>(
    fields: &mut NumericFeatureFields,
    key: u16,
    name: &str,
) -> Result<Option<T>, String> {
    if fields.contains_key(&key) {
        take(fields, key, name).map(Some)
    } else {
        Ok(None)
    }
}

fn finish(kind: &str, fields: NumericFeatureFields) -> Result<(), String> {
    if fields.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "feature kind '{kind}' contains unknown v1 field ids {:?}",
            fields.keys().collect::<Vec<_>>()
        ))
    }
}

pub(crate) fn encode(feature: &FeatureType) -> NumericFeatureFields {
    let mut fields = NumericFeatureFields::new();
    match feature {
        FeatureType::Origin => {}
        FeatureType::Box { w, h, d } => {
            put(&mut fields, 0, w);
            put(&mut fields, 1, h);
            put(&mut fields, 2, d);
        }
        FeatureType::Cylinder { r, h } => {
            put(&mut fields, 0, r);
            put(&mut fields, 1, h);
        }
        FeatureType::Sketch {
            cs,
            curves,
            shapes,
            corner_mods,
            mirrors,
            on_face,
            entity_ids,
            next_entity_id,
            solver,
        } => {
            put(&mut fields, 0, cs);
            put(&mut fields, 1, curves);
            put(&mut fields, 2, shapes);
            put(&mut fields, 3, corner_mods);
            put(&mut fields, 4, mirrors);
            put(&mut fields, 5, on_face);
            put(&mut fields, 6, entity_ids);
            put(&mut fields, 7, next_entity_id);
            let mut legacy_solver = solver.clone();
            if let Some(model) = legacy_solver.as_mut() {
                model.patterns.clear();
            }
            put(&mut fields, 8, &legacy_solver);
        }
        FeatureType::Extrude {
            depth,
            region_indices,
            mode,
            target,
            depth_expr,
            ..
        } => {
            put(&mut fields, 0, depth);
            put(&mut fields, 1, region_indices);
            put(&mut fields, 2, mode);
            put(&mut fields, 3, target);
            put(&mut fields, 4, depth_expr);
        }
        FeatureType::EdgeMod {
            target,
            edge,
            dist,
            dist_expr,
            kind,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, edge);
            put(&mut fields, 2, dist);
            put(&mut fields, 3, dist_expr);
            // Field 4 was the pre-Phase-3 construction-replay hint. Keep its
            // numeric slot reserved so v5 payload numbering remains stable.
            put(&mut fields, 4, &());
            put(&mut fields, 5, kind);
        }
        FeatureType::VariableSet { variables } => put(&mut fields, 0, variables),
        FeatureType::Import { .. } => {}
        FeatureType::Revolve {
            axis,
            angle_deg,
            angle_expr,
            region_indices,
            mode,
            target,
        } => {
            put(&mut fields, 0, axis);
            put(&mut fields, 1, angle_deg);
            put(&mut fields, 2, angle_expr);
            put(&mut fields, 3, region_indices);
            put(&mut fields, 4, mode);
            put(&mut fields, 5, target);
        }
        FeatureType::Loft {
            sections,
            mode,
            target,
            ..
        } => {
            put(&mut fields, 0, sections);
            put(&mut fields, 1, mode);
            put(&mut fields, 2, target);
        }
        FeatureType::Sweep {
            profile_sketch,
            profile_region,
            path_sketch,
            mode,
            target,
            ..
        } => {
            put(&mut fields, 0, profile_sketch);
            put(&mut fields, 1, profile_region);
            put(&mut fields, 2, path_sketch);
            put(&mut fields, 3, mode);
            put(&mut fields, 4, target);
        }
        FeatureType::Shell {
            target,
            thickness,
            thickness_expr,
            open_faces,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, thickness);
            put(&mut fields, 2, thickness_expr);
            put(&mut fields, 3, open_faces);
        }
        FeatureType::Hole {
            target,
            position,
            direction,
            diameter,
            diameter_expr,
            depth,
            kind,
            standard,
            manufacturing,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, position);
            put(&mut fields, 2, direction);
            put(&mut fields, 3, diameter);
            put(&mut fields, 4, diameter_expr);
            put(&mut fields, 5, depth);
            put(&mut fields, 6, kind);
            put(&mut fields, 7, standard);
            if manufacturing.is_some() {
                put(&mut fields, 8, manufacturing);
            }
        }
        FeatureType::Pattern { source, kind } => {
            put(&mut fields, 0, source);
            put(&mut fields, 1, kind);
        }
        FeatureType::BodyTransform {
            source,
            translation,
            copy,
        } => {
            put(&mut fields, 0, source);
            put(&mut fields, 1, translation);
            put(&mut fields, 2, copy);
        }
        FeatureType::Thread {
            target,
            face,
            internal,
            pitch,
            depth,
            angle_deg,
            right_handed,
            starts,
            length,
            flip,
            designation,
            standard,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, face);
            put(&mut fields, 2, internal);
            put(&mut fields, 3, pitch);
            put(&mut fields, 4, depth);
            put(&mut fields, 5, angle_deg);
            put(&mut fields, 6, right_handed);
            put(&mut fields, 7, starts);
            put(&mut fields, 8, length);
            put(&mut fields, 9, flip);
            put(&mut fields, 10, designation);
            put(&mut fields, 11, standard);
        }
        FeatureType::DatumPlane { def } => put(&mut fields, 0, def),
        FeatureType::DatumAxis { def } => put(&mut fields, 0, def),
        FeatureType::DatumPoint { def } => put(&mut fields, 0, def),
        FeatureType::BodyJoin { sources } => put(&mut fields, 0, sources),
        FeatureType::BodyCut {
            target,
            tool,
            keep_tool,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, tool);
            put(&mut fields, 2, keep_tool);
        }
        FeatureType::BodyIntersect {
            target,
            tool,
            keep_tool,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, tool);
            put(&mut fields, 2, keep_tool);
        }
        FeatureType::BodySplit {
            target,
            plane,
            face,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, plane);
            put(&mut fields, 2, face);
        }
        FeatureType::BodyScale {
            source,
            factor,
            factor_expr,
            center,
        } => {
            put(&mut fields, 0, source);
            put(&mut fields, 1, factor);
            put(&mut fields, 2, factor_expr);
            put(&mut fields, 3, center);
        }
        FeatureType::FaceOffset {
            target,
            face,
            distance,
            distance_expr,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, face);
            put(&mut fields, 2, distance);
            put(&mut fields, 3, distance_expr);
        }
        FeatureType::FaceMove {
            target,
            face,
            translation,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, face);
            put(&mut fields, 2, translation);
        }
        FeatureType::FaceDelete { target, face } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, face);
        }
        FeatureType::FaceThicken {
            target,
            face,
            thickness,
            thickness_expr,
            reverse,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, face);
            put(&mut fields, 2, thickness);
            put(&mut fields, 3, thickness_expr);
            put(&mut fields, 4, reverse);
        }
        FeatureType::ImportStl { .. } => {}
        FeatureType::FeaturePattern {
            target,
            source_feature,
            kind,
            compute_mode,
            extent_policy,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, source_feature);
            put(&mut fields, 2, kind);
            put(&mut fields, 3, compute_mode);
            put(&mut fields, 4, extent_policy);
        }
        FeatureType::Draft {
            target,
            faces,
            neutral,
            angle_deg,
            angle_expr,
            flip_pull,
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, faces);
            put(&mut fields, 2, neutral);
            put(&mut fields, 3, angle_deg);
            put(&mut fields, 4, angle_expr);
            put(&mut fields, 5, flip_pull);
        }
    }
    fields
}

/// Encode one feature using the numeric layout registered for `payload_schema`.
/// Schema 1 remains byte-for-byte stable; Extrude v2 appends only new slots.
pub(crate) fn encode_for_schema(
    feature: &FeatureType,
    payload_schema: u16,
) -> NumericFeatureFields {
    let mut fields = encode(feature);
    if let FeatureType::Extrude {
        draft_angle_deg,
        draft_angle_expr,
        ..
    } = feature
    {
        if payload_schema >= 2 {
            put(&mut fields, 5, draft_angle_deg);
            put(&mut fields, 6, draft_angle_expr);
        }
    }
    if let FeatureType::Sweep {
        total_twist_deg,
        total_twist_expr,
        ..
    } = feature
    {
        if payload_schema >= 2 {
            put(&mut fields, 5, total_twist_deg);
            put(&mut fields, 6, total_twist_expr);
        }
    }
    if let FeatureType::Loft { surface_mode, .. } = feature {
        if payload_schema >= 2 {
            put(&mut fields, 3, surface_mode);
        }
    }
    if let FeatureType::Sketch { solver, .. } = feature {
        if payload_schema >= 3 {
            let patterns = solver
                .as_ref()
                .map(|model| model.patterns.as_slice())
                .unwrap_or(&[]);
            put(&mut fields, 9, &patterns);
        }
    }
    fields
}

pub(crate) fn decode_with_decoder(
    kind: &str,
    payload_schema: u16,
    decoder: crate::document::FeaturePayloadDecoder,
    fields: NumericFeatureFields,
) -> Result<FeatureType, String> {
    match decoder {
        crate::document::FeaturePayloadDecoder::NumericFieldsV1 => decode_v1(kind, fields),
        crate::document::FeaturePayloadDecoder::NumericFieldsV2 => decode_v2(kind, fields),
        crate::document::FeaturePayloadDecoder::NumericFieldsV3 => decode_v3(kind, fields),
        crate::document::FeaturePayloadDecoder::StepAssetV1
        | crate::document::FeaturePayloadDecoder::StlAssetV1 => Err(format!(
            "feature kind '{kind}' schema {payload_schema} requires a content-addressed asset payload"
        )),
    }
}

fn decode_v3(kind: &str, mut fields: NumericFeatureFields) -> Result<FeatureType, String> {
    if kind != "sketch.sketch" {
        return decode_v2(kind, fields);
    }
    let mut solver = take::<Option<SketchSolverModel>>(&mut fields, 8, "solver")?;
    let patterns =
        take::<Vec<crate::sketch::SketchPatternOperation>>(&mut fields, 9, "associative patterns")?;
    if !patterns.is_empty() {
        solver
            .get_or_insert_with(SketchSolverModel::default)
            .patterns = patterns;
    }
    let feature = FeatureType::Sketch {
        cs: take::<CoordinateSystem>(&mut fields, 0, "coordinate system")?,
        curves: take::<SketchCurves>(&mut fields, 1, "curves")?,
        shapes: take::<Vec<SketchShape>>(&mut fields, 2, "shapes")?,
        corner_mods: take::<Vec<CornerMod>>(&mut fields, 3, "corner modifiers")?,
        mirrors: take::<Vec<SketchMirror>>(&mut fields, 4, "mirrors")?,
        on_face: take(&mut fields, 5, "on face")?,
        entity_ids: take::<Vec<EntityId>>(&mut fields, 6, "entity ids")?,
        next_entity_id: take(&mut fields, 7, "next entity id")?,
        solver,
    };
    finish(kind, fields)?;
    Ok(feature)
}

fn decode_v2(kind: &str, mut fields: NumericFeatureFields) -> Result<FeatureType, String> {
    let feature = match kind {
        "part.extrude" => FeatureType::Extrude {
            depth: take(&mut fields, 0, "depth")?,
            region_indices: take(&mut fields, 1, "regions")?,
            mode: take::<ExtrudeMode>(&mut fields, 2, "mode")?,
            target: take(&mut fields, 3, "target")?,
            depth_expr: take(&mut fields, 4, "depth expression")?,
            draft_angle_deg: take(&mut fields, 5, "draft angle")?,
            draft_angle_expr: take(&mut fields, 6, "draft angle expression")?,
        },
        "part.sweep" => FeatureType::Sweep {
            profile_sketch: take(&mut fields, 0, "profile sketch")?,
            profile_region: take(&mut fields, 1, "profile region")?,
            path_sketch: take(&mut fields, 2, "path sketch")?,
            mode: take::<ExtrudeMode>(&mut fields, 3, "mode")?,
            target: take(&mut fields, 4, "target")?,
            total_twist_deg: take(&mut fields, 5, "total twist")?,
            total_twist_expr: take(&mut fields, 6, "total twist expression")?,
        },
        "part.loft" => FeatureType::Loft {
            sections: take::<Vec<(String, usize)>>(&mut fields, 0, "sections")?,
            mode: take::<ExtrudeMode>(&mut fields, 1, "mode")?,
            target: take(&mut fields, 2, "target")?,
            surface_mode: take::<LoftSurfaceMode>(&mut fields, 3, "surface mode")?,
        },
        "datum.plane" => FeatureType::DatumPlane {
            def: take::<DatumPlaneDef>(&mut fields, 0, "definition")?,
        },
        "datum.axis" => FeatureType::DatumAxis {
            def: take::<DatumAxisDef>(&mut fields, 0, "definition")?,
        },
        "datum.point" => FeatureType::DatumPoint {
            def: take::<DatumPointDef>(&mut fields, 0, "definition")?,
        },
        _ => return decode_v1(kind, fields),
    };
    finish(kind, fields)?;
    Ok(feature)
}

#[cfg(test)]
fn decode(
    kind: &str,
    payload_schema: u16,
    fields: NumericFeatureFields,
) -> Result<FeatureType, String> {
    let registration = crate::document::FeatureRegistry::get(kind)
        .ok_or_else(|| format!("unknown feature kind '{kind}'"))?;
    let decoder = registration
        .payload_decoder(payload_schema)
        .ok_or_else(|| unsupported_payload_schema(registration, payload_schema))?;
    decode_with_decoder(kind, payload_schema, decoder, fields)
}

pub(crate) fn unsupported_payload_schema(
    registration: &crate::document::FeatureRegistration,
    payload_schema: u16,
) -> String {
    let supported: Vec<_> = registration
        .payload_decoders
        .iter()
        .map(|decoder| decoder.schema)
        .collect();
    format!(
        "unsupported payload schema {payload_schema} for feature kind '{}'; supported schemas: {supported:?}",
        registration.kind_id
    )
}

fn decode_v1(kind: &str, mut fields: NumericFeatureFields) -> Result<FeatureType, String> {
    let feature = match kind {
        "core.origin" => FeatureType::Origin,
        "part.box" => FeatureType::Box {
            w: take(&mut fields, 0, "width")?,
            h: take(&mut fields, 1, "height")?,
            d: take(&mut fields, 2, "depth")?,
        },
        "part.cylinder" => FeatureType::Cylinder {
            r: take(&mut fields, 0, "radius")?,
            h: take(&mut fields, 1, "height")?,
        },
        "sketch.sketch" => FeatureType::Sketch {
            cs: take::<CoordinateSystem>(&mut fields, 0, "coordinate system")?,
            curves: take::<SketchCurves>(&mut fields, 1, "curves")?,
            shapes: take::<Vec<SketchShape>>(&mut fields, 2, "shapes")?,
            corner_mods: take::<Vec<CornerMod>>(&mut fields, 3, "corner modifiers")?,
            mirrors: take::<Vec<SketchMirror>>(&mut fields, 4, "mirrors")?,
            on_face: take(&mut fields, 5, "on face")?,
            entity_ids: take::<Vec<EntityId>>(&mut fields, 6, "entity ids")?,
            next_entity_id: take(&mut fields, 7, "next entity id")?,
            solver: take::<Option<SketchSolverModel>>(&mut fields, 8, "solver")?,
        },
        "part.extrude" => FeatureType::Extrude {
            depth: take(&mut fields, 0, "depth")?,
            region_indices: take(&mut fields, 1, "regions")?,
            mode: take::<ExtrudeMode>(&mut fields, 2, "mode")?,
            target: take(&mut fields, 3, "target")?,
            depth_expr: take(&mut fields, 4, "depth expression")?,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
        "part.edge_mod" => FeatureType::EdgeMod {
            target: take(&mut fields, 0, "target")?,
            edge: take::<EdgeRef>(&mut fields, 1, "edge")?,
            dist: take(&mut fields, 2, "distance")?,
            dist_expr: take(&mut fields, 3, "distance expression")?,
            kind: {
                // Replay hints are intentionally discarded: Phase 3 evaluates
                // the current body through the native operation pipeline.
                fields.remove(&4).ok_or_else(|| {
                    "feature payload is missing field 4 (reserved replay slot)".to_string()
                })?;
                take::<CornerKind>(&mut fields, 5, "kind")?
            },
        },
        "document.variables" => FeatureType::VariableSet {
            variables: take::<Vec<Variable>>(&mut fields, 0, "variables")?,
        },
        "part.revolve" => FeatureType::Revolve {
            axis: take::<AxisBase>(&mut fields, 0, "axis")?,
            angle_deg: take(&mut fields, 1, "angle")?,
            angle_expr: take(&mut fields, 2, "angle expression")?,
            region_indices: take(&mut fields, 3, "regions")?,
            mode: take::<ExtrudeMode>(&mut fields, 4, "mode")?,
            target: take(&mut fields, 5, "target")?,
        },
        "part.loft" => FeatureType::Loft {
            sections: take::<Vec<(String, usize)>>(&mut fields, 0, "sections")?,
            mode: take::<ExtrudeMode>(&mut fields, 1, "mode")?,
            target: take(&mut fields, 2, "target")?,
            surface_mode: LoftSurfaceMode::Ruled,
        },
        "part.sweep" => FeatureType::Sweep {
            profile_sketch: take(&mut fields, 0, "profile sketch")?,
            profile_region: take(&mut fields, 1, "profile region")?,
            path_sketch: take(&mut fields, 2, "path sketch")?,
            mode: take::<ExtrudeMode>(&mut fields, 3, "mode")?,
            target: take(&mut fields, 4, "target")?,
            total_twist_deg: 0.0,
            total_twist_expr: None,
        },
        "part.shell" => FeatureType::Shell {
            target: take(&mut fields, 0, "target")?,
            thickness: take(&mut fields, 1, "thickness")?,
            thickness_expr: take(&mut fields, 2, "thickness expression")?,
            open_faces: take::<Vec<FaceRef>>(&mut fields, 3, "open faces")?,
        },
        "part.hole" => FeatureType::Hole {
            target: take(&mut fields, 0, "target")?,
            position: take(&mut fields, 1, "position")?,
            direction: take(&mut fields, 2, "direction")?,
            diameter: take(&mut fields, 3, "diameter")?,
            diameter_expr: take(&mut fields, 4, "diameter expression")?,
            depth: take(&mut fields, 5, "depth")?,
            kind: take::<HoleKind>(&mut fields, 6, "kind")?,
            standard: take_if_present::<Option<StandardReference>>(
                &mut fields,
                7,
                "standard reference",
            )?
            .flatten(),
            manufacturing: take_if_present::<Option<HoleManufacturingMetadata>>(
                &mut fields,
                8,
                "manufacturing metadata",
            )?
            .flatten(),
        },
        "part.pattern" => FeatureType::Pattern {
            source: take(&mut fields, 0, "source")?,
            kind: take::<PatternKind>(&mut fields, 1, "kind")?,
        },
        "part.feature_pattern" => FeatureType::FeaturePattern {
            target: take(&mut fields, 0, "target")?,
            source_feature: take(&mut fields, 1, "source feature")?,
            kind: take::<FeaturePatternKind>(&mut fields, 2, "pattern kind")?,
            compute_mode: take::<FeaturePatternComputeMode>(&mut fields, 3, "compute mode")?,
            extent_policy: take::<FeaturePatternExtentPolicy>(&mut fields, 4, "extent policy")?,
        },
        "part.draft" => FeatureType::Draft {
            target: take(&mut fields, 0, "target")?,
            faces: take::<Vec<FaceRef>>(&mut fields, 1, "faces")?,
            neutral: take::<DraftNeutral>(&mut fields, 2, "neutral")?,
            angle_deg: take(&mut fields, 3, "angle")?,
            angle_expr: take(&mut fields, 4, "angle expression")?,
            flip_pull: take(&mut fields, 5, "flip pull")?,
        },
        "part.transform" => FeatureType::BodyTransform {
            source: take(&mut fields, 0, "source")?,
            translation: take(&mut fields, 1, "translation")?,
            copy: take(&mut fields, 2, "copy")?,
        },
        "part.thread" => FeatureType::Thread {
            target: take(&mut fields, 0, "target")?,
            face: take::<FaceRef>(&mut fields, 1, "face")?,
            internal: take(&mut fields, 2, "internal")?,
            pitch: take(&mut fields, 3, "pitch")?,
            depth: take(&mut fields, 4, "depth")?,
            angle_deg: take(&mut fields, 5, "angle")?,
            right_handed: take(&mut fields, 6, "right handed")?,
            starts: take(&mut fields, 7, "starts")?,
            length: take(&mut fields, 8, "length")?,
            flip: take(&mut fields, 9, "flip")?,
            designation: take(&mut fields, 10, "designation")?,
            standard: take_if_present::<Option<StandardReference>>(
                &mut fields,
                11,
                "standard reference",
            )?
            .flatten(),
        },
        "datum.plane" => FeatureType::DatumPlane {
            def: take::<DatumPlaneDef>(&mut fields, 0, "definition")?,
        },
        "datum.axis" => FeatureType::DatumAxis {
            def: take::<DatumAxisDef>(&mut fields, 0, "definition")?,
        },
        "datum.point" => FeatureType::DatumPoint {
            def: take::<DatumPointDef>(&mut fields, 0, "definition")?,
        },
        "part.join" => FeatureType::BodyJoin {
            sources: take(&mut fields, 0, "sources")?,
        },
        "part.cut" => FeatureType::BodyCut {
            target: take(&mut fields, 0, "target")?,
            tool: take(&mut fields, 1, "tool")?,
            keep_tool: take(&mut fields, 2, "keep tool")?,
        },
        "part.intersect" => FeatureType::BodyIntersect {
            target: take(&mut fields, 0, "target")?,
            tool: take(&mut fields, 1, "tool")?,
            keep_tool: take(&mut fields, 2, "keep tool")?,
        },
        "part.split" => FeatureType::BodySplit {
            target: take(&mut fields, 0, "target")?,
            plane: take::<PlaneBase>(&mut fields, 1, "plane")?,
            face: take::<Option<FaceRef>>(&mut fields, 2, "face")?,
        },
        "part.scale" => FeatureType::BodyScale {
            source: take(&mut fields, 0, "source")?,
            factor: take(&mut fields, 1, "factor")?,
            factor_expr: take(&mut fields, 2, "factor expression")?,
            center: take(&mut fields, 3, "center")?,
        },
        "direct.face_offset" => FeatureType::FaceOffset {
            target: take(&mut fields, 0, "target")?,
            face: take::<FaceRef>(&mut fields, 1, "face")?,
            distance: take(&mut fields, 2, "distance")?,
            distance_expr: take(&mut fields, 3, "distance expression")?,
        },
        "direct.face_move" => FeatureType::FaceMove {
            target: take(&mut fields, 0, "target")?,
            face: take::<FaceRef>(&mut fields, 1, "face")?,
            translation: take(&mut fields, 2, "translation")?,
        },
        "direct.face_delete" => FeatureType::FaceDelete {
            target: take(&mut fields, 0, "target")?,
            face: take::<FaceRef>(&mut fields, 1, "face")?,
        },
        "direct.face_thicken" => FeatureType::FaceThicken {
            target: take(&mut fields, 0, "target")?,
            face: take::<FaceRef>(&mut fields, 1, "face")?,
            thickness: take(&mut fields, 2, "thickness")?,
            thickness_expr: take(&mut fields, 3, "thickness expression")?,
            reverse: take(&mut fields, 4, "reverse")?,
        },
        "exchange.step_import" => {
            return Err("STEP imports must use a content-addressed asset payload".into())
        }
        "exchange.stl_import" => {
            return Err("STL imports must use a content-addressed asset payload".into())
        }
        _ => return Err(format!("unknown feature kind '{kind}'")),
    };
    match &feature {
        FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace { .. },
        }
        | FeatureType::DatumAxis {
            def:
                DatumAxisDef::Edge { .. }
                | DatumAxisDef::CylindricalOrConicalFace { .. }
                | DatumAxisDef::TwoVertices { .. },
        }
        | FeatureType::DatumPoint {
            def:
                DatumPointDef::Vertex { .. }
                | DatumPointDef::Midpoint { .. }
                | DatumPointDef::EdgeMidpoint { .. }
                | DatumPointDef::CircleCenter { .. },
        } => {
            return Err(format!(
                "feature kind '{kind}' uses a geometry-derived datum definition that requires payload schema 2"
            ));
        }
        _ => {}
    }
    finish(kind, fields)?;
    Ok(feature)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrude_v1_decodes_zero_draft_and_v2_round_trips_expression() {
        let feature = FeatureType::Extrude {
            depth: 12.0,
            region_indices: vec![2],
            mode: ExtrudeMode::Join,
            target: Some("body".into()),
            depth_expr: Some("height".into()),
            draft_angle_deg: -3.5,
            draft_angle_expr: Some("-draft".into()),
        };

        let v1 = decode_with_decoder(
            "part.extrude",
            1,
            crate::document::FeaturePayloadDecoder::NumericFieldsV1,
            encode_for_schema(&feature, 1),
        )
        .unwrap();
        assert!(matches!(
            v1,
            FeatureType::Extrude {
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
                ..
            }
        ));

        let v2 = decode_with_decoder(
            "part.extrude",
            2,
            crate::document::FeaturePayloadDecoder::NumericFieldsV2,
            encode_for_schema(&feature, 2),
        )
        .unwrap();
        assert!(matches!(
            v2,
            FeatureType::Extrude {
                draft_angle_deg,
                draft_angle_expr: Some(expression),
                ..
            } if draft_angle_deg == -3.5 && expression == "-draft"
        ));
    }

    #[test]
    fn sweep_v1_decodes_zero_twist_and_v2_round_trips_signed_expression() {
        let feature = FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 2,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: -135.0,
            total_twist_expr: Some("-3*twist".into()),
        };
        let v1 = decode_with_decoder(
            "part.sweep",
            1,
            crate::document::FeaturePayloadDecoder::NumericFieldsV1,
            encode_for_schema(&feature, 1),
        )
        .unwrap();
        assert!(matches!(
            v1,
            FeatureType::Sweep {
                total_twist_deg: 0.0,
                total_twist_expr: None,
                ..
            }
        ));

        let v2 = decode_with_decoder(
            "part.sweep",
            2,
            crate::document::FeaturePayloadDecoder::NumericFieldsV2,
            encode_for_schema(&feature, 2),
        )
        .unwrap();
        assert!(matches!(
            v2,
            FeatureType::Sweep {
                total_twist_deg,
                total_twist_expr: Some(expression),
                ..
            } if total_twist_deg == -135.0 && expression == "-3*twist"
        ));
    }

    #[test]
    fn loft_v1_remains_ruled_and_v2_round_trips_smooth_mode() {
        let feature = FeatureType::Loft {
            sections: vec![("bottom".into(), 0), ("top".into(), 2)],
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        };
        let v1 = decode_with_decoder(
            "part.loft",
            1,
            crate::document::FeaturePayloadDecoder::NumericFieldsV1,
            encode_for_schema(&feature, 1),
        )
        .unwrap();
        assert!(matches!(
            v1,
            FeatureType::Loft {
                surface_mode: LoftSurfaceMode::Ruled,
                ..
            }
        ));

        let v2 = decode_with_decoder(
            "part.loft",
            2,
            crate::document::FeaturePayloadDecoder::NumericFieldsV2,
            encode_for_schema(&feature, 2),
        )
        .unwrap();
        assert!(matches!(
            v2,
            FeatureType::Loft {
                surface_mode: LoftSurfaceMode::Smooth,
                sections,
                ..
            } if sections == vec![("bottom".into(), 0), ("top".into(), 2)]
        ));
        let newer = decode("part.loft", 3, encode_for_schema(&feature, 2))
            .expect_err("an unknown future Loft schema must reject");
        assert!(newer.contains("unsupported payload schema 3"));
        assert!(newer.contains("supported schemas: [1, 2]"));
    }

    #[test]
    fn sketch_v3_adds_associative_patterns_without_changing_v2_solver_payload() {
        let pattern = crate::sketch::SketchPatternOperation {
            id: EntityId(20),
            sources: vec![EntityId(5)],
            kind: crate::sketch::SketchPatternKind::Linear {
                direction: [1.0, 0.0],
                spacing: crate::sketch::Dimension {
                    value: 4.0,
                    expr: Some("pitch".into()),
                },
                count: 3,
            },
        };
        let feature = FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 21,
            solver: Some(SketchSolverModel {
                patterns: vec![pattern.clone()],
                ..SketchSolverModel::default()
            }),
        };

        let v2 = decode_with_decoder(
            "sketch.sketch",
            2,
            crate::document::FeaturePayloadDecoder::NumericFieldsV2,
            encode_for_schema(&feature, 2),
        )
        .unwrap();
        assert!(matches!(
            v2,
            FeatureType::Sketch {
                solver: Some(SketchSolverModel { patterns, .. }),
                ..
            } if patterns.is_empty()
        ));

        let v3 = decode_with_decoder(
            "sketch.sketch",
            3,
            crate::document::FeaturePayloadDecoder::NumericFieldsV3,
            encode_for_schema(&feature, 3),
        )
        .unwrap();
        assert!(matches!(
            v3,
            FeatureType::Sketch {
                solver: Some(SketchSolverModel { patterns, .. }),
                ..
            } if patterns == vec![pattern]
        ));
    }

    #[test]
    fn body_ops_payloads_have_stable_v1_round_trips() {
        let intersect = FeatureType::BodyIntersect {
            target: "target".into(),
            tool: "tool".into(),
            keep_tool: true,
        };
        let decoded = decode("part.intersect", 1, encode(&intersect)).unwrap();
        assert!(matches!(
            decoded,
            FeatureType::BodyIntersect { target, tool, keep_tool }
                if target == "target" && tool == "tool" && keep_tool
        ));

        let split = FeatureType::BodySplit {
            target: "target".into(),
            plane: PlaneBase::Datum("datum".into()),
            face: None,
        };
        let decoded = decode("part.split", 1, encode(&split)).unwrap();
        assert!(matches!(
            decoded,
            FeatureType::BodySplit {
                target,
                plane: PlaneBase::Datum(datum),
                face: None,
            } if target == "target" && datum == "datum"
        ));

        let scale = FeatureType::BodyScale {
            source: "source".into(),
            factor: 2.5,
            factor_expr: Some("scale_factor".into()),
            center: [1.0, 2.0, 3.0],
        };
        let decoded = decode("part.scale", 1, encode(&scale)).unwrap();
        assert!(matches!(
            decoded,
            FeatureType::BodyScale {
                source,
                factor,
                factor_expr: Some(expression),
                center,
            } if source == "source"
                && factor == 2.5
                && expression == "scale_factor"
                && center == [1.0, 2.0, 3.0]
        ));
    }

    #[test]
    fn feature_pattern_has_an_independent_stable_v1_payload() {
        let feature = FeatureType::FeaturePattern {
            target: "plate".into(),
            source_feature: "hole_2".into(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::Datum("pattern_axis".into()),
                total_angle_deg: 270.0,
                total_angle_expr: Some("pattern_angle".into()),
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        };
        let decoded = decode("part.feature_pattern", 1, encode(&feature)).unwrap();
        assert!(matches!(
            decoded,
            FeatureType::FeaturePattern {
                target,
                source_feature,
                kind: FeaturePatternKind::Circular {
                    axis: AxisBase::Datum(axis),
                    total_angle_deg,
                    total_angle_expr: Some(expression),
                    count: 4,
                },
                compute_mode: FeaturePatternComputeMode::Identical,
                extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
            } if target == "plate"
                && source_feature == "hole_2"
                && axis == "pattern_axis"
                && total_angle_deg == 270.0
                && expression == "pattern_angle"
        ));
    }

    #[test]
    fn draft_has_an_independent_stable_v1_payload() {
        let neutral = FaceRef {
            centroid: [0.0, 0.0, 12.0],
            normal: [0.0, 0.0, 1.0],
            topology: None,
        };
        let selected = FaceRef {
            centroid: [5.0, 0.0, 6.0],
            normal: [1.0, 0.0, 0.0],
            topology: None,
        };
        let feature = FeatureType::Draft {
            target: "body".into(),
            faces: vec![selected.clone()],
            neutral: DraftNeutral::Face(neutral.clone()),
            angle_deg: -4.5,
            angle_expr: Some("-taper".into()),
            flip_pull: true,
        };
        let decoded = decode("part.draft", 1, encode(&feature)).unwrap();
        assert!(matches!(
            decoded,
            FeatureType::Draft {
                target,
                faces,
                neutral: DraftNeutral::Face(decoded_neutral),
                angle_deg,
                angle_expr: Some(expression),
                flip_pull: true,
            } if target == "body"
                && faces == vec![selected]
                && decoded_neutral == neutral
                && angle_deg == -4.5
                && expression == "-taper"
        ));
    }

    #[test]
    fn geometry_datums_require_v2_while_legacy_datums_remain_v1() {
        let legacy = FeatureType::DatumPoint {
            def: DatumPointDef::Coords { p: [1.0, 2.0, 3.0] },
        };
        assert!(matches!(
            decode("datum.point", 1, encode(&legacy)).unwrap(),
            FeatureType::DatumPoint {
                def: DatumPointDef::Coords { p: [1.0, 2.0, 3.0] }
            }
        ));

        let geometry = FeatureType::DatumPlane {
            def: DatumPlaneDef::PlanarFace {
                face: FaceRef {
                    centroid: [4.0, 5.0, 6.0],
                    normal: [0.0, 1.0, 0.0],
                    topology: None,
                },
            },
        };
        let v1_error = decode("datum.plane", 1, encode(&geometry))
            .expect_err("geometry datum must not decode as schema v1");
        assert!(v1_error.contains("requires payload schema 2"));
        assert!(matches!(
            decode_with_decoder(
                "datum.plane",
                2,
                crate::document::FeaturePayloadDecoder::NumericFieldsV2,
                encode_for_schema(&geometry, 2),
            )
            .unwrap(),
            FeatureType::DatumPlane {
                def: DatumPlaneDef::PlanarFace { face }
            } if face.centroid == [4.0, 5.0, 6.0]
                && face.normal == [0.0, 1.0, 0.0]
        ));
    }

    #[test]
    fn phase5_direct_edit_payloads_have_stable_v1_round_trips() {
        let face = FaceRef {
            centroid: [1.0, 2.0, 3.0],
            normal: [0.0, 0.0, 1.0],
            topology: None,
        };
        let offset = FeatureType::FaceOffset {
            target: "imported".into(),
            face: face.clone(),
            distance: -2.5,
            distance_expr: Some("-wall".into()),
        };
        assert!(matches!(
            decode("direct.face_offset", 1, encode(&offset)).unwrap(),
            FeatureType::FaceOffset {
                target,
                distance,
                distance_expr: Some(expression),
                ..
            } if target == "imported" && distance == -2.5 && expression == "-wall"
        ));
        let moved = FeatureType::FaceMove {
            target: "imported".into(),
            face: face.clone(),
            translation: [0.0, 0.0, 4.0],
        };
        assert!(matches!(
            decode("direct.face_move", 1, encode(&moved)).unwrap(),
            FeatureType::FaceMove { translation, .. } if translation == [0.0, 0.0, 4.0]
        ));
        let deleted = FeatureType::FaceDelete {
            target: "imported".into(),
            face: face.clone(),
        };
        assert!(matches!(
            decode("direct.face_delete", 1, encode(&deleted)).unwrap(),
            FeatureType::FaceDelete { target, .. } if target == "imported"
        ));
        let thickened = FeatureType::FaceThicken {
            target: "imported".into(),
            face,
            thickness: 1.25,
            thickness_expr: Some("sheet".into()),
            reverse: true,
        };
        assert!(matches!(
            decode("direct.face_thicken", 1, encode(&thickened)).unwrap(),
            FeatureType::FaceThicken {
                thickness,
                thickness_expr: Some(expression),
                reverse: true,
                ..
            } if thickness == 1.25 && expression == "sheet"
        ));
    }

    #[test]
    fn payload_decoder_dispatch_is_schema_aware() {
        let feature = FeatureType::Box {
            w: 1.0,
            h: 2.0,
            d: 3.0,
        };
        assert!(matches!(
            decode("part.box", 1, encode(&feature)),
            Ok(FeatureType::Box { w, h, d }) if w == 1.0 && h == 2.0 && d == 3.0
        ));

        for unsupported in [0, 2] {
            let error = decode("part.box", unsupported, encode(&feature))
                .expect_err("unregistered payload schema must fail");
            assert!(error.contains(&format!("unsupported payload schema {unsupported}")));
            assert!(error.contains("supported schemas: [1]"));
        }
    }
}
