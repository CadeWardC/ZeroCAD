//! Numeric-field v1 payload DTOs for `.zcad` recipes.
//!
//! These mappings are persistence ABI. Rust variant and field names may change;
//! the numeric slots below may not. A future field-layout change increments the
//! feature kind's payload version and adds an explicit decoder.

use crate::parametric::{
    AxisBase, DatumAxisDef, DatumPlaneDef, DatumPointDef, EdgeRef, ExtrudeMode, FaceRef,
    FeatureType, HoleKind, PatternKind, PlaneBase, Variable,
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
            put(&mut fields, 8, solver);
        }
        FeatureType::Extrude {
            depth,
            region_indices,
            mode,
            target,
            depth_expr,
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
        } => {
            put(&mut fields, 0, target);
            put(&mut fields, 1, position);
            put(&mut fields, 2, direction);
            put(&mut fields, 3, diameter);
            put(&mut fields, 4, diameter_expr);
            put(&mut fields, 5, depth);
            put(&mut fields, 6, kind);
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
    }
    fields
}

pub(crate) fn decode(kind: &str, mut fields: NumericFeatureFields) -> Result<FeatureType, String> {
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
        },
        "part.sweep" => FeatureType::Sweep {
            profile_sketch: take(&mut fields, 0, "profile sketch")?,
            profile_region: take(&mut fields, 1, "profile region")?,
            path_sketch: take(&mut fields, 2, "path sketch")?,
            mode: take::<ExtrudeMode>(&mut fields, 3, "mode")?,
            target: take(&mut fields, 4, "target")?,
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
        },
        "part.pattern" => FeatureType::Pattern {
            source: take(&mut fields, 0, "source")?,
            kind: take::<PatternKind>(&mut fields, 1, "kind")?,
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
        "exchange.step_import" => {
            return Err("STEP imports must use a content-addressed asset payload".into())
        }
        _ => return Err(format!("unknown feature kind '{kind}'")),
    };
    finish(kind, fields)?;
    Ok(feature)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase35_payloads_have_stable_v1_round_trips() {
        let intersect = FeatureType::BodyIntersect {
            target: "target".into(),
            tool: "tool".into(),
            keep_tool: true,
        };
        let decoded = decode("part.intersect", encode(&intersect)).unwrap();
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
        let decoded = decode("part.split", encode(&split)).unwrap();
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
        let decoded = decode("part.scale", encode(&scale)).unwrap();
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
}
