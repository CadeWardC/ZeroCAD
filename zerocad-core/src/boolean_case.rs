//! Canonical, portable boolean verification cases.
//!
//! The binary identity is authoritative. JSON is only a readable transport for
//! promoted regressions and deliberately does not participate in hashing.

use openrcad::algo::{boolean_bodies_operation_with_classes_policy_and_cancel, BooleanOp};
use openrcad::foundation::{Ax1, Pnt, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad::primitives::{
    make_box_operation, make_cone_operation, make_cylinder_operation, make_sphere_operation,
};
use openrcad::topo::Solid;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fmt;

const CASE_MAGIC: &[u8; 8] = b"ZBCV1\0\0\0";
const MAX_ABS_LOG_SCALE: i8 = 9;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanVerificationOp {
    Union,
    Cut,
    Intersect,
}

impl BooleanVerificationOp {
    fn tag(self) -> u8 {
        match self {
            Self::Union => 0,
            Self::Cut => 1,
            Self::Intersect => 2,
        }
    }

    fn kernel(self) -> BooleanOp {
        match self {
            Self::Union => BooleanOp::Fuse,
            Self::Cut => BooleanOp::Cut,
            Self::Intersect => BooleanOp::Common,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanContactClass {
    Disjoint,
    Touching,
    Overlapping,
    Contained,
    Coincident,
}

impl BooleanContactClass {
    fn tag(self) -> u8 {
        match self {
            Self::Disjoint => 0,
            Self::Touching => 1,
            Self::Overlapping => 2,
            Self::Contained => 3,
            Self::Coincident => 4,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum AnalyticPrimitiveV1 {
    Box {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Cone {
        radius1: f64,
        radius2: f64,
        height: f64,
    },
    Sphere {
        radius: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidTransformV1 {
    pub translation: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_radians: f64,
}

impl Default for RigidTransformV1 {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_radians: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanOperandV1 {
    pub primitive: AnalyticPrimitiveV1,
    #[serde(default)]
    pub transform: RigidTransformV1,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanCaseV1 {
    pub operation: BooleanVerificationOp,
    pub object: BooleanOperandV1,
    pub tool: BooleanOperandV1,
    pub contact_class: BooleanContactClass,
    /// Decimal exponent applied uniformly to dimensions and translations.
    pub logarithmic_scale: i8,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct BooleanCaseIdentity([u8; 32]);

impl BooleanCaseIdentity {
    pub fn bytes(self) -> [u8; 32] {
        self.0
    }

    pub fn hex(self) -> String {
        self.0.iter().map(|byte| format!("{byte:02x}")).collect()
    }
}

impl fmt::Debug for BooleanCaseIdentity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("BooleanCaseIdentity")
            .field(&self.hex())
            .finish()
    }
}

impl fmt::Display for BooleanCaseIdentity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.hex())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BooleanCaseError {
    NonFinite { field: &'static str },
    InvalidParameter { field: &'static str },
    InvalidRotationAxis,
    LogScaleOutOfRange(i8),
    PrimitiveBuild(String),
}

impl fmt::Display for BooleanCaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "boolean case field '{field}' is not finite"),
            Self::InvalidParameter { field } => {
                write!(f, "boolean case field '{field}' is outside its domain")
            }
            Self::InvalidRotationAxis => write!(f, "boolean case rotation axis is degenerate"),
            Self::LogScaleOutOfRange(value) => {
                write!(f, "boolean case logarithmic scale {value} is outside ±9")
            }
            Self::PrimitiveBuild(error) => write!(f, "boolean case primitive failed: {error}"),
        }
    }
}

impl std::error::Error for BooleanCaseError {}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanReplayOutcome {
    Resolved,
    Rejected,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BooleanReplaySummary {
    pub identity: String,
    pub outcome: BooleanReplayOutcome,
    pub body_count: usize,
    pub vertex_count: usize,
    pub edge_count: usize,
    pub face_count: usize,
    pub diagnostic: Option<String>,
}

impl BooleanCaseV1 {
    pub fn validate(&self) -> Result<(), BooleanCaseError> {
        if self.logarithmic_scale.abs() > MAX_ABS_LOG_SCALE {
            return Err(BooleanCaseError::LogScaleOutOfRange(self.logarithmic_scale));
        }
        validate_operand(&self.object)?;
        validate_operand(&self.tool)
    }

    /// Fixed-order canonical binary encoding used for stable corpus identity.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, BooleanCaseError> {
        self.validate()?;
        let mut bytes = Vec::with_capacity(192);
        bytes.extend_from_slice(CASE_MAGIC);
        bytes.push(self.operation.tag());
        bytes.push(self.contact_class.tag());
        bytes.push(self.logarithmic_scale as u8);
        encode_operand(&mut bytes, &self.object);
        encode_operand(&mut bytes, &self.tool);
        Ok(bytes)
    }

    pub fn identity(&self) -> Result<BooleanCaseIdentity, BooleanCaseError> {
        let digest = Sha256::digest(self.canonical_bytes()?);
        Ok(BooleanCaseIdentity(digest.into()))
    }

    /// Replay on the canonical OpenRCAD boundary. Rejection is data, never a
    /// panic, so promoted cases can run unchanged in stable Windows CI.
    pub fn replay(&self) -> Result<BooleanReplaySummary, BooleanCaseError> {
        let identity = self.identity()?.hex();
        let scale = 10.0f64.powi(self.logarithmic_scale as i32);
        let object = build_operand(&self.object, scale)?;
        let tool = build_operand(&self.tool, scale)?;
        let result = boolean_bodies_operation_with_classes_policy_and_cancel(
            &object,
            &tool,
            self.operation.kernel(),
            None,
            None,
            &TolerancePolicy::STANDARD,
            &openrcad::foundation::NeverCancelled,
        );
        Ok(match result {
            Ok(result) => {
                let bodies = result.value.bodies;
                BooleanReplaySummary {
                    identity,
                    outcome: BooleanReplayOutcome::Resolved,
                    body_count: bodies.len(),
                    vertex_count: bodies.iter().map(Solid::vertex_count).sum(),
                    edge_count: bodies.iter().map(Solid::edge_count).sum(),
                    face_count: bodies.iter().map(Solid::face_count).sum(),
                    diagnostic: None,
                }
            }
            Err(error) => BooleanReplaySummary {
                identity,
                outcome: BooleanReplayOutcome::Rejected,
                body_count: 0,
                vertex_count: 0,
                edge_count: 0,
                face_count: 0,
                diagnostic: Some(error.to_string()),
            },
        })
    }

    /// Deterministic bounded decoder used by both fuzz targets. Every byte
    /// string maps to finite, ordinary mechanical parameters.
    pub fn from_fuzz_bytes(bytes: &[u8]) -> Self {
        let byte = |index: usize| bytes.get(index).copied().unwrap_or(index as u8);
        let unit = |index: usize| f64::from(byte(index)) / 255.0;
        let primitive = |offset: usize| match byte(offset) % 4 {
            0 => AnalyticPrimitiveV1::Box {
                size: [
                    0.25 + unit(offset + 1) * 40.0,
                    0.25 + unit(offset + 2) * 40.0,
                    0.25 + unit(offset + 3) * 40.0,
                ],
            },
            1 => AnalyticPrimitiveV1::Cylinder {
                radius: 0.25 + unit(offset + 1) * 15.0,
                height: 0.25 + unit(offset + 2) * 40.0,
            },
            2 => AnalyticPrimitiveV1::Cone {
                radius1: 0.25 + unit(offset + 1) * 15.0,
                radius2: unit(offset + 2) * 15.0,
                height: 0.25 + unit(offset + 3) * 40.0,
            },
            _ => AnalyticPrimitiveV1::Sphere {
                radius: 0.25 + unit(offset + 1) * 15.0,
            },
        };
        let transform = |offset: usize| RigidTransformV1 {
            translation: [
                (unit(offset) - 0.5) * 30.0,
                (unit(offset + 1) - 0.5) * 30.0,
                (unit(offset + 2) - 0.5) * 30.0,
            ],
            rotation_axis: [
                0.25 + unit(offset + 3),
                0.25 + unit(offset + 4),
                0.25 + unit(offset + 5),
            ],
            rotation_radians: (unit(offset + 6) - 0.5) * std::f64::consts::TAU,
        };
        Self {
            operation: match byte(0) % 3 {
                0 => BooleanVerificationOp::Union,
                1 => BooleanVerificationOp::Cut,
                _ => BooleanVerificationOp::Intersect,
            },
            object: BooleanOperandV1 {
                primitive: primitive(1),
                transform: transform(8),
            },
            tool: BooleanOperandV1 {
                primitive: primitive(16),
                transform: transform(24),
            },
            contact_class: match byte(31) % 5 {
                0 => BooleanContactClass::Disjoint,
                1 => BooleanContactClass::Touching,
                2 => BooleanContactClass::Overlapping,
                3 => BooleanContactClass::Contained,
                _ => BooleanContactClass::Coincident,
            },
            logarithmic_scale: (byte(32) % 7) as i8 - 3,
        }
    }
}

fn validate_finite(field: &'static str, values: &[f64]) -> Result<(), BooleanCaseError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(BooleanCaseError::NonFinite { field })
    }
}

fn validate_operand(operand: &BooleanOperandV1) -> Result<(), BooleanCaseError> {
    let valid = match &operand.primitive {
        AnalyticPrimitiveV1::Box { size } => {
            validate_finite("box.size", size)?;
            size.iter().all(|value| *value > 0.0)
        }
        AnalyticPrimitiveV1::Cylinder { radius, height } => {
            validate_finite("cylinder", &[*radius, *height])?;
            *radius > 0.0 && *height > 0.0
        }
        AnalyticPrimitiveV1::Cone {
            radius1,
            radius2,
            height,
        } => {
            validate_finite("cone", &[*radius1, *radius2, *height])?;
            *radius1 > 0.0 && *radius2 >= 0.0 && *height > 0.0
        }
        AnalyticPrimitiveV1::Sphere { radius } => {
            validate_finite("sphere.radius", &[*radius])?;
            *radius > 0.0
        }
    };
    if !valid {
        return Err(BooleanCaseError::InvalidParameter { field: "primitive" });
    }
    validate_finite("transform.translation", &operand.transform.translation)?;
    validate_finite("transform.rotation_axis", &operand.transform.rotation_axis)?;
    validate_finite(
        "transform.rotation_radians",
        &[operand.transform.rotation_radians],
    )?;
    let axis_norm2: f64 = operand
        .transform
        .rotation_axis
        .iter()
        .map(|component| component * component)
        .sum();
    if axis_norm2 <= f64::EPSILON {
        return Err(BooleanCaseError::InvalidRotationAxis);
    }
    Ok(())
}

fn canonical_f64(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else {
        value.to_bits()
    }
}

fn push_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&canonical_f64(value).to_le_bytes());
}

fn encode_operand(bytes: &mut Vec<u8>, operand: &BooleanOperandV1) {
    match &operand.primitive {
        AnalyticPrimitiveV1::Box { size } => {
            bytes.push(0);
            size.iter().for_each(|value| push_f64(bytes, *value));
        }
        AnalyticPrimitiveV1::Cylinder { radius, height } => {
            bytes.push(1);
            push_f64(bytes, *radius);
            push_f64(bytes, *height);
        }
        AnalyticPrimitiveV1::Cone {
            radius1,
            radius2,
            height,
        } => {
            bytes.push(2);
            push_f64(bytes, *radius1);
            push_f64(bytes, *radius2);
            push_f64(bytes, *height);
        }
        AnalyticPrimitiveV1::Sphere { radius } => {
            bytes.push(3);
            push_f64(bytes, *radius);
        }
    }
    operand
        .transform
        .translation
        .iter()
        .for_each(|value| push_f64(bytes, *value));
    operand
        .transform
        .rotation_axis
        .iter()
        .for_each(|value| push_f64(bytes, *value));
    push_f64(bytes, operand.transform.rotation_radians);
}

fn build_operand(operand: &BooleanOperandV1, scale: f64) -> Result<Solid, BooleanCaseError> {
    let base = match &operand.primitive {
        AnalyticPrimitiveV1::Box { size } => make_box_operation(
            &Pnt::origin(),
            size[0] * scale,
            size[1] * scale,
            size[2] * scale,
        ),
        AnalyticPrimitiveV1::Cylinder { radius, height } => make_cylinder_operation(
            &openrcad::foundation::Ax2::new(Pnt::origin(), openrcad::foundation::Dir::dz()),
            radius * scale,
            height * scale,
        ),
        AnalyticPrimitiveV1::Cone {
            radius1,
            radius2,
            height,
        } => make_cone_operation(
            &openrcad::foundation::Ax2::new(Pnt::origin(), openrcad::foundation::Dir::dz()),
            radius1 * scale,
            radius2 * scale,
            height * scale,
        ),
        AnalyticPrimitiveV1::Sphere { radius } => {
            make_sphere_operation(&Pnt::origin(), radius * scale)
        }
    }
    .map_err(|error| BooleanCaseError::PrimitiveBuild(error.to_string()))?
    .value;

    let axis = operand.transform.rotation_axis;
    let direction = openrcad::foundation::Dir::new(axis[0], axis[1], axis[2]);
    let rotation = Trsf::rotation(
        &Ax1::new(Pnt::origin(), direction),
        operand.transform.rotation_radians,
    );
    let translation = Trsf::translation(GeomVec::new(
        operand.transform.translation[0] * scale,
        operand.transform.translation[1] * scale,
        operand.transform.translation[2] * scale,
    ));
    Ok(base.transformed(&translation.multiply(&rotation)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_operand(translation: [f64; 3]) -> BooleanOperandV1 {
        BooleanOperandV1 {
            primitive: AnalyticPrimitiveV1::Box {
                size: [10.0, 10.0, 10.0],
            },
            transform: RigidTransformV1 {
                translation,
                ..RigidTransformV1::default()
            },
        }
    }

    #[test]
    fn identity_normalizes_negative_zero_and_is_json_independent() {
        let mut case = BooleanCaseV1 {
            operation: BooleanVerificationOp::Union,
            object: box_operand([-0.0, 0.0, 0.0]),
            tool: box_operand([5.0, 0.0, 0.0]),
            contact_class: BooleanContactClass::Overlapping,
            logarithmic_scale: 0,
        };
        let negative_zero_identity = case.identity().unwrap();
        case.object.transform.translation[0] = 0.0;
        assert_eq!(negative_zero_identity, case.identity().unwrap());
        let json = serde_json::to_string_pretty(&case).unwrap();
        let decoded: BooleanCaseV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.identity().unwrap(), case.identity().unwrap());
    }

    #[test]
    fn overlapping_box_replay_is_watertight_and_deterministic() {
        let case = BooleanCaseV1 {
            operation: BooleanVerificationOp::Union,
            object: box_operand([0.0, 0.0, 0.0]),
            tool: box_operand([5.0, 0.0, 0.0]),
            contact_class: BooleanContactClass::Overlapping,
            logarithmic_scale: 0,
        };
        let first = case.replay().unwrap();
        let second = case.replay().unwrap();
        assert_eq!(first, second);
        assert_eq!(first.outcome, BooleanReplayOutcome::Resolved);
        assert_eq!(first.body_count, 1);
    }

    #[test]
    fn invalid_and_non_finite_cases_never_reach_the_kernel() {
        let mut case = BooleanCaseV1 {
            operation: BooleanVerificationOp::Cut,
            object: box_operand([0.0, 0.0, 0.0]),
            tool: box_operand([2.0, 2.0, 2.0]),
            contact_class: BooleanContactClass::Contained,
            logarithmic_scale: 0,
        };
        case.tool.transform.translation[0] = f64::NAN;
        assert!(matches!(
            case.replay(),
            Err(BooleanCaseError::NonFinite { .. })
        ));
        case.tool.transform.translation[0] = 2.0;
        case.logarithmic_scale = 10;
        assert!(matches!(
            case.identity(),
            Err(BooleanCaseError::LogScaleOutOfRange(10))
        ));
    }
}
