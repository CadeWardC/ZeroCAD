//! Asset-bound replay cases. V1 primitive identities remain unchanged.
use super::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Operand {
    Primitive {
        operand: BooleanOperandV1,
    },
    Step {
        sha256: String,
        data: String,
    },
    Document {
        sha256: String,
        data: Vec<u8>,
        body_id: String,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BooleanCaseV2 {
    pub operation: BooleanVerificationOp,
    pub object: Operand,
    pub tool: Operand,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryObservation {
    pub body_count: usize,
    pub volume: f64,
    pub surface_area: f64,
    pub centroid: [f64; 3],
}

impl GeometryObservation {
    /// Geometry tolerances are explicit; counts are always exact. This catches
    /// healthy but incorrect results that topology-count assertions miss.
    pub fn compare(&self, expected: &Self, relative: f64, absolute: f64) -> Result<(), String> {
        if !relative.is_finite() || !absolute.is_finite() || relative < 0. || absolute < 0. {
            return Err("Invalid observation tolerance".into());
        }
        if self.body_count != expected.body_count {
            return Err("Body count differs".into());
        }
        for (name, a, b) in [
            ("volume", self.volume, expected.volume),
            ("area", self.surface_area, expected.surface_area),
            ("centroid x", self.centroid[0], expected.centroid[0]),
            ("centroid y", self.centroid[1], expected.centroid[1]),
            ("centroid z", self.centroid[2], expected.centroid[2]),
        ] {
            if !a.is_finite() || !b.is_finite() || (a - b).abs() > absolute + relative * b.abs() {
                return Err(format!("{name}: observed {a}, expected {b}"));
            }
        }
        Ok(())
    }
}

impl Operand {
    fn canonical_bytes(&self) -> Result<Vec<u8>, String> {
        let mut bytes = Vec::new();
        match self {
            Self::Primitive { operand } => {
                validate_operand(operand).map_err(|e| e.to_string())?;
                bytes.push(0);
                encode_operand(&mut bytes, operand);
            }
            Self::Step { sha256, data } => {
                bytes.push(1);
                bytes.extend(checked_digest(sha256, data.as_bytes())?);
            }
            Self::Document {
                sha256,
                data,
                body_id,
            } => {
                bytes.push(2);
                bytes.extend(checked_digest(sha256, data)?);
                bytes.extend((body_id.len() as u64).to_le_bytes());
                bytes.extend(body_id.as_bytes());
            }
        }
        Ok(bytes)
    }

    fn build(&self) -> Result<Solid, String> {
        self.canonical_bytes()?;
        match self {
            Self::Primitive { operand } => build_operand(operand, 1.).map_err(|e| e.to_string()),
            Self::Step { data, .. } => openrcad::exchange::read_step_str_operation(data)
                .map(|result| result.value)
                .map_err(|e| e.to_string()),
            Self::Document { data, body_id, .. } => {
                let loaded = crate::read_document_from_slice(data, &crate::LoadOptions::default())
                    .map_err(|e| e.to_string())?;
                let bodies = loaded
                    .document
                    .evaluated_kernel_bodies(&Default::default())?;
                let (_, mut parts) = bodies
                    .into_iter()
                    .find(|(id, _)| id == body_id)
                    .ok_or_else(|| format!("Missing recipe body {body_id}"))?;
                if parts.len() != 1 {
                    return Err("Replay operand must have exactly one connected solid".into());
                }
                Ok(parts.remove(0))
            }
        }
    }
}

fn checked_digest(expected: &str, data: &[u8]) -> Result<Vec<u8>, String> {
    if data.len() > 32 * 1024 * 1024 {
        return Err("Replay asset exceeds 32 MiB".into());
    }
    let digest = Sha256::digest(data);
    let actual: String = digest.iter().map(|byte| format!("{byte:02x}")).collect();
    if expected != actual {
        return Err("Replay asset SHA-256 mismatch".into());
    }
    Ok(digest.to_vec())
}

pub fn asset_sha256(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

impl BooleanCaseV2 {
    pub fn identity(&self) -> Result<BooleanCaseIdentity, String> {
        let mut bytes = b"ZBCV2\0\0\0".to_vec();
        bytes.push(self.operation.tag());
        bytes.extend(self.object.canonical_bytes()?);
        bytes.extend(self.tool.canonical_bytes()?);
        Ok(BooleanCaseIdentity(Sha256::digest(bytes).into()))
    }

    pub fn replay(&self) -> Result<GeometryObservation, String> {
        self.identity()?;
        let object = self.object.build()?;
        let tool = self.tool.build()?;
        let result = boolean_bodies_operation_with_classes_policy_and_cancel(
            &object,
            &tool,
            self.operation.kernel(),
            None,
            None,
            &TolerancePolicy::STANDARD,
            &openrcad::foundation::NeverCancelled,
        )
        .map_err(|e| e.to_string())?;
        observe_bodies(result.value.bodies)
    }
}

pub(super) fn observe_bodies(bodies: Vec<Solid>) -> Result<GeometryObservation, String> {
    let mut observation = GeometryObservation {
        body_count: bodies.len(),
        volume: 0.,
        surface_area: 0.,
        centroid: [0.; 3],
    };
    if bodies.is_empty() {
        return Ok(observation);
    }
    for solid in bodies {
        let mesh = openrcad::mesh::tessellate_checked_with_policy(
            &solid,
            0.01,
            0.1,
            &TolerancePolicy::STANDARD,
        )
        .map_err(|e| e.to_string())?;
        let properties = openrcad::mesh::mass_properties(&mesh).ok_or("Invalid mass properties")?;
        observation.volume += properties.volume;
        observation.surface_area += properties.surface_area;
        for (sum, coordinate) in observation.centroid.iter_mut().zip(properties.centroid) {
            *sum += coordinate * properties.volume;
        }
    }
    if !observation.volume.is_finite() || observation.volume <= 0. {
        return Err("Nonpositive or nonfinite result volume".into());
    }
    for coordinate in &mut observation.centroid {
        *coordinate /= observation.volume;
    }
    Ok(observation)
}
