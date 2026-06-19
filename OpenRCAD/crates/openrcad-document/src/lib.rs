#![forbid(unsafe_code)]
//! Parametric document history for sketches and feature recompute.
//!
//! The first document layer is deliberately small: sketches own 2D intent,
//! features reference sketches or prior feature results, and recompute rebuilds
//! solids in insertion order. That gives applications a Fusion/FreeCAD-like
//! modeling spine without pulling in UI or solver choices.

pub mod zcad_format;

use core::fmt;

use openrcad_algo::{boolean_checked, chamfer, fillet, BlendError, BooleanError, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_sketch::{EntityId, Profile, Sketch, SketchError, SketchPlane};
use openrcad_topo::{HealthError, HealthReport, Solid};
use serde::{Deserialize, Serialize};

pub use zcad_format::{
    load_zcad, read_zcad, save_zcad, write_zcad, CachedMesh, LoadedZcad, ZcadDocument, ZcadError,
    ZcadMetadata,
};

/// Stable handle to a sketch in a [`Document`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SketchId(pub usize);

/// Stable handle to a feature in a [`Document`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FeatureId(pub usize);

/// How a generated solid joins the document.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Operation {
    /// Keep the generated solid as a new independent body.
    NewBody,
    /// Fuse the generated solid into an earlier feature result.
    Fuse(FeatureId),
    /// Cut the generated solid from an earlier feature result.
    Cut(FeatureId),
    /// Keep only the common volume with an earlier feature result.
    Common(FeatureId),
}

/// A feature stored in parametric history.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Feature {
    name: String,
    kind: FeatureKind,
    result: Option<Solid>,
}

impl Feature {
    /// Feature name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Feature definition.
    pub fn kind(&self) -> &FeatureKind {
        &self.kind
    }

    /// Cached recompute result.
    pub fn result(&self) -> Option<&Solid> {
        self.result.as_ref()
    }
}

/// Supported feature definitions.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FeatureKind {
    /// Extrude a closed sketch profile by depth.
    Extrude {
        /// Sketch containing the profile.
        sketch: SketchId,
        /// Entity to use as the closed profile.
        entity: EntityId,
        /// Extrusion depth along the sketch plane normal.
        depth: f64,
        /// New-body or boolean composition operation.
        operation: Operation,
    },
    /// Boolean operation between two earlier feature results.
    Boolean {
        /// Object solid.
        left: FeatureId,
        /// Tool solid.
        right: FeatureId,
        /// Boolean operation.
        op: BooleanOp,
    },
    /// Fillet all supported edges of an earlier feature result.
    Fillet {
        /// Input feature.
        input: FeatureId,
        /// Fillet radius.
        radius: f64,
    },
    /// Chamfer all supported edges of an earlier feature result.
    Chamfer {
        /// Input feature.
        input: FeatureId,
        /// Chamfer distance.
        distance: f64,
    },
}

/// Error returned by document editing or recompute.
#[derive(Debug)]
pub enum DocumentError {
    /// Sketch handle is invalid.
    MissingSketch(SketchId),
    /// Feature handle is invalid.
    MissingFeature(FeatureId),
    /// Feature has not been recomputed yet.
    MissingResult(FeatureId),
    /// Sketch validation failed.
    Sketch(SketchError),
    /// A numeric input was invalid.
    NonPositiveDimension { value: f64 },
    /// Blend feature failed.
    Blend(BlendError),
    /// Boolean feature failed.
    Boolean(BooleanError),
    /// Recompute produced unhealthy topology.
    UnhealthyResult {
        /// Feature that produced the result.
        feature: FeatureId,
        /// Health errors reported by topology.
        errors: Vec<HealthError>,
    },
    /// Recompute produced a structurally valid but open/non-watertight solid.
    NonWatertightResult {
        /// Feature that produced the result.
        feature: FeatureId,
        /// Health report for the result.
        report: HealthReport,
    },
}

impl fmt::Display for DocumentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingSketch(id) => write!(f, "missing sketch {:?}", id),
            Self::MissingFeature(id) => write!(f, "missing feature {:?}", id),
            Self::MissingResult(id) => write!(f, "feature {:?} has no recomputed result", id),
            Self::Sketch(err) => write!(f, "{err}"),
            Self::NonPositiveDimension { value } => {
                write!(f, "dimension must be positive, got {value}")
            }
            Self::Blend(err) => write!(f, "{err}"),
            Self::Boolean(err) => write!(f, "{err}"),
            Self::UnhealthyResult { feature, errors } => {
                write!(
                    f,
                    "feature {:?} produced unhealthy topology: {errors:?}",
                    feature
                )
            }
            Self::NonWatertightResult { feature, report } => {
                write!(
                    f,
                    "feature {:?} produced a non-watertight solid: {report:?}",
                    feature
                )
            }
        }
    }
}

impl std::error::Error for DocumentError {}

impl From<SketchError> for DocumentError {
    fn from(value: SketchError) -> Self {
        Self::Sketch(value)
    }
}

impl From<BlendError> for DocumentError {
    fn from(value: BlendError) -> Self {
        Self::Blend(value)
    }
}

impl From<BooleanError> for DocumentError {
    fn from(value: BooleanError) -> Self {
        Self::Boolean(value)
    }
}

/// A parametric CAD document containing sketches and feature history.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
    sketches: Vec<Sketch>,
    features: Vec<Feature>,
}

impl Document {
    /// Create an empty document.
    pub fn new() -> Self {
        Self::default()
    }

    /// All sketches.
    pub fn sketches(&self) -> &[Sketch] {
        &self.sketches
    }

    /// All features in recompute order.
    pub fn features(&self) -> &[Feature] {
        &self.features
    }

    /// Return a copy of the parametric recipe without cached solid results.
    ///
    /// This is what lightweight `.zcad` files store: sketches and feature
    /// history are authoritative, and geometry is regenerated on open.
    pub fn without_cached_results(&self) -> Self {
        let mut out = self.clone();
        for feature in &mut out.features {
            feature.result = None;
        }
        out
    }

    /// Add a new sketch and return its handle.
    pub fn add_sketch(&mut self, name: impl Into<String>, plane: SketchPlane) -> SketchId {
        let id = SketchId(self.sketches.len());
        self.sketches.push(Sketch::new(name, plane));
        id
    }

    /// Get an immutable sketch.
    pub fn sketch(&self, id: SketchId) -> Result<&Sketch, DocumentError> {
        self.sketches
            .get(id.0)
            .ok_or(DocumentError::MissingSketch(id))
    }

    /// Get a mutable sketch.
    pub fn sketch_mut(&mut self, id: SketchId) -> Result<&mut Sketch, DocumentError> {
        self.sketches
            .get_mut(id.0)
            .ok_or(DocumentError::MissingSketch(id))
    }

    /// Add an extrude feature and recompute the document.
    pub fn extrude(
        &mut self,
        name: impl Into<String>,
        sketch: SketchId,
        entity: EntityId,
        depth: f64,
        operation: Operation,
    ) -> Result<FeatureId, DocumentError> {
        ensure_positive(depth)?;
        self.sketch(sketch)?.entity(entity)?;
        self.push_feature(
            name,
            FeatureKind::Extrude {
                sketch,
                entity,
                depth,
                operation,
            },
        )
    }

    /// Add a boolean feature and recompute the document.
    pub fn boolean(
        &mut self,
        name: impl Into<String>,
        left: FeatureId,
        right: FeatureId,
        op: BooleanOp,
    ) -> Result<FeatureId, DocumentError> {
        self.ensure_feature(left)?;
        self.ensure_feature(right)?;
        self.push_feature(name, FeatureKind::Boolean { left, right, op })
    }

    /// Add a fillet feature and recompute the document.
    pub fn fillet(
        &mut self,
        name: impl Into<String>,
        input: FeatureId,
        radius: f64,
    ) -> Result<FeatureId, DocumentError> {
        ensure_positive(radius)?;
        self.ensure_feature(input)?;
        self.push_feature(name, FeatureKind::Fillet { input, radius })
    }

    /// Add a chamfer feature and recompute the document.
    pub fn chamfer(
        &mut self,
        name: impl Into<String>,
        input: FeatureId,
        distance: f64,
    ) -> Result<FeatureId, DocumentError> {
        ensure_positive(distance)?;
        self.ensure_feature(input)?;
        self.push_feature(name, FeatureKind::Chamfer { input, distance })
    }

    /// Rebuild every feature from the current sketch and feature definitions.
    ///
    /// Recompute preserves the feature result even if downstream topology health
    /// diagnostics find issues. Use [`feature_health`](Self::feature_health) when
    /// an application wants to surface warnings or block export.
    pub fn recompute(&mut self) -> Result<(), DocumentError> {
        for feature in &mut self.features {
            feature.result = None;
        }

        for index in 0..self.features.len() {
            let id = FeatureId(index);
            let kind = self.features[index].kind.clone();
            let solid = self.recompute_feature(id, &kind)?;
            ensure_feature_result_healthy(id, &solid)?;
            self.features[index].result = Some(solid);
        }
        Ok(())
    }

    /// Get a recomputed feature result.
    pub fn solid(&self, id: FeatureId) -> Result<&Solid, DocumentError> {
        self.features
            .get(id.0)
            .ok_or(DocumentError::MissingFeature(id))?
            .result
            .as_ref()
            .ok_or(DocumentError::MissingResult(id))
    }

    /// Run topology health diagnostics on a recomputed feature result.
    pub fn feature_health(&self, id: FeatureId) -> Result<HealthReport, DocumentError> {
        Ok(self.solid(id)?.health_report())
    }

    fn push_feature(
        &mut self,
        name: impl Into<String>,
        kind: FeatureKind,
    ) -> Result<FeatureId, DocumentError> {
        let id = FeatureId(self.features.len());
        self.features.push(Feature {
            name: name.into(),
            kind,
            result: None,
        });
        if let Err(err) = self.recompute() {
            self.features.pop();
            let _ = self.recompute();
            return Err(err);
        }
        Ok(id)
    }

    fn ensure_feature(&self, id: FeatureId) -> Result<(), DocumentError> {
        self.features
            .get(id.0)
            .map(|_| ())
            .ok_or(DocumentError::MissingFeature(id))
    }

    fn recompute_feature(&self, id: FeatureId, kind: &FeatureKind) -> Result<Solid, DocumentError> {
        match *kind {
            FeatureKind::Extrude {
                sketch,
                entity,
                depth,
                operation,
            } => {
                let sketch_ref = self.sketch(sketch)?;
                let profile = sketch_ref.profile(entity)?;
                let tool = extrude_profile(sketch_ref.plane(), &profile, depth);
                apply_operation(self, id, tool, operation)
            }
            FeatureKind::Boolean { left, right, op } => {
                Ok(boolean_checked(self.solid(left)?, self.solid(right)?, op)?)
            }
            FeatureKind::Fillet { input, radius } => Ok(fillet(self.solid(input)?, radius)?),
            FeatureKind::Chamfer { input, distance } => Ok(chamfer(self.solid(input)?, distance)?),
        }
    }
}

fn apply_operation(
    document: &Document,
    self_id: FeatureId,
    tool: Solid,
    operation: Operation,
) -> Result<Solid, DocumentError> {
    match operation {
        Operation::NewBody => Ok(tool),
        Operation::Fuse(target) => Ok(boolean_checked(
            document.solid_before(target, self_id)?,
            &tool,
            BooleanOp::Fuse,
        )?),
        Operation::Cut(target) => Ok(boolean_checked(
            document.solid_before(target, self_id)?,
            &tool,
            BooleanOp::Cut,
        )?),
        Operation::Common(target) => Ok(boolean_checked(
            document.solid_before(target, self_id)?,
            &tool,
            BooleanOp::Common,
        )?),
    }
}

trait SolidBefore {
    fn solid_before(&self, target: FeatureId, current: FeatureId) -> Result<&Solid, DocumentError>;
}

impl SolidBefore for Document {
    fn solid_before(&self, target: FeatureId, current: FeatureId) -> Result<&Solid, DocumentError> {
        if target.0 >= current.0 {
            return Err(DocumentError::MissingResult(target));
        }
        self.solid(target)
    }
}

fn extrude_profile(plane: SketchPlane, profile: &Profile, depth: f64) -> Solid {
    match profile {
        Profile::Rectangle {
            corner,
            width,
            height,
        } => match plane {
            SketchPlane::XY => make_box(
                &Pnt::new(corner.x(), corner.y(), 0.0),
                *width,
                *height,
                depth,
            ),
            SketchPlane::XZ => make_box(
                &Pnt::new(corner.x(), 0.0, corner.y()),
                *width,
                depth,
                *height,
            ),
            SketchPlane::YZ => make_box(
                &Pnt::new(0.0, corner.x(), corner.y()),
                depth,
                *width,
                *height,
            ),
        },
        Profile::Circle { center, radius } => {
            let axis = match plane {
                SketchPlane::XY => {
                    Ax2::new_axes(Pnt::new(center.x(), center.y(), 0.0), Dir::dz(), Dir::dx())
                }
                SketchPlane::XZ => {
                    Ax2::new_axes(Pnt::new(center.x(), 0.0, center.y()), Dir::dy(), Dir::dx())
                }
                SketchPlane::YZ => {
                    Ax2::new_axes(Pnt::new(0.0, center.x(), center.y()), Dir::dx(), Dir::dy())
                }
            };
            make_cylinder(&axis, *radius, depth)
        }
    }
}

fn ensure_positive(value: f64) -> Result<(), DocumentError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(DocumentError::NonPositiveDimension { value })
    }
}

fn ensure_feature_result_healthy(id: FeatureId, solid: &Solid) -> Result<(), DocumentError> {
    let report = solid.health_report();
    if !report.is_healthy() {
        return Err(DocumentError::UnhealthyResult {
            feature: id,
            errors: report.errors,
        });
    }
    if !solid.is_watertight() {
        return Err(DocumentError::NonWatertightResult {
            feature: id,
            report,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_sketch_extrudes_to_box() {
        let mut doc = Document::new();
        let sketch = doc.add_sketch("base", SketchPlane::XY);
        let rect = doc
            .sketch_mut(sketch)
            .unwrap()
            .rectangle(0.0, 0.0, 10.0, 20.0)
            .unwrap();

        let base = doc
            .extrude("base", sketch, rect, 30.0, Operation::NewBody)
            .unwrap();
        let solid = doc.solid(base).unwrap();

        assert_eq!(solid.vertex_count(), 8);
        assert_eq!(solid.edge_count(), 12);
        assert_eq!(solid.face_count(), 6);
    }

    #[test]
    fn circle_sketch_extrudes_to_cylinder() {
        let mut doc = Document::new();
        let sketch = doc.add_sketch("pin", SketchPlane::XY);
        let circle = doc
            .sketch_mut(sketch)
            .unwrap()
            .circle(5.0, 5.0, 2.0)
            .unwrap();

        let pin = doc
            .extrude("pin", sketch, circle, 10.0, Operation::NewBody)
            .unwrap();
        let solid = doc.solid(pin).unwrap();

        assert_eq!(solid.vertex_count(), 6);
        assert_eq!(solid.edge_count(), 9);
        assert_eq!(solid.face_count(), 5);
    }

    #[test]
    fn cut_operation_caches_watertight_result() {
        let mut doc = Document::new();
        let base_sketch = doc.add_sketch("base", SketchPlane::XY);
        let base_rect = doc
            .sketch_mut(base_sketch)
            .unwrap()
            .rectangle(0.0, 0.0, 10.0, 10.0)
            .unwrap();
        let base = doc
            .extrude("base", base_sketch, base_rect, 10.0, Operation::NewBody)
            .unwrap();

        let tool_sketch = doc.add_sketch("tool", SketchPlane::XY);
        let tool_rect = doc
            .sketch_mut(tool_sketch)
            .unwrap()
            .rectangle(5.0, 0.0, 10.0, 10.0)
            .unwrap();
        let cut = doc
            .extrude("cut", tool_sketch, tool_rect, 10.0, Operation::Cut(base))
            .unwrap();
        let solid = doc.solid(cut).unwrap();

        let (lo, hi) = solid.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5);
        assert!((hi.x() - 5.0).abs() < 1e-5);
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
    }

    #[test]
    fn fillet_and_chamfer_features_wrap_existing_algorithms() {
        let mut doc = Document::new();
        let sketch = doc.add_sketch("base", SketchPlane::XY);
        let rect = doc
            .sketch_mut(sketch)
            .unwrap()
            .rectangle(0.0, 0.0, 10.0, 10.0)
            .unwrap();
        let base = doc
            .extrude("base", sketch, rect, 10.0, Operation::NewBody)
            .unwrap();
        let rounded = doc.fillet("round", base, 1.0).unwrap();
        let chamfered = doc.chamfer("bevel", base, 1.0).unwrap();

        assert!(doc.solid(rounded).unwrap().face_count() > doc.solid(base).unwrap().face_count());
        assert!(doc.solid(chamfered).unwrap().face_count() > doc.solid(base).unwrap().face_count());
    }

    #[test]
    fn failed_boolean_feature_rolls_back_history() {
        let mut doc = Document::new();

        let a_sketch = doc.add_sketch("a", SketchPlane::XY);
        let a_rect = doc
            .sketch_mut(a_sketch)
            .unwrap()
            .rectangle(0.0, 0.0, 10.0, 10.0)
            .unwrap();
        let a = doc
            .extrude("a", a_sketch, a_rect, 10.0, Operation::NewBody)
            .unwrap();

        let b_sketch = doc.add_sketch("b", SketchPlane::XY);
        let b_rect = doc
            .sketch_mut(b_sketch)
            .unwrap()
            .rectangle(5.0, 5.0, 10.0, 10.0)
            .unwrap();
        let b = doc
            .extrude("b", b_sketch, b_rect, 10.0, Operation::NewBody)
            .unwrap();

        let before = doc.features().len();
        let err = doc
            .boolean("bad corner fuse", a, b, BooleanOp::Fuse)
            .expect_err("known partial-imprint union should be rejected");

        assert!(
            matches!(
                err,
                DocumentError::Boolean(BooleanError::InvalidOutput { .. })
                    | DocumentError::Boolean(BooleanError::NonWatertightOutput { .. })
                    | DocumentError::UnhealthyResult { .. }
                    | DocumentError::NonWatertightResult { .. }
            ),
            "unexpected document error: {err:?}"
        );
        assert_eq!(doc.features().len(), before);
        assert!(doc.solid(a).is_ok());
        assert!(doc.solid(b).is_ok());
    }
}
