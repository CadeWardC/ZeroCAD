#![forbid(unsafe_code)]
//! Lightweight 2D sketches and closed profiles for parametric modeling.
//!
//! This crate intentionally stores sketch intent, not solved constraints. It is
//! the first stable layer for document history: rectangles and circles become
//! closed profiles today, while lines are preserved for future constraints and
//! wire/profile builders.

use core::fmt;

use openrcad_foundation::Pnt2d;
use serde::{Deserialize, Serialize};

/// Stable handle to a sketch entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EntityId(pub usize);

/// Built-in sketch planes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SketchPlane {
    /// XY plane, extruding along +Z.
    XY,
    /// XZ plane, extruding along +Y.
    XZ,
    /// YZ plane, extruding along +X.
    YZ,
}

/// A sketch entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SketchEntity {
    /// Axis-aligned rectangle in sketch coordinates.
    Rectangle {
        /// Lower-left corner in sketch coordinates.
        corner: Pnt2d,
        /// Width along sketch X.
        width: f64,
        /// Height along sketch Y.
        height: f64,
    },
    /// Circle in sketch coordinates.
    Circle {
        /// Center point in sketch coordinates.
        center: Pnt2d,
        /// Radius.
        radius: f64,
    },
    /// Construction or profile edge for future profile solving.
    Line {
        /// Start point.
        start: Pnt2d,
        /// End point.
        end: Pnt2d,
    },
}

/// Closed profiles that can currently drive solid features.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Profile {
    /// Rectangular profile.
    Rectangle {
        /// Lower-left corner in sketch coordinates.
        corner: Pnt2d,
        /// Width along sketch X.
        width: f64,
        /// Height along sketch Y.
        height: f64,
    },
    /// Circular profile.
    Circle {
        /// Center point in sketch coordinates.
        center: Pnt2d,
        /// Radius.
        radius: f64,
    },
}

/// A validation error for sketch input.
#[derive(Clone, Debug, PartialEq)]
pub enum SketchError {
    /// Width, height, radius, or line length must be positive.
    NonPositiveDimension { value: f64 },
    /// The entity handle does not exist in this sketch.
    MissingEntity(EntityId),
    /// The entity is not a closed profile supported by this release.
    NotAProfile(EntityId),
}

impl fmt::Display for SketchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveDimension { value } => {
                write!(f, "dimension must be positive, got {value}")
            }
            Self::MissingEntity(id) => write!(f, "missing sketch entity {:?}", id),
            Self::NotAProfile(id) => write!(f, "sketch entity {:?} is not a closed profile", id),
        }
    }
}

impl std::error::Error for SketchError {}

/// A named 2D sketch on a principal plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Sketch {
    name: String,
    plane: SketchPlane,
    entities: Vec<SketchEntity>,
}

impl Sketch {
    /// Create an empty sketch.
    pub fn new(name: impl Into<String>, plane: SketchPlane) -> Self {
        Self {
            name: name.into(),
            plane,
            entities: Vec::new(),
        }
    }

    /// Sketch name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Sketch plane.
    pub fn plane(&self) -> SketchPlane {
        self.plane
    }

    /// All entities in insertion order.
    pub fn entities(&self) -> &[SketchEntity] {
        &self.entities
    }

    /// Add an axis-aligned rectangle.
    pub fn rectangle(
        &mut self,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    ) -> Result<EntityId, SketchError> {
        ensure_positive(width)?;
        ensure_positive(height)?;
        self.push(SketchEntity::Rectangle {
            corner: Pnt2d::new(x, y),
            width,
            height,
        })
    }

    /// Add a circle.
    pub fn circle(&mut self, x: f64, y: f64, radius: f64) -> Result<EntityId, SketchError> {
        ensure_positive(radius)?;
        self.push(SketchEntity::Circle {
            center: Pnt2d::new(x, y),
            radius,
        })
    }

    /// Add a line segment. Lines are stored but not yet profile-extrudable.
    pub fn line(&mut self, x0: f64, y0: f64, x1: f64, y1: f64) -> Result<EntityId, SketchError> {
        let start = Pnt2d::new(x0, y0);
        let end = Pnt2d::new(x1, y1);
        ensure_positive(start.distance(&end))?;
        self.push(SketchEntity::Line { start, end })
    }

    /// Get an entity by handle.
    pub fn entity(&self, id: EntityId) -> Result<&SketchEntity, SketchError> {
        self.entities
            .get(id.0)
            .ok_or(SketchError::MissingEntity(id))
    }

    /// Extract a closed profile from an entity.
    pub fn profile(&self, id: EntityId) -> Result<Profile, SketchError> {
        match self.entity(id)? {
            SketchEntity::Rectangle {
                corner,
                width,
                height,
            } => Ok(Profile::Rectangle {
                corner: *corner,
                width: *width,
                height: *height,
            }),
            SketchEntity::Circle { center, radius } => Ok(Profile::Circle {
                center: *center,
                radius: *radius,
            }),
            SketchEntity::Line { .. } => Err(SketchError::NotAProfile(id)),
        }
    }

    fn push(&mut self, entity: SketchEntity) -> Result<EntityId, SketchError> {
        let id = EntityId(self.entities.len());
        self.entities.push(entity);
        Ok(id)
    }
}

fn ensure_positive(value: f64) -> Result<(), SketchError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(SketchError::NonPositiveDimension { value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_extracts_profile() {
        let mut sketch = Sketch::new("base", SketchPlane::XY);
        let rect = sketch.rectangle(1.0, 2.0, 10.0, 20.0).unwrap();

        assert_eq!(
            sketch.profile(rect).unwrap(),
            Profile::Rectangle {
                corner: Pnt2d::new(1.0, 2.0),
                width: 10.0,
                height: 20.0
            }
        );
    }

    #[test]
    fn circle_extracts_profile() {
        let mut sketch = Sketch::new("hole", SketchPlane::XY);
        let circle = sketch.circle(5.0, 6.0, 2.0).unwrap();

        assert_eq!(
            sketch.profile(circle).unwrap(),
            Profile::Circle {
                center: Pnt2d::new(5.0, 6.0),
                radius: 2.0
            }
        );
    }

    #[test]
    fn invalid_dimensions_are_rejected() {
        let mut sketch = Sketch::new("bad", SketchPlane::XY);

        assert!(matches!(
            sketch.rectangle(0.0, 0.0, 0.0, 1.0),
            Err(SketchError::NonPositiveDimension { .. })
        ));
        assert!(matches!(
            sketch.circle(0.0, 0.0, f64::NAN),
            Err(SketchError::NonPositiveDimension { .. })
        ));
    }

    #[test]
    fn line_is_not_a_profile_yet() {
        let mut sketch = Sketch::new("construction", SketchPlane::XY);
        let line = sketch.line(0.0, 0.0, 1.0, 0.0).unwrap();

        assert_eq!(sketch.profile(line), Err(SketchError::NotAProfile(line)));
    }
}
