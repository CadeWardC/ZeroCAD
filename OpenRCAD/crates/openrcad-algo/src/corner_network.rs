//! Canonical planning and validation for multi-band blend corners.
//!
//! A corner is planned before any face is trimmed. Incident bands are ordered
//! geometrically around the material-side axis, so selection/traversal order
//! cannot change sector ownership. The planner is deliberately geometry-only:
//! operation code remains responsible for atomically imprinting and committing
//! a verified patch network.

use core::cmp::Ordering;

use openrcad_foundation::{Dir, Pnt, ToleranceContext, Vec as GeomVec};

use crate::band_topology::BandSupportKind;

/// One transition band meeting a blend corner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerIncidentBand {
    /// Contact point on the intended corner envelope.
    pub contact: Pnt,
    /// Requested rolling-ball radius at this incident band.
    pub radius: f64,
    /// Analytic family of the band surface.
    pub support: BandSupportKind,
}

/// Evidence produced before a corner-network candidate may be constructed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerNetworkCertificate {
    pub valence: usize,
    pub maximum_radius_error: f64,
    pub minimum_angular_spacing: f64,
}

/// Exact common rolling-sphere solution for equal offsets of planar supports.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerTangentSphere {
    center: Pnt,
    radius: f64,
    maximum_offset_error: f64,
}

impl CornerTangentSphere {
    /// Intersect the material-inward offsets of three through six planes that
    /// meet at `corner`. The solve uses corner-relative coordinates so small
    /// features remain conditioned at far world origins.
    pub fn new(
        corner: Pnt,
        outward_normals: impl IntoIterator<Item = Dir>,
        radius: f64,
        tolerance: &ToleranceContext,
    ) -> Result<Self, CornerNetworkError> {
        let normals = outward_normals.into_iter().collect::<Vec<_>>();
        if !(CornerNetworkPlan::MIN_VERIFIED_VALENCE..=CornerNetworkPlan::MAX_VERIFIED_VALENCE)
            .contains(&normals.len())
        {
            return Err(CornerNetworkError::UnsupportedValence {
                valence: normals.len(),
            });
        }
        if !radius.is_finite() || radius <= tolerance.arithmetic_floor {
            return Err(CornerNetworkError::InvalidRadius { index: 0, radius });
        }

        // Normal equations for n_i . displacement = -radius. The matrix is
        // positive semidefinite; a near-zero determinant means the offset
        // planes do not constrain one unique rolling-sphere center.
        let mut matrix = [[0.0_f64; 3]; 3];
        let mut right = [0.0_f64; 3];
        for normal in &normals {
            let n = [normal.x(), normal.y(), normal.z()];
            for row in 0..3 {
                right[row] -= n[row] * radius;
                for column in 0..3 {
                    matrix[row][column] += n[row] * n[column];
                }
            }
        }
        let scale = matrix
            .iter()
            .flatten()
            .map(|value| value.abs())
            .fold(0.0, f64::max);
        let determinant = determinant3(matrix);
        let determinant_floor = f64::EPSILON.sqrt() * scale.powi(3).max(f64::MIN_POSITIVE);
        if !determinant.is_finite() || determinant.abs() <= determinant_floor {
            return Err(CornerNetworkError::RankDeficientOffsets);
        }
        let displacement = solve3(matrix, right, determinant);
        if !displacement.iter().all(|value| value.is_finite()) {
            return Err(CornerNetworkError::RankDeficientOffsets);
        }

        let maximum_offset_error = normals.iter().fold(0.0_f64, |maximum, normal| {
            let normal = GeomVec::from_dir(*normal);
            maximum.max(
                (normal.dot(&GeomVec::new(
                    displacement[0],
                    displacement[1],
                    displacement[2],
                )) + radius)
                    .abs(),
            )
        });
        if maximum_offset_error > tolerance.policy.intersection {
            return Err(CornerNetworkError::NonConcurrentOffsets {
                maximum_error: maximum_offset_error,
            });
        }
        Ok(Self {
            center: corner + GeomVec::new(displacement[0], displacement[1], displacement[2]),
            radius,
            maximum_offset_error,
        })
    }

    #[inline]
    pub const fn center(&self) -> Pnt {
        self.center
    }

    #[inline]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    #[inline]
    pub const fn maximum_offset_error(&self) -> f64 {
        self.maximum_offset_error
    }
}

fn determinant3(matrix: [[f64; 3]; 3]) -> f64 {
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
}

fn solve3(matrix: [[f64; 3]; 3], right: [f64; 3], determinant: f64) -> [f64; 3] {
    let mut result = [0.0; 3];
    for column in 0..3 {
        let mut replaced = matrix;
        for row in 0..3 {
            replaced[row][column] = right[row];
        }
        result[column] = determinant3(replaced) / determinant;
    }
    result
}

/// Traversal-independent plan for a three-through-six-valent corner.
#[derive(Clone, Debug, PartialEq)]
pub struct CornerNetworkPlan {
    center: Pnt,
    material_axis: GeomVec,
    incidents: Vec<CornerIncidentBand>,
    certificate: CornerNetworkCertificate,
}

impl CornerNetworkPlan {
    pub const MIN_VERIFIED_VALENCE: usize = 3;
    pub const MAX_VERIFIED_VALENCE: usize = 6;

    /// Validate and canonically order incident bands without mutating topology.
    pub fn new(
        center: Pnt,
        incidents: impl IntoIterator<Item = CornerIncidentBand>,
        tolerance: &ToleranceContext,
    ) -> Result<Self, CornerNetworkError> {
        let incidents: Vec<_> = incidents.into_iter().collect();
        let valence = incidents.len();
        if !(Self::MIN_VERIFIED_VALENCE..=Self::MAX_VERIFIED_VALENCE).contains(&valence) {
            return Err(CornerNetworkError::UnsupportedValence { valence });
        }

        let mut directions = Vec::with_capacity(valence);
        let mut maximum_radius_error = 0.0_f64;
        for (index, incident) in incidents.iter().enumerate() {
            if !incident.radius.is_finite() || incident.radius <= tolerance.arithmetic_floor {
                return Err(CornerNetworkError::InvalidRadius {
                    index,
                    radius: incident.radius,
                });
            }
            let displacement = incident.contact - center;
            let distance = displacement.magnitude();
            if !distance.is_finite() || distance <= tolerance.arithmetic_floor {
                return Err(CornerNetworkError::DegenerateContact { index });
            }
            let radius_error = (distance - incident.radius).abs();
            maximum_radius_error = maximum_radius_error.max(radius_error);
            if radius_error > tolerance.policy.intersection {
                return Err(CornerNetworkError::ContactOffEnvelope {
                    index,
                    error: radius_error,
                });
            }
            directions.push(displacement / distance);
        }

        for first in 0..valence {
            for second in first + 1..valence {
                if incidents[first]
                    .contact
                    .distance(&incidents[second].contact)
                    <= tolerance.welding
                {
                    return Err(CornerNetworkError::DuplicateContact { first, second });
                }
            }
        }

        // Sort before reduction: floating-point addition is not associative, so
        // summing in caller traversal order can rotate a symmetric network's
        // derived axis by a few ulps and change its canonical first sector.
        let mut paired: Vec<_> = incidents.into_iter().zip(directions).collect();
        paired.sort_by(|left, right| compare_vector(left.1, right.1));
        let axis_sum = paired
            .iter()
            .map(|(_, direction)| *direction)
            .fold(GeomVec::ZERO, |sum, direction| sum + direction);
        let material_axis = axis_sum
            .normalized_vec()
            .ok_or(CornerNetworkError::AmbiguousMaterialAxis)?;

        let mut reference_candidates: Vec<_> = paired
            .iter()
            .enumerate()
            .map(|(index, (_, direction))| {
                let direction = *direction;
                let projected = direction - material_axis * direction.dot(&material_axis);
                (index, direction, projected)
            })
            .collect();
        reference_candidates.sort_by(|left, right| {
            right
                .2
                .magnitude_squared()
                .total_cmp(&left.2.magnitude_squared())
                .then_with(|| compare_vector(left.1, right.1))
        });
        let reference = reference_candidates[0]
            .2
            .normalized_vec()
            .ok_or(CornerNetworkError::AmbiguousMaterialAxis)?;
        let transverse = material_axis
            .cross(&reference)
            .normalized_vec()
            .ok_or(CornerNetworkError::AmbiguousMaterialAxis)?;

        let mut ordered: Vec<_> = paired
            .into_iter()
            .map(|(incident, direction)| {
                let projected = direction - material_axis * direction.dot(&material_axis);
                let angle = projected.dot(&transverse).atan2(projected.dot(&reference));
                (angle, direction, incident)
            })
            .collect();
        ordered.sort_by(|left, right| {
            left.0
                .total_cmp(&right.0)
                .then_with(|| compare_vector(left.1, right.1))
        });

        let full_turn = 2.0 * core::f64::consts::PI;
        let mut minimum_angular_spacing = full_turn;
        for index in 0..valence {
            let current = ordered[index].0;
            let next = if index + 1 == valence {
                ordered[0].0 + full_turn
            } else {
                ordered[index + 1].0
            };
            minimum_angular_spacing = minimum_angular_spacing.min(next - current);
        }
        let angular_floor = tolerance.policy.angular.max(
            tolerance.convergence / incidents_radius_scale(&ordered, tolerance.arithmetic_floor),
        );
        if minimum_angular_spacing <= angular_floor {
            return Err(CornerNetworkError::AmbiguousOrdering {
                minimum_spacing: minimum_angular_spacing,
            });
        }

        Ok(Self {
            center,
            material_axis,
            incidents: ordered
                .into_iter()
                .map(|(_, _, incident)| incident)
                .collect(),
            certificate: CornerNetworkCertificate {
                valence,
                maximum_radius_error,
                minimum_angular_spacing,
            },
        })
    }

    #[inline]
    pub const fn center(&self) -> Pnt {
        self.center
    }

    #[inline]
    pub const fn material_axis(&self) -> GeomVec {
        self.material_axis
    }

    #[inline]
    pub fn incidents(&self) -> &[CornerIncidentBand] {
        &self.incidents
    }

    #[inline]
    pub const fn certificate(&self) -> CornerNetworkCertificate {
        self.certificate
    }

    /// Whether this plan has the exact equal-radius orthogonal trihedron for
    /// which a spherical rolling-ball corner is an analytic specialization.
    pub fn is_orthogonal_three_valent(&self, tolerance: &ToleranceContext) -> bool {
        if self.incidents.len() != 3 {
            return false;
        }
        let radius = self.incidents[0].radius;
        let angular = tolerance.policy.angular.max(tolerance.convergence / radius);
        for incident in &self.incidents[1..] {
            if (incident.radius - radius).abs() > tolerance.policy.intersection {
                return false;
            }
        }
        let directions: Vec<_> = self
            .incidents
            .iter()
            .map(|incident| (incident.contact - self.center) / incident.radius)
            .collect();
        (0..3).all(|first| {
            (first + 1..3).all(|second| directions[first].dot(&directions[second]).abs() <= angular)
        })
    }
}

fn incidents_radius_scale(
    ordered: &[(f64, GeomVec, CornerIncidentBand)],
    arithmetic_floor: f64,
) -> f64 {
    ordered
        .iter()
        .map(|(_, _, incident)| incident.radius)
        .fold(arithmetic_floor, f64::max)
}

fn compare_vector(left: GeomVec, right: GeomVec) -> Ordering {
    left.x()
        .total_cmp(&right.x())
        .then_with(|| left.y().total_cmp(&right.y()))
        .then_with(|| left.z().total_cmp(&right.z()))
}

/// Typed pre-construction failures for blend-corner networks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CornerNetworkError {
    UnsupportedValence {
        valence: usize,
    },
    InvalidRadius {
        index: usize,
        radius: f64,
    },
    DegenerateContact {
        index: usize,
    },
    ContactOffEnvelope {
        index: usize,
        error: f64,
    },
    DuplicateContact {
        first: usize,
        second: usize,
    },
    AmbiguousMaterialAxis,
    AmbiguousOrdering {
        minimum_spacing: f64,
    },
    RankDeficientOffsets,
    NonConcurrentOffsets {
        maximum_error: f64,
    },
    IncompleteSelection {
        selected_bands: usize,
        vertex_valence: usize,
    },
}

impl core::fmt::Display for CornerNetworkError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedValence { valence } => write!(
                formatter,
                "fillet corner valence {valence} is outside the verified 3..=6 range"
            ),
            Self::InvalidRadius { index, radius } => {
                write!(
                    formatter,
                    "fillet corner band {index} has invalid radius {radius}"
                )
            }
            Self::DegenerateContact { index } => {
                write!(
                    formatter,
                    "fillet corner band {index} has a degenerate contact"
                )
            }
            Self::ContactOffEnvelope { index, error } => write!(
                formatter,
                "fillet corner band {index} misses its rolling envelope by {error}"
            ),
            Self::DuplicateContact { first, second } => write!(
                formatter,
                "fillet corner bands {first} and {second} have coincident contacts"
            ),
            Self::AmbiguousMaterialAxis => {
                formatter.write_str("fillet corner material axis is ambiguous")
            }
            Self::AmbiguousOrdering { minimum_spacing } => write!(
                formatter,
                "fillet corner ordering is ambiguous at angular spacing {minimum_spacing}"
            ),
            Self::RankDeficientOffsets => formatter
                .write_str("fillet corner offset planes do not define one stable sphere center"),
            Self::NonConcurrentOffsets { maximum_error } => write!(
                formatter,
                "fillet corner offset planes miss a common sphere by {maximum_error}"
            ),
            Self::IncompleteSelection {
                selected_bands,
                vertex_valence,
            } => write!(
                formatter,
                "fillet corner selects {selected_bands} of {vertex_valence} incident bands"
            ),
        }
    }
}

impl std::error::Error for CornerNetworkError {}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{BndBox, TolerancePolicy};

    fn context(center: Pnt, radius: f64) -> ToleranceContext {
        let mut bounds = BndBox::new();
        bounds.add(&Pnt::new(
            center.x() - radius,
            center.y() - radius,
            center.z() - radius,
        ));
        bounds.add(&Pnt::new(
            center.x() + radius,
            center.y() + radius,
            center.z() + radius,
        ));
        ToleranceContext::derive(&TolerancePolicy::STANDARD, &[bounds], Some(radius), 1.0)
            .expect("corner tolerance context")
    }

    fn ring(center: Pnt, radius: f64, valence: usize) -> Vec<CornerIncidentBand> {
        (0..valence)
            .map(|index| {
                let angle = 2.0 * core::f64::consts::PI * index as f64 / valence as f64;
                let direction = GeomVec::new(angle.cos(), angle.sin(), 0.5)
                    .normalized_vec()
                    .unwrap();
                CornerIncidentBand {
                    contact: center + direction * radius,
                    radius,
                    support: BandSupportKind::Cylinder,
                }
            })
            .collect()
    }

    #[test]
    fn canonical_order_is_input_independent_for_verified_valences() {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for center in [Pnt::ORIGIN, Pnt::new(1.0e9, -1.0e9, 5.0e8)] {
                for valence in 3..=6 {
                    let incidents = ring(center, scale, valence);
                    let tolerance = context(center, scale);
                    let expected =
                        CornerNetworkPlan::new(center, incidents.iter().copied(), &tolerance)
                            .unwrap();
                    for shift in 0..valence {
                        let reordered = (0..valence)
                            .rev()
                            .map(|index| incidents[(index + shift) % valence]);
                        let actual = CornerNetworkPlan::new(center, reordered, &tolerance).unwrap();
                        assert_eq!(actual.incidents(), expected.incidents());
                        assert_eq!(actual.certificate().valence, valence);
                    }
                }
            }
        }
    }

    #[test]
    fn orthogonal_three_valent_plan_selects_exact_sphere_specialization() {
        let center = Pnt::new(1.0e6, -2.0e6, 3.0e6);
        let radius = 4.0;
        let incidents =
            [GeomVec::DX, GeomVec::DY, GeomVec::DZ].map(|direction| CornerIncidentBand {
                contact: center + direction * radius,
                radius,
                support: BandSupportKind::Cylinder,
            });
        let tolerance = context(center, radius);
        let plan = CornerNetworkPlan::new(center, incidents, &tolerance).unwrap();
        assert!(plan.is_orthogonal_three_valent(&tolerance));
    }

    #[test]
    fn unsupported_or_ambiguous_inputs_reject_before_geometry() {
        let center = Pnt::ORIGIN;
        let tolerance = context(center, 1.0);
        assert!(matches!(
            CornerNetworkPlan::new(center, ring(center, 1.0, 2), &tolerance),
            Err(CornerNetworkError::UnsupportedValence { valence: 2 })
        ));
        assert!(matches!(
            CornerNetworkPlan::new(center, ring(center, 1.0, 7), &tolerance),
            Err(CornerNetworkError::UnsupportedValence { valence: 7 })
        ));
        let duplicate = CornerIncidentBand {
            contact: Pnt::new(1.0, 0.0, 0.0),
            radius: 1.0,
            support: BandSupportKind::Cylinder,
        };
        assert!(matches!(
            CornerNetworkPlan::new(center, [duplicate; 3], &tolerance),
            Err(CornerNetworkError::DuplicateContact { .. })
        ));
    }

    #[test]
    fn tangent_sphere_solve_is_scale_and_far_origin_stable() {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for corner in [Pnt::ORIGIN, Pnt::new(1.0e9, -1.0e9, 5.0e8)] {
                let radius = 2.0 * scale;
                let tolerance = context(corner, radius);
                let sphere = CornerTangentSphere::new(
                    corner,
                    [
                        Dir::new(1.0, 0.0, 0.0),
                        Dir::new(0.0, 1.0, 0.0),
                        Dir::new(0.0, 0.0, 1.0),
                    ],
                    radius,
                    &tolerance,
                )
                .unwrap();
                let expected = corner + GeomVec::new(-radius, -radius, -radius);
                assert!(sphere.center().distance(&expected) <= tolerance.policy.intersection);
                assert!(sphere.maximum_offset_error() <= tolerance.policy.intersection);
            }
        }
    }
}
