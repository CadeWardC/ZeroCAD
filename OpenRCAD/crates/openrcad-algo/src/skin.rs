//! Skinning: build a solid through a sequence of closed polygon RINGS —
//! the shared engine behind loft (rings = resampled profile sections) and
//! sweep (rings = one profile transported along a path's frames).
//!
//! Consecutive rings are joined by one quad face per segment (planar when the
//! four corners are coplanar, ruled otherwise); the first and last rings are
//! capped with planar polygon faces. Rings are aligned automatically —
//! winding direction and starting index — so callers only provide ordered
//! ring points. Consumes sampled polygons, not curves: sections/paths with
//! arcs arrive pre-sampled (ZeroCAD regions already are), so lateral walls
//! are faceted at the sampling density, like any sketched polygon extrude.

use core::fmt;

use openrcad_foundation::{tolerance, Dir, Pnt, TolerancePolicy, Vec as GeomVec};
use openrcad_geom::{GeomCurve, GeomSurface, Line, Plane, RuledSurface};
use openrcad_topo::{Edge, Face, Orientation, Solid, Wire};

use crate::revolve::{loop_agrees_with_surface, reversed_wire};
use crate::sew::sew_shell_with_policy as sew_with_policy;

/// Errors reported by [`skin_polygon_rings`].
#[derive(Clone, Debug, PartialEq)]
pub enum SkinError {
    /// Fewer than two rings, or a ring with fewer than three points.
    TooFewRings,
    /// Rings have different point counts (resample before skinning).
    RingMismatch,
    /// Sections contain different numbers of material holes.
    HoleCountMismatch {
        section: usize,
        expected: usize,
        actual: usize,
    },
    /// Corresponding hole rings have different point counts.
    HoleRingMismatch {
        section: usize,
        hole: usize,
        expected: usize,
        actual: usize,
    },
    /// Two or more hole assignments have indistinguishable geometric cost.
    AmbiguousHoleCorrespondence { section: usize },
    /// A ring is degenerate (zero area / repeated points).
    DegenerateRing,
    /// The skinned shell did not close watertight.
    NotWatertight,
    /// The supplied document tolerance policy is internally inconsistent.
    InvalidTolerancePolicy(String),
}

impl fmt::Display for SkinError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewRings => f.write_str("skin: need ≥ 2 rings of ≥ 3 points"),
            Self::RingMismatch => f.write_str("skin: all rings must have the same point count"),
            Self::HoleCountMismatch {
                section,
                expected,
                actual,
            } => write!(
                f,
                "skin: section {section} has {actual} holes; expected {expected}"
            ),
            Self::HoleRingMismatch {
                section,
                hole,
                expected,
                actual,
            } => write!(
                f,
                "skin: section {section} hole {hole} has {actual} points; expected {expected}"
            ),
            Self::AmbiguousHoleCorrespondence { section } => write!(
                f,
                "skin: section {section} has ambiguous hole correspondence"
            ),
            Self::DegenerateRing => f.write_str("skin: a ring is degenerate"),
            Self::NotWatertight => f.write_str("skin: result did not close watertight"),
            Self::InvalidTolerancePolicy(reason) => {
                write!(f, "skin: invalid tolerance policy: {reason}")
            }
        }
    }
}

/// One complete polygonal skin section. Inner loops bound voids and may appear
/// in any input order; correspondence is resolved geometrically between
/// consecutive sections.
#[derive(Clone, Debug, PartialEq)]
pub struct SectionLoops {
    pub outer: Vec<Pnt>,
    pub holes: Vec<Vec<Pnt>>,
}

impl std::error::Error for SkinError {}

/// Newell normal of a 3D polygon.
fn ring_normal(ring: &[Pnt]) -> Option<Dir> {
    let mut n = GeomVec::ZERO;
    for i in 0..ring.len() {
        let p = ring[i];
        let q = ring[(i + 1) % ring.len()];
        n += GeomVec::new(
            (p.y() - q.y()) * (p.z() + q.z()),
            (p.z() - q.z()) * (p.x() + q.x()),
            (p.x() - q.x()) * (p.y() + q.y()),
        );
    }
    n.normalized()
}

/// Build a solid through `rings` (ordered ring points; each ring closed
/// implicitly, first point NOT repeated at the end).
#[deprecated(note = "use skin_polygon_rings_with_policy")]
pub fn skin_polygon_rings(rings: &[Vec<Pnt>]) -> Result<Solid, SkinError> {
    skin_polygon_rings_with_policy(rings, &TolerancePolicy::STANDARD)
}

pub fn skin_polygon_rings_with_policy(
    rings: &[Vec<Pnt>],
    policy: &TolerancePolicy,
) -> Result<Solid, SkinError> {
    policy
        .validate()
        .map_err(|error| SkinError::InvalidTolerancePolicy(error.to_string()))?;
    if rings.len() < 2 || rings.iter().any(|r| r.len() < 3) {
        return Err(SkinError::TooFewRings);
    }
    let n = rings[0].len();
    if rings.iter().any(|r| r.len() != n) {
        return Err(SkinError::RingMismatch);
    }

    // Align every ring to the previous one: match winding (flip if the ring
    // normals oppose) and rotate the start index to the cyclically-nearest
    // correspondence, so the skin doesn't twist.
    let mut aligned: Vec<Vec<Pnt>> = Vec::with_capacity(rings.len());
    aligned.push(rings[0].clone());
    let mut prev_normal = ring_normal(&rings[0]).ok_or(SkinError::DegenerateRing)?;
    for ring in &rings[1..] {
        let mut r = ring.clone();
        let rn = ring_normal(&r).ok_or(SkinError::DegenerateRing)?;
        if GeomVec::from_dir(rn).dot(&GeomVec::from_dir(prev_normal)) < 0.0 {
            r.reverse();
        }
        prev_normal = ring_normal(&r).ok_or(SkinError::DegenerateRing)?;
        let prev = aligned.last().unwrap();
        let mut best_shift = 0usize;
        let mut best_cost = f64::INFINITY;
        for shift in 0..n {
            let cost: f64 = (0..n).map(|j| prev[j].distance(&r[(j + shift) % n])).sum();
            if cost < best_cost {
                best_cost = cost;
                best_shift = shift;
            }
        }
        let shifted: Vec<Pnt> = (0..n).map(|j| r[(j + best_shift) % n]).collect();
        aligned.push(shifted);
    }

    let mut faces: Vec<Face> = Vec::new();

    // Laterals between consecutive rings.
    for w in aligned.windows(2) {
        let (r0, r1) = (&w[0], &w[1]);
        for j in 0..n {
            let a = r0[j];
            let b = r0[(j + 1) % n];
            let c = r1[(j + 1) % n];
            let d = r1[j];
            // Degenerate segment (repeated ring point) → no face.
            if a.distance(&b) < tolerance::CONFUSION && c.distance(&d) < tolerance::CONFUSION {
                continue;
            }
            let wire = Wire::from_edges([
                Edge::between_points(a, b),
                Edge::between_points(b, c),
                Edge::between_points(c, d),
                Edge::between_points(d, a),
            ]);
            // Planar quad when the four corners are coplanar; ruled otherwise.
            let ab = b - a;
            let ad = d - a;
            let quad_n = ab.cross(&ad);
            let coplanar = quad_n
                .normalized()
                .map(|nrm| ((c - a).dot(&GeomVec::from_dir(nrm))).abs() < 1e-7)
                .unwrap_or(false);
            let surface = if coplanar {
                let nrm = quad_n.normalized().ok_or(SkinError::DegenerateRing)?;
                GeomSurface::plane(Plane::from_point_normal(a, nrm))
            } else {
                let dir_ab = (b - a).normalized().ok_or(SkinError::DegenerateRing)?;
                let dir_dc = (c - d).normalized().ok_or(SkinError::DegenerateRing)?;
                let bottom = GeomCurve::line(Line::from_point_dir(a, dir_ab));
                let top = GeomCurve::line(Line::from_point_dir(d, dir_dc));
                GeomSurface::ruled(RuledSurface::new(bottom, top))
            };
            // Loops must wind CCW in the surface's own uv (the revolve
            // lesson): the tessellator winds triangles from
            // orientation ⊗ intrinsic normal and ignores loop direction.
            let center = Pnt::new(
                (a.x() + b.x() + c.x() + d.x()) / 4.0,
                (a.y() + b.y() + c.y() + d.y()) / 4.0,
                (a.z() + b.z() + c.z() + d.z()) / 4.0,
            );
            let wire = if loop_agrees_with_surface(&wire, &surface, center) {
                wire
            } else {
                reversed_wire(&wire)
            };
            faces.push(Face::new(Some(surface), wire));
        }
    }

    // Caps on the first and last rings.
    for (ring, first) in [(&aligned[0], true), (aligned.last().unwrap(), false)] {
        let nrm = ring_normal(ring).ok_or(SkinError::DegenerateRing)?;
        let edges: Vec<Edge> = (0..n)
            .filter(|&j| ring[j].distance(&ring[(j + 1) % n]) > tolerance::CONFUSION)
            .map(|j| Edge::between_points(ring[j], ring[(j + 1) % n]))
            .collect();
        if edges.len() < 3 {
            return Err(SkinError::DegenerateRing);
        }
        let _ = first; // caps are planar: sew canonicalizes their orientation.
        faces.push(Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(ring[0], nrm))),
            Wire::from_edges(edges),
        ));
    }

    let solid = Solid::new(
        sew_with_policy(&faces, policy)
            .map_err(|error| SkinError::InvalidTolerancePolicy(error.to_string()))?,
    );
    if !solid.is_watertight_with_policy(policy) {
        return Err(SkinError::NotWatertight);
    }
    Ok(solid)
}

/// Skin complete material sections, including through-holes. Caps are built as
/// one planar face with inner wires, while each matched hole receives its own
/// oppositely-oriented lateral wall.
pub fn skin_section_loops_with_policy(
    sections: &[SectionLoops],
    policy: &TolerancePolicy,
) -> Result<Solid, SkinError> {
    skin_section_loops_impl(sections, policy, true, false)
}

/// Skin sections whose point and hole ordering is already authoritative, as it
/// is for one profile transported by a sweep frame. Geometric ring alignment is
/// deliberately disabled so it cannot erase an explicit total twist. A closed
/// skin expects the caller to append an exact copy of its first section after
/// the final path section; that seam is connected without planar end caps.
pub fn skin_ordered_section_loops_with_policy(
    sections: &[SectionLoops],
    closed: bool,
    policy: &TolerancePolicy,
) -> Result<Solid, SkinError> {
    skin_section_loops_impl(sections, policy, false, closed)
}

fn skin_section_loops_impl(
    sections: &[SectionLoops],
    policy: &TolerancePolicy,
    align_geometrically: bool,
    closed: bool,
) -> Result<Solid, SkinError> {
    policy
        .validate()
        .map_err(|error| SkinError::InvalidTolerancePolicy(error.to_string()))?;
    if sections.len() < 2 || sections.iter().any(|section| section.outer.len() < 3) {
        return Err(SkinError::TooFewRings);
    }
    let outer_points = sections[0].outer.len();
    if sections
        .iter()
        .any(|section| section.outer.len() != outer_points)
    {
        return Err(SkinError::RingMismatch);
    }
    let hole_count = sections[0].holes.len();
    for (section_index, section) in sections.iter().enumerate() {
        if section.holes.len() != hole_count {
            return Err(SkinError::HoleCountMismatch {
                section: section_index,
                expected: hole_count,
                actual: section.holes.len(),
            });
        }
        if section.holes.iter().any(|hole| hole.len() < 3) {
            return Err(SkinError::DegenerateRing);
        }
    }

    let mut aligned = Vec::with_capacity(sections.len());
    let first_outer_normal = ring_normal(&sections[0].outer).ok_or(SkinError::DegenerateRing)?;
    let first_holes = sections[0]
        .holes
        .iter()
        .map(|hole| orient_hole(hole, first_outer_normal))
        .collect::<Result<Vec<_>, _>>()?;
    aligned.push(SectionLoops {
        outer: sections[0].outer.clone(),
        holes: first_holes,
    });

    for (section_index, section) in sections.iter().enumerate().skip(1) {
        if !align_geometrically {
            let outer_normal = ring_normal(&section.outer).ok_or(SkinError::DegenerateRing)?;
            let holes = section
                .holes
                .iter()
                .map(|hole| orient_hole(hole, outer_normal))
                .collect::<Result<Vec<_>, _>>()?;
            aligned.push(SectionLoops {
                outer: section.outer.clone(),
                holes,
            });
            continue;
        }
        let previous = aligned.last().expect("first section was inserted");
        let outer = align_ring(&previous.outer, &section.outer).ok_or(SkinError::RingMismatch)?;
        let outer_normal = ring_normal(&outer).ok_or(SkinError::DegenerateRing)?;
        let candidates = section
            .holes
            .iter()
            .map(|hole| orient_hole(hole, outer_normal))
            .collect::<Result<Vec<_>, _>>()?;
        let correspondence = match_holes(
            &previous.holes,
            &candidates,
            section_index,
            policy.classification,
        )?;
        let mut holes = Vec::with_capacity(hole_count);
        for (hole_index, candidate_index) in correspondence.into_iter().enumerate() {
            let expected = previous.holes[hole_index].len();
            let actual = candidates[candidate_index].len();
            if actual != expected {
                return Err(SkinError::HoleRingMismatch {
                    section: section_index,
                    hole: hole_index,
                    expected,
                    actual,
                });
            }
            holes.push(
                align_ring(&previous.holes[hole_index], &candidates[candidate_index])
                    .ok_or(SkinError::DegenerateRing)?,
            );
        }
        aligned.push(SectionLoops { outer, holes });
    }

    let mut faces = Vec::new();
    let outer_rings: Vec<_> = aligned
        .iter()
        .map(|section| section.outer.clone())
        .collect();
    append_lateral_faces(&outer_rings, &mut faces)?;
    for hole_index in 0..hole_count {
        let hole_rings: Vec<_> = aligned
            .iter()
            .map(|section| section.holes[hole_index].clone())
            .collect();
        append_lateral_faces(&hole_rings, &mut faces)?;
    }

    if !closed {
        for section in [&aligned[0], aligned.last().unwrap()] {
            let normal = ring_normal(&section.outer).ok_or(SkinError::DegenerateRing)?;
            let surface = GeomSurface::plane(Plane::from_point_normal(section.outer[0], normal));
            let outer = ring_wire(&section.outer)?;
            let inners = section
                .holes
                .iter()
                .map(|hole| ring_wire(hole))
                .collect::<Result<Vec<_>, _>>()?;
            faces.push(Face::with_wires(
                Some(surface),
                Some(outer),
                inners,
                Orientation::Forward,
            ));
        }
    }

    let solid = Solid::new(
        sew_with_policy(&faces, policy)
            .map_err(|error| SkinError::InvalidTolerancePolicy(error.to_string()))?,
    );
    if !solid.is_watertight_with_policy(policy) {
        return Err(SkinError::NotWatertight);
    }
    Ok(solid)
}

fn align_ring(previous: &[Pnt], current: &[Pnt]) -> Option<Vec<Pnt>> {
    if previous.len() != current.len() || previous.len() < 3 {
        return None;
    }
    let mut ring = current.to_vec();
    let previous_normal = ring_normal(previous)?;
    let current_normal = ring_normal(&ring)?;
    if GeomVec::from_dir(current_normal).dot(&GeomVec::from_dir(previous_normal)) < 0.0 {
        ring.reverse();
    }
    let count = ring.len();
    let best_shift = (0..count).min_by(|first, second| {
        let cost = |shift: usize| {
            (0..count)
                .map(|index| previous[index].distance(&ring[(index + shift) % count]))
                .sum::<f64>()
        };
        cost(*first).total_cmp(&cost(*second))
    })?;
    Some(
        (0..count)
            .map(|index| ring[(index + best_shift) % count])
            .collect(),
    )
}

fn orient_hole(hole: &[Pnt], outer_normal: Dir) -> Result<Vec<Pnt>, SkinError> {
    let mut hole = hole.to_vec();
    let normal = ring_normal(&hole).ok_or(SkinError::DegenerateRing)?;
    if GeomVec::from_dir(normal).dot(&GeomVec::from_dir(outer_normal)) > 0.0 {
        hole.reverse();
    }
    Ok(hole)
}

fn ring_centroid(ring: &[Pnt]) -> Pnt {
    let count = ring.len() as f64;
    Pnt::new(
        ring.iter().map(Pnt::x).sum::<f64>() / count,
        ring.iter().map(Pnt::y).sum::<f64>() / count,
        ring.iter().map(Pnt::z).sum::<f64>() / count,
    )
}

fn ring_area_scale(ring: &[Pnt]) -> f64 {
    let mut area_vector = GeomVec::ZERO;
    for index in 0..ring.len() {
        let a = GeomVec::new(ring[index].x(), ring[index].y(), ring[index].z());
        let b = GeomVec::new(
            ring[(index + 1) % ring.len()].x(),
            ring[(index + 1) % ring.len()].y(),
            ring[(index + 1) % ring.len()].z(),
        );
        area_vector += a.cross(&b);
    }
    (area_vector.magnitude() * 0.5).sqrt()
}

fn hole_cost(first: &[Pnt], second: &[Pnt]) -> f64 {
    let centroid_cost = ring_centroid(first)
        .distance(&ring_centroid(second))
        .powi(2);
    let scale_cost = (ring_area_scale(first) - ring_area_scale(second)).powi(2);
    centroid_cost + scale_cost
}

fn match_holes(
    previous: &[Vec<Pnt>],
    current: &[Vec<Pnt>],
    section: usize,
    tolerance: f64,
) -> Result<Vec<usize>, SkinError> {
    let count = previous.len();
    if count == 0 {
        return Ok(Vec::new());
    }
    let costs: Vec<Vec<f64>> = previous
        .iter()
        .map(|first| {
            current
                .iter()
                .map(|second| hole_cost(first, second))
                .collect()
        })
        .collect();
    if count > 8 {
        let mut used = vec![false; count];
        let mut assignment = Vec::with_capacity(count);
        for row in &costs {
            let mut available: Vec<_> = row
                .iter()
                .enumerate()
                .filter(|(index, _)| !used[*index])
                .collect();
            available.sort_by(|a, b| a.1.total_cmp(b.1));
            if available.len() > 1 && (*available[1].1 - *available[0].1).abs() <= tolerance.powi(2)
            {
                return Err(SkinError::AmbiguousHoleCorrespondence { section });
            }
            let chosen = available[0].0;
            used[chosen] = true;
            assignment.push(chosen);
        }
        return Ok(assignment);
    }

    fn visit(
        row: usize,
        costs: &[Vec<f64>],
        used: &mut [bool],
        assignment: &mut Vec<usize>,
        cost: f64,
        best: &mut Option<(f64, Vec<usize>)>,
        second: &mut f64,
    ) {
        if row == costs.len() {
            if best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
                if let Some((best_cost, _)) = best.as_ref() {
                    *second = *best_cost;
                }
                *best = Some((cost, assignment.clone()));
            } else if cost < *second {
                *second = cost;
            }
            return;
        }
        for column in 0..costs.len() {
            if used[column] {
                continue;
            }
            used[column] = true;
            assignment.push(column);
            visit(
                row + 1,
                costs,
                used,
                assignment,
                cost + costs[row][column],
                best,
                second,
            );
            assignment.pop();
            used[column] = false;
        }
    }

    let mut best = None;
    let mut second = f64::INFINITY;
    visit(
        0,
        &costs,
        &mut vec![false; count],
        &mut Vec::with_capacity(count),
        0.0,
        &mut best,
        &mut second,
    );
    let (best_cost, assignment) = best.expect("non-empty square assignment");
    if second.is_finite() && (second - best_cost).abs() <= tolerance.powi(2) * count as f64 {
        return Err(SkinError::AmbiguousHoleCorrespondence { section });
    }
    Ok(assignment)
}

fn append_lateral_faces(rings: &[Vec<Pnt>], faces: &mut Vec<Face>) -> Result<(), SkinError> {
    let count = rings[0].len();
    for pair in rings.windows(2) {
        let (first, second) = (&pair[0], &pair[1]);
        for index in 0..count {
            let a = first[index];
            let b = first[(index + 1) % count];
            let c = second[(index + 1) % count];
            let d = second[index];
            if a.distance(&b) < tolerance::CONFUSION && c.distance(&d) < tolerance::CONFUSION {
                continue;
            }
            let wire = Wire::from_edges([
                Edge::between_points(a, b),
                Edge::between_points(b, c),
                Edge::between_points(c, d),
                Edge::between_points(d, a),
            ]);
            let ab = b - a;
            let ad = d - a;
            let quad_normal = ab.cross(&ad);
            let coplanar = quad_normal
                .normalized()
                .map(|normal| {
                    ((c - a).dot(&GeomVec::from_dir(normal))).abs() < tolerance::CONFUSION
                })
                .unwrap_or(false);
            let surface = if coplanar {
                let normal = quad_normal.normalized().ok_or(SkinError::DegenerateRing)?;
                GeomSurface::plane(Plane::from_point_normal(a, normal))
            } else {
                let bottom_direction = (b - a).normalized().ok_or(SkinError::DegenerateRing)?;
                let top_direction = (c - d).normalized().ok_or(SkinError::DegenerateRing)?;
                GeomSurface::ruled(RuledSurface::new(
                    GeomCurve::line(Line::from_point_dir(a, bottom_direction)),
                    GeomCurve::line(Line::from_point_dir(d, top_direction)),
                ))
            };
            let center = Pnt::new(
                (a.x() + b.x() + c.x() + d.x()) / 4.0,
                (a.y() + b.y() + c.y() + d.y()) / 4.0,
                (a.z() + b.z() + c.z() + d.z()) / 4.0,
            );
            let wire = if loop_agrees_with_surface(&wire, &surface, center) {
                wire
            } else {
                reversed_wire(&wire)
            };
            faces.push(Face::new(Some(surface), wire));
        }
    }
    Ok(())
}

fn ring_wire(ring: &[Pnt]) -> Result<Wire, SkinError> {
    let edges: Vec<_> = (0..ring.len())
        .filter(|index| {
            ring[*index].distance(&ring[(*index + 1) % ring.len()]) > tolerance::CONFUSION
        })
        .map(|index| Edge::between_points(ring[index], ring[(index + 1) % ring.len()]))
        .collect();
    if edges.len() < 3 {
        return Err(SkinError::DegenerateRing);
    }
    Ok(Wire::from_edges(edges))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_mesh::mass_properties;

    fn square_ring(half: f64, z: f64) -> Vec<Pnt> {
        vec![
            Pnt::new(-half, -half, z),
            Pnt::new(half, -half, z),
            Pnt::new(half, half, z),
            Pnt::new(-half, half, z),
        ]
    }

    fn square_hole(center_x: f64, center_y: f64, half: f64, z: f64) -> Vec<Pnt> {
        square_ring(half, z)
            .into_iter()
            .map(|point| Pnt::new(point.x() + center_x, point.y() + center_y, point.z()))
            .collect()
    }

    fn volume(solid: &Solid) -> f64 {
        let mesh = openrcad_mesh::tessellate(solid, 0.002, 0.05);
        mass_properties(&mesh).expect("closed mesh").volume
    }

    #[test]
    fn straight_skin_is_a_box() {
        let solid = skin_polygon_rings(&[square_ring(1.0, 0.0), square_ring(1.0, 3.0)]).unwrap();
        assert!(solid.is_watertight());
        let v = volume(&solid);
        assert!((v - 12.0).abs() < 1e-6, "box volume {v}");
    }

    #[test]
    fn tapered_skin_is_a_frustum() {
        // Square 2×2 → 1×1 over height 3: pyramidal frustum,
        // V = h/3 (A0 + A1 + √(A0·A1)).
        let solid = skin_polygon_rings(&[square_ring(1.0, 0.0), square_ring(0.5, 3.0)]).unwrap();
        assert!(solid.is_watertight());
        let exact = 3.0 / 3.0 * (4.0 + 1.0 + 2.0);
        let v = volume(&solid);
        assert!(
            (v - exact).abs() / exact < 0.01,
            "frustum volume {v} vs {exact}"
        );
    }

    #[test]
    fn misaligned_rings_are_realigned() {
        // Second ring rotated two steps and wound the other way: alignment
        // must recover the plain box, not a bow-tie.
        let mut r1 = square_ring(1.0, 3.0);
        r1.reverse();
        r1.rotate_left(2);
        let solid = skin_polygon_rings(&[square_ring(1.0, 0.0), r1]).unwrap();
        assert!(solid.is_watertight());
        let v = volume(&solid);
        assert!((v - 12.0).abs() < 1e-6, "realigned volume {v}");
    }

    #[test]
    fn multi_ring_skin_through_offset_sections() {
        // Three sections with the middle one shifted sideways: a sheared tube.
        // Shearing preserves volume per slab.
        let mut mid = square_ring(1.0, 2.0);
        for p in &mut mid {
            *p = Pnt::new(p.x() + 1.5, p.y(), p.z());
        }
        let solid =
            skin_polygon_rings(&[square_ring(1.0, 0.0), mid, square_ring(1.0, 4.0)]).unwrap();
        assert!(solid.is_watertight());
        let v = volume(&solid);
        assert!((v - 16.0).abs() / 16.0 < 0.01, "sheared volume {v}");
    }

    #[test]
    fn reordered_holes_receive_inner_caps_and_oriented_walls() {
        let bottom = SectionLoops {
            outer: square_ring(5.0, 0.0),
            holes: vec![
                square_hole(-2.0, 0.0, 1.0, 0.0),
                square_hole(2.0, 0.0, 1.0, 0.0),
            ],
        };
        let top = SectionLoops {
            outer: square_ring(5.0, 4.0),
            holes: vec![
                square_hole(2.0, 0.0, 1.0, 4.0),
                square_hole(-2.0, 0.0, 1.0, 4.0),
            ],
        };
        let solid = skin_section_loops_with_policy(&[bottom, top], &TolerancePolicy::STANDARD)
            .expect("hole order must not define correspondence");
        assert!(solid.is_watertight());
        assert_eq!(
            solid
                .faces()
                .iter()
                .filter(|face| face.inner_wires().len() == 2)
                .count(),
            2,
            "both caps must own both inner wires"
        );
        let value = volume(&solid);
        assert!(
            (value - 368.0).abs() / 368.0 < 0.01,
            "holed prism volume {value}"
        );
    }

    #[test]
    fn symmetric_hole_motion_is_unresolved() {
        let bottom = SectionLoops {
            outer: square_ring(5.0, 0.0),
            holes: vec![
                square_hole(-2.0, 0.0, 0.5, 0.0),
                square_hole(2.0, 0.0, 0.5, 0.0),
            ],
        };
        let top = SectionLoops {
            outer: square_ring(5.0, 4.0),
            holes: vec![
                square_hole(0.0, -2.0, 0.5, 4.0),
                square_hole(0.0, 2.0, 0.5, 4.0),
            ],
        };
        assert_eq!(
            skin_section_loops_with_policy(&[bottom, top], &TolerancePolicy::STANDARD).unwrap_err(),
            SkinError::AmbiguousHoleCorrespondence { section: 1 }
        );
    }

    #[test]
    fn bad_inputs_are_rejected() {
        assert_eq!(
            skin_polygon_rings(&[square_ring(1.0, 0.0)]).unwrap_err(),
            SkinError::TooFewRings
        );
        let tri = vec![
            Pnt::new(0.0, 0.0, 3.0),
            Pnt::new(1.0, 0.0, 3.0),
            Pnt::new(0.0, 1.0, 3.0),
        ];
        assert_eq!(
            skin_polygon_rings(&[square_ring(1.0, 0.0), tri]).unwrap_err(),
            SkinError::RingMismatch
        );
    }
}
