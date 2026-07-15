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

use openrcad_foundation::{tolerance, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomCurve, GeomSurface, Line, Plane, RuledSurface};
use openrcad_topo::{Edge, Face, Solid, Wire};

use crate::revolve::{loop_agrees_with_surface, reversed_wire};
use crate::sew::{compatibility_policy, sew_with_policy};

/// Errors reported by [`skin_polygon_rings`].
#[derive(Clone, Debug, PartialEq)]
pub enum SkinError {
    /// Fewer than two rings, or a ring with fewer than three points.
    TooFewRings,
    /// Rings have different point counts (resample before skinning).
    RingMismatch,
    /// A ring is degenerate (zero area / repeated points).
    DegenerateRing,
    /// The skinned shell did not close watertight.
    NotWatertight,
}

impl fmt::Display for SkinError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewRings => f.write_str("skin: need ≥ 2 rings of ≥ 3 points"),
            Self::RingMismatch => f.write_str("skin: all rings must have the same point count"),
            Self::DegenerateRing => f.write_str("skin: a ring is degenerate"),
            Self::NotWatertight => f.write_str("skin: result did not close watertight"),
        }
    }
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
pub fn skin_polygon_rings(rings: &[Vec<Pnt>]) -> Result<Solid, SkinError> {
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

    let policy = compatibility_policy(tolerance::CONFUSION * 10.0);
    let solid =
        Solid::new(sew_with_policy(&faces, &policy).expect("compatibility policy is valid"));
    if !solid.is_watertight() {
        return Err(SkinError::NotWatertight);
    }
    Ok(solid)
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
