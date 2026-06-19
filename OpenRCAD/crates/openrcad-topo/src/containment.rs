//! 2D containment (point-in-polygon) tests shared across the kernel.
//!
//! The boolean split pass (in `openrcad-algo`) and the face-partitioning
//! builder (this crate) both need to answer "is this `(u, v)` inside this
//! trimming polygon?" Historically there were two copies: a naive
//! division-based one here in topology (ill-conditioned for near-horizontal
//! edges) and the robust exact-predicate one in the intersection engine. The
//! robust version lives here now — the single lower-crate home both layers can
//! import without an `algo -> topo` cycle.

use openrcad_foundation::predicates::orient2d;
use openrcad_foundation::Pnt2d;

/// Robust Jordan-curve point-in-polygon test in 2D.
///
/// Crossing-number ray test, but each "does the +x ray cross this edge?"
/// decision is made with the exact [`orient2d`] predicate rather than the
/// classic floating-point slope comparison `q.x < (…)/(pj.y - pi.y)`. That
/// division is ill-conditioned for near-horizontal edges (and the usual
/// `+ 1e-15` guard silently biases the result); `orient2d` is division-free and
/// promotes to exact arithmetic exactly when the sign is in doubt, so a point
/// near a sliver edge is classified consistently with the same edge seen from
/// an adjacent face. Operates in surface `(u, v)` parameter space, where an
/// orientation test is still a valid 2D predicate.
pub fn point_in_polygon_2d(q: (f64, f64), poly: &[(f64, f64)]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let qp = Pnt2d::new(q.0, q.1);
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[j], poly[i]); // directed edge a -> b
        // Does the edge straddle the horizontal line through q?
        if (a.1 > q.1) != (b.1 > q.1) {
            // The +x ray from q crosses edge a->b iff q lies on the side of the
            // directed edge that faces the crossing: for an upward edge that is
            // the left side (orient2d > 0), for a downward edge the right side.
            let side = orient2d(Pnt2d::new(a.0, a.1), Pnt2d::new(b.0, b.1), qp);
            let upward = b.1 > a.1;
            if upward == (side > 0.0) {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::point_in_polygon_2d;

    #[test]
    fn unit_square_classifies_corners_and_centre() {
        let square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        assert!(point_in_polygon_2d((5.0, 5.0), &square));
        assert!(!point_in_polygon_2d((-1.0, 5.0), &square));
        assert!(!point_in_polygon_2d((11.0, 5.0), &square));
    }

    #[test]
    fn near_sliver_edge_is_stable() {
        // A triangle with a nearly-horizontal edge; the naive division-based
        // test would flip the answer for a probe hugging that edge.
        let tri = [(0.0, 0.0), (10.0, 1e-12), (0.0, 10.0)];
        // Centroid is clearly inside.
        assert!(point_in_polygon_2d((1.0, 3.0), &tri));
        // Just outside the sliver edge's far end.
        assert!(!point_in_polygon_2d((11.0, 1e-12), &tri));
    }
}
