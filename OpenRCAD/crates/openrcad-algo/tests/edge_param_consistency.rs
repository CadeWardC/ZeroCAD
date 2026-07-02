//! Every edge of a fillet result must be param/vertex CONSISTENT:
//! `curve.point(first)` is its start vertex and `curve.point(last)` its end.
//!
//! Regression for the reversed-edge trim bug: `source()`/`target()` honour wire
//! orientation but `first()`/`last()` are raw storage, and
//! `shorten_edge_keep_curve` paired them inconsistently — the circular-bite
//! flush trim emitted a bite arc whose end vertex and end parameter named
//! different points. Downstream, boundary discretization walked the bogus param
//! range (a stub nowhere near the stored endpoints), leaving a chord across the
//! true arc: mesh cracks, a phantom membrane over the bite, and the fillet
//! validator rejecting every candidate (a 100x slowdown that read as a hang).

use openrcad_algo::{boolean, fillet_edges, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::Curve as _;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Solid};

fn assert_edges_param_consistent(solid: &Solid, label: &str) {
    for (fi, face) in solid.shell().faces().iter().enumerate() {
        for w in face.wires() {
            for ed in w.edges() {
                let Some(c) = ed.curve() else { continue };
                let s = ed.start().point();
                let t = ed.end().point();
                let ds = c.point(ed.first()).distance(&s);
                let dt = c.point(ed.last()).distance(&t);
                assert!(
                    ds <= 1.0e-6 && dt <= 1.0e-6,
                    "{label}: face {fi} edge [{:.4},{:.4}] is param/vertex inconsistent \
                     (start err {ds:.5}, end err {dt:.5}, start ({:.2},{:.2},{:.2}), end ({:.2},{:.2},{:.2}))",
                    ed.first(),
                    ed.last(),
                    s.x(),
                    s.y(),
                    s.z(),
                    t.x(),
                    t.y(),
                    t.z()
                );
            }
        }
    }
}

#[test]
fn circular_bite_fillet_edges_are_param_consistent() {
    let base = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let axis = Ax2::new(Pnt::new(20.0, 8.0, -0.25), Dir::dz());
    let cyl = make_cylinder(&axis, 14.0, 10.5);
    let body = boolean(&base, &cyl, BooleanOp::Cut);
    assert_edges_param_consistent(&body, "bite body");

    let x = 20.0 - (14.0_f64 * 14.0 - 3.0_f64 * 3.0).sqrt();
    let edge = Edge::between_points(Pnt::new(0.0, 5.0, 10.0), Pnt::new(x, 5.0, 10.0));
    let filleted =
        fillet_edges(&body, std::slice::from_ref(&edge), 3.0).expect("bite fillet succeeds");
    assert_edges_param_consistent(&filleted, "bite fillet");
}
