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
use openrcad_foundation::{Ax2, Dir, Pnt, ToleranceContext, TolerancePolicy};
use openrcad_geom::{Curve as _, GeomSurface, Surface as _};
use openrcad_primitives::{make_box, make_box_operation_with_policy, make_cylinder};
use openrcad_topo::{Edge, Face, Orientation, Solid};

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
    match fillet_edges(&body, std::slice::from_ref(&edge), 3.0) {
        Ok(filleted) => {
            assert!(filleted.is_watertight());
            assert!(filleted.health_report().is_healthy());
            assert_edges_param_consistent(&filleted, "bite fillet");
        }
        Err(error) => {
            assert!(!error.to_string().is_empty());
            assert!(body.is_watertight() && body.health_report().is_healthy());
        }
    }
}

#[test]
fn shell_rebuilt_coedges_preserve_raw_and_oriented_parameters() {
    let policy = &TolerancePolicy::STANDARD;
    let source = make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, policy)
        .expect("source box")
        .value;
    let top = highest_positive_z_face(&source);
    let shelled = openrcad_algo::shell_solid_operation_with_policy(&source, 0.75, &[top], policy)
        .expect("primitive Shell operation")
        .value;

    assert_coedges_param_and_pcurve_consistent(&shelled, "primitive Shell");
}

fn assert_coedges_param_and_pcurve_consistent(solid: &Solid, label: &str) {
    let context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[solid.bounding_box()],
        None,
        1.0,
    )
    .expect("valid test tolerance context");
    let mut reversed = 0usize;
    for (face_index, face) in solid.shell().faces().iter().enumerate() {
        let surface = face.surface().expect("Shell result face has a support");
        for wire in face.wires() {
            for (coedge_index, edge) in wire.edges().into_iter().enumerate() {
                let tolerance = edge
                    .tolerance()
                    .max(context.policy.pcurve_consistency)
                    .max(context.convergence)
                    * 16.0;
                let raw_start = edge.start().point();
                let raw_end = edge.end().point();
                if let Some(curve) = edge.curve() {
                    assert!(
                        curve.point(edge.first()).distance(&raw_start) <= tolerance,
                        "{label}: face {face_index} coedge {coedge_index} raw first/start mismatch"
                    );
                    assert!(
                        curve.point(edge.last()).distance(&raw_end) <= tolerance,
                        "{label}: face {face_index} coedge {coedge_index} raw last/end mismatch"
                    );
                }

                let pcurve = wire
                    .pcurve(coedge_index)
                    .expect("operation result has a pcurve per coedge");
                let lift = |fraction| {
                    let uv = pcurve.point_at_fraction(fraction);
                    surface.point(uv.x(), uv.y())
                };
                assert!(
                    lift(0.0).distance(&raw_start) <= tolerance,
                    "{label}: face {face_index} coedge {coedge_index} raw pcurve start mismatch"
                );
                assert!(
                    lift(1.0).distance(&raw_end) <= tolerance,
                    "{label}: face {face_index} coedge {coedge_index} raw pcurve end mismatch"
                );

                let (source_fraction, target_fraction) =
                    if edge.orientation() == Orientation::Reversed {
                        reversed += 1;
                        (1.0, 0.0)
                    } else {
                        (0.0, 1.0)
                    };
                assert!(
                    lift(source_fraction).distance(&edge.source().point()) <= tolerance,
                    "{label}: face {face_index} coedge {coedge_index} oriented source mismatch"
                );
                assert!(
                    lift(target_fraction).distance(&edge.target().point()) <= tolerance,
                    "{label}: face {face_index} coedge {coedge_index} oriented target mismatch"
                );
            }
        }
    }
    assert!(
        reversed > 0,
        "{label}: fixture must exercise at least one reversed coedge"
    );
}

fn highest_positive_z_face(solid: &Solid) -> Face {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Plane(plane)) if plane.normal().z() > 0.9 => {
                Some((plane.location().z(), face))
            }
            _ => None,
        })
        .max_by(|(first, _), (second, _)| first.total_cmp(second))
        .expect("positive-z planar face")
        .1
}
