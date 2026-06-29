//! Tracking test for `fillet_problem.zcad` (Issue B): filleting a top edge of an
//! extruded sketch.
//!
//! Two layered causes were found and the first is fixed: the arc-shaped wall the
//! fillet's end runs into was built as a `Ruled` surface (the prism's
//! arc-to-cylinder test used a 1e-8 rad tolerance, too tight for an arc fitted
//! from f32 sketch samples), which the rolling-ball trim rejected with
//! `NotPlaneOrAnalytic`. That wall is now a clean `Cylinder` (see
//! `prism::tests::slightly_tilted_circle_axis_still_makes_a_cylinder`), and the
//! corner trim now preserves arc sub-edges instead of straightening them.
//!
//! The second cause is now covered by OpenRCAD's synthetic
//! `fillet_planar_edge_runs_out_into_tangent_curved_wall` regression: the
//! filleted edge terminates at a curved wall tangent to one of the fillet's own
//! faces, and the endpoint trim must lie on that wall's analytic cylinder.
//!
//! Skips gracefully if the model file isn't present (it lives in the repo root
//! and may not be committed).

use std::collections::HashSet;
use zerocad_core::read_zcad;

#[test]
#[ignore = "optional local fillet_problem.zcad fixture; synthetic tangent-wall runout regression covers this path"]
fn fillet_problem_zcad_fillets_without_warning() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../fillet_problem.zcad");
    let Ok(bytes) = std::fs::read(path) else {
        eprintln!("skipping: {path} not found");
        return;
    };
    let loaded = read_zcad(&bytes).expect("parse .zcad");

    let (_bodies, warnings) = loaded
        .graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate model");

    let fillet_warning = warnings.iter().find(|w| w.contains("couldn't be rounded"));
    assert!(
        fillet_warning.is_none(),
        "the fillet must apply now, but got warning: {fillet_warning:?}"
    );
}
