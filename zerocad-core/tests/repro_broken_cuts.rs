//! `broken-cuts.zcad`: a through hole (inner circle of a pair of concentric
//! circles) followed by a counterbore ring cut whose inner wall coincides with
//! the hole's wall. The ring tool's axis is anti-parallel to the hole's and its
//! rim arcs are split at different angles, which used to leave a kept tool wall
//! piece / a sliver face and a rejected (non-manifold) boolean.

use std::collections::HashSet;
use zerocad_core::{read_document_from_slice, LoadOptions};

#[test]
fn counterbore_ring_around_through_hole_applies() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/broken-cuts.zcad"
    );
    let bytes = std::fs::read(path).unwrap();
    let loaded = read_document_from_slice(&bytes, &LoadOptions::default()).expect("parse");
    let (bodies, warnings) = loaded
        .document
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("evaluate");
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(bodies.len(), 1);
    // L-shaped plate (4584.4) minus the through hole (~117.4) minus the ring
    // counterbore (annulus area ~17.6 x 12.7 deep ~ 223.7).
    let volume = bodies[0].1.mass_properties().expect("closed body").volume;
    assert!((4200.0..4300.0).contains(&volume), "volume {volume}");
}
