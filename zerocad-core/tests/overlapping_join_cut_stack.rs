//! Long histories of overlapping parallel-prism joins and cuts, checked against
//! an analytic CSG membership oracle at checkpoints along the history.
#[path = "support/csg_oracle.rs"]
mod csg_oracle;

use csg_oracle::{judge, Lattice, Profile, Step};
use zerocad_core::ExtrudeMode;

fn stack() -> Vec<Step> {
    use ExtrudeMode::{Cut, Join, NewBody};
    let rect = |min, max| Profile::Rect { min, max };
    let circle = |center, radius| Profile::Circle { center, radius };
    vec![
        Step::along_z("base block", rect((0., 0.), (60., 40.)), 0., 10., NewBody),
        Step::along_z(
            "J1 overhang, flush on base top",
            rect((40., 10.), (80., 30.)),
            10.,
            8.,
            Join,
        ),
        Step::along_z(
            "J2 cylinder through base side and J1",
            circle((70., 20.), 12.),
            0.,
            14.,
            Join,
        ),
        Step::along_z(
            "J3 mid-height box crossing base edge and J1",
            rect((20., -10.), (50., 15.)),
            5.,
            10.,
            Join,
        ),
        Step::along_z(
            "J4 cylinder overlapping J3",
            circle((35., -5.), 8.),
            0.,
            20.,
            Join,
        ),
        Step::along_z(
            "C1 through hole crossing J1, J3 and base",
            circle((42., 12.), 7.),
            -2.,
            30.,
            Cut,
        ),
        Step::along_z(
            "C2 slot through J2, J1 and base",
            rect((5., 18.), (85., 22.)),
            -2.,
            30.,
            Cut,
        ),
        Step::along_z(
            "C3 pocket from below crossing J2 boundary",
            circle((60., 30.), 10.),
            -2.,
            8.,
            Cut,
        ),
        Step::along_z(
            "C4 hole overlapping C1 and J4",
            circle((36., 2.), 6.),
            -2.,
            30.,
            Cut,
        ),
        Step::along_z(
            "J5 refill across slot and holes",
            rect((30., 5.), (45., 35.)),
            0.,
            12.,
            Join,
        ),
        Step::along_z(
            "C5 slab splitting the part",
            rect((55., -20.), (58., 60.)),
            -2.,
            30.,
            Cut,
        ),
    ]
}

/// Checkpoints instead of every prefix keep the debug run affordable: a failed
/// feature's warning already names its step, and each checkpoint precedes a
/// feature that could mask an earlier wrong result (J5 refills C1/C4 material).
#[test]
fn overlapping_join_cut_stack_matches_csg_oracle() {
    let steps = stack();
    // Offsets keep the lattice off the integer-aligned primitive boundaries.
    let lattice = Lattice {
        origin: [-12.37, -14.29, -3.13],
        spacing: [3.29, 3.43, 3.61],
        counts: [32, 20, 9],
    };
    // Until C5 the base strip at x < 5 bridges the C2 slot. C5 severs x 55..58,
    // leaving the left side (still bridged, also by J5) plus the right side,
    // which the slot splits into lumps below and above it.
    for (count, expected_parts, stage) in [
        (5, 1, "joins only"),
        (9, 1, "before the J5 refill"),
        (steps.len(), 3, "full stack"),
    ] {
        let outcome = judge(&steps[..count], lattice);
        let rejected: Vec<_> = outcome
            .rejected
            .iter()
            .map(|&index| steps[index].label)
            .collect();
        assert!(
            rejected.is_empty() && outcome.unattributed.is_empty() && outcome.problems.is_empty(),
            "{stage} ({count} steps): rejected {rejected:?}, other warnings {:?}\n    {}",
            outcome.unattributed,
            outcome.problems.join("\n    ")
        );
        assert_eq!(outcome.parts, expected_parts, "{stage}: part count");
    }
}
