//! Chained-boolean invariants, isolated from any sketch or text pipeline.
//!
//! Discovered through text emboss: engraving a two-letter string removed about
//! 2.5x the material its tools occupy, while raising the same string added
//! exactly the sum of its parts. These fixtures reproduce both directions with
//! plain boxes so the fix can be developed and verified in the kernel.

use openrcad_algo::{boolean_checked, BooleanOp};
use openrcad_foundation::{Pnt, Trsf, Vec as GeomVec};
use openrcad_primitives::make_box;
use openrcad_topo::Solid;

/// Signed volume by the divergence theorem over the tessellated boundary.
fn volume(solid: &Solid) -> f64 {
    let mesh = openrcad_mesh::tessellate(solid, 0.05, 0.35);
    let mut six_v = 0.0;
    for [a, b, c] in &mesh.triangles {
        let (p, q, r) = (
            mesh.vertices[*a as usize],
            mesh.vertices[*b as usize],
            mesh.vertices[*c as usize],
        );
        six_v += p.x() * (q.y() * r.z() - q.z() * r.y()) - p.y() * (q.x() * r.z() - q.z() * r.x())
            + p.z() * (q.x() * r.y() - q.y() * r.x());
    }
    (six_v / 6.0).abs()
}

fn stud(x: f64) -> Solid {
    make_box(&Pnt::origin(), 4.0, 4.0, 2.0)
        .transformed(&Trsf::translation(GeomVec::new(x, 6.0, 4.0)))
}

fn plate() -> Solid {
    make_box(&Pnt::origin(), 60.0, 20.0, 5.0)
}

/// Chained union is correct: two disjoint studs add exactly their own volume.
#[test]
fn chained_union_accumulates_every_tool() {
    let base = plate();
    let start = volume(&base);
    let one = boolean_checked(&base, &stud(6.0), BooleanOp::Fuse).expect("first fuse");
    let two = boolean_checked(&one, &stud(20.0), BooleanOp::Fuse).expect("second fuse");

    // Each stud stands 1 mm proud of the 5 mm plate, so each adds 4 x 4 x 1.
    let added = volume(&two) - start;
    assert!(
        (added - 32.0).abs() <= 1.0,
        "two studs should add 32 mm3, added {added}"
    );
}

/// Chained difference is correct too: two cuts remove exactly their overlap.
///
/// Kept alongside the union case because a text-emboss discrepancy was
/// initially misattributed to chained booleans. Both directions are sound; the
/// fault was a too-thin tool overlap in the caller.
#[test]
fn chained_difference_removes_only_its_tools() {
    let base = plate();
    let start = volume(&base);
    let one = boolean_checked(&base, &stud(6.0), BooleanOp::Cut).expect("first cut");
    let two = boolean_checked(&one, &stud(20.0), BooleanOp::Cut).expect("second cut");

    let removed = start - volume(&two);
    assert!(
        (removed - 32.0).abs() <= 1.0,
        "two studs should remove 32 mm3, removed {removed}"
    );
}
