//! A face wider than half a turn must classify points near both of its ends.
//! Found by the ZeroCAD join/cut census (chain 45): a small bore straddling a
//! large bore's wall leaves that wall as one cylinder face about 335° wide. A
//! block fused over both bores then cuts the wall's end in a 0.05 mm arc,
//! whose midpoint the face-containment test put on the wrong 2π branch (it
//! aligned angles to the mean of the boundary samples, which for so wide a
//! face lies more than π from its ends). The arc was dropped, the block's
//! floor was left with a dangling chain, and the fuse had free edges.
use openrcad_algo::{boolean::point_in_solid, boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};

#[test]
fn block_fused_over_a_wide_merged_bore_wall() {
    let (c1, r1) = ((29.152794589622, 14.770213923243), 6.910074129631);
    let (c2, r2) = ((28.984300341871, 20.062515776044), 2.064321687118);
    let bore = |(x, y): (f64, f64), r: f64| {
        make_cylinder_operation(&Ax2::new(Pnt::new(x, y, -2.0), Dir::dz()), r, 30.0)
            .expect("bore")
            .value
    };
    let (block_x, block_y, block_z) = (49.951870316218, 45.196999274986, 9.784224106657);
    let mut solid = make_box_operation(&Pnt::origin(), block_x, block_y, block_z)
        .expect("block")
        .value;
    for (name, tool) in [("large bore", bore(c1, r1)), ("small bore", bore(c2, r2))] {
        solid = boolean_operation(&solid, &tool, BooleanOp::Cut)
            .unwrap_or_else(|error| panic!("{name}: {error:?}"))
            .value;
    }
    // The block stops 0.05 mm inside the large bore's far side and rises
    // from inside the material through the top face.
    let (lo, hi) = (
        (8.238627495027, 8.307903653298, 1.063804174639),
        (30.448905013841, 37.859323827583, 10.359476433857),
    );
    let block = make_box_operation(
        &Pnt::new(lo.0, lo.1, lo.2),
        hi.0 - lo.0,
        hi.1 - lo.1,
        hi.2 - lo.2,
    )
    .expect("block")
    .value;
    let fused = boolean_operation(&solid, &block, BooleanOp::Fuse)
        .expect("the block fuses over the bores")
        .value;
    fused
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("valid solid");

    // Material oracle on a lattice kept clear of every boundary.
    let in_bore =
        |x: f64, y: f64| (x - c1.0).hypot(y - c1.1) < r1 || (x - c2.0).hypot(y - c2.1) < r2;
    let mut checked = 0;
    for i in 0..26 {
        for j in 0..24 {
            for k in 0..6 {
                let p = (
                    0.913 + 1.9 * i as f64,
                    0.871 + 1.9 * j as f64,
                    0.517 + 1.9 * k as f64,
                );
                let in_base = p.0 < block_x && p.1 < block_y && p.2 < block_z && !in_bore(p.0, p.1);
                let in_block = (lo.0..hi.0).contains(&p.0)
                    && (lo.1..hi.1).contains(&p.1)
                    && (lo.2..hi.2).contains(&p.2);
                let near_wall = ((p.0 - c1.0).hypot(p.1 - c1.1) - r1).abs() < 0.05
                    || ((p.0 - c2.0).hypot(p.1 - c2.1) - r2).abs() < 0.05;
                if near_wall {
                    continue;
                }
                checked += 1;
                assert_eq!(
                    point_in_solid(&Pnt::new(p.0, p.1, p.2), &fused),
                    in_base || in_block,
                    "material at {p:?}"
                );
            }
        }
    }
    assert!(checked > 3000);
}
