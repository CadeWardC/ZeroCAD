//! Counterbore ring whose inner wall is (to f32 noise) the through hole's own
//! cylinder, with the tool axis parallel and anti-parallel to the hole's.
use openrcad::foundation::{Ax2, Dir, Pnt};
use openrcad::primitives::{
    make_box_operation as make_box, make_cylinder_operation as make_cylinder,
};
use zerocad_core::mock_kernel::difference;

fn run(r: f64, big: f64, eps: f64, ox: f64, oy: f64, flip: bool) -> Result<(), String> {
    let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 30.0, 20.0, 22.8)
        .unwrap()
        .value;
    let hole = make_cylinder(&Ax2::new(Pnt::new(ox, oy, -1.0), Dir::dz()), r, 25.0)
        .unwrap()
        .value;
    let drilled = difference(&block, &hole).ok_or("hole failed")?;
    let (z0, dir) = if flip {
        (22.8, Dir::new(0.0, 0.0, -1.0))
    } else {
        (10.1, Dir::dz())
    };
    let h = if flip { 12.7 } else { 13.0 };
    let outer = make_cylinder(&Ax2::new(Pnt::new(ox, oy, z0), dir), big, h)
        .unwrap()
        .value;
    let inner = make_cylinder(
        &Ax2::new(Pnt::new(ox, oy, if flip { 23.0 } else { 9.0 }), dir),
        r + eps,
        16.0,
    )
    .unwrap()
    .value;
    let ring = difference(&outer, &inner).ok_or("ring failed")?;
    let parts = zerocad_core::mock_kernel::difference_bodies(&drilled, &ring).ok_or("cb failed")?;
    if parts.len() != 1 {
        return Err(format!("parts {}", parts.len()));
    }
    if !parts[0].health_report().is_healthy() {
        return Err("unhealthy".into());
    }
    parts[0]
        .validate_strict_with_policy(&Default::default())
        .map_err(|e| format!("{e:?}"))
}

#[test]
fn ring_cut_with_inner_wall_on_existing_hole() {
    let mut bad = vec![];
    for flip in [false, true] {
        for eps in [
            0.0, 3.6e-7, -3.6e-7, 1e-6, -1e-6, 2e-6, -2e-6, 5e-6, -5e-6, 9e-6, -9e-6, 1e-5, -1e-5,
            2e-5, -2e-5, 1e-4, -1e-4,
        ] {
            for (r, big, ox, oy) in [
                (1.2806249_f64, 2.6907248, 11.8, 7.55),
                (1.0, 2.5, 5.0, 5.0),
                (3.0, 4.0, 10.0, 10.0),
            ] {
                if let Err(e) = run(r, big, eps, ox, oy, flip) {
                    bad.push(format!("flip={flip} eps={eps} r={r}: {e}"));
                }
            }
        }
    }
    // Within the classification tolerance (1e-5) the walls are one surface and
    // every cut must apply; f32 sketch noise is ~2e-7..1e-6. Beyond it the
    // faces are a genuine sub-micron sliver pair, which may be refused (clean
    // `None`) but must never come back unhealthy.
    bad.retain(|line| {
        let eps: f64 = line
            .split("eps=")
            .nth(1)
            .and_then(|s| s.split(' ').next())
            .unwrap()
            .parse()
            .unwrap();
        eps.abs() <= 9e-6 || !line.ends_with("cb failed")
    });
    assert!(bad.is_empty(), "{:#?}", bad);
}

/// Two same-radius blind holes from opposite faces whose spans overlap: the
/// second tool's wall coincides with the first bore's over the overlap, and
/// the bore ends sit inside the tool (census coaxial chain 430).
fn overlapping_blind_holes(
    r: f64,
    eps: f64,
    dir_a_up: bool,
    dir_b_up: bool,
    phase_a: f64,
    phase_b: f64,
    shift: (f64, f64),
) -> Result<(), String> {
    let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 61.2, 42.9, 10.11)
        .unwrap()
        .value;
    let cyl = |z: f64, up: bool, len: f64, r: f64| {
        let dir = if up {
            Dir::dz()
        } else {
            Dir::new(0.0, 0.0, -1.0)
        };
        make_cylinder(&Ax2::new(Pnt::new(30.0, 20.0, z), dir), r, len)
            .unwrap()
            .value
    };
    // Rotating about the bore's own axis changes only where the seam sits.
    let spin = |solid: openrcad::topo::Solid, phase: f64| {
        let axis = openrcad::foundation::Ax1::new(Pnt::new(30.0, 20.0, 0.0), Dir::dz());
        solid.transformed(&openrcad::foundation::Trsf::rotation(&axis, phase))
    };
    let a = if dir_a_up {
        cyl(-2.0, true, 9.91, r)
    } else {
        cyl(7.91, false, 9.91, r)
    };
    let b = if dir_b_up {
        cyl(3.83, true, 8.28, r + eps)
    } else {
        cyl(12.11, false, 8.28, r + eps)
    };
    let b = b.transformed(&openrcad::foundation::Trsf::translation(
        openrcad::foundation::Vec::new(shift.0, shift.1, 0.0),
    ));
    let (a, b) = (spin(a, phase_a), spin(b, phase_b));
    let first = difference(&block, &a).ok_or("first failed")?;
    let parts = zerocad_core::mock_kernel::difference_bodies(&first, &b).ok_or("second failed")?;
    if parts.len() != 1 {
        return Err(format!("parts {}", parts.len()));
    }
    parts[0]
        .validate_strict_with_policy(&Default::default())
        .map_err(|e| format!("{e:?}"))
}

#[test]
fn overlapping_coaxial_blind_holes_apply() {
    let mut bad = vec![];
    for (a, b) in [(true, false), (true, true), (false, false), (false, true)] {
        for eps in [0.0, 3.6e-7, -3.6e-7, 1e-6, -1e-6, 5e-6, -5e-6] {
            for (pa, pb) in [(0.0, 0.0), (0.0, 1.0), (0.7, 0.0), (2.1, 4.4), (3.9, 0.3)] {
                for shift in [(0.0, 0.0), (2e-6, 0.0), (0.0, -3e-6), (-2e-6, 2e-6)] {
                    if let Err(e) =
                        overlapping_blind_holes(3.794306573644894, eps, a, b, pa, pb, shift)
                    {
                        bad.push(format!(
                            "a_up={a} b_up={b} eps={eps} phases=({pa},{pb}) shift={shift:?}: {e}"
                        ));
                    }
                }
            }
        }
    }
    assert!(bad.is_empty(), "{:#?}", bad);
}

/// Census coaxial chain 430: a bore from below, a narrower boss starting just
/// inside the bore's closed end (a stub hanging in the void, the rest buried in
/// material), then a same-radius blind hole from above overlapping the bore.
fn bore_boss_then_overlapping_hole(
    phases: (f64, f64, f64),
    shift: (f64, f64),
) -> Result<(), String> {
    let spin = |solid: openrcad::topo::Solid, phase: f64, dx: f64, dy: f64| {
        let axis = openrcad::foundation::Ax1::new(Pnt::new(30.0, 20.0, 0.0), Dir::dz());
        solid
            .transformed(&openrcad::foundation::Trsf::rotation(&axis, phase))
            .transformed(&openrcad::foundation::Trsf::translation(
                openrcad::foundation::Vec::new(dx, dy, 0.0),
            ))
    };
    let cyl = |z: f64, up: bool, len: f64, r: f64| {
        let dir = if up {
            Dir::dz()
        } else {
            Dir::new(0.0, 0.0, -1.0)
        };
        make_cylinder(&Ax2::new(Pnt::new(30.0, 20.0, z), dir), r, len)
            .unwrap()
            .value
    };
    let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 61.2, 42.9, 10.11)
        .unwrap()
        .value;
    let r = 3.794306573644894;
    let bore = spin(cyl(7.91, false, 9.91, r), phases.0, 0.0, 0.0);
    let boss = spin(cyl(7.616, true, 4.94, 2.6256), phases.1, shift.0, shift.1);
    let top = spin(cyl(12.11, false, 8.28, r), phases.2, -shift.0, shift.1);
    let drilled = difference(&block, &bore).ok_or("bore failed")?;
    let with_boss = zerocad_core::mock_kernel::union(&drilled, &boss).ok_or("boss failed")?;
    let parts =
        zerocad_core::mock_kernel::difference_bodies(&with_boss, &top).ok_or("top hole failed")?;
    // The boss tip above the top cutter's end (z 12.11..12.56) is a floating disc.
    if parts.len() != 2 {
        return Err(format!("parts {}", parts.len()));
    }
    for part in &parts {
        part.validate_strict_with_policy(&Default::default())
            .map_err(|e| format!("{e:?}"))?;
    }
    Ok(())
}

#[test]
fn bore_boss_then_overlapping_hole_applies() {
    let mut bad = vec![];
    for phases in [
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 2.0),
        (0.3, 4.0, 2.2),
        (-2.9, 0.7, 1.9),
        (5.5, 5.5, 0.1),
    ] {
        for shift in [(0.0, 0.0), (2e-6, 0.0), (0.0, -3e-6), (-2e-6, 2e-6)] {
            if let Err(e) = bore_boss_then_overlapping_hole(phases, shift) {
                bad.push(format!("phases={phases:?} shift={shift:?}: {e}"));
            }
        }
    }
    assert!(bad.is_empty(), "{:#?}", bad);
}

/// A wider coaxial cylinder whose end cap slices a narrower cylinder leaves its
/// tip as a free disc; the seam phases must not matter.
#[test]
fn wider_coaxial_cutter_ending_mid_boss_leaves_the_tip() {
    let mut bad = vec![];
    for (pa, pb) in [(0.0, 0.0), (0.0, 1.0), (1.0, 2.0), (4.0, 0.3), (-2.9, 1.9)] {
        let axis = openrcad::foundation::Ax1::new(Pnt::new(30.0, 20.0, 0.0), Dir::dz());
        let spin = |s: openrcad::topo::Solid, a: f64| {
            s.transformed(&openrcad::foundation::Trsf::rotation(&axis, a))
        };
        let boss = make_cylinder(&Ax2::new(Pnt::new(30.0, 20.0, 7.6), Dir::dz()), 2.6, 4.9)
            .unwrap()
            .value;
        let tool = make_cylinder(&Ax2::new(Pnt::new(30.0, 20.0, 3.8), Dir::dz()), 3.79, 8.3)
            .unwrap()
            .value;
        match zerocad_core::mock_kernel::difference_bodies(&spin(boss, pa), &spin(tool, pb)) {
            None => bad.push(format!("phases ({pa},{pb}) failed")),
            Some(parts) if parts.len() != 1 => {
                bad.push(format!("phases ({pa},{pb}) parts {}", parts.len()))
            }
            Some(parts) => {
                if let Err(e) = parts[0].validate_strict_with_policy(&Default::default()) {
                    bad.push(format!("phases ({pa},{pb}) invalid {e:?}"));
                }
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// Census coaxial chain 248: a boss sticking out of the block is cut through
/// below its tip (the tip floats), then a same-radius through cut ends exactly
/// on the tip's bottom face (coplanar contact, coincident rim).
#[test]
fn through_cut_ending_on_a_floating_tip_applies() {
    let r = 3.699215868332817;
    let mut bad = vec![];
    for phases in [
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 2.0),
        (0.3, 4.0, 2.2),
        (-2.9, 0.7, 1.9),
        (5.5, 5.5, 0.1),
    ] {
        for shift in [(0.0, 0.0), (2e-6, 0.0), (0.0, -3e-6), (-2e-6, 2e-6)] {
            let run = || -> Result<(), String> {
                let spin = |s: openrcad::topo::Solid, a: f64, d: (f64, f64)| {
                    let axis = openrcad::foundation::Ax1::new(Pnt::new(30.0, 20.0, 0.0), Dir::dz());
                    s.transformed(&openrcad::foundation::Trsf::rotation(&axis, a))
                        .transformed(&openrcad::foundation::Trsf::translation(
                            openrcad::foundation::Vec::new(d.0, d.1, 0.0),
                        ))
                };
                let cyl = |z: f64, len: f64| {
                    make_cylinder(&Ax2::new(Pnt::new(30.0, 20.0, z), Dir::dz()), r, len)
                        .unwrap()
                        .value
                };
                let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 68.7, 34.7, 8.36)
                    .unwrap()
                    .value;
                let boss = spin(cyl(6.496, 4.64), phases.0, (0.0, 0.0));
                let first = spin(cyl(4.76, 5.6), phases.1, shift);
                let second = spin(cyl(-2.0, 12.36), phases.2, (-shift.0, shift.1));
                let body = zerocad_core::mock_kernel::union(&block, &boss).ok_or("boss failed")?;
                let parts = zerocad_core::mock_kernel::difference_bodies(&body, &first)
                    .ok_or("first cut failed")?;
                if parts.len() != 2 {
                    return Err(format!("first parts {}", parts.len()));
                }
                // The cut acts on the main part; the tip is untouched.
                let volume = |s: &openrcad::topo::Solid| {
                    zerocad_core::MockMesh::from_solid(s)
                        .mass_properties()
                        .map_or(0.0, |m| m.volume)
                };
                let main = parts
                    .iter()
                    .max_by(|a, b| volume(a).partial_cmp(&volume(b)).unwrap())
                    .unwrap();
                let parts = zerocad_core::mock_kernel::difference_bodies(main, &second)
                    .ok_or("second cut failed")?;
                if parts.len() != 1 {
                    return Err(format!("second parts {}", parts.len()));
                }
                parts[0]
                    .validate_strict_with_policy(&Default::default())
                    .map_err(|e| format!("{e:?}"))
            };
            if let Err(e) = run() {
                bad.push(format!("phases={phases:?} shift={shift:?}: {e}"));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// Census coaxial chain 131 (reduced): two same-radius blind holes from
/// opposite faces that do not meet (a solid web between them).
#[test]
fn separate_coaxial_blind_holes_leave_the_web() {
    let r = 7.943611877334662;
    let mut bad = vec![];
    for (pa, pb) in [(0.0, 0.0), (0.0, 1.0), (2.2, 5.1), (-2.9, 0.7)] {
        let spin = |s: openrcad::topo::Solid, a: f64| {
            let axis = openrcad::foundation::Ax1::new(Pnt::new(28.5, 22.9, 0.0), Dir::dz());
            s.transformed(&openrcad::foundation::Trsf::rotation(&axis, a))
        };
        let cyl = |z: f64, len: f64| {
            make_cylinder(&Ax2::new(Pnt::new(28.5, 22.9, z), Dir::dz()), r, len)
                .unwrap()
                .value
        };
        let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 57.02, 45.88, 11.667)
            .unwrap()
            .value;
        let top = spin(cyl(7.8385, 5.8287), pa);
        let bottom = spin(cyl(-2.0, 6.7674), pb);
        let result = zerocad_core::mock_kernel::difference_bodies(&block, &top)
            .and_then(|parts| zerocad_core::mock_kernel::difference_bodies(&parts[0], &bottom));
        match result {
            None => bad.push(format!("({pa},{pb}) failed")),
            Some(parts) => {
                let volume: f64 = parts
                    .iter()
                    .map(|p| {
                        zerocad_core::MockMesh::from_solid(p)
                            .mass_properties()
                            .map_or(0.0, |m| m.volume)
                    })
                    .sum();
                let expected = 57.02 * 45.88 * 11.667
                    - std::f64::consts::PI * r * r * (11.667 - 7.8385)
                    - std::f64::consts::PI * r * r * (4.7674);
                if parts.len() != 1 || (volume - expected).abs() > 1e-3 * expected {
                    bad.push(format!(
                        "({pa},{pb}) parts {} volume {volume} expected {expected}",
                        parts.len()
                    ));
                }
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
