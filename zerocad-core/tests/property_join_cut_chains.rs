//! Randomized chains of overlapping prism joins and cuts, judged against an
//! analytic CSG oracle.
//!
//! Soundness is strict for every chain: whatever the model reports, its solids
//! must be valid and equal the oracle with rejected steps left out. For
//! generic placements (continuous coordinates, so no coincident faces) it must
//! also be complete: a step may be rejected only when its tool shares no
//! volume with the material present, i.e. a join into space or a cut in air.
//! Side cuts sweep along Y, so they cross the Z-prism record and exercise the
//! general boolean path; the rest exercise the exact sectional path.
//!
//! Every run replays the saved seeds in `property_join_cut_chains.proptest-regressions`
//! plus 2 fresh cases per property; set `PROPTEST_CASES` (e.g. 60) for a real sweep.
#[path = "support/csg_oracle.rs"]
mod csg_oracle;

use csg_oracle::{judge, tool_overlaps_material, Axis, Lattice, Profile, Step};
use proptest::prelude::*;
use zerocad_core::ExtrudeMode;

/// One step's kind and six unit parameters, mapped onto the block by
/// [`to_step`]. Plain numbers keep shrinking effective.
type RawStep = (u8, [f64; 6]);

#[derive(Clone, Copy, Debug)]
struct Block {
    width: f64,
    depth: f64,
    height: f64,
}

fn to_step(block: Block, (kind, p): RawStep, snap: bool) -> Step {
    use ExtrudeMode::{Cut, Join};
    let Block {
        width: w,
        depth: d,
        height: h,
    } = block;
    // Degenerate chains snap to whole millimetres, so faces, edges, and
    // circle tangencies coincide with the block and with each other.
    let q = |value: f64| if snap { value.round() } else { value };
    let center = (q(w * (0.1 + 0.8 * p[0])), q(d * (0.1 + 0.8 * p[1])));
    let rect = |half: (f64, f64)| Profile::Rect {
        min: (center.0 - q(half.0), center.1 - q(half.1)),
        max: (center.0 + q(half.0), center.1 + q(half.1)),
    };
    let circle = |radius: f64| Profile::Circle {
        center,
        radius: q(radius).max(1.0),
    };
    let z_step = |label, profile, offset: f64, depth: f64, mode| Step {
        label,
        profile,
        axis: Axis::Z,
        offset: q(offset),
        depth: q(depth),
        mode,
        draft_deg: 0.0,
    };
    match kind % 6 {
        0 => z_step(
            "join box",
            rect((2.0 + 0.4 * w * p[2], 2.0 + 0.4 * d * p[3])),
            h * (0.1 + 0.8 * p[4]),
            3.0 + 10.0 * p[5],
            Join,
        ),
        1 => z_step(
            "join cylinder",
            circle(2.0 + 10.0 * p[2]),
            h * (0.1 + 0.8 * p[4]),
            3.0 + 10.0 * p[5],
            Join,
        ),
        2 => z_step(
            "through slot",
            rect((1.0 + 0.3 * w * p[2], 1.0 + 0.3 * d * p[3])),
            -2.0,
            h + 20.0,
            Cut,
        ),
        3 => z_step(
            "through hole",
            circle(1.5 + 8.0 * p[2]),
            -2.0,
            h + 20.0,
            Cut,
        ),
        4 => {
            let profile = if p[4] < 0.5 {
                rect((1.0 + 0.25 * w * p[2], 1.0 + 0.25 * d * p[3]))
            } else {
                circle(1.5 + 8.0 * p[2])
            };
            // Blind pocket up from the bottom face.
            z_step("pocket", profile, -2.0, 2.0 + h * (0.2 + 0.6 * p[5]), Cut)
        }
        _ => {
            // Profile coordinates are (z, x) for a Y sweep.
            let zc = q(h * (0.2 + 0.6 * p[0]));
            let xc = q(w * (0.1 + 0.8 * p[1]));
            let profile = if p[4] < 0.5 {
                let (hz, hx) = (q(1.0 + 0.3 * h * p[2]), q(2.0 + 0.3 * w * p[3]));
                Profile::Rect {
                    min: (zc - hz, xc - hx),
                    max: (zc + hz, xc + hx),
                }
            } else {
                Profile::Circle {
                    center: (zc, xc),
                    radius: q(1.0 + 0.35 * h * p[2]).max(1.0),
                }
            };
            let depth = if p[5] < 0.5 {
                d + 20.0
            } else {
                2.0 + 0.6 * d * p[5]
            };
            Step {
                label: "side cut",
                profile,
                axis: Axis::Y,
                offset: -2.0,
                depth: q(depth),
                mode: Cut,
                draft_deg: 0.0,
            }
        }
    }
}

fn chain(block: Block, raw: &[RawStep], snap: bool) -> Vec<Step> {
    let base = Step::along_z(
        "base block",
        Profile::Rect {
            min: (0.0, 0.0),
            max: (block.width, block.depth),
        },
        0.0,
        block.height,
        ExtrudeMode::NewBody,
    );
    std::iter::once(base)
        .chain(raw.iter().map(|&step| to_step(block, step, snap)))
        .collect()
}

/// A polygon of `kind` (0 triangle, 1 L-shape, 2 hexagon) spanning
/// `half` about `center`, rotated by `turn` radians. Only the L is concave.
fn polygon(kind: usize, center: (f64, f64), half: (f64, f64), turn: f64) -> Profile {
    let (sin, cos) = turn.sin_cos();
    let place = |(x, y): (f64, f64)| {
        let (x, y) = (x * half.0, y * half.1);
        (center.0 + x * cos - y * sin, center.1 + x * sin + y * cos)
    };
    let ring = |count: usize| -> Vec<(f64, f64)> {
        (0..count)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / count as f64;
                place((angle.cos(), angle.sin()))
            })
            .collect()
    };
    let vertices = match kind {
        0 => ring(3),
        1 => [
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 0.0),
            (0.0, 0.0),
            (0.0, 1.0),
            (-1.0, 1.0),
        ]
        .map(place)
        .to_vec(),
        _ => ring(6),
    };
    Profile::polygon(&vertices)
}

/// A slot of half-length `half` along `turn` radians about `center`.
fn slot(center: (f64, f64), half: f64, radius: f64, turn: f64) -> Profile {
    let (sin, cos) = turn.sin_cos();
    Profile::Slot {
        a: (center.0 - half * cos, center.1 - half * sin),
        b: (center.0 + half * cos, center.1 + half * sin),
        radius,
    }
}

/// Largest draft (degrees) that keeps the far section of a profile of
/// half-extents `half` swept `depth` well clear of collapsing.
fn max_draft(half: (f64, f64), depth: f64) -> f64 {
    (0.3 * half.0.min(half.1) / depth.abs()).atan().to_degrees()
}

/// Fractional part of `value * scale`: a second independent-looking unit
/// parameter from one raw value, so each step keeps six raw numbers.
fn spread(value: f64, scale: f64) -> f64 {
    (value * scale).fract()
}

/// The mixed-axis step kinds (0..6, unchanged so their seeds replay) plus
/// broader coverage: X-axis cuts and bosses, polygon and slot profiles,
/// drafted bosses and pockets, and tilted sketch planes.
fn to_broad_step(block: Block, (kind, p): RawStep) -> Step {
    use std::f64::consts::{FRAC_PI_4, TAU};
    use ExtrudeMode::{Cut, Join};
    let Block {
        width: w,
        depth: d,
        height: h,
    } = block;
    let center = (w * (0.1 + 0.8 * p[0]), d * (0.1 + 0.8 * p[1]));
    let turn = TAU * spread(p[3], 11.0);
    let step = |label, profile, axis, offset, depth, mode, draft_deg| Step {
        label,
        profile,
        axis,
        offset,
        depth,
        mode,
        draft_deg,
    };
    match kind % 14 {
        kind @ 0..=5 => to_step(block, (kind, p), false),
        6 => {
            // Profile coordinates are (y, z) for an X sweep.
            let at = (d * (0.1 + 0.8 * p[0]), h * (0.2 + 0.6 * p[1]));
            let profile = match (p[4] * 3.0) as usize {
                0 => Profile::Rect {
                    min: (at.0 - 2.0 - 0.3 * d * p[2], at.1 - 1.0 - 0.3 * h * p[3]),
                    max: (at.0 + 2.0 + 0.3 * d * p[2], at.1 + 1.0 + 0.3 * h * p[3]),
                },
                1 => Profile::Circle {
                    center: at,
                    radius: 1.0 + 0.35 * h * p[2],
                },
                _ => slot(at, 2.0 + 0.3 * d * p[3], 1.0 + 0.25 * h * p[2], 0.0),
            };
            let depth = if p[5] < 0.5 {
                w + 20.0
            } else {
                2.0 + 0.6 * w * p[5]
            };
            step("x side cut", profile, Axis::X, -2.0, depth, Cut, 0.0)
        }
        7 => {
            // A boss standing out of the +X face.
            let at = (d * (0.1 + 0.8 * p[0]), h * (0.2 + 0.6 * p[1]));
            let profile = match (p[4] * 3.0) as usize {
                0 => Profile::Rect {
                    min: (at.0 - 1.0 - 0.3 * d * p[2], at.1 - 1.0 - 0.3 * h * p[3]),
                    max: (at.0 + 1.0 + 0.3 * d * p[2], at.1 + 1.0 + 0.3 * h * p[3]),
                },
                1 => Profile::Circle {
                    center: at,
                    radius: 1.0 + 0.4 * h * p[2],
                },
                _ => slot(at, 1.0 + 0.3 * d * p[3], 1.0 + 0.3 * h * p[2], 0.0),
            };
            let offset = w * (0.6 + 0.35 * p[5]);
            step(
                "x side boss",
                profile,
                Axis::X,
                offset,
                3.0 + 10.0 * spread(p[5], 7.0),
                Join,
                0.0,
            )
        }
        8 => {
            let half = (3.0 + 0.3 * w * p[2], 3.0 + 0.3 * d * spread(p[2], 7.0));
            let profile = polygon((p[4] * 3.0) as usize, center, half, turn);
            let offset = h * (0.1 + 0.8 * p[5]);
            let depth = 3.0 + 10.0 * spread(p[5], 7.0);
            step("polygon boss", profile, Axis::Z, offset, depth, Join, 0.0)
        }
        9 => {
            let profile = if p[4] < 0.75 {
                let half = (1.5 + 0.25 * w * p[2], 1.5 + 0.25 * d * spread(p[2], 7.0));
                polygon((p[4] * 4.0) as usize, center, half, turn)
            } else {
                slot(
                    center,
                    1.0 + 0.25 * w * p[2],
                    1.0 + 4.0 * spread(p[2], 7.0),
                    turn,
                )
            };
            step(
                "polygon through cut",
                profile,
                Axis::Z,
                -2.0,
                h + 20.0,
                Cut,
                0.0,
            )
        }
        10 => {
            let half = (3.0 + 0.3 * w * p[2], 3.0 + 0.3 * d * spread(p[2], 7.0));
            let profile = if p[4] < 0.5 {
                Profile::Rect {
                    min: (center.0 - half.0, center.1 - half.1),
                    max: (center.0 + half.0, center.1 + half.1),
                }
            } else {
                polygon(if p[4] < 0.75 { 0 } else { 2 }, center, half, turn)
            };
            // Mostly tapering in; a quarter flare out.
            let draft = if spread(p[5], 13.0) < 0.25 {
                -(1.0 + 7.0 * spread(p[5], 3.0))
            } else {
                1.0 + 11.0 * spread(p[5], 3.0)
            };
            let offset = h * (0.1 + 0.8 * p[5]);
            let depth = 3.0 + 10.0 * spread(p[5], 7.0);
            let draft = draft.clamp(-max_draft(half, depth), max_draft(half, depth));
            step("drafted boss", profile, Axis::Z, offset, depth, Join, draft)
        }
        11 => {
            let half = (1.5 + 0.25 * w * p[2], 1.5 + 0.25 * d * spread(p[2], 7.0));
            let profile = if p[4] < 0.5 {
                Profile::Rect {
                    min: (center.0 - half.0, center.1 - half.1),
                    max: (center.0 + half.0, center.1 + half.1),
                }
            } else {
                polygon(2, center, half, turn)
            };
            // Down from above the top face; positive draft narrows the floor.
            let draft = if spread(p[5], 13.0) < 0.25 {
                -(1.0 + 7.0 * spread(p[5], 3.0))
            } else {
                1.0 + 14.0 * spread(p[5], 3.0)
            };
            let depth = -(2.0 + h * (0.2 + 0.6 * p[5]));
            let draft = draft.clamp(-max_draft(half, depth), max_draft(half, depth));
            step(
                "drafted pocket",
                profile,
                Axis::Z,
                h + 2.0,
                depth,
                Cut,
                draft,
            )
        }
        kind => {
            // Tilted planes: through cuts (12) and leaning bosses (13).
            let axis = Axis::tilted(TAU * p[4], (10.0 + 35.0 * p[5]).to_radians().min(FRAC_PI_4));
            let height = if kind == 12 {
                h * (0.2 + 0.6 * spread(p[5], 7.0))
            } else {
                h * (0.5 + 0.4 * spread(p[5], 7.0))
            };
            let (s, t, a) = axis.split([center.0, center.1, height]);
            let at = (s, t);
            let size = 1.5 + 0.2 * w.min(d) * p[2];
            let profile = match (spread(p[4], 5.0) * 3.0) as usize {
                0 => Profile::Rect {
                    min: (at.0 - size, at.1 - size * (0.5 + p[3])),
                    max: (at.0 + size, at.1 + size * (0.5 + p[3])),
                },
                1 => Profile::Circle {
                    center: at,
                    radius: size,
                },
                _ => slot(at, size, 1.0 + 0.3 * size * p[3], turn),
            };
            if kind == 12 {
                let reach = w + d + h;
                step(
                    "tilted through cut",
                    profile,
                    axis,
                    a - reach,
                    2.0 * reach,
                    Cut,
                    0.0,
                )
            } else {
                let depth = 3.0 + 10.0 * spread(p[5], 3.0);
                step("tilted boss", profile, axis, a, depth, Join, 0.0)
            }
        }
    }
}

fn broad_chain(block: Block, raw: &[RawStep]) -> Vec<Step> {
    let mut steps = chain(block, &[], false);
    steps.extend(raw.iter().map(|&step| to_broad_step(block, step)));
    steps
}

/// Short census label for a step's profile.
fn profile_name(step: &Step) -> &'static str {
    match (step.profile, step.draft_deg != 0.0) {
        (Profile::Rect { .. }, false) => "rect",
        (Profile::Rect { .. }, true) => "drafted rect",
        (Profile::Circle { .. }, _) => "circle",
        (Profile::Polygon { .. }, false) => "polygon",
        (Profile::Polygon { .. }, true) => "drafted polygon",
        (Profile::Slot { .. }, _) => "slot",
    }
}

/// Coaxial cylinders: every step is a circle on one shared axis, so walls of
/// the same radius coincide with the existing bore (to f32 noise). Each step
/// draws its own seam phase (a rotated sketch frame) and sweep direction (a
/// frame whose normal is -Z), and the sketch round-trips radii and frames
/// through f32. `p[2]` picks the radius (the base bore or a wider/narrower
/// counterbore), `p[3]`/`p[4]` the sweep span, `p[5]` the phase.
fn coaxial_chain(block: Block, raw: &[RawStep]) -> Vec<Step> {
    use ExtrudeMode::{Cut, Join};
    let Block {
        width: w,
        depth: d,
        height: h,
    } = block;
    let center = (w * 0.5, d * 0.5);
    let base_radius = 2.5 + 0.1 * w * raw.first().map_or(0.3, |step| step.1[2]);
    let base = Step::along_z(
        "base block",
        Profile::Rect {
            min: (0.0, 0.0),
            max: (w, d),
        },
        0.0,
        h,
        ExtrudeMode::NewBody,
    );
    let steps = raw.iter().map(|&(kind, p)| {
        let phase = std::f64::consts::TAU * p[5];
        let (sin, cos) = phase.sin_cos();
        let down = p[4] < 0.5;
        let (u, v) = if down {
            ([cos, sin, 0.0], [sin, -cos, 0.0])
        } else {
            ([cos, sin, 0.0], [-sin, cos, 0.0])
        };
        let radius = match (p[2] * 3.0) as usize {
            0 => base_radius,
            1 => base_radius + 0.8 + 2.0 * p[3],
            _ => (base_radius - 0.8 - 0.4 * p[3]).max(1.0),
        };
        // World z-span of the tool, then expressed along the frame's normal.
        let (z0, z1, mode, label) = match kind % 4 {
            0 => (-2.0, h + 2.0, Cut, "coaxial through hole"),
            1 => (
                h * (0.2 + 0.6 * p[3]),
                h * (0.3 + 0.7 * p[3]) + 3.0,
                Join,
                "coaxial boss",
            ),
            2 => (
                h - 2.0 - h * 0.7 * p[3],
                h + 2.0,
                Cut,
                "coaxial top blind hole",
            ),
            _ => (-2.0, 2.0 + h * 0.8 * p[3], Cut, "coaxial bottom blind hole"),
        };
        let (offset, depth) = if down { (-z1, z1 - z0) } else { (z0, z1 - z0) };
        Step {
            label,
            profile: Profile::Circle {
                center: (
                    center.0 * u[0] + center.1 * u[1],
                    center.0 * v[0] + center.1 * v[1],
                ),
                radius,
            },
            axis: Axis::Tilted { u, v },
            offset,
            depth,
            mode,
            draft_deg: 0.0,
        }
    });
    std::iter::once(base).chain(steps).collect()
}

fn lattice(block: Block) -> Lattice {
    // Spacings are irrational-ish so snapped (integer) faces are not sampled.
    Lattice {
        origin: [-3.137, -3.291, -2.713],
        spacing: [
            (block.width + 8.0) / 23.7,
            (block.depth + 8.0) / 17.3,
            (block.height + 16.0) / 11.9,
        ],
        counts: [24, 18, 12],
    }
}

fn check_chain(
    steps: &[Step],
    lattice: Lattice,
    require_complete: bool,
) -> Result<(), TestCaseError> {
    let outcome = judge(steps, lattice);
    if !outcome.classifier_disagreements.is_empty() {
        // A kernel point-classifier fault, not a modelling error; keep it
        // visible without failing the modelling property.
        eprintln!(
            "point_in_solid disagreed with the mesh and oracle at {:?}",
            outcome.classifier_disagreements
        );
    }
    let describe = || {
        steps
            .iter()
            .enumerate()
            .map(|(index, step)| format!("  {index}: {step:?}"))
            .collect::<Vec<_>>()
            .join("\n")
    };
    prop_assert!(
        outcome.problems.is_empty(),
        "unsound result (rejected {:?}):\n    {}\nchain:\n{}",
        outcome.rejected,
        outcome.problems.join("\n    "),
        describe()
    );
    prop_assert!(
        outcome.unattributed.is_empty(),
        "unattributed warnings {:?}\nchain:\n{}",
        outcome.unattributed,
        describe()
    );
    if require_complete {
        for &index in &outcome.rejected {
            prop_assert!(
                !tool_overlaps_material(steps, &outcome.rejected, index),
                "step {index} ({}) overlaps material but was rejected: {:?}\nchain:\n{}",
                steps[index].label,
                outcome.warnings,
                describe()
            );
        }
    }
    Ok(())
}

fn block() -> impl Strategy<Value = Block> {
    (40.0..70.0_f64, 30.0..50.0_f64, 8.0..14.0_f64).prop_map(|(width, depth, height)| Block {
        width,
        depth,
        height,
    })
}

fn raw_steps(kinds: std::ops::Range<u8>) -> impl Strategy<Value = Vec<RawStep>> {
    prop::collection::vec((kinds, prop::array::uniform6(0.0..1.0_f64)), 3..=7)
}

fn config() -> ProptestConfig {
    let cases = std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(2);
    ProptestConfig {
        cases,
        max_shrink_iters: 64,
        ..ProptestConfig::default()
    }
}

proptest! {
    #![proptest_config(config())]

    /// Z-axis joins, cuts, and pockets: the exact sectional path, complete.
    #[test]
    fn generic_parallel_chains_are_exact(block in block(), raw in raw_steps(0..5)) {
        let steps = chain(block, &raw, false);
        check_chain(&steps, lattice(block), true)?;
    }

    /// Side cuts mixed in: records cross axes, so the general boolean runs.
    /// Complete as well as sound: every step whose tool overlaps material
    /// must apply (0 refusals in the 500-chain census as of 2026-10-01). The
    /// saved seeds are the former kernel-hardening worklist.
    #[test]
    fn generic_mixed_axis_chains_are_exact(block in block(), raw in raw_steps(0..6)) {
        let steps = chain(block, &raw, false);
        check_chain(&steps, lattice(block), true)?;
    }

    /// Broad coverage: X-axis cuts and bosses, polygon and slot profiles,
    /// drafted bosses and pockets, tilted sketch planes. Complete as well as
    /// sound (0 refusals in the 500-chain broad census as of 2026-10-02).
    #[test]
    fn generic_broad_chains_are_exact(block in block(), raw in raw_steps(0..14)) {
        let steps = broad_chain(block, &raw);
        check_chain(&steps, lattice(block), true)?;
    }

    /// Same-axis cylinders: walls of equal radius coincide with the existing
    /// bore up to f32 noise, from either sweep direction and any seam phase.
    /// Sound but not yet complete: the 500-chain `ZEROCAD_CENSUS_SET=coaxial`
    /// census has one wrongful refusal left (chain 373, see
    /// `.claude/wip/coaxial-chain-373.md`); make this strict when it reaches 0.
    #[test]
    fn generic_coaxial_chains_are_sound(block in block(), raw in raw_steps(0..4)) {
        let steps = coaxial_chain(block, &raw);
        check_chain(&steps, lattice(block), false)?;
    }

    /// Whole-millimetre placements make flush faces and tangencies. Edge-only
    /// contact can be a legitimate non-manifold rejection, so only soundness
    /// is required here.
    #[test]
    fn degenerate_chains_are_sound(block in block().prop_map(|b| Block {
        width: b.width.round(),
        depth: b.depth.round(),
        height: b.height.round(),
    }), raw in raw_steps(0..6)) {
        let steps = chain(block, &raw, true);
        check_chain(&steps, lattice(block), false)?;
    }
}

/// Kernel error names the census groups refusals by, most specific first.
const CENSUS_ERRORS: [&str; 9] = [
    "UvLoopSelfIntersection",
    "NonManifoldEdge",
    "FreeEdge",
    "InvalidEulerCharacteristic",
    "LoopNotContiguous",
    "EnclosedVoid",
    "non-watertight",
    "recovery not certified",
    "needs a geometry repair",
];

/// Completeness census (opt-in, never fails): with `ZEROCAD_CENSUS=<chains>`
/// set, evaluate that many deterministic chains and report every step that
/// overlaps material but was refused, grouped by the step's shape and by
/// kernel error. This is the scoreboard for the kernel boolean work.
/// `ZEROCAD_CENSUS_SET=broad` draws from the broad step kinds instead of the
/// mixed-axis ones, `coaxial` from same-axis circles (coincident bore walls); `ZEROCAD_CENSUS_DUMP=1` prints each failing chain.
#[test]
fn mixed_axis_refusal_census() {
    use proptest::strategy::ValueTree;
    use proptest::test_runner::{RngAlgorithm, TestRng, TestRunner};
    use std::collections::BTreeMap;
    let Some(chains) = std::env::var("ZEROCAD_CENSUS")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
    else {
        return;
    };
    let dump = std::env::var_os("ZEROCAD_CENSUS_DUMP").is_some();
    // `ZEROCAD_CENSUS_STEP=1` also writes every final part to STEP, reads it
    // back strictly, and compares validity and volume.
    let step_check = std::env::var_os("ZEROCAD_CENSUS_STEP").is_some();
    let mut step_failures = 0;
    // `ZEROCAD_CENSUS_ONLY=3,17` re-runs just those chains (same numbering).
    let only: Option<Vec<usize>> = std::env::var("ZEROCAD_CENSUS_ONLY").ok().map(|list| {
        list.split(',')
            .filter_map(|index| index.trim().parse().ok())
            .collect()
    });
    // `ZEROCAD_CENSUS_DROP=1,2` leaves those steps out of every chain, for
    // reducing a failing chain (step numbers are the dump's indices).
    let drop: Vec<usize> = std::env::var("ZEROCAD_CENSUS_DROP")
        .map(|list| {
            list.split(',')
                .filter_map(|index| index.trim().parse().ok())
                .collect()
        })
        .unwrap_or_default();
    let mut runner = TestRunner::new_with_rng(
        ProptestConfig::default(),
        TestRng::deterministic_rng(RngAlgorithm::ChaCha),
    );
    let set = std::env::var("ZEROCAD_CENSUS_SET").unwrap_or_default();
    let (broad, coaxial) = (set == "broad", set == "coaxial");
    let strategy = (
        block(),
        raw_steps(if broad {
            0..14
        } else if coaxial {
            0..4
        } else {
            0..6
        }),
    );
    let (mut refusals, mut unsound) = (0, 0);
    let mut by_shape: BTreeMap<String, usize> = BTreeMap::new();
    let mut by_error: BTreeMap<&str, usize> = BTreeMap::new();
    for chain_index in 0..chains {
        let (block, raw) = strategy.new_tree(&mut runner).expect("strategy").current();
        if only
            .as_ref()
            .is_some_and(|only| !only.contains(&chain_index))
        {
            continue;
        }
        let steps: Vec<Step> = if broad {
            broad_chain(block, &raw)
        } else if coaxial {
            coaxial_chain(block, &raw)
        } else {
            chain(block, &raw, false)
        }
        .into_iter()
        .enumerate()
        .filter(|(index, _)| !drop.contains(index))
        .map(|(_, step)| step)
        .collect();
        let outcome = judge(&steps, lattice(block));
        if step_check {
            for failure in step_round_trip_failures(&steps, chain_index) {
                step_failures += 1;
                eprintln!("census chain {chain_index} STEP round trip: {failure}\n{steps:#?}");
            }
        }
        if !outcome.problems.is_empty() {
            unsound += 1;
            eprintln!(
                "census chain {chain_index} UNSOUND (rejected {:?}): {:?}
{steps:#?}",
                outcome.rejected, outcome.problems
            );
            if dump {
                if let Ok(bodies) =
                    csg_oracle::build(&steps).debug_kernel_solids(&Default::default())
                {
                    for (id, parts) in &bodies {
                        for part in parts {
                            let mesh = zerocad_core::MockMesh::from_solid(part);
                            eprintln!(
                                "  body {id}: volume {:?} faces {} aabb {:?}",
                                mesh.mass_properties().map(|m| m.volume),
                                part.faces().len(),
                                zerocad_core::mock_kernel::solid_aabb(part)
                            );
                        }
                    }
                }
            }
        }
        for &index in &outcome.rejected {
            if !tool_overlaps_material(&steps, &outcome.rejected, index) {
                continue;
            }
            refusals += 1;
            let step = &steps[index];
            let other_axis_circle = steps[..index].iter().any(|earlier| {
                earlier.axis != step.axis && matches!(earlier.profile, Profile::Circle { .. })
            });
            let shape = format!(
                "{} {}{}",
                step.label,
                profile_name(step),
                if other_axis_circle {
                    " (cross-axis cylinder present)"
                } else {
                    ""
                }
            );
            *by_shape.entry(shape).or_default() += 1;
            let warning = outcome.warnings.join(" ");
            let error = CENSUS_ERRORS
                .iter()
                .find(|name| warning.contains(*name))
                .copied()
                .unwrap_or("other");
            *by_error.entry(error).or_default() += 1;
            if dump {
                eprintln!(
                    "census chain {chain_index} step {index} refused: {warning}
{steps:#?}"
                );
            }
        }
    }
    eprintln!("census: {chains} chains, {refusals} wrongful refusals, {unsound} unsound");
    if step_check {
        eprintln!("census: {step_failures} STEP round-trip failures");
    }
    for (shape, count) in &by_shape {
        eprintln!("  {count:4}  {shape}");
    }
    for (error, count) in &by_error {
        eprintln!("  {count:4}  error: {error}");
    }
}

/// Write each final part of `steps` to STEP and read it back strictly; any
/// import failure, invalid solid, or volume change is reported.
fn step_round_trip_failures(steps: &[Step], chain_index: usize) -> Vec<String> {
    let graph = csg_oracle::build(steps);
    let Ok(bodies) = graph.debug_kernel_solids(&Default::default()) else {
        return Vec::new();
    };
    let volume = |solid: &openrcad::topo::Solid| -> Option<f64> {
        zerocad_core::MockMesh::from_solid(solid)
            .mass_properties()
            .map(|properties| properties.volume)
    };
    let mut failures = Vec::new();
    let parts = bodies.iter().flat_map(|(_, parts)| parts);
    for (index, part) in parts.enumerate() {
        let path = std::env::temp_dir().join(format!(
            "zerocad_census_step_{}_{chain_index}_{index}.step",
            std::process::id()
        ));
        let path = path.to_string_lossy().to_string();
        if let Err(error) = openrcad::exchange::write_step(part, &path) {
            failures.push(format!("part {index}: write failed: {error}"));
            continue;
        }
        let read = openrcad::exchange::read_step_operation(&path);
        let _ = std::fs::remove_file(&path);
        let back = match read {
            Ok(result) => result.value,
            Err(error) => {
                failures.push(format!("part {index}: strict import failed: {error:?}"));
                continue;
            }
        };
        match (volume(part), volume(&back)) {
            (Some(before), Some(after))
                if (after - before).abs() > 1e-6 * before.abs().max(1.0) =>
            {
                failures.push(format!("part {index}: volume {before} -> {after}"));
            }
            (Some(_), None) => failures.push(format!("part {index}: re-imported part has no mesh")),
            _ => {}
        }
    }
    failures
}

/// Regressions the sweep found, reduced to whole-millimetre placements.
fn assert_exact(steps: &[Step], expected_parts: usize) {
    let lattice = Lattice {
        origin: [-3.137, -7.291, -2.713],
        spacing: [2.1, 2.1, 1.9],
        counts: [28, 24, 14],
    };
    let outcome = judge(steps, lattice);
    assert!(
        outcome.rejected.is_empty() && outcome.problems.is_empty(),
        "rejected {:?}: {:?}\n    {}",
        outcome.rejected,
        outcome.warnings,
        outcome.problems.join("\n    ")
    );
    assert_eq!(outcome.parts, expected_parts);
}

/// A boss overlapping a block corner leaves an overhang whose arc wraps the
/// corner; the chord of that arc crosses the block edge, which the strict
/// UV-loop audit used to report as a self-intersection.
#[test]
fn boss_overhanging_a_block_corner_joins() {
    use ExtrudeMode::{Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (43.0, 41.0),
                },
                0.0,
                11.0,
                NewBody,
            ),
            Step::along_z(
                "corner boss",
                Profile::Circle {
                    center: (38.0, 6.5),
                    radius: 10.5,
                },
                10.0,
                10.0,
                Join,
            ),
        ],
        1,
    );
}

/// A slot and a hole together sever a corner lump; a later Join elsewhere
/// must not be rejected merely because it does not reach that lump.
#[test]
fn join_after_cuts_severed_a_lump_keeps_the_lump_separate() {
    use ExtrudeMode::{Cut, Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (40.0, 35.0),
                },
                0.0,
                12.0,
                NewBody,
            ),
            Step::along_z(
                "slot off the left edge",
                Profile::Rect {
                    min: (-2.0, 21.0),
                    max: (13.0, 33.0),
                },
                -2.0,
                30.0,
                Cut,
            ),
            Step::along_z(
                "hole through the top edge",
                Profile::Circle {
                    center: (10.0, 31.5),
                    radius: 7.5,
                },
                -2.0,
                30.0,
                Cut,
            ),
            Step::along_z(
                "join far from the lump",
                Profile::Rect {
                    min: (8.0, -5.0),
                    max: (22.0, 15.0),
                },
                11.0,
                3.5,
                Join,
            ),
        ],
        2,
    );
}

/// A circle cut drawn in a side sketch must not be recorded as a vertical
/// through hole of the block's prism record: the circle's coordinates belong
/// to a different frame. The wrong record made a later sectional feature
/// rebuild material inside the sideways tunnel.
#[test]
fn side_circle_cut_is_not_recorded_as_a_vertical_hole() {
    use ExtrudeMode::{Cut, Join, NewBody};
    let steps = [
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (50.0, 45.0),
            },
            0.0,
            8.0,
            NewBody,
        ),
        Step {
            label: "sideways tunnel",
            profile: Profile::Circle {
                center: (2.0, 5.0),
                radius: 3.0,
            },
            axis: Axis::Y,
            offset: -2.0,
            depth: 65.0,
            mode: Cut,
            draft_deg: 0.0,
        },
        Step::along_z(
            "square slot through the tunnel",
            Profile::Rect {
                min: (4.0, 4.0),
                max: (6.0, 6.0),
            },
            -2.0,
            28.0,
            Cut,
        ),
        Step::along_z(
            "refill box",
            Profile::Rect {
                min: (3.0, 3.0),
                max: (7.0, 7.0),
            },
            1.0,
            3.0,
            Join,
        ),
    ];
    let outcome = judge(
        &steps,
        Lattice {
            origin: [-3.137, -7.291, -2.713],
            spacing: [0.93, 1.7, 0.61],
            counts: [30, 34, 20],
        },
    );
    // Rejections are allowed here (flush, snapped placements); wrong
    // geometry is not.
    assert!(
        outcome.problems.is_empty(),
        "rejected {:?}\n    {}",
        outcome.rejected,
        outcome.problems.join("\n    ")
    );
}

/// A mid-height boss partly refilling a through hole leaves a lens-shaped
/// section bounded by two arcs; its plane used to be derived from the two span
/// endpoints alone, which cannot orient it.
#[test]
fn boss_refilling_a_hole_builds_the_lens_section() {
    use ExtrudeMode::{Cut, Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (57.0, 48.0),
                },
                0.0,
                8.0,
                NewBody,
            ),
            Step::along_z(
                "hole notching the far edge",
                Profile::Circle {
                    center: (27.5, 41.0),
                    radius: 9.5,
                },
                -2.0,
                28.0,
                Cut,
            ),
            Step::along_z(
                "mid-height boss overlapping the hole",
                Profile::Circle {
                    center: (38.0, 29.0),
                    radius: 8.0,
                },
                0.8,
                3.0,
                Join,
            ),
        ],
        1,
    );
}

/// A box joined across a block whose side tunnel was cut along Y. The fused
/// front wall carries a boundary edge with a vertex just off its segment, so
/// the triangulator cannot recover that edge; its flip loop used to cycle to a
/// cubic budget (~50 s optimized, minutes in debug) before giving up. Exact
/// sweep coordinates: rounding them removes the near-collinear vertex.
#[test]
fn join_over_a_side_tunnel_tessellates_promptly() {
    use ExtrudeMode::{Cut, Join, NewBody};
    let step = |label, profile, axis, offset, depth, mode| Step {
        label,
        profile,
        axis,
        offset,
        depth,
        mode,
        draft_deg: 0.0,
    };
    let steps = [
        step(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (49.702231638981196, 45.02463197764338),
            },
            Axis::Z,
            0.0,
            9.201488563278437,
            NewBody,
        ),
        step(
            "side tunnel",
            Profile::Circle {
                center: (1.8467675338597123, 44.64548746774376),
                radius: 3.762929631726456,
            },
            Axis::Y,
            -2.0,
            65.02463197764338,
            Cut,
        ),
        step(
            "pocket",
            Profile::Circle {
                center: (30.193057536541453, 14.075933066539877),
                radius: 5.898682375397083,
            },
            Axis::Z,
            -2.0,
            8.537308586999686,
            Cut,
        ),
        step(
            "boss",
            Profile::Circle {
                center: (12.247415130233845, 12.399242673894971),
                radius: 4.942528726513588,
            },
            Axis::Z,
            4.595249519805727,
            10.361513352380959,
            Join,
        ),
        step(
            "slot",
            Profile::Rect {
                min: (28.343590491962658, 3.773898509059777),
                max: (47.15958125357143, 8.900157582403144),
            },
            Axis::Z,
            -2.0,
            5.095272908789385,
            Cut,
        ),
        step(
            "box overhanging the front wall",
            Profile::Rect {
                min: (12.573055473764672, -6.095646780560902),
                max: (49.299237495056275, 32.49470306153998),
            },
            Axis::Z,
            7.428223556602562,
            4.182121220295266,
            Join,
        ),
    ];
    let started = std::time::Instant::now();
    assert_exact(&steps, 1);
    let elapsed = started.elapsed();
    assert!(
        elapsed < std::time::Duration::from_secs(20),
        "chain took {elapsed:?}; the edge-recovery flip loop is cycling again"
    );
}

/// A pocket whose corner clips the rim of an earlier hole leaves a thin cell
/// between the pocket's edge and the hole's arc. The section planner sampled
/// that cell 0.0125 inside the arc, closer than the flattened circle's chord
/// sag, so it read the cell as material and refilled the hole above the
/// pocket. Exact sweep coordinates; the sliver is sampled densely.
#[test]
fn pocket_clipping_a_hole_rim_does_not_refill_the_hole() {
    use ExtrudeMode::{Cut, NewBody};
    let steps = [
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (44.70541439304844, 40.78060318956215),
            },
            0.0,
            10.012315949062112,
            NewBody,
        ),
        Step::along_z(
            "through hole",
            Profile::Circle {
                center: (24.875281647203725, 27.467008461337088),
                radius: 8.569909220711802,
            },
            -2.0,
            30.012315949062113,
            Cut,
        ),
        Step::along_z(
            "pocket clipping the hole rim",
            Profile::Rect {
                min: (6.643526119447068, 27.3654267888297),
                max: (17.520142042363535, 35.96883637018606),
            },
            -2.0,
            9.099739956305742,
            Cut,
        ),
    ];
    let outcome = judge(
        &steps,
        Lattice {
            origin: [16.0, 26.0, 0.031],
            spacing: [0.097, 0.193, 0.247],
            counts: [18, 30, 41],
        },
    );
    assert!(
        outcome.rejected.is_empty() && outcome.problems.is_empty(),
        "rejected {:?}: {:?}",
        outcome.rejected,
        outcome.problems
    );
    assert_exact(&steps, 1);
}

/// A side tunnel grazing past the widest point of a vertical hole meets its
/// wall in one loop. The kernel traced it as open pieces, never split the
/// faces, and returned a valid solid with a phantom slab inside the hole. The
/// cut may still be refused (completeness is the census target); it must
/// never be wrong.
#[test]
fn side_tunnel_grazing_a_hole_is_never_wrong() {
    use ExtrudeMode::{Cut, NewBody};
    let tunnel = |center, radius, depth| Step {
        label: "side tunnel",
        profile: Profile::Circle { center, radius },
        axis: Axis::Y,
        offset: -2.0,
        depth,
        mode: Cut,
        draft_deg: 0.0,
    };
    let steps = [
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (47.62425410470586, 33.89450436128614),
            },
            0.0,
            8.835048320896737,
            NewBody,
        ),
        tunnel(
            (2.717858522508033, 24.455700786399607),
            2.088603396195878,
            16.788764228951315,
        ),
        Step::along_z(
            "through hole",
            Profile::Circle {
                center: (7.437238573622317, 24.56758494682629),
                radius: 8.47739536547999,
            },
            -2.0,
            28.83504832089674,
            Cut,
        ),
        tunnel(
            (4.115005235872185, 13.601359326051771),
            2.4354769329312296,
            53.89450436128614,
        ),
    ];
    let outcome = judge(
        &steps,
        Lattice {
            origin: [8.013, 10.017, 0.011],
            spacing: [0.2, 0.4, 0.3],
            counts: [60, 60, 30],
        },
    );
    assert!(
        outcome.problems.is_empty(),
        "unsound (rejected {:?}): {:?}",
        outcome.rejected,
        outcome.problems
    );
}

/// A boss overhanging a block corner leaves a small corner piece of the block
/// top beside the boss. Its vertices sit a single-precision rounding (1.2e-7)
/// off the top plane, and the kernel's sliver check measured from a vertex,
/// so it took two pieces of one plane for a degenerate wall and refused the
/// side slot under them.
#[test]
fn slot_under_a_corner_boss_is_not_a_sliver() {
    use ExtrudeMode::{Cut, Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (52.30680086317662, 43.5217907382045),
                },
                0.0,
                8.694937769231585,
                NewBody,
            ),
            Step::along_z(
                "corner boss",
                Profile::Circle {
                    center: (45.20747000129385, 6.787699587398411),
                    radius: 8.675142816795013,
                },
                3.9212353626154015,
                7.658091957821667,
                Join,
            ),
            Step {
                label: "side slot",
                profile: Profile::Rect {
                    min: (2.001084536924215, 27.8174048083291),
                    max: (7.668429728892106, 47.05530356940072),
                },
                axis: Axis::Y,
                offset: -2.0,
                depth: 63.5217907382045,
                mode: Cut,
                draft_deg: 0.0,
            },
        ],
        1,
    );
}

/// A raised plate overhanging the block shares the block's front plane in
/// two pieces whose single-precision planes differ by a rounding tilt
/// (1e-8 rad, so 8e-7 apart across the part). Measuring that as a wall
/// thickness refused the side slot beneath; same-facing faces bound no wall.
#[test]
fn side_slot_under_an_overhanging_plate_is_not_a_sliver() {
    use ExtrudeMode::{Cut, Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (61.8745156840771, 33.836116843161946),
                },
                0.0,
                8.430529086758836,
                NewBody,
            ),
            Step::along_z(
                "plate",
                Profile::Rect {
                    min: (30.47299470900599, 7.551545039784633),
                    max: (72.75059360070156, 33.306127676122806),
                },
                5.4915251726243355,
                7.291509391139787,
                Join,
            ),
            Step::along_z(
                "slot",
                Profile::Rect {
                    min: (2.8663899077060204, 19.85168312309466),
                    max: (29.48625542949646, 24.068191504455918),
                },
                -2.0,
                28.43052908675884,
                Cut,
            ),
            Step {
                label: "side slot",
                profile: Profile::Rect {
                    min: (1.1038593627666136, 27.264023401912784),
                    max: (5.267778881933989, 46.303180284892136),
                },
                axis: Axis::Y,
                offset: -2.0,
                depth: 53.836116843161946,
                mode: Cut,
                draft_deg: 0.0,
            },
        ],
        1,
    );
}

/// A box joined across a blind side tunnel seals the tunnel's far end into an
/// enclosed void. The result is one part with a cavity, not a refusal.
#[test]
fn join_bridging_a_blind_tunnel_seals_a_cavity() {
    use ExtrudeMode::{Cut, Join, NewBody};
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (60.0, 36.0),
                },
                0.0,
                11.3,
                NewBody,
            ),
            Step {
                label: "blind tunnel",
                profile: Profile::Circle {
                    center: (5.5, 21.4),
                    radius: 2.9,
                },
                axis: Axis::Y,
                offset: -2.0,
                depth: 21.0,
                mode: Cut,
                draft_deg: 0.0,
            },
            Step::along_z(
                "bridge",
                Profile::Rect {
                    min: (-14.0, 10.3),
                    max: (30.0, 15.5),
                },
                1.2,
                8.1,
                Join,
            ),
        ],
        1,
    );
}

/// A side tunnel breaking out through the bottom face leaves arcs whose
/// circle reaches below the part. The join's sanity check compared two
/// conservative boxes, so the input's circle-bounded box (below z = 0)
/// was not inside the result's re-trimmed one and a valid union of an
/// overhanging plate was refused.
#[test]
fn plate_joined_over_a_tunnel_breaking_out_the_bottom() {
    use ExtrudeMode::{Cut, Join, NewBody};
    let tunnel = |center, radius, depth| Step {
        label: "side tunnel",
        profile: Profile::Circle { center, radius },
        axis: Axis::Y,
        offset: -2.0,
        depth,
        mode: Cut,
        draft_deg: 0.0,
    };
    assert_exact(
        &[
            Step::along_z(
                "block",
                Profile::Rect {
                    min: (0.0, 0.0),
                    max: (58.03661742674635, 44.378702409642315),
                },
                0.0,
                8.614592082859806,
                NewBody,
            ),
            tunnel(
                (1.8980022967709662, 22.816574769015947),
                3.2505305557234654,
                22.718273301787637,
            ),
            tunnel(
                (5.862361977790951, 7.991653540269629),
                3.0147877360049544,
                15.960789052171188,
            ),
            Step::along_z(
                "plate",
                Profile::Rect {
                    min: (20.2296237940877, -1.2557250335545618),
                    max: (70.54400801908804, 34.059614535699204),
                },
                2.084458823949696,
                4.590029574481609,
                Join,
            ),
        ],
        1,
    );
}

/// One placement per broad step kind, so the oracle's frames, profiles, and
/// draft sign are pinned against the model before any random sweep.
fn block_then(step: Step) -> [Step; 2] {
    [
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (50.0, 40.0),
            },
            0.0,
            10.0,
            ExtrudeMode::NewBody,
        ),
        step,
    ]
}

#[test]
fn broad_kinds_match_the_oracle() {
    use ExtrudeMode::{Cut, Join};
    let step = |label, profile, axis, offset, depth, mode, draft_deg| Step {
        label,
        profile,
        axis,
        offset,
        depth,
        mode,
        draft_deg,
    };
    let rect = |min, max| Profile::Rect { min, max };
    let cases = [
        step(
            "x side slot",
            Profile::Slot {
                a: (12.0, 5.0),
                b: (25.0, 5.0),
                radius: 2.5,
            },
            Axis::X,
            -2.0,
            70.0,
            Cut,
            0.0,
        ),
        step(
            "x side boss",
            Profile::Circle {
                center: (20.0, 5.0),
                radius: 3.5,
            },
            Axis::X,
            45.0,
            9.0,
            Join,
            0.0,
        ),
        step(
            "L boss",
            Profile::polygon(&[
                (10.0, 10.0),
                (30.0, 10.0),
                (30.0, 18.0),
                (18.0, 18.0),
                (18.0, 30.0),
                (10.0, 30.0),
            ]),
            Axis::Z,
            6.0,
            8.0,
            Join,
            0.0,
        ),
        step(
            "triangle through cut",
            Profile::polygon(&[(20.0, 8.0), (38.0, 14.0), (24.0, 30.0)]),
            Axis::Z,
            -2.0,
            30.0,
            Cut,
            0.0,
        ),
        step(
            "tapered boss",
            rect((15.0, 12.0), (35.0, 28.0)),
            Axis::Z,
            5.0,
            12.0,
            Join,
            10.0,
        ),
        step(
            "flared boss",
            rect((15.0, 12.0), (35.0, 28.0)),
            Axis::Z,
            5.0,
            12.0,
            Join,
            -6.0,
        ),
        step(
            "tapered pocket",
            rect((15.0, 12.0), (35.0, 28.0)),
            Axis::Z,
            12.0,
            -9.0,
            Cut,
            8.0,
        ),
        // Rotated hexagon breaking out of the x = 0 face, flaring 1.4 degrees:
        // its walls were near-flat ruled surfaces (f32 offset corners) and
        // the boolean refused them.
        step(
            "drafted hexagon breakout pocket",
            polygon(2, (6.9, 12.7), (7.8, 7.8), 0.3),
            Axis::Z,
            14.0,
            -4.5,
            Cut,
            -1.4,
        ),
        {
            let axis = Axis::tilted(0.7, 30_f64.to_radians());
            let (s, t, a) = axis.split([25.0, 20.0, 5.0]);
            step(
                "tilted through cut",
                Profile::Circle {
                    center: (s, t),
                    radius: 4.0,
                },
                axis,
                a - 60.0,
                120.0,
                Cut,
                0.0,
            )
        },
        {
            let axis = Axis::tilted(2.5, 25_f64.to_radians());
            let (s, t, a) = axis.split([25.0, 20.0, 8.0]);
            step(
                "tilted boss",
                rect((s - 4.0, t - 3.0), (s + 4.0, t + 3.0)),
                axis,
                a,
                9.0,
                Join,
                0.0,
            )
        },
    ];
    let mut failures = Vec::new();
    for case in cases {
        let steps = block_then(case);
        let outcome = judge(
            &steps,
            Lattice {
                origin: [-3.137, -3.291, -2.713],
                spacing: [1.13, 1.07, 0.71],
                counts: [52, 43, 36],
            },
        );
        if !outcome.rejected.is_empty() || !outcome.problems.is_empty() {
            failures.push(format!(
                "{}: rejected {:?} {:?}\n    {}",
                case.label,
                outcome.rejected,
                outcome.warnings,
                outcome.problems.join("\n    ")
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

fn assert_broad_exact(steps: &[Step]) {
    let outcome = judge(
        steps,
        Lattice {
            origin: [-3.137, -3.291, -2.713],
            spacing: [1.13, 1.07, 0.71],
            counts: [60, 50, 36],
        },
    );
    assert!(
        outcome.rejected.is_empty() && outcome.problems.is_empty(),
        "rejected {:?}: {:?}\n    {}",
        outcome.rejected,
        outcome.warnings,
        outcome.problems.join("\n    ")
    );
}

/// A slot's line ends are f32 sketch points a micron off its arcs, so each
/// junction weld leaves a measured residual that becomes an edge tolerance.
/// On a tilted plane the validator re-measures that residual an ulp larger
/// (prism seam lines, arc pcurves on the cap) and refused the extrusion.
/// Full-precision census placements: rounding them loses the tie.
#[test]
fn tilted_slot_boss_extrudes() {
    assert_broad_exact(&[
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (64.18753237876741, 46.08919340577647),
            },
            0.0,
            9.240973464981105,
            ExtrudeMode::NewBody,
        ),
        Step {
            label: "tilted slot boss",
            profile: Profile::Slot {
                a: (43.94421385861567, 41.201969555605885),
                b: (35.593435612447564, 26.93923765972283),
                radius: 1.8261608014429838,
            },
            axis: Axis::Tilted {
                u: [
                    0.8882922899680654,
                    0.014753760298495712,
                    -0.4590415385783131,
                ],
                v: [0.014753760298495712, 0.998051401797752, 0.06062776531092597],
            },
            offset: 25.63307401237047,
            depth: 8.070857668744516,
            mode: ExtrudeMode::Join,
            draft_deg: 0.0,
        },
    ]);
}

#[test]
fn tilted_slot_through_cut_applies() {
    assert_broad_exact(&[
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (53.896002793599735, 49.03084082374777),
            },
            0.0,
            13.160427852735044,
            ExtrudeMode::NewBody,
        ),
        Step {
            label: "tilted slot cut",
            profile: Profile::Slot {
                a: (39.453272562215844, 34.51152363516833),
                b: (45.57317245037045, 48.05732127997919),
                radius: 2.6585234031787204,
            },
            axis: Axis::Tilted {
                u: [
                    0.9625986363693652,
                    -0.010458870772304766,
                    0.27072952790932675,
                ],
                v: [
                    -0.010458870772304766,
                    0.997075294395358,
                    0.07570646820832679,
                ],
            },
            offset: -126.41125131199223,
            depth: 232.1745429401651,
            mode: ExtrudeMode::Cut,
            draft_deg: 0.0,
        },
    ]);
}

/// The cylinder's top circle crosses the drafted pocket's rim 9e-5 short of
/// a corner. Imprinted edges carried a 1e-4 tolerance floor, so that short
/// piece was "degenerate", and sewing also dropped it while its two vertices
/// stayed apart, leaving two open loops.
#[test]
fn cylinder_joined_just_short_of_a_drafted_pocket_corner() {
    assert_broad_exact(&[
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (64.19581340923749, 38.863794258062285),
            },
            0.0,
            8.063885515413475,
            ExtrudeMode::NewBody,
        ),
        Step {
            label: "drafted pocket",
            profile: Profile::polygon(&[
                (33.18335959563854, 21.94557276943101),
                (33.99215991654824, 13.387829202781578),
                (39.480074302289545, 12.040843188562295),
                (44.15918836712115, 19.251600740992437),
                (43.350388046211464, 27.80934430764186),
                (37.86247366047015, 29.15633032186115),
            ]),
            axis: Axis::Z,
            offset: 10.063885515413475,
            depth: -5.0424530348084105,
            mode: ExtrudeMode::Cut,
            draft_deg: 13.410557544438134,
        },
        Step::along_z(
            "cylinder",
            Profile::Circle {
                center: (33.62019333762168, 32.675869915605084),
                radius: 10.667851186708614,
            },
            1.904322497643094,
            10.933789394572193,
            ExtrudeMode::Join,
        ),
    ]);
}

/// A tilted slot cap's cylinder rulings are parameterized from a far axis
/// origin; the line/ellipse crossing with the block top lay outside the
/// subdivision search's [-100, 100] window, so the trim fell back to a
/// containment bisection 1e-3 off and left a dangling spur.
#[test]
fn tilted_slot_cut_beside_an_l_boss() {
    assert_broad_exact(&[
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (42.305778092914636, 48.67758922864307),
            },
            0.0,
            9.277019236492128,
            ExtrudeMode::NewBody,
        ),
        Step::along_z(
            "L boss",
            Profile::polygon(&[
                (9.644119236239371, 39.42846804567416),
                (13.539411099017835, 32.594023398026174),
                (22.676497208813622, 37.801705807926474),
                (20.72885127742439, 41.21892813175047),
                (29.86593738722018, 46.42661054165077),
                (27.918291455830946, 49.84383286547476),
            ]),
            5.045800203169289,
            11.84152767955861,
            ExtrudeMode::Join,
        ),
        Step {
            label: "tilted slot cut",
            profile: Profile::Slot {
                a: (18.519903819928853, 7.4502518253745205),
                b: (24.619183487166563, 21.222271468798603),
                radius: 2.680868379509902,
            },
            axis: Axis::Tilted {
                u: [0.8658217201479131, 0.10166639289255561, 0.4899149859683059],
                v: [
                    0.10166639289255561,
                    0.9229677451881365,
                    -0.37120679667611634,
                ],
            },
            offset: -100.42663634172702,
            depth: 200.52077311609966,
            mode: ExtrudeMode::Cut,
            draft_deg: 0.0,
        },
    ]);
}

/// The polygon cut severs a small lump the later slot misses, though its box
/// overlaps the slot's. The boolean on that lump succeeded and removed
/// nothing; treating that as a failure reverted the whole cut.
#[test]
fn tilted_slot_cut_skips_a_severed_lump() {
    assert_broad_exact(&[
        Step::along_z(
            "block",
            Profile::Rect {
                min: (0.0, 0.0),
                max: (43.826429145729755, 39.89955633632534),
            },
            0.0,
            8.376304237374478,
            ExtrudeMode::NewBody,
        ),
        Step {
            label: "tilted boss",
            profile: Profile::Rect {
                min: (11.354059168567698, 5.537667386698386),
                max: (24.61827384582539, 24.46761929327384),
            },
            axis: Axis::Tilted {
                u: [
                    0.9133698438793174,
                    0.050056495794689834,
                    0.40404192297412345,
                ],
                v: [
                    0.050056495794689834,
                    0.9710764370809487,
                    -0.23346284624095232,
                ],
            },
            offset: 2.6128484833546612,
            depth: 8.271322324668844,
            mode: ExtrudeMode::Join,
            draft_deg: 0.0,
        },
        Step::along_z(
            "L through cut",
            Profile::polygon(&[
                (18.384780171272194, 18.0469374841364),
                (25.581099958698328, 21.489245836046223),
                (22.39452559797406, 28.15094077498513),
                (18.796365704260992, 26.42978659903022),
                (15.609791343536726, 33.09148153796913),
                (12.01163144982366, 31.370327362014216),
            ]),
            -2.0,
            28.376304237374477,
            ExtrudeMode::Cut,
        ),
        Step {
            label: "tilted slot cut",
            profile: Profile::Slot {
                a: (28.54994073553057, 35.14978485994419),
                b: (14.243079853521131, 30.063292277819055),
                radius: 1.7359553133662664,
            },
            axis: Axis::Tilted {
                u: [0.9558366860537191, 0.05123792523622306, 0.28939748549586836],
                v: [0.05123792523622306, 0.940554167069388, -0.3357566586471472],
            },
            offset: -83.45633907663404,
            depth: 184.20457943885916,
            mode: ExtrudeMode::Cut,
            draft_deg: 0.0,
        },
    ]);
}

/// Two same-radius blind holes from opposite faces that do not meet leave the
/// web between them (census coaxial chain 131, reduced).
#[test]
fn coaxial_blind_holes_from_opposite_faces_leave_the_web() {
    let (w, d, h) = (57.02463019820678, 45.876058219523394, 11.667196039902185);
    let block = Step::along_z(
        "base block",
        Profile::Rect {
            min: (0.0, 0.0),
            max: (w, d),
        },
        0.0,
        h,
        ExtrudeMode::NewBody,
    );
    let mut bad = vec![];
    for (pa, pb) in [
        (0.0_f64, 0.0_f64),
        (0.9, 2.6),
        (-1.2638, -0.5757),
        (-1.2638, 0.0),
        (0.0, -0.5757),
        (1.0, 1.0),
    ] {
        let hole = |phase: f64, z0: f64, z1: f64| {
            let (sin, cos) = phase.sin_cos();
            let (u, v) = ([cos, sin, 0.0], [-sin, cos, 0.0]);
            Step {
                label: "blind hole",
                profile: Profile::Circle {
                    center: (
                        w / 2.0 * u[0] + d / 2.0 * u[1],
                        w / 2.0 * v[0] + d / 2.0 * v[1],
                    ),
                    radius: 7.943611877334662,
                },
                axis: Axis::Tilted { u, v },
                offset: z0,
                depth: z1 - z0,
                mode: ExtrudeMode::Cut,
                draft_deg: 0.0,
            }
        };
        let lattice = Lattice {
            origin: [-3.137, -7.291, -2.713],
            spacing: [2.1, 2.1, 1.9],
            counts: [28, 24, 14],
        };
        let outcome = judge(
            &[
                block,
                hole(pa, 7.83854288964291, h + 2.0),
                hole(pb, -2.0, 4.7673879),
            ],
            lattice,
        );
        if !outcome.problems.is_empty() || !outcome.rejected.is_empty() {
            bad.push(format!(
                "phases ({pa},{pb}): {:?} {:?}",
                outcome.rejected, outcome.problems
            ));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// The exact f32 sketch frames of a census chain: coincident circle arcs whose
/// midpoints straddled the arrangement's quantisation grid once left a
/// duplicate edge, and the cut removed the whole block around the web.
#[test]
fn coaxial_blind_holes_with_straddling_arc_midpoints_leave_the_web() {
    let block = Step::along_z(
        "base block",
        Profile::Rect {
            min: (0.0, 0.0),
            max: (57.02463019820678, 45.876058219523394),
        },
        0.0,
        11.667196039902185,
        ExtrudeMode::NewBody,
    );
    let a = Step {
        label: "top",
        profile: Profile::Circle {
            center: (-13.23964215340951, 34.114764654039405),
            radius: 7.943611877334662,
        },
        axis: Axis::Tilted {
            u: [0.30246509988775816, -0.9531604604419387, 0.0],
            v: [0.9531604604419387, 0.30246509988775816, 0.0],
        },
        offset: 7.83854288964291,
        depth: 5.828653150259275,
        mode: ExtrudeMode::Cut,
        draft_deg: 0.0,
    };
    let b = Step {
        label: "bottom",
        profile: Profile::Circle {
            center: (11.374837034704743, 34.7810059398049),
            radius: 7.943611877334662,
        },
        axis: Axis::Tilted {
            u: [0.8379704503903084, -0.5457156075032707, 0.0],
            v: [0.5457156075032707, 0.8379704503903084, 0.0],
        },
        offset: -2.0,
        depth: 6.767387953994474,
        mode: ExtrudeMode::Cut,
        draft_deg: 0.0,
    };
    assert_exact(&[block, a, b], 1);
}
