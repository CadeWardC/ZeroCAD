# Reused face-sketch extrusion repair

The `broken-extrusion.zcad` fixture reproduces an incorrect successful extrusion
when `Sketch_3` is reused after `Join_2`. Region 0 is a 2.25245 × 59 mm strip.
Previously the evaluator resolved the sketch against the body **after** its
first Join, chose a remaining support-face fragment, moved the sketch origin,
and reinterpreted region 0 as the 33.2 × 59 mm inner rectangle.

Extrude now resolves the sketch plane and reference boundary from the existing
body checkpoint preceding the sketch in semantic history. It still combines
the resulting tool with the current target body. Upstream edits therefore
remain associative, while downstream consumers cannot move their own input.
The path is shared by GUI exact previews, committed features, and headless
calls. No document recipe or feature schema changed.

Sparse hydrated checkpoints resume before any missing sketch-support snapshot;
old mesh and kernel accelerators are invalidated by cache ABI 17. Sketch input
sequence keys also participate in cache invalidation.

## Supported behavior and regression evidence

`repro_reused_face_sketch.rs` checks the fixture's exact strip bounds and volume,
warm evaluation, attachment writeback, save/reopen, and an upstream depth edit.
It also checks the adjacent-strip Join's added volume and strict solid validity.
The unit test `sparse_checkpoints_rebuild_missing_face_sketch_support` covers a
missing intermediate checkpoint with a retained downstream result.

The single-strip **New Body** extrusion is valid. The single-strip **Join** is
not: along x = -6.4, y = -20, z = 4.1 through 63.1, four faces would meet at one
edge. It is rejected atomically, preserving the original bodies. Selecting
regions 0 and 2 together fills the adjoining strip and produces a valid Join.
The regression verifies both outcomes; it does not weaken the solid-validity
checks to accept the original edge contact.

## Validation (2026-09-27)

- `cargo fmt --all -- --check`: passed.
- `cargo check`: passed.
- `cargo test`: passed, 1,522 tests and no ignored tests.
- `cargo test -p zerocad-core --lib`: passed, 546 tests, including the added
  sparse-checkpoint regression.
- `cargo test -p zerocad-core --test repro_reused_face_sketch`: all four passed.
- `cargo clippy -p zerocad-core --all-targets`: completed successfully; existing
  warnings elsewhere in the workspace remain (none in the added regressions or
  changed attachment logic).
- `cargo build --release -p zerocad-gui`: passed; optimized executable at
  `target/release/zerocad-gui.exe`.
