# Contributing to ZeroCAD

Thanks for your interest! ZeroCAD is an early-alpha parametric 3D CAD
application written in Rust. This guide covers how to build, test, and submit
changes. For the *design* of the system — the geometry pipeline, the parametric
graph, and the non-obvious invariants — read [README.md](README.md) first; it is
the architectural map and this guide does not repeat it.

## Prerequisites

- A recent stable Rust toolchain (install via [rustup](https://rustup.rs)).
- The workspace builds on Windows out of the box. On Linux you also need the
  system libraries `eframe`/`wgpu`/`winit`/`rfd` depend on (e.g. on Debian/Ubuntu:
  `libgtk-3-dev libxkbcommon-dev libwayland-dev`). The pure-geometry
  `zerocad-core` crate needs none of these.

## Build, run, test

```bash
cargo run --release          # release is strongly recommended — the truck solver is CPU-heavy
cargo test --workspace       # full test suite
cargo test -p zerocad-core   # geometry engine only (no GUI/system deps)
```

## Code style

Before opening a PR, please run:

```bash
cargo fmt --all
cargo clippy --workspace
```

CI runs `rustfmt --check` and `clippy` as **informational** (non-blocking) jobs
today, because the codebase is not yet fully `rustfmt`-normalized and has some
pre-existing clippy suggestions. Don't reformat unrelated code in a feature PR —
keep diffs focused. Normalizing formatting and turning these into required gates
is a welcome standalone PR.

The hard CI gates are **build + test** (Windows full workspace, Linux core).
A change that doesn't compile or breaks a test will be flagged.

## Adding a feature

The README has the authoritative checklists; the short version:

- **A new geometry/feature type** → follow *"Adding a new feature type — the
  checklist"* in the README, and **read *"Non-obvious invariants — read before
  touching geometry"* first.** Orientation, winding handedness, the 0.1 mm
  coplanarity overshoot, and the fragility of the `truck` boolean solver will
  bite you otherwise. Never call `truck_shapeops` directly — go through the
  guarded `mock_kernel::union` / `difference` wrappers.
- **Geometry changes need a regression test** in
  `zerocad-core/tests/realistic_modes.rs` that asserts the geometry actually
  *changed* (e.g. new hole-wall triangles), not merely that triangle counts
  differ.
- **A new user-facing command** should go through the shared action path so it
  works from both the menu and the keyboard: add a `ShortcutAction` variant
  (`zerocad-gui/src/shortcuts.rs`) and handle it in `ZeroCadApp::run_shortcut`.

## Pull requests

- Keep PRs small and focused on one change.
- Describe *what* changed and *why*; reference any issue it closes.
- Make sure `cargo test --workspace` passes locally.
- New geometry behavior must come with a test.

## Reporting bugs

Open a GitHub issue with steps to reproduce, what you expected, and what
happened. For geometry/boolean failures, the model that triggers it (a saved
`.zcad` file) is enormously helpful — booleans are configuration-sensitive.

## License

By contributing, you agree that your contributions are dual-licensed under
[MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at the user's option, the
same terms as the project.
