# Cuts crossing existing hole boundaries

The `cut-through-error.zcad` regression reproduces a rectangular cut across an
earlier cylindrical cut in a bracket. The saved feature is `extrude_37`; the
reported preview used `extrude_36`. Previously the operation preserved the
original material and reported an unresolved overlapping cut.

## Cause and repair

`openrcad-algo/src/imprint.rs` treated a single intersection span as an immediate
outer-wire split even when an endpoint belonged to an inner wire. The builder's
outer-only split precondition then failed. Such spans now enter the deferred
face partition graph.

`openrcad-topo/src/builder.rs` previously constructed that graph using only the
outer wire and the new intersection edges. Existing holes were redistributed
afterwards as whole wires. A hole crossed by a new cut cannot remain a whole
wire: it may become part of a notch or several resulting boundaries. The graph
now includes inner wires incident to the splitting edges, rejects traversal
into their original voids, and excludes those consumed wires from redistribution.
Untouched inner wires retain the existing containment-based distribution.

This changes the shared native boolean path, with no feature-specific geometry,
dimension offsets, new dependency, or relaxation of solid validation. Invalid
results still preserve the previous model. Geometry and mesh cache ABI 14 (or
later) forces old derived results to rebuild; the authoritative document schema
is unchanged.

## Regression evidence

Run `cargo test -p zerocad-core --test repro_cut_through_error`.

- The supplied model must produce one healthy, strictly validated solid without
  warnings on cold evaluation, warm evaluation and save/reopen.
- Its volume is compared with the analytic rectangular slab minus circular-strip
  overlap. Point membership checks cover the new slot, nearby material and the
  earlier holes.
- A synthetic matrix covers narrow and wide slots across cylindrical bores,
  partial-depth and full-depth cuts, and three scales/translations. It checks
  topology, independently calculated volumes and material membership, including
  cuts that separate a solid into multiple bodies.

Scope is intersection spans meeting existing trimmed hole boundaries. This is
not a claim that all tangent, degenerate, spline or overlapping-cut cases are
supported, or that behavior matches another CAD product universally. Runtime,
memory and package-size changes have not been benchmarked.

## Validation

- Formatting checks passed in both workspaces.
- `cargo check` and `cargo clippy -p zerocad-core --all-targets` passed.
- `python scripts/check-clippy.py` passed with no warnings beyond the reviewed
  baseline; its three script tests also passed.
- The focused regression suite passed, including all 12 synthetic cases.
- `cargo test -p zerocad-core -p zerocad-gui` passed: 982 tests.
- `cargo test --workspace` in `OpenRCAD` passed: 589 tests.
- `cargo test -p break-test` passed: 522 stress tests.
- `cargo build --release -p zerocad-gui --locked` passed. The Windows executable
  is `target/release/zerocad-gui.exe`.

The initial root `cargo test` could not replace a stress-test executable already
running in another process (Windows LNK1104). The application and stress packages
were subsequently run separately to avoid interrupting that process.
Another concurrent repair subsequently updated sewing and cache ABI 15. Both
cut regressions were rerun successfully against that combined workspace.
The release executable was rebuilt and `cargo check` passed again. A dedicated
copy of the executable and the supplied model is in `target/cut-through-repair/`.
