# Direct sketch dimension editing

The Dimension tool opens the value editor immediately when a circle, arc, or
line is clicked. Circles use diameter, arcs use radius, and rectangle sides use
the existing width or height driver. Existing drivers are reused instead of
adding competing constraints.

For a two-line angle or spacing, Shift-click the lines, then click empty space
to place the dimension. For point distances, select two points and place the
dimension horizontally, vertically, or diagonally. Unsupported ellipse and
spline size selections report the limitation.

The editor accepts numbers, arithmetic, and existing parameter names. Length
values and expressions use millimeters; angles use degrees. The optional New
variable field creates a document parameter and binds this dimension to it on
Apply or Enter. Names must be valid and unused. Parameters remain editable in
the existing Parameters UI. Conflicting or invalid values preserve the last
valid geometry and show an error. Cancel/Escape restores the pre-edit sketch.
Undo removes a newly created parameter together with its dimension edit.
Finish Sketch validates an open editor before committing.

Regression coverage in `sketch_ui.rs` and `constraints_panel.rs` checks direct
circle/rectangle picking, all four rectangle sides, driver reuse, named values,
invalid input, undo, document roundtrip, and parameter-driven updates. Existing
tests cover point distances, angles, parallel spacing, arcs, and downstream
extruded volume after editing. Native pointer interaction has not been manually
verified; these tests do not establish full Fusion behavior or latency targets.

Validation on 2026-09-26: `cargo check`, all 174 GUI tests, and the two core
dimension tests passed. Changed GUI files pass rustfmt. Workspace-wide
`cargo fmt --all -- --check` reports formatting differences in the independently
edited `zerocad-core/tests/repro_cut_through_error.rs`. Core Clippy completes
with existing warnings outside these changes. The full `cargo test` run was
stopped after more than 13 minutes, when the `misc_torture` test executable had
run for over five minutes (including the 100-feature and 36-hole stress tests).
No assertion failure was reported before termination; this is not a full-suite
pass. Local logs are in `target/dimension-{gui,core}-tests.log` and
`target/dimension-test-results.log`.
