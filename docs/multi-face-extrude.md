# Extruding multiple body faces

Select the first flat body face, then hold Ctrl or Shift while clicking the other
faces. Click **Extrude Faces (2)** (or the displayed selection count), enter a
distance, and confirm. All selected faces use the same distance and outward
direction. Positive distance defaults to Join; negative distance defaults to Cut.

Supported scope: planar faces with parallel, equally oriented normals, including
separate faces on the same body. Faces may lie on different parallel planes.
Curved faces and faces pointing in different directions are rejected before an
operation starts. Existing sketch-profile extrusion remains available.

The operation stages a helper sketch per face and commits all extrusions in one
Undo step. Cancel creates no features. Saved documents use existing sketch and
extrusion feature types; there is no file-format change. Each resulting extrusion
remains independently editable in history. Exact previews and commits use the
same face-attached body targets.

Regression coverage in `zerocad-gui/src/extrude.rs` uses the two end faces of a
U-shaped bracket. It checks Join and Cut volumes, preview/commit agreement,
save/reopen, one-step Undo, cancellation, and rejection of incompatible normals.
The interaction has not been manually verified in the desktop application.

Validation (2026-09-26): formatting and `cargo check` passed. All 176 GUI tests
and 545 core unit tests passed, including the five extrusion-preview tests.
`cargo clippy -p zerocad-core --all-targets` completed with warnings in existing
core code. The workspace test run was stopped after more than 30 minutes while
unrelated thread stress tests were still running; the separate application run
was also stopped before all integration tests finished. No assertion failures
were reported before stopping. Full-suite validation remains incomplete.
Local run logs are in `target/multi-face-validation/`.
