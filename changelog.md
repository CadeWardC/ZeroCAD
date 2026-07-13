# ZeroCAD Changelog

All notable changes to ZeroCAD are documented here.

---

## [Unreleased]

### 📐 Sketch & 3D Modeling

* **Sketch directly on 3D faces:** You can now draw sketches directly on any flat face of a 3D model. 
  * The edges and holes of the face are automatically turned into guidelines that you can snap to.
  * Drawing overlapping lines automatically cuts your sketch into ready-to-extrude shapes.
  * If you change the size of the 3D model earlier in your design history, your face sketch automatically moves and updates to match the new size.
* **Quick Push/Pull (Direct Modeling):** Pull or push a 3D face to add or cut material instantly without having to draw a sketch first. Existing holes are automatically preserved.
* **Real modeled threads, Fusion-style:** The Thread tool now cuts true helical thread geometry into your parts.
  * Threads are built from a handful of large, smooth spiral surfaces — the crest, each flank, and the root each select and shade as **one continuous coil face**, and the wireframe shows clean helix curves instead of a dense mesh grid.
  * **Tapped holes work:** apply an internal thread to a drilled hole and the thread is really cut into the hole wall — no more cosmetic-only warning.
  * External threads on bosses of larger bodies are cut the same way.
  * Threads use the proper flat-crest/flat-root ISO-style profile and cut straight through the ends — the thread cross-section is visible on the end faces, just like Fusion 360, instead of fading out near the ends.
  * Multi-start threads now model each start as real geometry.
  * Thread creation is effectively instant (milliseconds instead of seconds), which also removes the viewport lag threads used to cause.
  * **Threads no longer slow down the rest of your design:** a modeled thread used to be re-meshed on every edit and preview anywhere in the model (large threads could add nearly a second of lag per change). The thread mesh is now built once and reused, so editing other features on a threaded part stays fast.

* **Fusion-style live extrude previews:** push/pull now shows the *real* result while you drag.
  * While dragging an extrude into or out of a body, the actual merged/cut body continuously updates and follows the handle (computed in the background, always chasing your latest position) instead of waiting until you release the mouse.
  * Dragging a circle (or any curved profile) is now smooth: the orange/red preview volume used to be fully rebuilt through the geometry kernel on every mouse movement — it's now built once and stretched to the drag distance, so it tracks the cursor at full frame rate.
  * Typed and dragged Join/New Body values always keep the current lightweight volume visible until the exact result for those same inputs arrives; an older background result is now withheld completely, so it can no longer lag behind as an apparent second gray body, make the preview appear frozen, or briefly replace a newer commit.
* **Fixed: fillet/chamfer preview on cylinder rims.** Filleting or chamfering the circular edge of a cylinder sometimes showed a jagged, flat orange ring floating around the rim instead of a smooth preview band hugging the corner. The preview now correctly identifies which side of the edge is the cap and which is the wall, so the band wraps cleanly around the rim in every case.
* **Fixed: fillet/chamfer preview ring flipping outward on some cylinders.** On certain extruded cylinders the preview band around a rim still flared outward as a jagged faceted flange (or hid inside the body), and whether it happened could change from one part to the next in the same session. The stored "which way does the wall face" direction for a rim was decided by a numerical coin toss on full 360° walls; it is now read directly from the mesh right at the edge, so the preview band always hugs the correct side of the rim.
* **Fixed: fillet/chamfer preview showed only thin outlines instead of a visible band.** The instant preview band marks the surface that will remain after the corner is cut away — which lies *inside* the part, so the solid body was hiding it and all you saw were faint rings/lines at the edge. The band now shows through the body as a warm translucent overlay the moment you pick an edge, and it refines into the real rounded/beveled body once the exact computation finishes (usually well under a second).
* **Fixed: fillets/chamfers on small extruded cylinders were wrongly rejected.** Rounding the rim of a small sketched-and-extruded cylinder (e.g. Ø8mm) failed with a "candidate removed unselected circular-bite side span" error even though the blend was perfectly valid — a safety check meant for partial bite arcs misread the fully-selected closed rim. Rim fillets and chamfers on extruded circles now commit cleanly at any size that fits.
* **Better behavior when a fillet/chamfer size doesn't work:** if the exact computation fails at the current size, the tool now tells you immediately in the status bar and keeps the editing session open (showing your body unchanged plus the preview band) so you can just try another size — it also no longer silently re-attempts the failing computation in a loop in the background.
* **Fixed: a larger second fillet now miters cleanly into an established fillet.** Perpendicular edge fillets with different radii now share a true cylinder-to-cylinder setback seam and matching side-face runout instead of closing with an off-surface cap that produced four render cracks and left the body unchanged. Both smaller-then-larger and larger-then-smaller orders stay watertight and crack-free.
* **Tangent-chain fillets and chamfers on rounded pockets:** selecting a rounded pocket-rim segment now includes its tangent straight neighbours and rebuilds the whole line/arc chain as one analytic local operation. Inner-wire support faces and shared end caps are trimmed together, eliminating the former "simple outer-loop faces only" failure and keeping the result watertight without a slow global boolean fallback.
* **Immediate exact fillet/chamfer previews:** changing the size or switching between Fillet and Chamfer now starts the exact background solve in the same frame instead of waiting for a 100 ms settle delay. Newer input still cancels obsolete work, and the instant lightweight preview remains visible until the exact result arrives.
* **Stable inline sketch dimensions:** moving the pointer across the width, height, length, or angle boxes no longer interrupts the live shape preview, typing a multi-digit value no longer drops or replaces its first digits, and every box stays attached just outside the actual dimension-resolved shape instead of drifting with the raw cursor. The active field now uses a soft focus treatment with no selected-text glare; the first typed character silently replaces its live value, and Tab moves focus to the next dimension.

---

### 🖥️ Performance & Graphics (GPU Viewport)

We are moving the 3D viewport to a hardware-accelerated GPU renderer. This makes the viewport faster, smoother, and much cleaner to look at.

* **Faster navigation:** Modifying a part only updates the changed component, keeping overall performance fast.
* **Cleaner visual style:** 
  * Grid lines and axes now sit properly behind solid objects instead of showing through them.
  * Part edges look smooth and clean on high-resolution screens (no more jagged pixel lines).
  * Selected items stay highlighted even while you are previewing a command.
* **Real-time operation previews:** You can now see smooth, semi-transparent previews (such as red cut volumes) directly on the GPU as you drag or edit tools.
* **Instant hover response:** Hovering over a face instantly and accurately highlights it under the cursor.
* **Pixel-perfect face selection:** Clicking a face (and picking a face to sketch on) is now answered by the GPU for the exact pixel under your cursor — instant even on heavy models, and precise along curved silhouettes. Edge and corner clicks keep their comfortable grab distance and still take priority over faces.
* **Custom graphics settings:** Under **Settings → Viewport**, you can now choose your preferred graphics system (Vulkan, DirectX 12, or OpenGL), set anti-aliasing (smoothing) levels, and see which graphics card is active.
* **Crash-free fallbacks:** If your graphics card is not supported or lacks driver support, the app automatically switches back to CPU rendering instead of crashing.
* **Model edits no longer freeze the interface:** committed model rebuilds, extrude previews, and fillet/chamfer previews now run through one persistent background evaluator. Newer edits cancel obsolete work, so ZeroCAD prioritizes the model you are currently making instead of finishing stale previews first.
* **Fast first, exact after:** interactive tools show a lightweight preview immediately and replace it with the exact result. Fillet/chamfer exact work begins in the same frame; extrude exact previews retain their short input-settle window. Final commits still use full-quality geometry and tessellation.
* **Less memory copying on large models:** feature checkpoints, live previews, undo snapshots, and unchanged viewport geometry now share immutable mesh data instead of repeatedly copying every vertex and triangle.
  * Warm B-Rep evaluation checkpoints now use copy-on-write ownership, so handing a long model history to the background evaluator does not deep-copy its geometry on the interface thread.
* **Performance measurements for long histories:** repeatable cold- and warm-build benchmarks now track modeling performance so future changes can be checked for regressions.
* **No more hover synchronization stalls:** face highlighting now reads the GPU pick buffer asynchronously through a small ring of staging buffers instead of waiting for the GPU on the interface thread.
* **Cheaper idle frames:** unchanged GPU scenes reuse their existing offscreen texture, and save/export preparation no longer competes with viewport input on the interface thread.
* **Responsive cancellation inside geometry work:** superseded booleans and tessellation passes now receive the active cancellation signal inside the kernel and return no partial geometry.
* **Warm caches survive background evaluation:** completed worker checkpoints are installed back into the document, so the next edit can resume from the latest matching feature prefix.

---

### 💾 Saving & Project Files

* **New `.zcad` v3 recipe format:** project files now store a deterministic, explicitly versioned model recipe instead of serializing ZeroCAD's internal graph storage. Features, dependencies, attachments, variables, and document settings are written in a stable order, making the format safer to evolve and easier to validate.
* **Compact and fast-open project choices:** `.zcad` remains the small authoritative model file, while `.zcadh` can include display meshes plus reusable B-Rep feature checkpoints for a faster first view and faster first edit without changing the underlying editable recipe.
* **Bounded hydrated caches:** `.zcadh` keeps the final state, expensive feature prefixes, and history milestones under a configurable 128 MB default limit. Every checkpoint is recipe-, visibility-, and kernel-ABI-bound; stale or unhealthy cache data is ignored safely.
* **Background saving and export:** thumbnail rendering, project compression, hashing, STL/3MF generation, and disk writes now run away from the interface thread. A small delayed activity indicator appears only for work that takes long enough to notice.
* **Safer saves:** ZeroCAD now writes and synchronizes a temporary sibling file before replacing the existing project. If the final replacement fails, the previous file is restored instead of leaving a partially written model.
* **Reliable cache validation:** optional cached geometry is tied to the exact model recipe with a BLAKE3 content hash, so stale display data cannot silently be used for a changed model.

---

### 🧪 Reliability & Testing

* **Cancellation and cache regression tests:** automated tests verify that superseded evaluations stop cleanly and unchanged meshes are shared rather than copied.
* **Generated geometry stress tests:** property-based tests exercise many valid through-hole dimensions and check that results stay finite, watertight, healthy, and repeatable.
* **Geometry kernel required in CI:** the complete OpenRCAD workspace now builds and runs its tests as a required Linux CI job alongside ZeroCAD, catching kernel regressions before they reach the application.
