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
