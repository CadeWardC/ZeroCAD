//! Milestone demo: tessellate a primitive box and spin it, flat-shaded.
//!
//! Run with:
//!
//! ```text
//! cargo run -p openrcad-render --example spinning_box
//! ```

use openrcad_foundation::Pnt;
use openrcad_primitives::make_box;

fn main() {
    println!("Controls: left-drag orbit · middle/right-drag pan · scroll zoom.");
    println!("Left-click a face to select it (the face id prints here).");
    let solid = make_box(&Pnt::new(-1.0, -1.0, -1.0), 2.0, 2.0, 2.0);
    openrcad_render::run_solid(&solid, 0.02);
}
