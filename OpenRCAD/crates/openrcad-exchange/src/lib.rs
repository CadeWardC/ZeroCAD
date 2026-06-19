#![forbid(unsafe_code)]
//! Data exchange for OpenRCAD (OCCT `TKSTEP` / `TKSTL`).
//!
//! Reads and writes CAD data. STL export supports ASCII and binary output from a
//! [`TriangleMesh`](openrcad_mesh::TriangleMesh). STEP read/write supports the
//! OpenRCAD B-Rep subset used by the primitive and analytic-surface tests.

pub mod step_reader;
pub mod step_writer;
pub mod stl;

pub use step_reader::read_step;
pub use step_writer::write_step;
pub use stl::{write_stl_ascii, write_stl_binary};

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax2, Dir, Pnt};
    use openrcad_primitives::{make_box, make_cylinder, make_sphere};

    fn assert_close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-5, "Expected {} to be close to {}", a, b);
    }

    fn check_roundtrip(solid: &openrcad_topo::Solid, name: &str) {
        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join(format!("openrcad_test_{}.stp", name));
        let path_str = path.to_str().unwrap();

        // Write to STEP
        write_step(solid, path_str).expect("Failed to write STEP");

        // Read from STEP
        let parsed = read_step(path_str).expect("Failed to read STEP");

        // Clean up
        let _ = std::fs::remove_file(path);

        // Compare topological counts
        assert_eq!(solid.vertex_count(), parsed.vertex_count());
        assert_eq!(solid.edge_count(), parsed.edge_count());
        assert_eq!(solid.face_count(), parsed.face_count());

        // Compare bounding box
        let (lo_orig, hi_orig) = solid.bounding_box().corners().unwrap();
        let (lo_parsed, hi_parsed) = parsed.bounding_box().corners().unwrap();
        assert_close(lo_orig.x(), lo_parsed.x());
        assert_close(lo_orig.y(), lo_parsed.y());
        assert_close(lo_orig.z(), lo_parsed.z());
        assert_close(hi_orig.x(), hi_parsed.x());
        assert_close(hi_orig.y(), hi_parsed.y());
        assert_close(hi_orig.z(), hi_parsed.z());
    }

    #[test]
    fn step_box_roundtrip() {
        let s = make_box(&Pnt::origin(), 2.0, 3.0, 4.0);
        check_roundtrip(&s, "box");
    }

    #[test]
    fn step_cylinder_roundtrip() {
        let s = make_cylinder(&Ax2::new(Pnt::origin(), Dir::new(0.0, 0.0, 1.0)), 1.5, 5.0);
        check_roundtrip(&s, "cylinder");
    }

    #[test]
    fn step_sphere_roundtrip() {
        let s = make_sphere(&Pnt::origin(), 2.5);
        check_roundtrip(&s, "sphere");
    }
}
