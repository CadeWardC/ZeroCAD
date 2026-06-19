//! STL (stereolithography) triangle-mesh export.
//!
//! Both the human-readable ASCII form and the compact binary form are supported.
//! Per-triangle facet normals are computed from the cross product
//! `(v1 - v0) × (v2 - v0)`, matching every other STL exporter. This maps to
//! OCCT's `StlAPI_Writer`.

use std::io::{self, Write};

use openrcad_foundation::{Pnt, Vec};
use openrcad_mesh::TriangleMesh;

/// Write `mesh` as an ASCII STL document to `out`, with the solid named `name`.
///
/// `out` is anything that implements [`io::Write`] — a `File`, a `Vec<u8>`,
/// stdout, …. To get the document as a string, write to a `Vec<u8>` and decode
/// it (ASCII STL is UTF-7-clean).
pub fn write_stl_ascii(mesh: &TriangleMesh, name: &str, out: &mut dyn Write) -> io::Result<()> {
    writeln!(out, "solid {name}")?;
    for tri in &mesh.triangles {
        let [a, b, c] = tri.map(|i| {
            mesh.vertices
                .get(i as usize)
                .copied()
                .expect("triangle index in range")
        });
        let n = facet_normal(a, b, c);
        writeln!(out, "  facet normal {} {} {}", f(n.x()), f(n.y()), f(n.z()))?;
        writeln!(out, "    outer loop")?;
        writeln!(out, "      vertex {} {} {}", f(a.x()), f(a.y()), f(a.z()))?;
        writeln!(out, "      vertex {} {} {}", f(b.x()), f(b.y()), f(b.z()))?;
        writeln!(out, "      vertex {} {} {}", f(c.x()), f(c.y()), f(c.z()))?;
        writeln!(out, "    endloop")?;
        writeln!(out, "  endfacet")?;
    }
    writeln!(out, "endsolid {name}")?;
    Ok(())
}

/// Write `mesh` as a binary STL document to `out` (little-endian IEEE-754,
/// 80-byte header + 4-byte count + 50 bytes/triangle).
pub fn write_stl_binary(mesh: &TriangleMesh, out: &mut dyn Write) -> io::Result<()> {
    // 80-byte header (blank), then little-endian u32 triangle count.
    out.write_all(&[0u8; 80])?;
    out.write_all(&(mesh.triangles.len() as u32).to_le_bytes())?;
    for tri in &mesh.triangles {
        let [a, b, c] = tri.map(|i| {
            mesh.vertices
                .get(i as usize)
                .copied()
                .expect("triangle index in range")
        });
        let n = facet_normal(a, b, c);
        write_f32(out, n.x())?;
        write_f32(out, n.y())?;
        write_f32(out, n.z())?;
        write_f32_pnt(out, a)?;
        write_f32_pnt(out, b)?;
        write_f32_pnt(out, c)?;
        out.write_all(&[0u8, 0u8])?; // attribute byte count
    }
    Ok(())
}

/// The unit facet normal of triangle `a,b,c` (CCW front face), or the zero
/// vector for a degenerate triangle (which STL writes as `0 0 0`).
fn facet_normal(a: Pnt, b: Pnt, c: Pnt) -> Vec {
    let ab: Vec = b - a;
    let ac: Vec = c - a;
    ab.cross(&ac).normalized_vec().unwrap_or(Vec::ZERO)
}

#[inline]
fn write_f32(out: &mut dyn Write, v: f64) -> io::Result<()> {
    out.write_all(&(v as f32).to_le_bytes())
}

#[inline]
fn write_f32_pnt(out: &mut dyn Write, p: Pnt) -> io::Result<()> {
    write_f32(out, p.x())?;
    write_f32(out, p.y())?;
    write_f32(out, p.z())
}

/// Format an f64 for STL ASCII with enough digits to round-trip, trimming a
/// trailing `-0.0` to `0`.
fn f(v: f64) -> String {
    let mut s = format!("{:.6}", v);
    // Strip a negative zero so the output reads "0.000000" not "-0.000000".
    if s.trim_start_matches('-')
        .trim_start_matches('0')
        .trim_start_matches(['.', '0'])
        .is_empty()
    {
        s = s.trim_start_matches('-').to_string();
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_tri_square() -> TriangleMesh {
        // A unit square in the XY plane, split into two triangles. Normal +Z.
        TriangleMesh::from_buffers(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(1.0, 1.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
    }

    #[test]
    fn ascii_stl_has_expected_facets() {
        let m = two_tri_square();
        let mut buf: std::vec::Vec<u8> = std::vec::Vec::new();
        write_stl_ascii(&m, "square", &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.starts_with("solid square\n"));
        assert!(text.ends_with("endsolid square\n"));
        // Two facets, each with a +Z normal.
        assert_eq!(text.matches("facet normal").count(), 2);
        assert!(text.contains("facet normal 0.000000 0.000000 1.000000"));
    }

    #[test]
    fn binary_stl_header_and_count() {
        let m = two_tri_square();
        let mut buf: std::vec::Vec<u8> = std::vec::Vec::new();
        write_stl_binary(&m, &mut buf).unwrap();
        // 80 header + 4 count + 2*50 triangles = 184 bytes.
        assert_eq!(buf.len(), 80 + 4 + 2 * 50);
        let count = u32::from_le_bytes(buf[80..84].try_into().unwrap());
        assert_eq!(count, 2);
        // First normal is +Z.
        let nx = f32::from_le_bytes(buf[84..88].try_into().unwrap());
        let nz = f32::from_le_bytes(buf[92..96].try_into().unwrap());
        assert!((nx - 0.0).abs() < 1e-5);
        assert!((nz - 1.0).abs() < 1e-5);
    }

    #[test]
    fn degenerate_triangle_has_zero_normal() {
        // Collinear vertices -> zero normal, no panic.
        let n = facet_normal(
            Pnt::origin(),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(2.0, 0.0, 0.0),
        );
        assert_eq!(n, Vec::ZERO);
    }

    #[test]
    fn negative_zero_is_trimmed() {
        assert_eq!(f(-0.0), "0.000000");
        assert_eq!(f(0.0), "0.000000");
        assert_eq!(f(-1.5), "-1.500000");
    }
}
