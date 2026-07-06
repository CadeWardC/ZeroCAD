//! 3MF (3D Manufacturing Format) export.
//!
//! A 3MF file is an OPC package: a ZIP archive holding `[Content_Types].xml`,
//! `_rels/.rels`, and the model XML at `3D/3dmodel.model`. The workspace's
//! no-external-dependencies rule means the ZIP container is written by hand —
//! entries are STOREd (no compression; the XML is small and slicers don't
//! care), which needs only local headers, a central directory, and CRC-32.

use openrcad_mesh::TriangleMesh;
use std::io::{self, Write};

/// Write `objects` — `(name, mesh)` pairs — as a 3MF package to `path`.
/// Units are millimeters (ZeroCAD's base unit).
pub fn write_3mf(objects: &[(String, &TriangleMesh)], path: &str) -> io::Result<()> {
    let bytes = to_3mf_bytes(objects);
    std::fs::write(path, bytes)
}

/// The full 3MF package as bytes (see [`write_3mf`]).
pub fn to_3mf_bytes(objects: &[(String, &TriangleMesh)]) -> Vec<u8> {
    let mut zip = ZipWriter::new();
    zip.add_file("[Content_Types].xml", CONTENT_TYPES.as_bytes());
    zip.add_file("_rels/.rels", RELS.as_bytes());
    zip.add_file("3D/3dmodel.model", model_xml(objects).as_bytes());
    zip.finish()
}

const CONTENT_TYPES: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#;

const RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#;

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn model_xml(objects: &[(String, &TriangleMesh)]) -> String {
    let mut xml = String::with_capacity(4096);
    xml.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    xml.push('\n');
    xml.push_str(
        r#"<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">"#,
    );
    xml.push_str("\n <resources>\n");
    for (i, (name, mesh)) in objects.iter().enumerate() {
        let id = i + 1;
        xml.push_str(&format!(
            "  <object id=\"{id}\" type=\"model\" name=\"{}\">\n   <mesh>\n    <vertices>\n",
            xml_escape(name)
        ));
        for v in &mesh.vertices {
            xml.push_str(&format!(
                "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
                v.x(),
                v.y(),
                v.z()
            ));
        }
        xml.push_str("    </vertices>\n    <triangles>\n");
        for t in &mesh.triangles {
            xml.push_str(&format!(
                "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                t[0], t[1], t[2]
            ));
        }
        xml.push_str("    </triangles>\n   </mesh>\n  </object>\n");
    }
    xml.push_str(" </resources>\n <build>\n");
    for i in 0..objects.len() {
        xml.push_str(&format!("  <item objectid=\"{}\"/>\n", i + 1));
    }
    xml.push_str(" </build>\n</model>\n");
    xml
}

/// IEEE CRC-32 (the ZIP polynomial), bytewise table-free variant.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Minimal STORE-only ZIP writer: local headers + central directory + EOCD.
struct ZipWriter {
    out: Vec<u8>,
    central: Vec<u8>,
    entries: u16,
}

impl ZipWriter {
    fn new() -> Self {
        Self {
            out: Vec::new(),
            central: Vec::new(),
            entries: 0,
        }
    }

    fn add_file(&mut self, name: &str, data: &[u8]) {
        let offset = self.out.len() as u32;
        let crc = crc32(data);
        let n = data.len() as u32;
        let name_b = name.as_bytes();

        // Local file header.
        let w = &mut self.out;
        let _ = w.write_all(&0x0403_4B50u32.to_le_bytes());
        let _ = w.write_all(&20u16.to_le_bytes()); // version needed
        let _ = w.write_all(&0u16.to_le_bytes()); // flags
        let _ = w.write_all(&0u16.to_le_bytes()); // method: STORE
        let _ = w.write_all(&0u16.to_le_bytes()); // mod time
        let _ = w.write_all(&0x21u16.to_le_bytes()); // mod date (1980-01-01)
        let _ = w.write_all(&crc.to_le_bytes());
        let _ = w.write_all(&n.to_le_bytes()); // compressed size
        let _ = w.write_all(&n.to_le_bytes()); // uncompressed size
        let _ = w.write_all(&(name_b.len() as u16).to_le_bytes());
        let _ = w.write_all(&0u16.to_le_bytes()); // extra len
        let _ = w.write_all(name_b);
        let _ = w.write_all(data);

        // Central directory record.
        let c = &mut self.central;
        let _ = c.write_all(&0x0201_4B50u32.to_le_bytes());
        let _ = c.write_all(&20u16.to_le_bytes()); // version made by
        let _ = c.write_all(&20u16.to_le_bytes()); // version needed
        let _ = c.write_all(&0u16.to_le_bytes()); // flags
        let _ = c.write_all(&0u16.to_le_bytes()); // method
        let _ = c.write_all(&0u16.to_le_bytes()); // mod time
        let _ = c.write_all(&0x21u16.to_le_bytes()); // mod date
        let _ = c.write_all(&crc.to_le_bytes());
        let _ = c.write_all(&n.to_le_bytes());
        let _ = c.write_all(&n.to_le_bytes());
        let _ = c.write_all(&(name_b.len() as u16).to_le_bytes());
        let _ = c.write_all(&0u16.to_le_bytes()); // extra len
        let _ = c.write_all(&0u16.to_le_bytes()); // comment len
        let _ = c.write_all(&0u16.to_le_bytes()); // disk number
        let _ = c.write_all(&0u16.to_le_bytes()); // internal attrs
        let _ = c.write_all(&0u32.to_le_bytes()); // external attrs
        let _ = c.write_all(&offset.to_le_bytes());
        let _ = c.write_all(name_b);

        self.entries += 1;
    }

    fn finish(mut self) -> Vec<u8> {
        let cd_offset = self.out.len() as u32;
        let cd_size = self.central.len() as u32;
        self.out.extend_from_slice(&self.central);
        // End of central directory.
        let w = &mut self.out;
        let _ = w.write_all(&0x0605_4B50u32.to_le_bytes());
        let _ = w.write_all(&0u16.to_le_bytes()); // this disk
        let _ = w.write_all(&0u16.to_le_bytes()); // cd disk
        let _ = w.write_all(&self.entries.to_le_bytes());
        let _ = w.write_all(&self.entries.to_le_bytes());
        let _ = w.write_all(&cd_size.to_le_bytes());
        let _ = w.write_all(&cd_offset.to_le_bytes());
        let _ = w.write_all(&0u16.to_le_bytes()); // comment len
        self.out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;

    fn tri_mesh() -> TriangleMesh {
        let mut m = TriangleMesh::new();
        m.vertices = vec![
            Pnt::new(0.0, 0.0, 0.0),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(0.0, 1.0, 0.0),
        ];
        m.triangles = vec![[0, 1, 2]];
        m
    }

    #[test]
    fn package_has_zip_structure_and_model() {
        let mesh = tri_mesh();
        let bytes = to_3mf_bytes(&[("part".to_string(), &mesh)]);
        // Local header magic at the start, EOCD magic near the end.
        assert_eq!(&bytes[0..4], &0x0403_4B50u32.to_le_bytes());
        let eocd = 0x0605_4B50u32.to_le_bytes();
        assert!(
            bytes.windows(4).any(|w| w == eocd),
            "no end-of-central-directory record"
        );
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("3D/3dmodel.model"));
        assert!(text.contains("<triangle v1=\"0\" v2=\"1\" v3=\"2\"/>"));
        assert!(text.contains("unit=\"millimeter\""));
    }

    #[test]
    fn crc32_matches_known_vector() {
        // CRC-32 of "123456789" is the classic check value 0xCBF43926.
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }
}
