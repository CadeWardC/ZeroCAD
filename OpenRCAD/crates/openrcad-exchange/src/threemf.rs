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

/// A reusable assembly definition composed from one or more entries in the
/// mesh-object array passed to [`to_3mf_assembly_bytes`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ThreeMfDefinition {
    pub name: String,
    pub body_object_indices: Vec<usize>,
}

/// One transformed occurrence in the 3MF build.
///
/// `transform` follows the 3MF row-major 3x4 attribute order:
/// `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`.
#[derive(Clone, Debug, PartialEq)]
pub struct ThreeMfBuildItem {
    pub definition_index: usize,
    pub transform: [f64; 12],
}

/// Write a deduplicated 3MF assembly.
///
/// Every input mesh is emitted exactly once. Multi-body definitions become
/// component objects, while each build item references its definition with a
/// rigid transform. This keeps repeated occurrences compact instead of
/// flattening their triangles.
pub fn to_3mf_assembly_bytes(
    mesh_objects: &[(String, &TriangleMesh)],
    definitions: &[ThreeMfDefinition],
    build_items: &[ThreeMfBuildItem],
) -> io::Result<Vec<u8>> {
    if definitions
        .iter()
        .any(|definition| definition.body_object_indices.is_empty())
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "3MF definitions must reference at least one mesh object",
        ));
    }
    if definitions.iter().any(|definition| {
        definition
            .body_object_indices
            .iter()
            .any(|index| *index >= mesh_objects.len())
    }) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "3MF definition references an unknown mesh object",
        ));
    }
    if build_items.iter().any(|item| {
        item.definition_index >= definitions.len()
            || !item.transform.iter().all(|value| value.is_finite())
    }) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "3MF build item has an invalid definition or transform",
        ));
    }

    let model = assembly_model_xml(mesh_objects, definitions, build_items);
    let mut zip = ZipWriter::new();
    zip.add_file("[Content_Types].xml", CONTENT_TYPES.as_bytes());
    zip.add_file("_rels/.rels", RELS.as_bytes());
    zip.add_file("3D/3dmodel.model", model.as_bytes());
    Ok(zip.finish())
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

fn write_mesh_object(xml: &mut String, id: usize, name: &str, mesh: &TriangleMesh) {
    xml.push_str(&format!(
        "  <object id=\"{id}\" type=\"model\" name=\"{}\">\n   <mesh>\n    <vertices>\n",
        xml_escape(name)
    ));
    for vertex in &mesh.vertices {
        xml.push_str(&format!(
            "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
            vertex.x(),
            vertex.y(),
            vertex.z()
        ));
    }
    xml.push_str("    </vertices>\n    <triangles>\n");
    for triangle in &mesh.triangles {
        xml.push_str(&format!(
            "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
            triangle[0], triangle[1], triangle[2]
        ));
    }
    xml.push_str("    </triangles>\n   </mesh>\n  </object>\n");
}

fn assembly_model_xml(
    mesh_objects: &[(String, &TriangleMesh)],
    definitions: &[ThreeMfDefinition],
    build_items: &[ThreeMfBuildItem],
) -> String {
    let mut xml = String::with_capacity(4096);
    xml.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    xml.push('\n');
    xml.push_str(
        r#"<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">"#,
    );
    xml.push_str("\n <resources>\n");
    for (index, (name, mesh)) in mesh_objects.iter().enumerate() {
        write_mesh_object(&mut xml, index + 1, name, mesh);
    }

    let multi_body_count = definitions
        .iter()
        .filter(|definition| definition.body_object_indices.len() > 1)
        .count();
    let mut next_component_id = mesh_objects.len() + 1;
    let mut definition_object_ids = Vec::with_capacity(definitions.len());
    for definition in definitions {
        if definition.body_object_indices.len() == 1 {
            definition_object_ids.push(definition.body_object_indices[0] + 1);
            continue;
        }
        let object_id = next_component_id;
        next_component_id += 1;
        definition_object_ids.push(object_id);
        xml.push_str(&format!(
            "  <object id=\"{object_id}\" type=\"model\" name=\"{}\">\n   <components>\n",
            xml_escape(&definition.name)
        ));
        for mesh_index in &definition.body_object_indices {
            xml.push_str(&format!(
                "    <component objectid=\"{}\"/>\n",
                mesh_index + 1
            ));
        }
        xml.push_str("   </components>\n  </object>\n");
    }
    debug_assert_eq!(next_component_id, mesh_objects.len() + multi_body_count + 1);

    xml.push_str(" </resources>\n <build>\n");
    for item in build_items {
        let transform = item
            .transform
            .iter()
            .map(|value| {
                let value = if *value == 0.0 { 0.0 } else { *value };
                value.to_string()
            })
            .collect::<Vec<_>>()
            .join(" ");
        xml.push_str(&format!(
            "  <item objectid=\"{}\" transform=\"{transform}\"/>\n",
            definition_object_ids[item.definition_index]
        ));
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

    #[test]
    fn assembly_reuses_meshes_and_writes_transformed_build_items() {
        let first = tri_mesh();
        let second = tri_mesh();
        let bytes = to_3mf_assembly_bytes(
            &[
                ("body-a".to_string(), &first),
                ("body-b".to_string(), &second),
            ],
            &[ThreeMfDefinition {
                name: "fixture".into(),
                body_object_indices: vec![0, 1],
            }],
            &[
                ThreeMfBuildItem {
                    definition_index: 0,
                    transform: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                },
                ThreeMfBuildItem {
                    definition_index: 0,
                    transform: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 5.0, 6.0, 7.0],
                },
            ],
        )
        .unwrap();
        let text = String::from_utf8_lossy(&bytes);
        assert_eq!(text.matches("<mesh>").count(), 2);
        assert_eq!(text.matches("<component objectid=").count(), 2);
        assert_eq!(text.matches("<item objectid=\"3\"").count(), 2);
        assert!(text.contains("1 0 0 0 1 0 0 0 1 5 6 7"));
    }
}
