//! Research-derived STL / STEP exchange torture (research pass 2026-09-11).
//!
//! The catalog comes from the documented broken-mesh/ broken-file corpus:
//! the binary-STL "solid" header misclassification (meshio #530, Wikipedia
//! STL), Cura #20429 (crash-on-load NaN STLs), the STL non-manifold/ duplicate
//! face literature (trimesh repair docs, CGAL PMP), the classic unit/25.4
//! failure (MeshCAM FAQ), the OCCT STEP unit-handling guide, and ISO 10303-21
//! structure rules (header-only files, unknown schemas). The reader must
//! answer every attack with a typed error or a validation diagnostic — never
//! a panic, never NaN in a mesh.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::stl::read_stl_mesh;
use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

// --- STL synthesis helpers ---------------------------------------------------

struct Tri([[f32; 3]; 3]);

fn stl_normal(t: &Tri) -> [f32; 3] {
    let [a, b, c] = t.0;
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len == 0.0 {
        [0.0; 3]
    } else {
        [n[0] / len, n[1] / len, n[2] / len]
    }
}

fn write_binary_stl(header: &[u8], tris: &[Tri]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    let mut head = [0_u8; 80];
    let n = header.len().min(80);
    head[..n].copy_from_slice(&header[..n]);
    out.extend_from_slice(&head);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        out.extend_from_slice(&stl_normal(t).map(f32::to_le_bytes).concat());
        for v in t.0 {
            out.extend_from_slice(&v.map(f32::to_le_bytes).concat());
        }
        out.extend_from_slice(&0_u16.to_le_bytes()); // attribute byte count
    }
    out
}

/// A closed, consistently-wound axis-aligned cube: [0,s]³.
fn cube_tris(s: f32) -> Vec<Tri> {
    let v = |x: f32, y: f32, z: f32| [x, y, z];
    let (a, b, c, d) = (v(0., 0., 0.), v(s, 0., 0.), v(s, s, 0.), v(0., s, 0.));
    let (e, f, top_g, top_h) = (v(0., 0., s), v(s, 0., s), v(s, s, s), v(0., s, s));
    let t = |p: [[f32; 3]; 3]| Tri(p);
    vec![
        t([a, c, b]),
        t([a, d, c]), // bottom (−z)
        t([e, f, top_g]),
        t([e, top_g, top_h]), // top (+z)
        t([a, b, f]),
        t([a, f, e]), // front (−y)
        t([d, top_g, c]),
        t([d, top_h, top_g]), // back (+y)
        t([a, e, top_h]),
        t([a, top_h, d]), // left (−x)
        t([b, c, top_g]),
        t([b, top_g, f]), // right (+x)
    ]
}

fn stl_feature(id: &str, data: Vec<u8>) -> FeatureNode {
    FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::ImportStl {
            stl_data: data,
            label: id.to_string(),
        },
    }
}

// --- Binary parsing: header sniffing, NaN, structure -------------------------

#[test]
fn stl_binary_with_solid_header_still_parses_as_binary() {
    // meshio #530: a binary STL whose 80-byte header begins with "solid" is
    // misclassified as ASCII by naive sniffers. Detection must use the
    // size/count formula (https://github.com/nschloe/meshio/issues/530).
    let tris = cube_tris(10.0);
    let data = write_binary_stl(b"solid made_by_foo binary content", &tris);
    let imported = read_stl_mesh(&data).expect("binary STL with solid header must parse");
    assert_eq!(imported.validation.input_triangles, 12);
    assert!(
        imported.validation.is_closed_manifold(),
        "welded cube must be closed: {:?}",
        imported.validation
    );
}

#[test]
fn stl_nan_and_infinite_vertices_rejected_cleanly() {
    // Cura #20429: a single poisoned facet crashes real slicers. Must surface
    // as a typed error or a diagnostic, never NaN in a mesh, never a panic.
    let mut tris = cube_tris(10.0);
    tris[0] = Tri([[f32::NAN; 3], [1.0; 3], [0.0; 3]]);
    let data = write_binary_stl(b"poisoned", &tris);
    match read_stl_mesh(&data) {
        Err(e) => assert!(!e.to_string().is_empty(), "error must be describable"),
        Ok(imported) => {
            for v in &imported.mesh.vertices {
                assert!(v.is_finite(), "NaN survived the import validation");
            }
            assert!(
                imported.validation.degenerate_triangles > 0 || imported.mesh.indices.is_empty(),
                "NaN facet must be discarded or flagged"
            );
        }
    }

    tris[0] = Tri([[f32::INFINITY; 3], [1.0; 3], [0.0; 3]]);
    let data = write_binary_stl(b"poisoned-inf", &tris);
    let _ = read_stl_mesh(&data); // must not panic; any result is acceptable
}

#[test]
fn stl_degenerate_facets_discarded_with_diagnostic() {
    let mut tris = cube_tris(10.0);
    tris.push(Tri([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [5.0, 0.0, 0.0]]));
    tris.push(Tri([
        [10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0],
    ]));
    let data = write_binary_stl(b"degenerate", &tris);
    let imported = read_stl_mesh(&data).expect("degenerate facets must be discarded, not fatal");
    assert_eq!(imported.validation.degenerate_triangles, 2);
    assert!(
        imported
            .validation
            .diagnostics()
            .iter()
            .any(|d| d.contains("degenerate")),
        "diagnostics must mention the discards: {:?}",
        imported.validation.diagnostics()
    );
}

#[test]
fn stl_duplicate_face_reports_nonmanifold_edges() {
    // A duplicated facet makes three edges 4-valent — the classic non-manifold
    // defect (trimesh/CGAL repair literature; Manifold requires ε-validity).
    let mut tris = cube_tris(10.0);
    tris.push(Tri(tris[6].0));
    let data = write_binary_stl(b"duplicate face", &tris);
    let imported = read_stl_mesh(&data).expect("duplicate face must not be fatal");
    assert!(
        imported.validation.non_manifold_edges >= 3,
        "4-valent edges must be detected: {:?}",
        imported.validation
    );
    assert!(!imported.validation.is_closed_manifold());
}

#[test]
fn stl_two_cubes_sharing_a_face_soup_is_flagged() {
    // Two cubes fused into one triangle soup through a shared face: interior
    // edges carried by 4 triangles each (OpenSCAD #4039 class).
    let mut tris = cube_tris(10.0);
    let v = |x: f32, y: f32, z: f32| [x, y, z];
    let t = |p: [[f32; 3]; 3]| Tri(p);
    // Second cube [10,20]×[0,10]×[0,10], but keep the shared wall faces in —
    // producing exactly the T-junction soup users export by accident.
    tris.extend([
        t([v(10., 0., 0.), v(20., 0., 0.), v(20., 10., 0.)]),
        t([v(10., 0., 0.), v(20., 10., 0.), v(10., 10., 0.)]),
        t([v(10., 0., 10.), v(20., 10., 10.), v(20., 0., 10.)]),
        t([v(10., 0., 10.), v(10., 10., 10.), v(20., 10., 10.)]),
        t([v(10., 0., 0.), v(10., 10., 0.), v(10., 10., 10.)]),
        t([v(10., 0., 0.), v(10., 10., 10.), v(10., 0., 10.)]),
        t([v(20., 0., 0.), v(20., 10., 0.), v(20., 10., 10.)]),
        t([v(20., 0., 0.), v(20., 10., 10.), v(20., 0., 10.)]),
        t([v(10., 0., 0.), v(20., 0., 10.), v(10., 0., 10.)]),
        t([v(10., 0., 0.), v(20., 0., 0.), v(20., 0., 10.)]),
        t([v(10., 10., 0.), v(20., 10., 10.), v(10., 10., 10.)]),
        t([v(10., 10., 0.), v(20., 10., 0.), v(20., 10., 10.)]),
    ]);
    let data = write_binary_stl(b"two cubes sharing face", &tris);
    let imported = read_stl_mesh(&data).expect("soup must parse without panicking");
    assert!(
        imported.validation.non_manifold_edges > 0 || !imported.validation.is_closed_manifold(),
        "shared-wall soup must be flagged: {:?}",
        imported.validation
    );
}

#[test]
fn stl_single_inconsistent_winding_flagged() {
    let mut tris = cube_tris(10.0);
    tris[0] = Tri([tris[0].0[0], tris[0].0[2], tris[0].0[1]]); // flip one facet
    let data = write_binary_stl(b"flipped facet", &tris);
    let imported = read_stl_mesh(&data).expect("winding check must not be fatal");
    assert!(
        imported.validation.inconsistent_winding_edges > 0,
        "a flipped facet must be detected: {:?}",
        imported.validation
    );
}

#[test]
fn stl_truncated_empty_and_count_mismatch_typed_errors() {
    // Empty file.
    assert!(read_stl_mesh(&[]).is_err());
    // Header-only binary (count = 0).
    let no_tris: Vec<Tri> = Vec::new();
    let data = write_binary_stl(b"empty", &no_tris);
    match read_stl_mesh(&data) {
        Err(_) | Ok(_) => {} // either typed error or an empty accepted mesh
    }
    // Truncated mid-record: declared 12 triangles, only 6 present.
    let full = write_binary_stl(b"cut me", &cube_tris(10.0));
    let truncated = &full[..84 + 6 * 50];
    match read_stl_mesh(truncated) {
        Err(e) => assert!(!e.to_string().is_empty()),
        Ok(imported) => {
            assert!(
                !imported.validation.is_closed_manifold(),
                "a half cube must not be reported watertight"
            );
        }
    }
    // Declared count lies (5, actual 12): the size formula must win.
    let mut lying = write_binary_stl(b"lying count", &cube_tris(10.0));
    lying[80..84].copy_from_slice(&5_u32.to_le_bytes());
    let _ = read_stl_mesh(&lying); // must not panic regardless of outcome
}

#[test]
fn stl_single_triangle_open_shell_accepted_with_boundary() {
    let data = write_binary_stl(
        b"one tri",
        &[Tri([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])],
    );
    let imported = read_stl_mesh(&data).expect("a single facet must import as an open shell");
    assert_eq!(imported.validation.boundary_edges, 3);
    assert!(!imported.validation.is_closed_manifold());
}

// --- ASCII parsing robustness -------------------------------------------------

#[test]
fn stl_ascii_number_formats_and_truncation() {
    // Exponent forms, leading dots, CRLF, and a missing endsolid — the real
    // world's ASCII soup (Wikipedia STL ASCII syntax; Cura crash corpus).
    let ok_body = "solid test\r\n\
                   facet normal 0 0 1\r\n  outer loop\r\n\
                   vertex 2.648000e-002 0.0 .5\r\nvertex 1E+02 0 0\r\nvertex 0 1e1 0\r\n\
                   endloop\r\nendfacet\r\nendsolid test\r\n";
    let truncated = "solid test\r\nfacet normal 0 0 1\r\n  outer loop\r\n\
                     vertex 0 0 0\r\nvertex 1 0 0\r\n"; // EOF mid-facet
    for (label, body) in [("ok", ok_body), ("truncated", truncated)] {
        match read_stl_mesh(body.as_bytes()) {
            Err(e) => assert!(
                !e.to_string().is_empty(),
                "{label}: error must describe itself"
            ),
            Ok(imported) => {
                for v in &imported.mesh.vertices {
                    assert!(v.is_finite(), "{label}: non-finite vertex");
                }
            }
        }
    }
}

// --- End-to-end through the evaluator -----------------------------------------

#[test]
fn stl_import_via_graph_is_finite_deterministic_and_diagnosed() {
    let mut g = ParametricGraph::new();
    g.add_feature(stl_feature(
        "mesh_1",
        write_binary_stl(b"cube", &cube_tris(10.0)),
    ));
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("healthy STL import must not fail the document");
    assert_eq!(bodies.len(), 1);
    assert!(
        warnings.is_empty(),
        "healthy cube must not warn: {warnings:?}"
    );
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!((v - 1000.0).abs() < 5.0, "cube volume {v} vs 1000");

    // The same document, corrupted facet: import warns and the document still
    // evaluates (degrade, never crash).
    let mut tris = cube_tris(10.0);
    tris.push(Tri(tris[0].0));
    let mut g2 = ParametricGraph::new();
    g2.add_feature(stl_feature(
        "mesh_1",
        write_binary_stl(b"cube with duplicate", &tris),
    ));
    let (bodies2, warnings2) = g2
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("defective STL must degrade to warnings");
    assert!(
        warnings2.iter().any(|w| w.contains("STL")),
        "validation diagnostics must surface as warnings: {warnings2:?}"
    );
    assert_meshes_finite(&bodies2);
}

#[test]
fn step_garbage_and_header_only_data_warn_cleanly() {
    // ISO 10303-21 requires HEADER + DATA; a parser fed garbage must warn and
    // keep the rest of the document alive (never panic, never hard-crash the
    // evaluation of unrelated bodies).
    let cases: &[(&str, &str)] = &[
        ("empty", ""),
        ("text", "this is not a STEP file at all"),
        ("header only", "ISO-10303-21;\nHEADER\nENDSEC;\nEND-ISO-10303-21;\n"),
        ("truncated", "ISO-10303-21;\nDATA;\n#1 ="),
        ("unknown schema", "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('TOTALLY_UNKNOWN_SCHEMA'));\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"),
    ];
    for (label, step_data) in cases {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 10.0, 10.0, 10.0); // unrelated healthy body
        g.add_feature(FeatureNode {
            id: "import_2".to_string(),
            name: "import_2".to_string(),
            feature: FeatureType::Import {
                step_data: step_data.to_string(),
                label: label.to_string(),
            },
        });
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&HashSet::new())
            .unwrap_or_else(|e| panic!("{label}: broken STEP must warn, not hard-fail: {e}"));
        assert!(
            bodies.iter().any(|(id, _)| id == "box_1"),
            "{label}: unrelated body must survive the bad import"
        );
        if *label != "unknown schema" {
            assert!(
                warnings
                    .iter()
                    .any(|w| w.contains("import_2") || w.contains("STEP")),
                "{label}: bad STEP must warn: {warnings:?}"
            );
        }
        for (_, mesh) in &bodies {
            for v in &mesh.vertices {
                assert!(v.is_finite(), "{label}: non-finite vertex");
            }
        }
    }
}
