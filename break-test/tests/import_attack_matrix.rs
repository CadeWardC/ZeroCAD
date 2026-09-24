//! Small, deterministic hostile-file corpus. No external files or network needed.
mod common;
use common::*;
use zerocad_core::*;

fn document_bytes() -> Vec<u8> {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box", 10., 20., 30.);
    write_document_to_vec(
        &Document::from_graph(g, Unit::Millimeter),
        &Default::default(),
        &Default::default(),
    )
    .unwrap()
}

#[test]
fn every_container_truncation_is_rejected() {
    let bytes = document_bytes();
    for end in 0..bytes.len() {
        assert!(
            read_document_from_slice(&bytes[..end], &Default::default()).is_err(),
            "accepted truncation at {end}/{}",
            bytes.len()
        );
    }
    let loaded = read_document_from_slice(&bytes, &Default::default()).unwrap();
    assert!((total_volume(&common::eval(loaded.document.evaluator_graph())) - 6000.).abs() < 0.01);
}

#[test]
fn every_header_bit_flip_is_rejected() {
    let bytes = document_bytes();
    for index in 0..32 {
        for bit in 0..8 {
            let mut corrupt = bytes.clone();
            corrupt[index] ^= 1 << bit;
            assert!(
                read_document_from_slice(&corrupt, &Default::default()).is_err(),
                "accepted corrupt header byte {index}, bit {bit}"
            );
        }
    }
}

#[test]
fn hostile_section_lengths_and_offsets_are_rejected_before_allocation() {
    let bytes = document_bytes();
    // Container table: id/flags/codec, offset, stored size, decoded size.
    for field in [4, 12, 20] {
        for value in [u64::MAX, u64::MAX / 2, 1 << 40] {
            let mut corrupt = bytes.clone();
            corrupt[32 + field..32 + field + 8].copy_from_slice(&value.to_le_bytes());
            assert!(
                read_document_from_slice(&corrupt, &Default::default()).is_err(),
                "field={field} value={value}"
            );
        }
    }
}

#[test]
fn seeded_garbage_never_parses_as_a_document_or_solid_stl() {
    let mut state = 0x53c0_19a7_u32;
    for length in 0..512 {
        let bytes: Vec<u8> = (0..length)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                state as u8
            })
            .collect();
        assert!(
            read_document_from_slice(&bytes, &Default::default()).is_err(),
            "document seed length {length}"
        );
        assert!(
            stl::read_stl_mesh(&bytes).is_err(),
            "STL seed length {length}"
        );
    }
}

#[test]
fn binary_stl_triangle_counts_cannot_overflow_or_allocate_unboundedly() {
    for count in [1_u32, 2, 1_000_000, u32::MAX / 50, u32::MAX] {
        for solid_header in [false, true] {
            let mut bytes = vec![0; 84];
            if solid_header {
                bytes[..5].copy_from_slice(b"solid");
            }
            bytes[80..84].copy_from_slice(&count.to_le_bytes());
            assert!(
                stl::read_stl_mesh(&bytes).is_err(),
                "accepted count {count} without triangles"
            );
        }
    }
}

#[test]
fn binary_stl_rejects_nonfinite_coordinates_in_every_vertex_slot() {
    for slot in 3..12 {
        for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut bytes = vec![0; 134];
            bytes[80..84].copy_from_slice(&1_u32.to_le_bytes());
            // A valid triangle before injection: (0,0,0), (1,0,0), (0,1,0).
            bytes[84 + 6 * 4..84 + 7 * 4].copy_from_slice(&1_f32.to_le_bytes());
            bytes[84 + 10 * 4..84 + 11 * 4].copy_from_slice(&1_f32.to_le_bytes());
            bytes[84 + slot * 4..84 + (slot + 1) * 4].copy_from_slice(&value.to_le_bytes());
            assert!(
                stl::read_stl_mesh(&bytes).is_err(),
                "slot={slot}, value={value}"
            );
        }
    }
}
