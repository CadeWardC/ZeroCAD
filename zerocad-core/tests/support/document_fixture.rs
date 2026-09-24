//! Keep frozen files as reader-compatibility evidence while comparing recipes
//! through the current canonical writer (which stamps current cache ABIs).
pub fn canonical_fixture_bytes(path: &std::path::Path) -> Vec<u8> {
    let loaded = zerocad_core::read_document_file(path, &Default::default())
        .expect("read frozen document with the current compatible reader");
    zerocad_core::write_document_to_vec(&loaded.document, &Default::default(), &Default::default())
        .expect("write frozen recipe with current disposable cache metadata")
}
