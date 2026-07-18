# Phase 2 document architecture

Phase 2 separates the editable model recipe from runtime graph storage and from
disposable display/evaluation accelerators. This document is the persistence and
evaluation contract for contributors.

## Semantic model

`zerocad-core/src/document.rs` owns the stable semantic vocabulary:

- features have stable `FeatureKindId`s, payload versions, semantic-selector
  inputs, `SequenceKey`s, and active/suppressed state;
- bodies have stable ids and explicit ordered timelines;
- units and visibility are document presentation state, not geometry inputs;
- `SemanticSelector` resolves exact topology, then provenance, then semantic
  role. Geometric fallback is allowed only for unnamed legacy selections. A
  missing named selection fails instead of silently retargeting.

`Document` is the editable application root and privately owns
`ParametricGraph` as its efficient runtime projection. Each authoritative
`FeatureRecord` owns payload, inputs, ordering, suppression, and body membership;
`DocumentSemantics` owns only cross-feature body timelines and recorded output
provenance. Rebuilding the node map reconstructs derived scheduling state and
performs narrowly isolated legacy migration.

The feature registry is exhaustive. Adding a `FeatureType` requires a stable
kind id, payload version, evaluator kind, editor group, intrinsic dependency
mapping, and persistence DTO coverage.

Body evaluation uses one registry-driven contract: resolve the exact registered
evaluator, invoke into a candidate body state, validate that candidate,
record feature diagnostics/status, commit only healthy topology, then cache the
validated checkpoint. Feature-specific match arms are isolated behind that
boundary. Graph dependencies always precede consumers; `SequenceKey` orders
only simultaneously-ready features, so UI timeline order and dependency
legality remain independent.

The document browser groups features from `DocumentSemantics::bodies[*].timeline`
and exposes move-up/move-down editing. Runtime split outputs are display children
of those semantic bodies rather than inferred body ownership records.

## Stable feature DTOs

`zerocad-core/src/feature_dto.rs` maps every feature kind to a numeric-field CBOR
payload. Rust enum variant and field names are runtime implementation details and
must not become the persistent ABI. Unknown kinds, unsupported payload versions,
unknown numeric fields, duplicate fields, and missing required fields fail
decode explicitly.

STEP import bytes are required content-addressed assets. The feature payload
stores the asset hash and label; identical imports share one asset section.

## `.zcad` v5 profiles

The v5 container consists of a 32-byte header, 48-byte table entries, and
independent sections. Required sections are the manifest, recipe, and required
assets. Optional preview, mesh, and checkpoint sections are disposable.

Compact saves contain no mesh or B-Rep checkpoint data and permit only a tiny
preview. Hydrated saves add validated accelerators within one of the supported
bounded budgets. Optional sections can be removed or corrupted independently;
loading still returns the recipe with a diagnostic and rebuilds geometry.

Canonical document APIs are:

- `write_document`, `write_document_to_vec`, `write_document_file`;
- `read_document`, `read_document_from_slice`, `read_document_file`.

The file variants stream section-by-section and atomic writes preserve the
previous complete file until the new file is synced and ready to replace it.
Compatibility `write_zcad`/`read_zcad` APIs are deprecated and are not permitted
on GUI production paths.

## Cache and hash boundaries

The model hash covers canonical recipe bytes and sorted required asset hashes.
The presentation hash covers units and sorted hidden feature ids. Geometry cache
keys exclude visibility, so hide/show never rebuilds the model. Suppression is
part of the recipe and does invalidate downstream evaluation.

Hydrated mesh caches must match the model and tessellation ABI and pass buffer,
index, owner, and selection-metadata validation. Checkpoints must also match the
OpenRCAD/tolerance ABI, content hash, topology health, and watertightness.
Creation time is assigned once by the application, stored in document state,
and preserved by canonical load/save. Loads prune visibility entries whose
feature no longer exists and report each recovery as a diagnostic.

## Phase 3 closure of the Phase 2 deviation ledger

- **P2-D1 — Application container closed.** `Document` is now the application,
  undo, preview, persistence, document-worker, and evaluation-worker root. It
  privately owns `ParametricGraph` as its derived evaluator projection; explicit
  accessors define that boundary and existing UI field access is an ergonomic
  dereference, not competing ownership. **Phase 3 closure:** complete.
- **P2-D2 — Authoritative feature record closed.** `FeatureRecord` owns the
  payload, stable kind, payload version, selector inputs, sequence, state, and
  body membership together. `FeatureNode` is only an insertion command and no
  production edit path synchronizes a semantic sidecar. **Phase 3 closure:**
  complete.
- **P2-D3 — Relationship and body membership closed.** Semantic selectors and
  body timelines are the persisted relationship model; petgraph edges are
  derived scheduling data. `DocumentSemantics::body_outputs` records producer
  provenance directly. Phase 6 removed the last edit/load parser for the
  historical `feature::body:N` spelling; ownership now always comes from the
  persisted provenance record. **Phase 3 closure:** complete.
- **P2-D4 — Evaluator dispatch closed.** The registry maps every stable feature
  kind to one exact evaluator kind. The shared outer transaction performs
  resolve, invoke, validate, diagnostic/status recording, commit, and checkpoint
  caching. Feature payload destructuring occurs only after exact registry
  dispatch; there is no central `FeatureType` family match and every body
  mutation reaches the shared operation and commit services. Phase 3.5 added
  user-facing Intersect through that same Common adapter. The architecture gate
  now exercises record validation, semantic output ownership, evaluation, and
  canonical save/load behavior instead of searching implementation text.
  **Phase 3 closure:** complete.


## Migration rules

- v5 is an exact-version binary contract; v4 and plain JSON are rejected rather
  than guessed or partially interpreted.
- Phase 2 does not change the public `.zcad` feature meaning. It replaces the
  unstable serialization representation with versioned numeric DTOs.
- Legacy APIs remain only for frozen Phase 0 benchmarks/tests and explicit
  compatibility fixtures. New production code must use the document APIs.
- A future feature-schema migration increments its payload version and adds an
  explicit decoder; it must never reinterpret an existing numeric field.
- A future container migration increments `CURRENT_VERSION` and adds an
  intentional migration tool or reader. It must not weaken v5 size, digest,
  overlap, or decompression limits.

## Phase 0 performance comparison

The 2026-07-15 Phase 0 comparisons retained at
`target/phase0-current.json`, `target/phase0-current-full.json`, and
`target/phase0-current-full-rerun.json` preserved every feature, body, and
triangle count. Cold rebuild time improved by roughly 13–81% across the modeled
corpora and warm rebuild time improved by roughly 74–99%.

The frozen 10% gate remains intentionally unchanged and reports v5 save/open and
file-size differences against the v4 baseline. Compact files grew from
0.6–4.2 KiB to 1.0–9.5 KiB because v5 adds explicit semantic records, stable
numeric payloads, required-asset indexing, ABI manifests, section framing, and
independent digests. These files remain well below the Phase 2 compact overhead
limit of 40 KiB. The comparison also reports higher save/open time for long
histories because canonicalization and validation are now part of the measured
operation; the largest observed values were 15.35 ms to save and 7.32 ms to
open 500 features. The Phase 0 baseline was not rewritten or relaxed.

The full application rerun measured a 22,715,904-byte executable (+4.1%),
129,032,192-byte idle working set (-1.0%), and 153,489,408-byte peak working set
(-0.6%), all within the committed budget. A first launch immediately after the
release LTO link measured 1.38 seconds, while the repeat launch measured 169 ms;
the non-repeatable first-link/cache sample is retained rather than hidden, but
does not indicate new document work on the startup path.
