# External STEP fixtures

`nist-bracket1-part.stp` is an AP203 mechanical bracket authored by ST-ACIS
and obtained from the STEP Tools AP203 sample catalog:

https://steptools.com/docs/stpfiles/ap203/bracket1-part.stp

The catalog identifies its design source as the NIST DPPA Repository. NIST's
CAD/STEP test-case page states that its test cases, CAD models, and STEP files
may be used without restriction:

https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0

The fixture is committed byte-for-byte and must not be regenerated through
OpenRCAD. It exists to exercise an externally authored exchange file rather
than a round trip through ZeroCAD's own STEP writer. Phase 3 added its
`INTERSECTION_CURVE` geometry and explicit, reported legacy-pcurve recovery.
The Phase 3 operation gate now locks in a healthy, watertight import, complete
pcurves and topology history, and strict tessellation without replacing the
fixture. The Phase 0 corpus test continues to lock its provenance and the
committed baseline continues to lock the original corpus manifest.

Frozen SHA-256:
`E17A7657B0F251E93713A201BD3DC393A9F905AE477E43952BD9170D17F1A7FF`
