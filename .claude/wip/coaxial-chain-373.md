# Coaxial census chain 373 (open)

`ZEROCAD_CENSUS=500 ZEROCAD_CENSUS_SET=coaxial`: 1 wrongful refusal left (chain 373, step 6, Join -> FreeEdge).
Needs steps 1,2,3,5,6; step 6 is a boss whose wall coincides with an existing bore.

Kernel-level reduction (fails with ZERO noise whenever the three seam phases differ):
block 44.2x35.7x11.28 centred on (22.1,17.8); through bore r=3.876225; `union` a wider plug
(r=5.8119, z 6.10..10.87, inside the block, splits the bore); then `union` a same-radius boss
(r=3.876225, z 5.84..10.57) whose bottom cap sits inside the lower bore's void (z 5.84 < 6.10).
Result has FreeEdge: the bore wall (z 0..6.10) is NOT split at the boss cap's z=5.84 circle, so
the coincident strip z 5.84..6.10 survives with a dangling vertical slit (the tool seam line is
imprinted, the z=5.84 crosscut is not in the partition queue for the object wall).
Traced via DBG prints: imprint_curve_on_face(wall, z=5.84 crosscut) returns new_edges=1, but the
object wall's QUEUE at partition time only holds the vertical seam-line edge. Suspect the queued
crosscut is lost between split_tracked and run_partition (edge replaced by a later split_edge
during step A / try_split_edge, or removed by planarize/dedupe) - start there.
