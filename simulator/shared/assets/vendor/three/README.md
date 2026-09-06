# Vendored three.js

Vendored locally so `sf telemetry --web` and the SILS GUI's 3D view work
with no internet access. Both are typically used while the PC's Wi-Fi is
associated 1:1 with the vehicle's own SoftAP (or an offline workshop LAN),
which has no route to any CDN — a CDN-hosted three.js would never load in
that case, not just occasionally.

- `three.module.min.js` — three.js r160 (build/three.module.min.js)
- `addons/controls/OrbitControls.js` — examples/jsm/controls/OrbitControls.js
- `addons/loaders/STLLoader.js` — examples/jsm/loaders/STLLoader.js

Unmodified upstream files (MIT license, see each file's header) from
https://cdn.jsdelivr.net/npm/three@0.160.0/ . To update the version, replace
these three files with the same paths from the new tag and re-check that
both consumers' importmaps still point at `three.module.min.js`.
