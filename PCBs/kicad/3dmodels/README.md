# Third-party 3D models

Two models referenced by `mainboard.kicad_pcb` are not redistributed here:

| File | Used by |
|---|---|
| `PESD3V3L1BA.step` | 23 TVS diode placements |
| `NCV68061SOP95P275X110-6N.step` | 1 ideal-diode controller |

Drop them into this directory to restore the complete 3D view. Everything else
resolves from the stock KiCad 3D model library via `${KICAD9_3DMODEL_DIR}`.

The original design referenced these two through absolute paths on the author's
machine; those paths were replaced with `${KIPRJMOD}/3dmodels/` so the project
contains no machine-specific locations.
