# Support Fins → BambuStudio: native integration plan

Source: https://github.com/gittrahan/support-fins (MIT). Target: this fork (AGPL-3.0, fine to vendor MIT code with attribution).

## What support-fins actually is

- **The product is the JS geometry engine** in `web/` (~11k lines of vanilla ES modules, no deps, no boolean kernel). It takes a posed triangle soup (mm, Z-up) and returns closed shells: breakaway walls, wedges, one-layer tines, sway braces, a bed pad. The slicer unions them with the part.
- **Every plugin in `plugins/` is a thin host adapter** that runs that same JS bundle unmodified in an embedded V8 (mini-racer) or, where it can't (Prusa Lua, Onshape FeatureScript), ships a dumbed-down hand-placed fin. None of them contain a second implementation of the geometry.
- `plugins/orca/` is the closest analog: injects fin cross-sections as layer polygons at `posSlice`. It exists only because Orca's plugin API can't add volumes to the plate. We are inside the C++ app, so we don't need that hack.
- Specs you port from, not from the README:
  - `docs/FIN-SPEC.md` — the geometry numbers and every deliberate deviation
  - `plugins/shared/engine/options.json` — the full user-facing settings schema (key, type, default, range, showIf, tooltip)
  - `plugins/shared/engine/fins_entry.js` — the one entry point (`computeFins`, `drawWall`), the seating/frame contract, the 1 nm snap
  - `tests/*.test.js` + `tests/golden/*.json` — the behavioural spec (counts, wall endpoints, hashes) on `prototype/stress/models/*.stl`
  - `plugins/shared/ENGINE-SENSITIVITY.md` — why you must seat in float64 and snap to a grid

## Decision 1: port the engine to C++ (don't embed a JS runtime)

| Option | Verdict |
|---|---|
| Embed QuickJS and run the esbuild bundle | Tracks upstream for free, but it is an interpreter. V8 *jitless* already takes 80 s on a 75k-triangle part (their own number). Unusable in a slicer. |
| Run it in the existing wxWebView | JIT available, but UI-thread, async, no CLI/headless slicing, WebKitGTK on Linux is flaky. No. |
| **Port to C++ in `libslic3r`** | ~10k lines of float math on triangle soups. Maps directly onto `indexed_triangle_set`, Eigen, `AABBMesh` (replaces `inside.js` ray/nearest queries with something far faster), `ClipperUtils`. The Deno test suite is the parity oracle. **Do this.** |

Cost of the port: it forks the engine at a pinned commit. Upstream moves fast (commits dated this month). Mitigation: pin the commit in the source header, keep a module-map table (JS file → C++ file), re-run the upstream Deno suite against the C++ port via the golden JSON when syncing.

## Decision 2: fins are real `ModelVolume`s on the object (not slice-time injection)

Generated fins become `MODEL_PART` volumes on the `ModelObject`, one per piece (`prop`/`wedge`/`sway`/`pad`), tagged with a new `SupportFinInfo` (generator options + hash of the source mesh/pose) persisted in the 3MF the same way `text_info` is (`bbs_3mf.cpp`, `TEXT_INFO_TAG` pattern).

Why this beats the Orca-plugin route:
- Visible in Prepare, selectable/deletable per fin in the object list, exportable as STL/3MF (the whole point of the project: fins live in the file).
- Slicing just works: same region, unioned at `slice_volumes`, elephant-foot compensation, brim, arrange all see real geometry. The Orca plugin's frame calibration, EFC emulation and "fins only visible in Preview" limitations all disappear.
- Undo/redo, project save/load, multi-plate: free.

Known constraints to design for:
- **Instances with different rotations.** Fin volumes are in object space, so all instances share them. v1: generate from instance 0 and warn if other instances have a different rotation (offer "split to objects").
- **Negative volumes / modifiers.** Analysis input is model-part volumes only (same as the Orca plugin). Optionally subtract negative volumes with `MeshBoolean` first; defer.
- **Fins vs. supports.** Mirror the Orca rule: if the object has fin volumes and `enable_support` is on, show a per-object warning (`ConfigManipulation` / `Print::validate`), don't silently do both.

## Decision 3: UI is a gizmo, modelled on Brim Ears

`GLGizmoSupportFins` next to `GLGizmoBrimEars` in the toolbar (`GLGizmosManager.cpp:117`, icons under `resources/images/toolbar_*.svg`). ImGui side panel generated from `options.json`:

- Setup: Material (PLA/PETG), Overhang threshold (30–70°, from the plate, not the slicer's support angle)
- Tines: on/off, Tine grip 0–100 %. Layer height is read from the print preset, never shown.
- Clearances: Bed pad Off/Auto/Light/Sure hold
- Sway braces: on, Grip from, Tine spacing, Depth %
- Walls: Cutouts None/Diamond/Triangle/Arch/Lattice, Wide-face coverage %, Plate only
- Mode: **Auto** (Generate / Regenerate / Clear) and **Draw** (click two points on the part → one wall via `drawWall`; click a fin → select → Delete)
- Overlay while open: overhang faces red (`overFaces`), too-small faces amber (`smallFaces`), same as the site. Reuse the facet-colouring path from the painter gizmos.
- Readout line from `engine/report.js` ("4 walls, 20 tines; 1 overhang too shallow…").

Staleness: on instance rotation of an object that has fin volumes, mark the fins stale (amber badge in object list + gizmo). v1: user clicks Regenerate. v2: auto-regenerate after the rotate gizmo commits.

## Port scope (JS → `src/libslic3r/SupportFins/`)

| JS module(s) | Lines | C++ target | Notes |
|---|---|---|---|
| `overhangs.js`, `planes.js`, `pieces.js`, `solids.js`, `materials.js`, `fins/config.js`, `prop/config.js` | ~1.3k | `Topology.cpp`, `Overhangs.cpp`, `WallPatches.cpp`, `Config.hpp` | analysis + all tunables |
| `inside.js`, `kiss.js` | ~550 | `PartQueries.cpp` | replace with `AABBMesh` (`AABBMesh.hpp`) for inside/nearest/ray; keep semantics, keep the 1 nm snap |
| `prop.js` + `prop/*` (tracks, contact, sweep, tines, clearance, raster, lattice, …) | ~4.4k | `Walls.cpp`, `Tines.cpp`, `Tracks.cpp`, … | the bulk of the work; this is the "combined fin" |
| `fins.js` + `fins/*` (pad, wedges, fill, coverage, seating, shortwalls) | ~2.2k | `Fins.cpp` (orchestrator), `BedPad.cpp`, `Wedges.cpp`, `Fill.cpp` | `buildFins()` is the public entry |
| `sway.js`, `cutout.js`, `draw.js` | ~1.5k | `Sway.cpp`, `Cutouts.cpp`, `Draw.cpp` | |
| `plugins/shared/engine/fins_entry.js`, `seat.js` | ~400 | `SupportFins.hpp` public API: `compute_fins(its, Options) -> {pieces, over_faces, small_faces, stats}` | frame contract lives here |
| `orient.js` | 330 | optional, phase 5 | fin-aware orientation scoring; BambuStudio has its own orient |
| `stl.js`, `step.js`, `threemf.js`, `zip.js`, `app.js`, `ui/*` | — | **not ported** | BambuStudio already has all of it |

Public API sketch:

```cpp
namespace Slic3r::SupportFins {
struct Options { /* 1:1 with options.json keys, plus layer_height */ };
struct Piece   { std::string kind; indexed_triangle_set mesh; };
struct Result  { std::vector<Piece> pieces; std::vector<int> over_faces, small_faces; Stats stats; };
Result compute_fins(const indexed_triangle_set& posed_part_mm, const Options&);
std::optional<Piece> draw_wall(const indexed_triangle_set&, Vec3d a, Vec3d b, const Options&, std::string& reason);
}
```

## Tests

- `tests/libslic3r/test_support_fins.cpp` (Catch2, existing harness). Copy `prototype/stress/models/*.stl` (all under 100 KB) to `tests/data/support_fins/`.
- Port the scenes in `tests/golden.test.js`. Gate on: wall count, tine count, unserved count, per-wall `from`/`to` within 0.05 mm, piece kinds. **Do not** gate on the SHA-256 — C++ float ordering will never reproduce it.
- Port the count-level tests that matter for regressions: `tines.test.js`, `pad.test.js`, `sway.test.js`, `cutout.test.js`, `draw.test.js`, `plate_only.test.js`.
- Integration: a 3MF with fin volumes round-trips `SupportFinInfo`; slicing an l-bracket at 35° produces the same per-layer wall toolpaths as slicing the site's exported finned STL (the Orca README already validated that equivalence to 0.01 % extrusion).

## Phases

1. **Scaffold (small).** `SupportFinInfo` on `ModelVolume` + 3MF read/write; `GLGizmoSupportFins` shell with the options panel; Generate inserts a placeholder prism as a tagged volume. Proves the pipeline end to end: volume → Prepare view → slice → G-code → save/load → undo.
2. **Analysis.** Port topology/overhangs/wall-patches/part-queries. Gizmo paints red/amber overhang faces and reports counts. Parity with `tests/coverage.test.js`, `floating.test.js`, `unserved.test.js`.
3. **Auto mode, PLA defaults.** Port `prop/*`, `fins.js`, pad. Golden parity on `cube-x40`, `lbracket-x35`, `staircase-x40`, `sphere`, `torus-x30`. This is the first shippable milestone.
4. **Everything else.** Wedges, short walls, fill/coverage, cutouts, sway braces, Draw mode, PETG material, Plate only.
5. **Polish.** Stale-on-rotate, support-conflict warning, i18n strings (upstream has `web/i18n/de.js`), docs, confirm Windows ARM64 build (no new deps, so this should be a non-event). Optional: fin-aware orient.

## Risks, stated once

- **Float sensitivity.** Upstream admits tine placement flips under 1e-13 mm noise. Port in double, snap to 1 nm after seating exactly as `fins_entry.js` does, and resolve ties deterministically. Expect golden *hashes* to differ; *counts and endpoints* must not.
- **Upstream drift.** The port is a fork. Pin the commit, keep the module map, treat the upstream Deno suite as the spec when syncing.
- **Effort.** ~10k lines of dense geometry code. Phase 3 is the real work; phases 1–2 are days, 3–4 are weeks.
- **Attribution.** MIT requires the copyright notice. Add `src/libslic3r/SupportFins/LICENSE-support-fins` and credit Slant3D + Matthew Trahan in the source header, as upstream does.
