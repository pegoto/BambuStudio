# Support Fins engine port -- conventions

The C++ under `Engine/` is a line-for-line port of the printfins.com engine
(`$SUPPORT_FINS_DIR/web/*.js`, pinned at the commit in `UPSTREAM_COMMIT`). The goal
is **bit-for-bit parity of behaviour** with the JS: same walls, same tines, same
triangles in the same order. Readability against upstream matters more than C++ taste.

## Layout: one JS module = one C++ unit

| upstream | here |
|---|---|
| `web/overhangs.js` | `Engine/Overhangs.{hpp,cpp}` (done) |
| `web/pieces.js` | `Engine/Pieces.{hpp,cpp}` (done) |
| `web/solids.js` | `Engine/Solids.hpp` (done) |
| `web/materials.js` | `Engine/Materials.hpp` (done) |
| `web/fins/config.js` (FIN) | `Engine/FinConfig.hpp` (done) |
| `web/prop/config.js` (PROP) | `Engine/PropConfig.hpp` (done) |
| `web/planes.js` | `Engine/Planes.{hpp,cpp}` |
| `web/inside.js`, `web/kiss.js` | `Engine/Inside.{hpp,cpp}`, `Engine/Kiss.{hpp,cpp}` |
| `web/cutout.js` | `Engine/Cutout.{hpp,cpp}` |
| `web/prop/<x>.js` | `Engine/Prop/<X>.{hpp,cpp}` |
| `web/prop.js` | `Engine/Prop.{hpp,cpp}` |
| `web/draw.js` | `Engine/Draw.{hpp,cpp}` |
| `web/fins/<x>.js` | `Engine/Fins/<X>.{hpp,cpp}` |
| `web/fins.js` | `Engine/Fins.{hpp,cpp}` (header done, .cpp is a stub) |
| `web/sway.js` | `Engine/Sway.{hpp,cpp}` |
| `plugins/shared/engine/fins_entry.js` + `seat.js` | `Engine/Entry.{hpp,cpp}` (done) |

Keep upstream's function names (camelCase) and argument order. Keep upstream's comments
where they explain a number or a decision; drop the ones that are history. Put the JS
file name in the header comment of each unit.

## Types (`Engine/Math.hpp`)

- `Vec3d` = Eigen double vector (DontAlign). `Tris` = `std::vector<Vec3d>`, three per
  triangle: the JS "array of [x,y,z] triples". `Soup` = flat `std::vector<double>`, 9 per
  face: the JS Float64Array `pos`.
- `Rot` = 9 doubles, column-major (THREE.Matrix3.elements). `Offset {x,y,z}`.
- JS objects with a fixed shape become structs; JS `Map` keyed by number becomes
  `std::unordered_map` **plus an insertion-order vector when iteration order matters**
  (JS Maps iterate in insertion order -- it almost always matters).
- Optional / nullable JS fields: `std::optional`. `null` returns: `std::optional` or a
  bool + out-param. Never a sentinel number.

## JS semantics you must reproduce

- `Math.round` -> `js_round` (half toward +inf). `std::round` is WRONG for negatives.
- `Array.prototype.sort(cmp)` is **stable** -> `std::stable_sort`. Never `std::sort`.
- `arr.sort()` with NO comparator sorts lexicographically as strings. Check every bare
  `.sort()` and reproduce (usually it is on numbers, and it is a bug upstream; match it).
- `Float32Array` storage rounds to float32 at that point -> store as `float` or apply
  `fround`. Output triangles are float32 at the very end (Entry.cpp does it). Look for
  `new Float32Array` and `Math.fround` in the module you port.
- `Math.hypot` -> `hypot2`/`hypot3`. `Math.sign` -> `js_sign`. `x | 0` -> `int(x)`
  (trunc). `Math.max()` of an empty spread is `-Infinity`.
- Integer division: JS has none; `Math.floor(a / b)` -> `std::floor(a / b)` on doubles.
- `for ... of` over a Map/array is insertion order. `Object.keys` order: insertion order
  for string keys, ascending for integer-like keys.
- NaN comparisons are false in both languages; `Number.isFinite` -> `std::isfinite`.
- `??` and `||` defaults: `||` treats 0 and '' as missing. Reproduce exactly.
- String keys built from numbers (`${Math.round(x*1000)},...`) -> integer tuple keys.
  The hash/ordering of the map must not affect results; if it does (iteration), keep
  insertion order explicitly.
- `Math.hypot` -> `hypot2`/`hypot3` (V8's scaled Kahan algorithm, NOT `std::hypot`,
  which differs in the last bit on ~40% of inputs). `x.toFixed(n)` rounds ties away
  from zero on the exact decimal value; printf rounds to even (see Cutout.cpp toFixed9).
  V8's `Math.asin/acos/atan2/sin/cos/exp/log` are fdlibm; glibc can differ by 1-2 ulp.
  So far only a debug field (`lean`) is affected; if a decision flips on one of these,
  port that function from V8's src/base/ieee754.cc instead of tolerating it.
- No fused multiply-add: the standalone build passes `-ffp-contract=off` and the
  libslic3r build must too for these sources (aarch64 fuses by default).
- Floating point: keep the same operations in the same order. Do not "simplify" an
  expression (a*b/c vs a*(b/c) differ in the last bit and tines flip on it, see
  upstream `plugins/shared/ENGINE-SENSITIVITY.md`). No `-ffast-math`.
- Module-level mutable state (`PROP.gap = ...`, `PAD.style`, `CUT.pattern`, `PERP.gap`)
  is `thread_local` globals here (`FIN`, `PROP`; add yours the same way, declared in the
  module header and defined in its .cpp). `applyTunables` in `Fins.cpp` writes them.

## Building and testing

```
cmake -S standalone -B build-sf -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-sf
SUPPORT_FINS_DIR=/home/user/gittrahan/support-fins tools/gen_refs.sh refs   # once
./build-sf/sf_parity refs/*.json            # end-to-end parity (fails until buildFins is real)
ctest --test-dir build-sf                   # per-module unit tests
```

Every module you port gets a `tests/test_<module>.cpp` (plain `main()`, exit non-zero on
failure, no framework; it is auto-discovered by the standalone CMake) that pins the
module's output against the JS. To get the JS side's answer, write a small Node script
in `tools/` that imports the upstream module directly (`node tools/<x>_ref.mjs ...`
with `SUPPORT_FINS_DIR` set; see `tools/ref.mjs` for the STL reader and the site pose)
and dumps JSON; load it in the test with `nlohmann/json.hpp` (include path `src/`) or
embed small expected values directly. Node 22 runs the ES modules as-is. A module whose
only consumer is another module you did not port can be tested through a thin harness
that calls it the way its consumer does.

Compile with `-Wall -Wextra` clean. No new dependencies: Eigen and the standard library
only (nlohmann/json in tests/tools only).

## What NOT to do

- Do not re-architect, optimise, or "fix" upstream behaviour. If something looks like a
  bug, port it as is and leave a `// UPSTREAM: ...` comment. Parity first; improvements
  come after the port is pinned.
- Do not change `Engine/Math.hpp`, `Overhangs.hpp`, `Fins.hpp` public structs, or
  `Entry.*` without saying so in your report: other modules build on them. Adding
  fields is fine.
- Do not touch anything outside `src/libslic3r/SupportFins/`.
