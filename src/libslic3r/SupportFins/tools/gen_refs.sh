#!/usr/bin/env bash
# Regenerate the oracle dumps the parity runner consumes.
#   SUPPORT_FINS_DIR=/path/to/support-fins tools/gen_refs.sh refs/
# Scenes mirror upstream tests/golden.test.js (site pose: snapped 5-degree turns about
# world X, Y, Z) plus a few option variants. Dumps carry the posed soup, so they are
# self-contained; they are NOT committed (each is 0.5-3 MB).
set -euo pipefail
: "${SUPPORT_FINS_DIR:?set SUPPORT_FINS_DIR to the upstream checkout}"
out="${1:-refs}"; mkdir -p "$out"
here="$(cd "$(dirname "$0")" && pwd)"
models="$SUPPORT_FINS_DIR/prototype/stress/models"
run() { node "$here/ref.mjs" "$models/$1.stl" "$2" "$out/$3.json" "${4:-{\}}"; }
run cube 40,0,0 cube-x40-auto
run cube 20,0,0 cube-x20-auto
run cube 30,0,0 cube-x30-auto
run cube 40,0,0 cube-x40-petg '{"material":"petg"}'
run cube 40,0,0 cube-x40-notines '{"tines":false}'
run cube 40,0,0 cube-x40-nopad '{"padStyle":"off"}'
run cube 40,0,0 cube-x40-dense '{"tineDensity":1,"coverage":1}'
run cube 40,0,0 cube-x40-diamond '{"cutout":"diamond"}'
run lbracket 35,0,0 lbracket-x35-auto
run lbracket 0,0,0 lbracket-up
run lbracket 35,0,0 lbracket-x35-plateonly '{"plateOnly":true}'
run sphere 0,0,0 sphere-auto
run torus 30,0,0 torus-x30-auto
run staircase 40,0,0 staircase-x40-auto
run bar 0,0,0 bar-sway '{"sway":{"on":true}}'
for m in arch bar coin cone cube cylinder hexprism lbracket lowledge needle plate plus portal pyramid ramp sphere staircase torus tshape tube ushape wedge; do
  run "$m" 0,0,0 "stress-$m-up"
  run "$m" 40,0,0 "stress-$m-x40"
done
