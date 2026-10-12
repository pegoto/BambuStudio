#!/usr/bin/env node
// Reference oracle for Engine/Prop.* and Engine/Draw.* (web/prop.js, web/draw.js and
// plugins/shared/engine/draw_entry.js). Poses each model like tools/ref.mjs, seats +
// analyzes it exactly as fins_entry.js computeFins does (seatSoup, buildTopology,
// analyze(..., IDENTITY3)), then, per config (material tunables applied with fins.js
// applyTunables, iface off/'flat'/'all', cutout, and crestPart as buildFins sets it):
//   buildProps with the opts fins.js passes in prop and auto mode (auto: tines, coverage,
//     tineDensity, layerHeight, plateOnly, raster, and rasterVeto = the real wedgeVeto,
//     whose verdicts are recorded in call order so the C++ test can replay them),
//   buildShortWalls as fins/shortwalls.js calls it (the regions left bare),
//   drawnLine / drawnWall on click pairs (under undersides, too short, over the part, on
//     an upright side, in mid-air, at the plate, across the part) with several option sets,
//   draw_entry.js drawWall on some of those pairs in the posed frame.
// Everything (triangles in emission order, interface tags, every prop field, skipped,
// served lists, reasons, tine captures) goes into one JSON per scene for
// tests/test_prop.cpp.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/prop_ref.mjs [outDir] [sceneFilter]
// outDir defaults to refs_prop/ next to this tools/ directory.
import { readFileSync, writeFileSync, mkdirSync, readdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_prop');
const filter = process.argv[3] || null;
mkdirSync(outDir, { recursive: true });
const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);

const { PROP } = await imp('web/prop/config.js');
const { CUT } = await imp('web/cutout.js');
const crest = await imp('web/prop/crest.js');
const prop = await imp('web/prop.js');
const draw = await imp('web/draw.js');
const { applyTunables } = await imp('web/fins.js');
const { FIN } = await imp('web/fins/config.js');
const { PERP, wedgeVeto, bareAfterWedges } = await imp('web/fins/wedges.js');
const { seatedPartTris } = await imp('web/fins/seating.js');
const { findWallPatches } = await imp('web/planes.js');
const { MATERIAL } = await imp('web/materials.js');
const { drawWall } = await imp('plugins/shared/engine/draw_entry.js');
const { buildTopology, analyze, IDENTITY3 } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

// ---- tools/ref.mjs: STL reader and site pose -------------------------------------
function readSTL(file) {
  const buf = readFileSync(file);
  const isAscii = buf.subarray(0, 5).toString() === 'solid' && !/\0/.test(buf.subarray(0, 300).toString('latin1'))
    && buf.toString('latin1').includes('facet');
  const out = [];
  if (isAscii) {
    const re = /vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g;
    let m; const txt = buf.toString('latin1');
    while ((m = re.exec(txt))) out.push(+m[1], +m[2], +m[3]);
  } else {
    const n = buf.readUInt32LE(80);
    for (let t = 0; t < n; t++) {
      const o = 84 + t * 50 + 12;
      for (let v = 0; v < 9; v++) out.push(buf.readFloatLE(o + v * 4));
    }
  }
  return Float64Array.from(out);
}
function sitePose(deg) {
  const snap = (d) => Math.round(d / 5) * 5 * Math.PI / 180;
  const axis = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let q = [0, 0, 0, 1];
  const mul = (a, b) => [
    a[0] * b[3] + a[3] * b[0] + a[1] * b[2] - a[2] * b[1],
    a[1] * b[3] + a[3] * b[1] + a[2] * b[0] - a[0] * b[2],
    a[2] * b[3] + a[3] * b[2] + a[0] * b[1] - a[1] * b[0],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
  deg.forEach((d, i) => {
    if (!d) return;
    const h = snap(d) / 2, s = Math.sin(h);
    const r = [axis[i][0] * s, axis[i][1] * s, axis[i][2] * s, Math.cos(h)];
    q = mul(r, q);
  });
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}

// ---- scenes: upstream's golden set (every stress model at 0,0,0 and 40,0,0) + extras --
const MODELS = readdirSync(path.join(SF, 'prototype/stress/models')).filter((f) => f.endsWith('.stl'))
  .map((f) => f.slice(0, -4)).sort();
const SCENES = [];
for (const m of MODELS) for (const r of ['0,0,0', '40,0,0']) SCENES.push([m, r]);
SCENES.push(['cube', '20,0,0'], ['cube', '30,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'],
            ['portal', '0,60,0'], ['portal', '0,35,0'], ['ushape', '0,60,0'], ['arch', '5,0,0']);
// ...and, beyond the golden set, parts that reach the paths it does not: a bore (inBore),
// fixtures and coupons, and tools/tracks_ref.mjs's synthetic nets and small tubes
// (lattice struts in the raster slot, the small-tube rivalry), pre-posed.
for (const [m, f] of [['bore-coupon', 'prototype/calibration/bore/print/bore-coupon.stl'],
                      ['bowl', 'tests/fixtures/bowl.stl'], ['dome_ceiling', 'tests/fixtures/dome_ceiling.stl'],
                      ['torus_flat', 'tests/fixtures/torus_flat.stl'], ['angle-coupon', 'web/calibration/angle-coupon.stl'],
                      ['short-coupon', 'prototype/calibration/short/print/short-coupon.stl'],
                      ['lip-coupon', 'prototype/calibration/lip/print/lip-coupon.stl'],
                      ['span-coupon', 'prototype/calibration/span/print/span-coupon.stl']]) {
  SCENES.push([m, '0,0,0', f]);
}
SCENES.push(['bore-coupon', '40,0,0', 'prototype/calibration/bore/print/bore-coupon.stl']);

// ---- synthetic parts (tools/tracks_ref.mjs: tests/lattice.test.js, small_tubes.test.js) --
function voxelPos(filled, nx, ny, nz) {
  const V = 2;
  const at = (i, j, k) => i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz && filled(i, j, k);
  const P = (i, j, k) => [i * V, j * V, k * V], t = [];
  const q = (a, b, c, d) => t.push(a, b, c, a, c, d);
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) for (let k = 0; k < nz; k++) {
    if (!at(i, j, k)) continue;
    if (!at(i, j, k - 1)) q(P(i, j, k), P(i, j + 1, k), P(i + 1, j + 1, k), P(i + 1, j, k));
    if (!at(i, j, k + 1)) q(P(i, j, k + 1), P(i + 1, j, k + 1), P(i + 1, j + 1, k + 1), P(i, j + 1, k + 1));
    if (!at(i - 1, j, k)) q(P(i, j, k), P(i, j, k + 1), P(i, j + 1, k + 1), P(i, j + 1, k));
    if (!at(i + 1, j, k)) q(P(i + 1, j, k), P(i + 1, j + 1, k), P(i + 1, j + 1, k + 1), P(i + 1, j, k + 1));
    if (!at(i, j - 1, k)) q(P(i, j, k), P(i + 1, j, k), P(i + 1, j, k + 1), P(i, j, k + 1));
    if (!at(i, j + 1, k)) q(P(i, j + 1, k), P(i, j + 1, k + 1), P(i + 1, j + 1, k + 1), P(i + 1, j + 1, k));
  }
  const pos = new Float32Array(t.length * 3);
  t.forEach((p, n) => pos.set(p, n * 3));
  return Float64Array.from(pos);
}
const netPos = () => {
  const bar = (u) => u % 10 < 2;
  return voxelPos((i, j, k) =>
    (k >= 12 && k < 14 && (bar(i) || bar(j))) ||
    (k < 12 && bar(i) && bar(j) && (i < 2 || i >= 30) && (j < 2 || j >= 30)), 32, 32, 14);
};
const quad = (a, b, c, d) => [a, b, c, a, c, d];
function block(x0, x1, y0, y1, z0, z1) {
  const v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
             [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
  return [...quad(v[0], v[3], v[2], v[1]), ...quad(v[4], v[5], v[6], v[7]), ...quad(v[0], v[1], v[5], v[4]),
          ...quad(v[2], v[3], v[7], v[6]), ...quad(v[1], v[2], v[6], v[5]), ...quad(v[0], v[4], v[7], v[3])];
}
function cylinderY(cx, cz, r, y0, y1, n = 48) {
  const ring = (y) => Array.from({ length: n }, (_, i) => {
    const a = (2 * Math.PI * i) / n;
    return [cx + r * Math.cos(a), y, cz + r * Math.sin(a)];
  });
  const A = ring(y0), B = ring(y1), t = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    t.push(A[i], B[j], A[j], A[i], B[i], B[j]);
    t.push([cx, y0, cz], A[i], A[j]);
    t.push([cx, y1, cz], B[j], B[i]);
  }
  return t;
}
const f32 = (tris) => Float64Array.from(Float32Array.from(tris.flat()));
const SYNTH = {
  net: netPos,
  peg: () => f32([...block(-15, 15, -10, 10, 0, 24), ...cylinderY(0, 14, 2.5, 9, 18)]),
  fatpeg: () => f32([...block(-25, 25, -10, 10, 0, 60), ...cylinderY(0, 30, 8, 9, 23)]),
  pegs: () => f32([...block(-25, 25, -10, 10, 0, 60), ...block(-25, -5, -30, -10, 40, 44),
                   ...cylinderY(0, 30, 8, 9, 23), ...cylinderY(15, 12, 2.5, 9, 18),
                   ...block(5, 25, -10, 2, 50, 53).map(([x, y, z]) => [x, y - 22, z])]),
};
SCENES.push(['net', '20,0,0', SYNTH.net], ['net', '0,0,0', SYNTH.net], ['peg', '0,0,0', SYNTH.peg],
            ['fatpeg', '0,0,0', SYNTH.fatpeg], ['pegs', '0,0,0', SYNTH.pegs], ['pegs', '10,0,0', SYNTH.pegs]);

// ---- configs -----------------------------------------------------------------------
// mode 'prop': buildFinsCore's prop branch hands buildProps the whole opts.
// mode 'auto': the prop branch is entered with { ...opts, mode: 'prop', tines: withTines,
//   rasterVeto: wedgeVeto(...) }. draw: run the draw cases; entry: run drawWall too (only
//   with iface off: the C++ applyTunables always writes PROP.iface, the JS one leaves it).
const CFGS = [
  { material: 'pla', iface: false, cutout: 'none', mode: 'prop', opts: { layerHeight: 0.2 }, draw: true, entry: true },
  { material: 'pla', iface: false, cutout: 'none', mode: 'auto', opts: { tines: true, coverage: 0.5, tineDensity: 0, layerHeight: 0.2 }, capture: true, short: true },
  { material: 'petg', iface: 'flat', cutout: 'none', mode: 'auto', opts: { tines: true, coverage: 0, tineDensity: 1, layerHeight: 0.12 }, short: true, draw: true },
  { material: 'pla', iface: 'all', cutout: 'diamond', mode: 'auto', opts: { tines: false, coverage: 1, plateOnly: true, layerHeight: 0.2 }, draw: true, short: true },
  { material: 'pla', iface: false, cutout: 'none', mode: 'prop', opts: { tines: true, coverage: 0, raster: false, layerHeight: 0.12 }, short: true },
  { material: 'petg', iface: 'all', cutout: 'none', mode: 'prop', opts: { tines: true, plateOnly: true, coverage: 1, tineDensity: 1, layerHeight: 0.12 }, capture: true },
  { material: 'pla', iface: 'flat', cutout: 'diamond', mode: 'auto', opts: { tines: true, coverage: 0.5, raster: false, layerHeight: 0.2 } },
  // a stand-in veto (refuses every raster wall whose line has an odd station count), so the
  // replayed veto also says yes; capture on, so the kept slices are re-gathered around it
  { material: 'pla', iface: false, cutout: 'none', mode: 'auto', stubVeto: true, opts: { tines: true, coverage: 1, layerHeight: 0.2 }, capture: true },
];
const DRAW_OPTS = [
  {},
  { tines: true, topo: true, tineDensity: 0, layerHeight: 0.2 },
  { under: true, tines: true, topo: true, brace: true, layerHeight: 0.12, tineDensity: 1 },
  { plateOnly: true, tines: true, topo: true },
  { brace: true, plateOnly: false },
];
const ENTRY_OPTS = [
  {},
  { material: 'petg', tines: false, layerHeight: 0.12, plateOnly: true, cutout: 'diamond' },
  { tineDensity: 1, threshold: 40 },
];

const PROP0 = { ...PROP }, PERP0 = { ...PERP }, CUT0 = { ...CUT };
const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);
const flat = (tris) => { const a = []; for (const v of tris) a.push(v[0], v[1], v[2]); return a; };
const tagsOf = (tris) => tris.map((v) => (v[3] === 1 ? 1 : 0));
const line3 = (l) => (l ? l.map((p) => [p[0], p[1], p[2]]) : null);
function coverPitch(coverage) {   // fins.js (not exported)
  const c = Math.max(0, Math.min(1, coverage));
  return c <= 0.5
    ? FIN.coverSparse + ((0.5 - c) / 0.5) * (FIN.coverExtraSparse - FIN.coverSparse)
    : FIN.coverSparse - ((c - 0.5) / 0.5) * (FIN.coverSparse - FIN.coverDense);
}
const propOf = (q) => ({
  region: q.region ?? null, tines: q.tines ?? null, caps: q.caps ?? null, grip: q.grip ?? null,
  span: q.span ?? null, height: q.height ?? null, area: q.area ?? null, stations: q.stations ?? null,
  trimmed: q.trimmed ?? null, volume: q.volume ?? null, line: line3(q.line), id: q.id ?? null, kind: q.kind ?? null,
  triRanges: q.triRanges ?? null, partAttached: q.partAttached ?? null, inBore: q.inBore ?? null,
  squat: q.squat ?? null, triRange: q.triRange ?? null, raster: q.raster ?? null,
  keys: Object.keys(q).sort(),
});
const resultOf = (r) => ({
  triangles: flat(r.triangles), tags: tagsOf(r.triangles), props: r.props.map(propOf), skipped: r.skipped,
  served: r.served, servedRegions: r.servedRegions ?? null, tines: r.tines ?? null, sagRisk: r.sagRisk ?? null,
  sagRegions: r.sagRegions ? [...r.sagRegions] : null, volume: r.volume, rasterRegions: r.rasterRegions ?? null,
  keys: Object.keys(r).sort(),
});

let total = 0;
for (const [model, rotStr, src] of SCENES) {
  const name = `${typeof src === 'function' ? 'syn_' : ''}${model}-${rotStr.replace(/,/g, '_')}`;
  if (filter && !name.includes(filter)) continue;
  const t0 = performance.now();
  const raw = typeof src === 'function' ? src()
    : readSTL(path.join(SF, src ?? path.join('prototype/stress/models', `${model}.stl`)));
  const pose = sitePose(rotStr.split(',').map(Number));
  const posed = new Float64Array(raw.length);
  for (let i = 0; i < raw.length; i += 3) {
    const x = raw[i], y = raw[i + 1], z = raw[i + 2];
    posed[i] = pose[0] * x + pose[3] * y + pose[6] * z;
    posed[i + 1] = pose[1] * x + pose[4] * y + pose[7] * z;
    posed[i + 2] = pose[2] * x + pose[5] * y + pose[8] * z;
  }
  const { pos, shift } = seatSoup(posed);
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, 45, IDENTITY3);
  const rot = IDENTITY3, off = result.offset;
  const partTris = seatedPartTris(topo, rot, off);

  // ---- click pairs, PRINT frame ----
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < partTris.length; i += 3) {
    minX = Math.min(minX, partTris[i]); maxX = Math.max(maxX, partTris[i]);
    minY = Math.min(minY, partTris[i + 1]); maxY = Math.max(maxY, partTris[i + 1]);
    maxZ = Math.max(maxZ, partTris[i + 2]);
  }
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const pairs = [];
  result.regions.slice(0, 3).forEach((g) => {
    // the region's mean face centroid, and its lowest face centroid
    let sx = 0, sy = 0, sz = 0, low = null;
    for (const f of g.faces) {
      const c = [0, 1, 2].map((k) => (partTris[f * 9 + k] + partTris[f * 9 + 3 + k] + partTris[f * 9 + 6 + k]) / 3);
      sx += c[0]; sy += c[1]; sz += c[2];
      if (!low || c[2] < low[2]) low = c;
    }
    const c = [sx / g.faces.length, sy / g.faces.length, sz / g.faces.length];
    pairs.push(['under-x', [c[0] - 4, c[1], c[2]], [c[0] + 4, c[1], c[2]]]);
    pairs.push(['under-y', [c[0], c[1] - 6, c[2]], [c[0], c[1] + 6, c[2]]]);
    pairs.push(['under-diag', [low[0] - 1.6, low[1] - 1.6, low[2]], [low[0] + 1.6, low[1] + 1.6, low[2]]]);
    pairs.push(['short', [c[0] - 0.5, c[1], c[2]], [c[0] + 0.5, c[1], c[2]]]);
  });
  pairs.push(['top', [cx - 5, cy, maxZ], [cx + 5, cy, maxZ]]);
  pairs.push(['side', [maxX, cy - 5, maxZ / 2], [maxX, cy + 5, maxZ / 2]]);
  pairs.push(['air', [maxX + 20, cy - 5, 10], [maxX + 20, cy + 5, 10]]);
  pairs.push(['plate', [minX - 1, cy - 5, 0], [minX - 1, cy + 5, 0]]);
  pairs.push(['across', [minX, minY, maxZ * 0.6], [maxX, maxY, maxZ * 0.6]]);
  pairs.push(['same', [cx, cy, 3], [cx, cy, 5]]);

  const calls = [];
  for (let ci = 0; ci < CFGS.length; ci++) {
    const cfg = CFGS[ci];
    // ---- the config: defaults, then fins.js applyTunables, then crestPart as buildFins ----
    Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0);
    const mat = MATERIAL[cfg.material];
    applyTunables({ padH: mat.padH, padGrab: mat.padGrab, propGap: mat.propGap, padStyle: 'auto', cutout: cfg.cutout, iface: cfg.iface });
    crest.crestPart(crest.crestOn() ? partTris : null, cfg.opts.layerHeight);
    calls.push({ fn: 'cfg', cfg: ci, material: cfg.material, iface: cfg.iface === false ? 'off' : cfg.iface,
                 cutout: cfg.cutout, layerHeight: cfg.opts.layerHeight ?? null, gap: PROP.gap });

    // ---- buildProps, as fins.js calls it ----
    const opts = { mode: 'prop', ...cfg.opts };
    let veto = null;
    if (cfg.mode === 'auto') {
      const withTines = cfg.opts.tines ?? true;
      opts.tines = withTines;
      const coverage = Math.max(0, Math.min(1, cfg.opts.coverage ?? FIN.coverDefault));
      const wedgeable = findWallPatches(topo, rot, off).filter((p) =>
        p.n.z < -0.05 && p.area >= PERP.minArea && (p.u1 - p.u0) >= PERP.minWidth);
      const wedgeOpts = { tines: withTines, pitch: coverPitch(coverage), tineDensity: cfg.opts.tineDensity };
      const real = cfg.stubVeto ? (q) => (q.line?.length ?? 0) % 2 === 1 : wedgeVeto(wedgeable, topo, rot, off, wedgeOpts);
      veto = [];
      opts.rasterVeto = (q, normalProps) => {
        const r = real(q, normalProps);
        veto.push({ r, n: q.line?.length ?? 0, p0: q.line?.length ? q.line[0] : null });
        return r;
      };
    }
    if (cfg.capture) globalThis.__TINECAP = [];
    const built = prop.buildProps(topo, result, rot, opts);
    const cap = globalThis.__TINECAP;
    delete globalThis.__TINECAP;
    const { rasterVeto, ...optsDump } = opts;
    calls.push({ fn: 'buildProps', opts: optsDump, veto, capture: !!cfg.capture,
                 tinecap: cap ? cap.map((t) => [t.x, t.y, t.z, t.biteX, t.biteY]) : null, out: resultOf(built) });

    // ---- buildShortWalls, as fins/shortwalls.js calls it ----
    if (cfg.short) {
      let bare = bareAfterWedges(topo, rot, result, built.servedRegions ?? [], []);
      if (!bare.length) bare = result.regions.map((_, i) => i);
      const coverage = Math.max(0, Math.min(1, cfg.opts.coverage ?? FIN.coverDefault));
      const sopts = { ...opts, tines: cfg.opts.tines ?? true, coverage };
      delete sopts.rasterVeto;
      const sw = prop.buildShortWalls(topo, result, rot, sopts, new Set(bare));
      calls.push({ fn: 'buildShortWalls', opts: sopts, regions: bare, out: resultOf(sw) });
    }

    // ---- drawnLine / drawnWall ----
    if (cfg.draw) {
      for (const [kind, a, b] of pairs) {
        for (const [step, band, under] of [[null, null, null], [PROP.stationStep, 3, true], [0.5, Infinity, false]]) {
          const l = step === null ? draw.drawnLine(a, b, partTris) : draw.drawnLine(a, b, partTris, step, band, under);
          calls.push({ fn: 'drawnLine', kind, a, b, step, band, under, out: line3(l) });
        }
        for (const dopt of DRAW_OPTS) {
          for (const zBed of kind === 'plate' ? [0, 0.3] : [0]) {
            const o = { ...dopt };
            if (o.topo) { o.topo = topo; o.rot = rot; o.offset = off; }
            const r = draw.drawnWall(a, b, partTris, zBed, o);
            calls.push({ fn: 'drawnWall', kind, a, b, zBed, opts: dopt,
                         out: { ok: r.ok, reason: r.reason ?? null, tris: r.tris ? flat(r.tris) : null,
                                tags: r.tris ? tagsOf(r.tris) : null, length: r.length ?? null, height: r.height ?? null,
                                partAttached: r.partAttached ?? null, squat: r.squat ?? null, tines: r.tines ?? null,
                                braces: r.braces ?? null, top: line3(r.top), floors: r.floors ?? null,
                                keys: Object.keys(r).sort() } });
          }
        }
      }
    }
    // ---- draw_entry.js drawWall (posed frame) ----
    if (cfg.entry) {
      const toPosed = (p) => [p[0] - off.x + shift.x, p[1] - off.y + shift.y, p[2] - off.z + shift.z];
      for (const [kind, a, b] of pairs) {
        if (kind === 'short' || kind === 'same' || kind === 'plate') continue;
        for (const eo of ENTRY_OPTS) {
          const pa = toPosed(a), pb = toPosed(b);
          const r = drawWall(posed, pa, pb, eo);
          calls.push({ fn: 'drawWall', kind, a: pa, b: pb, options: eo,
                       out: r.ok ? { ok: true, triangles: Array.from(r.triangles), offset: r.offset, stats: r.stats }
                                 : { ok: false, reason: r.reason } });
        }
      }
    }
  }
  Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0);
  crest.crestPart(null);

  const json = JSON.stringify({ scene: name, model, rot: rotStr, soup: Array.from(posed), regions: result.regions.length,
                                PROP: PROP0, cfgs: CFGS.map(({ opts, ...c }) => ({ ...c, opts })), calls }, replacer);
  writeFileSync(path.join(outDir, `${name}.json`), json);
  total += json.length;
  const bp = calls.filter((c) => c.fn === 'buildProps');
  const dw = calls.filter((c) => c.fn === 'drawnWall');
  console.log(name, 'regions', result.regions.length,
    'props', bp.map((c) => c.out.props.length).join('/'),
    'raster', bp.map((c) => c.out.rasterRegions ?? '-').join('/'),
    'drawn ok', dw.filter((c) => c.out.ok).length, '/', dw.length,
    `${(json.length / 1e6).toFixed(1)}MB ${Math.round(performance.now() - t0)}ms`);
}
console.log(`total ${(total / 1e6).toFixed(1)}MB -> ${outDir}`);
