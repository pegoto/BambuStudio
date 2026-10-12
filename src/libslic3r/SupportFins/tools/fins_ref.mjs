#!/usr/bin/env node
// Reference oracle for Engine/Fins.* (web/fins.js) and its two helpers
// Engine/Fins/Shortwalls.* (web/fins/shortwalls.js) and Engine/Fins/Fill.* (web/fins/fill.js):
// what tools/ref.mjs (the end-to-end parity dumps) does not reach.
//
// Per scene (a model posed and seated as fins_entry.js computeFins does, OR -- like the
// site and tests/fill.test.js -- the raw model analyzed under a non-identity `rot`):
//   buildFins under many option sets: mode auto / full (the fill pass: coverage 0, 0.5,
//     1, tines off, Plate only, capped by FILL.maxWalls) / stabilize / prop / draw / an
//     unknown mode, lastResort false, raster false, bedPad false, tunables with padCustom,
//     iface flat / all, tinesPerWall, every pad style and a cutout, sway with every option.
//     The module state (PROP, PERP, PAD, CUT, FIN, FILL) is reset to its defaults before
//     each build and dumped after it (applyTunables' effect).
//   the internals, on scenes where Auto leaves red bare: fill.js leftoverPoints, groupsOf,
//     candidateLines (both bridges, narrow), localPatch, fillCoverage on Auto's result,
//     and shortwalls.js lastResortWalls (every region bare; against Auto's triangles).
// fill.js's internals are not exported upstream: a module loader hook appends an
// `export { ... }` line to it and to fins.js (coverPitch); nothing else changes.
// Triangles are dumped as FNV-1a-style hashes over their float64 bit patterns (whole
// list, and per fin record), with counts; lines, records and stats in full.
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/fins_ref.mjs [outDir] [sceneFilter]
// outDir defaults to refs_fins/ next to this tools/ directory.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { register } from 'node:module';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_fins');
const filter = process.argv[3] || null;
mkdirSync(outDir, { recursive: true });

const HOOK = `
const ADD = {
  '/web/fins/fill.js': 'export { leftoverPoints as __leftoverPoints, groupsOf as __groupsOf, candidateLines as __candidateLines, localPatch as __localPatch };',
  '/web/fins.js': 'export { coverPitch as __coverPitch };',
};
export async function load(url, context, next) {
  const r = await next(url, context);
  for (const [suffix, add] of Object.entries(ADD)) {
    if (url.startsWith(${JSON.stringify(pathToFileURL(SF).href)}) && url.endsWith(suffix)) {
      return { ...r, source: String(r.source) + '\\n' + add + '\\n', shortCircuit: true };
    }
  }
  return r;
}`;
register('data:text/javascript,' + encodeURIComponent(HOOK));

const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);
const { PROP } = await imp('web/prop/config.js');
const { CUT } = await imp('web/cutout.js');
const fins = await imp('web/fins.js');
const { FIN } = await imp('web/fins/config.js');
const { PAD } = await imp('web/fins/pad.js');
const { PERP } = await imp('web/fins/wedges.js');
const fill = await imp('web/fins/fill.js');
const { lastResortWalls } = await imp('web/fins/shortwalls.js');
const { reachOf, classify } = await imp('web/fins/coverage.js');
const { MATERIAL } = await imp('web/materials.js');
const { buildTopology, analyze, IDENTITY3 } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');
const { FILL } = fill;
if (!fill.__leftoverPoints || !fins.__coverPitch) throw new Error('loader hook did not expose the internals');

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
// tests/_util.js rotX (the site's rotation about X, as fill.test.js builds with)
const rotX = (d) => { const r = (d * Math.PI) / 180, c = Math.cos(r), s = Math.sin(r); return [1, 0, 0, 0, c, s, 0, -s, c]; };

// ---- synthetic parts --------------------------------------------------------------
// tests/_util.js block (Float32 soup), as fill.test.js's low-ledge tab uses it
function block(x0, x1, y0, y1, z0, z1) {
  const v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
             [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
  const q = (a, b, c, d) => [v[a], v[b], v[c], v[a], v[c], v[d]];
  const t = [...q(0, 3, 2, 1), ...q(4, 5, 6, 7), ...q(0, 1, 5, 4),
             ...q(2, 3, 7, 6), ...q(1, 2, 6, 5), ...q(0, 4, 7, 3)];
  return t.flat();
}
const tabPos = () => Float64Array.from(Float32Array.from([...block(-10, 10, -10, 10, 0, 20), ...block(-2, 2, 9.99, 13, 1.5, 6)]));
// tools/tracks_ref.mjs's lattice net (tests/lattice.test.js): struts every 10 mm on four legs
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

// ---- option sets --------------------------------------------------------------------
// Every BuildOpts field is passed explicitly (so no JS `undefined` default is in play).
const BASE = { mode: 'auto', bedPad: true, tines: true, tineDensity: 0, layerHeight: 0.2, coverage: 0.5,
               plateOnly: false, lastResort: true, raster: true };
const C = {
  auto: {},
  full: { mode: 'full' },
  full0: { mode: 'full', coverage: 0 },
  full1: { mode: 'full', coverage: 1, tineDensity: 1 },
  fullNoTines: { mode: 'full', tines: false },
  fullPlate: { mode: 'full', plateOnly: true },
  fullCap: { mode: 'full', FILL: { maxWalls: 1 } },
  fullTries: { mode: 'full', coverage: 0.8, FILL: { maxTries: 3, groupTries: 1 } },
  fullPetgAll: { mode: 'full', material: 'petg', iface: 'all', layerHeight: 0.12 },
  fullNoPad: { mode: 'full', bedPad: false },
  stabilize: { mode: 'stabilize' },
  prop: { mode: 'prop' },
  propNoTines: { mode: 'prop', tines: false, raster: false },
  propNoPad: { mode: 'prop', bedPad: false },
  draw: { mode: 'draw' },
  bogus: { mode: 'bogus', tines: false },
  autoNoLast: { lastResort: false },
  autoNoRaster: { raster: false, coverage: 0.9 },
  autoNoPad: { bedPad: false },
  autoNoTines: { tines: false, coverage: 0.2 },
  autoPlate: { plateOnly: true },
  noTunables: { tunables: null },
  tunCustom: { padStyle: 'custom', padCustom: { h: 0.8, gap: 0.1, grip: 0.1, margin: 5 } },
  tunCustomBad: { padStyle: 'custom', padCustom: { h: NaN, gap: 0.15, grip: Infinity, margin: 3 } },
  tunSure: { padStyle: 'sure', material: 'petg' },
  tunLight: { padStyle: 'light', layerHeight: 0.12 },
  tunFlat: { iface: 'flat', cutout: 'lattice' },
  tunAllProp: { mode: 'prop', iface: 'all', cutout: 'diamond' },
  tunOff: { iface: false, cutout: 'arch' },
  tunTpw: { tinesPerWall: 2 },
  tunTpw0: { mode: 'full', tinesPerWall: 0, cutout: 'triangle' },
  sway: { sway: { on: true } },
  swayAll: { sway: { on: true, gripFrom: 5, tineSpacing: 4, reach: 0.3, gap: 0.3 } },
  swayOff: { sway: { on: false, reach: 0.4 } },
  swayDraw: { mode: 'draw', sway: { on: true } },
  swayProp: { mode: 'prop', tines: false, iface: 'all', sway: { on: true, gripFrom: 0, tineSpacing: 10, reach: 0.05 } },
  swayPetg: { material: 'petg', layerHeight: 0.12, sway: { on: true, tineSpacing: 3, reach: 0.5 } },
  swayFull: { mode: 'full', sway: { on: true } },
  swayIface: { iface: 'all', sway: { on: true, gripFrom: 12 } },
};
const STD = ['auto', 'full'];
// [name, source, pose, configs, internals]
//   source: a stress model name, a path under SUPPORT_FINS_DIR, or a synthetic function
//   pose:   'seat:rx,ry,rz' (sitePose + seatSoup, as computeFins) or 'rotX:d' (raw, rot)
const SCENES = [
  ['cube-rx40', 'cube', 'rotX:40', ['auto', 'full', 'full0', 'full1', 'fullNoTines', 'fullPlate', 'fullCap', 'fullTries',
    'fullPetgAll', 'stabilize', 'prop', 'propNoTines', 'draw', 'bogus', 'autoNoLast', 'autoNoRaster', 'autoNoTines',
    'noTunables', 'tunCustom', 'tunCustomBad', 'tunFlat', 'tunAllProp', 'tunOff', 'tunTpw', 'tunTpw0'], true],
  ['hexprism-rx30', 'hexprism', 'rotX:30', ['auto', 'full', 'fullNoTines'], true],
  ['tab', tabPos, 'raw', ['auto', 'full', 'full1', 'autoNoLast'], true],
  ['net-20', netPos, 'seat:20,0,0', ['auto', 'autoNoLast', 'full', 'prop', 'autoNoTines'], true],
  ['net-0', netPos, 'seat:0,0,0', ['auto', 'autoNoLast', 'full'], true],
  ['lowledge-0', 'lowledge', 'seat:0,0,0', [...STD, 'autoNoLast', 'full0'], true],
  ['lowledge-40', 'lowledge', 'seat:40,0,0', [...STD, 'autoNoLast'], true],
  ['bowl', 'tests/fixtures/bowl.stl', 'seat:0,0,0', [...STD, 'autoNoLast'], true],
  ['dome_ceiling', 'tests/fixtures/dome_ceiling.stl', 'seat:0,0,0', [...STD, 'fullPlate'], true],
  ['torus_flat', 'tests/fixtures/torus_flat.stl', 'seat:0,0,0', STD, false],
  ['short-coupon', 'prototype/calibration/short/print/short-coupon.stl', 'seat:0,0,0', [...STD, 'autoNoLast'], true],
  ['lip-coupon', 'prototype/calibration/lip/print/lip-coupon.stl', 'seat:0,0,0', [...STD, 'autoNoLast'], true],
  ['span-coupon', 'prototype/calibration/span/print/span-coupon.stl', 'seat:0,0,0', [...STD, 'autoNoLast'], true],
  ['bore-coupon', 'prototype/calibration/bore/print/bore-coupon.stl', 'seat:40,0,0', STD, true],
  ['tshape-40', 'tshape', 'seat:40,0,0', ['auto', 'full', 'sway', 'swayAll', 'swayFull', 'swayIface', 'autoPlate'], true],
  ['bar-0', 'bar', 'seat:0,0,0', ['sway', 'swayAll', 'swayOff', 'swayDraw', 'swayProp', 'swayPetg', 'swayIface'], false],
  ['portal-0', 'portal', 'seat:0,0,0', ['auto', 'sway', 'swayAll', 'full'], true],
  ['sphere-0', 'sphere', 'seat:0,0,0', ['auto', 'autoNoPad', 'fullNoPad', 'prop', 'propNoPad', 'draw', 'full', 'tunSure', 'tunLight'], false],
  // a cube on its corner: seated on a POINT (the no-pad gate: no walls, no last resort, no fill)
  ['cube-pt', 'cube', 'seat:45,35,0', ['auto', 'autoNoPad', 'fullNoPad', 'propNoPad', 'stabilize', 'full'], false],
  ['cube-40', 'cube', 'seat:40,0,0', ['prop', 'bogus', 'draw', 'tunSure', 'tunLight', 'autoNoPad', 'stabilize'], false],
];

// ---- dumping ----------------------------------------------------------------------
const F = new Float64Array(1), U = new Uint32Array(F.buffer);
const hv = (h, x) => { F[0] = x; h = Math.imul(h ^ U[0], 16777619) >>> 0; return Math.imul(h ^ U[1], 16777619) >>> 0; };
function hashTris(tris, a = 0, b = tris.length) {
  let h = 2166136261;
  for (let i = a; i < b; i++) { const v = tris[i]; h = hv(h, v[0]); h = hv(h, v[1]); h = hv(h, v[2]); }
  return h;
}
function hashTags(tris) {
  let h = 2166136261, n = 0;
  for (const v of tris) { const t = v[3] === 1 ? 1 : 0; n += t; h = Math.imul(h ^ t, 16777619) >>> 0; }
  return { h, n };
}
const trisOf = (tris) => ({ n: tris.length, hash: hashTris(tris), tags: hashTags(tris) });
const line3 = (l) => (l ? l.map((p) => [p[0], p[1], p[2]]) : null);
const propOf = (q) => ({
  region: q.region ?? null, tines: q.tines ?? null, span: q.span ?? null, height: q.height ?? null, area: q.area ?? null,
  volume: q.volume ?? null, grip: q.grip ?? null, stations: q.stations ?? null, trimmed: q.trimmed ?? null,
  id: q.id ?? null, kind: q.kind ?? null, line: line3(q.line), triRanges: q.triRanges ?? null,
  partAttached: q.partAttached ?? null, inBore: q.inBore ?? null, squat: q.squat ?? null, raster: q.raster ?? null,
  short: q.short ?? null, fill: q.fill ?? null, joined: q.joined ?? null, braces: q.braces ?? null, ties: q.ties ?? null,
});
function builtOf(b) {
  return {
    triangles: trisOf(b.triangles), padTriangles: trisOf(b.padTriangles ?? []),
    pad: b.pad ?? null, mode: b.mode ?? null,
    fins: (b.fins ?? []).map((f) => ({
      id: f.id, kind: f.kind, height: f.height, length: f.length, span: f.span ?? null, tines: f.tines,
      triRanges: f.triRanges, line: line3(f.line), fill: f.fill ?? false, joined: f.joined ?? false, short: f.short ?? false,
      hashes: f.triRanges.map(([a, z]) => hashTris(b.triangles, a, z)),
    })),
    props: (b.props ?? []).map(propOf),
    volume: b.volume ?? null, skipped: b.skipped ?? null, servedRegions: b.servedRegions ?? null,
    braceCount: b.braceCount ?? null, propCount: b.propCount ?? null, tines: b.tines ?? null, unserved: b.unserved ?? null,
    sway: b.sway ? { ...b.sway, braces: b.sway.braces.map((r) => ({ foot: line3(r.foot), halfW: r.halfW, th: r.th, height: r.height,
      levels: r.levels.map((l) => ({ z: l.z, a: [l.a[0], l.a[1], l.a[2]], b: [l.b[0], l.b[1], l.b[2]] })) })) } : null,
    fill: b.fill ? { ...b.fill, unservedPts: b.fill.unservedPts } : null,
    floating: (b.floating ?? []).map((p) => ({ faces: p.faces, lowest: [p.lowest[0], p.lowest[1], p.lowest[2]], drop: p.drop })),
    seating: b.seating ?? null, sagRisk: b.sagRisk ?? null,
  };
}
const stateOf = () => ({ FINpadH: FIN.padH, PADgrab: PAD.grab, PADstyle: PAD.style, PADcustom: { ...PAD.custom },
  PROPgap: PROP.gap, PERPgap: PERP.gap, PROPiface: PROP.iface === false ? 'off' : PROP.iface,
  PROPtinesPerWall: PROP.tinesPerWall, CUTpattern: CUT.pattern });

const PROP0 = { ...PROP }, PERP0 = { ...PERP }, CUT0 = { ...CUT }, FIN0 = { ...FIN };
const PAD0 = { ...PAD, custom: { ...PAD.custom } }, FILL0 = { ...FILL, shifts: [...FILL.shifts] };
function reset() {
  Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0); Object.assign(FIN, FIN0);
  Object.assign(PAD, { ...PAD0, custom: { ...PAD0.custom } }); Object.assign(FILL, { ...FILL0, shifts: [...FILL0.shifts] });
}
const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);

function optsFor(name) {
  const c = C[name];
  const { material = 'pla', padStyle = 'auto', cutout = 'none', padCustom, iface, tinesPerWall, FILL: fillCfg, tunables: tun, ...rest } = c;
  const mat = MATERIAL[material];
  const opts = { ...BASE, ...rest };
  if (tun === null) opts.tunables = undefined;
  else {
    opts.tunables = { padH: mat.padH, padGrab: mat.padGrab, propGap: mat.propGap, padStyle, cutout };
    if (padCustom) opts.tunables.padCustom = padCustom;
    if (iface !== undefined) opts.tunables.iface = iface;
    if (tinesPerWall !== undefined) opts.tunables.tinesPerWall = tinesPerWall;
  }
  // computeFins' sway merge: the material's gap unless the option set has its own
  if (opts.sway) opts.sway = { gap: mat.propGap, ...opts.sway };
  return { opts, fillCfg: fillCfg ?? null };
}

let total = 0;
for (const [name, src, pose, cfgs, internals] of SCENES) {
  if (filter && !name.includes(filter)) continue;
  const t0 = performance.now();
  const raw = typeof src === 'function' ? src()
    : readSTL(path.join(SF, src.includes('/') ? src : path.join('prototype/stress/models', `${src}.stl`)));
  let soup, rot, seated;
  if (pose.startsWith('seat:')) {
    const P = sitePose(pose.slice(5).split(',').map(Number));
    const posed = new Float64Array(raw.length);
    for (let i = 0; i < raw.length; i += 3) {
      const x = raw[i], y = raw[i + 1], z = raw[i + 2];
      posed[i] = P[0] * x + P[3] * y + P[6] * z;
      posed[i + 1] = P[1] * x + P[4] * y + P[7] * z;
      posed[i + 2] = P[2] * x + P[5] * y + P[8] * z;
    }
    soup = posed; seated = true; rot = IDENTITY3;
  } else {
    soup = raw; seated = false;
    rot = pose.startsWith('rotX:') ? rotX(Number(pose.slice(5))) : IDENTITY3;
  }
  const mkTopo = () => {
    const pos = seated ? seatSoup(soup).pos : soup;
    return buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  };
  const topo = mkTopo();
  const result = analyze(topo, 45, rot);

  const builds = [];
  for (const cfg of cfgs) {
    reset();
    const { opts, fillCfg } = optsFor(cfg);
    if (fillCfg) Object.assign(FILL, fillCfg);
    const b = fins.buildFins(topo, result, rot, opts);
    const { tunables, ...rest } = opts;
    builds.push({ cfg, opts: rest, tunables: tunables ?? null, fillCfg, out: builtOf(b), state: stateOf() });
  }

  // ---- the internals, on Auto's result (PLA defaults) ----
  let inner = null;
  if (internals) {
    reset();
    const { opts } = optsFor('auto');
    const auto = fins.buildFins(topo, result, rot, opts);
    inner = { covs: [] };
    for (const coverage of [0.5, 1]) {
      const R = fill.fillReach(coverage);
      const tops = [];
      for (const w of auto.fins ?? []) for (const p of w.line ?? []) tops.push(p);
      const near = reachOf(tops, R);
      const cls = classify(topo, result, rot);
      const pts = fill.__leftoverPoints(topo, result, rot, near, cls);
      const groups = fill.__groupsOf(pts);
      const heavy = groups.map((g) => ({ g, w: g.reduce((s, i) => s + pts[i][3], 0) })).sort((p, q) => q.w - p.w).map((x) => x.g);
      const g0 = heavy[0] ?? [];
      const lines = (l) => l.map((c) => ({ a: [...c.a], b: [...c.b], weight: c.weight, aims: c.aims }));
      inner.covs.push({
        coverage, R, pts, groups,
        cand: g0.length ? lines(fill.__candidateLines(pts, g0, R, FILL.bridge)) : [],
        cand2: g0.length ? lines(fill.__candidateLines(pts, g0, R, 2 * FILL.sample)) : [],
        candNarrow: g0.length ? lines(fill.__candidateLines(pts, g0, R, FILL.bridge, true)) : [],
        local: g0.length ? fill.__localPatch(pts, g0) : [],
      });
    }
    // fillCoverage directly on Auto's result (withFill's call), coverage 0.5 and 0
    inner.fill = [];
    for (const coverage of [0.5, 0]) {
      const f = fill.fillCoverage(topo, result, rot, { ...opts, tines: true, coverage }, auto);
      inner.fill.push({ coverage, triangles: trisOf(f.triangles), props: f.props.map(propOf), tines: f.tines,
                        unserved: f.unserved, stats: f.stats });
    }
    // lastResortWalls: every region bare (no served list, no wedges, nothing built), and
    // against Auto's triangles
    const sopts = { ...opts, tines: true, coverage: 0.5 };
    inner.short = [];
    for (const [label, existing] of [['bare', []], ['auto', auto.triangles]]) {
      const s = lastResortWalls(topo, result, rot, sopts, [], [], existing);
      inner.short.push({ label, triangles: trisOf(s.triangles), props: s.props.map(propOf), served: s.served, tines: s.tines });
    }
  }

  const json = JSON.stringify({ scene: name, pose, seated, rot, soup: Array.from(soup), regions: result.regions.length,
                                defaults: { FILL: FILL0, coverPitch: [0, 0.25, 0.5, 0.75, 1].map((c) => fins.__coverPitch(c)),
                                            fillReach: [0, 0.5, 0.75, 1].map((c) => fill.fillReach(c)) },
                                builds, inner }, replacer);
  writeFileSync(path.join(outDir, `${name}.json`), json);
  total += json.length;
  console.log(name, 'regions', result.regions.length,
    builds.map((b) => `${b.cfg}:${b.out.fins.length}${b.out.fill ? `+f${b.out.fill.walls}` : ''}`).join(' '),
    inner ? `pts ${inner.covs[0].pts.length} fillWalls ${inner.fill.map((f) => f.stats.walls).join('/')} short ${inner.short.map((s) => s.props.length).join('/')}` : '',
    `${(json.length / 1e6).toFixed(1)}MB ${Math.round(performance.now() - t0)}ms`);
}
reset();
console.log(`total ${(total / 1e6).toFixed(1)}MB -> ${outDir}`);
