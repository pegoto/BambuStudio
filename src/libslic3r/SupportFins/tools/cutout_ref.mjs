#!/usr/bin/env node
// Reference oracle for Engine/Cutout.{hpp,cpp}: imports upstream web/cutout.js directly
// and dumps holesFor over a grid, and cutWall on synthetic walls built the way its two
// consumers build them (prop/sweep.js sweep + sweepBetween, fins/wedges.js emitBlade),
// for each CUTOUT_PATTERN. Usage:
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/cutout_ref.mjs [outDir]
// outDir defaults to refs_cutout/ next to tools/. tests/test_cutout.cpp reads it.
// The station builders below are copies of the callers' loops; each sweep/sweepBetween
// case is checked against the real upstream function (its output must begin with /
// equal what cutWall-or-ribbon emits here), so the copies cannot drift silently.
import { mkdirSync, writeFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_cutout');
mkdirSync(outDir, { recursive: true });

const web = (f) => import(pathToFileURL(path.join(SF, 'web', f)).href);
const { CUT, CUTOUT_PATTERNS, holesFor, cutWall } = await web('cutout.js');
const { ribbon } = await web('solids.js');
const { PROP } = await web('prop/config.js');
const { footFor, sweep, sweepBetween } = await web('prop/sweep.js');
const { PERP } = await web('fins/wedges.js');

const flat = (out) => out.flatMap((v) => [v[0], v[1], v[2]]);

// ---------------------------------------------------------------- holesFor grid
const kinds = [...CUTOUT_PATTERNS, 'bogus'];
const ws = [0, 3, 4, 4.0000001, 5.6, 6.5, 8, 9.25, 10, 13.3, 20];
const Hs = [0, 2.9, 3, 3.5, 4, 5.5, 7.3, 9.9, 12, 20, 33.3, 47.1, 60, 100];
const hf = [];
for (const kind of kinds) for (const w of ws) for (const H of Hs) {
  hf.push({ kind, w, H, holes: holesFor(kind, w, H) });
}
writeFileSync(path.join(outDir, 'holesFor.json'), JSON.stringify({ CUT, CUTOUT_PATTERNS, cases: hf }));

// ---------------------------------------------------------------- station builders
// prop/sweep.js sweep(), interface crest off (kinds null): returns { st, wall } or null.
function sweepStations(line, zBed, minH = PROP.minHeight) {
  const wall = [], flange = [], st = [];
  for (let i = 0; i < line.length; i++) {
    const p = line[i];
    const a = line[Math.max(0, i - 1)], b = line[Math.min(line.length - 1, i + 1)];
    let rx = b[0] - a[0], ry = b[1] - a[1];
    const rn = Math.hypot(rx, ry);
    if (rn < 1e-9) return null;
    rx /= rn; ry /= rn;
    const sx = ry, sy = -rx;
    const top = p[2] - PROP.gap;
    const h = top - zBed;
    if (h < minH) return null;
    const foot = footFor(h);
    const flangeH = Math.min(PROP.baseH, h / 2);
    const ztip = Math.max(top - PROP.tipH, zBed + flangeH + 0.1);
    const baseTop = zBed + flangeH;
    const P = (o, z) => [p[0] + sx * o, p[1] + sy * o, z];
    const [zt, zTop, wTop] = [ztip, top, PROP.tip / 2];
    wall.push([P(+PROP.th / 2, zBed), P(+PROP.th / 2, zt), P(+wTop, zTop),
               P(-wTop, zTop), P(-PROP.th / 2, zt), P(-PROP.th / 2, zBed)]);
    flange.push([P(+foot, zBed), P(+foot, baseTop), P(-foot, baseTop), P(-foot, zBed)]);
    st.push({ p, sx, sy, top: zTop, ztip: zt, holeTop: ztip, tipHalf: wTop, bot: zBed, botTip: baseTop, taperBot: false });
  }
  return { st, wall, flange };
}

// prop/sweep.js sweepBetween(), interface crest off.
function betweenStations(topLine, botLine, minH = PROP.minHeight) {
  const welded = PROP.footGap <= 0;
  const wall = [], st = [];
  for (let i = 0; i < topLine.length; i++) {
    const p = topLine[i];
    const a = topLine[Math.max(0, i - 1)], b = topLine[Math.min(topLine.length - 1, i + 1)];
    let rx = b[0] - a[0], ry = b[1] - a[1];
    const rn = Math.hypot(rx, ry);
    if (rn < 1e-9) return null;
    rx /= rn; ry /= rn;
    const sx = ry, sy = -rx;
    const top = p[2] - PROP.gap;
    const g = botLine[i][2] > 0 ? PROP.footGap : 0;
    const bot = botLine[i][2] + g;
    if (top - botLine[i][2] < minH) return null;
    const side = (k) => (welded || botLine[i].length < 5 ? bot : Math.min(botLine[i][k] + g, top - 0.5));
    const bN = side(3), bP = side(4);
    const bHi = Math.max(bN, bP);
    const h = top - bHi;
    const taper = Math.min(PROP.tipH, welded ? h / 2 : h - 0.1);
    const zBotTip = welded ? bot + taper : bHi;
    const P = (o, z) => [p[0] + sx * o, p[1] + sy * o, z];
    const [zTopTip, zTop, wTop] = [top - taper, top, PROP.tip / 2];
    wall.push(welded ? [
      P(+PROP.tip / 2, bot), P(+PROP.th / 2, zBotTip), P(+PROP.th / 2, zTopTip), P(+wTop, zTop),
      P(-wTop, zTop), P(-PROP.th / 2, zTopTip), P(-PROP.th / 2, zBotTip), P(-PROP.tip / 2, bot),
    ] : [
      P(+PROP.th / 2, bP), P(+PROP.th / 2, zTopTip), P(+wTop, zTop),
      P(-wTop, zTop), P(-PROP.th / 2, zTopTip), P(-PROP.th / 2, bN),
    ]);
    st.push({ p, sx, sy, top: zTop, ztip: zTopTip, holeTop: top - taper, tipHalf: wTop, bot: bHi, botTip: zBotTip, taperBot: welded, botN: bN, botP: bP });
  }
  return { st, wall };
}

// fins/wedges.js emitBlade(): stations + full sections (CUT_TOP 0.05, half PERP.th/2).
function bladeStations(top, half = PERP.th / 2) {
  const st = [], full = [];
  for (let i = 0; i < top.length; i++) {
    const a = top[Math.max(0, i - 1)], b = top[Math.min(top.length - 1, i + 1)];
    let rx = b[0] - a[0], ry = b[1] - a[1];
    const rn = Math.hypot(rx, ry);
    if (rn < 1e-9) { st.length = 0; break; }
    rx /= rn; ry /= rn;
    const sx = ry, sy = -rx;
    const p = top[i], z = p[2];
    const P = (o, zz) => [p[0] + sx * o, p[1] + sy * o, zz];
    full.push([P(+half, 0), P(+half, z), P(-half, z), P(-half, 0)]);
    st.push({ p, sx, sy, top: z, ztip: z - 0.05, bot: 0, botTip: PERP.footH, taperBot: false });
  }
  return { st, full };
}

// ---------------------------------------------------------------- lines
// A polyline through XY `path` (piecewise linear in a param u in [0,1]) sampled at
// ~stationStep, with z = zf(s) (s = arclength).
function lineAlong(len, zf, { dir = [1, 0], origin = [0, 0], curve = 0, step = PROP.stationStep } = {}) {
  const n = Math.max(1, Math.ceil(len / step));
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const s = (len * i) / n;
    let x, y;
    if (curve) {                         // arc of radius 1/curve, starting along dir
      const R = 1 / curve, th = s / R;
      const lx = R * Math.sin(th), ly = R * (1 - Math.cos(th));
      x = origin[0] + dir[0] * lx - dir[1] * ly; y = origin[1] + dir[1] * lx + dir[0] * ly;
    } else { x = origin[0] + dir[0] * s; y = origin[1] + dir[1] * s; }
    pts.push([x, y, zf(s, len)]);
  }
  return pts;
}
const d45 = [Math.SQRT1_2, Math.SQRT1_2], d30 = [Math.cos(0.5236), Math.sin(0.5236)];
const tops = {
  flat: (h) => () => h,
  steep: (h) => (s) => 2 + h + 1.5 * s,             // a 56deg roof rising along the wall
  steepDown: (h) => (s, L) => 2 + 1.5 * (L - s),
  shallow: (h) => (s) => h + 0.3 * s,
  peak: (h) => (s, L) => h + 0.9 * Math.min(s, L - s),
  wavy: (h) => (s) => h + 2.5 * Math.sin(s / 4),
  gable45: (h) => (s, L) => 2 + Math.min(s, L - s),  // a fin under a 45deg face both ways
};

const cases = [];
const wallOf = (w) => ({ th: w.th, tip: w.tip, minStations: w.minStations });
function record(name, builder, st, full, wall, extra = {}) {
  const out = [];
  const ret = cutWall(st, full, out, wall);
  cases.push({ name, builder, pattern: CUT.pattern, wall: wallOf(wall), st, full, ret, tris: flat(out), ...extra });
  return { ret, out };
}
const sameNums = (a, b) => a.length === b.length && a.every((v, i) => Object.is(v, b[i]) || v === b[i]);

// sweep walls: (name, len, top, h, zBed, opts)
const sweepWalls = [
  ['short6', 6, 'flat', 12, 0, {}],
  ['short11', 11, 'flat', 12, 0, {}],
  ['one14', 14, 'flat', 12, 0, {}],
  ['low30', 30, 'flat', 3.2, 0, {}],
  ['mid30', 30, 'flat', 6, 0, {}],
  ['tall30', 30, 'flat', 14, 0, {}],
  ['tall60', 60, 'flat', 25, 0, { dir: d30, origin: [5, -3] }],
  ['bed2', 40, 'flat', 12, 2, { dir: [0, -1] }],
  ['steep25', 25, 'steep', 0, 0, {}],
  ['steepDown25', 25, 'steepDown', 0, 0, { dir: d45 }],
  ['shallow40', 40, 'shallow', 6, 0, {}],
  ['peak36', 36, 'peak', 4, 0, {}],
  ['gable40', 40, 'gable45', 0, 0, {}],
  ['wavy50', 50, 'wavy', 9, 0, {}],
  ['curved40', 40, 'flat', 10, 0, { curve: 1 / 30 }],
  ['curvedTight30', 30, 'shallow', 8, 0, { curve: 1 / 12, dir: d45 }],
  ['coarse33', 33, 'flat', 15, 0, { step: 2.7 }],
];
const PATTERNS = [...CUTOUT_PATTERNS];
let checked = 0;
for (const pattern of [...PATTERNS, 'bogus']) {
  CUT.pattern = pattern;
  for (const [nm, len, tk, h, zBed, opts] of sweepWalls) {
    if (pattern === 'bogus' && nm !== 'tall30') continue;
    const line = lineAlong(len, tops[tk](h + zBed), opts);
    const b = sweepStations(line, zBed);
    if (!b) throw new Error(`sweep ${nm}: builder declined`);
    const { ret, out } = record(`sweep-${nm}-${pattern}`, 'sweep', b.st, b.wall, PROP);
    // check the copy against upstream sweep(): its out = (cut wall | ribbon(wall)) + ribbon(flange)
    const up = [];
    if (!sweep(line, zBed, up)) throw new Error(`sweep ${nm}: upstream declined`);
    const mine = out.slice(); if (!ret) ribbon(b.wall, mine); ribbon(b.flange, mine);
    if (!sameNums(flat(up), flat(mine))) throw new Error(`sweep ${nm}/${pattern}: builder copy disagrees with upstream`);
    checked++;
  }
}

// sweepBetween walls: (name, len, top, h, floor(s, L) -> [z] or [z, zN, zP], welded)
const betweenWalls = [
  ['plate30', 30, 'flat', 12, () => [0], false],
  ['part30', 30, 'flat', 14, () => [2], false],
  ['welded30', 30, 'flat', 14, () => [2], true],
  ['weldedLow30', 30, 'flat', 5, () => [1.5], true],
  ['slopeFloor45', 45, 'shallow', 16, (s) => [1 + 0.2 * s], false],
  ['tilted40', 40, 'flat', 15, (s) => [2, 2 + 0.05 * s, 2.6], false],
  ['tiltedSteep30', 30, 'steep', 4, (s) => [1, 1.4, 1 + 0.1 * s], false],
  ['weldedPeak36', 36, 'peak', 8, (s) => [1 + 0.1 * s], true],
  ['short10', 10, 'flat', 12, () => [1], false],
];
for (const pattern of PATTERNS) {
  CUT.pattern = pattern;
  for (const [nm, len, tk, h, floor, welded] of betweenWalls) {
    const keep = PROP.footGap;
    if (welded) PROP.footGap = 0;
    try {
      const topLine = lineAlong(len, tops[tk](h), { dir: d30 });
      const botLine = topLine.map((p, i) => {
        const s = (len * i) / (topLine.length - 1);
        const f = floor(s, len);
        return f.length === 1 ? [p[0], p[1], f[0]] : [p[0], p[1], f[0], f[1], f[2]];
      });
      const b = betweenStations(topLine, botLine);
      if (!b) throw new Error(`between ${nm}: builder declined`);
      const { ret, out } = record(`between-${nm}-${pattern}`, 'sweepBetween', b.st, b.wall, PROP);
      const up = [];
      if (!sweepBetween(topLine, botLine, up)) throw new Error(`between ${nm}: upstream declined`);
      const mine = out.slice(); if (!ret) ribbon(b.wall, mine);
      if (!sameNums(flat(up), flat(mine))) throw new Error(`between ${nm}/${pattern}: builder copy disagrees with upstream`);
      checked++;
    } finally { PROP.footGap = keep; }
  }
}

// wedge blades: a contact profile up a leaning face, top = contact - gap
const blades = [
  ['face45', 30, 45, [1, 0]],
  ['face30', 40, 30, d45],
  ['face60', 25, 60, [0, 1]],
  ['short', 8, 45, [1, 0]],
  ['twoStations', 3, 45, [1, 0]],
];
const bladeWall = { th: PERP.th, tip: PERP.th, minStations: 3 };
for (const pattern of PATTERNS) {
  CUT.pattern = pattern;
  for (const [nm, len, deg, dir] of blades) {
    const r = deg * Math.PI / 180;
    const nT = Math.max(2, Math.ceil(len / PERP.tStep));
    const top = [];
    for (let i = 0; i <= nT; i++) {
      const t = (len * i) / nT;
      if (nm === 'twoStations' && i > 1) break;
      top.push([3 + dir[0] * t * Math.cos(r), -1 + dir[1] * t * Math.cos(r), 0.5 + t * Math.sin(r) - PERP.gap]);
    }
    const b = bladeStations(top);
    record(`blade-${nm}-${pattern}`, 'emitBlade', b.st, b.full, bladeWall);
  }
}

// Crest-shaped stations (sweep with the interface crest on: holeTop below ztip's
// position, tipHalf narrower than tip / 2) and a minStations override.
for (const pattern of PATTERNS) {
  CUT.pattern = pattern;
  const line = lineAlong(36, tops.flat(15), { dir: d45 });
  const b = sweepStations(line, 0);
  const st = b.st.map((q, i) => ({ ...q, ztip: q.top - 0.35 - 0.01 * (i % 3), tipHalf: 0.5 - 0.02 * (i % 2) }));
  record(`crest-${pattern}`, 'sweep', st, b.wall, PROP);
  record(`minStations-${pattern}`, 'sweep', b.st, b.wall, { th: PROP.th, tip: PROP.tip, minStations: 50 });
}

CUT.pattern = 'none';
writeFileSync(path.join(outDir, 'cutWall.json'), JSON.stringify({ CUT, cases }));
const nTrue = cases.filter((c) => c.ret).length;
console.log(`holesFor: ${hf.length} cases; cutWall: ${cases.length} cases (${nTrue} cut), `
  + `${checked} builder copies verified against upstream sweep/sweepBetween -> ${outDir}`);
