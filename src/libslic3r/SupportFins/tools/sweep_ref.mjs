#!/usr/bin/env node
// Reference oracle for Engine/Prop/Sweep, Clearance, Squat (web/prop/sweep.js,
// clearance.js, squat.js). On stress models posed like tools/ref.mjs, it builds contact
// lines the way prop.js buildPass does (tubeLine / splitRegion / patchTracks), then runs
// prop.js's plate path, attached.js's part-attached path and draw.js-style calls over
// them, recording every call into the three modules (inputs, outputs, triangles in
// emission order, interface tags) for tests/test_sweep.cpp. Configs cover CUT.pattern
// none/diamond, PROP.iface off/'flat'/'all', tines on/off, gap 0.2/0.3, footGap 0.
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/sweep_ref.mjs [outDir]
// outDir defaults to refs_sweep/ next to this tools/ directory.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_sweep');
mkdirSync(outDir, { recursive: true });
const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);

const { PROP } = await imp('web/prop/config.js');
const { CUT } = await imp('web/cutout.js');
const surface = await imp('web/prop/surface.js');
const contact = await imp('web/prop/contact.js');
const mold = await imp('web/prop/mold.js');
const crest = await imp('web/prop/crest.js');
const SW = await imp('web/prop/sweep.js');
const CL = await imp('web/prop/clearance.js');
const SQ = await imp('web/prop/squat.js');
const tracks = await imp('web/prop/tracks.js');
const tines = await imp('web/prop/tines.js');
const attached = await imp('web/prop/attached.js');
const { insidePart, solidClearance } = await imp('web/inside.js');
const { buildTopology, analyze, IDENTITY3, MIN_REGION_AREA } = await imp('web/overhangs.js');
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

// ---- state ---------------------------------------------------------------------------
const PROP0 = { ...PROP };
const flat = (tris) => { const a = []; for (const v of tris) a.push(v[0], v[1], v[2]); return a; };
const tagsOf = (tris) => tris.map((v) => (v[3] === 1 ? 1 : 0));
const copyLine = (l) => l.map((p) => [p[0], p[1], p[2]]);
const PART_BAND = attached.PART_BAND;
const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);
const hitOf = (h) => (h ? { d: h.d, cosUp: h.cosUp, x: h.x, y: h.y, z: h.z } : null);

// The configs every scene runs (the C++ applies the same: PROP defaults + prop, CUT.pattern,
// crestPart(crestOn() ? part : null, layerHeight) as fins.js does).
const CFGS = [
  { prop: {}, pattern: 'none', tines: false, layerHeight: null, plateOnly: false },
  { prop: {}, pattern: 'diamond', tines: true, layerHeight: null, plateOnly: false },
  { prop: { iface: 'flat' }, pattern: 'none', tines: true, layerHeight: 0.2, plateOnly: false },
  { prop: { gap: 0.3, iface: 'all', ifaceLayers: 2 }, pattern: 'diamond', tines: false, layerHeight: 0.16, plateOnly: false },
  { prop: { gap: 0.3, iface: 'all', footGap: 0 }, pattern: 'none', tines: true, layerHeight: null, plateOnly: true },
  { prop: { iface: 'flat', ifaceLayers: 3, footGap: 0 }, pattern: 'diamond', tines: false, layerHeight: 0.12, plateOnly: true },
  { prop: { gap: 0.3 }, pattern: 'none', tines: false, layerHeight: null, plateOnly: false },
];
function applyCfg(cfg, partTris) {
  Object.assign(PROP, PROP0, cfg.prop);
  CUT.pattern = cfg.pattern;
  crest.crestPart(crest.crestOn() ? partTris : null, cfg.layerHeight ?? undefined);
}
function resetAll() { Object.assign(PROP, PROP0); CUT.pattern = 'none'; crest.crestPart(null); }

// ---- the recorders: each call into sweep/clearance/squat goes through one of these ----
let cases = [], curCfg = -1, topo = null, rot = IDENTITY3, off = null;
function recSweep(line, zBed, out, minH, src, check = false) {
  const inp = copyLine(line), from = out.length;
  const ret = minH === undefined ? SW.sweep(line, zBed, out) : SW.sweep(line, zBed, out, minH);
  const c = { fn: 'sweep', cfg: curCfg, src, line: inp, zBed, minH: minH ?? null, from, ret,
              out: flat(out.slice(from)), tags: tagsOf(out.slice(from)) };
  if (check && ret) {
    const hit = solidClearance(topo, rot, off, out.slice(from), 0.25);
    c.hit = hitOf(hit); c.welds = hit ? CL.welds(hit) : null;
  }
  cases.push(c);
  return ret;
}
function recBetween(top, floor, out, minH, src) {
  const from = out.length;
  const ret = minH === undefined ? SW.sweepBetween(top, floor, out) : SW.sweepBetween(top, floor, out, minH);
  cases.push({ fn: 'sweepBetween', cfg: curCfg, src, top: copyLine(top), floor: floor.map((f) => [...f]),
               minH: minH ?? null, from, ret, out: flat(out.slice(from)), tags: tagsOf(out.slice(from)) });
  return ret;
}
function recSquat(line, zBed, out, src) {
  const inp = copyLine(line), from = out.length;
  const ret = SQ.sweepSquat(line, zBed, out);
  cases.push({ fn: 'sweepSquat', cfg: curCfg, src, line: inp, zBed, from, ret,
               out: flat(out.slice(from)), tags: tagsOf(out.slice(from)) });
  return ret;
}
function recSquatBed(line, regionId, regionTris, out, claimed) {
  const inp = copyLine(line), cl = claimed ? [...claimed] : null, from = out.length;
  const placed = SQ.buildSquatBed(line, regionTris, topo, rot, off, out, claimed);
  cases.push({ fn: 'buildSquatBed', cfg: curCfg, line: inp, tris: regionId, claimed: cl, from,
               placed, out: flat(out.slice(from)), tags: tagsOf(out.slice(from)) });
  return placed;
}
function recClear(line) {
  const res = line.map((_, k) => CL.stationIsClear(line, k, topo, rot, off));
  cases.push({ fn: 'stationIsClear', cfg: curCfg, line: copyLine(line), res,
               path: CL.pathToPlateIsClear(line, topo, rot, off) });
  return res;
}
function recCert(line) {
  const res = line.map((_, k) => CL.stationCertified(line, k, topo, rot, off));
  cases.push({ fn: 'stationCertified', cfg: curCfg, line: copyLine(line), res });
  return res;
}
function recRun(usable) {
  const r = CL.longestRun(usable);
  cases.push({ fn: 'longestRun', inp: [...usable], out: r });
  return r;
}
function recTails(tall, low) {
  const r = CL.withLowTails(tall, low);
  cases.push({ fn: 'withLowTails', tall: [...tall], low: [...low], out: r });
  return r;
}
function recBody(pts) {
  const body = CL.bodyMask(pts);
  const span = [];
  for (const st of [false, true]) for (const sh of [false, true]) span.push(CL.minSpanFor(pts, body, st, sh));
  cases.push({ fn: 'bodyMask', cfg: curCfg, pts: copyLine(pts), out: body, tallBody: CL.tallBody(body),
               tallSpan: CL.tallSpan(pts, body), minSpanFor: span });
  return body;
}
function recIFS(line) {
  const inp = copyLine(line);
  CL.insertFloorStations(line);
  cases.push({ fn: 'insertFloorStations', cfg: curCfg, inp, out: copyLine(line) });
  return line;
}

// ---- attached.js buildPartAttached, copied with its sweepBetween / longestRun recorded
// (verified against upstream on a scratch `out` for every line) ------------------------
const BORE = { radius: 10, dirs: 8, walledMin: 6, step: 0.5 };
const zMidAt = (top, floor, k) => (floor[k][2] + (top[k][2] - PROP.gap)) / 2;
const buriedColumn = (top, floor, k) => insidePart(topo, rot, off, top[k][0], top[k][1], zMidAt(top, floor, k));
function enclosedColumn(top, floor, k) {
  const p = top[k], zMid = zMidAt(top, floor, k);
  let walled = 0;
  for (let d = 0; d < BORE.dirs; d++) {
    const ang = (d / BORE.dirs) * 2 * Math.PI;
    const dx = Math.cos(ang), dy = Math.sin(ang);
    for (let r = BORE.step; r <= BORE.radius; r += BORE.step) {
      if (insidePart(topo, rot, off, p[0] + dx * r, p[1] + dy * r, zMid)) { walled++; break; }
    }
  }
  return walled >= BORE.walledMin;
}
function clearBetween(top, floor, k) {
  const p = top[k], a = top[Math.max(0, k - 1)], b = top[Math.min(top.length - 1, k + 1)];
  const rx = b[0] - a[0], ry = b[1] - a[1];
  const rn = Math.hypot(rx, ry);
  if (rn < 1e-9) return true;
  const sx = ry / rn, sy = -rx / rn;
  const zTop = p[2] - PROP.gap, zBot = floor[k][2];
  const nP = Math.max(3, Math.ceil((zTop - zBot) / 1.5));
  for (let i = 1; i < nP; i++) {
    const z = zBot + ((zTop - zBot) * i) / nP;
    for (const m of [0.12, 0.24, PROP.sideClear]) {
      const w = PROP.th / 2 + m;
      if (insidePart(topo, rot, off, p[0] + sx * w, p[1] + sy * w, z)) return false;
      if (insidePart(topo, rot, off, p[0] - sx * w, p[1] - sy * w, z)) return false;
    }
  }
  return true;
}
function partAttached(line, partTris, out) {
  const top = copyLine(line);
  contact.contourTop(top, partTris, PART_BAND);
  contact.lowerSag(top, partTris, PART_BAND);
  contact.settleTop(top, partTris, 0.25, PART_BAND);
  const floor = mold.floorLine(top, partTris);
  let real = 0;
  for (const f of floor) if (f[2] > PROP.gap + 0.5) real++;
  if (real < Math.max(PROP.minStations, Math.ceil(floor.length * 0.5))) return {};
  const why = [];
  const ok = top.map((p, k) => {
    if (floor[k][2] <= PROP.gap + 0.5) return false;
    if ((p[2] - PROP.gap) - floor[k][2] < PROP.minHeight) { why.push('stub'); return false; }
    if (buriedColumn(top, floor, k)) { why.push('buried'); return false; }
    if (clearBetween(top, floor, k)) return true;
    why.push('blocked');
    return false;
  });
  const run = recRun(ok);
  if (!run || run[1] - run[0] < PROP.minStations) {
    const n = {};
    for (const w of why) n[w] = (n[w] ?? 0) + 1;
    const worst = Object.keys(n).sort((a, b) => n[b] - n[a])[0];
    return { floored: why.length * 2 >= floor.length && worst ? worst : 'stub' };
  }
  const subTop = top.slice(run[0], run[1]);
  const subFloor = floor.slice(run[0], run[1]);
  const span = Math.hypot(subTop[subTop.length - 1][0] - subTop[0][0], subTop[subTop.length - 1][1] - subTop[0][1]);
  if (span < PROP.minSpanPart) return { floored: 'stub' };
  const before = out.length;
  const m = mold.moldLine(subTop, partTris);
  if (!recBetween(m.top, m.floor, out, undefined, 'attached')) { out.length = before; return { floored: 'degenerate' }; }
  let walled = 0;
  for (let k = run[0]; k < run[1]; k++) if (enclosedColumn(top, floor, k)) walled++;
  const inBore = walled * 2 > run[1] - run[0];
  let height = 0, vol = 0;
  for (let i = 0; i < subTop.length; i++) height = Math.max(height, (subTop[i][2] - PROP.gap) - subFloor[i][2]);
  for (let i = before; i < out.length; i += 3) {
    const a = out[i], b = out[i + 1], c = out[i + 2];
    vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
          + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
  }
  return { ok: true, prop: { span, height, stations: subTop.length, volume: Math.abs(vol), partAttached: true, inBore,
                             line: subTop.map((p) => [p[0], p[1], p[2] - PROP.gap]) } };
}

// ---- prop.js buildPass's plate path for one line, every module call recorded ----------
function platePath(line, patch, regionId, regionTris, partTris, out, plateOnly, withTines) {
  const zBed = 0;
  const tineStepEff = tines.tineStepFor(undefined);
  const tineHeight = curLayer ?? PROP.tineH;
  const tri0 = out.length;
  // verify the attached copy against upstream on scratch arrays
  {
    const mine = [[0, 0, 0], [1, 0, 0], [0, 1, 0]], theirs = [[0, 0, 0], [1, 0, 0], [0, 1, 0]];
    const save = cases.length;
    const a = partAttached(line, partTris, mine);
    cases.length = save;
    const b = attached.buildPartAttached(line, partTris, topo, rot, off, theirs);
    if (JSON.stringify([a, flat(mine), tagsOf(mine)], replacer) !== JSON.stringify([b, flat(theirs), tagsOf(theirs)], replacer)) {
      throw new Error('attached.js copy disagrees with upstream buildPartAttached');
    }
    nVerified++;
  }
  let pa = partAttached(line, partTris, out);
  if (plateOnly && (pa.ok || pa.floored)) { out.length = tri0; pa = {}; }
  if (pa.ok && patch.smallTube) {
    const hit = solidClearance(topo, rot, off, out.slice(tri0), 0.25);
    if (hit && CL.welds(hit)) { out.length = tri0; return; }
  }
  if (pa.ok) {
    if (withTines) {
      const topLine = pa.prop.line.map((p) => [p[0], p[1], p[2] + PROP.gap]);
      tines.emitTines(topLine, partTris, topo, rot, off, out, tineStepEff, undefined, tineHeight, null, { z: Infinity });
    }
    return;
  }
  if (pa.floored) return;

  contact.contourTop(line, regionTris);
  contact.lowerSag(line, regionTris);
  if (recIFS(line).length) contact.contourTop(line, regionTris);

  const squatLine = line.map((p) => [p[0], p[1], p[2]]);
  const claimed = line.map(() => false);
  const runSquat = () => {
    for (const sq of recSquatBed(squatLine, regionId, regionTris, out, claimed)) {
      if (withTines) {
        tines.emitTines(sq.line.map((p) => [p[0], p[1], p[2] + PROP.gap]), regionTris, topo, rot, off, out,
                        tineStepEff, PROP.squatBrimH, tineHeight, null, { z: Infinity });
      }
    }
  };
  if (contact.straightness(line) > PROP.maxWander) { runSquat(); return; }

  const clr = recClear(line);
  const clear = line.map((p, k) => p[2] - PROP.gap >= PROP.minHeightSquat && clr[k]);
  const usable = recTails(line.map((p, k) => clear[k] && p[2] - PROP.gap >= PROP.minHeight), clear);
  const run = recRun(usable);
  if (!run || run[1] - run[0] < PROP.minStations) { runSquat(); return; }
  const sub = line.slice(run[0], run[1]);
  const body = recBody(sub);
  if (CL.tallSpan(sub, body) < CL.minSpanFor(sub, body, !!patch.smallTube, false)) { runSquat(); return; }

  contact.settleTop(sub, regionTris);
  const cert = recCert(sub);
  const lowA = sub.map((p, k) => p[2] - PROP.gap >= PROP.minHeightSquat && cert[k]);
  const tallA = sub.map((p, k) => lowA[k] && p[2] - PROP.gap >= PROP.minHeight);

  let placed = false;
  for (let tries = 0; tries < 4 && !placed; tries++) {
    const run2 = recRun(recTails(tallA, lowA));
    if (!run2 || run2[1] - run2[0] < PROP.minStations) break;
    const settled = sub.slice(run2[0], run2[1]);
    const settledBody = body.slice(run2[0], run2[1]);
    if (CL.tallSpan(settled, settledBody) < CL.minSpanFor(settled, settledBody, !!patch.smallTube, false)) break;
    const before = out.length;
    if (!recSweep(settled, zBed, out, PROP.minHeightSquat, 'plate', true)) { out.length = before; break; }
    const hit = solidClearance(topo, rot, off, out.slice(before), 0.25);
    if (hit && CL.welds(hit)) {
      out.length = before;
      let kBest = 0, dBest = Infinity;
      for (let k = 0; k < settled.length; k++) {
        const dx = settled[k][0] - hit.x, dy = settled[k][1] - hit.y;
        if (dx * dx + dy * dy < dBest) { dBest = dx * dx + dy * dy; kBest = k; }
      }
      const at = run2[0] + kBest;
      for (const k of [Math.max(0, at - 1), at, Math.min(lowA.length - 1, at + 1)]) { lowA[k] = false; tallA[k] = false; }
      continue;
    }
    if (withTines) {
      tines.emitTines(settled, regionTris, topo, rot, off, out, tineStepEff, undefined, tineHeight,
                      CL.tallBody(settledBody), { z: Infinity });
    }
    for (let k = run[0] + run2[0]; k < run[0] + run2[1]; k++) claimed[k] = true;
    placed = true;
  }
  runSquat();
}
let curLayer = null, nVerified = 0;

// ---- draw.js-style calls on a line: the plate wall, squat walls and their segments, and
// a part-attached wall over moldLine (always, so open-plate floors are exercised too) ----
function drawCalls(line, partTris, regionId, regionTris) {
  const scratch = () => { const o = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [2, 0, 0], [3, 0, 0], [2, 1, 0]]; crest.markInterface(o, 3); return o; };
  recSweep(copyLine(line), 0, scratch(), undefined, 'draw');
  recSweep(copyLine(line), 0, scratch(), PROP.minHeightSquat, 'draw');
  recSweep(copyLine(line), 0.25, scratch(), PROP.minHeightSquat, 'draw');
  recSquat(copyLine(line), 0, scratch(), 'draw');
  if (line.length >= 5) {
    recSquat(line.slice(0, 3), 0, scratch(), 'draw');
    recSweep(line.slice(2, line.length - 1), 0, scratch(), undefined, 'draw');
  }
  const top = copyLine(line);
  contact.contourTop(top, partTris, PART_BAND);
  contact.lowerSag(top, partTris, PART_BAND);
  contact.settleTop(top, partTris, 0.25, PART_BAND);
  const m = mold.moldLine(top, partTris);
  recBetween(m.top, m.floor, scratch(), undefined, 'draw');
  recBetween(m.top, m.floor, scratch(), PROP.minHeightSquat, 'draw');
  // floorLine's own (unmolded) floor, as draw.js's squat check reads it
  recBetween(top, mold.floorLine(top, partTris), scratch(), PROP.minHeightSquat, 'draw');
  // squat territory: the same track lowered into the minHeightSquat..minHeight band (with
  // a dip under the floor and a tall bump), with and without claimed stations
  const low = line.map((p, k) => [p[0], p[1], PROP.gap + (k % 9 === 4 ? 0.3 : k % 13 === 7 ? 2.5 : 0.65 + 0.12 * (k % 6))]);
  recSquatBed(low, regionId, regionTris, scratch(), null);
  recSquatBed(low, regionId, regionTris, scratch(), low.map((_, k) => k % 7 === 2));
}

// ---- scenes ------------------------------------------------------------------------
const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'],
  ['staircase', '40,0,0'], ['sphere', '0,0,0'], ['tube', '40,0,0'], ['lowledge', '0,0,0'],
  ['lowledge', '40,0,0'], ['ramp', '40,0,0'], ['plate', '40,0,0'], ['coin', '40,0,0'],
];
const MAX_LINES = 10;
const counts = {};

for (const [model, rotStr] of SCENES) {
  const raw = readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
  const pose = sitePose(rotStr.split(',').map(Number));
  const posed = new Float64Array(raw.length);
  for (let i = 0; i < raw.length; i += 3) {
    const x = raw[i], y = raw[i + 1], z = raw[i + 2];
    posed[i] = pose[0] * x + pose[3] * y + pose[6] * z;
    posed[i + 1] = pose[1] * x + pose[4] * y + pose[7] * z;
    posed[i + 2] = pose[2] * x + pose[5] * y + pose[8] * z;
  }
  const { pos } = seatSoup(posed);
  topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, 45, IDENTITY3);
  rot = IDENTITY3; off = result.offset;
  cases = [];
  const v = [0, 0, 0];

  // prop.js buildPass: the seated part, then per region its tris/pts, tube or patches
  const partTris = new Float64Array(pos.length);
  for (let f = 0; f < topo.nFaces; f++) for (let i = 0; i < 3; i++) {
    surface.seat(pos, f * 9 + i * 3, rot, off, v);
    partTris[f * 9 + i * 3] = v[0]; partTris[f * 9 + i * 3 + 1] = v[1]; partTris[f * 9 + i * 3 + 2] = v[2];
  }
  const trisTable = { part: Array.from(partTris) };
  const step = PROP.stationStep, rowSpan = PROP.maxUnsupportedSpan;
  const lines = [];   // { line, patch, region id, regionTris }
  for (let ri = 0; ri < result.regions.length; ri++) {
    const rFaces = result.regions[ri].faces;
    const regionTris = new Float64Array(rFaces.length * 9);
    const regionPts = [];
    let regionArea = 0;
    for (let k = 0; k < rFaces.length; k++) {
      regionArea += topo.area[rFaces[k]];
      let gx = 0, gy = 0, gz = 0;
      for (let i = 0; i < 3; i++) {
        surface.seat(pos, rFaces[k] * 9 + i * 3, rot, off, v);
        regionTris[k * 9 + i * 3] = v[0]; regionTris[k * 9 + i * 3 + 1] = v[1]; regionTris[k * 9 + i * 3 + 2] = v[2];
        regionPts.push([v[0], v[1], v[2]]);
        gx += v[0]; gy += v[1]; gz += v[2];
      }
      regionPts.push([gx / 3, gy / 3, gz / 3]);
    }
    const id = `r${ri}`;
    trisTable[id] = Array.from(regionTris);
    const tube = tracks.tubeLine(topo, rFaces, rot, regionPts, regionTris, step);
    const small = regionArea < PROP.tubeMinArea;
    const split = small || !tube?.length ? tracks.splitRegion(topo, rFaces, rot) : null;
    const patches = [];
    if (tube && tube.length) patches.push({ lines: tube, smallTube: small ? split : null });
    else for (const p of split) if (p.area >= MIN_REGION_AREA) patches.push(p);
    for (const patch of patches) {
      let ls = patch.lines;
      if (!ls) {
        const pts = [], patchTris = new Float64Array(patch.faces.length * 9);
        for (let k = 0; k < patch.faces.length; k++) {
          const f = patch.faces[k];
          let gx = 0, gy = 0, gz = 0;
          for (let i = 0; i < 3; i++) {
            surface.seat(pos, f * 9 + i * 3, rot, off, v);
            pts.push([v[0], v[1], v[2]]);
            patchTris[k * 9 + i * 3] = v[0]; patchTris[k * 9 + i * 3 + 1] = v[1]; patchTris[k * 9 + i * 3 + 2] = v[2];
            gx += v[0]; gy += v[1]; gz += v[2];
          }
          pts.push([gx / 3, gy / 3, gz / 3]);
        }
        ls = tracks.patchTracks(pts, patchTris, step, { topo, rot, offset: off }, rowSpan, patch.axis ?? null);
      }
      for (const l of ls) lines.push({ line: copyLine(l), patch, id, regionTris });
    }
  }
  // spread the kept lines over the scene (every k-th)
  const kept = [];
  const stride = Math.max(1, Math.ceil(lines.length / MAX_LINES));
  for (let i = 0; i < lines.length && kept.length < MAX_LINES; i += stride) kept.push(lines[i]);

  for (let ci = 0; ci < CFGS.length; ci++) {
    const cfg = CFGS[ci];
    applyCfg(cfg, partTris);
    curCfg = ci; curLayer = cfg.layerHeight;
    // the build's out, with an earlier wall's triangles (one tagged) in front
    const out = [[5, 5, 0], [6, 5, 0], [5, 6, 0], [7, 5, 0], [8, 5, 0], [7, 6, 0]];
    crest.markInterface(out, 3);
    for (const L of kept) platePath(copyLine(L.line), L.patch, L.id, L.regionTris, partTris, out, cfg.plateOnly, cfg.tines);
    if (ci === 0 || ci === 3 || ci === 4) for (const L of kept) drawCalls(L.line, partTris, L.id, L.regionTris);
    resetAll();
  }

  const name = `${model}-${rotStr.replace(/,/g, '_')}`;
  writeFileSync(path.join(outDir, `${name}.json`), JSON.stringify({
    scene: name, model, rot: rotStr, pos: Array.from(pos), offset: off, cfgs: CFGS, PROP: PROP0,
    nLines: lines.length, tris: trisTable, cases }, replacer));
  const cnt = {};
  for (const c of cases) cnt[c.fn] = (cnt[c.fn] ?? 0) + 1;
  const okS = cases.filter((c) => c.fn === 'sweep' && c.ret).length;
  const sq = cases.filter((c) => c.fn === 'buildSquatBed').reduce((s, c) => s + c.placed.length, 0);
  console.log(name, `lines ${lines.length} (kept ${kept.length})`, JSON.stringify(cnt), `sweeps ok ${okS}, squat walls ${sq}`);
  for (const k of Object.keys(cnt)) counts[k] = (counts[k] ?? 0) + cnt[k];
}

// ---- synthetic: the pure helpers over grids, and degenerate sweeps -----------------
{
  cases = []; curCfg = -1;
  const synCfgs = [
    { prop: {} }, { prop: { gap: 0.3 } }, { prop: { footMin: 2.0, footMax: 2.5, footRatio: 0.2 } },
    { prop: { maxUnsupportedSpan: 3, tipH: 0.8, baseH: 1.0 } }, { prop: { th: 1.2, tip: 0.4, footGap: 0 } },
    { prop: { iface: 'all', ifaceLayers: 2 } }, { prop: { iface: 'all', footGap: 0, gap: 0.3 } },
  ];
  const hs = [-1, 0, 0.3, 1.5, 7, 13.3, 20, 25, 40, 1e3, NaN, Infinity];
  const synLines = [
    [], [[0, 0, 5]], [[0, 0, 5], [0, 0, 5]], [[0, 0, 5], [1, 0, 5]], [[0, 0, 5], [1, 0, 5], [1, 0, 6], [2, 0, 6]],
    Array.from({ length: 9 }, (_, k) => [k, 0.1 * k, 0.5 + 0.4 * k]),
    Array.from({ length: 9 }, (_, k) => [3 - 0.5 * k, 2 + 0.8 * k, 4.5 - 0.5 * k]),
    Array.from({ length: 6 }, (_, k) => [k * 0.9, -k * 0.3, 1.2]),
    Array.from({ length: 6 }, (_, k) => [k, 0, 0.5 + (k % 2) * 3]),
    Array.from({ length: 12 }, (_, k) => [10 * Math.cos(k / 4), 10 * Math.sin(k / 4), 2 + Math.abs(k - 6)]),
  ];
  for (let ci = 0; ci < synCfgs.length; ci++) {
    curCfg = ci;
    Object.assign(PROP, PROP0, synCfgs[ci].prop);
    crest.crestPart(null);
    cases.push({ fn: 'footFor', cfg: ci, hs, out: hs.map((h) => SW.footFor(h)) });
    const ph = [];
    for (const top of [0.4, 1, 1.6, 2.5, 7.25, 40, NaN]) for (const z of [-1, 0, 0.3, 0.59, 0.6, 0.65, 0.7, 1.0, 1.55, 2.4, 5, 6, 7.1, 7.25, 39, 41, NaN]) ph.push([z, top, SW.profileHalf(z, top)]);
    cases.push({ fn: 'profileHalf', cfg: ci, cases: ph });
    const wl = [];
    for (const d of [0, 0.05, 0.13, 0.135, 0.1350001, 0.2, 0.2025, 0.204, 0.205, 0.21, 0.3, NaN]) for (const cu of [0, 0.5, 0.7, 0.7000001, 1, NaN]) wl.push([d, cu, CL.welds({ d, cosUp: cu })]);
    cases.push({ fn: 'welds', cfg: ci, cases: wl });
    for (const line of synLines) {
      const scratch = () => [[0, 0, 0], [1, 0, 0], [0, 1, 0]];
      if (line.length) {
        for (const minH of [undefined, 0.3]) recSweep(copyLine(line), 0, scratch(), minH, 'syn');
        recSweep(copyLine(line), 0.4, scratch(), 0.2, 'syn');
        recSquat(copyLine(line), 0, scratch(), 'syn');
        const fl = line.map((p, k) => (k % 3 === 0 ? [p[0], p[1], 0] : k % 3 === 1 ? [p[0], p[1], 0.6]
                                                                 : [p[0], p[1], 0.9, 0.7, 1.4]));
        for (const minH of [undefined, 0.6]) recBetween(copyLine(line), fl, scratch(), minH, 'syn');
        recBetween(copyLine(line), line.map((p) => [p[0], p[1], 0.3, 2.5, -0.1]), scratch(), 0.6, 'syn');
      }
      recIFS(copyLine(line));
      recBody(line);
    }
    // insertFloorStations: ends under the floor
    const fz = PROP.minHeightSquat + PROP.gap + 0.02;
    for (const zs of [[fz - 0.5, fz + 0.3, fz + 1, fz + 0.2, fz - 0.1], [fz - 1, fz - 0.5, fz + 2, fz + 2], [fz - 1, fz - 1],
                      [fz, fz + 0.0005, fz + 0.5], [fz + 1, fz + 1, fz - 0.3, fz - 0.3, fz - 0.2], [fz - 0.2], [fz + 0.5, fz - 0.4]]) {
      recIFS(zs.map((z, k) => [k * 1.1, k * 0.2, z]));
    }
  }
  const masks = [[], [false], [true], [true, true, false, true, true, true], [false, true, true, false, true, true],
                 [true, false, true, false], [false, false, false], [true, true, true, true]];
  for (const m of masks) recRun(m);
  for (const t of masks) for (const l of masks) if (l.length === t.length || l.length === 0) recTails(t, l);
  recTails([false, true, true, false, false], [true, true, false, true, true]);
  recTails([false, false, true, false, false], [true, true, true, true, true]);
  recTails([false, true, false], [true, true, true, true, true]);   // low longer than tall (UPSTREAM)
  Object.assign(PROP, PROP0);
  writeFileSync(path.join(outDir, `synthetic.json`), JSON.stringify({ scene: 'synthetic', cfgs: synCfgs, PROP: PROP0, cases }, replacer));
  const cnt = {};
  for (const c of cases) cnt[c.fn] = (cnt[c.fn] ?? 0) + 1;
  console.log('synthetic', JSON.stringify(cnt));
}
console.log(`attached.js copy verified against upstream on ${nVerified} lines -> ${outDir}`);
