#!/usr/bin/env node
// Reference oracle for Engine/Fins/Pad.* (web/fins/pad.js) and Engine/Fins/Coverage.*
// (web/fins/coverage.js). Poses each model like tools/ref.mjs, seats + analyzes it
// exactly as plugins/shared/engine/fins_entry.js computeFins does (seatSoup,
// buildTopology, analyze(..., IDENTITY3)), then:
//
//   buildPad  -- under each PAD state (applied through fins.js applyTunables, the way a
//                build request carries it): the site default (PLA, auto), PLA and PETG
//                (padH 0.3, grab -0.10) under each style auto/light/sure/custom, and a
//                Custom with a gap and a negative grip. The contact points are
//                bedContact's, the part is seatedPartTris', as fins.js buildFinsCore
//                calls it. Dumps the contact points, the returned record and the
//                emitted triangles (flat, emission order; float64 in <scene>.bin).
//                fins.js only builds a pad when bedArea < FIN.padMinArea (`gate`); the
//                pad is built here regardless, so buildPad itself is pinned everywhere.
//   classify  -- classify(topo, res, rot), seated (IDENTITY3) and under an arbitrary
//                rotation (analyze(topo, 45, rot)).
//   reachOf   -- `tops` built as fill.js builds them (every Auto fin's `line` points,
//                from the upstream buildFins), several R, queried at the overhang faces'
//                centroids + corners and around each top; then add() and re-query.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/pad_ref.mjs [outDir]
//
// outDir defaults to refs_pad/ next to tools/.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR;
if (!SF) { console.error('set SUPPORT_FINS_DIR'); process.exit(2); }
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_pad');
mkdirSync(outDir, { recursive: true });

const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);
const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3 } = await imp('web/overhangs.js');
const { buildPad, PAD } = await imp('web/fins/pad.js');
const { classify, reachOf } = await imp('web/fins/coverage.js');
const { bedContact, seatedPartTris } = await imp('web/fins/seating.js');
const { FIN } = await imp('web/fins/config.js');
const { applyTunables, buildFins } = await imp('web/fins.js');
const { fillReach } = await imp('web/fins/fill.js');
const { MATERIAL } = await imp('web/materials.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['lbracket', '35,0,0'], ['sphere', '0,0,0'], ['torus', '30,0,0'],
  ['cone', '0,0,0'], ['cone', '180,0,0'], ['staircase', '40,0,0'], ['plate', '40,0,0'], ['pyramid', '40,0,0'], ['cube', '0,0,0'],
  ['bore4', null], ['bore10', null],
];

// --- copied from tools/ref.mjs -------------------------------------------------------
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

// tests/_scene.js sitePose: snapped 5-degree turns about world X, then Y, then Z, premultiplied.
function sitePose(deg) {
  const snap = (d) => Math.round(d / 5) * 5 * Math.PI / 180;
  const axis = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let q = [0, 0, 0, 1]; // x,y,z,w
  const mul = (a, b) => [ // a*b (three.js Quaternion.multiplyQuaternions)
    a[0] * b[3] + a[3] * b[0] + a[1] * b[2] - a[2] * b[1],
    a[1] * b[3] + a[3] * b[1] + a[2] * b[0] - a[0] * b[2],
    a[2] * b[3] + a[3] * b[2] + a[0] * b[1] - a[1] * b[0],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
  deg.forEach((d, i) => {
    if (!d) return;
    const h = snap(d) / 2, s = Math.sin(h);
    const r = [axis[i][0] * s, axis[i][1] * s, axis[i][2] * s, Math.cos(h)];
    q = mul(r, q); // premultiply
  });
  return quatToRot(q);
}
// ---------------------------------------------------------------------------------------

function quatToRot([x, y, z, w]) {
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}
function oddRot() {
  const ax = [0.3, -0.5, 0.8], n = Math.hypot(...ax), h = 0.7 / 2, s = Math.sin(h);
  return quatToRot([ax[0] / n * s, ax[1] / n * s, ax[2] / n * s, Math.cos(h)]);
}

/**
 * A block with a round horizontal bore along Y (radius r, n segments), standing on the
 * plate: the tiny-hole case classify exists for (no stress model has one). Each face of
 * the block's front/back is a square annulus matched ray-for-ray to the bore circle, and
 * the four side faces are strips through the same square-boundary points, so the mesh
 * is watertight; every triangle is wound outward (into the bore for the bore's wall).
 */
function boreBlock(r, W, L, n) {
  const sq = [], ci = [];
  for (let k = 0; k < n; k++) {
    const a = (2 * Math.PI * k) / n, c = Math.cos(a), s = Math.sin(a);
    const m = Math.max(Math.abs(c), Math.abs(s));
    sq.push([W * c / m, W * s / m]);   // ray to the square boundary (x, z)
    ci.push([r * c, r * s]);
  }
  const tris = [];
  const P = (xz, y) => [xz[0], y, xz[1] + W];
  const put = (a, b, c, want) => {   // wind (a, b, c) so its normal points along `want`
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    const nn = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    if (nn[0] * want[0] + nn[1] * want[1] + nn[2] * want[2] < 0) tris.push(a, c, b); else tris.push(a, b, c);
  };
  for (let k = 0; k < n; k++) {
    const k1 = (k + 1) % n;
    for (const y of [-L, L]) {         // front/back annuli
      const want = [0, Math.sign(y), 0];
      put(P(sq[k], y), P(sq[k1], y), P(ci[k1], y), want);
      put(P(sq[k], y), P(ci[k1], y), P(ci[k], y), want);
    }
    // the bore's wall, facing its axis
    const mid = [(ci[k][0] + ci[k1][0]) / 2, (ci[k][1] + ci[k1][1]) / 2];
    const inw = [-mid[0], 0, -mid[1]];
    put(P(ci[k], -L), P(ci[k1], -L), P(ci[k1], L), inw);
    put(P(ci[k], -L), P(ci[k1], L), P(ci[k], L), inw);
    // the block's outer side, facing out
    const smid = [(sq[k][0] + sq[k1][0]) / 2, (sq[k][1] + sq[k1][1]) / 2];
    const out = Math.abs(smid[0]) > Math.abs(smid[1]) ? [Math.sign(smid[0]), 0, 0] : [0, 0, Math.sign(smid[1])];
    put(P(sq[k], -L), P(sq[k1], -L), P(sq[k1], L), out);
    put(P(sq[k], -L), P(sq[k1], L), P(sq[k], L), out);
  }
  return Float64Array.from(tris.flat());
}

const num = (v) => (Object.is(v, -0) ? '-0' : Number.isFinite(v) ? v : String(v));
const numObj = (o) => Object.fromEntries(Object.entries(o).map(([k, v]) => [k, typeof v === 'number' ? num(v) : v]));

// PAD states, applied through applyTunables exactly as a build request carries them
// (seat.js clearances: padH, padGrab, propGap, padStyle; plus padCustom as the site sends it).
const DEF_CUSTOM = { h: 0.5, gap: 0.0, grip: 0.05, margin: 4.0 };
const CONFIGS = [];
for (const mat of ['pla', 'petg']) {
  for (const style of ['auto', 'light', 'sure', 'custom']) {
    CONFIGS.push({ name: `${mat}-${style}`, mat, style, custom: DEF_CUSTOM, layerH: 0.2 });
  }
}
CONFIGS.push({ name: 'pla-custom-gap', mat: 'pla', style: 'custom', custom: { h: 0.6, gap: 0.15, grip: -0.05, margin: 3.0 }, layerH: 0.2 });
CONFIGS.push({ name: 'petg-custom-thin', mat: 'petg', style: 'custom', custom: { h: 0.04, gap: 0.3, grip: 0.1, margin: 6.5 }, layerH: 0.28 });
// layer heights: a slicer's finer/coarser layer, and the <= 0.05 / undefined fallbacks to FIN.tineH
const LAYER_EXTRA = [
  { name: 'pla-auto-l012', mat: 'pla', style: 'auto', custom: DEF_CUSTOM, layerH: 0.12 },
  { name: 'pla-auto-l028', mat: 'pla', style: 'auto', custom: DEF_CUSTOM, layerH: 0.28 },
  { name: 'pla-sure-l004', mat: 'pla', style: 'sure', custom: DEF_CUSTOM, layerH: 0.04 },
  { name: 'petg-auto-ldef', mat: 'petg', style: 'auto', custom: DEF_CUSTOM, layerH: null },
];

function applyConfig(c) {
  const m = MATERIAL[c.mat];
  applyTunables({ padH: m.padH, padGrab: m.padGrab, propGap: m.propGap, padStyle: c.style, padCustom: { ...c.custom }, cutout: 'none' });
}

function padState() {
  return { grab: PAD.grab, style: PAD.style, custom: { ...PAD.custom }, cell: PAD.cell, brimGap: PAD.brimGap,
           minGripOutline: PAD.minGripOutline, brimCell: PAD.brimCell, padH: FIN.padH, tineH: FIN.tineH };
}

function runPad(scene, contactPts, partTris, configs, bins) {
  const pads = [];
  for (const c of configs) {
    applyConfig(c);
    const out = [];
    const pad = c.layerH === null ? buildPad(contactPts, partTris, out) : buildPad(contactPts, partTris, out, c.layerH);
    const flat = new Float64Array(out.length * 3);
    out.forEach((p, i) => { flat[i * 3] = p[0]; flat[i * 3 + 1] = p[1]; flat[i * 3 + 2] = p[2]; });
    const at = bins.reduce((s, b) => s + b.length, 0);
    bins.push(flat);
    pads.push({ config: c.name, mat: c.mat, style: c.style, custom: c.custom, layerH: c.layerH, state: padState(),
                pad: pad ? numObj(pad) : null, nVerts: out.length, binAt: at });
  }
  return pads;
}

function runClassify(topo, res, rot) {
  const cls = classify(topo, res, rot);
  return { rot: rot.map(num), offset: numObj({ x: res.offset.x, y: res.offset.y, z: res.offset.z }),
           over: Array.from(res.over).join(''), cls: Array.from(cls).join('') };
}

function runReach(topo, res, tops) {
  const { pos, nFaces } = topo, off = res.offset;
  const V = (f, i) => {
    const o = f * 9 + i * 3;
    return [pos[o] + off.x, pos[o + 1] + off.y, pos[o + 2] + off.z];   // IDENTITY3
  };
  const qs = [];
  for (let f = 0; f < nFaces; f++) {
    if (!res.over[f]) continue;
    const a = V(f, 0), b = V(f, 1), c = V(f, 2);
    qs.push([(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3], a, b, c);
  }
  const Rs = [fillReach(0.5), fillReach(1), fillReach(0.75), 0.5];
  // around the tops: on the R boundary in plan, and at the 0..1.5 (-0.05) z limits
  const step = Math.max(1, Math.floor(tops.length / 60));
  for (let i = 0; i < tops.length; i += step) {
    const p = tops[i];
    for (const dz of [-0.06, -0.05, 0, 0.7, 1.5, 1.51]) {
      for (const [k, ang] of [[0, 0], [0.5, 1], [0.99, 2], [1, 0], [1, 3], [1.01, 4]]) {
        for (const R of [Rs[0], Rs[3]]) {
          const a = ang * 0.7;
          qs.push([p[0] + R * k * Math.cos(a), p[1] + R * k * Math.sin(a), p[2] + dz]);
        }
      }
    }
  }
  const runs = [];
  for (const R of Rs) {
    const half = tops.filter((_, i) => i % 2 === 0), rest = tops.filter((_, i) => i % 2 === 1);
    const near = reachOf(half, R);
    const before = qs.map((q) => (near(q[0], q[1], q[2]) ? 1 : 0)).join('');
    for (const p of rest) near.add(p);
    const after = qs.map((q) => (near(q[0], q[1], q[2]) ? 1 : 0)).join('');
    const full = reachOf(tops, R);
    const all = qs.map((q) => (full(q[0], q[1], q[2]) ? 1 : 0)).join('');
    runs.push({ R: num(R), before, after, all });
  }
  return { tops: tops.map((p) => p.map(num)), queries: qs.map((p) => p.map(num)), runs };
}

for (const [model, rotStr] of SCENES) {
  let posed;
  if (rotStr === null) {
    posed = model === 'bore4' ? boreBlock(2, 8, 5, 32) : boreBlock(5, 10, 6, 32);
  } else {
    const raw = readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
    const r = sitePose(rotStr.split(',').map(Number));
    posed = new Float64Array(raw.length);
    for (let i = 0; i < raw.length; i += 3) {
      const x = raw[i], y = raw[i + 1], z = raw[i + 2];
      posed[i] = r[0] * x + r[3] * y + r[6] * z;
      posed[i + 1] = r[1] * x + r[4] * y + r[7] * z;
      posed[i + 2] = r[2] * x + r[5] * y + r[8] * z;
    }
  }
  const scene = rotStr === null ? model : `${model}-x${rotStr.split(',')[0]}`;
  // exactly as computeFins
  const { pos } = seatSoup(posed);
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);
  // `over` is the topology's own buffer: read everything off this analyze now
  const classifySeated = runClassify(topo, result, IDENTITY3);
  const overA = Uint8Array.from(result.over);
  const resA = { ...result, over: overA };

  // ---- buildPad, as fins.js buildFinsCore calls it
  const contact = bedContact(topo, resA, IDENTITY3);
  const partTris = seatedPartTris(topo, IDENTITY3, resA.offset);
  const gate = resA.bedArea < FIN.padMinArea;
  const bins = [];
  // a face-seated part (no pad in fins.js: `gate` false) gets a few configs only -- its
  // brim-style pad covers the whole footprint at 0.1 mm and runs to ~750k vertices
  const few = CONFIGS.filter((c) => c.name === 'pla-auto' || c.name === 'petg-sure');
  const configs = scene === 'cube-x40' ? [...CONFIGS, ...LAYER_EXTRA] : gate ? CONFIGS : few;
  const pads = runPad(scene, contact.pts, partTris, configs, bins);
  // fewer than 3 contact points: null
  if (scene === 'cube-x40') {
    applyConfig(CONFIGS[0]);
    const out = [];
    const r2 = buildPad(contact.pts.slice(0, 2), partTris, out);
    pads.push({ config: 'two-points', mat: 'pla', style: 'auto', custom: DEF_CUSTOM, layerH: null, state: padState(), pad: r2,
                nVerts: out.length, binAt: bins.reduce((s, b) => s + b.length, 0), contactCount: 2 });
  }

  // ---- reachOf: tops as fill.js builds them (Auto's fins' lines). On a fresh topology,
  // so the build can't touch the one classify read.
  applyConfig(CONFIGS[0]);
  const topo2 = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const res2 = analyze(topo2, DEFAULT_THRESHOLD, IDENTITY3);
  const built = buildFins(topo2, res2, IDENTITY3, { mode: 'auto', layerHeight: 0.2, tunables: undefined });
  const tops = [];
  for (const w of built.fins ?? []) for (const p of w.line ?? []) tops.push(p);
  // a part with no Auto walls still gets queried: fall back to points under its overhang
  if (!tops.length) {
    for (let f = 0; f < topo.nFaces && tops.length < 40; f++) if (overA[f]) {
      const o = f * 9;
      tops.push([pos[o] + resA.offset.x, pos[o + 1] + resA.offset.y, pos[o + 2] + resA.offset.z - 0.5]);
    }
  }
  const reach = runReach(topo, resA, tops);

  // ---- classify under an arbitrary rotation
  const rotB = oddRot();
  const resB = analyze(topo, DEFAULT_THRESHOLD, rotB);
  const classifyRotated = runClassify(topo, resB, rotB);

  // restore the site default for whoever runs next
  applyConfig(CONFIGS[0]);

  const all = new Float64Array(bins.reduce((s, b) => s + b.length, 0));
  let o = 0;
  for (const b of bins) { all.set(b, o); o += b.length; }
  writeFileSync(path.join(outDir, `${scene}.bin`), Buffer.from(all.buffer));
  writeFileSync(path.join(outDir, `${scene}.json`), JSON.stringify({
    scene, model, rot: rotStr, soup: Array.from(posed).map(num),
    bedArea: num(resA.bedArea), padMinArea: FIN.padMinArea, gate,
    contact: contact.pts.map((p) => p.map(num)), mx: num(contact.mx), my: num(contact.my),
    pads, classify: [classifySeated, classifyRotated], reach,
  }));
  const cnt = (s) => [0, 1, 2].map((c) => [...s.cls].filter((x, i) => s.over[i] === '1' && +x === c).length);
  console.log(`${scene}: faces=${topo.nFaces} bedArea=${resA.bedArea.toFixed(1)} gate=${gate} contact=${contact.pts.length} ` +
    `pads=[${pads.map((p) => (p.pad ? `${p.config}:${p.pad.style}${p.pad.autoSure ? '(auto)' : ''}/${p.nVerts / 3}` : `${p.config}:null`)).join(' ')}] ` +
    `cls M/H/L seated=${cnt(classifySeated)} rotated=${cnt(classifyRotated)} tops=${tops.length} queries=${reach.queries.length}`);
}
