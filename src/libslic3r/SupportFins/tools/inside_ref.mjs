#!/usr/bin/env node
// Reference oracle for Engine/Inside.* and Engine/Kiss.* (web/inside.js, web/kiss.js).
// Poses each stress model like tools/ref.mjs, seats + analyzes it exactly as
// plugins/shared/engine/fins_entry.js computeFins does (seatSoup, buildTopology,
// analyze(..., IDENTITY3)), then evaluates every export of inside.js / kiss.js on
// deterministic inputs and dumps inputs + outputs as JSON for tests/test_inside.cpp.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/inside_ref.mjs [outDir]
//
// outDir defaults to refs_inside/ next to tools/. Two frames per model:
//   "seated"  rot = IDENTITY3, offset = analyze().offset  (what computeFins hands buildFins)
//   "rotated" the same topo under an arbitrary non-axis rotation, offset = analyze(topo, 45, rot).offset
// so the un-rotate / un-seat paths are exercised too. Every call is replayed in the
// same order by the C++ test, because solidClearance advances topo._scGen.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR;
if (!SF) { console.error('set SUPPORT_FINS_DIR'); process.exit(2); }
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_inside');
mkdirSync(outDir, { recursive: true });

const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);
const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3 } = await imp('web/overhangs.js');
const { insidePart, nearestPart, solidClearance } = await imp('web/inside.js');
const { kissEnds } = await imp('web/kiss.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

const MODELS = [
  ['cube', '40,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'], ['sphere', '0,0,0'],
  ['tube', '40,0,0'], ['staircase', '40,0,0'], ['plus', '40,0,0'],
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
  return quatToRot(q);
}
// ---------------------------------------------------------------------------------------

function quatToRot([x, y, z, w]) {
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}
// An arbitrary, non-axis-aligned orientation for the "rotated" frame.
function oddRot() {
  const ax = [0.3, -0.5, 0.8], n = Math.hypot(...ax), h = 0.7 / 2, s = Math.sin(h);
  return quatToRot([ax[0] / n * s, ax[1] / n * s, ax[2] / n * s, Math.cos(h)]);
}

let seed = 20261012;
const rnd = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scl = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (a) => { const l = Math.hypot(a[0], a[1], a[2]); return l > 0 ? scl(a, 1 / l) : [1, 0, 0]; };

// A box in an arbitrary frame (c + u*[-hu,hu] + v*[-hv,hv] + n*[n0,n1]), 12 triangles.
function box(c, u, v, n, hu, hv, n0, n1) {
  const P = (a, b, k) => add(add(add(c, scl(u, a)), scl(v, b)), scl(n, k));
  const p = [P(-hu, -hv, n0), P(hu, -hv, n0), P(hu, hv, n0), P(-hu, hv, n0),
             P(-hu, -hv, n1), P(hu, -hv, n1), P(hu, hv, n1), P(-hu, hv, n1)];
  const quads = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
  const out = [];
  for (const [a, b, cc, d] of quads) out.push(p[a], p[b], p[cc], p[a], p[cc], p[d]);
  return out;
}

// JSON can't hold NaN/Infinity or -0: encode them as strings, the test decodes.
const num = (v) => (Object.is(v, -0) ? '-0' : Number.isFinite(v) ? v : String(v));
const nums = (a) => a.map(num);
const numDeep = (v) => (Array.isArray(v) ? v.map(numDeep) : typeof v === 'number' ? num(v) : v);
const numObj = (o) => Object.fromEntries(Object.entries(o).map(([k, v]) => [k, num(v)]));

function runFrame(name, topo, rot, offset, kept) {
  const { pos, nFaces, nrm } = topo;
  const seatP = (x, y, z) => [rot[0] * x + rot[3] * y + rot[6] * z + offset.x,
                              rot[1] * x + rot[4] * y + rot[7] * z + offset.y,
                              rot[2] * x + rot[5] * y + rot[8] * z + offset.z];
  const rotN = (x, y, z) => [rot[0] * x + rot[3] * y + rot[6] * z,
                             rot[1] * x + rot[4] * y + rot[7] * z,
                             rot[2] * x + rot[5] * y + rot[8] * z];
  const V = (f, i) => seatP(pos[f * 9 + i * 3], pos[f * 9 + i * 3 + 1], pos[f * 9 + i * 3 + 2]);
  const centroid = (f) => { const a = V(f, 0), b = V(f, 1), c = V(f, 2); return [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3]; };
  const normal = (f) => rotN(nrm[f * 3], nrm[f * 3 + 1], nrm[f * 3 + 2]);

  let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let f = 0; f < nFaces; f++) for (let i = 0; i < 3; i++) {
    const p = V(f, i);
    for (let k = 0; k < 3; k++) { if (p[k] < lo[k]) lo[k] = p[k]; if (p[k] > hi[k]) hi[k] = p[k]; }
  }
  lo = lo.map((v) => v - 2); hi = hi.map((v) => v + 2);

  // sample faces deterministically
  const sample = (cap) => {
    const out = [], step = Math.max(1, Math.floor(nFaces / cap));
    for (let f = 0; f < nFaces && out.length < cap; f += step) out.push(f);
    return out;
  };

  // ---- query points
  const pts = [];
  const N = 15;
  for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) for (let k = 0; k < N; k++) {
    pts.push([lo[0] + (hi[0] - lo[0]) * i / (N - 1), lo[1] + (hi[1] - lo[1]) * j / (N - 1), lo[2] + (hi[2] - lo[2]) * k / (N - 1)]);
  }
  const faces = sample(150);
  for (const f of faces) {
    const a = V(f, 0), b = V(f, 1), c = V(f, 2);
    pts.push(a, b, c, scl(add(a, b), 0.5), scl(add(b, c), 0.5), scl(add(c, a), 0.5), centroid(f));
  }
  for (const f of faces) {
    const c = centroid(f), n = normal(f);
    for (const d of [0.1, 0.2, 0.45, 0.5, -0.1, -0.2, -0.45, -0.5]) pts.push(add(c, scl(n, d)));
  }

  const inside = pts.map((p) => insidePart(topo, rot, offset, p[0], p[1], p[2]));
  const nearest = [];
  for (const rMax of [null, 0.1, 0.25, 1.0]) {
    const res = pts.map((p) => {
      const h = rMax === null ? nearestPart(topo, rot, offset, p[0], p[1], p[2])
                              : nearestPart(topo, rot, offset, p[0], p[1], p[2], rMax);
      return h ? nums([h.d, h.cosUp]) : null;
    });
    nearest.push({ rMax, res });
  }

  // ---- kissEnds, the way prop/tines.js and sway.js call it
  const PROP_OPTS = { half: 0.25, back: 0.3, reach: 0.5 };   // PROP.tineW / 2, tineOverlap, tineReach
  const SWAY_OPTS = { half: 0.25, back: 0.3, reach: 0.3 };   // SWAY.tineW / 2, tineOverlap, tineReach
  const kiss = [];
  const kissCall = (x, y, dx, dy, zb, zt, o) => {
    const e = kissEnds(topo, rot, offset, x, y, dx, dy, zb, zt, o);
    kiss.push({ args: [x, y, dx, dy, zb, zt, o.half, o.back, o.reach].map(num),
                out: [e.bot[0], e.bot[1], e.top[0], e.top[1]].map(num) });
  };
  for (const f of sample(60)) {
    const n = normal(f), h = Math.hypot(n[0], n[1]);
    if (h < 0.2) continue;
    const nx = n[0] / h, ny = n[1] / h, c = centroid(f);
    for (const s0 of [-0.3, 0, 0.1, 0.25, 0.45, 0.8]) {
      const x = c[0] + nx * s0, y = c[1] + ny * s0;
      for (const tineH of [0.2, 0.3, 0.12]) {
        // prop/tines.js: top snapped to the layer grid, bottom one layer below
        const tineTop = Math.round(c[2] / tineH) * tineH, tineBot = tineTop - tineH;
        kissCall(x, y, -nx, -ny, tineBot, tineTop, PROP_OPTS);
        // sway.js: bottom snapped, top one layer above
        const bot = Math.round(c[2] / tineH) * tineH, top = bot + tineH;
        kissCall(x, y, -nx, -ny, bot, top, SWAY_OPTS);
      }
    }
  }
  // arbitrary headings from a sparse subset of the grid, and a few odd layers (zt - zb
  // == 2*TRIM_Z makes k infinite: NaN/inf propagation through the clamp)
  for (let i = 0; i < N * N * N; i += 23) {
    const p = pts[i], a = rnd() * 2 * Math.PI;
    kissCall(p[0], p[1], Math.cos(a), Math.sin(a), p[2] - 0.1, p[2] + 0.1, rnd() < 0.5 ? PROP_OPTS : SWAY_OPTS);
  }
  for (const f of sample(8)) {
    const c = centroid(f), n = normal(f), h = Math.hypot(n[0], n[1]) || 1;
    kissCall(c[0], c[1], -n[0] / h, -n[1] / h, c[2] - 0.01, c[2] + 0.01, PROP_OPTS);
    kissCall(c[0], c[1], -n[0] / h, -n[1] / h, 0, 0.02, PROP_OPTS);   // k = 0.01 / 0 exactly
    kissCall(c[0], c[1], -n[0] / h, -n[1] / h, c[2], c[2] + 0.02, SWAY_OPTS);
    kissCall(c[0], c[1], -n[0] / h, -n[1] / h, c[2] - 0.5, c[2] + 0.5, { half: 1.0, back: 0.0, reach: 1.25 });
  }

  // ---- solidClearance, on solids like prop.js / prop/squat.js hand it
  const solids = [];
  const scCall = (verts, rMax, setGen) => {
    if (setGen !== undefined) topo._scGen = setGen;
    const h = rMax === null ? solidClearance(topo, rot, offset, verts)
                            : solidClearance(topo, rot, offset, verts, rMax);
    solids.push({ verts: numDeep(verts), rMax, setGen: setGen ?? null,
                  res: h ? nums([h.d, h.cosUp, h.x, h.y, h.z]) : null, genAfter: topo._scGen });
  };
  // slabs facing a part face at a set distance (negative: buried, pierces)
  for (const f of sample(40)) {
    const n = normal(f), c = centroid(f);
    const ref = Math.abs(n[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const u = unit(cross(n, ref)), v = cross(n, u);
    for (const d of [-0.3, 0, 0.05, 0.1, 0.2, 0.3, 0.44, 0.6]) {
      scCall(box(c, u, v, n, 0.6, 0.6, d, d + 0.8), 0.25);
      scCall(box(c, u, v, n, 0.6, 0.6, d, d + 0.8), 0.45);
    }
    // a slab tilted 30 degrees off the face, its near edge at 0.15
    const t = add(scl(n, Math.cos(Math.PI / 6)), scl(u, Math.sin(Math.PI / 6)));
    scCall(box(add(c, scl(n, 0.15)), cross(t, v), v, t, 0.5, 0.5, 0, 0.6), null);
  }
  // prop-style walls: plate to `gap` (0.2) under an overhang face, 1 mm thick, 3 mm long
  const keptFaces = [];
  for (let f = 0; f < nFaces; f++) if (kept[f]) keptFaces.push(f);
  const step = Math.max(1, Math.floor(keptFaces.length / 25));
  for (let i = 0; i < keptFaces.length; i += step) {
    const f = keptFaces[i], c = centroid(f), n = normal(f);
    if (c[2] < 0.6) continue;
    const h = Math.hypot(n[0], n[1]);
    const run = h > 1e-6 ? [-n[1] / h, n[0] / h, 0] : [1, 0, 0];
    const side = [-run[1], run[0], 0];
    const wall = box([c[0], c[1], 0], run, side, [0, 0, 1], 1.5, 0.5, 0, c[2] - 0.2);
    scCall(wall, 0.25);
    scCall(wall, 0.45);
  }
  // degenerate triangles: a point and a segment just off a face, plus a sliver
  for (const f of sample(10)) {
    const c = centroid(f), n = normal(f), a = V(f, 0), b = V(f, 1);
    const p = add(c, scl(n, 0.1));
    const s0 = add(a, scl(n, 0.12)), s1 = add(b, scl(n, 0.12));
    scCall([p, p, p], 0.45);
    scCall([s0, s1, s1], 0.45);
    scCall([s0, s1, scl(add(s0, s1), 0.5)], 0.45);
  }
  // far away (null), and the whole part in a box 1 mm off it (nothing within 0.45; all within 2)
  scCall(box([hi[0] + 50, hi[1] + 50, hi[2] + 50], [1, 0, 0], [0, 1, 0], [0, 0, 1], 1, 1, 0, 1), 0.45);
  const mid = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + 1];
  const hu = (hi[0] - lo[0]) / 2 - 1, hv = (hi[1] - lo[1]) / 2 - 1, hz = hi[2] - lo[2] - 2;
  const env = box(mid, [1, 0, 0], [0, 1, 0], [0, 0, 1], hu, hv, -1, hz);
  scCall(env, 0.45);
  scCall(env, 2.0);
  // the generation stamp's Int32 guard: a reset, a run straddling 2^31, a negative restart
  const f0 = sample(1)[0], c0 = centroid(f0), n0 = normal(f0);
  const ref0 = Math.abs(n0[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const u0 = unit(cross(n0, ref0)), v0 = cross(n0, u0);
  const near = [...box(c0, u0, v0, n0, 0.6, 0.6, 0.1, 0.9), ...box(c0, u0, v0, n0, 0.3, 0.3, 0.2, 0.5)];
  scCall(near, 0.45, 2000000005);
  scCall(near, 0.45, 2147483640);
  scCall(near, 0.45);
  scCall(near, 0.45, 2147483650);
  scCall(near, 0.45);

  return { name, rot: nums(rot), offset: numObj(offset), points: numDeep(pts), inside, nearest, kiss, solids };
}

for (const [model, rotStr] of MODELS) {
  const raw = readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
  const r = sitePose(rotStr.split(',').map(Number));
  const posed = new Float64Array(raw.length);
  for (let i = 0; i < raw.length; i += 3) {
    const x = raw[i], y = raw[i + 1], z = raw[i + 2];
    posed[i] = r[0] * x + r[3] * y + r[6] * z;
    posed[i + 1] = r[1] * x + r[4] * y + r[7] * z;
    posed[i + 2] = r[2] * x + r[5] * y + r[8] * z;
  }
  // exactly as computeFins
  const { pos } = seatSoup(posed);
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);
  const seated = { x: result.offset.x, y: result.offset.y, z: result.offset.z };
  const keptA = Uint8Array.from(result.kept);
  const frames = [runFrame('seated', topo, IDENTITY3, seated, keptA)];

  const rotB = oddRot();
  const resB = analyze(topo, DEFAULT_THRESHOLD, rotB);
  const offB = { x: resB.offset.x, y: resB.offset.y, z: resB.offset.z };
  frames.push(runFrame('rotated', topo, rotB, offB, Uint8Array.from(resB.kept)));

  const out = { model, rot: rotStr, soup: nums(Array.from(posed)), frames };
  const file = path.join(outDir, `${model}.json`);
  writeFileSync(file, JSON.stringify(out));
  const counts = frames.map((f) => `${f.name}: ${f.points.length} pts (${f.inside.filter(Boolean).length} in), ` +
    `${f.kiss.length} kiss, ${f.solids.length} solids (${f.solids.filter((s) => s.res).length} hit)`).join('; ');
  console.log(`${model} rot=${rotStr} faces=${topo.nFaces}: ${counts}`);
}
