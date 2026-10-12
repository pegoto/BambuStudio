#!/usr/bin/env node
// Reference oracle for Engine/Planes (web/planes.js). Poses an STL like tools/ref.mjs,
// seats it like plugins/shared/engine/fins_entry.js computeFins (seatSoup, then
// buildTopology + analyze with IDENTITY3), runs findWallPatches(topo, rot,
// result.offset, stats) and dumps every patch plus stats, plus sample answers of the
// other exports (faceAdjacency, patchProbe/patchCovers, patchPoint, tAtZ, zAt).
// Usage:
//   SUPPORT_FINS_DIR=/path/to/support-fins node planes_ref.mjs model.stl "rx,ry,rz" out.json [engineRot]
// `engineRot` (optional "rx,ry,rz") is passed to analyze/findWallPatches as `rot`
// instead of IDENTITY3, to exercise the rotation path. The dump carries the SEATED
// soup, so the C++ test consumes the same input bytes.
//
// The dumps tests/test_planes.cpp reads (refs_planes/ is gitignored), from this dir's parent:
//   M=$SUPPORT_FINS_DIR/prototype/stress/models; mkdir -p refs_planes
//   run() { node tools/planes_ref.mjs $M/$1.stl $2 refs_planes/$3.json $4; }
//   run cube 40,0,0 cube-x40;            run lbracket 35,0,0 lbracket-x35
//   run torus 30,0,0 torus-x30;          run staircase 40,0,0 staircase-x40
//   run sphere 0,0,0 sphere-up;          run cylinder 40,0,0 cylinder-x40
//   run tube 40,0,0 tube-x40;            run ramp 40,0,0 ramp-x40
//   run cone 20,0,0 cone-x20;            run ushape 0,0,0 ushape-up
//   run plus 30,0,0 plus-x30
//   run cube 0,0,0 cube-rot45 45,0,0;    run lbracket 0,0,0 lbracket-rot35-20-15 35,20,15
import { readFileSync, writeFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR;
if (!SF) { console.error('set SUPPORT_FINS_DIR'); process.exit(2); }
const [stlPath, rotStr = '0,0,0', outPath, engineRotStr] = process.argv.slice(2);
if (!stlPath || !outPath) { console.error('usage: planes_ref.mjs model.stl rx,ry,rz out.json [engineRot]'); process.exit(2); }

const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);
const planes = await imp('web/planes.js');
const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3 } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

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
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  // column-major 3x3 like three.js Matrix3.elements
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}

const raw = readSTL(stlPath);
const pose = sitePose(rotStr.split(',').map(Number));
const posed = new Float64Array(raw.length);
for (let i = 0; i < raw.length; i += 3) {
  const x = raw[i], y = raw[i + 1], z = raw[i + 2];
  posed[i] = pose[0] * x + pose[3] * y + pose[6] * z;
  posed[i + 1] = pose[1] * x + pose[4] * y + pose[7] * z;
  posed[i + 2] = pose[2] * x + pose[5] * y + pose[8] * z;
}

// fins_entry.js computeFins: seat, topology, analyze(IDENTITY3)
const { pos } = seatSoup(posed);
const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
const rot = engineRotStr ? sitePose(engineRotStr.split(',').map(Number)) : IDENTITY3;
const result = analyze(topo, DEFAULT_THRESHOLD, rot);
const stats = {};
const patches = planes.findWallPatches(topo, rot, result.offset, stats);

// Sample the other exports on each patch: a grid over (and a margin past) the patch's
// (u, t) box for patchProbe/patchCovers, plus patchPoint / tAtZ / zAt at a few spots.
const N = 7;
const samples = patches.map((p) => {
  const probes = [];
  const du = p.u1 - p.u0, dt = p.t1 - p.t0;
  for (let i = 0; i <= N; i++) for (let j = 0; j <= N; j++) {
    const u = p.u0 - 0.1 * du + (1.2 * du * i) / N;
    const t = p.t0 - 0.1 * dt + (1.2 * dt * j) / N;
    probes.push({ u, t, probe: planes.patchProbe(p, u, t), covers: planes.patchCovers(p, u, t) });
  }
  // the vertices themselves (on the edge of the coverage test)
  for (let i = 0; i < Math.min(p.tris.length, 90); i += 3) {
    const u = p.tris[i], t = p.tris[i + 1];
    probes.push({ u, t, probe: planes.patchProbe(p, u, t), covers: planes.patchCovers(p, u, t) });
  }
  const umid = (p.u0 + p.u1) / 2, tmid = (p.t0 + p.t1) / 2;
  const points = [[0, umid, tmid], [0.35, p.u0, p.t0], [-1.2, p.u1, p.t1], [2.5, umid, p.t0]].map(([w, u, t]) => ({ w, u, t, p: planes.patchPoint(p, w, u, t) }));
  const tz = [[0, p.z0], [0.2, (p.z0 + p.z1) / 2], [-0.5, p.z1]].map(([w, z]) => ({ w, z, t: planes.tAtZ(p, w, z) }));
  const za = [[0, p.t0], [0.2, tmid], [-0.5, p.t1]].map(([w, t]) => ({ w, t, z: planes.zAt(p, w, t) }));
  return { probes, points, tAtZ: tz, zAt: za };
});

const adj = planes.faceAdjacency(topo);

writeFileSync(outPath, JSON.stringify({
  model: path.basename(stlPath), pose: rotStr, engineRot: engineRotStr ?? null,
  rot: Array.from(rot),
  // the two cut constants findWallPatches derives with Math.sin / Math.cos
  leanCut: Math.sin((planes.MAX_LEAN_DEG * Math.PI) / 180) + 1e-6,
  agreeCut: Math.cos((planes.NORMAL_AGREE_DEG * Math.PI) / 180),
  constants: {
    MAX_LEAN_DEG: planes.MAX_LEAN_DEG, NORMAL_AGREE_DEG: planes.NORMAL_AGREE_DEG,
    FLAT_TOL_IN: planes.FLAT_TOL_IN, GROW_SLACK: planes.GROW_SLACK,
    MIN_PATCH_H: planes.MIN_PATCH_H, MIN_PATCH_W: planes.MIN_PATCH_W, MIN_PATCH_AREA: planes.MIN_PATCH_AREA,
  },
  pos: Array.from(pos),
  // the topology's per-face area / normal as the JS computed them (Math.hypot): the
  // C++ test feeds these in so it pins planes.js alone, independent of buildTopology
  topoArea: Array.from(topo.area), topoNrm: Array.from(topo.nrm),
  offset: result.offset,
  adjacency: { start: Array.from(adj.start), nbr: Array.from(adj.nbr) },
  stats,
  patches: patches.map((p) => ({
    faces: Array.from(p.faces), area: p.area, mid: p.mid, n: p.n, u: p.u, t: p.t,
    h: p.h, d: p.d, u0: p.u0, u1: p.u1, t0: p.t0, t1: p.t1, z0: p.z0, z1: p.z1,
    lean: p.lean, flatness: p.flatness, tris: Array.from(p.tris),
    keys: Object.keys(p),
  })),
  samples,
}));
console.log(`${path.basename(stlPath)} pose=${rotStr}${engineRotStr ? ` rot=${engineRotStr}` : ''}: ${patches.length} patches, stats ${JSON.stringify(stats)}`);
