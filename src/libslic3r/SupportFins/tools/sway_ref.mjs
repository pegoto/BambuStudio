#!/usr/bin/env node
// Reference oracle for Engine/Sway.* (web/sway.js). Poses each stress model like
// tools/ref.mjs, seats + analyzes it exactly as plugins/shared/engine/fins_entry.js
// computeFins does (seatSoup, buildTopology, analyze(..., IDENTITY3)), then runs every
// export of sway.js and dumps inputs + outputs as JSON for tests/test_sway.cpp:
//   - buildSwayBraces with the options fins.js passes (sway on, the form's defaults)
//     and with variants (gripFrom, tineSpacing, reach, tines off, gap, layer height,
//     interface tagging, clamped values, avoid.walls centrelines placed on the braces);
//   - faceIsUpright on every face;
//   - swayAtFace on picks over the largest upright patches, the way Draw mode calls it
//     (plain, against the Auto braces + walls, with allowStilt forced off, a roof face);
//   - swayClashes on every ordered pair of Auto ribs, swayClashesWall against the walls.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/sway_ref.mjs [outDir]
//
// outDir defaults to refs_sway/ next to tools/ (gitignored).
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR;
if (!SF) { console.error('set SUPPORT_FINS_DIR'); process.exit(2); }
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_sway');
mkdirSync(outDir, { recursive: true });

const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);
const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3 } = await imp('web/overhangs.js');
const sway = await imp('web/sway.js');
const { findWallPatches } = await imp('web/planes.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

const MODELS = [
  ['bar', '0,0,0'], ['needle', '0,0,0'], ['cylinder', '0,0,0'], ['tube', '0,0,0'],
  ['hexprism', '0,0,0'], ['plus', '0,0,0'], ['lbracket', '0,0,0'],
  ['cube', '0,0,0'], ['cube', '40,0,0'],
  // poses that reach the rarer refusals: a face starting high (stilt), a short face,
  // a base spreading under the face, no upright side at all
  ['lbracket', '0,90,0'], ['bar', '20,0,0'], ['hexprism', '20,0,0'], ['cube', '45,35,0'],
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
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  // column-major 3x3 like three.js Matrix3.elements
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}
// -------------------------------------------------------------------------------------

// non-finite numbers survive JSON as strings (the C++ test decodes them)
const enc = (v) => (Number.isFinite(v) ? (Object.is(v, -0) ? '-0' : v) : String(v));
const flat = (verts) => verts.flatMap((v) => [enc(v[0]), enc(v[1]), enc(v[2])]);
const tagsOf = (verts) => verts.map((v) => (v.length > 3 && v[3] === 1 ? 1 : 0));
const pt = (v) => [enc(v[0]), enc(v[1]), enc(v[2])];

/** Every field of a rib record (buildSwayRib / swayAtFace result, plus triRange). */
function ribDump(r, withTris) {
  if (!r) return null;
  const o = { keys: Object.keys(r), ok: r.ok };
  if (!r.ok) { o.reason = r.reason; return o; }
  Object.assign(o, {
    tines: r.tines, height: enc(r.height), depth: enc(r.depth), th: enc(r.th), stilt: enc(r.stilt),
    halfW: enc(r.halfW), foot: r.foot.map(pt),
    levels: r.levels.map((L) => ({ z: enc(L.z), a: pt(L.a), b: pt(L.b) })),
    nTris: r.tris.length,
  });
  if (r.triRange) o.triRange = r.triRange;
  if (withTris) { o.tris = flat(r.tris); o.tags = tagsOf(r.tris); }
  return o;
}

// fins.js buildFinsAndBraces: { ...opts.sway, tines, layerHeight, iface: PROP.iface, avoid: { walls } }
// with ui/finbuild.js swayOpts()'s form defaults (tests/_scene.js siteOpts, PLA gap 0.2).
const BASE = { on: true, gripFrom: 0, tineSpacing: 6, reach: 0.15, gap: 0.2,
               tines: true, layerHeight: 0.2, iface: 'off', avoid: { walls: [] } };
const VARIANTS = [
  ['default', {}],
  ['grip20', { gripFrom: 20 }],
  ['spacing3', { tineSpacing: 3 }],
  ['reach03', { reach: 0.3 }],
  ['notines', { tines: false }],
  ['gap03', { gap: 0.3 }],
  ['layer012', { layerHeight: 0.12 }],
  ['ifaceAll', { iface: 'all' }],
  ['ifaceFlat', { iface: 'flat' }],
  ['clamped', { reach: 0.9, tineSpacing: 0.5, gripFrom: -5, layerHeight: 0.01 }],
  ['clampedLow', { reach: 0.01 }],
];

/** Synthetic prop/wedge centrelines where a brace would go: one 2-point line across the
 *  first rib's foot, one single point just off the last rib's outer foot end, and a
 *  3-point polyline far away (never hit). */
function wallsFor(ribs) {
  const walls = [];
  if (ribs.length) {
    const f = ribs[0].foot;
    const mx = (f[0][0] + f[1][0]) / 2, my = (f[0][1] + f[1][1]) / 2;
    walls.push([[mx - 2, my - 2, 0], [mx + 2, my + 2, 5]]);
  }
  if (ribs.length > 1) {
    const f = ribs[ribs.length - 1].foot;
    walls.push([[f[1][0] + 3, f[1][1] - 1, 0]]);
  }
  walls.push([[500, 500, 0], [520, 500, 0], [520, 530, 3]]);
  return walls;
}

function runModel(name, poseStr) {
  const raw = readSTL(path.join(SF, 'prototype/stress/models', `${name}.stl`));
  const pose = sitePose(poseStr.split(',').map(Number));
  const posed = new Float64Array(raw.length);
  for (let i = 0; i < raw.length; i += 3) {
    const x = raw[i], y = raw[i + 1], z = raw[i + 2];
    posed[i] = pose[0] * x + pose[3] * y + pose[6] * z;
    posed[i + 1] = pose[1] * x + pose[4] * y + pose[7] * z;
    posed[i + 2] = pose[2] * x + pose[5] * y + pose[8] * z;
  }
  const { pos } = seatSoup(posed);
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const rot = IDENTITY3;
  const result = analyze(topo, DEFAULT_THRESHOLD, rot);

  const dumpBraces = (opts) => {
    const s = sway.buildSwayBraces(topo, result, rot, opts);
    return {
      keys: Object.keys(s),
      count: s.count, tines: s.tines, skipped: s.skipped, reason: s.reason ?? null,
      ribs: s.ribs.map((r) => ribDump(r, false)),
      triangles: flat(s.triangles), tags: tagsOf(s.triangles),
      _ribs: s.ribs,
    };
  };

  const runs = [];
  let auto = null;
  for (const [vname, extra] of VARIANTS) {
    const opts = { ...BASE, ...extra };
    const d = dumpBraces(opts);
    if (vname === 'default') auto = d._ribs;
    delete d._ribs;
    runs.push({ name: vname, opts, out: d });
  }
  const walls = wallsFor(auto);
  {
    const opts = { ...BASE, avoid: { walls } };
    const d = dumpBraces(opts);
    delete d._ribs;
    runs.push({ name: 'avoidWalls', opts: { ...opts, avoid: { walls: walls.map((l) => l.map(pt)) } }, out: d });
  }

  // clash checks on the Auto ribs: every ordered pair, and each against the walls
  const clashes = [];
  for (let i = 0; i < auto.length; i++) {
    for (let j = 0; j < auto.length; j++) {
      if (i !== j) clashes.push({ i, j, v: sway.swayClashes(auto[i], [auto[j]]) });
    }
  }
  const clashAll = auto.map((r, i) => sway.swayClashes(r, auto.filter((_, k) => k !== i)));
  const clashWall = auto.map((r) => sway.swayClashesWall(r, walls));
  const clashWallEach = auto.map((r) => walls.map((w) => sway.swayClashesWall(r, [w])));

  // swayClashes' level-by-level path: a copy of rib j with its foot parked far away (so
  // the foot test passes), its levels shifted by (dx, dy) and squashed in z by 0.73 (so
  // levelAt interpolates between rungs), against rib i. The C++ test builds the same
  // copy from its own rib j with the same operations.
  const SHIFTS = [-30, -12, -5, -2, 0, 2, 5, 12, 30];
  const levelClashes = [];
  for (let i = 0; i < auto.length; i++) {
    for (let j = 0; j < auto.length; j++) {
      for (const dx of SHIFTS) for (const dy of SHIFTS) {
        const r = auto[j];
        const copy = { foot: [[1e4, 1e4, 0], [1e4 + 1, 1e4, 0]], halfW: r.halfW, th: r.th, height: r.height * 0.73,
                       levels: r.levels.map((L) => ({ z: L.z * 0.73, a: [L.a[0] + dx, L.a[1] + dy, L.a[2]],
                                                     b: [L.b[0] + dx, L.b[1] + dy, L.b[2]] })) };
        levelClashes.push(sway.swayClashes(auto[i], [copy]) ? 1 : 0);
      }
    }
  }

  // faceIsUpright on every face
  const upright = [];
  for (let f = 0; f < topo.nFaces; f++) upright.push(sway.faceIsUpright(topo, rot, f) ? 1 : 0);

  // Draw-mode picks: faces of the largest patches, at the face centroid and shifted
  // across it, plus a non-upright face (a roof / floor) for the refusal.
  const patches = findWallPatches(topo, rot, result.offset);
  const picks = [];
  const centroid = (f) => {
    const c = [0, 0, 0];
    for (let k = 0; k < 3; k++) for (let a = 0; a < 3; a++) {
      c[a] += topo.pos[f * 9 + k * 3 + a] + [result.offset.x, result.offset.y, result.offset.z][a];
    }
    return c.map((v) => v / 3);
  };
  // across the first 8 patches, 5 spots each, with and without allowStilt forced off
  for (const p of patches.slice(0, 8)) {
    for (let k = 0; k <= 4; k++) {
      const u = p.u0 + ((p.u1 - p.u0) * k) / 4;
      const point = [p.u.x * u + p.n.x * p.d, p.u.y * u + p.n.y * p.d, 0];
      picks.push({ face: p.faces[0], point, avoid: 'none' });
      picks.push({ face: p.faces[0], point, avoid: 'none', allowStilt: false });
    }
  }
  for (const p of patches.slice(0, 4)) {
    const fs = [p.faces[0], p.faces[p.faces.length - 1]];
    for (const f of fs) {
      const c = centroid(f);
      picks.push({ face: f, point: c, avoid: 'none' });
      picks.push({ face: f, point: [c[0] + 5 * p.u.x, c[1] + 5 * p.u.y, c[2]], avoid: 'none' });
    }
    const c = centroid(p.faces[0]);
    picks.push({ face: p.faces[0], point: c, avoid: 'auto' });
    picks.push({ face: p.faces[0], point: c, avoid: 'walls' });
    picks.push({ face: p.faces[0], point: c, avoid: 'none', allowStilt: false });
    picks.push({ face: p.faces[0], point: c, avoid: 'none', gripFrom: 25, tineSpacing: 4 });
  }
  let roof = -1;
  for (let f = 0; f < topo.nFaces; f++) if (!upright[f]) { roof = f; break; }
  if (roof >= 0) picks.push({ face: roof, point: centroid(roof), avoid: 'none' });
  // the UI's swayOpts(): no `on`, avoid passed separately
  const UI = { gripFrom: 0, tineSpacing: 6, reach: 0.15, gap: 0.2, tines: true, iface: 'off', layerHeight: 0.2 };
  const atFace = picks.map((pk) => {
    const opts = { ...UI };
    if ('allowStilt' in pk) opts.allowStilt = pk.allowStilt;
    if ('gripFrom' in pk) { opts.gripFrom = pk.gripFrom; opts.tineSpacing = pk.tineSpacing; }
    const avoid = pk.avoid === 'auto' ? { braces: auto, walls: [] }
      : pk.avoid === 'walls' ? { braces: [], walls } : { braces: [], walls: [] };
    const r = sway.swayAtFace(topo, result, rot, pk.face, pk.point, opts, avoid);
    // and the new brace against the Auto ones / the walls
    const vsAuto = r.ok ? sway.swayClashes(r, auto) : null;
    const vsWalls = r.ok ? sway.swayClashesWall(r, walls) : null;
    return { ...pk, point: pk.point.map(enc), opts, r: ribDump(r, true), vsAuto, vsWalls };
  });

  return {
    model: name, pose: poseStr, rot: Array.from(rot),
    leanCut: Math.sin((sway.SWAY.maxLeanDeg * Math.PI) / 180),
    SWAY: sway.SWAY,
    pos: Array.from(pos),
    topoArea: Array.from(topo.area), topoNrm: Array.from(topo.nrm),
    offset: { x: enc(result.offset.x), y: enc(result.offset.y), z: enc(result.offset.z) },
    bearings: patches.map((p) => Math.atan2(p.n.y, p.n.x)),
    walls: walls.map((l) => l.map(pt)),
    runs, clashes, levelClashes, shifts: SHIFTS, clashAll, clashWall, clashWallEach, upright, atFace,
  };
}

for (const [name, poseStr] of MODELS) {
  const d = runModel(name, poseStr);
  const tag = `${name}-${poseStr.split(',').join('_')}`;
  writeFileSync(path.join(outDir, `${tag}.json`), JSON.stringify(d));
  const def = d.runs[0].out;
  console.log(`${tag}: count ${def.count} tines ${def.tines} skipped ${def.skipped} reason ${def.reason}; `
    + `runs ${d.runs.map((r) => `${r.name}:${r.out.count}/${r.out.tines}`).join(' ')}; `
    + `atFace ok ${d.atFace.filter((a) => a.r?.ok).length}/${d.atFace.length}`);
}
