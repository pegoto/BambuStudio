#!/usr/bin/env node
// Reference oracle for Engine/Prop/Tines.* (web/prop/tines.js).
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/tines_ref.mjs [outDir]
//
// outDir defaults to refs_tines/ next to tools/. Per scene (posed like tools/ref.mjs,
// seated + analyzed exactly as plugins/shared/engine/fins_entry.js computeFins does):
//
// 1. CAPTURE. The whole pipeline (buildFins, as computeFins calls it) runs under a few
//    option sets, with web/prop/tines.js swapped -- by a module resolve hook, so every
//    importer (prop.js, draw.js via fill.js, fins/wedges.js) gets it -- for a thin
//    wrapper that records each emitTines call: its arguments, the caller (file:line),
//    the PROP state it ran under, and what it appended to `out` (triangles, interface
//    tags, count, grip). So every wall top line is exactly the one prop.js / draw.js /
//    wedges.js built, and every call shape (plate body + tails, part-attached, squat,
//    drawn, wedge) is the real one.
// 2. REPLAY. Every distinct wall (line, minTop, body, gap) is driven again directly:
//    tineDensity 0 / 0.5 / 1 x layerHeight 0.2 / 0.12 / 0.28 x PROP.tinesPerWall 0 / 5,
//    each with PROP.iface off and 'all' (the triangles must not change, only the tags),
//    with globalThis.__TINECAP on (each placed tine's seed + bite heading); plus the
//    all-defaults call shape (wedges.js / tests: six arguments) and one call in a frame
//    turned about z (non-identity rot, through biteDirsAt / insidePart / kissEnds).
// 3. tineStepFor over a sweep of densities (incl. undefined/null/NaN/+-Infinity/-0),
//    under the default PROP and a modified tineStep/tineStepSparse, into stepfor.json.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import { register } from 'node:module';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_tines');
mkdirSync(outDir, { recursive: true });
const url = (p) => pathToFileURL(path.join(SF, p)).href;

// ---- the capture seam: redirect web/prop/tines.js to a recording wrapper -------------
const TINES = url('web/prop/tines.js');
const WRAP = 'data:text/javascript,' + encodeURIComponent(`
import * as real from ${JSON.stringify(TINES + '?real')};
export const tineStepFor = real.tineStepFor;
export function emitTines(...a) {
  const h = globalThis.__TINES_HOOK;
  const pre = h ? h.pre(a, new Error().stack) : null;
  const n = real.emitTines(...a);
  if (h) h.post(pre, a, n);
  return n;
}`);
const HOOKS = 'data:text/javascript,' + encodeURIComponent(`
export async function resolve(spec, ctx, next) {
  const r = await next(spec, ctx);
  if (r.url === ${JSON.stringify(TINES)}) return { ...r, url: ${JSON.stringify(WRAP)}, shortCircuit: true };
  return r;
}`);
register(HOOKS);

const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3 } = await import(url('web/overhangs.js'));
const { buildFins } = await import(url('web/fins.js'));
const { seatSoup, clearances } = await import(url('plugins/shared/engine/seat.js'));
const { ENGINE_DEFAULTS } = await import(url('plugins/shared/engine/fins_entry.js'));
const { PROP } = await import(url('web/prop/config.js'));
const { emitTines, tineStepFor } = await import(TINES + '?real');

const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['cube', '30,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'],
  ['staircase', '40,0,0'], ['sphere', '0,0,0'], ['tube', '40,0,0'], ['cylinder', '40,0,0'], ['coin', '40,0,0'],
  ['ramp', '40,0,0'], ['plus', '40,0,0'], ['needle', '40,0,0'], ['lowledge', '40,0,0'],
  // the other call shapes: walls on the part (prop.js part-attached: a floor under the
  // overhang; portal / ushape on their side have a flat ceiling, no bite; tilted 60: tines), squat
  // walls (brim minTop; they sit under near-level faces, so no tine lands -- see the
  // `squat` replay below), wedges (the six-argument call)
  ['portal', '0,60,0'], ['ushape', '0,60,0'], ['portal', '0,90,0'], ['lowledge', '0,0,0'],
  ['portal', '0,35,0'], ['arch', '5,0,0'],
];
// pipeline option sets for the capture: [options, extra tunables]
const VARIANTS = [
  [{}, {}],
  [{ tineDensity: 1, layerHeight: 0.12 }, { iface: 'all' }],
  [{ tineDensity: 0.5, layerHeight: 0.28 }, { tinesPerWall: 5 }],
  [{ tineDensity: 1, material: 'petg' }, {}],
  [{ mode: 'full', tineDensity: 0.5 }, {}],            // + fins/fill.js -> draw.js drawnWall
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
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  return [1 - (yy + zz), xy + wz, xz - wy, xy - wz, 1 - (xx + zz), yz + wx, xz + wy, yz - wx, 1 - (xx + yy)];
}
// ---------------------------------------------------------------------------------------

// JSON can't hold NaN/Infinity or -0: encode them as strings, the test decodes.
const num = (v) => (Object.is(v, -0) ? '-0' : Number.isFinite(v) ? v : String(v));
const numDeep = (v) => (Array.isArray(v) ? v.map(numDeep) : typeof v === 'number' ? num(v) : v);
const flat = (out, from) => { const r = []; for (let i = from; i < out.length; i++) r.push(num(out[i][0]), num(out[i][1]), num(out[i][2])); return r; };
// interface tags of out[from..] as run lengths, starting with an untagged run
const tagRuns = (out, from) => {
  const runs = []; let cur = 0, len = 0;
  for (let i = from; i < out.length; i++) {
    const t = out[i][3] === 1 ? 1 : 0;
    if (t !== cur) { runs.push(len); cur = t; len = 0; }
    len++;
  }
  runs.push(len);
  return runs;
};
const IFACE = (v) => (v === 'all' ? 'all' : v === 'flat' ? 'flat' : v ? String(v) : 'off');
const propState = () => ({ gap: PROP.gap, tinesPerWall: PROP.tinesPerWall, iface: IFACE(PROP.iface),
                           ifaceLayers: PROP.ifaceLayers });
const callerOf = (stack) => {
  for (const l of stack.split('\n').slice(1)) {
    const m = /(web\/[\w/.-]+\.js):(\d+)/.exec(l);
    if (m && !m[1].endsWith('prop/tines.js')) return `${m[1]}:${m[2]}`;
  }
  return '?';
};
const DUMMY = () => [[1, 2, 3], [4, 5, 6], [7, 8, 9]];   // a pre-existing untagged triangle

const resetProp = () => { PROP.iface = false; PROP.tinesPerWall = 0; PROP.gap = 0.2; };

// one direct emitTines call; returns its record
function drive(line, topo, rot, off, a /* {stepArg, minTop, tineH, body, grip, argc} */) {
  const out = DUMMY(), grip = a.grip ? { z: Infinity } : null;
  globalThis.__TINECAP = [];
  const args = [line, null, topo, rot, off, out, a.stepArg, a.minTop, a.tineH, a.body, grip].slice(0, a.argc);
  const n = emitTines(...args);
  const caps = globalThis.__TINECAP.map((c) => [c.x, c.y, c.z, c.biteX, c.biteY].map(num));
  delete globalThis.__TINECAP;
  if (out.slice(0, 3).some((v) => v[3] === 1)) throw new Error('tagged before from');
  return { n, out, tris: flat(out, 3), tags: tagRuns(out, 3), grip: grip ? num(grip.z) : null, caps };
}

const stats = { calls: 0, walls: 0, runs: 0, tines: 0, bySrc: {} };
for (const [model, rotStr] of SCENES) {
  const raw = readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
  const r = sitePose(rotStr.split(',').map(Number));
  const posed = new Float64Array(raw.length);
  for (let i = 0; i < raw.length; i += 3) {
    const x = raw[i], y = raw[i + 1], z = raw[i + 2];
    posed[i] = r[0] * x + r[3] * y + r[6] * z;
    posed[i + 1] = r[1] * x + r[4] * y + r[7] * z;
    posed[i + 2] = r[2] * x + r[5] * y + r[8] * z;
  }

  // ---- 1. capture, through buildFins exactly as computeFins calls it
  const calls = [];
  let topoRef = null, offRef = null;
  for (const [options, extra] of VARIANTS) {
    resetProp();
    const opts = { ...ENGINE_DEFAULTS, ...options };
    const { tunables } = clearances(opts);
    const { pos } = seatSoup(posed);
    const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
    const result = analyze(topo, opts.threshold, IDENTITY3);
    topoRef ??= topo; offRef ??= { x: result.offset.x, y: result.offset.y, z: result.offset.z };
    globalThis.__TINES_HOOK = {
      pre: (a, stack) => ({
        src: callerOf(stack), from: a[5].length, argc: a.length,
        line: a[0].map((p) => [p[0], p[1], p[2]]),
        rot: [...a[3]], offset: { x: a[4].x, y: a[4].y, z: a[4].z },
        stepArg: a[6], minTop: a[7], tineH: a[8], body: a[9] ? [...a[9]] : a[9],
        grip: a[10] ? a[10].z : a[10], prop: propState(),
      }),
      post: (pre, a, n) => {
        if (a[2] !== topo) throw new Error('emitTines on a different topology');
        const out = a[5];
        calls.push({ ...pre, n, tris: flat(out, pre.from), tags: tagRuns(out, pre.from),
                     gripAfter: a[10] ? a[10].z : null });
      },
    };
    buildFins(topo, result, IDENTITY3, {
      mode: opts.mode, bedPad: opts.bedPad && opts.padStyle !== 'off', tines: opts.tines,
      tineDensity: opts.tineDensity, layerHeight: opts.layerHeight, coverage: opts.coverage,
      plateOnly: opts.plateOnly, sway: undefined, tunables: { ...tunables, ...extra },
    });
    delete globalThis.__TINES_HOOK;
  }
  resetProp();
  const und = (v) => (v === undefined ? 'undefined' : v === null ? null : numDeep(v));
  const callsOut = calls.map((c) => {
    stats.bySrc[c.src] = (stats.bySrc[c.src] || 0) + 1;
    return {
      src: c.src, argc: c.argc, line: numDeep(c.line), rot: numDeep(c.rot),
      offset: { x: num(c.offset.x), y: num(c.offset.y), z: num(c.offset.z) },
      stepArg: und(c.stepArg), minTop: und(c.minTop), tineH: und(c.tineH), body: und(c.body),
      grip: und(c.grip), prop: c.prop, n: c.n, tris: c.tris, tags: c.tags, gripAfter: und(c.gripAfter),
    };
  });
  stats.calls += calls.length;

  // ---- 2. replay every distinct wall
  const topo = topoRef, off = offRef;
  const seen = new Map();
  for (const c of calls) {
    if (c.line.length < 2) continue;
    const key = JSON.stringify([c.line, c.minTop ?? null, c.body ?? null, c.prop.gap]);
    if (!seen.has(key)) seen.set(key, c);
  }
  // a frame turned about z: the same walls, rotated with the part
  const th = 0.7, cz = Math.cos(th), sz = Math.sin(th);
  const rotZ = [cz, sz, 0, -sz, cz, 0, 0, 0, 1];
  const resZ = analyze(topo, DEFAULT_THRESHOLD, rotZ);
  const offZ = { x: resZ.offset.x, y: resZ.offset.y, z: resZ.offset.z };
  const toZ = (p) => {
    const mx = p[0] - off.x, my = p[1] - off.y, mz = p[2] - off.z;   // back to the mesh frame (rot = I)
    return [rotZ[0] * mx + rotZ[3] * my + rotZ[6] * mz + offZ.x,
            rotZ[1] * mx + rotZ[4] * my + rotZ[7] * mz + offZ.y,
            rotZ[2] * mx + rotZ[5] * my + rotZ[8] * mz + offZ.z];
  };

  const walls = [];
  for (const c of seen.values()) {
    const runs = [];
    for (const density of [0, 0.5, 1]) {
      for (const tineH of [0.2, 0.12, 0.28]) {
        for (const tpw of [0, 5]) {
          PROP.gap = c.prop.gap; PROP.tinesPerWall = tpw;
          const a = { stepArg: tineStepFor(density), minTop: c.minTop, tineH, body: c.body ?? null, grip: true, argc: 11 };
          PROP.iface = false;
          const off0 = drive(c.line, topo, IDENTITY3, off, a);
          PROP.iface = 'all';
          const all = drive(c.line, topo, IDENTITY3, off, a);
          if (all.n !== off0.n || JSON.stringify(all.tris) !== JSON.stringify(off0.tris)) throw new Error('iface changed geometry');
          runs.push({ density, tineH, tpw, n: off0.n, tris: off0.tris, tagsOff: off0.tags, tagsAll: all.tags,
                      grip: off0.grip, caps: off0.caps });
          stats.tines += off0.n;
          stats.runs += 2;
        }
      }
    }
    resetProp();
    PROP.gap = c.prop.gap;
    const dflt = drive(c.line, topo, IDENTITY3, off, { argc: 6 });
    // the squat-wall call shape (prop.js / draw.js: minTop = PROP.squatBrimH, no body)
    const squat = drive(c.line, topo, IDENTITY3, off,
                        { stepArg: tineStepFor(0), minTop: PROP.squatBrimH, tineH: 0.2, body: null, grip: true, argc: 11 });
    const lineZ = c.line.map(toZ);
    const rotated = drive(lineZ, topo, rotZ, offZ,
                          { stepArg: tineStepFor(1), minTop: c.minTop, tineH: 0.2, body: c.body ?? null, grip: true, argc: 11 });
    resetProp();
    stats.runs += 3;
    walls.push({
      src: c.src, gap: c.prop.gap, line: numDeep(c.line), minTop: und(c.minTop), body: und(c.body ?? null),
      runs, dflt: { n: dflt.n, tris: dflt.tris, caps: dflt.caps },
      squat: { n: squat.n, tris: squat.tris, grip: squat.grip, caps: squat.caps },
      rotated: { line: numDeep(lineZ), n: rotated.n, tris: rotated.tris, grip: rotated.grip, caps: rotated.caps },
    });
  }
  stats.walls += walls.length;

  const file = path.join(outDir, `${model}-${rotStr.replace(/,/g, '_')}.json`);
  writeFileSync(file, JSON.stringify({
    model, rot: rotStr, soup: Array.from(posed).map(num),
    offset: { x: num(off.x), y: num(off.y), z: num(off.z) },
    rotZ: numDeep(rotZ), offsetZ: { x: num(offZ.x), y: num(offZ.y), z: num(offZ.z) },
    calls: callsOut, walls,
  }));
  const srcs = {};
  for (const c of calls) srcs[c.src] = (srcs[c.src] || 0) + 1;
  console.log(`${model} ${rotStr}: faces=${topo.nFaces} calls=${calls.length} (${calls.reduce((s, c) => s + c.n, 0)} tines) ` +
              `walls=${walls.length} replay tines=${walls.reduce((s, w) => s + w.runs.reduce((t, r) => t + r.n, 0), 0)} ` +
              `rotated tines=${walls.reduce((s, w) => s + w.rotated.n, 0)} squat tines=${walls.reduce((s, w) => s + w.squat.n, 0)} ${JSON.stringify(srcs)}`);
}

// ---- 3. tineStepFor
const DENS = [undefined, null, -1, -0, 0, 1e-300, 0.05, 0.1, 0.25, 1 / 3, 0.5, 0.7, 0.75, 0.9, 0.95, 1, 1.5, 2,
              NaN, Infinity, -Infinity];
const stepFor = [];
for (const [ts, tss] of [[PROP.tineStep, PROP.tineStepSparse], [1.7, 6.3], [2.5, 2.5]]) {
  const save = [PROP.tineStep, PROP.tineStepSparse];
  PROP.tineStep = ts; PROP.tineStepSparse = tss;
  for (const d of DENS) stepFor.push({ tineStep: ts, tineStepSparse: tss, density: d === undefined ? 'undefined' : d === null ? null : num(d), out: num(tineStepFor(d)) });
  for (let i = 0; i <= 100; i++) stepFor.push({ tineStep: ts, tineStepSparse: tss, density: num(i / 100), out: num(tineStepFor(i / 100)) });
  [PROP.tineStep, PROP.tineStepSparse] = save;
}
writeFileSync(path.join(outDir, 'stepfor.json'), JSON.stringify(stepFor));
console.log(`total: ${stats.calls} captured calls, ${stats.walls} distinct walls, ${stats.runs} replays, ${stepFor.length} tineStepFor; callers ${JSON.stringify(stats.bySrc)}`);
