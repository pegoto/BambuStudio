#!/usr/bin/env node
// Reference oracle for Engine/Fins/Wedges.* (web/fins/wedges.js).
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/wedges_ref.mjs [outDir] [sceneFilter]
//
// outDir defaults to refs_wedges/ next to tools/. Per scene (posed like tools/ref.mjs,
// seated + analyzed exactly as plugins/shared/engine/fins_entry.js computeFins does:
// seatSoup, buildTopology, analyze(..., IDENTITY3)):
//   patches     findWallPatches(topo, rot, offset), identified (the C++ recomputes them)
//   grip        gripPatches: the downward patches (indices) and the faceMap in insertion order
//   cols        perpColumns on EVERY patch: [u0+inset, u1-inset] at the three coverage
//               pitches (fins.js coverPitch 0 / 0.5 / 1) and PERP.pitch, and [u0, u1] at 10,
//               under PERP.gap 0.2 and 0.3
//   perp        buildPerpFins under 8 states (PLA / PETG tunables applied with fins.js
//               applyTunables x iface off / 'all' x CUT.pattern none / diamond; crestPart as
//               buildFins sets it, layerHeight 0.2 PLA / 0.12 PETG): on every `wedgeable`
//               patch (exactly fins.js's filter) with wedgeOpts tines on/off x pitch from
//               coverage 0 / 0.5 / 1 x tineDensity 0 / 1, plus `{}`; on the other downward
//               patches (Draw's gripPatches) with `{}`. Results (triangles in emission
//               order, interface tags, tines, count, every wedge record field) are deduped
//               into `results`.
//   builds      three auto-mode configs: upstream prop.js buildProps exactly as fins.js
//               calls it, with rasterVeto = the real wedgeVeto wrapped to record every
//               call (q, the normalProps object's identity, the verdict); then a fresh
//               wedgeVeto over every base prop; propServesPatch(p, base.props) and
//               propServesPatch(p, [q]) for every patch x prop; fins.js's wedge loop
//               (served / count per wedgeable patch), and bareAfterWedges /
//               unservedAfterWedges with the real servedRegions and wedgeTris (and with
//               no wedges), plus one bareAfterWedges in a frame turned about z.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_wedges');
const filter = process.argv[3] || null;
mkdirSync(outDir, { recursive: true });
const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);

const { PROP } = await imp('web/prop/config.js');
const { CUT } = await imp('web/cutout.js');
const crest = await imp('web/prop/crest.js');
const prop = await imp('web/prop.js');
const { applyTunables } = await imp('web/fins.js');
const { FIN } = await imp('web/fins/config.js');
const W = await imp('web/fins/wedges.js');
const { PERP } = W;
const { seatedPartTris } = await imp('web/fins/seating.js');
const { findWallPatches } = await imp('web/planes.js');
const { MATERIAL } = await imp('web/materials.js');
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
// ---------------------------------------------------------------------------------------

const MODELS = ['arch', 'bar', 'coin', 'cone', 'cube', 'cylinder', 'hexprism', 'lbracket', 'lowledge', 'needle',
                'plate', 'plus', 'portal', 'pyramid', 'ramp', 'sphere', 'staircase', 'torus', 'tshape', 'tube',
                'ushape', 'wedge'];
const SCENES = [
  ...MODELS.map((m) => [m, '40,0,0']),
  ['cube', '20,0,0'], ['cube', '30,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'],
  ['portal', '0,35,0'], ['portal', '0,60,0'], ['ramp', '60,0,0'], ['wedge', '0,0,0'], ['wedge', '0,30,0'],
  ['plate', '60,0,0'],
];

// JSON can't hold NaN/Infinity or -0: encode them as strings, the test decodes.
const num = (v) => (Object.is(v, -0) ? '-0' : Number.isFinite(v) ? v : String(v));
const flat = (tris) => { const r = []; for (const v of tris) r.push(num(v[0]), num(v[1]), num(v[2])); return r; };
const pts = (l) => (l ?? []).map((p) => [num(p[0]), num(p[1]), num(p[2])]);
// interface tags as run lengths, starting with an untagged run
const tagRuns = (tris) => {
  const runs = []; let cur = 0, len = 0;
  for (const v of tris) {
    const t = v[3] === 1 ? 1 : 0;
    if (t !== cur) { runs.push(len); cur = t; len = 0; }
    len++;
  }
  runs.push(len);
  return runs;
};
function coverPitch(coverage) {   // fins.js (not exported)
  const c = Math.max(0, Math.min(1, coverage));
  return c <= 0.5
    ? FIN.coverSparse + ((0.5 - c) / 0.5) * (FIN.coverExtraSparse - FIN.coverSparse)
    : FIN.coverSparse - ((c - 0.5) / 0.5) * (FIN.coverSparse - FIN.coverDense);
}
const PITCHES = [0, 0.5, 1].map(coverPitch);

const PROP0 = { ...PROP }, PERP0 = { ...PERP }, CUT0 = { ...CUT };
const STATES = [];
for (const material of ['pla', 'petg'])
  for (const iface of [false, 'all'])
    for (const cutout of ['none', 'diamond'])
      STATES.push({ material, iface, cutout, layerHeight: material === 'pla' ? 0.2 : 0.12 });
// buildFins' entry for a state: defaults, then applyTunables, then crestPart
function enter(st, partTris) {
  Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0);
  const mat = MATERIAL[st.material];
  applyTunables({ padH: mat.padH, padGrab: mat.padGrab, propGap: mat.propGap, padStyle: 'auto', cutout: st.cutout, iface: st.iface });
  crest.crestPart(crest.crestOn() ? partTris : null, st.layerHeight);
}
const stateDump = (st) => ({ material: st.material, iface: st.iface === false ? 'off' : st.iface, cutout: st.cutout,
                             layerHeight: st.layerHeight, propGap: PROP.gap, perpGap: PERP.gap, cut: CUT.pattern });

// wedgeOpts sets: tines x coverage pitch x tineDensity, and {}
const FULL_OPTS = [];
for (const tines of [true, false])
  for (const pitch of PITCHES)
    for (const tineDensity of [0, 1]) FULL_OPTS.push({ tines, pitch, tineDensity });
const optsDump = (o) => ({ tines: o.tines ?? null, pitch: o.pitch === undefined ? null : num(o.pitch),
                           tineDensity: o.tineDensity === undefined ? null : num(o.tineDensity) });

const BUILDS = [
  { state: 0, opts: { tines: true, coverage: 0.5, tineDensity: 0, layerHeight: 0.2 } },              // pla off none
  { state: 7, opts: { tines: true, coverage: 1, tineDensity: 1, layerHeight: 0.12 } },               // petg all diamond
  { state: 0, opts: { tines: false, coverage: 0, layerHeight: 0.2 } },
];

let total = { perp: 0, results: 0, wedges: 0, veto: 0, vetoYes: 0, cols: 0 };
for (const [model, rotStr] of SCENES) {
  const name = `${model}-${rotStr.replace(/,/g, '_')}`;
  if (filter && !name.includes(filter)) continue;
  const t0 = performance.now();
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
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, 45, IDENTITY3);
  const rot = IDENTITY3, off = result.offset;
  const partTris = seatedPartTris(topo, rot, off);

  Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0);
  const all = findWallPatches(topo, rot, off);
  const patches = all.map((p) => ({ seed: p.faces[0], faces: p.faces.length, area: num(p.area), nz: num(p.n.z),
                                    u0: num(p.u0), u1: num(p.u1), t0: num(p.t0), t1: num(p.t1) }));
  const idx = new Map(all.map((p, i) => [p, i]));

  // ---- gripPatches
  const g = W.gripPatches(topo, result, rot);
  const gIdx = g.patches.map((p) => {
    const i = all.findIndex((q) => q.faces[0] === p.faces[0] && q.faces.length === p.faces.length && q.area === p.area);
    if (i < 0) throw new Error('grip patch not in findWallPatches');
    return i;
  });
  const gPos = new Map(g.patches.map((p, i) => [p, i]));
  const faceMap = [...g.faceMap].map(([f, p]) => [f, gPos.get(p)]);

  // ---- perpColumns on every patch
  const cols = [];
  for (const gap of [0.2, 0.3]) {
    PERP.gap = gap;
    all.forEach((p, i) => {
      const args = [...[...PITCHES, PERP.pitch].map((pitch) => [p.u0 + PERP.inset, p.u1 - PERP.inset, pitch]), [p.u0, p.u1, 10]];
      for (const [lo, hi, pitch] of args) {
        cols.push({ patch: i, gap, lo: num(lo), hi: num(hi), pitch: num(pitch), out: W.perpColumns(p, lo, hi, pitch).map(num) });
      }
    });
  }
  PERP.gap = PERP0.gap;
  total.cols += cols.length;

  // ---- buildPerpFins
  const wedgeableOf = () => all.filter((p) =>
    p.n.z < -0.05 && p.area >= PERP.minArea && (p.u1 - p.u0) >= PERP.minWidth);
  const results = [], resIdx = new Map();
  const resOf = (w) => {
    const r = { triangles: flat(w.triangles), tags: tagRuns(w.triangles), tines: w.tines, count: w.count,
                wedges: w.wedges.map((wd) => ({ triRange: wd.triRange, line: pts(wd.line), height: num(wd.height), span: num(wd.span) })),
                keys: Object.keys(w).sort() };
    const key = JSON.stringify(r);
    if (!resIdx.has(key)) { resIdx.set(key, results.length); results.push(r); total.wedges += w.wedges.length; }
    return resIdx.get(key);
  };
  const perp = [], states = [];
  for (let si = 0; si < STATES.length; si++) {
    enter(STATES[si], partTris);
    states.push(stateDump(STATES[si]));
    const wedgeable = new Set(wedgeableOf());
    for (const p of g.patches) {
      const optsList = wedgeable.has(all[gIdx[gPos.get(p)]]) ? [...FULL_OPTS, {}] : [{}];
      for (const o of optsList) {
        const w = W.buildPerpFins(p, topo, rot, off, o);
        perp.push({ patch: gIdx[gPos.get(p)], state: si, opts: optsDump(o), res: resOf(w) });
      }
    }
  }
  total.perp += perp.length; total.results += results.length;

  // ---- buildProps + wedgeVeto + propServesPatch + bareAfterWedges, as fins.js
  const builds = [];
  for (const b of BUILDS) {
    const st = STATES[b.state];
    enter(st, partTris);
    const withTines = b.opts.tines ?? true;
    const coverage = Math.max(0, Math.min(1, b.opts.coverage ?? FIN.coverDefault));
    const wedgeable = wedgeableOf();
    const wedgeOpts = { tines: withTines, pitch: coverPitch(coverage), tineDensity: b.opts.tineDensity };
    const real = W.wedgeVeto(wedgeable, topo, rot, off, wedgeOpts);
    const npIds = new Map(), normals = [], vetoCalls = [];
    const propSnap = JSON.stringify(PROP), perpSnap = JSON.stringify(PERP);
    const rasterVeto = (q, normalProps) => {
      if (JSON.stringify(PROP) !== propSnap || JSON.stringify(PERP) !== perpSnap) throw new Error('state changed under the veto');
      if (!npIds.has(normalProps)) {
        npIds.set(normalProps, normals.length);
        normals.push(normalProps.map((n) => ({ line: pts(n.line), squat: !!n.squat })));
      }
      const r = real(q, normalProps);
      vetoCalls.push({ line: pts(q.line), squat: !!q.squat, np: npIds.get(normalProps), r });
      return r;
    };
    const base = prop.buildProps(topo, result, rot, { ...b.opts, mode: 'prop', tines: withTines, rasterVeto });
    // a fresh closure over every base prop (normalProps = base.props)
    const veto2 = W.wedgeVeto(wedgeable, topo, rot, off, wedgeOpts);
    const post = base.props.map((q) => veto2(q, base.props));
    // and with no normal walls at all (every wedgeable patch that builds is open)
    const veto3 = W.wedgeVeto(wedgeable, topo, rot, off, wedgeOpts), none = [];
    const postNone = base.props.map((q) => veto3(q, none));
    const serves = all.map((p) => [W.propServesPatch(p, base.props), base.props.map((q) => W.propServesPatch(p, [q]))]);
    // fins.js's wedge loop
    const wedgeTris = [], loop = [];
    for (const p of wedgeable) {
      const served = W.propServesPatch(p, base.props);
      if (served) { loop.push({ patch: idx.get(p), served, count: null }); continue; }
      const w = W.buildPerpFins(p, topo, rot, off, wedgeOpts);
      loop.push({ patch: idx.get(p), served, count: w.count });
      if (!w.count) continue;
      for (const v of w.triangles) wedgeTris.push(v);
    }
    const servedRegions = base.servedRegions ?? [];
    const bare = W.bareAfterWedges(topo, rot, result, servedRegions, wedgeTris);
    const unserved = W.unservedAfterWedges(topo, rot, result, servedRegions, wedgeTris);
    const bareNone = W.bareAfterWedges(topo, rot, result, [], []);
    const bareWedges = W.bareAfterWedges(topo, rot, result, [], wedgeTris);   // the wedges' own credit
    builds.push({
      state: b.state, opts: b.opts, wedgeOpts: optsDump(wedgeOpts), wedgeable: wedgeable.map((p) => idx.get(p)),
      props: base.props.map((q) => ({ line: pts(q.line), squat: !!q.squat, kind: q.kind ?? null, id: q.id ?? null })),
      servedRegions, normals, vetoCalls, post, postNone, serves, loop, wedgeTrisLen: wedgeTris.length,
      wedgeTrisSum: num(wedgeTris.reduce((s, v) => s + v[0] + 2 * v[1] + 3 * v[2], 0)),
      bare, unserved, bareNone, bareWedges,
    });
    total.veto += vetoCalls.length + post.length + postNone.length;
    total.vetoYes += vetoCalls.filter((c) => c.r).length + post.filter(Boolean).length + postNone.filter(Boolean).length;
    if (b === BUILDS[0]) {
      // a frame turned about z (bareAfterWedges' own rotation): the identity build's wedges
      builds.at(-1).wedgeTris = flat(wedgeTris);
    }
  }
  // bareAfterWedges in a turned frame (after everything else: analyze refills scratch)
  Object.assign(PROP, PROP0); Object.assign(PERP, PERP0); Object.assign(CUT, CUT0);
  crest.crestPart(null);
  const th = 0.7, cz = Math.cos(th), sz = Math.sin(th);
  const rotZ = [cz, sz, 0, -sz, cz, 0, 0, 0, 1];
  const resZ = analyze(topo, 45, rotZ);
  const wt0 = builds[0].wedgeTris;
  const wtris = [];
  for (let i = 0; i < wt0.length; i += 3) wtris.push([wt0[i], wt0[i + 1], wt0[i + 2]]);
  const bareRot = { rot: rotZ, offset: { x: num(resZ.offset.x), y: num(resZ.offset.y), z: num(resZ.offset.z) },
                    regions: resZ.regions.length, served: [0],
                    out: W.bareAfterWedges(topo, rotZ, resZ, [0], wtris) };

  const file = path.join(outDir, `${name}.json`);
  writeFileSync(file, JSON.stringify({
    model, rot: rotStr, soup: Array.from(posed).map(num),
    offset: { x: num(off.x), y: num(off.y), z: num(off.z) }, regions: result.regions.length,
    PERP: PERP0, patches, grip: { idx: gIdx, faceMap }, cols, states, results, perp, builds, bareRot,
  }));
  console.log(`${name}: faces=${topo.nFaces} patches=${all.length} grip=${g.patches.length} ` +
              `wedgeable=${builds[0].wedgeable.length} perp=${perp.length} results=${results.length} ` +
              `wedges=${results.reduce((s, r) => s + r.count, 0)} veto=${builds.map((b) => b.vetoCalls.length + '/' + b.post.length + '/' + b.postNone.filter(Boolean).length).join(',')} ` +
              `bare=${builds.map((b) => b.bare.length + '/' + b.bareWedges.length + '/' + b.bareNone.length).join(',')} (${Math.round(performance.now() - t0)} ms)`);
}
console.log(`total: ${JSON.stringify(total)}`);
