#!/usr/bin/env node
// Reference oracle for Engine/Prop/Attached and Engine/Prop/Raster (web/prop/attached.js,
// raster.js). On stress models posed like tools/ref.mjs (seatSoup, buildTopology,
// analyze(DEFAULT_THRESHOLD, IDENTITY3) as fins_entry.js does), per config:
//   - the lines prop.js buildPass hands buildPartAttached: the normal pass's (tubeLine /
//     splitRegion + patchTracks) and the raster pass's (rasterTracks) for every region,
//     each run through buildPartAttached on a fresh `out` (with a tagged prefix), then
//     prop.js's part-attached tine glue (emitTines on the wall top + gap) when tines are
//     on; the module internals (buriedColumn / enclosedColumn / clearBetween) on every
//     station of the settled top;
//   - rasterTracks for every region, rasterRest on each raster line over a set of runs;
//   - the race: prop.js buildPass (the normal pass) and withRaster driving buildPass for
//     the regions it wants, with the tine capture on/off and fins.js's wedgeVeto as
//     opts.rasterVeto where auto mode passes one; every rasterPass / veto call recorded.
//     When withRaster wants nothing, the raster pass over every region is raced directly
//     (raceRegions). The race internals (overhangFaces, heldBy, nearRegion, keepers,
//     planGap, halfFoot, gripZ) are dumped too.
// buildPass and the internals are not exported upstream: a module loader hook appends
// `export { ... }` lines to prop.js / attached.js / raster.js (nothing else changes).
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/attached_ref.mjs [outDir]
// outDir defaults to refs_attached/ next to this tools/ directory.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { register } from 'node:module';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_attached');
mkdirSync(outDir, { recursive: true });

// ---- expose the internals: append export lines to three upstream modules ----------
const HOOK = `
const ADD = {
  '/web/prop.js': 'export { buildPass as __buildPass };',
  '/web/prop/attached.js': 'export { buriedColumn as __buriedColumn, enclosedColumn as __enclosedColumn, clearBetween as __clearBetween };',
  '/web/prop/raster.js': 'export { overhangFaces as __overhangFaces, heldBy as __heldBy, wallTops as __wallTops, halfFoot as __halfFoot, gripZ as __gripZ, planGap as __planGap, rasterWanted as __rasterWanted, nearRegion as __nearRegion, keepers as __keepers, raceRegions as __raceRegions };',
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
const propjs = await imp('web/prop.js');
const surface = await imp('web/prop/surface.js');
const contact = await imp('web/prop/contact.js');
const mold = await imp('web/prop/mold.js');
const crest = await imp('web/prop/crest.js');
const tracks = await imp('web/prop/tracks.js');
const tines = await imp('web/prop/tines.js');
const CL = await imp('web/prop/clearance.js');
const AT = await imp('web/prop/attached.js');
const RA = await imp('web/prop/raster.js');
const { findWallPatches } = await imp('web/planes.js');
const { FIN } = await imp('web/fins/config.js');
const { PERP, wedgeVeto } = await imp('web/fins/wedges.js');
const { buildTopology, analyze, DEFAULT_THRESHOLD, IDENTITY3, MIN_REGION_AREA } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');
if (!propjs.__buildPass || !AT.__clearBetween || !RA.__raceRegions) throw new Error('loader hook did not expose the internals');

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

// ---- helpers -------------------------------------------------------------------------
const PROP0 = { ...PROP };
const flat = (tris) => { const a = []; for (const v of tris) a.push(v[0], v[1], v[2]); return a; };
const tagStr = (tris) => tris.map((v) => (v[3] === 1 ? '1' : '0')).join('');
const copyLine = (l) => l.map((p) => [p[0], p[1], p[2]]);
const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);
const capArr = (c) => (c ? c.map((t) => [t.x, t.y, t.z, t.biteX, t.biteY]) : null);
// fins.js coverPitch (not exported)
const coverPitch = (coverage) => {
  const c = Math.max(0, Math.min(1, coverage));
  return c <= 0.5
    ? FIN.coverSparse + ((0.5 - c) / 0.5) * (FIN.coverExtraSparse - FIN.coverSparse)
    : FIN.coverSparse - ((c - 0.5) / 0.5) * (FIN.coverSparse - FIN.coverDense);
};
// the build's out before a call: one earlier triangle pair, the second tagged
const PREFIX = () => { const o = [[5, 5, 0], [6, 5, 0], [5, 6, 0], [7, 5, 0], [8, 5, 0], [7, 6, 0]]; crest.markInterface(o, 3); return o; };

// A prop record as JSON: every key prop.js / raceRegions write (undefined keys dropped).
const propOf = (q) => {
  const o = {};
  for (const k of ['region', 'tines', 'caps', 'grip', 'span', 'height', 'area', 'stations', 'trimmed', 'volume',
                   'line', 'id', 'kind', 'triRanges', 'partAttached', 'inBore', 'squat', 'triRange', 'raster']) {
    if (q[k] !== undefined) o[k] = q[k];
  }
  const extra = Object.keys(q).filter((k) => !(k in o) && q[k] !== undefined);
  if (extra.length) throw new Error(`prop record has unexpected keys ${extra}`);
  return o;
};
// A buildProps result as JSON. `refs`: [[src, start, len], ...] instead of the triangles,
// for a raced result whose vertices are the very objects of the two passes.
function resultOf(r, srcs = null) {
  const o = {
    props: r.props.map(propOf), skipped: r.skipped, served: r.served, servedRegions: r.servedRegions,
    tines: r.tines, sagRisk: r.sagRisk, volume: r.volume, tags: tagStr(r.triangles), nTri: r.triangles.length,
  };
  if (r.sagRegions) o.sagRegions = [...r.sagRegions];
  if (r.rasterRegions !== undefined) o.rasterRegions = r.rasterRegions;
  const known = ['triangles', 'props', 'skipped', 'served', 'servedRegions', 'tines', 'sagRisk', 'volume', 'sagRegions', 'rasterRegions'];
  const extra = Object.keys(r).filter((k) => !known.includes(k));
  if (extra.length) throw new Error(`result has unexpected keys ${extra}`);
  if (!srcs) { o.triangles = flat(r.triangles); return o; }
  const maps = srcs.map((s) => { const m = new Map(); s.forEach((v, i) => { if (!m.has(v)) m.set(v, i); }); return m; });
  const runs = [];
  for (const v of r.triangles) {
    let src = -1, idx = -1;
    for (let s = 0; s < maps.length && src < 0; s++) if (maps[s].has(v)) { src = s; idx = maps[s].get(v); }
    if (src < 0) throw new Error('raced vertex from neither pass');
    const last = runs[runs.length - 1];
    if (last && last[0] === src && last[1] + last[2] === idx) last[2]++;
    else runs.push([src, idx, 1]);
  }
  o.refs = runs;
  return o;
}

// ---- configs -------------------------------------------------------------------------
// coverage -> rowSpan (coverRowSpan); tines; layerHeight (crestPart and the tine height);
// veto: 'wedge' = fins.js's wedgeVeto as opts.rasterVeto (auto mode), 'odd' = a stand-in
// that vetoes every odd-id raster wall (wedgeVeto never fires on these scenes); cap: globalThis.__TINECAP on (with two
// earlier entries in it, so capFrom = 2).
const CFGS = [
  { prop: {}, coverage: 0.5, tines: false, layerHeight: null, veto: false, cap: false },
  { prop: { iface: 'all' }, coverage: 0, tines: true, layerHeight: null, veto: 'wedge', cap: true },
  { prop: { gap: 0.3, footGap: 0 }, coverage: 1, tines: true, layerHeight: 0.16, veto: 'wedge', cap: true },
  { prop: { gap: 0.3, iface: 'all' }, coverage: 0.5, tines: true, layerHeight: 0.12, veto: false, cap: true },
  { prop: { footGap: 0 }, coverage: 1, tines: true, layerHeight: null, veto: 'odd', cap: true },
  { prop: { iface: 'all', footGap: 0, gap: 0.3 }, coverage: 0, tines: false, layerHeight: null, veto: 'wedge', cap: false },
];
const CAP0 = [{ x: 1, y: 2, z: 3, biteX: 1, biteY: 0 }, { x: -1, y: 0.5, z: 0.25, biteX: 0, biteY: -1 }];

// None of the stress scenes the port has used so far puts part under an overhang at
// these poses (buildPartAttached declines on every line), so part-attached territory
// gets its own: stress models posed with a floor under the overhang (the inside of a
// lying torus / tube, a portal on its side, ...) and generated tunnels (syn:*) -- a
// narrow one whose walls read inBore, a wider one, and a low one that floors 'stub'.
const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'], ['staircase', '40,0,0'],
  ['sphere', '0,0,0'], ['tube', '40,0,0'], ['cylinder', '40,0,0'], ['plate', '40,0,0'], ['ramp', '40,0,0'],
  ['lowledge', '0,0,0'], ['lowledge', '40,0,0'], ['coin', '40,0,0'], ['plus', '40,0,0'], ['tshape', '40,0,0'],
  ['tshape', '60,0,0'], ['portal', '40,0,0'], ['ushape', '40,0,0'], ['arch', '40,0,0'],
  ['torus', '90,0,0'], ['tube', '90,0,0'], ['tube', '75,0,0'], ['portal', '90,0,0'], ['portal', '60,0,0'],
  ['cylinder', '90,0,0'], ['ushape', '0,90,0'],
  ['syn:tunnel6', '0,0,0'], ['syn:tunnel6', '0,0,35'], ['syn:tunnel10', '10,0,0'], ['syn:tunnel10', '0,0,0'],
  ['syn:lowslot', '0,0,0'], ['syn:midslot', '0,0,0'], ['syn:slit', '0,0,0'], ['syn:slit', '0,0,20'],
];

// A W x D x H box with a tunnel x in [a, b], z in [c, d] through it along y: closed,
// outward-facing, no T-junctions (each holed face is four trapezoids).
function tunnelBox(W, D, H, a, b, c, d) {
  const out = [];
  const tri = (p, q, r, n) => {
    const ux = q[0] - p[0], uy = q[1] - p[1], uz = q[2] - p[2], vx = r[0] - p[0], vy = r[1] - p[1], vz = r[2] - p[2];
    const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
    if (cx * n[0] + cy * n[1] + cz * n[2] < 0) [q, r] = [r, q];
    out.push(...p, ...q, ...r);
  };
  const quad = (p, q, r, s, n) => { tri(p, q, r, n); tri(p, r, s, n); };
  quad([0, 0, 0], [W, 0, 0], [W, D, 0], [0, D, 0], [0, 0, -1]);
  quad([0, 0, H], [W, 0, H], [W, D, H], [0, D, H], [0, 0, 1]);
  quad([0, 0, 0], [0, D, 0], [0, D, H], [0, 0, H], [-1, 0, 0]);
  quad([W, 0, 0], [W, D, 0], [W, D, H], [W, 0, H], [1, 0, 0]);
  for (const [y, ny] of [[0, -1], [D, 1]]) {
    const n = [0, ny, 0], P = (x, z) => [x, y, z];
    quad(P(0, 0), P(W, 0), P(b, c), P(a, c), n);
    quad(P(W, 0), P(W, H), P(b, d), P(b, c), n);
    quad(P(W, H), P(0, H), P(a, d), P(b, d), n);
    quad(P(0, H), P(0, 0), P(a, c), P(a, d), n);
  }
  quad([a, 0, d], [b, 0, d], [b, D, d], [a, D, d], [0, 0, -1]);   // the ceiling: the overhang
  quad([a, 0, c], [b, 0, c], [b, D, c], [a, D, c], [0, 0, 1]);    // the floor under it
  quad([a, 0, c], [a, D, c], [a, D, d], [a, 0, d], [1, 0, 0]);
  quad([b, 0, c], [b, D, c], [b, D, d], [b, 0, d], [-1, 0, 0]);
  return Float64Array.from(out);
}
const SYN = {
  tunnel6: () => tunnelBox(30, 40, 20, 12, 18, 4, 10),
  tunnel10: () => tunnelBox(36, 30, 24, 8, 26, 3, 13),
  lowslot: () => tunnelBox(40, 30, 12, 6, 34, 2, 3.4),
  slit: () => tunnelBox(30, 40, 20, 14.3, 15.7, 4, 10),
  midslot: () => tunnelBox(40, 30, 12, 6, 34, 2, 4.5),
};
const only = process.env.SCENE ? process.env.SCENE.split(';') : null;
const tot = { attached: 0, ok: 0, floored: {}, none: 0, inBore: 0, rasterTracks: 0, swaps: 0, races: 0, raced: 0, vetoed: 0 };

for (const [model, rotStr] of SCENES) {
  const name = `${model.replace('syn:', 'syn_')}-${rotStr.replace(/,/g, '_')}`;
  if (only && !only.includes(name)) continue;
  const t0 = Date.now();
  const raw = model.startsWith('syn:') ? SYN[model.slice(4)]() : readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
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
  const result = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);
  const rot = IDENTITY3, off = result.offset;
  const v = [0, 0, 0];

  // prop.js buildPass: the seated part, then per region its tris / pts
  const partTris = new Float64Array(pos.length);
  for (let f = 0; f < topo.nFaces; f++) for (let i = 0; i < 3; i++) {
    surface.seat(pos, f * 9 + i * 3, rot, off, v);
    partTris[f * 9 + i * 3] = v[0]; partTris[f * 9 + i * 3 + 1] = v[1]; partTris[f * 9 + i * 3 + 2] = v[2];
  }
  const regions = result.regions.map((g) => {
    const rFaces = g.faces;
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
    return { rFaces, regionTris, regionPts, regionArea };
  });

  const cases = [];
  for (let ci = 0; ci < CFGS.length; ci++) {
    const cfg = CFGS[ci];
    Object.assign(PROP, PROP0, cfg.prop);
    crest.crestPart(crest.crestOn() ? partTris : null, cfg.layerHeight ?? undefined);
    const step = PROP.stationStep;
    const rowSpan = propjs.coverRowSpan(cfg.coverage);
    const support = { topo, rot, offset: off };
    const tineStepEff = tines.tineStepFor(undefined);
    const tineHeight = cfg.layerHeight ?? PROP.tineH;

    // ---- the lines: normal pass (tube / split patches) and raster (rasterTracks) ----
    const lines = [];   // { src, region, line }
    for (let ri = 0; ri < regions.length; ri++) {
      const { rFaces, regionTris, regionPts, regionArea } = regions[ri];
      const tube = tracks.tubeLine(topo, rFaces, rot, regionPts, regionTris, step);
      const small = regionArea < PROP.tubeMinArea;
      const split = small || !tube?.length ? tracks.splitRegion(topo, rFaces, rot) : null;
      const patches = [];
      if (tube && tube.length) patches.push({ lines: tube });
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
          ls = tracks.patchTracks(pts, patchTris, step, support, rowSpan, patch.axis ?? null);
        }
        for (const l of ls) lines.push({ src: 'normal', region: ri, line: copyLine(l) });
      }
      // raster: every region (buildPass gives a lattice net to strutPatches instead)
      const rl = RA.rasterTracks(regionPts, regionTris, partTris, step, rowSpan, support);
      cases.push({ fn: 'rasterTracks', cfg: ci, region: ri, step, rowSpan, lines: rl.map(copyLine),
                   spacing: rl.spacing === undefined ? null : rl.spacing });
      tot.rasterTracks++;
      for (const l of rl) {
        lines.push({ src: 'raster', region: ri, line: copyLine(l) });
        // rasterRest over runs: the whole line, a middle run, a short head, a short tail
        const n = l.length, m = PROP.minStations;
        const runs = [[0, n], [m, n - m], [1, n], [0, n - 1], [Math.floor(n / 2), Math.floor(n / 2) + 1],
                      [m - 1, n - m + 1], [m + 1, n - m - 1]].filter(([a, b]) => a >= 0 && b <= n && a <= b);
        cases.push({ fn: 'rasterRest', cfg: ci, line: copyLine(l),
                     runs: runs.map((r) => ({ run: r, rest: RA.rasterRest(l, r).map((p) => ({ from: p.from, line: copyLine(p) })) })) });
      }
    }

    // ---- buildPartAttached on every line, prop.js's tine glue after it ----
    for (const L of lines) {
      const out = PREFIX(), from = out.length;
      const inp = copyLine(L.line);
      const pa = AT.buildPartAttached(L.line, partTris, topo, rot, off, out);
      if (JSON.stringify(inp) !== JSON.stringify(L.line)) throw new Error('buildPartAttached mutated its line');
      const c = { fn: 'attached', cfg: ci, src: L.src, region: L.region, line: inp, from,
                  ok: !!pa.ok, floored: pa.floored ?? null, empty: !pa.ok && !pa.floored, prop: pa.prop ?? null };
      tot.attached++;
      if (pa.ok) { tot.ok++; if (pa.prop.inBore) tot.inBore++; }
      else if (pa.floored) tot.floored[pa.floored] = (tot.floored[pa.floored] ?? 0) + 1;
      else tot.none++;
      c.wallEnd = out.length;
      if (pa.ok && cfg.tines) {
        const grip = { z: Infinity };
        const topLine = pa.prop.line.map((p) => [p[0], p[1], p[2] + PROP.gap]);
        c.nT = tines.emitTines(topLine, partTris, topo, rot, off, out, tineStepEff, undefined, tineHeight, null, grip);
        c.grip = grip.z;
      }
      c.out = flat(out.slice(from));
      c.tags = tagStr(out.slice(from));
      // the internals on the settled top, as buildPartAttached reads it
      const top = copyLine(L.line);
      contact.contourTop(top, partTris, AT.PART_BAND);
      contact.lowerSag(top, partTris, AT.PART_BAND);
      contact.settleTop(top, partTris, 0.25, AT.PART_BAND);
      const floor = mold.floorLine(top, partTris);
      c.top = copyLine(top);
      c.floor = floor.map((f) => [...f]);
      if (floor.some((f) => f[2] > PROP.gap + 0.5)) {
        c.buried = top.map((_, k) => AT.__buriedColumn(top, floor, k, topo, rot, off));
        c.clear = top.map((_, k) => AT.__clearBetween(top, floor, k, topo, rot, off));
        c.enclosed = top.map((_, k) => AT.__enclosedColumn(top, floor, k, topo, rot, off));
      }
      cases.push(c);
    }

    // ---- the race ----
    globalThis.__TINECAP = cfg.cap ? CAP0.map((t) => ({ ...t })) : undefined;
    const capFrom = globalThis.__TINECAP?.length ?? 0;
    const opts = { coverage: cfg.coverage, tines: cfg.tines, ...(cfg.layerHeight !== null ? { layerHeight: cfg.layerHeight } : {}) };
    const normal = propjs.__buildPass(topo, result, rot, opts, null);
    const capBefore = capArr(globalThis.__TINECAP);
    let vetoCalls = null;
    if (cfg.veto) {
      const wedgeable = findWallPatches(topo, rot, off).filter((p) =>
        p.n.z < -0.05 && p.area >= PERP.minArea && (p.u1 - p.u0) >= PERP.minWidth);
      const veto = cfg.veto === 'odd' ? (q) => q.id % 2 === 1
        : wedgeVeto(wedgeable, topo, rot, off, { tines: cfg.tines, pitch: coverPitch(cfg.coverage), tineDensity: undefined });
      vetoCalls = [];
      opts.rasterVeto = (q, np) => {
        const cap = globalThis.__TINECAP, l0 = cap?.length ?? 0;
        if (np !== normal.props) throw new Error('veto not handed normal.props');
        const d = veto(q, np);
        vetoCalls.push({ id: q.id, region: q.region, d, caps: cap ? capArr(cap.slice(l0)) : null });
        if (d) tot.vetoed++;
        return d;
      };
    }
    let rec = null;
    const raced = RA.withRaster(topo, result, rot, opts, normal, capFrom, (regions) => {
      const r = propjs.__buildPass(topo, result, rot, opts, regions);
      if (rec) throw new Error('rasterPass called twice');
      rec = { want: [...regions], res: r, props: r.props, caps: capArr(globalThis.__TINECAP) };
      return r;
    });
    const race = { fn: 'race', cfg: ci, capFrom, capBefore, normal: resultOf(normal), want: rec ? rec.want : [],
                   vetoCalls: rec ? vetoCalls : null, same: raced === normal, capAfter: capArr(globalThis.__TINECAP) };
    tot.races++;
    let b;
    if (rec) {
      race.raster = resultOf({ ...rec.res, props: rec.props });
      race.rasterCaps = rec.caps;
      b = rec.res;   // its props are the vetoed ones now, as raceRegions saw them
    } else {
      // nothing wanted: race the raster pass over every region directly
      const all = new Set(result.regions.map((_, i) => i));
      const keep = globalThis.__TINECAP;
      if (keep) globalThis.__TINECAP = [];
      b = propjs.__buildPass(topo, result, rot, opts, all);
      globalThis.__TINECAP = keep;
      race.rasterAll = resultOf(b);
      const direct = RA.__raceRegions(RA.__overhangFaces(topo, result, rot), result, normal, b);
      race.direct = resultOf(direct, [normal.triangles, b.triangles]);
    }
    if (!race.same) { race.raced = resultOf(raced, [normal.triangles, b.triangles]); tot.swaps++; }
    tot.raced++;
    // the internals over (normal, b)
    const faces = RA.__overhangFaces(topo, result, rot);
    race.faces = faces;
    race.wanted = [...RA.__rasterWanted(faces, result, normal)];
    const per = [];
    for (let ri = 0; ri < result.regions.length; ri++) {
      const qa = normal.props.filter((q) => q.region === ri), qb = b.props.filter((q) => q.region === ri);
      const near = RA.__nearRegion(faces, ri);
      const e = { ri, near: near.length, nearHeld: RA.__heldBy(near, RA.__wallTops(normal.props)),
                  ha: RA.__heldBy(near, RA.__wallTops(qa)), hb: RA.__heldBy(near, RA.__wallTops(qb)) };
      if (qb.length) {
        const k = RA.__keepers(qa, qb, near);
        e.keepers = k ? k.map((q) => qa.indexOf(q)) : null;
      }
      per.push(e);
    }
    race.regions = per;
    const pa = normal.props.slice(0, 30), pb = b.props.slice(0, 30);
    race.planGap = pa.map((q) => pb.map((r) => RA.__planGap(q.line ?? [], r.line ?? [])));
    race.halfFoot = [pa.map((q) => RA.__halfFoot(q)), pb.map((q) => RA.__halfFoot(q))];
    race.gripZ = [pa.map((q) => RA.__gripZ([q])), RA.__gripZ(normal.props), RA.__gripZ(b.props), RA.__gripZ([])];
    race.gripRise = [0, 0.5, 2, 40, Infinity].map((z) => RA.gripRise(z));
    cases.push(race);
    globalThis.__TINECAP = undefined;
    Object.assign(PROP, PROP0);
    crest.crestPart(null);
  }

  writeFileSync(path.join(outDir, `${name}.json`), JSON.stringify({
    scene: name, model, rot: rotStr, threshold: DEFAULT_THRESHOLD, pos: Array.from(pos), offset: off,
    cfgs: CFGS, cap0: capArr(CAP0), PROP: PROP0, nRegions: result.regions.length,
    consts: { PART_BAND: AT.PART_BAND, RACE_REL: RA.RACE_REL, RACE_ABS: RA.RACE_ABS, RASTER_SKIP_HELD: RA.RASTER_SKIP_HELD,
              GRIP_RISE: RA.GRIP_RISE, GRIP_RISE_REL: RA.GRIP_RISE_REL },
    cases }, replacer));
  const att = cases.filter((c) => c.fn === 'attached');
  const races = cases.filter((c) => c.fn === 'race');
  console.log(name, `regions ${result.regions.length}, attached ${att.length} (ok ${att.filter((c) => c.ok).length},`,
              `floored ${att.filter((c) => c.floored).length}), swaps ${races.filter((c) => !c.same).length}/${races.length},`,
              `wanted ${races.filter((c) => c.want.length).length}, ${((Date.now() - t0) / 1000).toFixed(1)}s`);
}
console.log(JSON.stringify(tot));
