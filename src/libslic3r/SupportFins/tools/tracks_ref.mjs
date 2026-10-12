#!/usr/bin/env node
// Reference oracle for Engine/Prop/Tracks.*, Lattice.*, Rival.* (web/prop/tracks.js,
// lattice.js, rival.js). Poses each stress model like tools/ref.mjs, seats + analyzes it
// exactly as plugins/shared/engine/fins_entry.js computeFins does (seatSoup,
// buildTopology, analyze(..., IDENTITY3)), then drives the three modules the way
// web/prop.js buildPass drives them for every region, and dumps everything as JSON for
// tests/test_tracks.cpp.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/tracks_ref.mjs [outDir]
//
// outDir defaults to refs_tracks/ next to tools/. Per region:
//   splitRegion, tubeLine (step 1 and 0.7), latticeStruts (+ strutPatches),
//   patchTracks on every split patch / the whole region (rasterTracks' call) / every
//   non-tube strut, at coverage 0, 0.5, 1 (coverRowSpan), with the part as support,
//   plus once with no support.
// tracks.js's module-internal helpers (rowOffsets, lowEdges, freeEdge, convexAbout,
// edgePoints) are captured from an instrumented copy of tracks.js (imports rewritten
// to the real modules, each helper wrapped to record its arguments and result) run on
// the same inputs; the copy's exports must agree with the real module's.
// And rival.js: buildPass's patch loop (normal pass, and a raster-slot pass with the
// lattice struts; and 'weak', the normal pass with every small tube's walls refused,
// so the patch path wins) with a deterministic stand-in wall builder, through tubeRivalry.
// plateOnly only changes buildPartAttached's outcome, never these modules' inputs, so
// it is not a dimension here.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_tracks');
mkdirSync(outDir, { recursive: true });
const url = (p) => pathToFileURL(path.join(SF, p)).href;
const imp = (p) => import(url(p));

const { PROP } = await imp('web/prop/config.js');
const tracks = await imp('web/prop/tracks.js');
const lattice = await imp('web/prop/lattice.js');
const { tubeRivalry } = await imp('web/prop/rival.js');
const { seat } = await imp('web/prop/surface.js');
const { coverRowSpan } = await imp('web/prop.js');
const { buildTopology, analyze, IDENTITY3, MIN_REGION_AREA } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');

// ---- instrumented copy of tracks.js -----------------------------------------------
const HELPERS = ['edgePoints', 'convexAbout', 'rowOffsets', 'lowEdges', 'freeEdge'];
let src = readFileSync(path.join(SF, 'web/prop/tracks.js'), 'utf8');
src = src.replace("'../inside.js'", `'${url('web/inside.js')}'`)
  .replace("'../planes.js'", `'${url('web/planes.js')}'`)
  .replace("'./config.js'", `'${url('web/prop/config.js')}'`)
  .replace("'./contact.js'", `'${url('web/prop/contact.js')}'`)
  .replace("'./surface.js'", `'${url('web/prop/surface.js')}'`);
for (const h of HELPERS) {
  if (!src.includes(`function ${h}(`)) throw new Error(`tracks.js has no ${h}`);
  src = src.replace(`function ${h}(`, `function ${h}__orig(`);
  src += `\nfunction ${h}(...a) { const r = ${h}__orig(...a); globalThis.__TRK_REC?.('${h}', a, r); return r; }\n`;
}
const tracksI = await import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));

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

// ---- synthetic parts (tests/lattice.test.js, tests/small_tubes.test.js) -----------
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
const rotX = (d) => { const r = (d * Math.PI) / 180, c = Math.cos(r), s = Math.sin(r); return [1, 0, 0, 0, c, s, 0, -s, c]; };
const quad = (a, b, c, d) => [a, b, c, a, c, d];
function block(x0, x1, y0, y1, z0, z1) {
  const v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
             [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
  return [...quad(v[0], v[3], v[2], v[1]), ...quad(v[4], v[5], v[6], v[7]), ...quad(v[0], v[1], v[5], v[4]),
          ...quad(v[2], v[3], v[7], v[6]), ...quad(v[1], v[2], v[6], v[5]), ...quad(v[0], v[4], v[7], v[3])];
}
function cylinderY(cx, cz, r, y0, y1, n = 48) {
  const ring = (y) => Array.from({ length: n }, (_, i) => {
    const a = (2 * Math.PI * i) / n;
    return [cx + r * Math.cos(a), y, cz + r * Math.sin(a)];
  });
  const A = ring(y0), B = ring(y1), t = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    t.push(A[i], B[j], A[j], A[i], B[i], B[j]);
    t.push([cx, y0, cz], A[i], A[j]);
    t.push([cx, y1, cz], B[j], B[i]);
  }
  return t;
}
const f32 = (tris) => Float64Array.from(Float32Array.from(tris.flat()));
const pegPos = () => f32([...block(-15, 15, -10, 10, 0, 24), ...cylinderY(0, 14, 2.5, 9, 18)]);
const fatPegPos = () => f32([...block(-25, 25, -10, 10, 0, 60), ...cylinderY(0, 30, 8, 9, 23)]);
// several regions: a ledge and a flat cantilever first, the small tubes after, so the
// race's restore/shift runs with other regions' walls in between
const pegsPos = () => f32([...block(-25, 25, -10, 10, 0, 60), ...block(-25, -5, -30, -10, 40, 44),
                           ...cylinderY(0, 30, 8, 9, 23), ...cylinderY(15, 12, 2.5, 9, 18),
                           ...block(5, 25, -10, 2, 50, 53).map(([x, y, z]) => [x, y - 22, z])]);

// ---- scenes ------------------------------------------------------------------------
const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['lbracket', '35,0,0'], ['torus', '30,0,0'],
  ['staircase', '40,0,0'], ['sphere', '0,0,0'], ['tube', '40,0,0'], ['cylinder', '40,0,0'],
  ['plate', '40,0,0'], ['ramp', '40,0,0'], ['lowledge', '0,0,0'], ['lowledge', '40,0,0'],
  ['coin', '40,0,0'], ['plus', '40,0,0'], ['needle', '40,0,0'], ['ushape', '40,0,0'],
  // synthetic: [name, pos(), rot] -- the part is analyzed under `rot` directly
  ['net', netPos, rotX(20)], ['net', netPos, rotX(0)], ['peg', pegPos, IDENTITY3], ['fatpeg', fatPegPos, IDENTITY3],
  ['pegs', pegsPos, IDENTITY3], ['pegs', pegsPos, rotX(10)],
];

const PROP0 = { ...PROP };
const COVERAGES = [0, 0.5, 1];
const MAX_REC = 4000;       // per helper per scene
const line3 = (l) => l.map((p) => [p[0], p[1], p[2]]);
const linesOf = (ls) => ls.map(line3);

for (const [model, rotOrPos, rotSyn] of SCENES) {
  Object.assign(PROP, PROP0);
  let pos, rot, name, rotStr;
  if (typeof rotOrPos === 'string') {
    rotStr = rotOrPos;
    const raw = readSTL(path.join(SF, 'prototype/stress/models', `${model}.stl`));
    const pose = sitePose(rotStr.split(',').map(Number));
    const posed = new Float64Array(raw.length);
    for (let i = 0; i < raw.length; i += 3) {
      const x = raw[i], y = raw[i + 1], z = raw[i + 2];
      posed[i] = pose[0] * x + pose[3] * y + pose[6] * z;
      posed[i + 1] = pose[1] * x + pose[4] * y + pose[7] * z;
      posed[i + 2] = pose[2] * x + pose[5] * y + pose[8] * z;
    }
    pos = seatSoup(posed).pos;
    rot = IDENTITY3;
    name = `${model}-${rotStr.replace(/,/g, '_')}`;
  } else {
    pos = rotOrPos();
    rot = rotSyn;
    rotStr = 'synthetic';
    const deg = Math.round((Math.atan2(rot[5], rot[4]) * 180) / Math.PI);
    name = `${model}-x${deg}`;
  }
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, 45, rot);
  const off = result.offset;
  const support = { topo, rot, offset: off };
  const step = PROP.stationStep;
  const v = [0, 0, 0];

  // ids for the arrays the C++ test rebuilds itself (region / patch / strut tris + pts)
  const ids = new Map();
  const idOf = (a) => (a == null ? null : ids.get(a) ?? '?');
  const rec = { edgePoints: [], convexAbout: [], rowOffsets: [], lowEdges: [], freeEdge: [] };
  const probeOf = (p) => (p ? { tris: idOf(p.patchTris), ux: p.ux, uy: p.uy, vx: p.vx, vy: p.vy, uLo: p.uLo, uHi: p.uHi } : null);
  globalThis.__TRK_REC = (fn, a, r) => {
    const list = rec[fn];
    if (list.length >= MAX_REC) return;
    if (fn === 'edgePoints') {
      list.push({ tris: idOf(a[0]), pts: idOf(a[1]), gap: a[2], out: r.flat() });
    } else if (fn === 'convexAbout') {
      list.push({ rn: a[0].map(([n, ar]) => [n[0], n[1], n[2], ar]), tris: idOf(a[1]),
                  cx: a[2], cy: a[3], ux: a[4], uy: a[5], out: r });
    } else if (fn === 'rowOffsets') {
      list.push({ vLo: a[0], vHi: a[1], rowSpan: a[2], probe: probeOf(a[3]), rows: [...r], spacing: r.spacing });
    } else if (fn === 'lowEdges') {
      const [vLo, vHi] = [a[1], a[2]];
      const probesAt = [vLo - 1, vLo, (vLo + vHi) / 2, vHi, vHi + 3, ...r.map((e) => e.v)];
      list.push({ probe: probeOf(a[0]), vLo, vHi, edges: r.map((e) => [e.v, e.dir]),
                  low: probesAt.map((x) => [x, r.low(x)]) });
    } else if (fn === 'freeEdge') {
      list.push({ probe: probeOf(a[0]), vEdge: a[1], dir: a[2], out: r });
    }
  };
  const same = (a, b, what) => {
    const ja = JSON.stringify(a), jb = JSON.stringify(b);
    if (ja !== jb) throw new Error(`${name}: instrumented copy disagrees on ${what}`);
  };

  const cases = [];
  const regionsOut = result.regions.map((g) => ({ faces: [...g.faces], area: g.area }));
  const rowSpans = COVERAGES.map((c) => coverRowSpan(c));

  // seat one face list as prop.js does: tris + pts (3 vertices + centroid per face)
  const seatFaces = (faces) => {
    const tris = new Float64Array(faces.length * 9), pts = [];
    for (let k = 0; k < faces.length; k++) {
      let gx = 0, gy = 0, gz = 0;
      for (let i = 0; i < 3; i++) {
        seat(pos, faces[k] * 9 + i * 3, rot, off, v);
        tris[k * 9 + i * 3] = v[0]; tris[k * 9 + i * 3 + 1] = v[1]; tris[k * 9 + i * 3 + 2] = v[2];
        pts.push([v[0], v[1], v[2]]);
        gx += v[0]; gy += v[1]; gz += v[2];
      }
      pts.push([gx / 3, gy / 3, gz / 3]);
    }
    return { tris, pts };
  };
  const seated = (f) => [0, 1, 2].map((i) => seat(pos, f * 9 + i * 3, rot, off, [0, 0, 0]));
  const dumpPatch = (p) => ({
    faces: [...p.faces], area: p.area, region: p.region ?? null, strut: p.strut === true,
    axis: p.axis ?? null, lines: p.lines ? linesOf(p.lines) : null,
    reach: p.reach ? Array.from(p.reach) : null, tris: p.tris ? idOf(p.tris) : null,
    smallTube: p.smallTube ? p.smallTube.map(dumpPatch) : null,
  });
  const tracksCase = (o, pts, tris, sup, span, axis) => {
    const out = tracks.patchTracks(pts, tris, step, sup, span, axis);
    same(out, tracksI.patchTracks(pts, tris, step, sup, span, axis), 'patchTracks');
    same(out.spacing, tracksI.patchTracks(pts, tris, step, sup, span, axis).spacing, 'patchTracks.spacing');
    cases.push({ fn: 'patchTracks', ...o, step, support: !!sup, span, axis, out: linesOf(out), spacing: out.spacing ?? null });
  };

  for (let ri = 0; ri < result.regions.length; ri++) {
    const rFaces = result.regions[ri].faces;
    const { tris: regionTris, pts: regionPts } = seatFaces(rFaces);
    ids.set(regionTris, `r${ri}`); ids.set(regionPts, `r${ri}`);

    // splitRegion
    const split = tracks.splitRegion(topo, rFaces, rot);
    cases.push({ fn: 'splitRegion', region: ri, out: split.map((p) => ({ faces: p.faces, area: p.area })) });

    // tubeLine, as buildPass calls it (and at a finer step)
    for (const st of [step, 0.7]) {
      const tube = tracks.tubeLine(topo, rFaces, rot, regionPts, regionTris, st);
      same(tube, tracksI.tubeLine(topo, rFaces, rot, regionPts, regionTris, st), 'tubeLine');
      cases.push({ fn: 'tubeLine', region: ri, step: st, out: tube ? linesOf(tube) : null });
    }

    // patchTracks on every split patch (buildPass's patch path; slivers too)
    split.forEach((p, k) => {
      const { tris, pts } = seatFaces(p.faces);
      ids.set(tris, `r${ri}p${k}`); ids.set(pts, `r${ri}p${k}`);
      COVERAGES.forEach((c, ci) => tracksCase({ src: 'patch', region: ri, patch: k, coverage: c }, pts, tris, support, rowSpans[ci], null));
      tracksCase({ src: 'patch', region: ri, patch: k, coverage: null }, pts, tris, null, PROP.maxUnsupportedSpan, null);
    });
    // the whole region (rasterTracks' call)
    COVERAGES.forEach((c, ci) => tracksCase({ src: 'region', region: ri, patch: -1, coverage: c }, regionPts, regionTris, support, rowSpans[ci], null));

    // lattice
    const struts = lattice.latticeStruts(topo, rFaces, seated);
    cases.push({ fn: 'latticeStruts', region: ri, out: struts ? { groups: struts.map((g) => [...g]), lost: struts.lost } : null });
    if (struts) {
      const sp = lattice.strutPatches(topo, rot, struts, seated, step, ri, regionTris);
      cases.push({ fn: 'strutPatches', region: ri, step, out: sp.map(dumpPatch) });
      sp.forEach((p, k) => {
        if (p.lines) return;
        const { tris, pts } = seatFaces(p.faces);
        ids.set(tris, `r${ri}s${k}`); ids.set(pts, `r${ri}s${k}`);
        COVERAGES.forEach((c, ci) => tracksCase({ src: 'strut', region: ri, patch: k, coverage: c }, pts, tris, support, rowSpans[ci], p.axis));
      });
    }
  }
  globalThis.__TRK_REC = null;

  // ---- rival.js: buildPass's patch loop with a stand-in wall builder ---------------
  const partTris = new Float64Array(pos.length);
  for (let f = 0; f < topo.nFaces; f++) {
    for (let i = 0; i < 3; i++) {
      seat(pos, f * 9 + i * 3, rot, off, v);
      partTris[f * 9 + i * 3] = v[0]; partTris[f * 9 + i * 3 + 1] = v[1]; partTris[f * 9 + i * 3 + 2] = v[2];
    }
  }
  const rivalRuns = [];
  for (const mode of ['normal', 'weak', 'raster']) {
    COVERAGES.forEach((coverage, ci) => {
      const rowSpan = rowSpans[ci];
      const patches = [];
      let slivers0 = 0;
      for (let ri = 0; ri < result.regions.length; ri++) {
        const rFaces = result.regions[ri].faces;
        const { tris: regionTris, pts: regionPts } = seatFaces(rFaces);
        let regionArea = 0;
        for (const f of rFaces) regionArea += topo.area[f];
        if (mode === 'raster') {
          const struts = lattice.latticeStruts(topo, rFaces, seated);
          if (struts) { if (struts.lost) slivers0++; patches.push(...lattice.strutPatches(topo, rot, struts, seated, step, ri, regionTris)); continue; }
          const lines = tracks.patchTracks(regionPts, regionTris, step, support, rowSpan);
          if (lines.length) patches.push({ faces: rFaces, area: regionArea, region: ri, tris: regionTris, lines });
          continue;
        }
        const tube = tracks.tubeLine(topo, rFaces, rot, regionPts, regionTris, step);
        const small = regionArea < PROP.tubeMinArea;
        const split = small || !tube?.length ? tracks.splitRegion(topo, rFaces, rot) : null;
        if (tube && tube.length) {
          patches.push({ faces: rFaces, area: regionArea, region: ri, tris: regionTris, lines: tube, smallTube: small ? split : null });
          continue;
        }
        for (const p of split) {
          if (p.area < MIN_REGION_AREA) { slivers0++; continue; }
          p.region = ri; p.tris = regionTris;
          patches.push(p);
        }
      }
      const out = [], props = [];
      const skipped = { noLine: 0, wanders: 0, stub: 0, blocked: 0, degenerate: 0, buried: 0, weld: 0, sliver: slivers0, onPart: 0 };
      const served = new Set(), tally = { tines: 0, sagRisk: false, sagRegions: new Set() };
      let nextId = 0;
      const rivalry = tubeRivalry({ out, props, skipped, served, tally }, topo, result, partTris);
      const order = [];
      for (const patch of patches) {
        const mark = rivalry.begin(patch);
        order.push([patch.region, patch.faces.length, patch.area, patch.rival ? 1 : 0, patch.smallTube ? 1 : 0]);
        let lines;
        if (patch.lines) lines = patch.lines;
        else {
          const { tris, pts } = seatFaces(patch.faces);
          lines = tracks.patchTracks(pts, tris, step, support, rowSpan, patch.axis ?? null);
        }
        if (!lines.length) { skipped.noLine++; continue; }
        // stand-in walls: one fan of triangles under each line, some refused
        for (let li = 0; li < lines.length; li++) {
          const line = lines[li];
          if (mode === 'weak' && patch.smallTube) { skipped.weld++; continue; }   // the tube always loses
          if (line.length < 4) { skipped.stub++; continue; }
          if ((li + patch.faces.length) % 4 === 3) { skipped.blocked++; continue; }
          const tri0 = out.length;
          for (let k = 0; k + 1 < line.length; k++) {
            const a = line[k], b = line[k + 1];
            const top = [a[0], a[1], a[2] - PROP.gap];
            if (k % 2) top[3] = 1;                    // a crest/tine interface tag
            out.push(top, [b[0], b[1], b[2] - PROP.gap], [a[0], a[1], 0]);
          }
          tally.tines += line.length;
          if (line.length > 25) tally.sagRisk = true;
          served.add(patch.region);
          props.push({ region: patch.region, stations: line.length, id: nextId++, triRanges: [[tri0, out.length]] });
        }
        if (patch.smallTube) rivalry.tubeDone(patch, mark, patches);
      }
      rivalry.finish();
      rivalRuns.push({
        mode, coverage, rowSpan, order,
        out: out.flatMap((p) => [p[0], p[1], p[2]]), tags: out.map((p) => (p[3] === 1 ? 1 : 0)),
        props, skipped, served: [...served], tines: tally.tines, sagRisk: tally.sagRisk,
      });
    });
  }

  for (const [fn, list] of Object.entries(rec)) for (const r of list) cases.push({ fn, ...r });

  const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);
  writeFileSync(path.join(outDir, `${name}.json`), JSON.stringify({
    scene: name, model, rot: rotStr, rotM: rot, pos: Array.from(pos), offset: off, PROP: PROP0,
    MIN_REGION_AREA, rowSpans, coverages: COVERAGES, regions: regionsOut, cases, rival: rivalRuns,
  }, replacer));
  const counts = {};
  for (const c of cases) counts[c.fn] = (counts[c.fn] ?? 0) + 1;
  const kept = rivalRuns.map((r) => r.props.length).join('/');
  console.log(name, 'regions', result.regions.length, JSON.stringify(counts), 'rival props', kept);
}
