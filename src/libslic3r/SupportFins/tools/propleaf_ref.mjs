#!/usr/bin/env node
// Reference oracle for the prop leaf modules (web/prop/surface.js, contact.js, mold.js,
// crest.js, brace.js, ties.js): drives every export with inputs built the way their
// upstream callers build them, on stress models posed like tools/ref.mjs does, and dumps
// inputs + outputs as JSON for tests/test_propleaf.cpp.
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/propleaf_ref.mjs [outDir]
// outDir defaults to refs_propleaf/ next to this tools/ directory.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR || '/home/user/gittrahan/support-fins';
const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_propleaf');
mkdirSync(outDir, { recursive: true });
const imp = (p) => import(pathToFileURL(path.join(SF, p)).href);

const { PROP } = await imp('web/prop/config.js');
const surface = await imp('web/prop/surface.js');
const contact = await imp('web/prop/contact.js');
const mold = await imp('web/prop/mold.js');
const crest = await imp('web/prop/crest.js');
const brace = await imp('web/prop/brace.js');
const ties = await imp('web/prop/ties.js');
const { ribbon } = await imp('web/solids.js');
const { buildTopology, analyze, IDENTITY3 } = await imp('web/overhangs.js');
const { seatSoup } = await imp('plugins/shared/engine/seat.js');
const { seatedPartTris } = await imp('web/fins/seating.js');

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

// ---- PROP state: every case runs from the defaults plus its own overrides ----------
const PROP0 = { ...PROP };
function withProp(over, fn) {
  Object.assign(PROP, PROP0, over);
  try { return fn(); } finally { Object.assign(PROP, PROP0); }
}
// crest.js module state is set through crestPart only; the case records what was set.
const flat = (tris) => { const a = []; for (const v of tris) a.push(v[0], v[1], v[2]); return a; };
const tagsOf = (tris) => tris.map((v) => (v[3] === 1 ? 1 : 0));
const copyLine = (l) => l.map((p) => [p[0], p[1], p[2]]);
const PART_BAND = 3.0;

// ---- scenes ------------------------------------------------------------------------
const SCENES = [
  ['cube', '40,0,0'], ['cube', '20,0,0'], ['lbracket', '35,0,0'], ['sphere', '0,0,0'],
  ['torus', '30,0,0'], ['staircase', '40,0,0'], ['ushape', '30,0,0'], ['portal', '60,0,0'],
  ['tshape', '60,0,0'], ['arch', '30,0,0'], ['ramp', '60,0,0'], ['tube', '40,0,0'],
  ['plus', '60,0,0'], ['lowledge', '0,0,0'], ['lowledge', '30,0,0'], ['cone', '90,0,0'],
  ['coin', '90,0,0'], ['cylinder', '90,0,0'], ['needle', '30,0,0'],
];

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
  const topo = buildTopology({ getAttribute: (k) => (k === 'position' ? { array: pos } : null) });
  const result = analyze(topo, 45, IDENTITY3);
  const rot = IDENTITY3, off = result.offset;
  const partTris = seatedPartTris(topo, rot, off);
  const trisTable = { part: Array.from(partTris) };
  const trisById = { part: partTris };
  const cases = [];

  // seat(): raw pos (the seated soup) through a non-trivial rotation, as prop.js calls it
  {
    const r2 = sitePose([25, 10, 70]), o2 = { x: 1.5, y: -2.25, z: 0.125 };
    for (let f = 0; f < Math.min(topo.nFaces, 6); f++) {
      for (let i = 0; i < 3; i++) {
        const idx = f * 9 + i * 3;
        const out = surface.seat(pos, idx, r2, o2, [0, 0, 0]);
        const outI = surface.seat(pos, idx, rot, off, [0, 0, 0]);
        cases.push({ fn: 'seat', pos: Array.from(pos.slice(f * 9, f * 9 + 9)), i: i * 3, rot: r2, off: o2, out });
        cases.push({ fn: 'seat', pos: Array.from(pos.slice(f * 9, f * 9 + 9)), i: i * 3, rot, off, out: outI });
      }
    }
  }

  // regions, seated as prop.js does: regionTris + regionPts (3 vertices + centroid per face)
  const regions = [];
  const v = [0, 0, 0];
  for (let ri = 0; ri < Math.min(3, result.regions.length); ri++) {
    const rFaces = result.regions[ri].faces;
    const regionTris = new Float64Array(rFaces.length * 9);
    const regionPts = [];
    for (let k = 0; k < rFaces.length; k++) {
      let gx = 0, gy = 0, gz = 0;
      for (let i = 0; i < 3; i++) {
        surface.seat(pos, rFaces[k] * 9 + i * 3, rot, off, v);
        regionTris[k * 9 + i * 3] = v[0];
        regionTris[k * 9 + i * 3 + 1] = v[1];
        regionTris[k * 9 + i * 3 + 2] = v[2];
        regionPts.push([v[0], v[1], v[2]]);
        gx += v[0]; gy += v[1]; gz += v[2];
      }
      regionPts.push([gx / 3, gy / 3, gz / 3]);
    }
    const id = `r${ri}`;
    trisTable[id] = Array.from(regionTris);
    trisById[id] = regionTris;
    regions.push({ id, tris: regionTris, pts: regionPts });
  }

  // ---- surface queries: a grid over each array's bbox (a little past it), plus its
  // vertices and edge midpoints (exactly on edges), plus far outside
  for (const id of Object.keys(trisById)) {
    const t = trisById[id];
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < t.length; i += 3) {
      x0 = Math.min(x0, t[i]); x1 = Math.max(x1, t[i]); y0 = Math.min(y0, t[i + 1]); y1 = Math.max(y1, t[i + 1]);
    }
    const pts = [];
    const N = 17, mx = (x1 - x0) * 0.05 + 0.1, my = (y1 - y0) * 0.05 + 0.1;
    for (let a = 0; a <= N; a++) for (let b = 0; b <= N; b++) {
      pts.push([x0 - mx + (x1 - x0 + 2 * mx) * a / N, y0 - my + (y1 - y0 + 2 * my) * b / N]);
    }
    for (let i = 0; i < Math.min(t.length, 9 * 40); i += 3) pts.push([t[i], t[i + 1]]);
    for (let f = 0; f < Math.min(t.length, 9 * 20); f += 9) pts.push([(t[f] + t[f + 3]) / 2, (t[f + 1] + t[f + 4]) / 2]);
    pts.push([x0 - 50, y0], [x1 + 1e-10, y1], [x0 - 2e-9, (y0 + y1) / 2], [1e6, 1e6]);
    cases.push({
      fn: 'surface', tris: id, pts,
      zAt: pts.map(([x, y]) => surface.surfaceZAt(t, x, y)),
      zs: pts.map(([x, y]) => surface.surfaceZsAt(t, x, y)),
      hits: pts.map(([x, y]) => surface.surfaceHitsAt(t, x, y)),
    });
  }
  // an empty array
  trisTable.empty = []; trisById.empty = new Float64Array(0);
  cases.push({ fn: 'surface', tris: 'empty', pts: [[0, 0]], zAt: [surface.surfaceZAt(trisById.empty, 0, 0)],
               zs: [surface.surfaceZsAt(trisById.empty, 0, 0)], hits: [surface.surfaceHitsAt(trisById.empty, 0, 0)] });

  // ---- straightness: synthetic lines
  const synth = [
    [], [[0, 0, 1]], [[0, 0, 1], [1, 0, 1]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
    [[0, 0, 0], [0, 0, 1], [0, 0, 2]],
    Array.from({ length: 24 }, (_, k) => [10 * Math.cos(k * Math.PI / 12), 10 * Math.sin(k * Math.PI / 12), 3]),
    Array.from({ length: 9 }, (_, k) => [k * 0.7, 0.3 * k + 0.05 * (k % 3), 2 + 0.1 * k]),
  ];
  for (const line of synth) cases.push({ fn: 'straightness', line, out: contact.straightness(line) });

  // ---- contact lines per region, and the passes over them
  const lines = [];   // [x, y, surface z] stations for the downstream passes
  for (const r of regions) {
    for (const nSamples of [3, 8, 24, 60]) {
      const out = withProp({}, () => contact.contactLine(r.pts, r.tris, nSamples));
      cases.push({ fn: 'contactLine', pts: r.pts, tris: r.id, nSamples, out,
                   straight: out ? contact.straightness(out) : null });
      if (out && nSamples === 24) lines.push({ tris: r.id, line: out });
    }
    // with a different tip (contourTop reads PROP.tip)
    const out2 = withProp({ tip: 0.4 }, () => contact.contactLine(r.pts, r.tris, 16));
    cases.push({ fn: 'contactLine', prop: { tip: 0.4 }, pts: r.pts, tris: r.id, nSamples: 16, out: out2,
                 straight: out2 ? contact.straightness(out2) : null });
    // a track the way patchTracks/tubeLine lay one: stations along the principal axis
    // at stationStep, z from surfaceZAt (null splits)
    const pts = r.pts;
    let cx = 0, cy = 0;
    for (const p of pts) { cx += p[0]; cy += p[1]; }
    cx /= pts.length; cy /= pts.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const p of pts) { const dx = p[0] - cx, dy = p[1] - cy; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    const tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const lam = tr / 2 + Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
    let ux = sxy, uy = lam - sxx;
    if (Math.hypot(ux, uy) < 1e-9) { ux = 1; uy = 0; }
    const un = Math.hypot(ux, uy); ux /= un; uy /= un;
    let lo = Infinity, hi = -Infinity;
    for (const p of pts) { const u = (p[0] - cx) * ux + (p[1] - cy) * uy; lo = Math.min(lo, u); hi = Math.max(hi, u); }
    for (const shift of [0, 1.3]) {
      const cur = [];
      const n = Math.max(2, Math.ceil((hi - lo) / PROP.stationStep));
      for (let k = 0; k <= n; k++) {
        const u = lo + ((hi - lo) * k) / n;
        const x = cx + ux * u - uy * shift, y = cy + uy * u + ux * shift;
        const z = surface.surfaceZAt(r.tris, x, y);
        if (z === null) { if (cur.length >= 3) break; cur.length = 0; continue; }
        cur.push([x, y, z]);
      }
      if (cur.length >= 2) lines.push({ tris: r.id, line: cur });
    }
  }
  // the passes: plate path (region tris, no band) and part-attached (part tris, PART_BAND)
  const settled = [];
  for (const { tris: tid, line } of lines) {
    for (const [ttid, band] of [[tid, Infinity], ['part', PART_BAND]]) {
      for (const prop of [{}, { gap: 0.15, tip: 0.5 }]) {
        const t = trisById[ttid];
        for (const lift of [0, 0.6]) {
          const inp = line.map((p) => [p[0], p[1], p[2] + lift]);
          const ct = withProp(prop, () => contact.contourTop(copyLine(inp), t, band));
          const ls = withProp(prop, () => contact.lowerSag(copyLine(inp), t, band));
          const st = withProp(prop, () => contact.settleTop(copyLine(inp), t, 0.25, band));
          const st2 = withProp(prop, () => contact.settleTop(copyLine(inp), t, 0.6, band));
          // the attached.js chain on one copy
          const chain = withProp(prop, () => {
            const top = copyLine(inp);
            contact.contourTop(top, t, band); contact.lowerSag(top, t, band); contact.settleTop(top, t, 0.25, band);
            return top;
          });
          cases.push({ fn: 'passes', prop, tris: ttid, band, line: inp, contourTop: ct, lowerSag: ls,
                       settleTop: st, settleTop06: st2, chain, straight: contact.straightness(inp) });
          if (lift === 0 && Object.keys(prop).length === 0) settled.push({ line: chain, region: tid });
        }
      }
    }
  }

  // ---- floors: floorLine / moldLine over the settled tops against the whole part
  // plus synthetic tops placed high over the part (a floor under them)
  const tops = settled.map((s) => s.line);
  {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, z1 = -Infinity;
    for (let i = 0; i < partTris.length; i += 3) {
      x0 = Math.min(x0, partTris[i]); x1 = Math.max(x1, partTris[i]);
      y0 = Math.min(y0, partTris[i + 1]); y1 = Math.max(y1, partTris[i + 1]); z1 = Math.max(z1, partTris[i + 2]);
    }
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    for (const [ang, zTop, len] of [[0, z1 + 2, x1 - x0], [0.7, z1 * 0.6, (x1 - x0) * 0.8], [1.9, z1 * 0.35, 6]]) {
      const n = Math.max(3, Math.ceil(len));
      const top = [];
      for (let k = 0; k <= n; k++) {
        const u = -len / 2 + (len * k) / n;
        top.push([cx + Math.cos(ang) * u, cy + Math.sin(ang) * u, zTop + 0.05 * k]);
      }
      tops.push(top);
    }
  }
  for (const top of tops) {
    for (const prop of [{}, { footGap: 0 }, { footGap: 0.35, th: 1.2 }]) {
      for (const [margin, moldF] of [[1.0, false], [1.0, true], [0.5, false]]) {
        const fl = withProp(prop, () => mold.floorLine(top, partTris, margin, moldF));
        cases.push({ fn: 'floorLine', prop, top, margin, mold: moldF, out: fl });
      }
      const m = withProp(prop, () => mold.moldLine(copyLine(top), partTris));
      cases.push({ fn: 'moldLine', prop, top, out: m });
    }
  }

  // ---- crest
  // crestCut over a grid of tops / tapers / floors / kinds / widths
  for (const [iface, ifaceLayers, layerHeight] of [['flat', 1, 0.2], ['all', 3, 0.12], ['all', 2, undefined]]) {
    const st = { iface, ifaceLayers };
    withProp(st, () => {
      crest.crestPart(null, layerHeight);
      const P = (o, z) => [1 + 0.3 * o, 2 - 0.7 * o, z];
      const cuts = [];
      for (const top of [0.9, 3, 10.5])
        for (const ztip of [top - 1.5, top - 0.3, top + 0.2])
          for (const floor of [0, top - 0.5, top - 2.0])
            for (const kind of ['flat', 'slope', null])
              for (const [th, tip] of [[undefined, undefined], [1.2, 0.4]]) {
                const c = crest.crestCut(top, ztip, floor, kind, th, tip);
                cuts.push({ top, ztip, floor, kind, th: th ?? null, tip: tip ?? null,
                            out: c ? { cut: c.cut, zt: c.zt, wc: c.wc, ring: c.ring(P) } : null });
              }
      cases.push({ fn: 'crestCut', prop: st, layerHeight: layerHeight ?? null, crestH: crest.crestH(), cuts,
                   flatCut: [0.3, 2, 7.25].map((t) => crest.flatCut(t)),
                   flatCrestRing: [[2, 0.5], [7.25, 0.8]].map(([t, h]) => crest.flatCrestRing(P, t, h)) });
    });
  }
  // crestOn / crestKinds against the part, per mode; then the sweep.js crest path
  const crestModes = [[false, 1, 0.2], ['flat', 1, 0.2], ['flat', 0, 0.2], ['all', 1, 0.2], ['all', 2, 0.16], ['flat', 3, undefined]];
  const crestLines = lines.map((l) => l.line).slice(0, 6).concat(tops.slice(-1));
  for (const [iface, ifaceLayers, layerHeight] of crestModes) {
    const st = { iface, ifaceLayers };
    for (const [pi, partId] of [[0, 'part'], [1, null]]) {
      if (pi === 1 && iface !== 'all') continue;     // no part: only 'all' crests anything
      withProp(st, () => {
        crest.crestPart(partId ? partTris : null, layerHeight);
        for (const line of crestLines) {
          const kinds = crest.crestKinds(line);
          // sweep.js's loop (without cutWall), its crest bookkeeping verbatim
          const zBed = 0;
          const wall = [], flange = [], cr = [];
          let ok = true;
          for (let i = 0; i < line.length; i++) {
            const p = line[i];
            const a = line[Math.max(0, i - 1)];
            const b = line[Math.min(line.length - 1, i + 1)];
            let rx = b[0] - a[0], ry = b[1] - a[1];
            const rn = Math.hypot(rx, ry);
            if (rn < 1e-9) { ok = false; break; }
            rx /= rn; ry /= rn;
            const sx = ry, sy = -rx;
            const top = p[2] - PROP.gap;
            const h = top - zBed;
            if (h < 0.3) { ok = false; break; }
            const foot = 2.0;
            const flangeH = Math.min(PROP.baseH, h / 2);
            const ztip = Math.max(top - PROP.tipH, zBed + flangeH + 0.1);
            const baseTop = zBed + flangeH;
            const P = (o, z) => [p[0] + sx * o, p[1] + sy * o, z];
            const c = crest.crestCut(top, ztip, zBed, kinds?.[i]);
            cr.push(c?.ring(P) ?? null);
            const [zt, zTop, wTop] = c ? [c.zt, c.cut, c.wc] : [ztip, top, PROP.tip / 2];
            wall.push([P(+PROP.th / 2, zBed), P(+PROP.th / 2, zt), P(+wTop, zTop),
                       P(-wTop, zTop), P(-PROP.th / 2, zt), P(-PROP.th / 2, zBed)]);
            flange.push([P(+foot, zBed), P(+foot, baseTop), P(-foot, baseTop), P(-foot, zBed)]);
          }
          const out = [];
          let fromTine = null;
          if (ok) {
            // something already in `out` (an earlier wall), untagged
            out.push([0, 0, 0], [1, 0, 0], [0, 1, 0]);
            const from = out.length;
            ribbon(wall, out);
            ribbon(flange, out);
            if (kinds) { crest.dropCollapsed(out, from); crest.emitCrest(cr, out); }
            // tined: emitTines tags its comb under 'all' (a stand-in comb: a few triangles)
            fromTine = out.length;
            for (let k = 0; k < 2; k++) out.push([k, 5, 1], [k + 1, 5, 1], [k, 6, 1.5]);
            if (crest.crestOn() && PROP.iface === 'all') crest.markInterface(out, fromTine);
          }
          const split = crest.splitInterface(out);
          cases.push({ fn: 'crestWall', prop: st, part: partId, layerHeight: layerHeight ?? null,
                       on: crest.crestOn(), crestH: crest.crestH(), line, kinds, ok,
                       out: flat(out), tags: tagsOf(out),
                       nBody: split.body.length, nIface: split.iface.length });
        }
      });
    }
  }
  crest.crestPart(null);
  // dropCollapsed / markInterface / emitCrest on hand-built input: collapsed corners,
  // a ragged tail, null gaps between crest runs, a lone section
  {
    const A = [0, 0, 0], B = [1, 0, 0], C = [0, 1, 0], D = [0, 0, 1];
    const base = () => [A, B, C, [...A], [...A], B, B, C, [...C], A, B, D, C, [-0, 0, 0], [0, 0, 0], D, C];
    for (const from of [0, 3, 6]) {
      const out = base().map((p) => [...p]);
      crest.dropCollapsed(out, from);
      cases.push({ fn: 'dropCollapsed', inp: flat(base()), from, out: flat(out) });
    }
    const sec = (z) => [[0, 0, z], [1, 0, z], [1, 1, z], [0, 1, z]];
    const sections = [sec(0), sec(1), null, sec(2), null, sec(3), sec(4), sec(5), null, null];
    const out = [[9, 9, 9], [8, 8, 8], [7, 7, 7]];
    crest.emitCrest(sections, out);
    crest.markInterface(out, 1);   // partial: from inside a triangle
    cases.push({ fn: 'emitCrest', sections, out: flat(out), tags: tagsOf(out),
                 body: flat(crest.splitInterface(out).body), iface: flat(crest.splitInterface(out).iface) });
  }

  // ---- braces: short walls standing tall under the part, at several heights and
  // headings, plus one in open air; floors from floorLine as draw.js passes them
  {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < partTris.length; i += 3) {
      x0 = Math.min(x0, partTris[i]); x1 = Math.max(x1, partTris[i]);
      y0 = Math.min(y0, partTris[i + 1]); y1 = Math.max(y1, partTris[i + 1]);
    }
    const walls = [];
    const G = 9;
    for (let a = 1; a < G; a++) for (let b = 1; b < G; b++) {
      const x = x0 + (x1 - x0) * a / G, y = y0 + (y1 - y0) * b / G;
      const z = surface.surfaceZAt(partTris, x, y);
      if (z !== null && z > 7) walls.push([x, y]);
    }
    const picked = [];
    for (let k = 0; k < walls.length && picked.length < 6; k += Math.max(1, Math.floor(walls.length / 6))) picked.push(walls[k]);
    const heads = [0, 0.65, Math.PI / 2, 2.4];
    const wallsIn = [];
    picked.forEach(([x, y], k) => {
      const ang = heads[k % heads.length];
      const top = [];
      for (let i = 0; i < 3 + (k % 2); i++) {
        const xx = x + Math.cos(ang) * 0.5 * i, yy = y + Math.sin(ang) * 0.5 * i;
        const z = surface.surfaceZAt(partTris, xx, yy);
        if (z === null) return;
        top.push([xx, yy, z]);
      }
      wallsIn.push(top);
    });
    // open air beside the part, tall
    wallsIn.push([[x1 + 30, y0, 30], [x1 + 31, y0, 30], [x1 + 32, y0, 30.2]]);
    wallsIn.push([[x1 + 30, y0, 30], [x1 + 30, y0 + 0.6, 30]]);
    wallsIn.push([[x1 + 30, y0, 5], [x1 + 31, y0, 5]]);                         // stocky
    wallsIn.push([[x0 - 1.2, (y0 + y1) / 2, 40], [x0 - 1.2, (y0 + y1) / 2 + 1, 40]]);   // hugging the part
    wallsIn.push([[x1 + 30, y0, 30]]);                                             // a single station
    for (const top of wallsIn) {
      for (const prop of [{}, { gap: 0.25, footMin: 2.0 }]) {
        withProp(prop, () => {
          const fl = mold.floorLine(top, partTris).map((p) => p[2]);
          for (const [floorKind, floorZ] of [['floor', (i) => fl[i]], ['plate', () => 0]]) {
            for (const clearKind of ['none', 'all', 'alt', 'never']) {
              const calls = [];
              let nCall = 0;
              const clear = clearKind === 'none' ? undefined : (t) => {
                calls.push([t.length, t[0][0], t[0][1], t[0][2]]);
                nCall++;
                return clearKind === 'all' ? true : clearKind === 'never' ? false : nCall % 2 === 0;
              };
              const out = [];
              const built = brace.braceWall(top, floorZ, partTris, 0, out, clear);
              cases.push({ fn: 'braceWall', prop, top, floorKind, floors: top.map((_, i) => floorZ(i)), zBed: 0,
                           clear: clearKind, built, calls, out: flat(out) });
            }
          }
          // zBed above the plate (the floor of a raised bed)
          const out = [];
          const built = brace.braceWall(top, (i) => 0.3, partTris, 0.3, out);
          cases.push({ fn: 'braceWall', prop, top, floorKind: 'const', floors: top.map(() => 0.3), zBed: 0.3,
                       clear: 'none', built, calls: [], out: flat(out) });
        });
      }
    }
  }

  // ---- ties: pairs of plate points at several spans and heights, near and away from the part
  {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < partTris.length; i += 3) {
      x0 = Math.min(x0, partTris[i]); x1 = Math.max(x1, partTris[i]);
      y0 = Math.min(y0, partTris[i + 1]); y1 = Math.max(y1, partTris[i + 1]);
    }
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const anchors = [[cx, cy], [x0 - 2, cy], [x1 + 25, y0], [x0 + (x1 - x0) * 0.2, y1 + 0.5]];
    for (const [ax, ay] of anchors) {
      for (const [span, ang] of [[1.5, 0], [3, 0.3], [6, Math.PI / 2], [9.5, 2.2], [12, 4], [13, 1]]) {
        const a = [ax, ay], b = [ax + span * Math.cos(ang), ay + span * Math.sin(ang)];
        for (const zMax of [4, 12.5, 30]) {
          for (const prop of [{}, { th: 1.2, sideClear: 0.5 }]) {
            const out = [];
            const n = withProp(prop, () => ties.tieWalls([...a, zMax, 0], b, zMax, partTris, 0, out));
            cases.push({ fn: 'tieWalls', prop, a, b, zMax, zBed: 0, n, out: flat(out) });
          }
          const out = [];
          const n = ties.tieWalls(a, b, zMax + 0.4, partTris, 0.4, out);
          cases.push({ fn: 'tieWalls', prop: {}, a, b, zMax: zMax + 0.4, zBed: 0.4, n, out: flat(out) });
        }
      }
    }
  }

  const name = `${model}-${rotStr.replace(/,/g, '_')}`;
  const consts = {
    BRACE: brace.BRACE, TIE: ties.TIE,
    PROP: PROP0,
  };
  const replacer = (k, val) => (typeof val === 'number' && !Number.isFinite(val) ? String(val) : val);
  writeFileSync(path.join(outDir, `${name}.json`),
    JSON.stringify({ scene: name, model, rot: rotStr, consts, tris: trisTable, cases }, replacer));
  const counts = {};
  for (const c of cases) counts[c.fn] = (counts[c.fn] ?? 0) + 1;
  console.log(name, JSON.stringify(counts));
}
