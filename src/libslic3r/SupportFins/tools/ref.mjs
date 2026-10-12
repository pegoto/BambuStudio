#!/usr/bin/env node
// Reference oracle for the C++ port: runs the upstream JS engine (gittrahan/support-fins,
// plugins/shared/engine/fins_entry.js) on an STL at a site pose and dumps everything the
// C++ port must reproduce. Usage:
//   SUPPORT_FINS_DIR=/path/to/support-fins node ref.mjs model.stl "rx,ry,rz" out.json [optionsJSON]
// The dump holds the POSED soup (float64, before seating), so the C++ test consumes the
// same input bytes and the comparison is engine vs engine, not STL reader vs STL reader.
import { readFileSync, writeFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import path from 'node:path';

const SF = process.env.SUPPORT_FINS_DIR;
if (!SF) { console.error('set SUPPORT_FINS_DIR'); process.exit(2); }
const [stlPath, rotStr = '0,0,0', outPath, optsJson = '{}'] = process.argv.slice(2);
if (!stlPath || !outPath) { console.error('usage: ref.mjs model.stl rx,ry,rz out.json [options]'); process.exit(2); }

const entry = await import(pathToFileURL(path.join(SF, 'plugins/shared/engine/fins_entry.js')).href);

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
const rot = sitePose(rotStr.split(',').map(Number));
const posed = new Float64Array(raw.length);
for (let i = 0; i < raw.length; i += 3) {
  const x = raw[i], y = raw[i + 1], z = raw[i + 2];
  posed[i] = rot[0] * x + rot[3] * y + rot[6] * z;
  posed[i + 1] = rot[1] * x + rot[4] * y + rot[7] * z;
  posed[i + 2] = rot[2] * x + rot[5] * y + rot[8] * z;
}
const options = JSON.parse(optsJson);
const t0 = performance.now();
const r = entry.computeFins(posed, options);
const ms = performance.now() - t0;
writeFileSync(outPath, JSON.stringify({
  model: path.basename(stlPath), rot: rotStr, options, ms: Math.round(ms),
  soup: Array.from(posed),
  offset: r.offset, stats: r.stats, pieces: r.pieces,
  overFaces: Array.from(r.overFaces), smallFaces: Array.from(r.smallFaces),
  triangles: Array.from(r.triangles),
}));
console.log(`${path.basename(stlPath)} rot=${rotStr}: ${JSON.stringify(r.stats)} (${Math.round(ms)} ms, ${r.triangles.length / 9} tris)`);
