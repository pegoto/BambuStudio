#!/usr/bin/env node
// Reference oracle for Engine/Ieee754.* (JS Math.* transcendentals under Node 22 /
// V8 12.4). Needs no upstream checkout. Writes raw little-endian float64 files for
// tests/test_ieee754.cpp:
//   inputs.f64            N unary inputs (the same set for every function)
//   <fn>.f64              N outputs of Math.<fn> on inputs.f64
//   atan2_y.f64, atan2_x.f64, atan2.f64   grid axes (Ny, Nx) and Ny*Nx results,
//                         row-major: atan2[i*Nx + j] = Math.atan2(y[i], x[j])
//   pow_x.f64, pow_y.f64, pow.f64         same for Math.pow(x[i], y[j])
//   meta.json             counts and the Node / V8 version that produced them
//
//   node tools/ieee754_ref.mjs [outDir]
//
// outDir defaults to refs_ieee754/ next to tools/ (gitignored). Deterministic: the
// pseudo-random parts use a fixed seed.
import { writeFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const outDir = process.argv[2] || path.join(here, '..', 'refs_ieee754');
mkdirSync(outDir, { recursive: true });

// ---- bit helpers ---------------------------------------------------------------------
const dv = new DataView(new ArrayBuffer(8));
const fromWords = (hi, lo) => { dv.setUint32(0, lo >>> 0, true); dv.setUint32(4, hi >>> 0, true); return dv.getFloat64(0, true); };
const hiWord = (d) => { dv.setFloat64(0, d, true); return dv.getUint32(4, true); };
const loWord = (d) => { dv.setFloat64(0, d, true); return dv.getUint32(0, true); };
// neighbours of d in the double order (finite d only; through zero via the sign)
const nextUp = (d) => {
  if (Number.isNaN(d) || d === Infinity) return d;
  if (d === 0) return Number.MIN_VALUE;
  let hi = hiWord(d), lo = loWord(d);
  if (d > 0) { lo = (lo + 1) >>> 0; if (lo === 0) hi = (hi + 1) >>> 0; }
  else { if (lo === 0) hi = (hi - 1) >>> 0; lo = (lo - 1) >>> 0; }
  return fromWords(hi, lo);
};
const nextDown = (d) => -nextUp(-d);

// ---- deterministic PRNG (xorshift128, 32-bit words) ------------------------------------
let s0 = 0x9E3779B9, s1 = 0x243F6A88, s2 = 0xB7E15162, s3 = 0x7F4A7C15;
const u32 = () => {
  const t = s0 ^ (s0 << 11);
  s0 = s1; s1 = s2; s2 = s3;
  s3 = (s3 ^ (s3 >>> 19) ^ (t ^ (t >>> 8))) >>> 0;
  return s3;
};
const unif = () => (u32() * 2097152 + (u32() >>> 11)) / 9007199254740992; // [0,1), 53 bits
const uniform = (a, b) => a + (b - a) * unif();
// log-uniform magnitude: biased exponent uniform in [e0, e1], random mantissa
const logUniform = (e0, e1) => fromWords(((e0 + (u32() % (e1 - e0 + 1))) << 20) | (u32() & 0xFFFFF), u32());
const sign = (d) => (u32() & 1 ? -d : d);
const randomBits = () => fromWords(u32(), u32());

// ---- special values ---------------------------------------------------------------------
const specials = [];
const sp = (d) => { specials.push(d); };
for (const d of [0, -0, Infinity, -Infinity, NaN, Number.MIN_VALUE, Number.MAX_VALUE,
                 2.2250738585072014e-308 /* min normal */, 2.225073858507201e-308 /* max denormal */,
                 Math.PI, Math.E, Math.LN2, Math.LN10, Math.SQRT2, Math.SQRT1_2, 0.5, 1, 2, 3, 10,
                 1e-300, 1e300, 709.782712893384, 709.7827128933841, -745.1332191019411, -745.1332191019412,
                 -708.3964185322641, 7.09782712893383973096e+02, -7.45133219101941108420e+02,
                 0.9999999999999999, 1.0000000000000002, 1e-8, 1e-16, 1e-20, 1e22, 2 ** 53, 2 ** 31, 2 ** 52]) {
  sp(d); sp(-d);
}
// denormals
for (let i = 0; i < 64; i++) { sp(fromWords(0, 1 << (i % 32))); sp(fromWords(i % 21 < 20 ? 1 << (i % 20) : 0xFFFFF, u32())); }
// powers of two, every exponent, both signs, and their neighbours
for (let e = -1074; e <= 1023; e++) { const d = 2 ** e; sp(d); sp(-d); sp(nextUp(d)); sp(nextDown(d)); }
// multiples of pi/4 and pi/2 (small and huge, where argument reduction works hardest)
for (let k = -256; k <= 256; k++) { const d = k * Math.PI / 4; sp(d); sp(nextUp(d)); sp(nextDown(d)); }
for (const k of [1e3, 1e5, 1e6, 1e8, 1e10, 1e15, 1e20, 1e100, 1e200, 1e300]) { sp(k * Math.PI / 2); sp(-k * Math.PI); }
// fdlibm branch points: every high-word constant in V8's src/base/ieee754.cc (ix/hx
// thresholds, the pi/2 multiples table, coefficient words -- harmless extras), as
// doubles with that high word, low word 0 / 1 / 0xFFFFFFFF, their predecessor, both
// signs; short constants are mantissa thresholds (log's 0x6147a, 0x95f64, sqrt2's
// 0x6a09e, pow's 0x3988e / 0xbb67a, ...) applied in [0.5,1), [1,2), [2,4).
const V8_HEX = `0x00080000 0x000FFFFF 0x00100000 0x00200000 0x01700000 0x03EB6F7D 0x0561BB4F 0x05980F 0x0739F7
0x1529FC 0x1556CA 0x1A600000 0x1CB129 0x1DA9E3 0x2E000000 0x3C600000 0x3C700788 0x3C7A2B7F 0x3C800000 0x3C81A626
0x3C900000 0x3C91A626 0x3CA1A626 0x3CF84698 0x3D59FEF3 0x3DD0B461 0x3DE5D93A 0x3DE705FC 0x3DEA39EF 0x3E200000
0x3E21EE9E 0x3E300000 0x3E400000 0x3E4CFDEB 0x3E54AE0B 0x3E663769 0x3E700000 0x3E74442D 0x3EC71DE3 0x3EFA01A0
0x3F023DE1 0x3F11566A 0x3F49EFE0 0x3F669E 0x3F811111 0x3F90AD3A 0x3FA55555 0x3FA97B4B 0x3FB10D66 0x3FB3B8C5
0x3FB745CD 0x3FC24924 0x3FC2B000 0x3FC55555 0x3FC9C155 0x3FCA7E28 0x3FCD864A 0x3FD17460 0x3FD33333 0x3FD34413
0x3FD55555 0x3FD62E42 0x3FD62E43 0x3FDA827A 0x3FDB6DB6 0x3FDC0000 0x3FDDAC67 0x3FE00000 0x3FE2B803 0x3FE33333
0x3FE45F30 0x3FE59428 0x3FE60000 0x3FE62E42 0x3FE62E43 0x3FE90000 0x3FE921FB 0x3FEEC709 0x3FEF3333 0x3FEF730B
0x3FEFFFFF 0x3FF00000 0x3FF0A2B2 0x3FF30000 0x3FF71547 0x3FF921FB 0x3FF9F160 0x3FFE03E6 0x40000000 0x40002AE5
0x4002D97C 0x40038000 0x4005BF0A 0x400921FB 0x4012D97C 0x401921FB 0x401F6A7A 0x4022D97C 0x4025FDBB 0x402921FB
0x402C463A 0x402F6A7A 0x4031475C 0x4032D97C 0x40346B9C 0x4035FDBB 0x40360000 0x40378FDB 0x403921FB 0x403AB41B
0x403C463A 0x403DD85A 0x403F6A7A 0x40407E4C 0x4041475C 0x4042106C 0x4042D97C 0x4043687A 0x4043A28C 0x40446B9C
0x404534AC 0x4045FDBB 0x4046C6CB 0x40478FDB 0x404858EB 0x404921FB 0x40862E42 0x40900000 0x4090CC00 0x413921FB
0x41700000 0x41B00000 0x41E00000 0x43400000 0x43500000 0x43F00000 0x44100000 0x7FEFFFFF 0x7FF00000
0x3988E 0x6147A 0x6A09E 0x6B851 0x95F64 0xBB67A 0xFFFFF 0x80000`;
for (const tok of V8_HEX.split(/\s+/)) {
  const v = parseInt(tok, 16);
  const highs = v < 0x100000 ? [0x3FE00000 | v, 0x3FF00000 | v, 0x40000000 | v] : [v];
  for (const h of highs) for (const lo of [0, 1, 0xFFFFFFFF, 0x80000000]) {
    const d = fromWords(h, lo);
    sp(d); sp(-d); sp(nextDown(d));
  }
}

// ---- the doubles the engine actually feeds ---------------------------------------------
const engine = [];
const en = (d) => { engine.push(d); };
for (let i = 0; i <= 720; i++) {
  const deg = i / 2;
  en((deg * Math.PI) / 180); en(deg * Math.PI / 180); en(deg / 180 * Math.PI); en(-(deg * Math.PI) / 180);
  en(deg);
}
for (const n of [3, 4, 5, 6, 8, 10, 12, 16, 18, 20, 24, 32, 36, 40, 48, 60, 64, 72, 90, 96, 120, 128, 180, 360]) {
  for (let k = 0; k <= n; k++) {
    en(2 * Math.PI * k / n); en((2 * Math.PI * k) / n); en(k * 2 * Math.PI / n); en(Math.PI * 2 * k / n);
    en((k / n) * 2 * Math.PI); en(k * Math.PI / n); en(-2 * Math.PI * k / n);
  }
}
for (let k = 0; k < 16; k++) en(k * Math.PI / 8);
// sines/cosines of those angles: what asin/acos/atan2 see as unit-vector components
const angles = engine.slice();
for (const a of angles) { en(Math.sin(a)); en(Math.cos(a)); en(Math.tan(a)); }

// ---- unary input set ---------------------------------------------------------------------
const inputs = [...specials, ...engine];
for (let i = 0; i < 100000; i++) inputs.push(uniform(-10, 10));            // dense [-10,10]
for (let i = 0; i < 100000; i++) inputs.push(sign(logUniform(26, 2020)));  // ~1e-300 .. 1e300, both signs
for (let i = 0; i < 40000; i++) inputs.push(uniform(-1, 1));               // asin/acos/atanh domain
for (let i = 0; i < 20000; i++) inputs.push(1 + sign(logUniform(900, 1022)) * unif()); // near 1 (log, pow, acosh)
for (let i = 0; i < 20000; i++) inputs.push(uniform(-760, 760));           // exp / sinh / cosh range edges
for (let i = 0; i < 20000; i++) inputs.push(randomBits());                 // anything, NaN payloads too
const N = inputs.length;

const FNS = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'exp', 'expm1', 'log', 'log1p', 'log2', 'log10',
             'cbrt', 'sinh', 'cosh', 'tanh', 'asinh', 'acosh', 'atanh'];
const write = (name, arr) => writeFileSync(path.join(outDir, name), Buffer.from(Float64Array.from(arr).buffer));
write('inputs.f64', inputs);
for (const fn of FNS) {
  const f = Math[fn], out = new Float64Array(N);
  for (let i = 0; i < N; i++) out[i] = f(inputs[i]);
  write(`${fn}.f64`, out);
}

// ---- 2-D grids for atan2 and pow -----------------------------------------------------------
const GRID = 2048;
const axis = (seed, fill) => {
  const set = [];
  for (const d of seed) if (set.length < GRID) set.push(d);
  while (set.length < GRID) set.push(fill(set.length));
  return set;
};
const gridSpecials = [0, -0, Infinity, -Infinity, NaN, Number.MIN_VALUE, -Number.MIN_VALUE, Number.MAX_VALUE,
  -Number.MAX_VALUE, 1, -1, 0.5, -0.5, 2, -2, 1e-300, -1e-300, 1e300, -1e300, 0.9999999999999999,
  1.0000000000000002, -0.9999999999999999, -1.0000000000000002, 2.2250738585072014e-308, Math.PI, -Math.PI];
const pickEngine = (step) => angles.filter((_, i) => i % step === 0).flatMap((a) => [Math.sin(a), Math.cos(a)]);
const mixed = (i) => {
  switch (i % 4) {
    case 0: return uniform(-10, 10);
    case 1: return sign(logUniform(26, 2020));
    case 2: return uniform(-1, 1);
    default: return sign(logUniform(1003, 1043)); // ~2^-20 .. 2^20
  }
};
const at2y = axis([...gridSpecials, ...pickEngine(9)], mixed);
const at2x = axis([...gridSpecials, ...pickEngine(7)], mixed);
const powExps = [];
for (let k = -64; k <= 64; k++) { powExps.push(k, k + 0.5); }
for (const d of [1 / 3, -1 / 3, 2 / 3, 0.25, -0.25, 1.5, 2.5, 1e-10, -1e-10, 1e10, -1e10, 2 ** 31, 2 ** 31 + 1,
                 2 ** 53, 2 ** 53 + 2, 2 ** 64, -(2 ** 64), 1023, 1024, 1025, -1074, -1075, -1076, 0.1, -0.1])
  powExps.push(d);
const powBases = [];
for (let k = -20; k <= 20; k++) powBases.push(k, k / 2, k / 10);
for (let e = -60; e <= 60; e += 3) { powBases.push(2 ** e, -(2 ** e), nextUp(2 ** e), nextDown(2 ** e)); }
const powX = axis([...gridSpecials, ...powBases, ...pickEngine(23)],
  (i) => (i % 3 === 0 ? uniform(0, 10) : mixed(i)));
const powY = axis([...gridSpecials, ...powExps],
  (i) => (i % 3 === 0 ? uniform(-4, 4) : mixed(i)));

const grid = (A, B, f) => {
  const out = new Float64Array(A.length * B.length);
  for (let i = 0; i < A.length; i++) for (let j = 0; j < B.length; j++) out[i * B.length + j] = f(A[i], B[j]);
  return out;
};
write('atan2_y.f64', at2y); write('atan2_x.f64', at2x);
write('atan2.f64', grid(at2y, at2x, Math.atan2));
write('pow_x.f64', powX); write('pow_y.f64', powY);
write('pow.f64', grid(powX, powY, Math.pow));
// the ** operator must agree with Math.pow (both are base::ieee754::pow in V8 12.4)
let starDiff = 0;
for (let i = 0; i < powX.length; i += 7) for (let j = 0; j < powY.length; j++) {
  const a = powX[i] ** powY[j], b = Math.pow(powX[i], powY[j]);
  if (!(Object.is(a, b) || (Number.isNaN(a) && Number.isNaN(b)))) starDiff++;
}

writeFileSync(path.join(outDir, 'meta.json'), JSON.stringify({
  node: process.version, v8: process.versions.v8, n: N, specials: specials.length, engine: engine.length,
  functions: FNS, grid: GRID, starVsPowDiffs: starDiff,
}, null, 1));
console.log(`node ${process.version} (V8 ${process.versions.v8}): ${N} unary inputs x ${FNS.length} functions, ` +
            `atan2/pow grids ${GRID}x${GRID}; ** vs Math.pow differences: ${starDiff}`);
