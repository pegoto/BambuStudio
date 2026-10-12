// Port of web/fins/pad.js -- PAD and the bed pad (conforming oval, or the brim-style one).
#include "Pad.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>

namespace Slic3r { namespace SupportFins {

thread_local PadConfig PAD;

namespace {

// ---------------------------------------------------------------------------------------
// JS Math.sin / Math.cos: V8's fdlibm port (src/base/ieee754.cc at V8 12.4.254.21, the
// Node 22 build, which leaves V8_USE_LIBM_TRIG_FUNCTIONS off), copied verbatim but for
// names. glibc's sin/cos are correctly rounded and fdlibm's are not quite: on the 'sure'
// pad's ring, sin(2*PI*32/48) differs in the last bit, which moves a third of the oval's
// vertices by 1 ulp and can flip a surfaceZAt hit under them. -ffp-contract=off is
// required (the build sets it).
//
// The following is adapted from fdlibm (http://www.netlib.org/fdlibm).
// ====================================================
// Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//
// Developed at SunSoft, a Sun Microsystems, Inc. business.
// Permission to use, copy, modify, and distribute this
// software is freely granted, provided that this notice
// is preserved.
// ====================================================
// The original source code covered by the above license above has been
// modified significantly by Google Inc.
// Copyright 2016 the V8 project authors. All rights reserved.
namespace fdlibm {

template <class To, class From> inline To bit_cast(const From &v) {
    static_assert(sizeof(To) == sizeof(From), "bit_cast size");
    To r;
    std::memcpy(&r, &v, sizeof r);
    return r;
}

#define EXTRACT_WORDS(ix0, ix1, d)               \
  do {                                           \
    uint64_t bits = bit_cast<uint64_t>(d); \
    (ix0) = bits >> 32;                          \
    (ix1) = bits & 0xFFFFFFFFu;                  \
  } while (false)

/* Get the more significant 32 bit int from a double.  */

#define GET_HIGH_WORD(i, d)                      \
  do {                                           \
    uint64_t bits = bit_cast<uint64_t>(d); \
    (i) = bits >> 32;                            \
  } while (false)

/* Get the less significant 32 bit int from a double.  */

#define GET_LOW_WORD(i, d)                       \
  do {                                           \
    uint64_t bits = bit_cast<uint64_t>(d); \
    (i) = bits & 0xFFFFFFFFu;                    \
  } while (false)

/* Set a double from two 32 bit ints.  */

#define INSERT_WORDS(d, ix0, ix1)             \
  do {                                        \
    uint64_t bits = 0;                        \
    bits |= static_cast<uint64_t>(ix0) << 32; \
    bits |= static_cast<uint32_t>(ix1);       \
    (d) = bit_cast<double>(bits);       \
  } while (false)

/* Set the more significant 32 bits of a double from an int.  */

#define SET_HIGH_WORD(d, v)                      \
  do {                                           \
    uint64_t bits = bit_cast<uint64_t>(d); \
    bits &= 0x0000'0000'FFFF'FFFF;               \
    bits |= static_cast<uint64_t>(v) << 32;      \
    (d) = bit_cast<double>(bits);          \
  } while (false)

/* Set the less significant 32 bits of a double from an int.  */

#define SET_LOW_WORD(d, v)                       \
  do {                                           \
    uint64_t bits = bit_cast<uint64_t>(d); \
    bits &= 0xFFFF'FFFF'0000'0000;               \
    bits |= static_cast<uint32_t>(v);            \
    (d) = bit_cast<double>(bits);          \
  } while (false)

int32_t ieee754_rem_pio2(double x, double* y);
int kernel_rem_pio2(double* x, double* y, int e0, int nx, int prec,
                      const int32_t* ipio2);
double kernel_cos(double x, double y);
double kernel_sin(double x, double y, int iy);

int32_t ieee754_rem_pio2(double x, double *y) {
  /*
   * Table of constants for 2/pi, 396 Hex digits (476 decimal) of 2/pi
   */
  static const int32_t two_over_pi[] = {
      0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62, 0x95993C,
      0x439041, 0xFE5163, 0xABDEBB, 0xC561B7, 0x246E3A, 0x424DD2, 0xE00649,
      0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129, 0xA73EE8, 0x8235F5, 0x2EBB44,
      0x84E99C, 0x7026B4, 0x5F7E41, 0x3991D6, 0x398353, 0x39F49C, 0x845F8B,
      0xBDF928, 0x3B1FF8, 0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D,
      0x367ECF, 0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5,
      0xF17B3D, 0x0739F7, 0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08, 0x560330,
      0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3, 0x91615E, 0xE61B08,
      0x659985, 0x5F14A0, 0x68408D, 0xFFD880, 0x4D7327, 0x310606, 0x1556CA,
      0x73A8C9, 0x60E27B, 0xC08C6B,
  };

  static const int32_t npio2_hw[] = {
      0x3FF921FB, 0x400921FB, 0x4012D97C, 0x401921FB, 0x401F6A7A, 0x4022D97C,
      0x4025FDBB, 0x402921FB, 0x402C463A, 0x402F6A7A, 0x4031475C, 0x4032D97C,
      0x40346B9C, 0x4035FDBB, 0x40378FDB, 0x403921FB, 0x403AB41B, 0x403C463A,
      0x403DD85A, 0x403F6A7A, 0x40407E4C, 0x4041475C, 0x4042106C, 0x4042D97C,
      0x4043A28C, 0x40446B9C, 0x404534AC, 0x4045FDBB, 0x4046C6CB, 0x40478FDB,
      0x404858EB, 0x404921FB,
  };

  /*
   * invpio2:  53 bits of 2/pi
   * pio2_1:   first  33 bit of pi/2
   * pio2_1t:  pi/2 - pio2_1
   * pio2_2:   second 33 bit of pi/2
   * pio2_2t:  pi/2 - (pio2_1+pio2_2)
   * pio2_3:   third  33 bit of pi/2
   * pio2_3t:  pi/2 - (pio2_1+pio2_2+pio2_3)
   */

  static const double
      zero = 0.00000000000000000000e+00,    /* 0x00000000, 0x00000000 */
      half = 5.00000000000000000000e-01,    /* 0x3FE00000, 0x00000000 */
      two24 = 1.67772160000000000000e+07,   /* 0x41700000, 0x00000000 */
      invpio2 = 6.36619772367581382433e-01, /* 0x3FE45F30, 0x6DC9C883 */
      pio2_1 = 1.57079632673412561417e+00,  /* 0x3FF921FB, 0x54400000 */
      pio2_1t = 6.07710050650619224932e-11, /* 0x3DD0B461, 0x1A626331 */
      pio2_2 = 6.07710050630396597660e-11,  /* 0x3DD0B461, 0x1A600000 */
      pio2_2t = 2.02226624879595063154e-21, /* 0x3BA3198A, 0x2E037073 */
      pio2_3 = 2.02226624871116645580e-21,  /* 0x3BA3198A, 0x2E000000 */
      pio2_3t = 8.47842766036889956997e-32; /* 0x397B839A, 0x252049C1 */

  double z, w, t, r, fn;
  double tx[3];
  int32_t e0, i, j, nx, n, ix, hx;
  uint32_t low;

  z = 0;
  GET_HIGH_WORD(hx, x); /* high word of x */
  ix = hx & 0x7FFFFFFF;
  if (ix <= 0x3FE921FB) { /* |x| ~<= pi/4 , no need for reduction */
    y[0] = x;
    y[1] = 0;
    return 0;
  }
  if (ix < 0x4002D97C) { /* |x| < 3pi/4, special case with n=+-1 */
    if (hx > 0) {
      z = x - pio2_1;
      if (ix != 0x3FF921FB) { /* 33+53 bit pi is good enough */
        y[0] = z - pio2_1t;
        y[1] = (z - y[0]) - pio2_1t;
      } else { /* near pi/2, use 33+33+53 bit pi */
        z -= pio2_2;
        y[0] = z - pio2_2t;
        y[1] = (z - y[0]) - pio2_2t;
      }
      return 1;
    } else { /* negative x */
      z = x + pio2_1;
      if (ix != 0x3FF921FB) { /* 33+53 bit pi is good enough */
        y[0] = z + pio2_1t;
        y[1] = (z - y[0]) + pio2_1t;
      } else { /* near pi/2, use 33+33+53 bit pi */
        z += pio2_2;
        y[0] = z + pio2_2t;
        y[1] = (z - y[0]) + pio2_2t;
      }
      return -1;
    }
  }
  if (ix <= 0x413921FB) { /* |x| ~<= 2^19*(pi/2), medium size */
    t = std::fabs(x);
    n = static_cast<int32_t>(t * invpio2 + half);
    fn = static_cast<double>(n);
    r = t - fn * pio2_1;
    w = fn * pio2_1t; /* 1st round good to 85 bit */
    if (n < 32 && ix != npio2_hw[n - 1]) {
      y[0] = r - w; /* quick check no cancellation */
    } else {
      uint32_t high;
      j = ix >> 20;
      y[0] = r - w;
      GET_HIGH_WORD(high, y[0]);
      i = j - ((high >> 20) & 0x7FF);
      if (i > 16) { /* 2nd iteration needed, good to 118 */
        t = r;
        w = fn * pio2_2;
        r = t - w;
        w = fn * pio2_2t - ((t - r) - w);
        y[0] = r - w;
        GET_HIGH_WORD(high, y[0]);
        i = j - ((high >> 20) & 0x7FF);
        if (i > 49) { /* 3rd iteration need, 151 bits acc */
          t = r;      /* will cover all possible cases */
          w = fn * pio2_3;
          r = t - w;
          w = fn * pio2_3t - ((t - r) - w);
          y[0] = r - w;
        }
      }
    }
    y[1] = (r - y[0]) - w;
    if (hx < 0) {
      y[0] = -y[0];
      y[1] = -y[1];
      return -n;
    } else {
      return n;
    }
  }
  /*
   * all other (large) arguments
   */
  if (ix >= 0x7FF00000) { /* x is inf or NaN */
    y[0] = y[1] = x - x;
    return 0;
  }
  /* set z = std::scalbn(|x|,ilogb(x)-23) */
  GET_LOW_WORD(low, x);
  SET_LOW_WORD(z, low);
  e0 = (ix >> 20) - 1046; /* e0 = ilogb(z)-23; */
  SET_HIGH_WORD(z, ix - static_cast<int32_t>(static_cast<uint32_t>(e0) << 20));
  for (i = 0; i < 2; i++) {
    tx[i] = static_cast<double>(static_cast<int32_t>(z));
    z = (z - tx[i]) * two24;
  }
  tx[2] = z;
  nx = 3;
  while (tx[nx - 1] == zero) nx--; /* skip zero term */
  n = kernel_rem_pio2(tx, y, e0, nx, 2, two_over_pi);
  if (hx < 0) {
    y[0] = -y[0];
    y[1] = -y[1];
    return -n;
  }
  return n;
}

inline double kernel_cos(double x, double y) {
  static const double
      one = 1.00000000000000000000e+00, /* 0x3FF00000, 0x00000000 */
      C1 = 4.16666666666666019037e-02,  /* 0x3FA55555, 0x5555554C */
      C2 = -1.38888888888741095749e-03, /* 0xBF56C16C, 0x16C15177 */
      C3 = 2.48015872894767294178e-05,  /* 0x3EFA01A0, 0x19CB1590 */
      C4 = -2.75573143513906633035e-07, /* 0xBE927E4F, 0x809C52AD */
      C5 = 2.08757232129817482790e-09,  /* 0x3E21EE9E, 0xBDB4B1C4 */
      C6 = -1.13596475577881948265e-11; /* 0xBDA8FAE9, 0xBE8838D4 */

  double a, iz, z, r, qx;
  int32_t ix;
  GET_HIGH_WORD(ix, x);
  ix &= 0x7FFFFFFF;                           /* ix = |x|'s high word*/
  if (ix < 0x3E400000) {                      /* if x < 2**27 */
    if (static_cast<int>(x) == 0) return one; /* generate inexact */
  }
  z = x * x;
  r = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
  if (ix < 0x3FD33333) { /* if |x| < 0.3 */
    return one - (0.5 * z - (z * r - x * y));
  } else {
    if (ix > 0x3FE90000) { /* x > 0.78125 */
      qx = 0.28125;
    } else {
      INSERT_WORDS(qx, ix - 0x00200000, 0); /* x/4 */
    }
    iz = 0.5 * z - qx;
    a = one - qx;
    return a - (iz - (z * r - x * y));
  }
}

int kernel_rem_pio2(double *x, double *y, int e0, int nx, int prec,
                      const int32_t *ipio2) {
  /* Constants:
   * The hexadecimal values are the intended ones for the following
   * constants. The decimal values may be used, provided that the
   * compiler will convert from decimal to binary accurately enough
   * to produce the hexadecimal values shown.
   */
  static const int init_jk[] = {2, 3, 4, 6}; /* initial value for jk */

  static const double PIo2[] = {
      1.57079625129699707031e+00, /* 0x3FF921FB, 0x40000000 */
      7.54978941586159635335e-08, /* 0x3E74442D, 0x00000000 */
      5.39030252995776476554e-15, /* 0x3CF84698, 0x80000000 */
      3.28200341580791294123e-22, /* 0x3B78CC51, 0x60000000 */
      1.27065575308067607349e-29, /* 0x39F01B83, 0x80000000 */
      1.22933308981111328932e-36, /* 0x387A2520, 0x40000000 */
      2.73370053816464559624e-44, /* 0x36E38222, 0x80000000 */
      2.16741683877804819444e-51, /* 0x3569F31D, 0x00000000 */
  };

  static const double
      zero = 0.0,
      one = 1.0,
      two24 = 1.67772160000000000000e+07,  /* 0x41700000, 0x00000000 */
      twon24 = 5.96046447753906250000e-08; /* 0x3E700000, 0x00000000 */

  int32_t jz, jx, jv, jp, jk, carry, n, iq[20], i, j, k, m, q0, ih;
  double z, fw, f[20] = {}, fq[20] = {}, q[20] = {};   // zeroed: silences -Wmaybe-uninitialized (only written entries are read)

  /* initialize jk*/
  jk = init_jk[prec];
  jp = jk;

  /* determine jx,jv,q0, note that 3>q0 */
  jx = nx - 1;
  jv = (e0 - 3) / 24;
  if (jv < 0) jv = 0;
  q0 = e0 - 24 * (jv + 1);

  /* set up f[0] to f[jx+jk] where f[jx+jk] = ipio2[jv+jk] */
  j = jv - jx;
  m = jx + jk;
  for (i = 0; i <= m; i++, j++) {
    f[i] = (j < 0) ? zero : static_cast<double>(ipio2[j]);
  }

  /* compute q[0],q[1],...q[jk] */
  for (i = 0; i <= jk; i++) {
    for (j = 0, fw = 0.0; j <= jx; j++) fw += x[j] * f[jx + i - j];
    q[i] = fw;
  }

  jz = jk;
recompute:
  /* distill q[] into iq[] reversingly */
  for (i = 0, j = jz, z = q[jz]; j > 0; i++, j--) {
    fw = static_cast<double>(static_cast<int32_t>(twon24 * z));
    iq[i] = static_cast<int32_t>(z - two24 * fw);
    z = q[j - 1] + fw;
  }

  /* compute n */
  z = std::scalbn(z, q0);           /* actual value of z */
  z -= 8.0 * std::floor(z * 0.125); /* trim off integer >= 8 */
  n = static_cast<int32_t>(z);
  z -= static_cast<double>(n);
  ih = 0;
  if (q0 > 0) { /* need iq[jz-1] to determine n */
    i = (iq[jz - 1] >> (24 - q0));
    n += i;
    iq[jz - 1] -= i << (24 - q0);
    ih = iq[jz - 1] >> (23 - q0);
  } else if (q0 == 0) {
    ih = iq[jz - 1] >> 23;
  } else if (z >= 0.5) {
    ih = 2;
  }

  if (ih > 0) { /* q > 0.5 */
    n += 1;
    carry = 0;
    for (i = 0; i < jz; i++) { /* compute 1-q */
      j = iq[i];
      if (carry == 0) {
        if (j != 0) {
          carry = 1;
          iq[i] = 0x1000000 - j;
        }
      } else {
        iq[i] = 0xFFFFFF - j;
      }
    }
    if (q0 > 0) { /* rare case: chance is 1 in 12 */
      switch (q0) {
        case 1:
          iq[jz - 1] &= 0x7FFFFF;
          break;
        case 2:
          iq[jz - 1] &= 0x3FFFFF;
          break;
      }
    }
    if (ih == 2) {
      z = one - z;
      if (carry != 0) z -= std::scalbn(one, q0);
    }
  }

  /* check if recomputation is needed */
  if (z == zero) {
    j = 0;
    for (i = jz - 1; i >= jk; i--) j |= iq[i];
    if (j == 0) { /* need recomputation */
      for (k = 1; jk >= k && iq[jk - k] == 0; k++) {
        /* k = no. of terms needed */
      }

      for (i = jz + 1; i <= jz + k; i++) { /* add q[jz+1] to q[jz+k] */
        f[jx + i] = ipio2[jv + i];
        for (j = 0, fw = 0.0; j <= jx; j++) fw += x[j] * f[jx + i - j];
        q[i] = fw;
      }
      jz += k;
      goto recompute;
    }
  }

  /* chop off zero terms */
  if (z == 0.0) {
    jz -= 1;
    q0 -= 24;
    while (iq[jz] == 0) {
      jz--;
      q0 -= 24;
    }
  } else { /* break z into 24-bit if necessary */
    z = std::scalbn(z, -q0);
    if (z >= two24) {
      fw = static_cast<double>(static_cast<int32_t>(twon24 * z));
      iq[jz] = z - two24 * fw;
      jz += 1;
      q0 += 24;
      iq[jz] = fw;
    } else {
      iq[jz] = z;
    }
  }

  /* convert integer "bit" chunk to floating-point value */
  fw = std::scalbn(one, q0);
  for (i = jz; i >= 0; i--) {
    q[i] = fw * iq[i];
    fw *= twon24;
  }

  /* compute PIo2[0,...,jp]*q[jz,...,0] */
  for (i = jz; i >= 0; i--) {
    for (fw = 0.0, k = 0; k <= jp && k <= jz - i; k++) fw += PIo2[k] * q[i + k];
    fq[jz - i] = fw;
  }

  /* compress fq[] into y[] */
  switch (prec) {
    case 0:
      fw = 0.0;
      for (i = jz; i >= 0; i--) fw += fq[i];
      y[0] = (ih == 0) ? fw : -fw;
      break;
    case 1:
    case 2:
      fw = 0.0;
      for (i = jz; i >= 0; i--) fw += fq[i];
      y[0] = (ih == 0) ? fw : -fw;
      fw = fq[0] - fw;
      for (i = 1; i <= jz; i++) fw += fq[i];
      y[1] = (ih == 0) ? fw : -fw;
      break;
    case 3: /* painful */
      for (i = jz; i > 0; i--) {
        fw = fq[i - 1] + fq[i];
        fq[i] += fq[i - 1] - fw;
        fq[i - 1] = fw;
      }
      for (i = jz; i > 1; i--) {
        fw = fq[i - 1] + fq[i];
        fq[i] += fq[i - 1] - fw;
        fq[i - 1] = fw;
      }
      for (fw = 0.0, i = jz; i >= 2; i--) fw += fq[i];
      if (ih == 0) {
        y[0] = fq[0];
        y[1] = fq[1];
        y[2] = fw;
      } else {
        y[0] = -fq[0];
        y[1] = -fq[1];
        y[2] = -fw;
      }
  }
  return n & 7;
}

inline double kernel_sin(double x, double y, int iy) {
  static const double
      half = 5.00000000000000000000e-01, /* 0x3FE00000, 0x00000000 */
      S1 = -1.66666666666666324348e-01,  /* 0xBFC55555, 0x55555549 */
      S2 = 8.33333333332248946124e-03,   /* 0x3F811111, 0x1110F8A6 */
      S3 = -1.98412698298579493134e-04,  /* 0xBF2A01A0, 0x19C161D5 */
      S4 = 2.75573137070700676789e-06,   /* 0x3EC71DE3, 0x57B1FE7D */
      S5 = -2.50507602534068634195e-08,  /* 0xBE5AE5E6, 0x8A2B9CEB */
      S6 = 1.58969099521155010221e-10;   /* 0x3DE5D93A, 0x5ACFD57C */

  double z, r, v;
  int32_t ix;
  GET_HIGH_WORD(ix, x);
  ix &= 0x7FFFFFFF;      /* high word of x */
  if (ix < 0x3E400000) { /* |x| < 2**-27 */
    if (static_cast<int>(x) == 0) return x;
  } /* generate inexact */
  z = x * x;
  v = z * x;
  r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
  if (iy == 0) {
    return x + v * (S1 + z * r);
  } else {
    return x - ((z * (half * y - v * r) - y) - v * S1);
  }
}

double js_cos(double x) {
  double y[2], z = 0.0;
  int32_t n, ix;

  /* High word of x. */
  GET_HIGH_WORD(ix, x);

  /* |x| ~< pi/4 */
  ix &= 0x7FFFFFFF;
  if (ix <= 0x3FE921FB) {
    return kernel_cos(x, z);
  } else if (ix >= 0x7FF00000) {
    /* cos(Inf or NaN) is NaN */
    return x - x;
  } else {
    /* argument reduction needed */
    n = ieee754_rem_pio2(x, y);
    switch (n & 3) {
      case 0:
        return kernel_cos(y[0], y[1]);
      case 1:
        return -kernel_sin(y[0], y[1], 1);
      case 2:
        return -kernel_cos(y[0], y[1]);
      default:
        return kernel_sin(y[0], y[1], 1);
    }
  }
}

double js_sin(double x) {
  double y[2], z = 0.0;
  int32_t n, ix;

  /* High word of x. */
  GET_HIGH_WORD(ix, x);

  /* |x| ~< pi/4 */
  ix &= 0x7FFFFFFF;
  if (ix <= 0x3FE921FB) {
    return kernel_sin(x, z, 0);
  } else if (ix >= 0x7FF00000) {
    /* sin(Inf or NaN) is NaN */
    return x - x;
  } else {
    /* argument reduction needed */
    n = ieee754_rem_pio2(x, y);
    switch (n & 3) {
      case 0:
        return kernel_sin(y[0], y[1], 1);
      case 1:
        return kernel_cos(y[0], y[1]);
      case 2:
        return -kernel_sin(y[0], y[1], 1);
      default:
        return -kernel_cos(y[0], y[1]);
    }
  }
}
#undef EXTRACT_WORDS
#undef GET_HIGH_WORD
#undef GET_LOW_WORD
#undef INSERT_WORDS
#undef SET_HIGH_WORD
#undef SET_LOW_WORD

} // namespace fdlibm
// ---------------------------------------------------------------------------------------

constexpr double PI = 3.141592653589793;   // Math.PI
constexpr double NaN_ = std::numeric_limits<double>::quiet_NaN();

/// JS Math.min(a, b) / Math.max(a, b): NaN if either is NaN; -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN_;
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN_;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}

/// JS ToInt32 (what `^` does to its operands): truncate, wrap mod 2^32.
inline int32_t toInt32(double v) {
    if (!std::isfinite(v)) return 0;
    double t = std::trunc(v);
    t = std::fmod(t, 4294967296.0);
    if (t < 0) t += 4294967296.0;
    return int32_t(uint32_t(t));
}

/// +x.toFixed(9). toFixed picks the n nearest x * 1e9 on the EXACT decimal value of x,
/// ties to the larger magnitude; glibc's %.9f breaks exact ties to even, so round from
/// the exact expansion (as Cutout.cpp's toFixed9). A negative input that rounds to zero
/// keeps its sign (+"-0.000000000" is -0); -0 itself prints "0.000000000" (it is not < 0).
double toFixed9(double x) {
    if (!std::isfinite(x) || std::fabs(x) >= 1e21) return x;   // toFixed falls back to ToString(x)
    static thread_local char buf[1200];
    std::snprintf(buf, sizeof(buf), "%.1100f", std::fabs(x)); // exact: a double has <= 1074 fraction digits
    char *dot = std::strchr(buf, '.');
    std::string keep(buf, size_t(dot - buf) + 10);           // integer part, '.', 9 digits
    if (dot[10] >= '5') {                                     // round the magnitude up
        int i = int(keep.size()) - 1;
        for (; i >= 0; i--) {
            if (keep[i] == '.') continue;
            if (keep[i] == '9') { keep[i] = '0'; continue; }
            keep[i]++;
            break;
        }
        if (i < 0) keep.insert(keep.begin(), '1');
    }
    const double r = std::strtod(keep.c_str(), nullptr);
    return x < 0 ? -r : r;
}

struct Outline {
    std::vector<double> segs;   // [x0, y0, x1, y1, ...]
    double length = 0;
};

/**
 * The part's first-layer outline: its section at the layer's mid-height (where a
 * slicer cuts), as a flat [x0, y0, x1, y1, ...] segment list, plus its length.
 */
Outline firstLayerOutline(const SurfTris &partTris, double zc) {
    Outline o;
    for (size_t i = 0; i < partTris.size(); i += 9) {
        double pts[3][2];
        int n = 0;
        for (int a = 0; a < 3; a++) {
            const size_t p = i + a * 3, q = i + ((a + 1) % 3) * 3;
            const double za = partTris[p + 2] - zc, zb = partTris[q + 2] - zc;
            if ((za < 0) == (zb < 0)) continue;
            const double t = za / (za - zb);
            pts[n][0] = partTris[p] + t * (partTris[q] - partTris[p]);
            pts[n][1] = partTris[p + 1] + t * (partTris[q + 1] - partTris[p + 1]);
            n++;
        }
        if (n != 2) continue;
        o.segs.push_back(pts[0][0]); o.segs.push_back(pts[0][1]);
        o.segs.push_back(pts[1][0]); o.segs.push_back(pts[1][1]);
        o.length += hypot2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]);
    }
    return o;
}

struct Frame { double cx, cy, ax, ay, bx, by, r1, r2; };

/**
 * The BRIM-STYLE pad (PAD.style 'light', and 'custom'): the same oval, `H` tall (one
 * layer for 'light'), standing `g` off the part's first-layer outline. `grab` raises
 * (or, negative, lowers) its top against the part's underside; 'light' uses 0.
 *
 * The gap has to come through the SLICER, which merges any slice gap under ~0.098mm,
 * so it is held exactly on the first layer: the pad's top there ramps LINEARLY with
 * the true distance from the first-layer outline, crossing mid-height exactly at `g`.
 * It is also held below the lowest underside within `g` (plus `grab`).
 *
 * The mesh is columns across the oval's long axis, each running rim to rim with the
 * same number of rows, at `brimCell` spacing. The two tips close with a fan; the
 * bottom is one flat fan (the ellipse is convex).
 */
PadResult brimPad(const SurfTris &partTris, const std::vector<Vec2d> &contact, const Frame &e, double h, double g,
                  double grab, double layerH, const std::vector<double> &outline, Tris &out) {
    const double cx = e.cx, cy = e.cy, ax = e.ax, ay = e.ay, bx = e.bx, by = e.by, r1 = e.r1, r2 = e.r2;
    const double H = std::isfinite(h) && h > 0.05 ? h : FIN.tineH;
    const double cell = PAD.brimCell;
    const double zc = layerH / 2;                 // first layer's mid-height: the slicer's cut
    std::vector<std::array<double, 2>> ring;
    if (g > 0) for (int k = 0; k < 16; k++) ring.push_back({g * fdlibm::js_cos(k * PI / 8), g * fdlibm::js_sin(k * PI / 8)});

    // The part's first-layer outline (firstLayerOutline, cut at zc), bucketed.
    const double B = 0.5, reach = jsMax(g, 0) + 2 * cell;
    std::unordered_map<int32_t, std::vector<double>> bucket;
    // JS `i * 73856093 ^ j * 19349663`: both products are exact doubles, then ToInt32.
    auto key = [](double i, double j) { return toInt32(i * 73856093) ^ toInt32(j * 19349663); };
    for (size_t i = 0; i < outline.size(); i += 4) {
        const double x0 = outline[i], y0 = outline[i + 1], x1 = outline[i + 2], y1 = outline[i + 3];
        for (double bi = std::floor((jsMin(x0, x1) - reach) / B); bi <= std::floor((jsMax(x0, x1) + reach) / B); bi++) {
            for (double bj = std::floor((jsMin(y0, y1) - reach) / B); bj <= std::floor((jsMax(y0, y1) + reach) / B); bj++) {
                std::vector<double> &arr = bucket[key(bi, bj)];
                arr.push_back(x0); arr.push_back(y0); arr.push_back(x1); arr.push_back(y1);
            }
        }
    }
    // Signed distance to the outline, capped at `reach` (negative = inside it).
    auto dist = [&](double x, double y) {
        const auto it = bucket.find(key(std::floor(x / B), std::floor(y / B)));
        double d2 = reach * reach;
        if (it != bucket.end()) {
            const std::vector<double> &arr = it->second;
            for (size_t k = 0; k < arr.size(); k += 4) {
                const double x0 = arr[k], y0 = arr[k + 1], dx = arr[k + 2] - x0, dy = arr[k + 3] - y0;
                const double l2 = dx * dx + dy * dy;
                const double t = l2 > 0 ? jsMax(0, jsMin(1, ((x - x0) * dx + (y - y0) * dy) / l2)) : 0;
                const double ex = x0 + t * dx - x, ey = y0 + t * dy - y;
                d2 = jsMin(d2, ex * ex + ey * ey);
            }
        }
        const double d = std::sqrt(d2);
        const auto low = surfaceZAt(partTris, x, y);
        return low && *low < zc ? -d : d;
    };
    // Ramp slope: one cell below the crossing is still above the 0.05 floor, so the
    // two vertices around the crossing are never clamped and interpolate exactly.
    const double k = jsMax(0.01, zc - 0.05) / cell;
    auto top = [&](double x, double y) {
        double low = surfaceZAt(partTris, x, y).value_or(INF);
        for (const auto &d : ring) low = jsMin(low, surfaceZAt(partTris, x + d[0], y + d[1]).value_or(INF));
        // With no gap asked for (Custom gap 0) grip alone decides -- a bite is a bite.
        const double ramp = g > 0 ? zc + k * (dist(x, y) - g) : INF;
        return jsMax(0.05, jsMin(jsMin(H, low + grab), ramp));
    };
    auto at = [&](double s, double t) { return Vec2d(cx + ax * s + bx * t, cy + ay * s + by * t); };
    auto vert = [&](double s, double t) { const Vec2d p = at(s, t); return Vec3d(p[0], p[1], top(p[0], p[1])); };

    // Rows are FRACTIONS of each column's half-width, shared by every column so the
    // grid stays structured. They are fine (brimCell) only across the band where the
    // part comes within the pad's height -- found by a coarse scan -- and 1.2mm apart
    // elsewhere.
    const int nS = int(jsMax(3, std::ceil((2 * r1) / cell) + 1));
    auto half = [&](double s) { const double q = s / r1; return r2 * std::sqrt(jsMax(0, 1 - q * q)); };
    double band = 0;
    for (double s = -r1 + 0.25; s < r1; s += 0.5) {
        const double te = half(s);
        if (te < cell) continue;
        for (double t = 0; t <= te; t += cell) {
            for (const double sg : {1.0, -1.0}) {
                const Vec2d p = at(s, sg * t);
                if (top(p[0], p[1]) < H - 1e-9) band = jsMax(band, t / te);
            }
        }
    }
    const double fine = cell / r2, coarse = jsMax(fine, 1.2 / r2);
    const double edge = jsMin(1, band + 2 * fine);
    // new Set([-1, 1]) ... [...fr].sort((a, b) => a - b): a Set stores -0 as +0 and
    // dedupes by value, so the rows are the distinct values, ascending.
    std::vector<double> rows{-1, 1};
    auto addRow = [&](double v) { rows.push_back(v == 0 ? 0.0 : v); };
    for (double f = 0; f <= edge + 1e-12; f += fine) { addRow(toFixed9(jsMin(f, 1))); addRow(-jsMin(toFixed9(f), 1)); }
    for (double f = edge + coarse; f < 1; f += coarse) { addRow(toFixed9(f)); addRow(-(toFixed9(f))); }
    std::stable_sort(rows.begin(), rows.end(), [](double a, double b) { return a - b < 0; });
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    const int nT = int(rows.size());
    std::vector<std::vector<Vec3d>> cols;   // interior columns i = 1 .. nS-2
    for (int i = 1; i < nS - 1; i++) {
        const double s = -r1 + (2 * r1 * i) / (nS - 1);
        const double te = half(s);
        std::vector<Vec3d> col;
        col.reserve(rows.size());
        for (const double f : rows) col.push_back(vert(s, te * f));
        cols.push_back(std::move(col));
    }
    const Vec3d tipL = vert(-r1, 0), tipR = vert(r1, 0);
    Tris tris;
    auto tri = [&](const Vec3d &a, const Vec3d &b, const Vec3d &d) { tris.push_back(a); tris.push_back(b); tris.push_back(d); };
    for (int j = 0; j < nT - 1; j++) {
        tri(tipL, cols[0][j + 1], cols[0][j]);
        const std::vector<Vec3d> &L = cols[cols.size() - 1];
        tri(tipR, L[j], L[j + 1]);
    }
    for (size_t i = 0; i + 1 < cols.size(); i++) {
        const std::vector<Vec3d> &A = cols[i], &Bc = cols[i + 1];
        // wound like the tip fans, side wall and bottom
        for (int j = 0; j < nT - 1; j++) { tri(A[j], Bc[j + 1], Bc[j]); tri(A[j], A[j + 1], Bc[j + 1]); }
    }
    // boundary ring (tipL, the +t rim left to right, tipR, the -t rim right to left),
    // then its side wall and the flat bottom fan
    std::vector<Vec3d> rim;
    rim.push_back(tipL);
    for (const auto &col : cols) rim.push_back(col[nT - 1]);
    rim.push_back(tipR);
    for (auto it = cols.rbegin(); it != cols.rend(); ++it) rim.push_back((*it)[0]);
    std::vector<Vec3d> flat;
    for (const Vec3d &v : rim) flat.emplace_back(v[0], v[1], 0);
    const Vec3d b0(cx, cy, 0);
    for (size_t k2 = 0; k2 < rim.size(); k2++) {
        const size_t m = (k2 + 1) % rim.size();
        tri(rim[k2], flat[k2], flat[m]); tri(rim[k2], flat[m], rim[m]);
        tri(b0, flat[m], flat[k2]);
    }
    // One winding was picked above; flip the lot if it came out inside-out.
    double vol = 0;
    for (size_t i = 0; i < tris.size(); i += 3) {
        const Vec3d &p = tris[i], &q = tris[i + 1], &r = tris[i + 2];
        vol += p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) + p[2] * (q[0] * r[1] - q[1] * r[0]);
    }
    out.reserve(out.size() + tris.size());
    for (size_t i = 0; i < tris.size(); i += 3) {
        if (vol < 0) { out.push_back(tris[i]); out.push_back(tris[i + 2]); out.push_back(tris[i + 1]); }
        else { out.push_back(tris[i]); out.push_back(tris[i + 1]); out.push_back(tris[i + 2]); }
    }
    PadResult res;
    res.r1 = r1; res.r2 = r2;
    res.cells = int(cols.size()) * nT;
    res.height = H;
    res.points = int(contact.size());
    res.oval = true;
    res.style = PAD.style == "custom" ? "custom" : "light";
    return res;
}

} // namespace

/**
 * A breakaway pad under the part's bed contact.
 *
 * A part tilted into a strong orientation rests on an EDGE, so its bed contact is
 * near zero and it peels off before the fins have anything to hold. The pad is a
 * wide, thin footprint -- thin enough to cut off, wide enough to stick.
 *
 * It is fed the part's lowest VERTICES for its outline AND the seated part triangles,
 * so its TOP can conform to the part instead of welding to it: a radial mesh of the
 * contact ellipse, each vertex given its own top height. OUTBOARD (no part overhead)
 * rises to the full padH; under the part it caps at part_low + grab.
 */
std::optional<PadResult> buildPad(const std::vector<Vec2d> &contact, const SurfTris &partTris, Tris &out, double layerH) {
    if (contact.size() < 3) return std::nullopt;

    double cx = 0, cy = 0;
    for (const Vec2d &p : contact) { cx += p[0]; cy += p[1]; }
    cx /= double(contact.size()); cy /= double(contact.size());

    // principal axis of the contact, so a long thin edge gets a long thin pad
    // instead of a circle sized to its length
    double sxx = 0, sxy = 0, syy = 0;
    for (const Vec2d &p : contact) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr / 2 + std::sqrt(jsMax(0, (tr * tr) / 4 - det));
    double ax = sxy, ay = lam - sxx;
    if (hypot2(ax, ay) < 1e-9) { ax = 1; ay = 0; }
    const double an = hypot2(ax, ay); ax /= an; ay /= an;
    const double bx = -ay, by = ax;

    double e1 = 0, e2 = 0;
    for (const Vec2d &p : contact) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        e1 = jsMax(e1, std::abs(dx * ax + dy * ay));
        e2 = jsMax(e2, std::abs(dx * bx + dy * by));
    }
    const double margin = PAD.style == "custom" ? PAD.custom.margin : FIN.padMargin;
    const double r1 = e1 + margin, r2 = e2 + margin;

    // Height rule per vertex: OUTBOARD (no part overhead) rises to the full padH for
    // bed grip; UNDER the part it caps at part_low + grab.
    double sureGrab = PAD.grab;   // raised to >= 0 below when Light swaps to Sure hold
    auto conform = [&](double x, double y) {
        const auto low = surfaceZAt(partTris, x, y);
        if (!low) return FIN.padH;
        // grab may be negative (a PETG gap). Floor at 0.05 so every column stays
        // positive -- the disc watertight -- and a gap pad still kisses the resting edge.
        return jsMax(0.05, jsMin(FIN.padH, *low + sureGrab));
    };
    const Frame frame{cx, cy, ax, ay, bx, by, r1, r2};
    const double L1 = std::isfinite(layerH) && layerH > 0.05 ? layerH : FIN.tineH;
    // Light holds by first-layer squish along the part's first-layer OUTLINE. Below
    // PAD.minGripOutline of outline (a point, a cone tip, a small nub), Auto builds
    // Sure hold and says so (autoSure); otherwise Light. An explicit Light or Custom
    // is the user's call; `smallFoot` rides along so the UI can warn.
    const Outline outline = firstLayerOutline(partTris, L1 / 2);
    const bool smallFoot = outline.length < PAD.minGripOutline;
    auto tag = [&](PadResult pad) { pad.smallFoot = smallFoot; pad.outline = outline.length; return pad; };
    const bool light = PAD.style == "light" || (PAD.style == "auto" && !smallFoot);
    if (light) return tag(brimPad(partTris, contact, frame, L1, PAD.brimGap, 0, L1, outline.segs, out));
    if (PAD.style == "custom") {
        const PadConfig::Custom &c = PAD.custom;
        return tag(brimPad(partTris, contact, frame, c.h, c.gap, c.grip, L1, outline.segs, out));
    }
    const bool autoSure = PAD.style == "auto";
    // The swap exists to HOLD a part that barely touches the plate, so the pad must
    // meet it: flush or the material's tack, never a gap.
    if (autoSure) sureGrab = jsMax(PAD.grab, 0);
    const int nTheta = FIN.padSegs;
    const int nRing = int(jsMax(2, std::ceil(jsMax(r1, r2) / PAD.cell)));

    // Ring/segment vertex in world XY, at radial fraction `fr` and angle index `j`.
    auto vAt = [&](double fr, int j) {
        const double a = (2 * PI * j) / nTheta;
        const double s = r1 * fr * fdlibm::js_cos(a), t = r2 * fr * fdlibm::js_sin(a);
        return Vec2d(cx + ax * s + bx * t, cy + ay * s + by * t);
    };
    // Precompute the vertex ring positions + their conformed tops once (reused by the
    // top cap, the side wall, and the flat bottom), so every shared edge is keyed from
    // bit-identical coordinates and the soup stays edge-manifold.
    std::vector<std::vector<Vec3d>> V;   // V[i][j] = [x, y, topZ], i in 0..nRing, j in 0..nTheta-1
    for (int i = 0; i <= nRing; i++) {
        const double fr = double(i) / nRing;
        std::vector<Vec3d> row;
        row.reserve(size_t(nTheta));
        for (int j = 0; j < nTheta; j++) {
            const Vec2d p = vAt(fr, j);
            row.emplace_back(p[0], p[1], conform(p[0], p[1]));
        }
        V.push_back(std::move(row));
    }
    const Vec3d centreTop = V[0][0];   // ring 0 collapses to the centre (fr = 0)
    auto tri = [&](const Vec3d &a, const Vec3d &b, const Vec3d &c) { out.push_back(a); out.push_back(b); out.push_back(c); };
    double maxTop = 0;
    for (const auto &row : V) for (const Vec3d &v : row) if (v[2] > maxTop) maxTop = v[2];

    for (int j = 0; j < nTheta; j++) {
        const int jn = (j + 1) % nTheta;
        // TOP surface (normal up). Inner fan from the centre, then quad strips outward.
        tri(centreTop, V[1][j], V[1][jn]);
        for (int i = 1; i < nRing; i++) {
            tri(V[i][j], V[i + 1][j], V[i + 1][jn]);
            tri(V[i][j], V[i + 1][jn], V[i][jn]);
        }
        // BOTTOM surface at z=0 (normal down: reverse the top winding).
        const Vec3d b0(cx, cy, 0);
        auto bi = [&](int i, int k) { return Vec3d(V[i][k][0], V[i][k][1], 0); };
        tri(b0, bi(1, jn), bi(1, j));
        for (int i = 1; i < nRing; i++) {
            tri(bi(i, j), bi(i + 1, jn), bi(i + 1, j));
            tri(bi(i, j), bi(i, jn), bi(i + 1, jn));
        }
        // SIDE wall around the outer ring, top down to the plate (normal outward).
        const int oR = nRing;
        const Vec3d &tj = V[oR][j], &tjn = V[oR][jn];
        const Vec3d bj(tj[0], tj[1], 0), bjn(tjn[0], tjn[1], 0);
        tri(tj, bj, bjn);
        tri(tj, bjn, tjn);
    }

    PadResult res;
    res.r1 = r1; res.r2 = r2;
    res.cells = nTheta * nRing;
    res.height = maxTop;
    res.points = int(contact.size());
    res.oval = true;
    res.style = "sure";
    res.autoSure = autoSure;
    res.smallFoot = smallFoot;
    res.outline = outline.length;
    return res;
}

}} // namespace
