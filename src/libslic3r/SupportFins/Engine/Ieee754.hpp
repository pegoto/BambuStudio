// Ieee754.hpp -- JS Math.* transcendentals, bit-exact with Node 22 (V8 12.4.254.21).
//
// V8 does not call the C library for Math.sin/cos/tan/asin/acos/atan/atan2/exp/log/
// pow/cbrt/...: it ships its own fdlibm port (src/base/ieee754.cc), which differs from
// glibc in the last bit on a few percent of inputs. Every transcendental in Engine/
// goes through these (see CONVENTIONS.md); std::sqrt, floor, ceil, fabs, fmod are
// exact in both and stay as they are. Math.hypot is hypot2/hypot3 in Math.hpp.
#pragma once

namespace Slic3r { namespace SupportFins {

double js_sin(double x);              // Math.sin
double js_cos(double x);              // Math.cos
double js_tan(double x);              // Math.tan
double js_asin(double x);             // Math.asin
double js_acos(double x);             // Math.acos
double js_atan(double x);             // Math.atan
double js_atan2(double y, double x);  // Math.atan2
double js_exp(double x);              // Math.exp
double js_expm1(double x);            // Math.expm1
double js_log(double x);              // Math.log
double js_log1p(double x);            // Math.log1p
double js_log2(double x);             // Math.log2
double js_log10(double x);            // Math.log10
double js_cbrt(double x);             // Math.cbrt
double js_sinh(double x);             // Math.sinh
double js_cosh(double x);             // Math.cosh
double js_tanh(double x);             // Math.tanh
double js_asinh(double x);            // Math.asinh
double js_acosh(double x);            // Math.acosh
double js_atanh(double x);            // Math.atanh
double js_pow(double x, double y);    // Math.pow and the ** operator

}} // namespace Slic3r::SupportFins
