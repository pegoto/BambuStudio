// Support Fins engine -- shared math types and JS-semantics helpers.
//
// Port of the printfins.com engine (https://github.com/gittrahan/support-fins, MIT,
// (c) Matthew Trahan; fin technique by Slant3D). The C++ mirrors the JS module for
// module, function for function, so a diff against upstream stays readable. See
// ../CONVENTIONS.md before touching anything here.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace Slic3r { namespace SupportFins {

using Vec3d = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vec2d = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;

/// Column-major 3x3 rotation, THREE.Matrix3.elements order:
/// rotated z of (x,y,z) = r[2]*x + r[5]*y + r[8]*z.
using Rot = std::array<double, 9>;
inline constexpr Rot IDENTITY3 = {1, 0, 0, 0, 1, 0, 0, 0, 1};

/// Triangle soup, 9 doubles per face (JS Float64Array `pos`).
using Soup = std::vector<double>;

/// A list of vertices, three per triangle (JS "array of [x,y,z] triples").
using Tris = std::vector<Vec3d>;

struct Offset { double x = 0, y = 0, z = 0; };

inline constexpr double INF = std::numeric_limits<double>::infinity();

/// JS Math.round: half rounds toward +infinity (Math.round(-2.5) == -2).
inline double js_round(double v) { return std::floor(v + 0.5); }
/// JS Math.trunc / Math.floor / Math.ceil map to std:: directly.
/// JS Math.fround: round to float32 and back.
inline double fround(double v) { return double(float(v)); }
/// JS Math.hypot for 2 and 3 args.
inline double hypot2(double a, double b) { return std::hypot(a, b); }
inline double hypot3(double a, double b, double c) { return std::hypot(a, b, c); }
/// JS Math.sign
inline double js_sign(double v) { return v > 0 ? 1.0 : v < 0 ? -1.0 : 0.0; }

/// seat.js snap(): 1 nm grid.
inline double snap_nm(double v) { return js_round(v * 1e6) / 1e6; }

/// Apply a column-major rotation then an offset (the (rot, offset) seating every
/// module does inline).
inline Vec3d seat(const Rot &r, const Offset &o, double x, double y, double z) {
    return Vec3d(r[0] * x + r[3] * y + r[6] * z + o.x,
                 r[1] * x + r[4] * y + r[7] * z + o.y,
                 r[2] * x + r[5] * y + r[8] * z + o.z);
}
inline Vec3d rotate(const Rot &r, double x, double y, double z) {
    return Vec3d(r[0] * x + r[3] * y + r[6] * z,
                 r[1] * x + r[4] * y + r[7] * z,
                 r[2] * x + r[5] * y + r[8] * z);
}

}} // namespace Slic3r::SupportFins
