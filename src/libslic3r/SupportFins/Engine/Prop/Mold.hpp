// Port of web/prop/mold.js -- the floor a PART-ATTACHED wall stands on: `floorLine`
// reads it under each station, `moldLine` adds stations where it changes fast so a
// lifted wall's bottom (sweepBetween) traces the part.
#pragma once
#include "../Math.hpp"
#include "Contact.hpp"
#include "Surface.hpp"
#include <cmath>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// One floorLine station. JS: [x, y, z] (welded, footGap 0) or, lifted (footGap > 0),
/// [x, y, z, zN, zP]: zN / zP the floor under the bottom's -w / +w sides (the
/// across-wall `sx` sign), from sideFloors. `p[k]` and `p.length()` read it the way
/// the JS indexes the array (k = 3, 4 on a 3-element station reads NaN, JS undefined).
struct FloorPt {
    double x = 0, y = 0, z = 0;
    bool   sides = false;     // JS length === 5
    double zN = 0, zP = 0;    // [3], [4]
    FloorPt() = default;
    FloorPt(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    FloorPt(double x_, double y_, double z_, double zN_, double zP_) : x(x_), y(y_), z(z_), sides(true), zN(zN_), zP(zP_) {}
    size_t length() const { return sides ? 5 : 3; }
    double operator[](int k) const {
        switch (k) {
        case 0: return x; case 1: return y; case 2: return z;
        case 3: return sides ? zN : NAN; case 4: return sides ? zP : NAN;
        default: return NAN;
        }
    }
};
using FloorLine = std::vector<FloorPt>;

/// For each station of `topLine`, the HIGHEST part surface strictly below
/// `top z - margin`, or 0 (the plate) where nothing intervenes, read across the
/// bottom's width (and, with `mold`, footGap ahead and behind along the wall for the
/// side floors). Lifted (PROP.footGap > 0) each station also carries its side floors.
FloorLine floorLine(const ContactLine &topLine, const SurfTris &tris, double margin = 1.0, bool mold = false);

/// moldLine's result (JS `{ top, floor }`) for sweepBetween.
struct Mold {
    ContactLine top;
    FloorLine   floor;
};

/// MOLD a lifted wall's bottom to the part: floorLine's stations plus midpoints wherever
/// the floor (level or either side) steps more than MOLD_STEP between two of them, down
/// to MOLD_MIN apart, up to 8 passes. Welded (footGap 0): { top, floorLine(top) }.
Mold moldLine(ContactLine top, const SurfTris &tris);

inline constexpr double MAX_DROP = 2.75, MAX_HANG = 1.0;   // sideFloors' tilt limits
inline constexpr double MOLD_STEP = 0.25, MOLD_MIN = 0.1;

}} // namespace
