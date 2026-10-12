// Port of web/solids.js -- closed-solid emitters shared by the wall builders and
// the cutouts. Each pushes outward-wound triangles, as vertex triples, onto `out`.
#pragma once
#include "Math.hpp"
#include <functional>

namespace Slic3r { namespace SupportFins {

/// A ring of k vertices; consecutive rings are bridged with quads, ends fan-capped.
using Section = std::vector<Vec3d>;

/// ribbon(secs, out): bridge a run of cross-sections into one closed solid.
inline void ribbon(const std::vector<Section> &secs, Tris &out) {
    const size_t k = secs[0].size();
    auto tri = [&](const Vec3d &a, const Vec3d &b, const Vec3d &c) { out.push_back(a); out.push_back(c); out.push_back(b); };
    for (size_t i = 0; i + 1 < secs.size(); i++) {
        for (size_t j = 0; j < k; j++) {
            const size_t j2 = (j + 1) % k;
            tri(secs[i][j], secs[i][j2], secs[i + 1][j2]);
            tri(secs[i][j], secs[i + 1][j2], secs[i + 1][j]);
        }
    }
    for (size_t j = 1; j + 1 < k; j++) { // end caps
        tri(secs[0][0], secs[0][j + 1], secs[0][j]);
        const Section &e = secs.back();
        tri(e[0], e[j], e[j + 1]);
    }
}

/// A 2-D polygon vertex (a, b) in the plane the frame callback P maps.
using Poly2 = std::vector<Vec2d>;
/// P(a, b, c) -> world point, the right-handed frame (a, b, c) of the caller.
using Frame = std::function<Vec3d(double, double, double)>;

/// boxExtrude(poly, lo, hi, P, out): extrude a CCW polygon from c = lo to c = hi.
inline void boxExtrude(const Poly2 &poly, double lo, double hi, const Frame &P, Tris &out) {
    const size_t n = poly.size();
    std::vector<Vec3d> vlo, vhi; vlo.reserve(n); vhi.reserve(n);
    for (const auto &p : poly) vlo.push_back(P(p.x(), p.y(), lo));
    for (const auto &p : poly) vhi.push_back(P(p.x(), p.y(), hi));
    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        out.push_back(vlo[i]); out.push_back(vlo[j]); out.push_back(vhi[j]);
        out.push_back(vlo[i]); out.push_back(vhi[j]); out.push_back(vhi[i]);
    }
    for (size_t i = 1; i + 1 < n; i++) {
        out.push_back(vhi[0]); out.push_back(vhi[i]); out.push_back(vhi[i + 1]);
        out.push_back(vlo[0]); out.push_back(vlo[i + 1]); out.push_back(vlo[i]);
    }
}

/// loftExtrude(polyLo, polyHi, lo, hi, P, out): boxExtrude with a different
/// (same-size) outline at the top.
inline void loftExtrude(const Poly2 &polyLo, const Poly2 &polyHi, double lo, double hi, const Frame &P, Tris &out) {
    const size_t n = polyLo.size();
    std::vector<Vec3d> vlo, vhi; vlo.reserve(n); vhi.reserve(n);
    for (const auto &p : polyLo) vlo.push_back(P(p.x(), p.y(), lo));
    for (const auto &p : polyHi) vhi.push_back(P(p.x(), p.y(), hi));
    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        out.push_back(vlo[i]); out.push_back(vlo[j]); out.push_back(vhi[j]);
        out.push_back(vlo[i]); out.push_back(vhi[j]); out.push_back(vhi[i]);
    }
    for (size_t i = 1; i + 1 < n; i++) {
        out.push_back(vhi[0]); out.push_back(vhi[i]); out.push_back(vhi[i + 1]);
        out.push_back(vlo[0]); out.push_back(vlo[i + 1]); out.push_back(vlo[i]);
    }
}

}} // namespace
