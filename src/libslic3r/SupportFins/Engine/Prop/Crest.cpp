// Port of web/prop/crest.js. See Crest.hpp.
#include "Crest.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SupportFins {

namespace {
// The seated part the 'flat' test reads the underside from, and the print's layer
// height: set at each build's entry (buildFins, drawnWall).
thread_local std::shared_ptr<const SurfTris> part;
thread_local double layerH = PropConfig{}.tineH;   // JS: PROP.tineH at module load

inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}

/// Is the part's underside at contact point p (x, y, surface z) flat enough?
bool flatAt(const Vec3d &p) {
    if (!part) return false;
    bool have = false;
    SurfHit best{0, 0};
    for (const SurfHit &h : surfaceHitsAt(*part, p[0], p[1])) {
        const double z = h[0];
        if (!have || std::abs(z - p[2]) < std::abs(best[0] - p[2])) { best = h; have = true; }
    }
    // the face the wall stands under: down-facing, at the contact
    return have && std::abs(best[0] - p[2]) < 0.5
        && -best[1] >= std::cos((PROP.ifaceFlatDeg * M_PI) / 180);
}
} // namespace

bool crestOn() { return PROP.iface != Iface::Off && PROP.ifaceLayers > 0; }

void crestPart(std::shared_ptr<const SurfTris> tris, double layerHeight) {
    part = std::move(tris);
    layerH = (layerHeight != 0 && !std::isnan(layerHeight)) ? layerHeight : PROP.tineH;
}

double crestH() { return PROP.ifaceLayers * layerH; }

std::optional<CrestKinds> crestKinds(const std::vector<Vec3d> &pts) {
    if (!crestOn()) return std::nullopt;
    CrestKinds k;
    k.reserve(pts.size());
    for (const Vec3d &p : pts) k.push_back(flatAt(p) ? CrestKind::Flat : PROP.iface == Iface::All ? CrestKind::Slope : CrestKind::None);
    CrestKinds r(k.size(), CrestKind::None);
    for (size_t i = 0; i < k.size(); i++) {
        const bool prev = i > 0 && k[i - 1] != CrestKind::None;
        const bool next = i + 1 < k.size() && k[i + 1] != CrestKind::None;
        r[i] = k[i] != CrestKind::None && (prev || next) ? k[i] : CrestKind::None;
    }
    return r;
}

void markInterface(Tris &out, size_t from, IfaceTags *tags) {
    if (!tags) return;
    if (tags->size() < out.size()) tags->resize(out.size(), 0);
    for (size_t i = from; i < out.size(); i++) (*tags)[i] = 1;
}

void emitCrest(const std::vector<std::optional<Section>> &sections, Tris &out, IfaceTags *tags) {
    const size_t from = out.size();
    for (size_t i = 0; i < sections.size();) {
        size_t j = i;
        while (j < sections.size() && sections[j]) j++;
        if (j - i >= 2) {
            std::vector<Section> run;
            run.reserve(j - i);
            for (size_t k = i; k < j; k++) run.push_back(*sections[k]);
            ribbon(run, out);
        }
        i = j + 1;
    }
    markInterface(out, from, tags);
}

void dropCollapsed(Tris &out, size_t from, IfaceTags *tags) {
    auto same = [](const Vec3d &a, const Vec3d &b) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; };
    if (tags && tags->size() < out.size()) tags->resize(out.size(), 0);
    size_t w = from;
    for (size_t i = from; i + 2 < out.size(); i += 3) {
        const Vec3d a = out[i], b = out[i + 1], c = out[i + 2];
        if (same(a, b) || same(b, c) || same(a, c)) continue;
        if (tags) { const uint8_t ta = (*tags)[i], tb = (*tags)[i + 1], tc = (*tags)[i + 2];
                    (*tags)[w] = ta; (*tags)[w + 1] = tb; (*tags)[w + 2] = tc; }
        out[w++] = a; out[w++] = b; out[w++] = c;
    }
    if (w < out.size()) out.resize(w);   // JS out.length = w (only ever shrinks here)
    if (tags && tags->size() > out.size()) tags->resize(out.size());
}

InterfaceSplit splitInterface(const Tris &tris, const IfaceTags *tags) {
    InterfaceSplit s;
    for (size_t i = 0; i + 2 < tris.size(); i += 3) {
        const bool isIface = tags && i < tags->size() && (*tags)[i] == 1;
        Tris &dst = isIface ? s.iface : s.body;
        dst.push_back(tris[i]); dst.push_back(tris[i + 1]); dst.push_back(tris[i + 2]);
        if (isIface) s.ifaceTags.insert(s.ifaceTags.end(), 3, uint8_t(1));
    }
    return s;
}

Section CrestCut::ring(const Frame2 &P) const {
    return {P(+wb, cb), P(+wt, top), P(-wt, top), P(-wb, cb)};
}

std::optional<CrestCut> crestCut(double top, double ztip, double floor, CrestKind kind, double th, double tip) {
    if (kind == CrestKind::None) return std::nullopt;
    const double cut = jsMax(top - crestH(), (floor + top) / 2);
    const double cb = cut - CREST_OVERLAP;
    CrestCut c;
    c.cut = cut; c.cb = cb; c.top = top;
    if (kind == CrestKind::Flat) {
        c.zt = cut; c.wc = th / 2;
        c.wb = th / 2; c.wt = th / 2;
        return c;
    }
    const double zt = jsMin(ztip, cut - jsMin(0.05, (cut - floor) / 2));
    auto half = [&](double z) { return th / 2 - ((th - tip) / 2) * (jsMax(0, z - zt) / jsMax(1e-6, top - zt)); };
    const double wb = half(cb);
    c.zt = zt; c.wc = half(cut);
    c.wb = wb; c.wt = tip / 2;
    return c;
}

double flatCut(double top) { return jsMax(top - crestH(), top / 2); }

Section flatCrestRing(const Frame2 &P, double top, double half) {
    const double cb = flatCut(top) - CREST_OVERLAP;
    return {P(+half, cb), P(+half, top), P(-half, top), P(-half, cb)};
}

}} // namespace
