// STUB -- replaced by the fins.js port. Builds nothing, so the entry pipeline and the
// analysis parity can be exercised before the wall builders exist.
#include "Fins.hpp"
#include "FinConfig.hpp"
#include "PropConfig.hpp"
#include "Cutout.hpp"
#include "Fins/Pad.hpp"
#include "Fins/Wedges.hpp"
#include <cmath>

namespace Slic3r { namespace SupportFins {

void applyTunables(const Tunables &t) {
    FIN.padH = t.padH;
    // set(PAD, 'grab', t.padGrab); PAD.style only if one of auto/light/sure/custom;
    // each PAD.custom field only if finite (fins.js `set` skips non-finite values)
    if (std::isfinite(t.padGrab)) PAD.grab = t.padGrab;
    if (t.padStyle == "auto" || t.padStyle == "light" || t.padStyle == "sure" || t.padStyle == "custom") PAD.style = t.padStyle;
    if (std::isfinite(t.padCustom.h)) PAD.custom.h = t.padCustom.h;
    if (std::isfinite(t.padCustom.gap)) PAD.custom.gap = t.padCustom.gap;
    if (std::isfinite(t.padCustom.grip)) PAD.custom.grip = t.padCustom.grip;
    if (std::isfinite(t.padCustom.margin)) PAD.custom.margin = t.padCustom.margin;
    PROP.gap = t.propGap;
    if (std::isfinite(t.propGap)) PERP.gap = t.propGap;
    PROP.iface = t.iface;
    if (t.tinesPerWall >= 0) PROP.tinesPerWall = t.tinesPerWall;
    if (isCutoutPattern(t.cutout)) CUT.pattern = t.cutout;
}

Built buildFins(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts) {
    applyTunables(opts.tunables);
    Built b;
    b.mode = opts.mode;
    b.unserved = int(result.regions.size());
    b.floating = floatingPieces(topo, result, rot);
    return b;
}

}} // namespace
