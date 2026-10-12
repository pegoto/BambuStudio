// STUB -- replaced by the fins.js port. Builds nothing, so the entry pipeline and the
// analysis parity can be exercised before the wall builders exist.
#include "Fins.hpp"
#include "FinConfig.hpp"
#include "PropConfig.hpp"
#include "Cutout.hpp"

namespace Slic3r { namespace SupportFins {

void applyTunables(const Tunables &t) {
    FIN.padH = t.padH;
    PROP.gap = t.propGap;
    PROP.iface = t.iface;
    if (t.tinesPerWall >= 0) PROP.tinesPerWall = t.tinesPerWall;
    if (isCutoutPattern(t.cutout)) CUT.pattern = t.cutout;
    // PAD.grab / PAD.style / PAD.custom / PERP.gap: set by the owners of those modules
    // once they exist.
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
