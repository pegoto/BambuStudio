// Port of web/prop/config.js -- PROP. Every number a breakaway wall is built from.
// The reasoning behind each value is in the upstream file; it is not repeated here.
// Mutable module state in the JS (applyTunables writes gap/tinesPerWall/iface);
// thread_local here.
#pragma once

namespace Slic3r { namespace SupportFins {

enum class Iface { Off, Flat, All };

struct PropConfig {
    double th = 1.0;
    double gap = 0.2;
    double footGap = 0.2;
    double tip = 0.6;
    double baseH = 0.6;
    double tipH = 1.5;
    Iface  iface = Iface::Off;
    int    ifaceLayers = 1;
    double ifaceFlatDeg = 10;
    double footMax = 3.0;
    double footMin = 1.6;
    double footRatio = 0.12;
    double minSpan = 7.0;
    double minSpanShort = 4.0;
    double maxShortAspect = 6.0;
    double minHeight = 1.5;
    double minHeightSquat = 0.6;
    double minSpanSquat = 4.0;
    double squatBrimH = 0.4;
    double squatBrimW = 2.5;
    double lowBandMin = 1.0;
    double stationStep = 1.0;
    int    minStations = 3;
    int    clearProbes = 12;
    double sideClear = 0.35;
    double maxWander = 0.10;
    double contourSlopeMin = 0.15;
    double splitAgreeDeg = 15;
    double tubeSpreadDeg = 25;
    double tubeCurvedFrac = 0.4;
    double tubeMinArea = 300;
    double tubeSmallMinArea = 12;
    double minSpanTube = 3.0;
    double minSpanPart = 3.0;
    double tubeConvexFrac = 0.7;
    double tubeTwoSidedFrac = 0.25;
    double maxUnsupportedSpan = 12.0;
    double edgeInset = 0.6;
    double edgeFlatDeg = 30;
    // --- TINES ---
    double tineH = 0.2;
    double tineW = 0.5;
    int    tinesPerWall = 0;
    double tineReach = 0.5;
    double tineStep = 2.0;
    double tineStepSparse = 5.0;
    double tineOverlap = 0.3;
    int    minGripTines = 3;
    double tineEdgeBand = 8.0;
    double tineMidFactor = 2.0;
    double tineSlopeMin = 1.0;
    double tineTopClear = 0.5;
};

extern thread_local PropConfig PROP;

}} // namespace
