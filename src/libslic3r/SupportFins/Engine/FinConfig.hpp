// Port of web/fins/config.js -- FIN. Mutable module state in the JS (fins.js
// applyTunables writes FIN.padH); thread_local here so concurrent builds on
// different threads never see each other's material.
#pragma once

namespace Slic3r { namespace SupportFins {

struct FinConfig {
    // --- from docs/FIN-SPEC.md, stated on camera. Do not "tune" these. ---
    double tineH = 0.2;
    // --- ours, derived or measured ---
    double padH = 0.5;
    double padMargin = 4.0;
    int    padSegs = 48;
    double padMinArea = 60.0;
    double coverExtraSparse = 88;
    double coverSparse = 55;
    double coverDense = 22;
    double coverDefault = 0.5;
};

extern thread_local FinConfig FIN;

}} // namespace
