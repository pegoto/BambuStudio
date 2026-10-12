// Port of web/materials.js
#pragma once
#include <string>

namespace Slic3r { namespace SupportFins {

struct Material {
    double padH, padGrab, propGap, density;
};

/// web/materials.js MATERIAL. Keys: "pla", "petg". Returns nullptr for an unknown key.
inline const Material *material_of(const std::string &key) {
    static const Material pla  {0.5,  0.05, 0.2, 1.24};
    static const Material petg {0.3, -0.10, 0.3, 1.27};
    if (key == "pla") return &pla;
    if (key == "petg") return &petg;
    return nullptr;
}

}} // namespace
