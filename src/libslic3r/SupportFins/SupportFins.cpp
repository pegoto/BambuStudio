///|/ Support Fins -- libslic3r-facing API (see SupportFins.hpp).
///|/
///|/ Port of the printfins.com engine (https://github.com/gittrahan/support-fins, MIT,
///|/ (c) Matthew Trahan; technique by Slant3D). This unit is the host adapter: it plays
///|/ the role of upstream's plugins/shared/py/supportfins_host.py and the Orca plugin's
///|/ posed_part_soup(), but writes the fins into the model as ordinary volumes.
#include "SupportFins.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "Engine/Entry.hpp"
#include "Engine/Materials.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace Slic3r {

namespace sf = Slic3r::SupportFins;

namespace {

// ---------------------------------------------------------------------------------
// key=value;key=value helpers. Locale independent (classic "C" numbers) so a project
// saved under a comma-decimal locale reads back anywhere.

// Escape the separators so any string value survives the round trip.
std::string kv_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
        case '%': out += "%25"; break;
        case ';': out += "%3B"; break;
        case '=': out += "%3D"; break;
        case '\n': out += "%0A"; break;
        case '\r': out += "%0D"; break;
        default: out += c;
        }
    }
    return out;
}

bool kv_unescape(const std::string &s, std::string &out)
{
    out.clear();
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '%') {
            out += s[i];
            continue;
        }
        if (i + 2 >= s.size())
            return false;
        auto hex = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        const int hi = hex(s[i + 1]), lo = hex(s[i + 2]);
        if (hi < 0 || lo < 0)
            return false;
        out += char(hi * 16 + lo);
        i += 2;
    }
    return true;
}

// Split "k=v;k=v" into (key, raw value) pairs; empty items are skipped. Returns false
// on an item without '='.
bool kv_split(const std::string &s, std::vector<std::pair<std::string, std::string>> &out)
{
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t end = s.find(';', pos);
        if (end == std::string::npos)
            end = s.size();
        std::string item = s.substr(pos, end - pos);
        // tolerate whitespace around items (hand-edited files)
        const size_t b = item.find_first_not_of(" \t\r\n");
        const size_t e = item.find_last_not_of(" \t\r\n");
        item = b == std::string::npos ? std::string() : item.substr(b, e - b + 1);
        if (!item.empty()) {
            const size_t eq = item.find('=');
            if (eq == std::string::npos || eq == 0)
                return false;
            out.emplace_back(item.substr(0, eq), item.substr(eq + 1));
        }
        pos = end + 1;
    }
    return true;
}

std::string fmt_double(double v)
{
    std::ostringstream ss;
    ss.imbue(std::locale::classic());
    ss << std::setprecision(17) << v;
    return ss.str();
}

bool parse_double(const std::string &s, double &out)
{
    if (s.empty())
        return false;
    std::istringstream ss(s);
    ss.imbue(std::locale::classic());
    double v;
    ss >> v;
    if (ss.fail())
        return false;
    ss >> std::ws;
    if (!ss.eof())
        return false;
    out = v;
    return true;
}

bool parse_int(const std::string &s, int &out)
{
    if (s.empty())
        return false;
    std::istringstream ss(s);
    ss.imbue(std::locale::classic());
    long long v;
    ss >> v;
    if (ss.fail())
        return false;
    ss >> std::ws;
    if (!ss.eof() || v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max())
        return false;
    out = int(v);
    return true;
}

bool parse_bool(const std::string &s, bool &out)
{
    if (s == "1" || s == "true" || s == "True" || s == "TRUE") { out = true; return true; }
    if (s == "0" || s == "false" || s == "False" || s == "FALSE") { out = false; return true; }
    return false;
}

bool parse_vec3(const std::string &s, Vec3d &out)
{
    Vec3d v;
    size_t pos = 0;
    for (int i = 0; i < 3; ++i) {
        size_t end = s.find(',', pos);
        if (i < 2 && end == std::string::npos)
            return false;
        if (i == 2) {
            if (end != std::string::npos)
                return false;
            end = s.size();
        }
        if (!parse_double(s.substr(pos, end - pos), v(i)))
            return false;
        pos = end + 1;
    }
    out = v;
    return true;
}

std::string fmt_vec3(const Vec3d &v) { return fmt_double(v.x()) + "," + fmt_double(v.y()) + "," + fmt_double(v.z()); }

bool one_of(const std::string &v, std::initializer_list<const char *> allowed)
{
    for (const char *a : allowed)
        if (v == a)
            return true;
    return false;
}

// Clamp a number to [lo, hi]; a non-finite one takes the default and fails.
bool clamp_number(double &v, double lo, double hi, double dflt)
{
    if (!std::isfinite(v)) {
        v = dflt;
        return false;
    }
    v = std::clamp(v, lo, hi);
    return true;
}

// 64 bit FNV-1a.
struct Fnv1a
{
    uint64_t h = 0xcbf29ce484222325ull;
    void bytes(const void *data, size_t len)
    {
        const unsigned char *p = static_cast<const unsigned char *>(data);
        for (size_t i = 0; i < len; ++i) {
            h ^= uint64_t(p[i]);
            h *= 0x100000001b3ull;
        }
    }
    void i64(int64_t v)
    {
        // fixed little-endian byte order so the hash is the same on every platform
        unsigned char b[8];
        for (int i = 0; i < 8; ++i)
            b[i] = static_cast<unsigned char>((uint64_t(v) >> (8 * i)) & 0xff);
        bytes(b, 8);
    }
    std::string hex() const
    {
        static const char *digits = "0123456789abcdef";
        std::string out(16, '0');
        for (int i = 0; i < 16; ++i)
            out[15 - i] = digits[(h >> (4 * i)) & 0xf];
        return out;
    }
};

// Exact-equality key of a float vertex; -0 and +0 are the same point.
struct VertexKey
{
    std::array<uint32_t, 3> bits;
    bool operator==(const VertexKey &o) const { return bits == o.bits; }
};
struct VertexKeyHash
{
    size_t operator()(const VertexKey &k) const
    {
        uint64_t h = 0xcbf29ce484222325ull;
        for (uint32_t b : k.bits) {
            h ^= b;
            h *= 0x100000001b3ull;
        }
        return size_t(h);
    }
};
VertexKey vertex_key(float x, float y, float z)
{
    VertexKey k;
    const float v[3] = {x + 0.f, y + 0.f, z + 0.f};   // + 0.f turns -0 into +0
    for (int i = 0; i < 3; ++i)
        std::memcpy(&k.bits[i], &v[i], sizeof(float));
    return k;
}

std::string piece_name(const std::string &kind, int fin_number, int sway_number)
{
    if (kind == "pad")
        return "Bed pad";
    if (kind == "sway")
        return "Sway brace " + std::to_string(sway_number);
    if (kind == "wedge")
        return "Fin " + std::to_string(fin_number) + " (wedge)";
    if (kind == "drawn")
        return "Fin " + std::to_string(fin_number) + " (drawn)";
    return "Fin " + std::to_string(fin_number) + " (wall)";
}

SupportFinOptions checked(const SupportFinOptions &options)
{
    SupportFinOptions opts = options;
    if (!opts.validate())
        throw std::invalid_argument("Support Fins: invalid option (material must be pla|petg, bed pad off|auto|light|sure, "
                                    "cutout none|diamond|triangle|arch|lattice, layer height > 0): " + options.to_string());
    return opts;
}

const ModelInstance &instance_of(const ModelObject &object, int instance_idx)
{
    if (instance_idx < 0 || size_t(instance_idx) >= object.instances.size())
        throw std::invalid_argument("Support Fins: the object has no instance " + std::to_string(instance_idx));
    return *object.instances[size_t(instance_idx)];
}

} // namespace

// ---------------------------------------------------------------------------------
// SupportFinOptions

bool SupportFinOptions::operator==(const SupportFinOptions &o) const
{
    return material == o.material && threshold == o.threshold && tines == o.tines && tine_density == o.tine_density &&
           layer_height == o.layer_height && pad_style == o.pad_style && sway == o.sway && sway_grip_from == o.sway_grip_from &&
           sway_tine_spacing == o.sway_tine_spacing && sway_reach == o.sway_reach && cutout == o.cutout && coverage == o.coverage &&
           plate_only == o.plate_only;
}

bool SupportFinOptions::validate()
{
    const SupportFinOptions dflt;
    bool ok = true;
    // options.json ranges
    ok &= clamp_number(threshold, 30., 70., dflt.threshold);
    ok &= clamp_number(tine_density, 0., 1., dflt.tine_density);
    ok &= clamp_number(sway_grip_from, 0., 2000., dflt.sway_grip_from);
    ok &= clamp_number(sway_tine_spacing, 2., 30., dflt.sway_tine_spacing);
    ok &= clamp_number(sway_reach, 0.05, 0.5, dflt.sway_reach);
    ok &= clamp_number(coverage, 0., 1., dflt.coverage);
    // layerHeight is hostSupplied (the print preset's): used as is, only refused when
    // it is not a positive number (fins_entry.js optionsFromDialog).
    if (!std::isfinite(layer_height) || !(layer_height > 0.)) {
        layer_height = dflt.layer_height;
        ok = false;
    }
    if (!one_of(material, {"pla", "petg"})) {
        material = dflt.material;
        ok = false;
    }
    if (!one_of(pad_style, {"off", "auto", "light", "sure"})) {
        pad_style = dflt.pad_style;
        ok = false;
    }
    if (!one_of(cutout, {"none", "diamond", "triangle", "arch", "lattice"})) {
        cutout = dflt.cutout;
        ok = false;
    }
    return ok;
}

std::string SupportFinOptions::to_string() const
{
    std::string s;
    s += "material=" + kv_escape(material);
    s += ";threshold=" + fmt_double(threshold);
    s += ";tines=" + std::string(tines ? "1" : "0");
    s += ";tine_density=" + fmt_double(tine_density);
    s += ";layer_height=" + fmt_double(layer_height);
    s += ";pad_style=" + kv_escape(pad_style);
    s += ";sway=" + std::string(sway ? "1" : "0");
    s += ";sway_grip_from=" + fmt_double(sway_grip_from);
    s += ";sway_tine_spacing=" + fmt_double(sway_tine_spacing);
    s += ";sway_reach=" + fmt_double(sway_reach);
    s += ";cutout=" + kv_escape(cutout);
    s += ";coverage=" + fmt_double(coverage);
    s += ";plate_only=" + std::string(plate_only ? "1" : "0");
    return s;
}

std::optional<SupportFinOptions> SupportFinOptions::from_string(const std::string &str)
{
    std::vector<std::pair<std::string, std::string>> items;
    if (!kv_split(str, items))
        return std::nullopt;
    SupportFinOptions o;
    for (const auto &[key, value] : items) {
        bool ok = true;
        if (key == "material")                ok = kv_unescape(value, o.material);
        else if (key == "threshold")          ok = parse_double(value, o.threshold);
        else if (key == "tines")              ok = parse_bool(value, o.tines);
        else if (key == "tine_density")       ok = parse_double(value, o.tine_density);
        else if (key == "layer_height")       ok = parse_double(value, o.layer_height);
        else if (key == "pad_style")          ok = kv_unescape(value, o.pad_style);
        else if (key == "sway")               ok = parse_bool(value, o.sway);
        else if (key == "sway_grip_from")     ok = parse_double(value, o.sway_grip_from);
        else if (key == "sway_tine_spacing")  ok = parse_double(value, o.sway_tine_spacing);
        else if (key == "sway_reach")         ok = parse_double(value, o.sway_reach);
        else if (key == "cutout")             ok = kv_unescape(value, o.cutout);
        else if (key == "coverage")           ok = parse_double(value, o.coverage);
        else if (key == "plate_only")         ok = parse_bool(value, o.plate_only);
        // else: a key from a newer version, ignored
        if (!ok)
            return std::nullopt;
    }
    return o;
}

// ---------------------------------------------------------------------------------
// SupportFinInfo

std::string SupportFinInfo::to_string() const
{
    std::string s;
    s += "kind=" + kv_escape(kind);
    s += ";id=" + std::to_string(piece_id);
    s += ";hash=" + kv_escape(source_hash);
    s += ";drawn=" + std::string(drawn ? "1" : "0");
    if (drawn) {
        s += ";a=" + fmt_vec3(draw_a);
        s += ";b=" + fmt_vec3(draw_b);
    }
    return s;
}

std::optional<SupportFinInfo> SupportFinInfo::from_string(const std::string &str)
{
    std::vector<std::pair<std::string, std::string>> items;
    if (!kv_split(str, items))
        return std::nullopt;
    SupportFinInfo info;
    for (const auto &[key, value] : items) {
        bool ok = true;
        if (key == "kind")       ok = kv_unescape(value, info.kind);
        else if (key == "id")    ok = parse_int(value, info.piece_id);
        else if (key == "hash")  ok = kv_unescape(value, info.source_hash);
        else if (key == "drawn") ok = parse_bool(value, info.drawn);
        else if (key == "a")     ok = parse_vec3(value, info.draw_a);
        else if (key == "b")     ok = parse_vec3(value, info.draw_b);
        // else: a key from a newer version, ignored
        if (!ok)
            return std::nullopt;
    }
    return info;
}

// ---------------------------------------------------------------------------------
// Engine

SupportFinResult generate_support_fins(const indexed_triangle_set &posed_part, const SupportFinOptions &options)
{
    const SupportFinOptions opts = checked(options);

    SupportFinResult out;
    if (posed_part.indices.empty())
        return out;

    // fins_entry.js computeFins(positions): 9 doubles per face, already posed.
    sf::Soup soup;
    soup.reserve(posed_part.indices.size() * 9);
    for (const stl_triangle_vertex_indices &f : posed_part.indices)
        for (int k = 0; k < 3; ++k) {
            const stl_vertex &v = posed_part.vertices[size_t(f(k))];
            soup.push_back(double(v.x()));
            soup.push_back(double(v.y()));
            soup.push_back(double(v.z()));
        }

    sf::EntryOptions eo;
    eo.mode        = "auto";
    eo.bedPad      = opts.pad_style != "off";
    eo.padStyle    = opts.pad_style;
    eo.tines       = opts.tines;
    eo.tineDensity = opts.tine_density;
    eo.coverage    = opts.coverage;
    eo.plateOnly   = opts.plate_only;
    eo.layerHeight = opts.layer_height;
    eo.threshold   = opts.threshold;
    eo.material    = opts.material;
    eo.cutout      = opts.cutout;
    if (opts.sway) {
        // optionsFromDialog drops sway unless it is on; computeFins gives braces the
        // material's gap unless the host sets its own.
        sf::SwayOpts sw;
        sw.on          = true;
        sw.gripFrom    = opts.sway_grip_from;
        sw.tineSpacing = opts.sway_tine_spacing;
        sw.reach       = opts.sway_reach;
        if (const sf::Material *mat = sf::material_of(opts.material))
            sw.gap = mat->propGap;
        eo.sway = sw;
    }

    const sf::EntryResult res = sf::computeFins(soup, eo);

    // seated = input + offset, so world = seated - offset.
    const size_t n_tris = res.triangles.size() / 9;
    for (const sf::PieceRange &pr : res.pieces) {
        SupportFinPiece piece;
        piece.kind = pr.kind;
        if (pr.id == "pad" || !parse_int(pr.id, piece.id))
            piece.id = -1;
        // Weld on the seated float32 values: the engine emits bit-identical floats for
        // a vertex shared by two triangles, so exact equality is the right test.
        std::unordered_map<VertexKey, int, VertexKeyHash> index_of;
        for (const auto &[first, end] : pr.ranges) {
            const size_t a = size_t(std::max(first, 0)), b = std::min(size_t(std::max(end, 0)), n_tris);
            for (size_t t = a; t < b; ++t) {
                stl_triangle_vertex_indices face;
                for (int k = 0; k < 3; ++k) {
                    const float *p = &res.triangles[t * 9 + size_t(k) * 3];
                    const VertexKey key = vertex_key(p[0], p[1], p[2]);
                    auto it = index_of.find(key);
                    if (it == index_of.end()) {
                        const int idx = int(piece.mesh.vertices.size());
                        piece.mesh.vertices.emplace_back(float(double(p[0]) - res.offset.x),
                                                         float(double(p[1]) - res.offset.y),
                                                         float(double(p[2]) - res.offset.z));
                        it = index_of.emplace(key, idx).first;
                    }
                    face(k) = it->second;
                }
                // a face with a repeated corner has no area: drop it
                if (face(0) != face(1) && face(1) != face(2) && face(0) != face(2))
                    piece.mesh.indices.push_back(face);
            }
        }
        if (!piece.mesh.indices.empty())
            out.pieces.push_back(std::move(piece));
    }

    out.over_faces  = res.overFaces;
    out.small_faces = res.smallFaces;

    const sf::EntryStats &s = res.stats;
    SupportFinStats &st     = out.stats;
    st.overhang_regions = s.overhangRegions;
    st.walls            = s.braces + s.props;   // tined + plain, as the site counts
    st.tines            = s.tines;
    st.sway_braces      = s.swayBraces;
    st.sway_tines       = s.swayTines;
    st.sway_skipped     = s.swaySkipped;
    st.sway_reason      = s.swayReason;
    st.unserved         = s.unserved;
    st.floating         = s.floating;
    st.floating_drop    = s.floatingDrop;
    st.fin_triangles    = s.finTriangles;
    st.sway_triangles   = s.swayTriangles;
    st.pad_triangles    = s.padTriangles;
    return out;
}

std::optional<SupportFinPiece> draw_support_fin_wall(const indexed_triangle_set & /*posed_part*/, const Vec3d & /*a*/, const Vec3d & /*b*/,
                                                     const SupportFinOptions &options, std::string &reason)
{
    // TODO(draw): wire Engine drawWall (web/draw.js port, Engine/Draw.*) once it exists.
    (void) checked(options);
    reason = "Draw mode is not available yet";
    return std::nullopt;
}

// plugins/shared/py/supportfins_host.py host_report()
std::string support_fins_report(const SupportFinStats &stats)
{
    auto plural = [](int n, const char *word) { return std::to_string(n) + " " + word + (n == 1 ? "" : "s"); };
    std::vector<std::string> parts;
    parts.emplace_back(plural(stats.walls, "wall") + ", " + plural(stats.tines, "tine"));
    if (stats.sway_braces)   // they hold the tall sides, not an overhang: never added to the walls
        parts[0] += ", " + plural(stats.sway_braces, "sway brace");
    if (stats.unserved)
        parts.emplace_back(std::to_string(stats.unserved) + (stats.unserved == 1 ? " overhang is" : " overhangs are") +
                           " too shallow for a fin this way up (tilt the part steeper)");
    if (stats.floating) {
        std::ostringstream drop;
        drop.imbue(std::locale::classic());
        drop << std::fixed << std::setprecision(1) << stats.floating_drop;
        parts.emplace_back((stats.floating == 1 ?
                                "one piece isn't joined to the rest: it starts " + drop.str() + " mm up" :
                                std::to_string(stats.floating) + " pieces aren't joined to the rest: the first starts " + drop.str() + " mm up") +
                           ", held only by supports");
    }
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i)
            out += "; ";
        out += parts[i];
    }
    return out;
}

// ---------------------------------------------------------------------------------
// Model glue

indexed_triangle_set support_fins_posed_part(const ModelObject &object, int instance_idx)
{
    indexed_triangle_set out;
    if (instance_idx < 0 || size_t(instance_idx) >= object.instances.size())
        return out;
    const Transform3d &inst = object.instances[size_t(instance_idx)]->get_matrix();
    for (const ModelVolume *volume : object.volumes) {
        if (!volume->is_model_part() || volume->is_support_fin())
            continue;
        const indexed_triangle_set &its = volume->mesh().its;
        if (its.indices.empty())
            continue;
        const Transform3d m = inst * volume->get_matrix();
        // A mirrored transform flips every triangle's winding; the engine reads
        // overhangs from face normals, so restore outward-facing order.
        const bool flip = m.linear().determinant() < 0.;
        const int base = int(out.vertices.size());
        out.vertices.reserve(out.vertices.size() + its.vertices.size());
        for (const stl_vertex &v : its.vertices)
            out.vertices.emplace_back((m * v.cast<double>()).cast<float>());
        out.indices.reserve(out.indices.size() + its.indices.size());
        for (const stl_triangle_vertex_indices &f : its.indices)
            out.indices.emplace_back(base + f(0), base + (flip ? f(2) : f(1)), base + (flip ? f(1) : f(2)));
    }
    return out;
}

std::string support_fins_source_hash(const indexed_triangle_set &posed_part, const SupportFinOptions &options)
{
    // The engine seats the part (XY bbox centred, lowest point at z = 0) before it does
    // anything, so the fins do not depend on where the object stands on the plate:
    // hash the seated geometry, so moving the object does not mark its fins stale.
    // Coordinates are quantised to 1 um, well above float32 noise at plate scale, so the
    // float rounding of a translated pose does not change the hash either.
    Fnv1a h;
    double x0 = 0., x1 = 0., y0 = 0., y1 = 0., z0 = 0.;
    if (!posed_part.vertices.empty()) {
        x0 = y0 = z0 = std::numeric_limits<double>::max();
        x1 = y1 = -std::numeric_limits<double>::max();
        for (const stl_vertex &v : posed_part.vertices) {
            x0 = std::min(x0, double(v.x())); x1 = std::max(x1, double(v.x()));
            y0 = std::min(y0, double(v.y())); y1 = std::max(y1, double(v.y()));
            z0 = std::min(z0, double(v.z()));
        }
    }
    const double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1);
    h.i64(int64_t(posed_part.vertices.size()));
    for (const stl_vertex &v : posed_part.vertices) {
        h.i64(int64_t(std::llround((double(v.x()) - cx) * 1000.)));
        h.i64(int64_t(std::llround((double(v.y()) - cy) * 1000.)));
        h.i64(int64_t(std::llround((double(v.z()) - z0) * 1000.)));
    }
    h.i64(int64_t(posed_part.indices.size()));
    for (const stl_triangle_vertex_indices &f : posed_part.indices)
        for (int k = 0; k < 3; ++k)
            h.i64(int64_t(f(k)));
    const std::string opts = options.to_string();
    h.bytes(opts.data(), opts.size());
    return h.hex();
}

SupportFinStats apply_support_fins(ModelObject &object, const SupportFinOptions &options, int instance_idx)
{
    const SupportFinOptions opts = checked(options);
    instance_of(object, instance_idx);

    // Everything that can throw runs before the object is touched.
    const indexed_triangle_set posed = support_fins_posed_part(object, instance_idx);
    if (posed.indices.empty())
        throw std::invalid_argument("Support Fins: the object has no model parts to support");
    const std::string hash = support_fins_source_hash(posed, opts);
    SupportFinResult result = generate_support_fins(posed, opts);

    // Drop the previous generated set (drawn walls stay), highest index first.
    for (size_t i = object.volumes.size(); i-- > 0;) {
        const ModelVolume *v = object.volumes[i];
        if (v->is_support_fin() && !v->support_fin->drawn)
            object.delete_volume(i);
    }
    int fin_number = 0, sway_number = 0;
    for (ModelVolume *v : object.volumes)
        if (v->is_support_fin()) {
            v->support_fin->source_hash = hash;
            ++fin_number;
        }

    // delete_volume() may have folded the last remaining volume's matrix into the
    // instances, so read the instance transform only now. The pieces are in the world
    // frame: a volume matrix of inst^-1 (composed with the centring add_volume applies)
    // puts them back exactly where the engine built them.
    const Transform3d inst_inv = instance_of(object, instance_idx).get_matrix().inverse();
    for (SupportFinPiece &piece : result.pieces) {
        if (piece.mesh.indices.empty())
            continue;
        ModelVolume *v = object.add_volume(TriangleMesh(std::move(piece.mesh)), ModelVolumeType::MODEL_PART, /*modify_to_center_geometry=*/true);
        // add_volume centred the mesh: mesh = world - shift, volume matrix = translate(shift).
        v->set_transformation(Transform3d(inst_inv * v->get_matrix()));
        if (piece.kind == "sway")
            ++sway_number;
        else if (piece.kind != "pad")
            ++fin_number;
        v->name = piece_name(piece.kind, fin_number, sway_number);
        SupportFinInfo info;
        info.kind        = piece.kind;
        info.piece_id    = piece.id;
        info.source_hash = hash;
        v->support_fin   = std::move(info);
    }

    object.support_fin_options = opts;
    object.invalidate_bounding_box();
    return result.stats;
}

bool add_drawn_support_fin(ModelObject &object, const Vec3d &a, const Vec3d &b, const SupportFinOptions &options, std::string &reason,
                           int instance_idx)
{
    const SupportFinOptions opts = checked(options);
    if (instance_idx < 0 || size_t(instance_idx) >= object.instances.size()) {
        reason = "The object has no such instance";
        return false;
    }
    const indexed_triangle_set posed = support_fins_posed_part(object, instance_idx);
    if (posed.indices.empty()) {
        reason = "The object has no model parts to support";
        return false;
    }
    std::optional<SupportFinPiece> piece = draw_support_fin_wall(posed, a, b, opts, reason);
    if (!piece || piece->mesh.indices.empty()) {
        if (reason.empty())
            reason = "No wall fits between these two points";
        return false;
    }

    int fin_number = 0;
    for (const ModelVolume *v : object.volumes)
        if (v->is_support_fin() && v->support_fin->kind != "pad" && v->support_fin->kind != "sway")
            ++fin_number;

    const Transform3d inst_inv = object.instances[size_t(instance_idx)]->get_matrix().inverse();
    ModelVolume *v = object.add_volume(TriangleMesh(std::move(piece->mesh)), ModelVolumeType::MODEL_PART, /*modify_to_center_geometry=*/true);
    v->set_transformation(Transform3d(inst_inv * v->get_matrix()));
    v->name = piece_name("drawn", fin_number + 1, 0);
    SupportFinInfo info;
    info.kind        = "drawn";
    info.piece_id    = piece->id;
    info.source_hash = support_fins_source_hash(posed, opts);
    info.drawn       = true;
    info.draw_a      = a;
    info.draw_b      = b;
    v->support_fin   = std::move(info);
    if (!object.support_fin_options)
        object.support_fin_options = opts;
    object.invalidate_bounding_box();
    return true;
}

int clear_support_fins(ModelObject &object)
{
    int removed = 0;
    for (size_t i = object.volumes.size(); i-- > 0;)
        if (object.volumes[i]->is_support_fin()) {
            object.delete_volume(i);
            ++removed;
        }
    object.support_fin_options.reset();
    if (removed)
        object.invalidate_bounding_box();
    return removed;
}

bool support_fins_stale(const ModelObject &object, int instance_idx)
{
    if (!object.has_support_fins() || !object.support_fin_options)
        return false;
    if (instance_idx < 0 || size_t(instance_idx) >= object.instances.size())
        return false;
    SupportFinOptions opts = *object.support_fin_options;
    opts.validate();
    const std::string hash = support_fins_source_hash(support_fins_posed_part(object, instance_idx), opts);
    for (const ModelVolume *v : object.volumes)
        if (v->is_support_fin() && v->support_fin->source_hash != hash)
            return true;
    return false;
}

} // namespace Slic3r
