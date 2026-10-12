// Pins Engine/Ieee754.* (js_sin, js_atan2, js_pow, ...) against JS Math.* under Node 22:
//   node tools/ieee754_ref.mjs            # writes refs_ieee754/ (no upstream checkout needed)
//   ./build-sf/test_ieee754 [refs_ieee754_dir] [--verbose]
//
// For every function: the ~370k unary inputs in inputs.f64 (dense [-10,10], log-uniform
// 1e-300..1e300 both signs, every power of two, denormals, pi/4 multiples, each fdlibm
// branch-point high word, the angles and unit-vector components the engine feeds, ...);
// for atan2 and pow a 2048x2048 grid. Results must be BIT-IDENTICAL (compared as uint64;
// any NaN equals any NaN). The glibc (std::) column is informational: it shows how many
// inputs would break parity if Engine/ called libm instead.
#include "Engine/Ieee754.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace Slic3r::SupportFins;

static bool g_verbose = false;

static std::vector<double> load(const fs::path &p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return {};
    f.seekg(0, std::ios::end);
    const std::streamoff n = f.tellg();
    f.seekg(0);
    std::vector<double> v(size_t(n) / sizeof(double));
    f.read(reinterpret_cast<char *>(v.data()), std::streamsize(v.size() * sizeof(double)));
    return v;
}

static uint64_t bits(double d) { uint64_t u; std::memcpy(&u, &d, sizeof u); return u; }
static bool same(double a, double b) { return (std::isnan(a) && std::isnan(b)) || bits(a) == bits(b); }

struct Unary { const char *name; double (*js)(double); double (*libm)(double); };
struct Binary { const char *name; double (*js)(double, double); double (*libm)(double, double); const char *a, *b; };

int main(int argc, char **argv) {
    fs::path dir = fs::path(__FILE__).parent_path() / ".." / "refs_ieee754";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--verbose") g_verbose = true;
        else dir = argv[i];
    }
    const std::vector<double> in = load(dir / "inputs.f64");
    if (in.empty()) {
        std::printf("FAIL: no refs in %s -- generate them with: node tools/ieee754_ref.mjs\n", dir.string().c_str());
        return 1;
    }

    static const Unary U[] = {
        {"sin", js_sin, [](double x) { return std::sin(x); }},
        {"cos", js_cos, [](double x) { return std::cos(x); }},
        {"tan", js_tan, [](double x) { return std::tan(x); }},
        {"asin", js_asin, [](double x) { return std::asin(x); }},
        {"acos", js_acos, [](double x) { return std::acos(x); }},
        {"atan", js_atan, [](double x) { return std::atan(x); }},
        {"exp", js_exp, [](double x) { return std::exp(x); }},
        {"expm1", js_expm1, [](double x) { return std::expm1(x); }},
        {"log", js_log, [](double x) { return std::log(x); }},
        {"log1p", js_log1p, [](double x) { return std::log1p(x); }},
        {"log2", js_log2, [](double x) { return std::log2(x); }},
        {"log10", js_log10, [](double x) { return std::log10(x); }},
        {"cbrt", js_cbrt, [](double x) { return std::cbrt(x); }},
        {"sinh", js_sinh, [](double x) { return std::sinh(x); }},
        {"cosh", js_cosh, [](double x) { return std::cosh(x); }},
        {"tanh", js_tanh, [](double x) { return std::tanh(x); }},
        {"asinh", js_asinh, [](double x) { return std::asinh(x); }},
        {"acosh", js_acosh, [](double x) { return std::acosh(x); }},
        {"atanh", js_atanh, [](double x) { return std::atanh(x); }},
    };
    static const Binary B[] = {
        {"atan2", js_atan2, [](double y, double x) { return std::atan2(y, x); }, "atan2_y.f64", "atan2_x.f64"},
        {"pow", js_pow, [](double x, double y) { return std::pow(x, y); }, "pow_x.f64", "pow_y.f64"},
    };

    long totalBad = 0, totalChecks = 0;
    bool missing = false;
    std::printf("%-6s %10s %10s %12s\n", "fn", "inputs", "mismatch", "glibc-diff");
    for (const Unary &u : U) {
        const std::vector<double> ref = load(dir / (std::string(u.name) + ".f64"));
        if (ref.size() != in.size()) { std::printf("%-6s MISSING or wrong size\n", u.name); missing = true; continue; }
        long bad = 0, libmDiff = 0;
        for (size_t i = 0; i < in.size(); i++) {
            const double r = u.js(in[i]);
            if (!same(r, ref[i])) {
                if (bad < 10 || g_verbose)
                    std::printf("  %s(%a [%016llx]) = %a, JS %a\n", u.name, in[i], (unsigned long long) bits(in[i]), r, ref[i]);
                bad++;
            }
            if (!same(u.libm(in[i]), ref[i])) libmDiff++;
        }
        std::printf("%-6s %10zu %10ld %12ld\n", u.name, in.size(), bad, libmDiff);
        totalBad += bad;
        totalChecks += long(in.size());
    }
    for (const Binary &b : B) {
        const std::vector<double> A = load(dir / b.a), X = load(dir / b.b), ref = load(dir / (std::string(b.name) + ".f64"));
        if (A.empty() || X.empty() || ref.size() != A.size() * X.size()) { std::printf("%-6s MISSING or wrong size\n", b.name); missing = true; continue; }
        long bad = 0, libmDiff = 0;
        for (size_t i = 0; i < A.size(); i++)
            for (size_t j = 0; j < X.size(); j++) {
                const double e = ref[i * X.size() + j], r = b.js(A[i], X[j]);
                if (!same(r, e)) {
                    if (bad < 10 || g_verbose)
                        std::printf("  %s(%a, %a) = %a, JS %a\n", b.name, A[i], X[j], r, e);
                    bad++;
                }
                if (!same(b.libm(A[i], X[j]), e)) libmDiff++;
            }
        std::printf("%-6s %10zu %10ld %12ld   (%zux%zu grid)\n", b.name, ref.size(), bad, libmDiff, A.size(), X.size());
        totalBad += bad;
        totalChecks += long(ref.size());
    }
    std::printf("%ld checks, %ld not bit-identical to Node%s\n", totalChecks, totalBad, missing ? " (some refs missing)" : "");
    if (totalBad || missing) { std::printf("test_ieee754: FAILED\n"); return 1; }
    std::printf("test_ieee754: PASSED\n");
    return 0;
}
