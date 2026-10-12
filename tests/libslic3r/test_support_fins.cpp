#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/SupportFins/SupportFins.hpp"

#include <boost/filesystem.hpp>

#include <cmath>
#include <limits>

using namespace Slic3r;

namespace {

// A 20 mm cube tilted 40 degrees about X, standing on the plate.
indexed_triangle_set tilted_cube()
{
    indexed_triangle_set its = its_make_cube(20., 20., 20.);
    const Transform3d rot = Transform3d(Eigen::AngleAxisd(Geometry::deg2rad(40.), Vec3d::UnitX()));
    float z0 = std::numeric_limits<float>::max();
    for (stl_vertex &v : its.vertices) {
        v = (rot * v.cast<double>()).cast<float>();
        z0 = std::min(z0, v.z());
    }
    for (stl_vertex &v : its.vertices)
        v.z() -= z0;
    return its;
}

SupportFinOptions non_default_options()
{
    SupportFinOptions o;
    o.material          = "petg";
    o.threshold         = 52.5;
    o.tines             = false;
    o.tine_density      = 0.35;
    o.layer_height      = 0.12;
    o.pad_style         = "sure";
    o.sway              = true;
    o.sway_grip_from    = 12.75;
    o.sway_tine_spacing = 7.;
    o.sway_reach        = 0.22;
    o.cutout            = "lattice";
    o.coverage          = 0.85;
    o.plate_only        = true;
    return o;
}

} // namespace

TEST_CASE("SupportFinOptions string round trip", "[SupportFins]")
{
    SECTION("defaults") {
        const SupportFinOptions o;
        const auto back = SupportFinOptions::from_string(o.to_string());
        REQUIRE(back.has_value());
        CHECK(*back == o);
    }
    SECTION("every field changed") {
        const SupportFinOptions o = non_default_options();
        const auto back = SupportFinOptions::from_string(o.to_string());
        REQUIRE(back.has_value());
        CHECK(*back == o);
    }
    SECTION("doubles round trip exactly, edge values included") {
        SupportFinOptions o;
        o.threshold         = 30.;
        o.tine_density      = 0.1 + 0.2;   // 0.30000000000000004
        o.layer_height      = 0.08;
        o.sway_grip_from    = 2000.;
        o.sway_tine_spacing = 2.;
        o.sway_reach        = 0.05;
        o.coverage          = 1. / 3.;
        const auto back = SupportFinOptions::from_string(o.to_string());
        REQUIRE(back.has_value());
        CHECK(back->tine_density == o.tine_density);
        CHECK(back->coverage == o.coverage);
        CHECK(back->sway_reach == o.sway_reach);
        CHECK(*back == o);
    }
    SECTION("unknown keys are ignored, missing keys take the default") {
        const auto back = SupportFinOptions::from_string("material=petg;future_option=42;threshold=60;another=x=y");
        REQUIRE(back.has_value());
        CHECK(back->material == "petg");
        CHECK(back->threshold == 60.);
        CHECK(back->coverage == SupportFinOptions().coverage);
    }
    SECTION("empty string is all defaults") {
        const auto back = SupportFinOptions::from_string("");
        REQUIRE(back.has_value());
        CHECK(*back == SupportFinOptions());
    }
    SECTION("malformed values are refused") {
        CHECK_FALSE(SupportFinOptions::from_string("threshold=abc").has_value());
        CHECK_FALSE(SupportFinOptions::from_string("tines=maybe").has_value());
        CHECK_FALSE(SupportFinOptions::from_string("threshold").has_value());
    }
}

TEST_CASE("SupportFinOptions validate", "[SupportFins]")
{
    SupportFinOptions o;
    o.threshold    = 10.;
    o.coverage     = 1.5;
    o.sway_reach   = 0.;
    o.layer_height = 0.6;   // host supplied: kept as is
    CHECK(o.validate());
    CHECK(o.threshold == 30.);
    CHECK(o.coverage == 1.);
    CHECK(o.sway_reach == 0.05);
    CHECK(o.layer_height == 0.6);

    SupportFinOptions bad;
    bad.material = "abs";
    CHECK_FALSE(bad.validate());
    CHECK(bad.material == "pla");

    SupportFinOptions bad_layer;
    bad_layer.layer_height = 0.;
    CHECK_FALSE(bad_layer.validate());
}

TEST_CASE("SupportFinInfo string round trip", "[SupportFins]")
{
    SupportFinInfo info;
    info.kind        = "prop";
    info.piece_id    = 7;
    info.source_hash = "0123456789abcdef";
    auto back        = SupportFinInfo::from_string(info.to_string());
    REQUIRE(back.has_value());
    CHECK(back->kind == info.kind);
    CHECK(back->piece_id == info.piece_id);
    CHECK(back->source_hash == info.source_hash);
    CHECK_FALSE(back->drawn);

    info.kind   = "drawn";
    info.drawn  = true;
    info.draw_a = Vec3d(1.25, -3.5, 1. / 3.);
    info.draw_b = Vec3d(-0.1, 1e-9, 123.456);
    back        = SupportFinInfo::from_string(info.to_string());
    REQUIRE(back.has_value());
    CHECK(back->drawn);
    CHECK(back->draw_a == info.draw_a);
    CHECK(back->draw_b == info.draw_b);

    // separators inside a value survive
    info.kind = "a;b=c%d";
    back      = SupportFinInfo::from_string(info.to_string());
    REQUIRE(back.has_value());
    CHECK(back->kind == info.kind);

    CHECK(SupportFinInfo::from_string("kind=pad;id=-1;some_new_key=1").has_value());
    CHECK_FALSE(SupportFinInfo::from_string("id=notanumber").has_value());
}

TEST_CASE("Support fins report", "[SupportFins]")
{
    SupportFinStats s;
    CHECK(support_fins_report(s) == "0 walls, 0 tines");
    s.walls = 1;
    s.tines = 1;
    s.sway_braces = 2;
    CHECK(support_fins_report(s) == "1 wall, 1 tine, 2 sway braces");
    s.unserved      = 1;
    s.floating      = 2;
    s.floating_drop = 3.04;
    CHECK(support_fins_report(s) == "1 wall, 1 tine, 2 sway braces; 1 overhang is too shallow for a fin this way up (tilt the part steeper); "
                                    "2 pieces aren't joined to the rest: the first starts 3.0 mm up, held only by supports");
}

TEST_CASE("Support fins posed part", "[SupportFins]")
{
    Model model;
    ModelObject *obj = model.add_object();
    obj->add_volume(make_cube(20., 20., 20.));
    ModelInstance *inst = obj->add_instance();
    inst->set_offset(Vec3d(100., 80., 10.));
    inst->set_mirror(Vec3d(-1., 1., 1.));

    const indexed_triangle_set posed = support_fins_posed_part(*obj, 0);
    REQUIRE(posed.indices.size() == 12);
    // the mirror is undone in the winding: still an outward-facing closed solid
    CHECK(its_volume(posed) > 0.f);
    CHECK(std::abs(its_volume(posed) - 8000.f) < 1.f);

    // a fin volume is not part of the analysed part
    ModelVolume *fin = obj->add_volume(make_cube(2., 2., 2.));
    fin->support_fin = SupportFinInfo{"prop", 1, "x"};
    CHECK(obj->has_support_fins());
    CHECK(support_fins_posed_part(*obj, 0).indices.size() == 12);

    // the hash ignores where the object stands on the plate, not its pose
    const std::string h0 = support_fins_source_hash(posed, SupportFinOptions());
    inst->set_offset(Vec3d(30., 40., 10.));
    CHECK(support_fins_source_hash(support_fins_posed_part(*obj, 0), SupportFinOptions()) == h0);
    inst->set_rotation(Vec3d(Geometry::deg2rad(40.), 0., 0.));
    CHECK(support_fins_source_hash(support_fins_posed_part(*obj, 0), SupportFinOptions()) != h0);
    CHECK(support_fins_source_hash(posed, non_default_options()) != h0);
}

TEST_CASE("Support fins 3mf round trip", "[SupportFins][3mf]")
{
    namespace fs = boost::filesystem;
    const fs::path tmp = fs::temp_directory_path() / fs::unique_path("support_fins_%%%%-%%%%");
    fs::create_directories(tmp);
    set_temporary_dir(tmp.string());
    const std::string path = (tmp / "support_fins.3mf").string();

    const SupportFinOptions options = non_default_options();
    SupportFinInfo info;
    info.kind        = "prop";
    info.piece_id    = 3;
    info.source_hash = "feedface00c0ffee";

    {
        Model src;
        ModelObject *obj = src.add_object();
        obj->name = "cube";
        obj->add_volume(make_cube(20., 20., 20.));
        ModelVolume *fin = obj->add_volume(make_cube(2., 10., 5.));
        fin->name        = "Fin 1 (wall)";
        fin->support_fin = info;
        obj->support_fin_options = options;
        obj->add_instance();

        StoreParams params;
        params.path     = path.c_str();
        params.model    = &src;
        DynamicPrintConfig config;
        params.config   = &config;
        params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
        REQUIRE(store_bbs_3mf(params));
    }

    Model dst;
    DynamicPrintConfig        config;
    ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs             plates;
    std::vector<Preset *>     presets;
    bool                      is_bbl = false;
    Semver                    version;
    const bool ok = load_bbs_3mf(path.c_str(), &config, &ctxt, &dst, &plates, &presets, &is_bbl, &version, nullptr, LoadStrategy::LoadModel);
    release_PlateData_list(plates);
    fs::remove_all(tmp);
    REQUIRE(ok);
    REQUIRE(dst.objects.size() == 1);

    const ModelObject *obj = dst.objects.front();
    REQUIRE(obj->support_fin_options.has_value());
    CHECK(*obj->support_fin_options == options);
    REQUIRE(obj->volumes.size() == 2);
    CHECK(obj->has_support_fins());

    int tagged = 0;
    for (const ModelVolume *v : obj->volumes)
        if (v->is_support_fin()) {
            ++tagged;
            CHECK(v->support_fin->kind == info.kind);
            CHECK(v->support_fin->piece_id == info.piece_id);
            CHECK(v->support_fin->source_hash == info.source_hash);
            CHECK(v->name == "Fin 1 (wall)");
        }
    CHECK(tagged == 1);
}

TEST_CASE("Support fins generate smoke test", "[SupportFins]")
{
    const indexed_triangle_set cube = tilted_cube();

    SupportFinResult result;
    REQUIRE_NOTHROW(result = generate_support_fins(cube, SupportFinOptions()));
    CHECK(result.stats.walls >= 0);
    CHECK_FALSE(support_fins_report(result.stats).empty());

    SupportFinOptions bad;
    bad.material = "abs";
    CHECK_THROWS_AS(generate_support_fins(cube, bad), std::invalid_argument);

    // the same through the model: the object must stay intact whatever the engine returns
    Model model;
    ModelObject *obj = model.add_object();
    obj->add_volume(make_cube(20., 20., 20.));
    ModelInstance *inst = obj->add_instance();
    inst->set_rotation(Vec3d(Geometry::deg2rad(40.), 0., 0.));
    REQUIRE_NOTHROW(apply_support_fins(*obj, SupportFinOptions()));
    REQUIRE(obj->support_fin_options.has_value());
    CHECK_FALSE(support_fins_stale(*obj));
    CHECK(support_fins_posed_part(*obj, 0).indices.size() == 12);
    const int fins = int(obj->volumes.size()) - 1;
    CHECK(clear_support_fins(*obj) == fins);
    CHECK_FALSE(obj->has_support_fins());
    CHECK_FALSE(obj->support_fin_options.has_value());

    std::string reason;
    CHECK_FALSE(add_drawn_support_fin(*obj, Vec3d(0., 0., 5.), Vec3d(0., 10., 5.), SupportFinOptions(), reason));
    CHECK_FALSE(reason.empty());
}
