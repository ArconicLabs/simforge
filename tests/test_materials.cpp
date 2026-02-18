// About: Unit and integration tests for MaterialLibrary and lookup
// mass estimation mode in PhysicsStage.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <fstream>

#include <spdlog/spdlog.h>

#include "simforge/core/material_library.h"
#include "simforge/pipeline/builtin_stages.h"
#include "simforge/pipeline/pipeline.h"
#include "test_helpers.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;
using Catch::Matchers::WithinAbs;

static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    simforge::stages::register_builtin_stages();
    return true;
}();

static constexpr auto kTestYAML = R"(
materials:
  steel:
    density: 7800.0
    static_friction: 0.6
    dynamic_friction: 0.42
    restitution: 0.25
  rubber:
    density: 1100.0
    static_friction: 1.0
    dynamic_friction: 0.8
    restitution: 0.6
  aluminum:
    density: 2700.0
    static_friction: 0.47
    dynamic_friction: 0.35
    restitution: 0.3
)";

// ─── MaterialLibrary Unit Tests ──────────────────────────────────

TEST_CASE("MaterialLibrary — from_string parses materials", "[materials]") {
    auto lib = MaterialLibrary::from_string(kTestYAML);

    REQUIRE(lib.size() == 3);
    REQUIRE(lib.has("steel"));
    REQUIRE(lib.has("rubber"));
    REQUIRE(lib.has("aluminum"));
    REQUIRE_FALSE(lib.has("wood"));
}

TEST_CASE("MaterialLibrary — find returns correct properties", "[materials]") {
    auto lib = MaterialLibrary::from_string(kTestYAML);

    auto* steel = lib.find("steel");
    REQUIRE(steel != nullptr);
    REQUIRE_THAT(steel->density, WithinAbs(7800.0f, 0.1f));
    REQUIRE_THAT(steel->static_friction, WithinAbs(0.6f, 0.01f));
    REQUIRE_THAT(steel->dynamic_friction, WithinAbs(0.42f, 0.01f));
    REQUIRE_THAT(steel->restitution, WithinAbs(0.25f, 0.01f));
    REQUIRE(steel->name == "steel");
}

TEST_CASE("MaterialLibrary — case-insensitive lookup", "[materials]") {
    auto lib = MaterialLibrary::from_string(kTestYAML);

    REQUIRE(lib.find("Steel") != nullptr);
    REQUIRE(lib.find("STEEL") != nullptr);
    REQUIRE(lib.find("sTeEl") != nullptr);

    // All should return the same density
    REQUIRE(lib.find("steel")->density == lib.find("STEEL")->density);
}

TEST_CASE("MaterialLibrary — find returns nullptr for unknown", "[materials]") {
    auto lib = MaterialLibrary::from_string(kTestYAML);
    REQUIRE(lib.find("titanium") == nullptr);
}

TEST_CASE("MaterialLibrary — names returns original case", "[materials]") {
    auto lib = MaterialLibrary::from_string(kTestYAML);
    auto names = lib.names();
    REQUIRE(names.size() == 3);
    // YAML map iteration order: check all three are present
    bool has_steel = false, has_rubber = false, has_aluminum = false;
    for (const auto& n : names) {
        if (n == "steel") has_steel = true;
        if (n == "rubber") has_rubber = true;
        if (n == "aluminum") has_aluminum = true;
    }
    REQUIRE(has_steel);
    REQUIRE(has_rubber);
    REQUIRE(has_aluminum);
}

TEST_CASE("MaterialLibrary — optional fields use defaults", "[materials]") {
    auto lib = MaterialLibrary::from_string(R"(
materials:
  minimal:
    density: 5000.0
)");
    auto* mat = lib.find("minimal");
    REQUIRE(mat != nullptr);
    REQUIRE_THAT(mat->density, WithinAbs(5000.0f, 0.1f));
    REQUIRE_THAT(mat->static_friction, WithinAbs(0.5f, 0.01f));
    REQUIRE_THAT(mat->dynamic_friction, WithinAbs(0.4f, 0.01f));  // 0.5 * 0.8
    REQUIRE_THAT(mat->restitution, WithinAbs(0.3f, 0.01f));
}

TEST_CASE("MaterialLibrary — from_file loads YAML", "[materials]") {
    auto tmp = std::filesystem::temp_directory_path() / "test_materials.yaml";
    {
        std::ofstream out(tmp);
        out << kTestYAML;
    }

    auto lib = MaterialLibrary::from_file(tmp);
    REQUIRE(lib.size() == 3);
    REQUIRE(lib.find("steel") != nullptr);

    std::filesystem::remove(tmp);
}

TEST_CASE("MaterialLibrary — from_file throws on missing file", "[materials]") {
    REQUIRE_THROWS_AS(
        MaterialLibrary::from_file("/nonexistent/path.yaml"),
        std::runtime_error);
}

TEST_CASE("MaterialLibrary — missing materials key throws", "[materials]") {
    REQUIRE_THROWS_AS(
        MaterialLibrary::from_string("other_key: 42"),
        std::runtime_error);
}

TEST_CASE("MaterialLibrary — empty library has size 0", "[materials]") {
    auto lib = MaterialLibrary::from_string("materials: {}");
    REQUIRE(lib.size() == 0);
    REQUIRE(lib.names().empty());
}

// ─── PhysicsStage Lookup Integration ─────────────────────────────

namespace {

struct TempDir {
    std::filesystem::path path;

    TempDir() {
        path = std::filesystem::temp_directory_path()
            / ("simforge_mat_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

}  // namespace

TEST_CASE("PhysicsStage — lookup mode uses material library", "[materials][integration]") {
    TempDir tmp;

    // Write a material library
    auto lib_path = tmp.path / "materials.yaml";
    {
        std::ofstream out(lib_path);
        out << kTestYAML;
    }

    // Write a test OBJ
    auto cube = test::make_cube();
    test::write_obj(cube, tmp.path / "cube.obj");

    TempDir out_dir;

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + tmp.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: primitive\n"
        "  physics:\n"
        "    mass_estimation: lookup\n"
        "    material_library: " + lib_path.string() + "\n"
        "  validate:\n"
        "    watertight: false\n"
        "    physics_plausibility: false\n"
    );

    Pipeline pipeline(config);
    pipeline.build();
    auto report = pipeline.run();

    REQUIRE(report.total_assets == 1);
    REQUIRE(report.passed == 1);

    // With no material hint and no PBR material name, lookup mode falls
    // back to geometry estimation with default material (density 1000).
    auto& asset_report = report.asset_reports[0];
    REQUIRE(asset_report.errors.empty());
}

TEST_CASE("PhysicsStage — lookup resolves metadata material", "[materials][integration]") {
    TempDir tmp;

    auto lib_path = tmp.path / "materials.yaml";
    {
        std::ofstream out(lib_path);
        out << kTestYAML;
    }

    // Configure PhysicsStage directly
    stages::PhysicsStage stage;
    auto config_yaml = YAML::Load(
        "mass_estimation: lookup\n"
        "material_library: " + lib_path.string() + "\n"
    );
    stage.configure(config_yaml);

    // Build an asset with material metadata
    Asset asset;
    asset.name = "steel_cube";
    asset.id = "test_steel";
    asset.status = AssetStatus::CollisionGenerated;
    asset.meshes.push_back(test::make_cube());
    asset.metadata["material"] = "steel";

    auto result = stage.process(std::move(asset));
    REQUIRE(result.is_ok());

    auto& processed = result.value();
    REQUIRE(processed.physics.has_value());
    // Steel density is 7800 — mass should reflect that, not default 1000
    REQUIRE_THAT(processed.physics->material.density, WithinAbs(7800.0f, 0.1f));
    REQUIRE(processed.physics->material.name == "steel");
}
