// End-to-end pipeline integration tests using programmatic fixtures.
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <spdlog/spdlog.h>

#include "simforge/pipeline/pipeline.h"
#include "simforge/pipeline/builtin_stages.h"
#include "test_helpers.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;

static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    simforge::stages::register_builtin_stages();
    return true;
}();

namespace {

struct TempDir {
    std::filesystem::path path;

    TempDir() {
        path = std::filesystem::temp_directory_path()
            / ("simforge_integ_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

}  // namespace

// ─── Full Pipeline ───────────────────────────────────────────────

TEST_CASE("Full pipeline — OBJ cube end-to-end", "[integration]") {
    TempDir src_dir;
    TempDir out_dir;

    // Write cube OBJ to source directory
    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
        "    scale_sanity: true\n"
        "  export:\n"
        "    catalog: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto report = pipeline.run();

    REQUIRE(report.total_assets == 1);
    REQUIRE(report.failed == 0);
    REQUIRE(report.passed == 1);

    auto& ar = report.asset_reports[0];
    REQUIRE(ar.final_status != AssetStatus::Failed);
    REQUIRE(ar.errors.empty());
    REQUIRE(ar.stages_completed.size() >= 4);
    REQUIRE(ar.stages_completed[0] == "ingest");
}

// ─── Partial Pipeline ────────────────────────────────────────────

TEST_CASE("Partial pipeline — ingest + validate only", "[integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto names = pipeline.stage_names();
    REQUIRE(names.size() == 2);
    REQUIRE(names[0] == "ingest");
    REQUIRE(names[1] == "validate");

    auto report = pipeline.run();

    REQUIRE(report.total_assets == 1);
    auto& ar = report.asset_reports[0];
    // ValidateStage sets status to Validated, not Ready (no export stage)
    REQUIRE(ar.final_status == AssetStatus::Validated);
}

// ─── Export Stage ────────────────────────────────────────────────

TEST_CASE("Export stage — catalog.json is written", "[integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
        "  export:\n"
        "    output_dir: " + out_dir.path.string() + "\n"
        "    catalog: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();
    pipeline.run();

    // Catalog should be written to output directory
    auto catalog_path = out_dir.path / "cube.catalog.json";
    REQUIRE(std::filesystem::exists(catalog_path));

    // Verify it contains expected fields
    std::ifstream f(catalog_path);
    std::string contents((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    auto json = nlohmann::json::parse(contents);

    REQUIRE(json.contains("id"));
    REQUIRE(json.contains("name"));
    REQUIRE(json["name"] == "cube");
    REQUIRE(json.contains("mesh_count"));
    REQUIRE(json.contains("total_triangles"));
}

// ─── Error Handling ──────────────────────────────────────────────

TEST_CASE("Pipeline — nonexistent source directory", "[integration]") {
    TempDir out_dir;

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: /tmp/simforge_does_not_exist_12345/\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto report = pipeline.run();

    // No assets discovered → nothing to process
    REQUIRE(report.total_assets == 0);
    REQUIRE(report.passed == 0);
    REQUIRE(report.failed == 0);
}

TEST_CASE("Pipeline — unrecognized file format", "[integration]") {
    TempDir src_dir;
    TempDir out_dir;

    // Write a file with an unrecognized extension
    {
        std::ofstream out(src_dir.path / "model.xyz123");
        out << "not a real format\n";
    }

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto report = pipeline.run();

    // .xyz123 is not a recognized format, so discover_assets skips it
    REQUIRE(report.total_assets == 0);
}

// ─── Single Asset Pipeline ───────────────────────────────────────

TEST_CASE("Export stage — all four export formats", "[integration][exporters]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [usda, urdf, mjcf, glb]\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
        "  export:\n"
        "    output_dir: " + out_dir.path.string() + "\n"
        "    catalog: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto report = pipeline.run();

    REQUIRE(report.total_assets == 1);
    REQUIRE(report.failed == 0);

    // Verify each export format produced a file
    REQUIRE(std::filesystem::exists(out_dir.path / "usda" / "cube.usda"));
    REQUIRE(std::filesystem::exists(out_dir.path / "urdf" / "cube.urdf"));
    REQUIRE(std::filesystem::exists(out_dir.path / "mjcf" / "cube.xml"));
    REQUIRE(std::filesystem::exists(out_dir.path / "glb" / "cube.glb"));

    // All should be non-empty
    REQUIRE(std::filesystem::file_size(out_dir.path / "usda" / "cube.usda") > 0);
    REQUIRE(std::filesystem::file_size(out_dir.path / "urdf" / "cube.urdf") > 0);
    REQUIRE(std::filesystem::file_size(out_dir.path / "mjcf" / "cube.xml") > 0);
    REQUIRE(std::filesystem::file_size(out_dir.path / "glb" / "cube.glb") > 0);

    // Verify catalog has export_targets for all 4
    auto catalog_path = out_dir.path / "cube.catalog.json";
    REQUIRE(std::filesystem::exists(catalog_path));

    std::ifstream f(catalog_path);
    std::string contents((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    auto json = nlohmann::json::parse(contents);

    REQUIRE(json.contains("export_targets"));
    auto& targets = json["export_targets"];
    REQUIRE(targets.contains("usda"));
    REQUIRE(targets.contains("urdf"));
    REQUIRE(targets.contains("mjcf"));
    REQUIRE(targets.contains("glb"));
}

// ─── LOD Integration ─────────────────────────────────────────────

TEST_CASE("Pipeline — optimize stage generates LOD meshes via meshoptimizer", "[integration][lod]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  optimize:\n"
        "    lod_levels: [medium, low]\n"
        "    max_triangles: [8, 4]\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
        "  export:\n"
        "    catalog: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    // Use run_single so we can inspect the asset directly
    Asset asset;
    asset.id = "lod-test";
    asset.name = "cube";
    asset.source_path = src_dir.path / "cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    REQUIRE(report.errors.empty());
    REQUIRE(report.stages_completed.size() >= 5);

    // The optimize stage should have been completed
    bool optimize_ran = false;
    for (const auto& s : report.stages_completed) {
        if (s == "optimize") optimize_ran = true;
    }
    REQUIRE(optimize_ran);
}

TEST_CASE("Pipeline — collision stage with primitive method", "[integration][collision]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: primitive\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "prim-test";
    asset.name = "cube";
    asset.source_path = src_dir.path / "cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    REQUIRE(report.errors.empty());

    bool collision_ran = false;
    for (const auto& s : report.stages_completed) {
        if (s == "collision") collision_ran = true;
    }
    REQUIRE(collision_ran);
}

// ─── Single Asset Pipeline ───────────────────────────────────────

TEST_CASE("Pipeline — run_single with programmatic asset", "[integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    auto obj_path = src_dir.path / "single.obj";
    test::write_obj(cube, obj_path);

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "test-single";
    asset.name = "single";
    asset.source_path = obj_path;
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    REQUIRE(report.final_status == AssetStatus::Validated);
    REQUIRE(report.errors.empty());
    REQUIRE(report.stages_completed.size() == 2);
}
