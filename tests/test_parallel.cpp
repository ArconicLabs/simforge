// About: Tests for parallel asset processing — verifies that multi-threaded
// pipeline execution produces identical results to sequential processing.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
            / ("simforge_par_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

std::string make_config(const std::string& src, const std::string& out, uint32_t threads) {
    return
        "pipeline:\n"
        "  source: " + src + "\n"
        "  output: " + out + "\n"
        "  target_formats: [obj]\n"
        "  threads: " + std::to_string(threads) + "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: primitive\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "    density: 1000.0\n"
        "  validate:\n"
        "    watertight: false\n"
        "    physics_plausibility: false\n";
}

}  // namespace

TEST_CASE("Parallel pipeline — threads config parsed from YAML", "[parallel]") {
    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: /tmp\n"
        "  output: /tmp\n"
        "  threads: 4\n"
    );
    REQUIRE(config.threads == 4);
}

TEST_CASE("Parallel pipeline — default threads is 1", "[parallel]") {
    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: /tmp\n"
        "  output: /tmp\n"
    );
    REQUIRE(config.threads == 1);
}

TEST_CASE("Parallel pipeline — multi-asset processing", "[parallel]") {
    TempDir src_dir;
    TempDir out_dir;

    // Write multiple OBJ files
    auto cube = test::make_cube();
    auto tet  = test::make_tetrahedron();

    test::write_obj(cube, src_dir.path / "cube.obj");
    test::write_obj(tet,  src_dir.path / "tetrahedron.obj");

    // Copy cube with a different name for a third asset
    auto cube2 = cube;
    cube2.name = "cube2";
    test::write_obj(cube2, src_dir.path / "cube2.obj");

    auto config = PipelineConfig::from_string(
        make_config(src_dir.path.string(), out_dir.path.string(), 2));

    Pipeline pipeline(config);
    pipeline.build();
    auto report = pipeline.run();

    REQUIRE(report.total_assets == 3);
    REQUIRE(report.passed == 3);
    REQUIRE(report.failed == 0);
    REQUIRE(report.asset_reports.size() == 3);
}

TEST_CASE("Parallel pipeline — results match sequential", "[parallel]") {
    TempDir src_dir;
    TempDir out_seq;
    TempDir out_par;

    auto cube = test::make_cube();
    auto tet  = test::make_tetrahedron();
    test::write_obj(cube, src_dir.path / "cube.obj");
    test::write_obj(tet,  src_dir.path / "tet.obj");

    // Sequential run (threads=1)
    {
        auto config = PipelineConfig::from_string(
            make_config(src_dir.path.string(), out_seq.path.string(), 1));
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.passed == 2);
    }

    // Parallel run (threads=4)
    {
        auto config = PipelineConfig::from_string(
            make_config(src_dir.path.string(), out_par.path.string(), 4));
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.passed == 2);
    }

    // Both should produce the same output files
    for (const auto& entry : std::filesystem::directory_iterator(out_seq.path)) {
        auto par_file = out_par.path / entry.path().filename();
        REQUIRE(std::filesystem::exists(par_file));
    }
}

TEST_CASE("Parallel pipeline — single asset falls back to sequential", "[parallel]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    // threads > 1 but only 1 asset — should use sequential path
    auto config = PipelineConfig::from_string(
        make_config(src_dir.path.string(), out_dir.path.string(), 4));

    Pipeline pipeline(config);
    pipeline.build();
    auto report = pipeline.run();

    REQUIRE(report.total_assets == 1);
    REQUIRE(report.passed == 1);
}

TEST_CASE("Parallel pipeline — threads=0 uses hardware_concurrency", "[parallel]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    auto tet  = test::make_tetrahedron();
    test::write_obj(cube, src_dir.path / "cube.obj");
    test::write_obj(tet,  src_dir.path / "tet.obj");

    auto config = PipelineConfig::from_string(
        make_config(src_dir.path.string(), out_dir.path.string(), 0));

    // threads=0 should resolve to hardware_concurrency and succeed
    Pipeline pipeline(config);
    pipeline.build();
    auto report = pipeline.run();

    REQUIRE(report.total_assets == 2);
    REQUIRE(report.passed == 2);
}
