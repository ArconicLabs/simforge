// About: Tests for incremental processing — SHA-256 hashing, content hash
// determinism, skip logic for unchanged assets, and --force override.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "simforge/core/hashing.h"
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
            / ("simforge_inc_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

std::string make_config(const std::string& src, const std::string& out) {
    return
        "pipeline:\n"
        "  source: " + src + "\n"
        "  output: " + out + "\n"
        "  target_formats: [obj]\n"
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
        "    physics_plausibility: false\n"
        "  export:\n"
        "    catalog: true\n";
}

}  // namespace

// ─── SHA-256 Hashing Unit Tests ──────────────────────────────────

TEST_CASE("sha256_string — known vector", "[incremental]") {
    // SHA-256 of empty string
    auto hash = sha256_string("");
    REQUIRE(hash == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST_CASE("sha256_string — deterministic", "[incremental]") {
    auto a = sha256_string("hello world");
    auto b = sha256_string("hello world");
    REQUIRE(a == b);
    REQUIRE(a.size() == 64);  // 32 bytes = 64 hex chars
}

TEST_CASE("sha256_string — different inputs differ", "[incremental]") {
    auto a = sha256_string("hello");
    auto b = sha256_string("world");
    REQUIRE(a != b);
}

TEST_CASE("sha256_file — matches string hash", "[incremental]") {
    TempDir tmp;
    auto path = tmp.path / "test.txt";
    {
        std::ofstream out(path);
        out << "test content";
    }
    auto file_hash = sha256_file(path);
    auto str_hash  = sha256_string("test content");
    REQUIRE(file_hash == str_hash);
}

TEST_CASE("sha256_file — throws on missing file", "[incremental]") {
    REQUIRE_THROWS_AS(sha256_file("/nonexistent/file"), std::runtime_error);
}

TEST_CASE("compute_asset_hash — includes config in hash", "[incremental]") {
    TempDir tmp;
    auto path = tmp.path / "asset.obj";
    {
        std::ofstream out(path);
        out << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    }

    auto hash_a = compute_asset_hash(path, "config_a");
    auto hash_b = compute_asset_hash(path, "config_b");
    REQUIRE(hash_a != hash_b);

    // Same config produces same hash
    auto hash_a2 = compute_asset_hash(path, "config_a");
    REQUIRE(hash_a == hash_a2);
}

// ─── Incremental Pipeline Tests ──────────────────────────────────

TEST_CASE("Incremental — second run skips unchanged assets", "[incremental][integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto yaml = make_config(src_dir.path.string(), out_dir.path.string());

    // First run — should process the asset
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }

    // Verify catalog was written with content_hash
    auto catalog_path = out_dir.path / "cube.catalog.json";
    REQUIRE(std::filesystem::exists(catalog_path));
    {
        std::ifstream in(catalog_path);
        auto catalog = nlohmann::json::parse(in);
        REQUIRE(catalog.contains("content_hash"));
        REQUIRE(!catalog["content_hash"].get<std::string>().empty());
    }

    // Second run — should skip the asset (total_assets = 0 because it was filtered)
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 0);
    }
}

TEST_CASE("Incremental — modified source triggers reprocess", "[incremental][integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto yaml = make_config(src_dir.path.string(), out_dir.path.string());

    // First run
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.passed == 1);
    }

    // Modify the source file
    {
        auto tet = test::make_tetrahedron();
        test::write_obj(tet, src_dir.path / "cube.obj");  // overwrite with different geometry
    }

    // Second run — should reprocess because hash changed
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }
}

TEST_CASE("Incremental — --force reprocesses unchanged assets", "[incremental][integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto yaml = make_config(src_dir.path.string(), out_dir.path.string());

    // First run
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.passed == 1);
    }

    // Second run with force — should reprocess even though unchanged
    {
        auto config = PipelineConfig::from_string(yaml);
        config.force = true;
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }
}

TEST_CASE("Incremental — config change triggers reprocess", "[incremental][integration]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    // Run with density 1000
    {
        auto config = PipelineConfig::from_string(make_config(
            src_dir.path.string(), out_dir.path.string()));
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.passed == 1);
    }

    // Run with density 7800 — different config → different hash → should reprocess
    auto yaml_modified =
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: primitive\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "    density: 7800.0\n"
        "  validate:\n"
        "    watertight: false\n"
        "    physics_plausibility: false\n"
        "  export:\n"
        "    catalog: true\n";

    {
        auto config = PipelineConfig::from_string(yaml_modified);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }
}

TEST_CASE("PipelineConfig — force flag parsed from YAML", "[incremental]") {
    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: /tmp\n"
        "  output: /tmp\n"
        "  force: true\n"
    );
    REQUIRE(config.force == true);
}

TEST_CASE("PipelineConfig — force defaults to false", "[incremental]") {
    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: /tmp\n"
        "  output: /tmp\n"
    );
    REQUIRE(config.force == false);
}
