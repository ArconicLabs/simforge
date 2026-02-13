#include <catch2/catch_test_macros.hpp>

#include <spdlog/spdlog.h>

#include "simforge/pipeline/pipeline.h"
#include "simforge/pipeline/stage.h"
#include "simforge/pipeline/builtin_stages.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;

// Ensure adapters and stages are available for tests
static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    simforge::stages::register_builtin_stages();
    return true;
}();

TEST_CASE("PipelineConfig from YAML string", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./test_assets/
  output: ./test_output/
  target_formats: [usd, urdf, mjcf]

stages:
  ingest:
    formats: [obj, stl]
  collision:
    method: coacd
    threshold: 0.05
  physics:
    mass_estimation: geometry
  validate:
    watertight: true
)");

    REQUIRE(config.source_dir == "./test_assets/");
    REQUIRE(config.output_dir == "./test_output/");
    REQUIRE(config.target_formats.size() == 3);
    REQUIRE(config.target_formats[0] == SourceFormat::USD);
    REQUIRE(config.target_formats[1] == SourceFormat::URDF);
    REQUIRE(config.target_formats[2] == SourceFormat::MJCF);
    REQUIRE(config.stage_order.size() == 4);
    REQUIRE(config.stage_order[0] == "ingest");
    REQUIRE(config.stage_order[1] == "collision");
}

TEST_CASE("PipelineConfig legacy single format", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./assets/
  output: ./out/
  target_format: gltf

stages:
  ingest:
    formats: [obj]
)");

    REQUIRE(config.target_formats.size() == 1);
    REQUIRE(config.target_formats[0] == SourceFormat::GLTF);
}

TEST_CASE("PipelineConfig defaults to USD", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./assets/
  output: ./out/

stages:
  ingest:
    formats: [obj]
)");

    REQUIRE(config.target_formats.size() == 1);
    REQUIRE(config.target_formats[0] == SourceFormat::USD);
}

TEST_CASE("StageRegistry", "[pipeline]") {
    auto& reg = StageRegistry::instance();

    // Built-in stages should be registered
    auto available = reg.available();
    REQUIRE(available.size() > 0);

    // Should have our core stages
    REQUIRE(reg.has("ingest"));
    REQUIRE(reg.has("collision"));
    REQUIRE(reg.has("physics"));
    REQUIRE(reg.has("validate"));
    REQUIRE(reg.has("export"));
}

TEST_CASE("Pipeline build with config", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./nonexistent/
  output: ./test_output/

stages:
  ingest:
    formats: [obj]
  validate:
    mesh_integrity: true
)");

    Pipeline pipeline(std::move(config));
    pipeline.build();

    auto names = pipeline.stage_names();
    REQUIRE(names.size() == 2);
    REQUIRE(names[0] == "ingest");
    REQUIRE(names[1] == "validate");
}

TEST_CASE("parse_format case-insensitive", "[pipeline]") {
    REQUIRE(parse_format("obj") == SourceFormat::OBJ);
    REQUIRE(parse_format("OBJ") == SourceFormat::OBJ);
    REQUIRE(parse_format("Obj") == SourceFormat::OBJ);
    REQUIRE(parse_format("stl") == SourceFormat::STL);
    REQUIRE(parse_format("GLTF") == SourceFormat::GLTF);
    REQUIRE(parse_format("nonsense") == SourceFormat::Unknown);
}

TEST_CASE("PipelineConfig with explicit stage_order", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./assets/
  output: ./out/
  stage_order: [ingest, validate, export]

stages:
  ingest:
    formats: [obj]
  validate:
    mesh_integrity: true
  export:
    catalog: true
)");

    REQUIRE(config.stage_order.size() == 3);
    REQUIRE(config.stage_order[0] == "ingest");
    REQUIRE(config.stage_order[1] == "validate");
    REQUIRE(config.stage_order[2] == "export");
}

TEST_CASE("PipelineConfig default stage order", "[pipeline]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ./assets/
  output: ./out/
)");

    // When no stages specified, should get the full default order
    REQUIRE(config.stage_order.size() == 6);
    REQUIRE(config.stage_order[0] == "ingest");
    REQUIRE(config.stage_order[5] == "export");
}
