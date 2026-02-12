#include <catch2/catch_test_macros.hpp>

#include "simforge/pipeline/pipeline.h"
#include "simforge/pipeline/stage.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;

// Ensure adapters are available for integration-level tests
static const bool _init = [] {
    simforge::adapters::register_builtin_adapters();
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
