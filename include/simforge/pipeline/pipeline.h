#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "simforge/core/types.h"
#include "simforge/pipeline/stage.h"

namespace simforge {

namespace fs = std::filesystem;

// ─── Pipeline Configuration ────────────────────────────────────────

struct PipelineConfig {
    fs::path                source_dir;
    fs::path                output_dir;
    std::vector<SourceFormat>   target_formats;   // one or more output formats
    std::vector<std::string>    stage_order;
    uint32_t                threads{1};       // 0 = auto (hardware_concurrency), 1 = sequential
    YAML::Node              raw;            // full parsed YAML for stage-level config

    static PipelineConfig from_file(const fs::path& config_path);
    static PipelineConfig from_string(const std::string& yaml_str);
};

// ─── Pipeline Report ───────────────────────────────────────────────

struct AssetReport {
    std::string                     asset_id;
    std::string                     asset_name;
    AssetStatus                     final_status;
    std::vector<ValidationResult>   validations;
    std::vector<std::string>        stages_completed;
    std::vector<StageError>         errors;
    double                          processing_time_ms{0.0};
};

struct PipelineReport {
    size_t                      total_assets{0};
    size_t                      passed{0};
    size_t                      failed{0};
    double                      total_time_ms{0.0};
    std::vector<AssetReport>    asset_reports;

    void print_summary() const;
    void write_json(const fs::path& path) const;
};

// ─── Pipeline ──────────────────────────────────────────────────────

class Pipeline {
public:
    explicit Pipeline(PipelineConfig config);

    /// Build the stage chain from config. Call after construction.
    void build();

    /// Discover assets in source directory.
    [[nodiscard]] std::vector<Asset> discover_assets() const;

    /// Run the full pipeline on all discovered assets.
    PipelineReport run();

    /// Run the pipeline on a single asset.
    AssetReport run_single(Asset asset);

    /// Dry-run: discover + validate config without processing.
    void dry_run() const;

    /// Get the list of stages that will be executed.
    [[nodiscard]] std::vector<std::string> stage_names() const;

private:
    PipelineConfig          config_;
    std::vector<StagePtr>   stages_;

    Asset run_stages(Asset asset, AssetReport& report);
    PipelineReport run_parallel(std::vector<Asset> assets);
};

}  // namespace simforge
