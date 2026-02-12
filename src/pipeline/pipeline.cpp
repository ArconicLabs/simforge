#include "simforge/pipeline/pipeline.h"

#include <chrono>
#include <fstream>
#include <iomanip>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace simforge {

// ─── PipelineConfig ────────────────────────────────────────────────

PipelineConfig PipelineConfig::from_file(const fs::path& config_path) {
    if (!fs::exists(config_path)) {
        throw std::runtime_error("Config file not found: " + config_path.string());
    }
    YAML::Node raw = YAML::LoadFile(config_path.string());
    return from_string(YAML::Dump(raw));
}

PipelineConfig PipelineConfig::from_string(const std::string& yaml_str) {
    PipelineConfig config;
    config.raw = YAML::Load(yaml_str);

    auto pipeline = config.raw["pipeline"];
    if (!pipeline) {
        throw std::runtime_error("Config missing 'pipeline' top-level key");
    }

    config.source_dir = pipeline["source"].as<std::string>("./raw_assets/");
    config.output_dir = pipeline["output"].as<std::string>("./sim_ready/");

    // Parse target format(s) — supports both single and list syntax
    if (auto fmts = pipeline["target_formats"]) {
        for (const auto& f : fmts) {
            auto fmt = parse_format(f.as<std::string>());
            if (fmt != SourceFormat::Unknown)
                config.target_formats.push_back(fmt);
        }
    } else if (auto fmt = pipeline["target_format"]) {
        auto parsed = parse_format(fmt.as<std::string>("usd"));
        if (parsed != SourceFormat::Unknown)
            config.target_formats.push_back(parsed);
    }

    if (config.target_formats.empty()) {
        config.target_formats.push_back(SourceFormat::USD);
    }

    // Determine stage order from 'stages' keys or explicit 'stage_order'
    if (auto order = pipeline["stage_order"]) {
        for (const auto& s : order) {
            config.stage_order.push_back(s.as<std::string>());
        }
    } else if (auto stages = config.raw["stages"]) {
        // Use the order keys appear in YAML (yaml-cpp preserves insertion order)
        for (auto it = stages.begin(); it != stages.end(); ++it) {
            config.stage_order.push_back(it->first.as<std::string>());
        }
    }

    // Default stage order if nothing specified
    if (config.stage_order.empty()) {
        config.stage_order = {"ingest", "collision", "physics", "optimize", "validate", "export"};
    }

    return config;
}

// ─── Pipeline ──────────────────────────────────────────────────────

Pipeline::Pipeline(PipelineConfig config)
    : config_(std::move(config)) {}

void Pipeline::build() {
    auto& registry = StageRegistry::instance();
    stages_.clear();

    for (const auto& stage_name : config_.stage_order) {
        if (!registry.has(stage_name)) {
            spdlog::warn("Stage '{}' not registered, skipping", stage_name);
            continue;
        }

        auto stage = registry.create(stage_name);

        // Pass stage-specific config if present
        if (auto stages_node = config_.raw["stages"]) {
            if (auto stage_config = stages_node[stage_name]) {
                stage->configure(stage_config);
            }
        }

        spdlog::info("Loaded stage: {}", stage->name());
        stages_.push_back(std::move(stage));
    }

    spdlog::info("Pipeline built with {} stages", stages_.size());
}

std::vector<Asset> Pipeline::discover_assets() const {
    std::vector<Asset> assets;

    if (!fs::exists(config_.source_dir)) {
        spdlog::error("Source directory does not exist: {}", config_.source_dir.string());
        return assets;
    }

    uint32_t counter = 0;
    for (const auto& entry : fs::recursive_directory_iterator(config_.source_dir)) {
        if (!entry.is_regular_file()) continue;

        auto fmt = detect_format(entry.path());
        if (fmt == SourceFormat::Unknown) continue;

        Asset asset;
        asset.id            = std::to_string(counter++);
        asset.name          = entry.path().stem().string();
        asset.source_path   = entry.path();
        asset.source_format = fmt;
        asset.status        = AssetStatus::Raw;

        spdlog::debug("Discovered: {} ({})", asset.name, format_to_string(fmt));
        assets.push_back(std::move(asset));
    }

    spdlog::info("Discovered {} assets in {}", assets.size(), config_.source_dir.string());
    return assets;
}

PipelineReport Pipeline::run() {
    PipelineReport report;
    auto start = std::chrono::high_resolution_clock::now();

    auto assets = discover_assets();
    report.total_assets = assets.size();

    for (auto& asset : assets) {
        auto asset_report = run_single(std::move(asset));
        if (asset_report.final_status == AssetStatus::Ready) {
            report.passed++;
        } else {
            report.failed++;
        }
        report.asset_reports.push_back(std::move(asset_report));
    }

    auto end = std::chrono::high_resolution_clock::now();
    report.total_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    return report;
}

AssetReport Pipeline::run_single(Asset asset) {
    AssetReport report;
    report.asset_id   = asset.id;
    report.asset_name = asset.name;

    auto start = std::chrono::high_resolution_clock::now();
    asset = run_stages(std::move(asset), report);
    auto end = std::chrono::high_resolution_clock::now();

    report.processing_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    report.final_status = asset.status;
    report.validations  = asset.validations;

    return report;
}

Asset Pipeline::run_stages(Asset asset, AssetReport& report) {
    for (auto& stage : stages_) {
        if (!stage->should_run(asset)) {
            spdlog::debug("Skipping stage '{}' for asset '{}'",
                          stage->name(), asset.name);
            continue;
        }

        spdlog::info("[{}] Processing '{}'...", stage->name(), asset.name);

        auto result = stage->process(std::move(asset));

        if (result.is_err()) {
            spdlog::error("[{}] Failed for '{}': {}",
                          result.error().stage_name,
                          result.error().asset_id,
                          result.error().message);
            report.errors.push_back(result.error());

            // Recover the asset in failed state
            asset = result.value();  // may be partial
            asset.status = AssetStatus::Failed;
            return asset;
        }

        asset = std::move(result.value());
        report.stages_completed.push_back(stage->name());
    }

    // If we made it through all stages, mark as ready
    if (asset.status != AssetStatus::Failed) {
        asset.status = AssetStatus::Ready;
    }

    return asset;
}

void Pipeline::dry_run() const {
    spdlog::info("=== DRY RUN ===");
    spdlog::info("Source:  {}", config_.source_dir.string());
    spdlog::info("Output:  {}", config_.output_dir.string());

    std::string fmt_list;
    for (const auto& f : config_.target_formats) {
        if (!fmt_list.empty()) fmt_list += ", ";
        fmt_list += format_to_string(f);
    }
    spdlog::info("Targets: {}", fmt_list);
    spdlog::info("Stages:");
    for (const auto& s : config_.stage_order) {
        auto& reg = StageRegistry::instance();
        spdlog::info("  {} {}", reg.has(s) ? "[OK]" : "[!!]", s);
    }

    auto assets = discover_assets();
    spdlog::info("Would process {} assets", assets.size());
    for (const auto& a : assets) {
        spdlog::info("  {} ({})", a.name, format_to_string(a.source_format));
    }
}

std::vector<std::string> Pipeline::stage_names() const {
    std::vector<std::string> names;
    for (const auto& s : stages_) names.push_back(s->name());
    return names;
}

// ─── PipelineReport ────────────────────────────────────────────────

void PipelineReport::print_summary() const {
    spdlog::info("════════════════════════════════════════");
    spdlog::info("  Pipeline Report");
    spdlog::info("════════════════════════════════════════");
    spdlog::info("  Total assets:  {}", total_assets);
    spdlog::info("  Passed:        {}", passed);
    spdlog::info("  Failed:        {}", failed);
    spdlog::info("  Time:          {:.1f} ms", total_time_ms);
    spdlog::info("════════════════════════════════════════");

    for (const auto& ar : asset_reports) {
        auto status = (ar.final_status == AssetStatus::Ready) ? "PASS" : "FAIL";
        spdlog::info("  [{}] {} ({:.0f} ms)", status, ar.asset_name,
                     ar.processing_time_ms);
        for (const auto& err : ar.errors) {
            spdlog::info("    ERROR [{}]: {}", err.stage_name, err.message);
        }
        for (const auto& v : ar.validations) {
            if (!v.passed) {
                spdlog::info("    WARN  [{}]: {}", v.check_name, v.message);
            }
        }
    }
}

void PipelineReport::write_json(const fs::path& path) const {
    nlohmann::json j;
    j["total_assets"]   = total_assets;
    j["passed"]         = passed;
    j["failed"]         = failed;
    j["total_time_ms"]  = total_time_ms;

    j["assets"] = nlohmann::json::array();
    for (const auto& ar : asset_reports) {
        nlohmann::json aj;
        aj["id"]                = ar.asset_id;
        aj["name"]              = ar.asset_name;
        aj["status"]            = static_cast<int>(ar.final_status);
        aj["time_ms"]           = ar.processing_time_ms;
        aj["stages_completed"]  = ar.stages_completed;

        aj["errors"] = nlohmann::json::array();
        for (const auto& e : ar.errors) {
            aj["errors"].push_back({
                {"stage", e.stage_name},
                {"asset", e.asset_id},
                {"message", e.message},
            });
        }

        aj["validations"] = nlohmann::json::array();
        for (const auto& v : ar.validations) {
            nlohmann::json vj;
            to_json(vj, v);
            aj["validations"].push_back(vj);
        }

        j["assets"].push_back(aj);
    }

    std::ofstream out(path);
    out << j.dump(2);
    spdlog::info("Report written to {}", path.string());
}

}  // namespace simforge
