#include "simforge/pipeline/pipeline.h"

#include <atomic>
#include <chrono>
#include <fstream>
#include <thread>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "simforge/core/hashing.h"

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

    // Thread count: 0 = auto, 1 = sequential (default)
    config.threads = pipeline["threads"].as<uint32_t>(1);
    config.force   = pipeline["force"].as<bool>(false);

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

        // Build effective config: start from YAML, inject pipeline-level
        // settings where the stage config doesn't override them
        YAML::Node stage_config;
        if (auto stages_node = config_.raw["stages"]) {
            if (stages_node[stage_name]) {
                stage_config = YAML::Clone(stages_node[stage_name]);
            }
        }

        // Inject pipeline-level target_formats into export stage when
        // it doesn't specify its own formats
        if (stage_name == "export" &&
            !stage_config["formats"] && !stage_config["format"]) {
            for (const auto& f : config_.target_formats) {
                auto s = format_to_string(f);
                std::transform(s.begin(), s.end(), s.begin(), ::tolower);
                stage_config["formats"].push_back(s);
            }
        }

        stage->configure(stage_config);

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

/// Check if an asset can be skipped based on its catalog hash.
bool Pipeline::should_skip_asset(const Asset& asset) const {
    if (config_.force || asset.content_hash.empty()) return false;

    auto catalog_path = config_.output_dir / (asset.name + ".catalog.json");
    if (!fs::exists(catalog_path)) return false;

    try {
        std::ifstream in(catalog_path);
        auto catalog = nlohmann::json::parse(in);
        if (catalog.contains("content_hash") &&
            catalog["content_hash"].get<std::string>() == asset.content_hash) {
            return true;
        }
    } catch (...) {
        // Corrupted catalog — reprocess
    }
    return false;
}

/// Canonical YAML dump of the stages config section for hash keying.
std::string Pipeline::stages_config_yaml() const {
    if (auto stages = config_.raw["stages"]) {
        return YAML::Dump(stages);
    }
    return {};
}

PipelineReport Pipeline::run() {
    auto start = std::chrono::high_resolution_clock::now();
    auto assets = discover_assets();

    // Compute content hashes for incremental processing
    auto config_yaml = stages_config_yaml();
    for (auto& asset : assets) {
        try {
            asset.content_hash = compute_asset_hash(asset.source_path, config_yaml);
        } catch (...) {
            // Hash failure is non-fatal — asset will be processed
        }
    }

    // Filter out unchanged assets (unless --force)
    std::vector<Asset> to_process;
    size_t skipped = 0;
    for (auto& asset : assets) {
        if (should_skip_asset(asset)) {
            spdlog::info("Skipping unchanged asset: {}", asset.name);
            skipped++;
        } else {
            to_process.push_back(std::move(asset));
        }
    }
    if (skipped > 0) {
        spdlog::info("Skipped {} unchanged asset(s)", skipped);
    }

    // Resolve effective thread count
    uint32_t effective_threads = config_.threads;
    if (effective_threads == 0) {
        effective_threads = std::max(1u, std::thread::hardware_concurrency());
    }

    PipelineReport report;
    report.total_assets = to_process.size();

    if (effective_threads > 1 && to_process.size() > 1) {
        report = run_parallel(std::move(to_process));
    } else {
        for (auto& asset : to_process) {
            auto asset_report = run_single(std::move(asset));
            if (asset_report.final_status == AssetStatus::Failed) {
                report.failed++;
            } else {
                report.passed++;
            }
            report.asset_reports.push_back(std::move(asset_report));
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    report.total_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    return report;
}

PipelineReport Pipeline::run_parallel(std::vector<Asset> assets) {
    uint32_t effective_threads = config_.threads;
    if (effective_threads == 0)
        effective_threads = std::max(1u, std::thread::hardware_concurrency());
    effective_threads = std::min(effective_threads, static_cast<uint32_t>(assets.size()));

    spdlog::info("Processing {} assets with {} threads", assets.size(), effective_threads);

    // Pre-create output directory to avoid filesystem races
    fs::create_directories(config_.output_dir);

    // Pre-size results vector for lock-free positional writes
    PipelineReport report;
    report.total_assets = assets.size();
    report.asset_reports.resize(assets.size());

    std::atomic<size_t> next_index{0};

    auto worker = [&](std::stop_token) {
        while (true) {
            size_t idx = next_index.fetch_add(1, std::memory_order_relaxed);
            if (idx >= assets.size()) break;

            report.asset_reports[idx] = run_single(std::move(assets[idx]));
        }
    };

    std::vector<std::jthread> threads;
    threads.reserve(effective_threads);
    for (uint32_t i = 0; i < effective_threads; ++i) {
        threads.emplace_back(worker);
    }

    // jthread destructors auto-join
    threads.clear();

    // Tally results
    for (const auto& ar : report.asset_reports) {
        if (ar.final_status == AssetStatus::Failed) {
            report.failed++;
        } else {
            report.passed++;
        }
    }

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
            asset = std::move(result.value());  // may be partial
            asset.status = AssetStatus::Failed;
            return asset;
        }

        asset = std::move(result.value());
        report.stages_completed.push_back(stage->name());
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
        auto status = (ar.final_status == AssetStatus::Failed) ? "FAIL" : "PASS";
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
