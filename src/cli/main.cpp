#include <iostream>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "simforge/core/types.h"
#include "simforge/pipeline/pipeline.h"
#include "simforge/pipeline/stage.h"
#include "simforge/adapters/adapter.h"

// Forward declaration — defined in builtin_adapters.cpp
namespace simforge::adapters {
    void register_builtin_adapters();
}

int main(int argc, char** argv) {
    CLI::App app{"simforge — asset pipeline for robotics simulation"};
    app.require_subcommand(1);

    // Global options
    std::string log_level = "info";
    app.add_option("--log-level", log_level, "Log level (trace/debug/info/warn/error)")
       ->default_val("info");

    // ─── process ───────────────────────────────────────────────────

    auto* process_cmd = app.add_subcommand("process", "Run the asset pipeline");

    std::string config_path = "simforge.yaml";
    std::string source_dir;
    std::string output_dir;
    bool dry_run = false;
    bool json_report = false;
    std::string report_path = "report.json";
    uint32_t threads = 0;  // 0 = use config value

    process_cmd->add_option("-c,--config", config_path, "Path to config file")
               ->default_val("simforge.yaml");
    process_cmd->add_option("-s,--source", source_dir, "Source directory (overrides config)");
    process_cmd->add_option("-o,--output", output_dir, "Output directory (overrides config)");
    process_cmd->add_option("-j,--threads", threads, "Number of threads (0=auto, 1=sequential)")
               ->default_val(0);
    bool force = false;
    process_cmd->add_flag("--force", force, "Reprocess all assets, ignore cached hashes");
    process_cmd->add_flag("--dry-run", dry_run, "Show what would be processed without running");
    process_cmd->add_flag("--json-report", json_report, "Write JSON report");
    process_cmd->add_option("--report-path", report_path, "Path for JSON report")
               ->default_val("report.json");

    // ─── inspect ───────────────────────────────────────────────────

    auto* inspect_cmd = app.add_subcommand("inspect", "Inspect a single asset file");

    std::string inspect_path;
    inspect_cmd->add_option("file", inspect_path, "Path to asset file")
               ->required();

    // ─── list-stages ───────────────────────────────────────────────

    auto* list_cmd = app.add_subcommand("list-stages", "List available pipeline stages");

    // ─── list-adapters ─────────────────────────────────────────────

    auto* adapters_cmd = app.add_subcommand("list-adapters", "List registered adapters");

    // ─── init ──────────────────────────────────────────────────────

    auto* init_cmd = app.add_subcommand("init", "Generate a default simforge.yaml config");

    std::string init_output = "simforge.yaml";
    init_cmd->add_option("-o,--output", init_output, "Output path")
            ->default_val("simforge.yaml");

    // ─── validate ──────────────────────────────────────────────────

    auto* validate_cmd = app.add_subcommand("validate", "Run validators on processed assets");

    std::string validate_dir;
    validate_cmd->add_option("dir", validate_dir, "Directory of processed assets")
                ->required();

    // ────────────────────────────────────────────────────────────────

    CLI11_PARSE(app, argc, argv);

    // Configure logging
    auto console = spdlog::stdout_color_mt("simforge");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::from_str(log_level));
    spdlog::set_pattern("[%H:%M:%S] [%^%l%$] %v");

    // Register built-in adapters
    simforge::adapters::register_builtin_adapters();

    // ─── Dispatch ──────────────────────────────────────────────────

    if (process_cmd->parsed()) {
        try {
            auto config = simforge::PipelineConfig::from_file(config_path);

            // CLI overrides
            if (!source_dir.empty()) config.source_dir = source_dir;
            if (!output_dir.empty()) config.output_dir = output_dir;
            if (threads > 0) config.threads = threads;
            if (force) config.force = true;

            simforge::Pipeline pipeline(std::move(config));
            pipeline.build();

            if (dry_run) {
                pipeline.dry_run();
                return 0;
            }

            auto report = pipeline.run();
            report.print_summary();

            if (json_report) {
                report.write_json(report_path);
            }

            return (report.failed > 0) ? 1 : 0;

        } catch (const std::exception& e) {
            spdlog::error("Pipeline error: {}", e.what());
            return 1;
        }
    }

    if (inspect_cmd->parsed()) {
        try {
            auto fmt = simforge::detect_format(inspect_path);
            spdlog::info("File:   {}", inspect_path);
            spdlog::info("Format: {}", simforge::format_to_string(fmt));

            auto& mgr = simforge::AdapterManager::instance();
            auto* importer = mgr.find_importer(fmt);

            if (!importer) {
                spdlog::error("No importer available for {}", simforge::format_to_string(fmt));
                return 1;
            }

            spdlog::info("Importer: {}", importer->name());
            auto meshes = importer->import(inspect_path);

            size_t total_v = 0, total_t = 0;
            for (auto& m : meshes) {
                m.recompute_bounds();
                spdlog::info("  Mesh '{}': {} verts, {} tris",
                             m.name, m.vertex_count(), m.triangle_count());
                spdlog::info("    Bounds: [{:.3f}, {:.3f}, {:.3f}] → [{:.3f}, {:.3f}, {:.3f}]",
                             m.bounds.min.x, m.bounds.min.y, m.bounds.min.z,
                             m.bounds.max.x, m.bounds.max.y, m.bounds.max.z);
                spdlog::info("    Watertight: {}", m.is_watertight() ? "yes" : "no");
                spdlog::info("    Volume: {:.6f} m³", m.compute_volume());
                total_v += m.vertex_count();
                total_t += m.triangle_count();
            }

            spdlog::info("Total: {} mesh(es), {} verts, {} tris",
                         meshes.size(), total_v, total_t);

        } catch (const std::exception& e) {
            spdlog::error("Inspect failed: {}", e.what());
            return 1;
        }
        return 0;
    }

    if (list_cmd->parsed()) {
        auto& reg = simforge::StageRegistry::instance();
        spdlog::info("Available pipeline stages:");
        for (const auto& name : reg.available()) {
            spdlog::info("  {}", name);
        }
        return 0;
    }

    if (adapters_cmd->parsed()) {
        auto& mgr = simforge::AdapterManager::instance();
        spdlog::info("Importers:");
        for (const auto& name : mgr.list_importers()) {
            spdlog::info("  {}", name);
        }
        spdlog::info("Exporters:");
        for (const auto& name : mgr.list_exporters()) {
            spdlog::info("  {}", name);
        }
        return 0;
    }

    if (init_cmd->parsed()) {
        std::ofstream out(init_output);
        out << R"(# simforge.yaml — asset pipeline configuration
# See https://github.com/aberrest/simforge for documentation

pipeline:
  source: ./raw_assets/
  output: ./sim_ready/

  # Export to one or more target formats.
  # Each asset is exported to every format listed here.
  # Supported: usd, usda, usdc, urdf, mjcf, gltf, glb, obj, stl, fbx
  target_formats: [usd]

  # Examples:
  #   target_formats: [usd, urdf, mjcf]     # Isaac Sim + Gazebo + MuJoCo
  #   target_formats: [usd, gltf]           # Isaac Sim + web viewer
  #   target_format: usd                    # single format (legacy syntax)

stages:
  ingest:
    formats: [obj, fbx, gltf, glb, stl, step, iges, urdf, mjcf, dae]

  collision:
    method: coacd               # coacd | convex_hull | triangle_mesh | primitive
    threshold: 0.05             # CoACD concavity threshold (lower = more precise)
    max_hulls: 32
    resolution: 2000

  physics:
    mass_estimation: geometry   # geometry | explicit | lookup
    density: 1000.0             # kg/m³ (water = 1000, plastic ≈ 950, steel ≈ 7800)
    friction: 0.5
    restitution: 0.3

  optimize:
    lod_levels: [high, medium, low]
    max_triangles: [50000, 10000, 2000]

  validate:
    watertight: true
    physics_plausibility: true
    collision_correctness: true
    mesh_integrity: true
    scale_sanity: true
    fail_on_warning: false

  export:
    # Inherits target_formats from pipeline level, or override per-stage:
    # formats: [usd, urdf]
    #
    # Advanced: per-format output subdirectories
    # formats:
    #   - format: usd
    #     subdir: isaac/
    #   - format: urdf
    #     subdir: gazebo/
    #   - format: mjcf
    #     subdir: mujoco/
    catalog: true
)";
        spdlog::info("Generated config at {}", init_output);
        return 0;
    }

    return 0;
}
