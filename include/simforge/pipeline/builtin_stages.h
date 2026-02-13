#pragma once

#include "simforge/pipeline/stage.h"
#include "simforge/adapters/adapter.h"
#include "simforge/validators/validator.h"

namespace simforge::stages {

// ─── Ingest Stage ──────────────────────────────────────────────────
// Reads source files via the adapter layer and populates Asset.meshes.

class IngestStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "ingest"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

    [[nodiscard]] bool should_run(const Asset& asset) const override {
        return asset.status == AssetStatus::Raw;
    }

private:
    std::vector<SourceFormat> accepted_formats_;
};

// ─── Collision Stage ───────────────────────────────────────────────
// Generates collision meshes from visual geometry.

class CollisionStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "collision"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

    [[nodiscard]] bool should_run(const Asset& asset) const override {
        return asset.status == AssetStatus::Ingested && !asset.meshes.empty();
    }

private:
    CollisionParams params_;
    std::string     generator_name_{"builtin"};
};

// ─── Physics Stage ─────────────────────────────────────────────────
// Annotates assets with physics properties (mass, friction, etc.).

class PhysicsStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "physics"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

    [[nodiscard]] bool should_run(const Asset& asset) const override {
        return asset.status == AssetStatus::CollisionGenerated;
    }

private:
    PhysicsMaterial default_material_;
    std::string     mass_mode_{"geometry"};  // "geometry", "explicit", "lookup"
};

// ─── Optimize Stage ────────────────────────────────────────────────
// Generates LODs and optimizes meshes for simulation performance.

class OptimizeStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "optimize"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

    [[nodiscard]] bool should_run(const Asset& asset) const override {
        return asset.status == AssetStatus::PhysicsAnnotated;
    }

private:
    struct LODConfig {
        LODLevel    level;
        uint32_t    max_triangles;
    };
    std::vector<LODConfig>  lod_configs_;
    std::string             generator_name_{"builtin"};
};

// ─── Validate Stage ────────────────────────────────────────────────
// Runs all configured validators against the asset.

class ValidateStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "validate"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

private:
    std::vector<ValidatorPtr>   validators_;
    bool                        fail_on_warning_{false};
};

// ─── Export Stage ──────────────────────────────────────────────────
// Writes the processed asset to the target format.

class ExportStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "export"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

private:
    struct ExportTarget {
        SourceFormat    format;
        fs::path        subdir;     // optional per-format subdirectory
    };

    std::vector<ExportTarget>   targets_;
    fs::path                    output_dir_;
    bool                        write_catalog_{true};
    bool                        unified_catalog_{true}; // single catalog.json for all formats

    Result<Asset> export_single(const Asset& asset, const ExportTarget& target);
};

/// Explicitly register all built-in stages. Safe to call multiple times.
/// Needed when linking simforge_core as a static library since the
/// SIMFORGE_REGISTER_STAGE auto-registration may be stripped by the linker.
void register_builtin_stages();

}  // namespace simforge::stages
