#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "simforge/core/types.h"

namespace simforge {

namespace fs = std::filesystem;

// ─── Mesh Importer ─────────────────────────────────────────────────

/// Interface for importing meshes from various formats.
/// Each concrete importer handles one or more SourceFormats.
class MeshImporter {
public:
    virtual ~MeshImporter() = default;

    [[nodiscard]] virtual std::string name() const = 0;

    /// Formats this importer can handle.
    [[nodiscard]] virtual std::vector<SourceFormat> supported_formats() const = 0;

    /// Import all meshes from a file.
    virtual std::vector<Mesh> import(const fs::path& path) = 0;

    /// Check if this importer can handle a specific file.
    [[nodiscard]] bool can_import(SourceFormat fmt) const;
};

using MeshImporterPtr = std::unique_ptr<MeshImporter>;

// ─── Mesh Exporter ─────────────────────────────────────────────────

/// Interface for exporting assets to target formats.
class MeshExporter {
public:
    virtual ~MeshExporter() = default;

    [[nodiscard]] virtual std::string name() const = 0;
    [[nodiscard]] virtual std::vector<SourceFormat> supported_formats() const = 0;

    /// Export a fully processed asset to the target format.
    virtual bool export_asset(const Asset& asset, const fs::path& output_path) = 0;
};

using MeshExporterPtr = std::unique_ptr<MeshExporter>;

// ─── Collision Generator ───────────────────────────────────────────

struct CollisionParams {
    CollisionType   method{CollisionType::ConvexDecomposition};
    float           threshold{0.05f};       // CoACD concavity threshold
    uint32_t        max_hulls{32};
    uint32_t        resolution{2000};       // voxelization resolution
    bool            preprocess{true};
};

/// Interface for generating collision meshes from visual geometry.
class CollisionGenerator {
public:
    virtual ~CollisionGenerator() = default;

    [[nodiscard]] virtual std::string name() const = 0;

    virtual CollisionMesh generate(const Mesh& visual_mesh,
                                   const CollisionParams& params) = 0;
};

using CollisionGeneratorPtr = std::unique_ptr<CollisionGenerator>;

// ─── LOD Generator ─────────────────────────────────────────────────

struct LODParams {
    uint32_t    target_triangles;
    float       quality{0.7f};  // 0-1, higher = better quality / slower
};

/// Interface for mesh decimation / LOD generation.
class LODGenerator {
public:
    virtual ~LODGenerator() = default;

    [[nodiscard]] virtual std::string name() const = 0;

    virtual Mesh decimate(const Mesh& source, const LODParams& params) = 0;
};

using LODGeneratorPtr = std::unique_ptr<LODGenerator>;

// ─── Adapter Manager ───────────────────────────────────────────────

/// Central registry for all adapters. The pipeline queries this
/// to find the right tool for each operation.
class AdapterManager {
public:
    static AdapterManager& instance();

    // Registration
    void register_importer(MeshImporterPtr importer);
    void register_exporter(MeshExporterPtr exporter);
    void register_collision_generator(CollisionGeneratorPtr gen);
    void register_lod_generator(LODGeneratorPtr gen);

    // Lookup
    [[nodiscard]] MeshImporter*         find_importer(SourceFormat fmt) const;
    [[nodiscard]] MeshExporter*         find_exporter(SourceFormat fmt) const;
    [[nodiscard]] CollisionGenerator*   find_collision_generator(const std::string& name) const;
    [[nodiscard]] LODGenerator*         find_lod_generator(const std::string& name) const;

    // Introspection
    [[nodiscard]] std::vector<std::string> list_importers() const;
    [[nodiscard]] std::vector<std::string> list_exporters() const;

    /// Clear all registered adapters. Primarily for test isolation.
    void reset();

private:
    AdapterManager() = default;

    std::vector<MeshImporterPtr>        importers_;
    std::vector<MeshExporterPtr>        exporters_;
    std::vector<CollisionGeneratorPtr>  collision_generators_;
    std::vector<LODGeneratorPtr>        lod_generators_;
};

}  // namespace simforge
