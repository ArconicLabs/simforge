#include "simforge/adapters/adapter.h"

#include <spdlog/spdlog.h>

namespace simforge {

// ─── MeshImporter ──────────────────────────────────────────────────

bool MeshImporter::can_import(SourceFormat fmt) const {
    auto fmts = supported_formats();
    return std::find(fmts.begin(), fmts.end(), fmt) != fmts.end();
}

// ─── AdapterManager ────────────────────────────────────────────────

AdapterManager& AdapterManager::instance() {
    static AdapterManager mgr;
    return mgr;
}

void AdapterManager::register_importer(MeshImporterPtr importer) {
    spdlog::debug("Registered importer: {}", importer->name());
    importers_.push_back(std::move(importer));
}

void AdapterManager::register_exporter(MeshExporterPtr exporter) {
    spdlog::debug("Registered exporter: {}", exporter->name());
    exporters_.push_back(std::move(exporter));
}

void AdapterManager::register_collision_generator(CollisionGeneratorPtr gen) {
    spdlog::debug("Registered collision generator: {}", gen->name());
    collision_generators_.push_back(std::move(gen));
}

void AdapterManager::register_lod_generator(LODGeneratorPtr gen) {
    spdlog::debug("Registered LOD generator: {}", gen->name());
    lod_generators_.push_back(std::move(gen));
}

MeshImporter* AdapterManager::find_importer(SourceFormat fmt) const {
    // Iterate in reverse so last-registered adapters take priority
    for (auto it = importers_.rbegin(); it != importers_.rend(); ++it) {
        if ((*it)->can_import(fmt)) return it->get();
    }
    return nullptr;
}

MeshExporter* AdapterManager::find_exporter(SourceFormat fmt) const {
    // Iterate in reverse so last-registered adapters take priority
    for (auto it = exporters_.rbegin(); it != exporters_.rend(); ++it) {
        auto fmts = (*it)->supported_formats();
        if (std::find(fmts.begin(), fmts.end(), fmt) != fmts.end())
            return it->get();
    }
    return nullptr;
}

CollisionGenerator* AdapterManager::find_collision_generator(const std::string& name) const {
    for (const auto& gen : collision_generators_) {
        if (gen->name() == name) return gen.get();
    }
    // Return first available if name doesn't match
    return collision_generators_.empty() ? nullptr : collision_generators_[0].get();
}

LODGenerator* AdapterManager::find_lod_generator(const std::string& name) const {
    for (const auto& gen : lod_generators_) {
        if (gen->name() == name) return gen.get();
    }
    return lod_generators_.empty() ? nullptr : lod_generators_[0].get();
}

std::vector<std::string> AdapterManager::list_importers() const {
    std::vector<std::string> names;
    for (const auto& imp : importers_) names.push_back(imp->name());
    return names;
}

std::vector<std::string> AdapterManager::list_exporters() const {
    std::vector<std::string> names;
    for (const auto& exp : exporters_) names.push_back(exp->name());
    return names;
}

void AdapterManager::reset() {
    importers_.clear();
    exporters_.clear();
    collision_generators_.clear();
    lod_generators_.clear();
}

}  // namespace simforge
