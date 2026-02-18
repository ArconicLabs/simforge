// About: YAML-backed material lookup table mapping material names to
// physics properties (density, friction, restitution) for the lookup
// mass estimation mode in PhysicsStage.
#pragma once

#include "simforge/core/types.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace YAML { class Node; }

namespace simforge {

class MaterialLibrary {
public:
    /// Load a material library from a YAML file.
    static MaterialLibrary from_file(const fs::path& path);

    /// Load a material library from a YAML string.
    static MaterialLibrary from_string(const std::string& yaml);

    /// Find a material by name (case-insensitive). Returns nullptr if not found.
    [[nodiscard]] const PhysicsMaterial* find(const std::string& name) const;

    /// Check if a material exists (case-insensitive).
    [[nodiscard]] bool has(const std::string& name) const;

    /// Return all material names (in original case).
    [[nodiscard]] std::vector<std::string> names() const;

    /// Number of materials in the library.
    [[nodiscard]] size_t size() const { return materials_.size(); }

private:
    /// Stored keyed by lowercase name for case-insensitive lookup.
    std::unordered_map<std::string, PhysicsMaterial> materials_;

    /// Original-case names in insertion order for names().
    std::vector<std::string> ordered_names_;

    static std::string to_lower(const std::string& s);
    void load_from_node(const YAML::Node& root);
};

}  // namespace simforge
