// About: MaterialLibrary implementation — parses YAML material tables and
// provides case-insensitive lookup of PhysicsMaterial properties.

#include "simforge/core/material_library.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace simforge {

std::string MaterialLibrary::to_lower(const std::string& s) {
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

void MaterialLibrary::load_from_node(const YAML::Node& root) {
    auto materials_node = root["materials"];
    if (!materials_node || !materials_node.IsMap()) {
        throw std::runtime_error("Material library YAML must have a top-level 'materials' map");
    }

    for (auto it = materials_node.begin(); it != materials_node.end(); ++it) {
        auto name = it->first.as<std::string>();
        auto node = it->second;

        PhysicsMaterial mat;
        mat.name             = name;
        mat.density          = node["density"].as<float>();
        mat.static_friction  = node["static_friction"].as<float>(0.5f);
        mat.dynamic_friction = node["dynamic_friction"].as<float>(mat.static_friction * 0.8f);
        mat.restitution      = node["restitution"].as<float>(0.3f);

        materials_[to_lower(name)] = std::move(mat);
        ordered_names_.push_back(name);
    }
}

MaterialLibrary MaterialLibrary::from_file(const fs::path& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open material library: " + path.string());
    }
    MaterialLibrary lib;
    lib.load_from_node(YAML::Load(in));
    return lib;
}

MaterialLibrary MaterialLibrary::from_string(const std::string& yaml) {
    MaterialLibrary lib;
    lib.load_from_node(YAML::Load(yaml));
    return lib;
}

const PhysicsMaterial* MaterialLibrary::find(const std::string& name) const {
    auto it = materials_.find(to_lower(name));
    return it != materials_.end() ? &it->second : nullptr;
}

bool MaterialLibrary::has(const std::string& name) const {
    return materials_.find(to_lower(name)) != materials_.end();
}

std::vector<std::string> MaterialLibrary::names() const {
    return ordered_names_;
}

}  // namespace simforge
