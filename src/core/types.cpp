#include "simforge/core/types.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>

namespace simforge {

// ─── Mesh ──────────────────────────────────────────────────────────

void Mesh::recompute_bounds() {
    if (vertices.empty()) return;

    bounds.min = bounds.max = vertices[0];
    for (const auto& v : vertices) {
        bounds.min.x = std::min(bounds.min.x, v.x);
        bounds.min.y = std::min(bounds.min.y, v.y);
        bounds.min.z = std::min(bounds.min.z, v.z);
        bounds.max.x = std::max(bounds.max.x, v.x);
        bounds.max.y = std::max(bounds.max.y, v.y);
        bounds.max.z = std::max(bounds.max.z, v.z);
    }
}

float Mesh::compute_volume() const {
    // Signed volume via divergence theorem (sum of signed tetrahedra).
    // Works for closed meshes; approximate for open ones.
    float vol = 0.0f;
    for (const auto& tri : faces) {
        if (tri.v0 >= vertices.size() || tri.v1 >= vertices.size() || tri.v2 >= vertices.size())
            continue;
        const auto& a = vertices[tri.v0];
        const auto& b = vertices[tri.v1];
        const auto& c = vertices[tri.v2];
        vol += (a.x * (b.y * c.z - c.y * b.z)
              - b.x * (a.y * c.z - c.y * a.z)
              + c.x * (a.y * b.z - b.y * a.z));
    }
    return std::abs(vol) / 6.0f;
}

bool Mesh::is_watertight() const {
    // A mesh is watertight if every edge is shared by exactly 2 triangles.
    // Edge represented as ordered pair of vertex indices.
    struct EdgeHash {
        size_t operator()(const std::pair<uint32_t, uint32_t>& e) const {
            return std::hash<uint64_t>{}(
                (static_cast<uint64_t>(e.first) << 32) | e.second);
        }
    };

    std::unordered_map<std::pair<uint32_t, uint32_t>, int, EdgeHash> edge_count;

    auto add_edge = [&](uint32_t a, uint32_t b) {
        auto key = std::make_pair(std::min(a, b), std::max(a, b));
        edge_count[key]++;
    };

    for (const auto& tri : faces) {
        add_edge(tri.v0, tri.v1);
        add_edge(tri.v1, tri.v2);
        add_edge(tri.v2, tri.v0);
    }

    return std::all_of(edge_count.begin(), edge_count.end(),
        [](const auto& pair) { return pair.second == 2; });
}

// ─── PhysicsProperties ─────────────────────────────────────────────

PhysicsProperties PhysicsProperties::estimate_from_mesh(
    Mesh& mesh, const PhysicsMaterial& mat) {

    PhysicsProperties props;
    props.material = mat;
    props.mass_estimated = true;

    // Ensure bounds are up-to-date before using them for inertia
    mesh.recompute_bounds();

    float volume = mesh.compute_volume();
    props.mass = volume * mat.density;

    // Center of mass: average of vertex positions (crude but serviceable)
    Vec3 com{};
    for (const auto& v : mesh.vertices) {
        com.x += v.x;
        com.y += v.y;
        com.z += v.z;
    }
    float n = static_cast<float>(mesh.vertices.size());
    if (n > 0) {
        com.x /= n;
        com.y /= n;
        com.z /= n;
    }
    props.center_of_mass = com;

    // Inertia: approximate as solid box from AABB
    auto ext = mesh.bounds.extents();
    float m = props.mass;
    props.inertia_diagonal = {
        (m / 12.0f) * (ext.y * ext.y + ext.z * ext.z),
        (m / 12.0f) * (ext.x * ext.x + ext.z * ext.z),
        (m / 12.0f) * (ext.x * ext.x + ext.y * ext.y),
    };

    return props;
}

// ─── Asset ─────────────────────────────────────────────────────────

Asset::Asset(const Asset& other)
    : id(other.id), name(other.name), source_path(other.source_path),
      source_format(other.source_format), meshes(other.meshes), lods(other.lods),
      collision(other.collision), physics(other.physics), materials(other.materials),
      status(other.status), validations(other.validations), metadata(other.metadata),
      kinematic_tree(other.kinematic_tree
          ? std::make_unique<KinematicTree>(*other.kinematic_tree) : nullptr),
      output_path(other.output_path) {}

Asset& Asset::operator=(const Asset& other) {
    if (this != &other) {
        id             = other.id;
        name           = other.name;
        source_path    = other.source_path;
        source_format  = other.source_format;
        meshes         = other.meshes;
        lods           = other.lods;
        collision      = other.collision;
        physics        = other.physics;
        materials      = other.materials;
        status         = other.status;
        validations    = other.validations;
        metadata       = other.metadata;
        kinematic_tree = other.kinematic_tree
            ? std::make_unique<KinematicTree>(*other.kinematic_tree) : nullptr;
        output_path    = other.output_path;
    }
    return *this;
}

bool Asset::all_validations_passed() const {
    return std::all_of(validations.begin(), validations.end(),
        [](const auto& v) { return v.passed; });
}

nlohmann::json Asset::to_catalog_entry() const {
    nlohmann::json entry;
    entry["id"]             = id;
    entry["name"]           = name;
    entry["source_path"]    = source_path.string();
    entry["source_format"]  = format_to_string(source_format);
    entry["output_path"]    = output_path.string();
    entry["status"]         = static_cast<int>(status);

    entry["mesh_count"]     = meshes.size();
    size_t total_tris = 0;
    for (const auto& m : meshes) total_tris += m.triangle_count();
    entry["total_triangles"] = total_tris;

    if (collision) {
        entry["collision"]["type"]       = static_cast<int>(collision->type);
        entry["collision"]["hull_count"] = collision->hull_count();
        entry["collision"]["volume"]     = collision->total_volume;
    }

    if (physics) {
        to_json(entry["physics"], *physics);
    }

    entry["validations"] = nlohmann::json::array();
    for (const auto& v : validations) {
        nlohmann::json vj;
        to_json(vj, v);
        entry["validations"].push_back(vj);
    }

    // Merge any extra metadata
    if (!metadata.empty()) {
        entry["metadata"] = metadata;
    }

    return entry;
}

// ─── Format Detection ──────────────────────────────────────────────

SourceFormat detect_format(const fs::path& path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    static const std::unordered_map<std::string, SourceFormat> ext_map = {
        {".obj",  SourceFormat::OBJ},
        {".fbx",  SourceFormat::FBX},
        {".gltf", SourceFormat::GLTF},
        {".glb",  SourceFormat::GLB},
        {".stl",  SourceFormat::STL},
        {".step", SourceFormat::STEP},
        {".stp",  SourceFormat::STEP},
        {".iges", SourceFormat::IGES},
        {".igs",  SourceFormat::IGES},
        {".urdf", SourceFormat::URDF},
        {".mjcf", SourceFormat::MJCF},
        {".xml",  SourceFormat::MJCF},      // ambiguous, but common for MJCF
        {".usd",  SourceFormat::USD},
        {".usda", SourceFormat::USDA},
        {".usdc", SourceFormat::USDC},
        {".usdz", SourceFormat::USDZ},
        {".dae",  SourceFormat::DAE},
    };

    auto it = ext_map.find(ext);
    return (it != ext_map.end()) ? it->second : SourceFormat::Unknown;
}

SourceFormat parse_format(const std::string& name) {
    std::string upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    if (upper == "OBJ")  return SourceFormat::OBJ;
    if (upper == "FBX")  return SourceFormat::FBX;
    if (upper == "GLTF") return SourceFormat::GLTF;
    if (upper == "GLB")  return SourceFormat::GLB;
    if (upper == "STL")  return SourceFormat::STL;
    if (upper == "STEP") return SourceFormat::STEP;
    if (upper == "IGES") return SourceFormat::IGES;
    if (upper == "URDF") return SourceFormat::URDF;
    if (upper == "MJCF") return SourceFormat::MJCF;
    if (upper == "USD")  return SourceFormat::USD;
    if (upper == "USDA") return SourceFormat::USDA;
    if (upper == "USDC") return SourceFormat::USDC;
    if (upper == "DAE")  return SourceFormat::DAE;
    return SourceFormat::Unknown;
}

std::string format_to_string(SourceFormat fmt) {
    switch (fmt) {
        case SourceFormat::OBJ:     return "OBJ";
        case SourceFormat::FBX:     return "FBX";
        case SourceFormat::GLTF:    return "GLTF";
        case SourceFormat::GLB:     return "GLB";
        case SourceFormat::STL:     return "STL";
        case SourceFormat::STEP:    return "STEP";
        case SourceFormat::IGES:    return "IGES";
        case SourceFormat::URDF:    return "URDF";
        case SourceFormat::MJCF:    return "MJCF";
        case SourceFormat::USD:     return "USD";
        case SourceFormat::USDA:    return "USDA";
        case SourceFormat::USDC:    return "USDC";
        case SourceFormat::USDZ:    return "USDZ";
        case SourceFormat::DAE:     return "DAE";
        default:                    return "Unknown";
    }
}

// ─── JSON serialization ────────────────────────────────────────────

void to_json(nlohmann::json& j, const Vec3& v) {
    j = {{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

void to_json(nlohmann::json& j, const AABB& b) {
    j = {{"min", nullptr}, {"max", nullptr}};
    to_json(j["min"], b.min);
    to_json(j["max"], b.max);
}

void to_json(nlohmann::json& j, const PhysicsMaterial& m) {
    j = {
        {"name", m.name},
        {"density", m.density},
        {"static_friction", m.static_friction},
        {"dynamic_friction", m.dynamic_friction},
        {"restitution", m.restitution},
    };
}

void to_json(nlohmann::json& j, const PhysicsProperties& p) {
    j = {
        {"mass", p.mass},
        {"is_static", p.is_static},
        {"mass_estimated", p.mass_estimated},
    };
    to_json(j["center_of_mass"], p.center_of_mass);
    to_json(j["inertia_diagonal"], p.inertia_diagonal);
    to_json(j["material"], p.material);
}

void to_json(nlohmann::json& j, const ValidationResult& v) {
    j = {
        {"passed", v.passed},
        {"check", v.check_name},
        {"message", v.message},
        {"score", v.score},
    };
}

}  // namespace simforge
