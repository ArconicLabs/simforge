#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

namespace simforge {

namespace fs = std::filesystem;

// ─── Geometry ──────────────────────────────────────────────────────

struct Vec3 {
    float x{0.0f}, y{0.0f}, z{0.0f};

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s)       const { return {x * s, y * s, z * s}; }
};

struct AABB {
    Vec3 min{};
    Vec3 max{};

    Vec3  center()  const { return (min + max) * 0.5f; }
    Vec3  extents() const { return max - min; }
    float volume()  const {
        auto e = extents();
        return e.x * e.y * e.z;
    }
};

struct Triangle {
    uint32_t v0, v1, v2;
};

// ─── Mesh ──────────────────────────────────────────────────────────

struct Mesh {
    std::string             name;
    std::vector<Vec3>       vertices;
    std::vector<Vec3>       normals;
    std::vector<Triangle>   faces;
    std::vector<Vec3>       uvs;        // flattened UV coords (2D stored as Vec3, z=0)
    AABB                    bounds{};

    [[nodiscard]] size_t triangle_count() const { return faces.size(); }
    [[nodiscard]] size_t vertex_count()   const { return vertices.size(); }
    [[nodiscard]] bool   empty()          const { return vertices.empty(); }

    void recompute_bounds();
    [[nodiscard]] float compute_volume() const;
    [[nodiscard]] bool  is_watertight()  const;
};

// ─── Collision ─────────────────────────────────────────────────────

enum class CollisionType {
    ConvexHull,
    ConvexDecomposition,
    TriangleMesh,
    Primitive,          // box, sphere, capsule
};

struct CollisionMesh {
    CollisionType           type{CollisionType::ConvexDecomposition};
    std::vector<Mesh>       hulls;
    float                   total_volume{0.0f};
    uint32_t                hull_count{0};
};

// ─── Physics ───────────────────────────────────────────────────────

struct PhysicsMaterial {
    std::string name{"default"};
    float       density{1000.0f};       // kg/m^3
    float       static_friction{0.5f};
    float       dynamic_friction{0.4f};
    float       restitution{0.3f};
};

struct PhysicsProperties {
    float                       mass{0.0f};         // kg
    Vec3                        center_of_mass{};
    Vec3                        inertia_diagonal{}; // principal moments
    PhysicsMaterial             material{};
    bool                        is_static{false};
    bool                        mass_estimated{false}; // true if inferred from geometry

    static PhysicsProperties estimate_from_mesh(Mesh& mesh, const PhysicsMaterial& mat);
};

// ─── Material / Appearance ─────────────────────────────────────────

struct PBRMaterial {
    std::string     name;
    Vec3            base_color{0.8f, 0.8f, 0.8f};
    float           metallic{0.0f};
    float           roughness{0.5f};
    float           opacity{1.0f};
    fs::path        albedo_map;
    fs::path        normal_map;
    fs::path        roughness_map;
};

// ─── LOD ───────────────────────────────────────────────────────────

enum class LODLevel : uint8_t {
    High   = 0,
    Medium = 1,
    Low    = 2,
};

struct LODMesh {
    LODLevel    level;
    Mesh        mesh;
    uint32_t    max_triangles;
};

// ─── Source Format ─────────────────────────────────────────────────

enum class SourceFormat {
    Unknown,
    OBJ,
    FBX,
    GLTF,
    GLB,
    STL,
    STEP,
    IGES,
    URDF,
    MJCF,
    USD,
    USDA,
    USDC,
    USDZ,
    DAE,        // Collada
};

[[nodiscard]] SourceFormat detect_format(const fs::path& path);
[[nodiscard]] SourceFormat parse_format(const std::string& name);
[[nodiscard]] std::string  format_to_string(SourceFormat fmt);

// ─── Asset ─────────────────────────────────────────────────────────

enum class AssetStatus {
    Raw,
    Ingested,
    CollisionGenerated,
    PhysicsAnnotated,
    Optimized,
    Validated,
    Ready,
    Failed,
};

struct ValidationResult {
    bool        passed{false};
    std::string check_name;
    std::string message;
    float       score{0.0f};       // 0.0 - 1.0 where applicable
};

struct Asset {
    // Identity
    std::string                     id;         // generated hash or user-provided
    std::string                     name;
    fs::path                        source_path;
    SourceFormat                    source_format{SourceFormat::Unknown};

    // Geometry
    std::vector<Mesh>               meshes;
    std::vector<LODMesh>            lods;

    // Collision
    std::optional<CollisionMesh>    collision;

    // Physics
    std::optional<PhysicsProperties> physics;

    // Appearance
    std::vector<PBRMaterial>        materials;

    // Metadata
    AssetStatus                     status{AssetStatus::Raw};
    std::vector<ValidationResult>   validations;
    nlohmann::json                  metadata;   // extensible key-value store

    // Output
    fs::path                        output_path;

    [[nodiscard]] bool all_validations_passed() const;
    [[nodiscard]] nlohmann::json to_catalog_entry() const;
};

// ─── JSON serialization ────────────────────────────────────────────

void to_json(nlohmann::json& j, const Vec3& v);
void to_json(nlohmann::json& j, const AABB& b);
void to_json(nlohmann::json& j, const PhysicsMaterial& m);
void to_json(nlohmann::json& j, const PhysicsProperties& p);
void to_json(nlohmann::json& j, const ValidationResult& v);

}  // namespace simforge
