// About: Core data types for SimForge — geometry, physics, materials,
// articulation (links, joints, actuators, sensors), and the Asset struct
// that flows through the pipeline.
#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
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

    [[nodiscard]] uint32_t hull_count() const {
        return static_cast<uint32_t>(hulls.size());
    }
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

// ─── Articulation ──────────────────────────────────────────────────

struct Quaternion {
    float w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f};
};

struct Pose {
    Vec3       position{};
    Quaternion orientation{};
};

/// A rigid body within an articulated asset. Structurally parallel
/// to the single-body Asset fields but scoped to one link in the tree.
struct Link {
    std::string                      name;
    std::vector<Mesh>                visual_meshes;
    std::optional<CollisionMesh>     collision;
    std::optional<PhysicsProperties> physics;
    std::vector<PBRMaterial>         materials;
    Pose                             origin{};
    nlohmann::json                   metadata;
};

enum class JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar,
    Spherical,
};

struct JointLimits {
    float lower{0.0f};
    float upper{0.0f};
    float velocity{0.0f};
    float effort{0.0f};
};

struct JointDynamics {
    float damping{0.0f};
    float friction{0.0f};
};

struct Joint {
    std::string                  name;
    JointType                    type{JointType::Fixed};
    std::string                  parent_link;
    std::string                  child_link;
    Pose                         origin{};
    Vec3                         axis{0.0f, 0.0f, 1.0f};
    std::optional<JointLimits>   limits;
    std::optional<JointDynamics> dynamics;
    nlohmann::json               metadata;
};

enum class ControlMode {
    Position,
    Velocity,
    Effort,
};

/// Core actuator model: joint-level basics. Extensible via the
/// properties bag for transmission, backlash, friction models.
struct Actuator {
    std::string    name;
    std::string    joint;
    ControlMode    control_mode{ControlMode::Position};
    float          gear_ratio{1.0f};
    float          max_torque{0.0f};
    float          max_velocity{0.0f};
    nlohmann::json properties;
};

/// Generic sensor abstraction. The type field is a free-form string
/// (e.g., "imu", "camera", "force_torque", "lidar", "contact").
/// All sensor-specific configuration lives in the properties bag.
struct Sensor {
    std::string    name;
    std::string    type;
    std::string    link;
    Pose           origin{};
    nlohmann::json properties;
};

/// Complete articulation model. Optional on Asset; when absent, the
/// asset is a single rigid body using Asset's flat fields.
struct KinematicTree {
    std::string           root_link;
    std::vector<Link>     links;
    std::vector<Joint>    joints;
    std::vector<Actuator> actuators;
    std::vector<Sensor>   sensors;

    [[nodiscard]] const Link*     find_link(const std::string& name) const;
    [[nodiscard]] const Joint*    find_joint(const std::string& name) const;
    [[nodiscard]] const Actuator* find_actuator_for_joint(const std::string& joint_name) const;
    [[nodiscard]] std::vector<const Joint*> child_joints(const std::string& link_name) const;

    /// Total degrees of freedom (non-fixed joints).
    [[nodiscard]] size_t dof() const;

    /// True if the graph forms a tree (no cycles, single root).
    [[nodiscard]] bool is_tree() const;

    /// Build internal index maps for O(1) lookup by name.
    void build_index();

    /// Mutable access for per-link stage processing.
    [[nodiscard]] Link* find_link_mut(const std::string& name);

private:
    std::unordered_map<std::string, size_t> link_index_;
    std::unordered_map<std::string, size_t> joint_index_;
};

[[nodiscard]] std::string joint_type_to_string(JointType type);
[[nodiscard]] JointType   parse_joint_type(const std::string& str);
[[nodiscard]] std::string control_mode_to_string(ControlMode mode);
[[nodiscard]] ControlMode parse_control_mode(const std::string& str);

// ─── Asset ─────────────────────────────────────────────────────────

enum class AssetStatus {
    Raw,
    Ingested,
    Articulated,
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
    Asset() = default;
    Asset(Asset&&) = default;
    Asset& operator=(Asset&&) = default;
    Asset(const Asset& other);
    Asset& operator=(const Asset& other);

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

    // Articulation (null for single-body assets)
    std::unique_ptr<KinematicTree>  kinematic_tree;

    // Output
    fs::path                        output_path;
    std::string                     content_hash;  // SHA-256 for incremental processing

    [[nodiscard]] bool is_articulated() const { return kinematic_tree != nullptr; }
    [[nodiscard]] bool all_validations_passed() const;
    [[nodiscard]] nlohmann::json to_catalog_entry() const;
};

// ─── JSON serialization ────────────────────────────────────────────

void to_json(nlohmann::json& j, const Vec3& v);
void to_json(nlohmann::json& j, const AABB& b);
void to_json(nlohmann::json& j, const PhysicsMaterial& m);
void to_json(nlohmann::json& j, const PhysicsProperties& p);
void to_json(nlohmann::json& j, const ValidationResult& v);
void to_json(nlohmann::json& j, const Quaternion& q);
void to_json(nlohmann::json& j, const Pose& p);
void to_json(nlohmann::json& j, const Link& l);
void to_json(nlohmann::json& j, const Joint& jt);
void to_json(nlohmann::json& j, const Actuator& a);
void to_json(nlohmann::json& j, const Sensor& s);
void to_json(nlohmann::json& j, const KinematicTree& kt);

}  // namespace simforge
