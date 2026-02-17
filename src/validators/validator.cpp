#include "simforge/validators/validator.h"
#include "simforge/validators/articulation_validators.h"

#include <cmath>
#include <spdlog/spdlog.h>

namespace simforge {

// ─── ValidatorRegistry ─────────────────────────────────────────────

ValidatorRegistry::ValidatorRegistry() {
    // Register built-in validators
    register_validator("watertight", [] { return std::make_unique<WatertightValidator>(); });
    register_validator("physics_plausibility", [] { return std::make_unique<PhysicsPlausibilityValidator>(); });
    register_validator("collision_correctness", [] { return std::make_unique<CollisionCorrectnessValidator>(); });
    register_validator("mesh_integrity", [] { return std::make_unique<MeshIntegrityValidator>(); });
    register_validator("scale_sanity", [] { return std::make_unique<ScaleSanityValidator>(); });
    register_validator("kinematic_tree", [] { return std::make_unique<KinematicTreeValidator>(); });
    register_validator("actuator_plausibility", [] { return std::make_unique<ActuatorValidator>(); });
    register_validator("sensor_plausibility", [] { return std::make_unique<SensorValidator>(); });
    register_validator("joint_limits", [] { return std::make_unique<JointLimitsValidator>(); });
}

ValidatorRegistry& ValidatorRegistry::instance() {
    static ValidatorRegistry reg;
    return reg;
}

void ValidatorRegistry::register_validator(const std::string& name, ValidatorFactory factory) {
    factories_[name] = std::move(factory);
}

ValidatorPtr ValidatorRegistry::create(const std::string& name) const {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
        throw std::runtime_error("Unknown validator: " + name);
    }
    return it->second();
}

std::vector<std::string> ValidatorRegistry::available() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : factories_) names.push_back(name);
    return names;
}

// ─── WatertightValidator ───────────────────────────────────────────

std::vector<ValidationResult> WatertightValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    for (const auto& mesh : asset.meshes) {
        ValidationResult r;
        r.check_name = "watertight:" + mesh.name;
        r.passed = mesh.is_watertight();
        r.score = r.passed ? 1.0f : 0.0f;
        r.message = r.passed
            ? "Mesh is watertight"
            : "Mesh has boundary edges (not manifold)";
        results.push_back(r);
    }

    return results;
}

// ─── PhysicsPlausibilityValidator ──────────────────────────────────

void PhysicsPlausibilityValidator::configure(const YAML::Node& config) {
    min_density_ = config["min_density"].as<float>(min_density_);
    max_density_ = config["max_density"].as<float>(max_density_);
}

std::vector<ValidationResult> PhysicsPlausibilityValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.physics) {
        results.push_back({false, "physics_plausibility", "No physics properties set", 0.0f});
        return results;
    }

    const auto& p = *asset.physics;

    // Mass check
    {
        ValidationResult r;
        r.check_name = "mass_positive";
        r.passed = p.mass > 0.0f || p.is_static;
        r.message = r.passed
            ? "Mass is positive (" + std::to_string(p.mass) + " kg)"
            : "Mass is zero or negative for non-static object";
        results.push_back(r);
    }

    // Density check (if we can compute volume)
    if (!asset.meshes.empty() && p.mass > 0.0f) {
        float volume = asset.meshes[0].compute_volume();
        if (volume > 0.0f) {
            float density = p.mass / volume;
            ValidationResult r;
            r.check_name = "density_plausible";
            r.passed = density >= min_density_ && density <= max_density_;
            r.score = r.passed ? 1.0f : 0.0f;
            r.message = "Implied density: " + std::to_string(density) + " kg/m³"
                + (r.passed ? " (plausible)" : " (suspicious)");
            results.push_back(r);
        }
    }

    return results;
}

// ─── CollisionCorrectnessValidator ─────────────────────────────────

void CollisionCorrectnessValidator::configure(const YAML::Node& config) {
    min_volume_ratio_ = config["min_volume_ratio"].as<float>(min_volume_ratio_);
    max_volume_ratio_ = config["max_volume_ratio"].as<float>(max_volume_ratio_);
    max_hull_count_   = config["max_hull_count"].as<uint32_t>(max_hull_count_);
}

std::vector<ValidationResult> CollisionCorrectnessValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.collision) {
        results.push_back({
            false, "collision_exists",
            "No collision mesh generated", 0.0f
        });
        return results;
    }

    const auto& coll = *asset.collision;

    // Hull count
    {
        ValidationResult r;
        r.check_name = "hull_count";
        r.passed = coll.hull_count() <= max_hull_count_;
        r.score = r.passed ? 1.0f : 0.0f;
        r.message = std::to_string(coll.hull_count()) + " hull(s)"
            + (r.passed ? "" : " (exceeds max " + std::to_string(max_hull_count_) + ")");
        results.push_back(r);
    }

    // Volume ratio (collision volume vs visual mesh volume)
    if (!asset.meshes.empty() && coll.total_volume > 0.0f) {
        float visual_volume = 0.0f;
        for (const auto& m : asset.meshes) {
            visual_volume += m.compute_volume();
        }

        if (visual_volume > 0.0f) {
            float ratio = coll.total_volume / visual_volume;
            ValidationResult r;
            r.check_name = "volume_ratio";
            r.passed = ratio >= min_volume_ratio_ && ratio <= max_volume_ratio_;
            r.score = r.passed ? 1.0f : std::clamp(1.0f - std::abs(ratio - 1.0f), 0.0f, 1.0f);
            r.message = "Collision/visual volume ratio: " + std::to_string(ratio)
                + (r.passed ? " (acceptable)" : " (out of range)");
            results.push_back(r);
        }
    }

    return results;
}

// ─── MeshIntegrityValidator ────────────────────────────────────────

std::vector<ValidationResult> MeshIntegrityValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    for (const auto& mesh : asset.meshes) {
        // Check for degenerate triangles
        uint32_t degenerate = 0;
        for (const auto& tri : mesh.faces) {
            if (tri.v0 == tri.v1 || tri.v1 == tri.v2 || tri.v0 == tri.v2) {
                degenerate++;
            }
            // Also check for out-of-bounds indices
            if (tri.v0 >= mesh.vertices.size() ||
                tri.v1 >= mesh.vertices.size() ||
                tri.v2 >= mesh.vertices.size()) {
                degenerate++;
            }
        }

        ValidationResult r;
        r.check_name = "mesh_integrity:" + mesh.name;
        r.passed = degenerate == 0;
        r.score = mesh.faces.empty() ? 0.0f
            : 1.0f - static_cast<float>(degenerate) / static_cast<float>(mesh.faces.size());
        r.message = r.passed
            ? "No degenerate triangles"
            : std::to_string(degenerate) + " degenerate triangle(s) found";
        results.push_back(r);

        // Check for empty meshes
        if (mesh.vertices.empty()) {
            results.push_back({false, "non_empty:" + mesh.name, "Mesh has no vertices", 0.0f});
        }
    }

    return results;
}

// ─── ScaleSanityValidator ──────────────────────────────────────────

void ScaleSanityValidator::configure(const YAML::Node& config) {
    expected_unit_ = config["unit"].as<float>(expected_unit_);
    min_extent_    = config["min_extent"].as<float>(min_extent_);
    max_extent_    = config["max_extent"].as<float>(max_extent_);
}

std::vector<ValidationResult> ScaleSanityValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    for (const auto& mesh : asset.meshes) {
        auto ext = mesh.bounds.extents();
        float max_dim = std::max({ext.x, ext.y, ext.z}) * expected_unit_;

        ValidationResult r;
        r.check_name = "scale_sanity:" + mesh.name;
        r.passed = max_dim >= min_extent_ && max_dim <= max_extent_;
        r.score = r.passed ? 1.0f : 0.0f;
        r.message = "Max extent: " + std::to_string(max_dim) + "m"
            + (r.passed ? " (plausible)"
               : max_dim < min_extent_ ? " (too small — wrong units?)"
               : " (too large — wrong units?)");
        results.push_back(r);
    }

    return results;
}

}  // namespace simforge
