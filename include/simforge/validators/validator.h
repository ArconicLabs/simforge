#pragma once

#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "simforge/core/types.h"

namespace simforge {

// ─── Validator Interface ───────────────────────────────────────────

class Validator {
public:
    virtual ~Validator() = default;

    [[nodiscard]] virtual std::string name() const = 0;

    virtual void configure(const YAML::Node& config) { (void)config; }

    /// Run validation check on an asset. Returns one or more results
    /// (a validator can check multiple properties).
    virtual std::vector<ValidationResult> validate(const Asset& asset) = 0;
};

using ValidatorPtr = std::unique_ptr<Validator>;

// ─── Built-in Validators ───────────────────────────────────────────

/// Checks that the mesh is watertight (manifold, no holes).
class WatertightValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "watertight"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

/// Checks mass vs. volume plausibility.
/// A 1cm^3 object shouldn't weigh 100kg, etc.
class PhysicsPlausibilityValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "physics_plausibility"; }
    void configure(const YAML::Node& config) override;
    std::vector<ValidationResult> validate(const Asset& asset) override;

private:
    float min_density_{10.0f};     // kg/m^3 (lighter than aerogel = suspicious)
    float max_density_{25000.0f};  // kg/m^3 (denser than osmium = suspicious)
};

/// Checks that collision hulls are a reasonable approximation
/// of the visual mesh (volume ratio, hull count, etc.).
class CollisionCorrectnessValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "collision_correctness"; }
    void configure(const YAML::Node& config) override;
    std::vector<ValidationResult> validate(const Asset& asset) override;

private:
    float min_volume_ratio_{0.5f};  // collision vol / visual vol
    float max_volume_ratio_{1.5f};
    uint32_t max_hull_count_{64};
};

/// Checks mesh integrity: degenerate triangles, duplicate vertices,
/// inverted normals, etc.
class MeshIntegrityValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "mesh_integrity"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

/// Checks that the asset bounding box is within expected real-world
/// dimensions (not accidentally in millimeters when expecting meters, etc.).
class ScaleSanityValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "scale_sanity"; }
    void configure(const YAML::Node& config) override;
    std::vector<ValidationResult> validate(const Asset& asset) override;

private:
    float expected_unit_{1.0f};     // 1.0 = meters
    float min_extent_{0.001f};      // 1mm minimum
    float max_extent_{100.0f};      // 100m maximum
};

// ─── Validator Registry ────────────────────────────────────────────

using ValidatorFactory = std::function<ValidatorPtr()>;

class ValidatorRegistry {
public:
    static ValidatorRegistry& instance();

    void         register_validator(const std::string& name, ValidatorFactory factory);
    ValidatorPtr create(const std::string& name) const;
    [[nodiscard]] std::vector<std::string> available() const;

private:
    ValidatorRegistry();  // registers built-ins in constructor
    std::unordered_map<std::string, ValidatorFactory> factories_;
};

}  // namespace simforge
