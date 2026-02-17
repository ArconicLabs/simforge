// About: Validators for articulated assets — kinematic tree structure,
// actuator plausibility, sensor configuration, and joint limits.
#pragma once

#include "simforge/validators/validator.h"

namespace simforge {

/// Validates kinematic tree structure: acyclic, all references resolve,
/// exactly one root, no orphan links, no duplicate names.
class KinematicTreeValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "kinematic_tree"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

/// Validates actuator configuration: references valid joints, limits
/// are positive, gear ratio is sane.
class ActuatorValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "actuator_plausibility"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

/// Validates sensor configuration: references valid links, type is
/// non-empty, known types have required properties.
class SensorValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "sensor_plausibility"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

/// Validates joint limits: revolute/prismatic have limits, lower < upper,
/// continuous joints have no position limits, fixed joints have no limits.
class JointLimitsValidator : public Validator {
public:
    [[nodiscard]] std::string name() const override { return "joint_limits"; }
    std::vector<ValidationResult> validate(const Asset& asset) override;
};

}  // namespace simforge
