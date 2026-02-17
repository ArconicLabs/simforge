// About: Implementation of articulation validators — kinematic tree
// structure, actuator plausibility, sensor config, and joint limits.
#include "simforge/validators/articulation_validators.h"

#include <unordered_set>

namespace simforge {

// ─── KinematicTreeValidator ────────────────────────────────────────

std::vector<ValidationResult> KinematicTreeValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.is_articulated()) {
        results.push_back({true, "kinematic_tree", "No kinematic tree (single-body asset)", 1.0f});
        return results;
    }

    const auto& tree = *asset.kinematic_tree;

    // Check for duplicate link names
    std::unordered_set<std::string> link_names;
    for (const auto& l : tree.links) {
        if (!link_names.insert(l.name).second) {
            results.push_back({false, "kinematic_tree",
                "Duplicate link name: " + l.name, 0.0f});
            return results;
        }
    }

    // Check for duplicate joint names
    std::unordered_set<std::string> joint_names;
    for (const auto& j : tree.joints) {
        if (!joint_names.insert(j.name).second) {
            results.push_back({false, "kinematic_tree",
                "Duplicate joint name: " + j.name, 0.0f});
            return results;
        }
    }

    // Check all joint references resolve to existing links
    for (const auto& j : tree.joints) {
        if (link_names.find(j.parent_link) == link_names.end()) {
            results.push_back({false, "kinematic_tree",
                "Joint '" + j.name + "' references unknown parent link '" + j.parent_link + "'",
                0.0f});
            return results;
        }
        if (link_names.find(j.child_link) == link_names.end()) {
            results.push_back({false, "kinematic_tree",
                "Joint '" + j.name + "' references unknown child link '" + j.child_link + "'",
                0.0f});
            return results;
        }
    }

    // Check tree structure
    if (!tree.is_tree()) {
        results.push_back({false, "kinematic_tree",
            "Graph has cycles or multiple roots", 0.0f});
        return results;
    }

    // Check root_link exists
    if (link_names.find(tree.root_link) == link_names.end()) {
        results.push_back({false, "kinematic_tree",
            "Root link '" + tree.root_link + "' not found", 0.0f});
        return results;
    }

    results.push_back({true, "kinematic_tree",
        std::to_string(tree.links.size()) + " links, " +
        std::to_string(tree.joints.size()) + " joints, " +
        std::to_string(tree.dof()) + " DOF",
        1.0f});

    return results;
}

// ─── ActuatorValidator ─────────────────────────────────────────────

std::vector<ValidationResult> ActuatorValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.is_articulated() || asset.kinematic_tree->actuators.empty()) {
        results.push_back({true, "actuator_plausibility", "No actuators to validate", 1.0f});
        return results;
    }

    const auto& tree = *asset.kinematic_tree;
    bool all_ok = true;

    for (const auto& act : tree.actuators) {
        // Check joint reference
        const auto* joint = tree.find_joint(act.joint);
        if (!joint) {
            results.push_back({false, "actuator_plausibility",
                "Actuator '" + act.name + "' references unknown joint '" + act.joint + "'",
                0.0f});
            all_ok = false;
            continue;
        }

        // Fixed joints should not be actuated
        if (joint->type == JointType::Fixed) {
            results.push_back({false, "actuator_plausibility",
                "Actuator '" + act.name + "' on fixed joint '" + act.joint + "'",
                0.0f});
            all_ok = false;
            continue;
        }

        // Limits should be positive
        if (act.max_torque < 0.0f) {
            results.push_back({false, "actuator_plausibility",
                "Actuator '" + act.name + "' has negative max_torque", 0.0f});
            all_ok = false;
        }
        if (act.max_velocity < 0.0f) {
            results.push_back({false, "actuator_plausibility",
                "Actuator '" + act.name + "' has negative max_velocity", 0.0f});
            all_ok = false;
        }

        // Gear ratio should be positive
        if (act.gear_ratio <= 0.0f || act.gear_ratio > 10000.0f) {
            results.push_back({false, "actuator_plausibility",
                "Actuator '" + act.name + "' has implausible gear ratio: " +
                std::to_string(act.gear_ratio),
                0.0f});
            all_ok = false;
        }
    }

    if (all_ok) {
        results.push_back({true, "actuator_plausibility",
            std::to_string(tree.actuators.size()) + " actuator(s) valid", 1.0f});
    }

    return results;
}

// ─── SensorValidator ───────────────────────────────────────────────

std::vector<ValidationResult> SensorValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.is_articulated() || asset.kinematic_tree->sensors.empty()) {
        results.push_back({true, "sensor_plausibility", "No sensors to validate", 1.0f});
        return results;
    }

    const auto& tree = *asset.kinematic_tree;
    bool all_ok = true;

    for (const auto& sensor : tree.sensors) {
        // Check link reference
        if (!tree.find_link(sensor.link)) {
            results.push_back({false, "sensor_plausibility",
                "Sensor '" + sensor.name + "' references unknown link '" + sensor.link + "'",
                0.0f});
            all_ok = false;
            continue;
        }

        // Type must be non-empty
        if (sensor.type.empty()) {
            results.push_back({false, "sensor_plausibility",
                "Sensor '" + sensor.name + "' has empty type", 0.0f});
            all_ok = false;
            continue;
        }

        // Known sensor types: check required properties
        if (sensor.type == "camera") {
            if (!sensor.properties.contains("width") || !sensor.properties.contains("height")) {
                results.push_back({false, "sensor_plausibility",
                    "Camera sensor '" + sensor.name + "' missing width/height", 0.0f});
                all_ok = false;
            }
        }
    }

    if (all_ok) {
        results.push_back({true, "sensor_plausibility",
            std::to_string(tree.sensors.size()) + " sensor(s) valid", 1.0f});
    }

    return results;
}

// ─── JointLimitsValidator ──────────────────────────────────────────

std::vector<ValidationResult> JointLimitsValidator::validate(const Asset& asset) {
    std::vector<ValidationResult> results;

    if (!asset.is_articulated() || asset.kinematic_tree->joints.empty()) {
        results.push_back({true, "joint_limits", "No joints to validate", 1.0f});
        return results;
    }

    const auto& tree = *asset.kinematic_tree;
    bool all_ok = true;

    for (const auto& joint : tree.joints) {
        if (joint.type == JointType::Fixed) {
            // Fixed joints should not have limits
            if (joint.limits) {
                results.push_back({false, "joint_limits",
                    "Fixed joint '" + joint.name + "' should not have limits", 0.0f});
                all_ok = false;
            }
            continue;
        }

        if (joint.type == JointType::Continuous) {
            // Continuous joints should not have position limits
            if (joint.limits && (joint.limits->lower != 0.0f || joint.limits->upper != 0.0f)) {
                results.push_back({false, "joint_limits",
                    "Continuous joint '" + joint.name + "' should not have position limits",
                    0.0f});
                all_ok = false;
            }
            continue;
        }

        if (joint.type == JointType::Revolute || joint.type == JointType::Prismatic) {
            if (!joint.limits) {
                results.push_back({false, "joint_limits",
                    "Joint '" + joint.name + "' (" + joint_type_to_string(joint.type) +
                    ") missing limits", 0.0f});
                all_ok = false;
                continue;
            }

            if (joint.limits->lower >= joint.limits->upper) {
                results.push_back({false, "joint_limits",
                    "Joint '" + joint.name + "' has lower >= upper limit", 0.0f});
                all_ok = false;
            }

            if (joint.limits->velocity < 0.0f) {
                results.push_back({false, "joint_limits",
                    "Joint '" + joint.name + "' has negative velocity limit", 0.0f});
                all_ok = false;
            }

            if (joint.limits->effort < 0.0f) {
                results.push_back({false, "joint_limits",
                    "Joint '" + joint.name + "' has negative effort limit", 0.0f});
                all_ok = false;
            }
        }
    }

    if (all_ok) {
        results.push_back({true, "joint_limits",
            std::to_string(tree.joints.size()) + " joint(s) limits valid", 1.0f});
    }

    return results;
}

}  // namespace simforge
