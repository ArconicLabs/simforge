// About: Implementation of articulated body types — KinematicTree methods,
// string conversion, and JSON serialization.
#include "simforge/core/types.h"

#include <queue>
#include <unordered_set>

namespace simforge {

// ─── KinematicTree ─────────────────────────────────────────────────

void KinematicTree::build_index() {
    link_index_.clear();
    joint_index_.clear();
    for (size_t i = 0; i < links.size(); ++i)
        link_index_[links[i].name] = i;
    for (size_t i = 0; i < joints.size(); ++i)
        joint_index_[joints[i].name] = i;
}

const Link* KinematicTree::find_link(const std::string& name) const {
    auto it = link_index_.find(name);
    if (it != link_index_.end()) return &links[it->second];
    // Fallback linear scan if index not built
    for (const auto& l : links)
        if (l.name == name) return &l;
    return nullptr;
}

Link* KinematicTree::find_link_mut(const std::string& name) {
    auto it = link_index_.find(name);
    if (it != link_index_.end()) return &links[it->second];
    for (auto& l : links)
        if (l.name == name) return &l;
    return nullptr;
}

const Joint* KinematicTree::find_joint(const std::string& name) const {
    auto it = joint_index_.find(name);
    if (it != joint_index_.end()) return &joints[it->second];
    for (const auto& j : joints)
        if (j.name == name) return &j;
    return nullptr;
}

const Actuator* KinematicTree::find_actuator_for_joint(const std::string& joint_name) const {
    for (const auto& a : actuators)
        if (a.joint == joint_name) return &a;
    return nullptr;
}

std::vector<const Joint*> KinematicTree::child_joints(const std::string& link_name) const {
    std::vector<const Joint*> result;
    for (const auto& j : joints)
        if (j.parent_link == link_name) result.push_back(&j);
    return result;
}

size_t KinematicTree::dof() const {
    size_t total = 0;
    for (const auto& j : joints) {
        switch (j.type) {
            case JointType::Fixed:      break;
            case JointType::Revolute:   total += 1; break;
            case JointType::Continuous: total += 1; break;
            case JointType::Prismatic:  total += 1; break;
            case JointType::Planar:     total += 2; break;
            case JointType::Spherical:  total += 3; break;
            case JointType::Floating:   total += 6; break;
        }
    }
    return total;
}

bool KinematicTree::is_tree() const {
    if (links.empty()) return true;

    // Build adjacency: parent -> children via joints
    std::unordered_map<std::string, std::vector<std::string>> children;
    std::unordered_set<std::string> child_set;

    for (const auto& j : joints) {
        children[j.parent_link].push_back(j.child_link);
        // A child appearing twice means it has multiple parents (not a tree)
        if (!child_set.insert(j.child_link).second) return false;
    }

    // Check single root: exactly one link that is not any joint's child
    std::unordered_set<std::string> link_names;
    for (const auto& l : links) link_names.insert(l.name);

    size_t root_count = 0;
    for (const auto& l : links) {
        if (child_set.find(l.name) == child_set.end()) root_count++;
    }
    if (root_count != 1) return false;

    // BFS to check all links are reachable from root and no cycles
    std::unordered_set<std::string> visited;
    std::queue<std::string> bfs;
    bfs.push(root_link);
    visited.insert(root_link);

    while (!bfs.empty()) {
        auto current = bfs.front();
        bfs.pop();
        if (children.count(current)) {
            for (const auto& child : children[current]) {
                if (!visited.insert(child).second) return false;
                bfs.push(child);
            }
        }
    }

    return visited.size() == links.size();
}

// ─── String conversion ─────────────────────────────────────────────

std::string joint_type_to_string(JointType type) {
    switch (type) {
        case JointType::Fixed:      return "fixed";
        case JointType::Revolute:   return "revolute";
        case JointType::Continuous: return "continuous";
        case JointType::Prismatic:  return "prismatic";
        case JointType::Floating:   return "floating";
        case JointType::Planar:     return "planar";
        case JointType::Spherical:  return "spherical";
    }
    return "fixed";
}

JointType parse_joint_type(const std::string& str) {
    if (str == "fixed")      return JointType::Fixed;
    if (str == "revolute")   return JointType::Revolute;
    if (str == "continuous") return JointType::Continuous;
    if (str == "prismatic")  return JointType::Prismatic;
    if (str == "floating")   return JointType::Floating;
    if (str == "planar")     return JointType::Planar;
    if (str == "spherical")  return JointType::Spherical;
    return JointType::Fixed;
}

std::string control_mode_to_string(ControlMode mode) {
    switch (mode) {
        case ControlMode::Position: return "position";
        case ControlMode::Velocity: return "velocity";
        case ControlMode::Effort:   return "effort";
    }
    return "position";
}

ControlMode parse_control_mode(const std::string& str) {
    if (str == "position") return ControlMode::Position;
    if (str == "velocity") return ControlMode::Velocity;
    if (str == "effort")   return ControlMode::Effort;
    return ControlMode::Position;
}

// ─── JSON serialization ────────────────────────────────────────────

void to_json(nlohmann::json& j, const Quaternion& q) {
    j = {{"w", q.w}, {"x", q.x}, {"y", q.y}, {"z", q.z}};
}

void to_json(nlohmann::json& j, const Pose& p) {
    j = nlohmann::json::object();
    to_json(j["position"], p.position);
    to_json(j["orientation"], p.orientation);
}

void to_json(nlohmann::json& j, const Link& l) {
    j = {
        {"name", l.name},
        {"visual_mesh_count", l.visual_meshes.size()},
        {"has_collision", l.collision.has_value()},
        {"has_physics", l.physics.has_value()},
        {"material_count", l.materials.size()},
    };
    to_json(j["origin"], l.origin);
    if (l.physics) {
        to_json(j["physics"], *l.physics);
    }
}

void to_json(nlohmann::json& j, const Joint& jt) {
    j = {
        {"name", jt.name},
        {"type", joint_type_to_string(jt.type)},
        {"parent_link", jt.parent_link},
        {"child_link", jt.child_link},
    };
    to_json(j["origin"], jt.origin);
    to_json(j["axis"], jt.axis);

    if (jt.limits) {
        j["limits"] = {
            {"lower", jt.limits->lower},
            {"upper", jt.limits->upper},
            {"velocity", jt.limits->velocity},
            {"effort", jt.limits->effort},
        };
    }
    if (jt.dynamics) {
        j["dynamics"] = {
            {"damping", jt.dynamics->damping},
            {"friction", jt.dynamics->friction},
        };
    }
}

void to_json(nlohmann::json& j, const Actuator& a) {
    j = {
        {"name", a.name},
        {"joint", a.joint},
        {"control_mode", control_mode_to_string(a.control_mode)},
        {"gear_ratio", a.gear_ratio},
        {"max_torque", a.max_torque},
        {"max_velocity", a.max_velocity},
    };
    if (!a.properties.empty()) {
        j["properties"] = a.properties;
    }
}

void to_json(nlohmann::json& j, const Sensor& s) {
    j = {
        {"name", s.name},
        {"type", s.type},
        {"link", s.link},
    };
    to_json(j["origin"], s.origin);
    if (!s.properties.empty()) {
        j["properties"] = s.properties;
    }
}

void to_json(nlohmann::json& j, const KinematicTree& kt) {
    j = {
        {"root_link", kt.root_link},
        {"dof", kt.dof()},
    };

    j["links"] = nlohmann::json::array();
    for (const auto& l : kt.links) {
        nlohmann::json lj;
        to_json(lj, l);
        j["links"].push_back(lj);
    }

    j["joints"] = nlohmann::json::array();
    for (const auto& jt : kt.joints) {
        nlohmann::json jj;
        to_json(jj, jt);
        j["joints"].push_back(jj);
    }

    j["actuators"] = nlohmann::json::array();
    for (const auto& a : kt.actuators) {
        nlohmann::json aj;
        to_json(aj, a);
        j["actuators"].push_back(aj);
    }

    j["sensors"] = nlohmann::json::array();
    for (const auto& s : kt.sensors) {
        nlohmann::json sj;
        to_json(sj, s);
        j["sensors"].push_back(sj);
    }
}

}  // namespace simforge
