// About: ArticulationStage implementation — loads and merges articulation
// data from source files, YAML config overlays, and sidecar metadata.
#include "simforge/pipeline/articulation_stage.h"

#include <fstream>
#include <unordered_set>

#include <spdlog/spdlog.h>

namespace simforge::stages {

void ArticulationStage::configure(const YAML::Node& config) {
    config_ = config;
    load_sidecars_ = config["sidecar"].as<bool>(true);
}

Result<Asset> ArticulationStage::process(Asset asset) {
    // Source 1: kinematic tree may already be populated by ArticulatedImporter
    bool had_source_tree = asset.is_articulated();

    // Source 2: sidecar metadata file
    if (load_sidecars_) {
        load_sidecar(asset);
    }

    // Source 3: YAML config overlay
    if (asset.is_articulated()) {
        apply_yaml_overlay(*asset.kinematic_tree);
    } else if (config_["links"] || config_["joints"] || config_["actuators"] || config_["sensors"]) {
        // No tree from source/sidecar, but YAML has articulation data
        auto tree = parse_tree_from_yaml(config_);
        asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));
    }

    // If there's still no articulation data, create a minimal single-link
    // tree from the asset's flat fields (only if YAML explicitly configured
    // links or if a sidecar was loaded)
    if (!asset.is_articulated()) {
        // No articulation data from any source — pass through
        asset.status = AssetStatus::Articulated;
        return Result<Asset>::ok(std::move(asset));
    }

    // Build index and validate structure
    asset.kinematic_tree->build_index();

    if (!asset.kinematic_tree->is_tree()) {
        return Result<Asset>::err({
            name(), asset.id,
            "Kinematic tree has cycles or disconnected links"
        }, std::move(asset));
    }

    size_t n_links  = asset.kinematic_tree->links.size();
    size_t n_joints = asset.kinematic_tree->joints.size();
    size_t total_dof = asset.kinematic_tree->dof();

    spdlog::info("  Articulation: {} links, {} joints, {} DOF",
                 n_links, n_joints, total_dof);

    if (!asset.kinematic_tree->actuators.empty()) {
        spdlog::info("  {} actuator(s)", asset.kinematic_tree->actuators.size());
    }
    if (!asset.kinematic_tree->sensors.empty()) {
        spdlog::info("  {} sensor(s)", asset.kinematic_tree->sensors.size());
    }

    asset.status = AssetStatus::Articulated;
    return Result<Asset>::ok(std::move(asset));
}

void ArticulationStage::load_sidecar(Asset& asset) const {
    auto stem = asset.source_path.stem().string();
    auto dir  = asset.source_path.parent_path();

    // Try .simforge.yaml then .simforge.json
    for (const auto& ext : {".simforge.yaml", ".simforge.json"}) {
        auto sidecar_path = dir / (stem + ext);
        if (!fs::exists(sidecar_path)) continue;

        spdlog::info("  Loading sidecar: {}", sidecar_path.string());

        try {
            auto sidecar = YAML::LoadFile(sidecar_path.string());
            auto tree = parse_tree_from_yaml(sidecar);

            if (asset.is_articulated()) {
                merge_trees(*asset.kinematic_tree, tree);
            } else {
                asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));
            }
        } catch (const std::exception& e) {
            spdlog::warn("  Failed to parse sidecar {}: {}", sidecar_path.string(), e.what());
        }

        return;  // only load the first found sidecar
    }
}

void ArticulationStage::apply_yaml_overlay(KinematicTree& tree) const {
    if (!config_["links"] && !config_["joints"] &&
        !config_["actuators"] && !config_["sensors"]) {
        return;
    }

    auto overlay = parse_tree_from_yaml(config_);
    merge_trees(tree, overlay);
}

KinematicTree ArticulationStage::parse_tree_from_yaml(const YAML::Node& node) {
    KinematicTree tree;

    if (auto links = node["links"]) {
        for (const auto& l : links) {
            tree.links.push_back(parse_link_from_yaml(l));
        }
    }

    if (auto joints = node["joints"]) {
        for (const auto& j : joints) {
            tree.joints.push_back(parse_joint_from_yaml(j));
        }
    }

    if (auto actuators = node["actuators"]) {
        for (const auto& a : actuators) {
            tree.actuators.push_back(parse_actuator_from_yaml(a));
        }
    }

    if (auto sensors = node["sensors"]) {
        for (const auto& s : sensors) {
            tree.sensors.push_back(parse_sensor_from_yaml(s));
        }
    }

    // Infer root_link: the first link that is not any joint's child
    if (!tree.links.empty() && tree.root_link.empty()) {
        std::unordered_set<std::string> children;
        for (const auto& j : tree.joints) children.insert(j.child_link);

        for (const auto& l : tree.links) {
            if (children.find(l.name) == children.end()) {
                tree.root_link = l.name;
                break;
            }
        }
        if (tree.root_link.empty()) {
            tree.root_link = tree.links[0].name;
        }
    }

    return tree;
}

Link ArticulationStage::parse_link_from_yaml(const YAML::Node& node) {
    Link link;
    link.name = node["name"].as<std::string>("");

    if (auto phys = node["physics"]) {
        PhysicsProperties props;
        props.mass = phys["mass"].as<float>(0.0f);
        if (auto com = phys["center_of_mass"]) {
            props.center_of_mass = {
                com[0].as<float>(0), com[1].as<float>(0), com[2].as<float>(0)
            };
        }
        link.physics = props;
    }

    return link;
}

Joint ArticulationStage::parse_joint_from_yaml(const YAML::Node& node) {
    Joint joint;
    joint.name        = node["name"].as<std::string>("");
    joint.type        = parse_joint_type(node["type"].as<std::string>("fixed"));
    joint.parent_link = node["parent"].as<std::string>("");
    joint.child_link  = node["child"].as<std::string>("");

    if (auto axis = node["axis"]) {
        joint.axis = {axis[0].as<float>(0), axis[1].as<float>(0), axis[2].as<float>(1)};
    }

    if (auto lim = node["limits"]) {
        JointLimits limits;
        limits.lower    = lim["lower"].as<float>(0);
        limits.upper    = lim["upper"].as<float>(0);
        limits.velocity = lim["velocity"].as<float>(0);
        limits.effort   = lim["effort"].as<float>(0);
        joint.limits = limits;
    }

    if (auto dyn = node["dynamics"]) {
        JointDynamics dynamics;
        dynamics.damping  = dyn["damping"].as<float>(0);
        dynamics.friction = dyn["friction"].as<float>(0);
        joint.dynamics = dynamics;
    }

    return joint;
}

Actuator ArticulationStage::parse_actuator_from_yaml(const YAML::Node& node) {
    Actuator act;
    act.name         = node["name"].as<std::string>("");
    act.joint        = node["joint"].as<std::string>("");
    act.control_mode = parse_control_mode(node["control_mode"].as<std::string>("position"));
    act.gear_ratio   = node["gear_ratio"].as<float>(1.0f);
    act.max_torque   = node["max_torque"].as<float>(0.0f);
    act.max_velocity = node["max_velocity"].as<float>(0.0f);

    if (auto props = node["properties"]) {
        // Convert YAML map to JSON
        for (auto it = props.begin(); it != props.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (it->second.IsScalar()) {
                // Try numeric first, fall back to string
                try {
                    act.properties[key] = it->second.as<float>();
                } catch (...) {
                    act.properties[key] = it->second.as<std::string>();
                }
            }
        }
    }

    return act;
}

Sensor ArticulationStage::parse_sensor_from_yaml(const YAML::Node& node) {
    Sensor sensor;
    sensor.name = node["name"].as<std::string>("");
    sensor.type = node["type"].as<std::string>("");
    sensor.link = node["link"].as<std::string>("");

    if (auto props = node["properties"]) {
        for (auto it = props.begin(); it != props.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (it->second.IsScalar()) {
                try {
                    sensor.properties[key] = it->second.as<float>();
                } catch (...) {
                    sensor.properties[key] = it->second.as<std::string>();
                }
            } else if (it->second.IsMap()) {
                // Nested map → JSON object
                nlohmann::json nested;
                for (auto sub = it->second.begin(); sub != it->second.end(); ++sub) {
                    auto subkey = sub->first.as<std::string>();
                    try {
                        nested[subkey] = sub->second.as<float>();
                    } catch (...) {
                        nested[subkey] = sub->second.as<std::string>();
                    }
                }
                sensor.properties[key] = nested;
            }
        }
    }

    return sensor;
}

void ArticulationStage::merge_trees(KinematicTree& base, const KinematicTree& overlay) {
    // Merge links by name: overlay fields overwrite base fields
    for (const auto& ol : overlay.links) {
        bool found = false;
        for (auto& bl : base.links) {
            if (bl.name == ol.name) {
                if (ol.physics) bl.physics = ol.physics;
                if (!ol.visual_meshes.empty()) bl.visual_meshes = ol.visual_meshes;
                if (ol.collision) bl.collision = ol.collision;
                if (!ol.materials.empty()) bl.materials = ol.materials;
                found = true;
                break;
            }
        }
        if (!found) base.links.push_back(ol);
    }

    // Merge joints by name
    for (const auto& oj : overlay.joints) {
        bool found = false;
        for (auto& bj : base.joints) {
            if (bj.name == oj.name) {
                bj = oj;  // overlay wins entirely
                found = true;
                break;
            }
        }
        if (!found) base.joints.push_back(oj);
    }

    // Merge actuators by name
    for (const auto& oa : overlay.actuators) {
        bool found = false;
        for (auto& ba : base.actuators) {
            if (ba.name == oa.name) {
                ba = oa;
                found = true;
                break;
            }
        }
        if (!found) base.actuators.push_back(oa);
    }

    // Merge sensors by name
    for (const auto& os : overlay.sensors) {
        bool found = false;
        for (auto& bs : base.sensors) {
            if (bs.name == os.name) {
                bs = os;
                found = true;
                break;
            }
        }
        if (!found) base.sensors.push_back(os);
    }

    // Update root_link if overlay specifies one
    if (!overlay.root_link.empty()) {
        base.root_link = overlay.root_link;
    }
}

// ─── Auto-register ─────────────────────────────────────────────────

SIMFORGE_REGISTER_STAGE(ArticulationStage, "articulation")

}  // namespace simforge::stages
