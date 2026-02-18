// About: MJCF importer — parses MuJoCo XML files into KinematicTree with
// links (bodies), joints, actuators, and sensors.
#include "simforge/adapters/adapter.h"

#include <spdlog/spdlog.h>
#include <tinyxml2.h>

namespace simforge::adapters {

namespace {

/// Safe float parse — returns fallback on malformed input
float safe_stof(const char* str, float fallback = 0.0f) {
    try { return std::stof(str); }
    catch (...) {
        spdlog::warn("Invalid float value: '{}'", str);
        return fallback;
    }
}

Vec3 parse_vec3(const char* str) {
    Vec3 v{};
    if (str) std::sscanf(str, "%f %f %f", &v.x, &v.y, &v.z);
    return v;
}

Quaternion parse_quat(const char* str) {
    Quaternion q;
    if (str) std::sscanf(str, "%f %f %f %f", &q.w, &q.x, &q.y, &q.z);
    return q;
}

Quaternion axisangle_to_quat(const char* str) {
    float ax = 0, ay = 0, az = 1, angle = 0;
    if (str) std::sscanf(str, "%f %f %f %f", &ax, &ay, &az, &angle);

    float half = angle * 0.5f;
    float s = std::sin(half);
    float len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len > 0) { ax /= len; ay /= len; az /= len; }

    Quaternion q;
    q.w = std::cos(half);
    q.x = ax * s;
    q.y = ay * s;
    q.z = az * s;
    return q;
}

Pose parse_body_pose(const tinyxml2::XMLElement* elem) {
    Pose pose;
    auto* pos = elem->Attribute("pos");
    if (pos) pose.position = parse_vec3(pos);

    auto* quat_str = elem->Attribute("quat");
    if (quat_str) {
        pose.orientation = parse_quat(quat_str);
    } else {
        auto* axisangle = elem->Attribute("axisangle");
        if (axisangle) pose.orientation = axisangle_to_quat(axisangle);
    }
    return pose;
}

JointType mujoco_joint_type(const char* type_str) {
    if (!type_str) return JointType::Revolute;  // MuJoCo default is hinge
    std::string t = type_str;
    if (t == "hinge")  return JointType::Revolute;
    if (t == "slide")  return JointType::Prismatic;
    if (t == "ball")   return JointType::Spherical;
    if (t == "free")   return JointType::Floating;
    return JointType::Revolute;
}

class MJCFImporter : public ArticulatedImporter {
public:
    [[nodiscard]] std::string name() const override { return "mjcf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::MJCF};
    }

    ArticulatedImportResult import_articulated(const fs::path& path) override {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
            throw std::runtime_error("Failed to parse MJCF: " + path.string());
        }

        auto* mujoco = doc.FirstChildElement("mujoco");
        if (!mujoco) {
            throw std::runtime_error("MJCF missing <mujoco> element");
        }

        ArticulatedImportResult result;
        base_dir_ = path.parent_path();

        // Collect mesh assets for geom references
        parse_mesh_assets(mujoco);

        // Walk the body hierarchy in <worldbody>
        auto* worldbody = mujoco->FirstChildElement("worldbody");
        if (worldbody) {
            for (auto* body = worldbody->FirstChildElement("body"); body;
                 body = body->NextSiblingElement("body")) {
                walk_body(body, "", result);
            }
        }

        // Parse actuators from top-level <actuator> section
        auto* actuator_section = mujoco->FirstChildElement("actuator");
        if (actuator_section) {
            parse_actuators(actuator_section, result.tree);
        }

        // Parse sensors from top-level <sensor> section
        auto* sensor_section = mujoco->FirstChildElement("sensor");
        if (sensor_section) {
            parse_sensors(sensor_section, result.tree);
        }

        // Root link is the first body under worldbody
        if (!result.tree.links.empty()) {
            result.tree.root_link = result.tree.links[0].name;
        }

        result.tree.build_index();

        spdlog::info("MJCF imported: {} links, {} joints, {} actuators, {} sensors",
                     result.tree.links.size(), result.tree.joints.size(),
                     result.tree.actuators.size(), result.tree.sensors.size());

        return result;
    }

private:
    fs::path base_dir_;
    // Maps mesh name → file path from <asset><mesh> declarations
    std::unordered_map<std::string, std::string> mesh_assets_;

    void parse_mesh_assets(const tinyxml2::XMLElement* mujoco) {
        mesh_assets_.clear();
        auto* asset = mujoco->FirstChildElement("asset");
        if (!asset) return;

        for (auto* mesh = asset->FirstChildElement("mesh"); mesh;
             mesh = mesh->NextSiblingElement("mesh")) {
            auto* name = mesh->Attribute("name");
            auto* file = mesh->Attribute("file");
            if (name && file) {
                mesh_assets_[name] = file;
            } else if (file && !name) {
                // MuJoCo uses filename stem as implicit name
                fs::path fp(file);
                mesh_assets_[fp.stem().string()] = file;
            }
        }
    }

    void walk_body(const tinyxml2::XMLElement* body_elem,
                   const std::string& parent_name,
                   ArticulatedImportResult& result) {
        Link link;
        link.name = body_elem->Attribute("name")
                        ? body_elem->Attribute("name")
                        : "body_" + std::to_string(result.tree.links.size());
        link.origin = parse_body_pose(body_elem);

        // Parse geoms for visual and collision meshes
        for (auto* geom = body_elem->FirstChildElement("geom"); geom;
             geom = geom->NextSiblingElement("geom")) {
            parse_geom(geom, link, result.all_meshes);
        }

        // Parse inertial
        auto* inertial = body_elem->FirstChildElement("inertial");
        if (inertial) {
            PhysicsProperties props;
            auto* mass_str = inertial->Attribute("mass");
            if (mass_str) props.mass = safe_stof(mass_str);

            auto* pos = inertial->Attribute("pos");
            if (pos) props.center_of_mass = parse_vec3(pos);

            auto* diag = inertial->Attribute("diaginertia");
            if (diag) props.inertia_diagonal = parse_vec3(diag);

            link.physics = props;
        }

        // Parse joints within this body
        for (auto* joint_elem = body_elem->FirstChildElement("joint"); joint_elem;
             joint_elem = joint_elem->NextSiblingElement("joint")) {
            Joint joint;
            joint.name = joint_elem->Attribute("name")
                             ? joint_elem->Attribute("name")
                             : "joint_" + std::to_string(result.tree.joints.size());

            joint.type = mujoco_joint_type(joint_elem->Attribute("type"));
            joint.parent_link = parent_name;
            joint.child_link = link.name;

            auto* axis_str = joint_elem->Attribute("axis");
            if (axis_str) joint.axis = parse_vec3(axis_str);

            auto* pos = joint_elem->Attribute("pos");
            if (pos) joint.origin.position = parse_vec3(pos);

            // Joint limits — MuJoCo uses "range" attribute
            auto* range = joint_elem->Attribute("range");
            if (range) {
                JointLimits lim;
                std::sscanf(range, "%f %f", &lim.lower, &lim.upper);
                joint.limits = lim;
            }

            // Damping and friction from joint attributes
            JointDynamics dyn;
            bool has_dyn = false;
            auto* damping = joint_elem->Attribute("damping");
            if (damping) { dyn.damping = safe_stof(damping); has_dyn = true; }
            auto* frictionloss = joint_elem->Attribute("frictionloss");
            if (frictionloss) { dyn.friction = safe_stof(frictionloss); has_dyn = true; }
            if (has_dyn) joint.dynamics = dyn;

            result.tree.joints.push_back(std::move(joint));
        }

        // If no joints but has a parent, create an implicit fixed joint
        if (body_elem->FirstChildElement("joint") == nullptr && !parent_name.empty()) {
            Joint fixed;
            fixed.name = parent_name + "_to_" + link.name + "_fixed";
            fixed.type = JointType::Fixed;
            fixed.parent_link = parent_name;
            fixed.child_link = link.name;
            result.tree.joints.push_back(std::move(fixed));
        }

        auto this_name = link.name;
        result.tree.links.push_back(std::move(link));

        // Recurse into child bodies
        for (auto* child = body_elem->FirstChildElement("body"); child;
             child = child->NextSiblingElement("body")) {
            walk_body(child, this_name, result);
        }
    }

    void parse_geom(const tinyxml2::XMLElement* geom,
                    Link& link,
                    std::vector<Mesh>& all_meshes) {
        auto* type = geom->Attribute("type");
        if (!type || std::string(type) != "mesh") return;

        auto* mesh_name = geom->Attribute("mesh");
        if (!mesh_name) return;

        auto it = mesh_assets_.find(mesh_name);
        if (it == mesh_assets_.end()) return;

        auto mesh_path = resolve_path(it->second);
        auto* importer = AdapterManager::instance().find_importer(
            detect_format(mesh_path));
        if (!importer) return;

        try {
            auto meshes = importer->import(mesh_path);
            // Classify by contype/conaffinity
            int contype = 1, conaffinity = 1;
            geom->QueryIntAttribute("contype", &contype);
            geom->QueryIntAttribute("conaffinity", &conaffinity);
            bool is_visual_only = (contype == 0 && conaffinity == 0);

            for (auto& m : meshes) {
                m.recompute_bounds();
                all_meshes.push_back(m);
                if (is_visual_only) {
                    link.visual_meshes.push_back(std::move(m));
                } else {
                    link.visual_meshes.push_back(m);
                    CollisionMesh cm;
                    cm.type = CollisionType::TriangleMesh;
                    cm.hulls.push_back(std::move(m));
                    link.collision = std::move(cm);
                }
            }
        } catch (const std::exception& e) {
            spdlog::warn("Failed to load mesh '{}': {}", mesh_name, e.what());
        }
    }

    void parse_actuators(const tinyxml2::XMLElement* section,
                         KinematicTree& tree) {
        // MuJoCo actuator types map to control modes
        struct ActuatorDef {
            const char* tag;
            ControlMode mode;
        };
        ActuatorDef defs[] = {
            {"motor",    ControlMode::Effort},
            {"position", ControlMode::Position},
            {"velocity", ControlMode::Velocity},
            {"general",  ControlMode::Effort},
        };

        for (const auto& def : defs) {
            for (auto* elem = section->FirstChildElement(def.tag); elem;
                 elem = elem->NextSiblingElement(def.tag)) {
                Actuator act;
                act.name = elem->Attribute("name")
                               ? elem->Attribute("name")
                               : "actuator_" + std::to_string(tree.actuators.size());

                auto* joint = elem->Attribute("joint");
                if (joint) act.joint = joint;

                act.control_mode = def.mode;

                auto* gear = elem->Attribute("gear");
                if (gear) act.gear_ratio = safe_stof(gear);

                // Parse force/ctrl range into max_torque
                auto* forcerange = elem->Attribute("forcerange");
                if (forcerange) {
                    float lo = 0, hi = 0;
                    std::sscanf(forcerange, "%f %f", &lo, &hi);
                    act.max_torque = std::max(std::abs(lo), std::abs(hi));
                }

                auto* ctrlrange = elem->Attribute("ctrlrange");
                if (ctrlrange) {
                    float lo = 0, hi = 0;
                    std::sscanf(ctrlrange, "%f %f", &lo, &hi);
                    act.properties["ctrl_range_lower"] = lo;
                    act.properties["ctrl_range_upper"] = hi;
                }

                tree.actuators.push_back(std::move(act));
            }
        }
    }

    void parse_sensors(const tinyxml2::XMLElement* section,
                       KinematicTree& tree) {
        // Walk all child elements — each is a sensor type in MuJoCo
        for (auto* elem = section->FirstChildElement(); elem;
             elem = elem->NextSiblingElement()) {
            Sensor sensor;
            sensor.name = elem->Attribute("name")
                              ? elem->Attribute("name")
                              : "sensor_" + std::to_string(tree.sensors.size());
            sensor.type = elem->Name();  // element name is the sensor type

            // Site-based sensors reference a site, which is attached to a body
            auto* site = elem->Attribute("site");
            if (site) sensor.properties["site"] = std::string(site);

            auto* body_attr = elem->Attribute("body");
            if (body_attr) sensor.link = body_attr;

            auto* joint = elem->Attribute("joint");
            if (joint) sensor.properties["joint"] = std::string(joint);

            auto* noise = elem->Attribute("noise");
            if (noise) sensor.properties["noise"] = safe_stof(noise);

            auto* cutoff = elem->Attribute("cutoff");
            if (cutoff) sensor.properties["cutoff"] = safe_stof(cutoff);

            tree.sensors.push_back(std::move(sensor));
        }
    }

    fs::path resolve_path(const std::string& filename) const {
        fs::path result(filename);
        if (result.is_relative()) {
            result = base_dir_ / result;
        }
        return result;
    }
};

}  // anonymous namespace

std::unique_ptr<ArticulatedImporter> make_mjcf_importer() {
    return std::make_unique<MJCFImporter>();
}

}  // namespace simforge::adapters
