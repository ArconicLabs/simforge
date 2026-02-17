// About: URDF importer — parses URDF XML files into KinematicTree with
// links, joints, actuators (from transmissions), and sensors.
#include "simforge/adapters/adapter.h"

#include <spdlog/spdlog.h>
#include <tinyxml2.h>
#include <unordered_set>

namespace simforge::adapters {

namespace {

Vec3 parse_xyz(const char* str) {
    Vec3 v{};
    if (str) std::sscanf(str, "%f %f %f", &v.x, &v.y, &v.z);
    return v;
}

Quaternion parse_rpy_to_quat(const char* str) {
    // URDF uses roll-pitch-yaw; store as-is in quaternion (simplified)
    // Full conversion would use proper RPY→quaternion math
    float r = 0, p = 0, y = 0;
    if (str) std::sscanf(str, "%f %f %f", &r, &p, &y);

    // RPY to quaternion
    float cr = std::cos(r * 0.5f), sr = std::sin(r * 0.5f);
    float cp = std::cos(p * 0.5f), sp = std::sin(p * 0.5f);
    float cy = std::cos(y * 0.5f), sy = std::sin(y * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

Pose parse_origin(const tinyxml2::XMLElement* elem) {
    Pose pose;
    if (!elem) return pose;
    auto* origin = elem->FirstChildElement("origin");
    if (!origin) return pose;

    pose.position = parse_xyz(origin->Attribute("xyz"));
    pose.orientation = parse_rpy_to_quat(origin->Attribute("rpy"));
    return pose;
}

class URDFImporter : public ArticulatedImporter {
public:
    [[nodiscard]] std::string name() const override { return "urdf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::URDF};
    }

    ArticulatedImportResult import_articulated(const fs::path& path) override {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
            throw std::runtime_error("Failed to parse URDF: " + path.string());
        }

        auto* robot = doc.FirstChildElement("robot");
        if (!robot) {
            throw std::runtime_error("URDF missing <robot> element");
        }

        ArticulatedImportResult result;
        auto base_dir = path.parent_path();

        // Parse links
        for (auto* elem = robot->FirstChildElement("link"); elem;
             elem = elem->NextSiblingElement("link")) {
            result.tree.links.push_back(parse_link(elem, base_dir, result.all_meshes));
        }

        // Parse joints
        for (auto* elem = robot->FirstChildElement("joint"); elem;
             elem = elem->NextSiblingElement("joint")) {
            result.tree.joints.push_back(parse_joint(elem));
        }

        // Parse transmissions → actuators
        for (auto* elem = robot->FirstChildElement("transmission"); elem;
             elem = elem->NextSiblingElement("transmission")) {
            auto act = parse_transmission(elem);
            if (!act.name.empty()) {
                result.tree.actuators.push_back(std::move(act));
            }
        }

        // Parse gazebo sensor plugins
        for (auto* gz = robot->FirstChildElement("gazebo"); gz;
             gz = gz->NextSiblingElement("gazebo")) {
            auto* ref = gz->Attribute("reference");
            for (auto* sensor_elem = gz->FirstChildElement("sensor"); sensor_elem;
                 sensor_elem = sensor_elem->NextSiblingElement("sensor")) {
                auto sensor = parse_gazebo_sensor(sensor_elem, ref ? ref : "");
                if (!sensor.name.empty()) {
                    result.tree.sensors.push_back(std::move(sensor));
                }
            }
        }

        // Determine root link
        std::unordered_set<std::string> children;
        for (const auto& j : result.tree.joints) {
            children.insert(j.child_link);
        }
        for (const auto& l : result.tree.links) {
            if (children.find(l.name) == children.end()) {
                result.tree.root_link = l.name;
                break;
            }
        }

        result.tree.build_index();

        spdlog::info("URDF imported: {} links, {} joints, {} actuators, {} sensors",
                     result.tree.links.size(), result.tree.joints.size(),
                     result.tree.actuators.size(), result.tree.sensors.size());

        return result;
    }

private:
    static Link parse_link(const tinyxml2::XMLElement* elem,
                           const fs::path& base_dir,
                           std::vector<Mesh>& all_meshes) {
        Link link;
        link.name = elem->Attribute("name") ? elem->Attribute("name") : "";

        // Visual geometry
        for (auto* vis = elem->FirstChildElement("visual"); vis;
             vis = vis->NextSiblingElement("visual")) {
            auto* geom = vis->FirstChildElement("geometry");
            if (!geom) continue;

            auto* mesh_elem = geom->FirstChildElement("mesh");
            if (mesh_elem) {
                auto* filename = mesh_elem->Attribute("filename");
                if (filename) {
                    auto mesh_path = resolve_path(filename, base_dir);
                    auto* importer = AdapterManager::instance().find_importer(
                        detect_format(mesh_path));
                    if (importer) {
                        try {
                            auto meshes = importer->import(mesh_path);
                            for (auto& m : meshes) {
                                m.recompute_bounds();
                                all_meshes.push_back(m);
                                link.visual_meshes.push_back(std::move(m));
                            }
                        } catch (const std::exception& e) {
                            spdlog::warn("Failed to load mesh '{}': {}", filename, e.what());
                        }
                    }
                }
            }
        }

        // Inertial
        auto* inertial = elem->FirstChildElement("inertial");
        if (inertial) {
            PhysicsProperties props;
            auto* mass_elem = inertial->FirstChildElement("mass");
            if (mass_elem) {
                mass_elem->QueryFloatAttribute("value", &props.mass);
            }

            auto* origin = inertial->FirstChildElement("origin");
            if (origin) {
                props.center_of_mass = parse_xyz(origin->Attribute("xyz"));
            }

            auto* inertia = inertial->FirstChildElement("inertia");
            if (inertia) {
                float ixx = 0, iyy = 0, izz = 0;
                inertia->QueryFloatAttribute("ixx", &ixx);
                inertia->QueryFloatAttribute("iyy", &iyy);
                inertia->QueryFloatAttribute("izz", &izz);
                props.inertia_diagonal = {ixx, iyy, izz};
            }

            link.physics = props;
        }

        // Collision geometry
        auto* coll = elem->FirstChildElement("collision");
        if (coll) {
            auto* geom = coll->FirstChildElement("geometry");
            if (geom) {
                auto* mesh_elem = geom->FirstChildElement("mesh");
                if (mesh_elem) {
                    auto* filename = mesh_elem->Attribute("filename");
                    if (filename) {
                        auto mesh_path = resolve_path(filename, base_dir);
                        auto* importer = AdapterManager::instance().find_importer(
                            detect_format(mesh_path));
                        if (importer) {
                            try {
                                auto meshes = importer->import(mesh_path);
                                CollisionMesh cm;
                                cm.type = CollisionType::TriangleMesh;
                                for (auto& m : meshes) {
                                    m.recompute_bounds();
                                    cm.hulls.push_back(std::move(m));
                                }
                                link.collision = std::move(cm);
                            } catch (...) {}
                        }
                    }
                }
            }
        }

        return link;
    }

    static Joint parse_joint(const tinyxml2::XMLElement* elem) {
        Joint joint;
        joint.name = elem->Attribute("name") ? elem->Attribute("name") : "";

        auto* type_str = elem->Attribute("type");
        if (type_str) joint.type = parse_joint_type(type_str);

        auto* parent = elem->FirstChildElement("parent");
        if (parent && parent->Attribute("link"))
            joint.parent_link = parent->Attribute("link");

        auto* child = elem->FirstChildElement("child");
        if (child && child->Attribute("link"))
            joint.child_link = child->Attribute("link");

        auto* origin = elem->FirstChildElement("origin");
        if (origin) {
            joint.origin.position = parse_xyz(origin->Attribute("xyz"));
            joint.origin.orientation = parse_rpy_to_quat(origin->Attribute("rpy"));
        }

        auto* axis = elem->FirstChildElement("axis");
        if (axis) {
            joint.axis = parse_xyz(axis->Attribute("xyz"));
        }

        auto* limit = elem->FirstChildElement("limit");
        if (limit) {
            JointLimits lim;
            limit->QueryFloatAttribute("lower", &lim.lower);
            limit->QueryFloatAttribute("upper", &lim.upper);
            limit->QueryFloatAttribute("velocity", &lim.velocity);
            limit->QueryFloatAttribute("effort", &lim.effort);
            joint.limits = lim;
        }

        auto* dynamics = elem->FirstChildElement("dynamics");
        if (dynamics) {
            JointDynamics dyn;
            dynamics->QueryFloatAttribute("damping", &dyn.damping);
            dynamics->QueryFloatAttribute("friction", &dyn.friction);
            joint.dynamics = dyn;
        }

        return joint;
    }

    static Actuator parse_transmission(const tinyxml2::XMLElement* elem) {
        Actuator act;
        act.name = elem->Attribute("name") ? elem->Attribute("name") : "";

        auto* joint_elem = elem->FirstChildElement("joint");
        if (joint_elem && joint_elem->Attribute("name")) {
            act.joint = joint_elem->Attribute("name");
        }

        auto* type_elem = elem->FirstChildElement("type");
        if (type_elem && type_elem->GetText()) {
            std::string type = type_elem->GetText();
            if (type.find("Position") != std::string::npos)
                act.control_mode = ControlMode::Position;
            else if (type.find("Velocity") != std::string::npos)
                act.control_mode = ControlMode::Velocity;
            else if (type.find("Effort") != std::string::npos)
                act.control_mode = ControlMode::Effort;
        }

        auto* ratio = elem->FirstChildElement("mechanicalReduction");
        if (ratio && ratio->GetText()) {
            act.gear_ratio = std::stof(ratio->GetText());
        }

        return act;
    }

    static Sensor parse_gazebo_sensor(const tinyxml2::XMLElement* elem,
                                      const std::string& ref_link) {
        Sensor sensor;
        sensor.name = elem->Attribute("name") ? elem->Attribute("name") : "";
        sensor.type = elem->Attribute("type") ? elem->Attribute("type") : "";
        sensor.link = ref_link;

        auto* rate = elem->FirstChildElement("update_rate");
        if (rate && rate->GetText()) {
            sensor.properties["update_rate"] = std::stof(rate->GetText());
        }

        return sensor;
    }

    static fs::path resolve_path(const std::string& filename, const fs::path& base_dir) {
        // Handle package:// URIs by stripping the prefix
        std::string path = filename;
        if (path.substr(0, 10) == "package://") {
            path = path.substr(10);
            // Strip package name (first path component)
            auto pos = path.find('/');
            if (pos != std::string::npos) {
                path = path.substr(pos + 1);
            }
        }

        fs::path result(path);
        if (result.is_relative()) {
            result = base_dir / result;
        }
        return result;
    }

};

}  // anonymous namespace

std::unique_ptr<ArticulatedImporter> make_urdf_importer() {
    return std::make_unique<URDFImporter>();
}

}  // namespace simforge::adapters
