// About: URDF exporter — writes robot description with links, joints,
// transmissions, and Gazebo sensor plugins. Falls back to single-link
// for non-articulated assets.
#include "simforge/adapters/exporters.h"
#include "simforge/adapters/mesh_writer.h"

#include <spdlog/spdlog.h>
#include <tinyxml2.h>

namespace simforge::adapters {

namespace {

std::string ftos(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return buf;
}

std::string vec3_str(const Vec3& v) {
    return ftos(v.x) + " " + ftos(v.y) + " " + ftos(v.z);
}

// Quaternion to RPY (roll-pitch-yaw) conversion
std::string quat_to_rpy_str(const Quaternion& q) {
    // Roll (x-axis)
    float sinr = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = std::atan2(sinr, cosr);

    // Pitch (y-axis)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    float pitch = (std::abs(sinp) >= 1.0f)
        ? std::copysign(3.14159265f / 2.0f, sinp)
        : std::asin(sinp);

    // Yaw (z-axis)
    float siny = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = std::atan2(siny, cosy);

    return ftos(roll) + " " + ftos(pitch) + " " + ftos(yaw);
}

void emit_origin(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* parent,
                 const Pose& pose) {
    auto* origin = doc.NewElement("origin");
    origin->SetAttribute("xyz", vec3_str(pose.position).c_str());
    origin->SetAttribute("rpy", quat_to_rpy_str(pose.orientation).c_str());
    parent->InsertEndChild(origin);
}

void emit_inertial(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* link_el,
                   const PhysicsProperties& phys) {
    auto* inertial = doc.NewElement("inertial");
    link_el->InsertEndChild(inertial);

    auto* mass = doc.NewElement("mass");
    mass->SetAttribute("value", ftos(phys.mass).c_str());
    inertial->InsertEndChild(mass);

    auto* origin = doc.NewElement("origin");
    origin->SetAttribute("xyz", vec3_str(phys.center_of_mass).c_str());
    origin->SetAttribute("rpy", "0 0 0");
    inertial->InsertEndChild(origin);

    auto* inertia = doc.NewElement("inertia");
    inertia->SetAttribute("ixx", ftos(phys.inertia_diagonal.x).c_str());
    inertia->SetAttribute("ixy", "0");
    inertia->SetAttribute("ixz", "0");
    inertia->SetAttribute("iyy", ftos(phys.inertia_diagonal.y).c_str());
    inertia->SetAttribute("iyz", "0");
    inertia->SetAttribute("izz", ftos(phys.inertia_diagonal.z).c_str());
    inertial->InsertEndChild(inertia);
}

class URDFExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "urdf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::URDF};
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        if (asset.is_articulated()) {
            return export_articulated(asset, output_path);
        }
        return export_single_body(asset, output_path);
    }

private:
    bool export_articulated(const Asset& asset, const fs::path& output_path) {
        const auto& tree = *asset.kinematic_tree;

        auto mesh_dir = output_path.parent_path() / "meshes";
        fs::create_directories(mesh_dir);

        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* robot = doc.NewElement("robot");
        robot->SetAttribute("name", asset.name.c_str());
        doc.InsertEndChild(robot);

        // Emit links
        for (const auto& link : tree.links) {
            auto* link_el = doc.NewElement("link");
            link_el->SetAttribute("name", link.name.c_str());
            robot->InsertEndChild(link_el);

            // Visual meshes
            for (size_t i = 0; i < link.visual_meshes.size(); i++) {
                auto filename = link.name + "_visual_" + std::to_string(i) + ".obj";
                auto path = mesh_dir / filename;
                write_obj(link.visual_meshes[i], path);

                auto* visual = doc.NewElement("visual");
                link_el->InsertEndChild(visual);

                emit_origin(doc, visual, link.origin);

                auto* geometry = doc.NewElement("geometry");
                visual->InsertEndChild(geometry);
                auto* mesh_el = doc.NewElement("mesh");
                mesh_el->SetAttribute("filename", ("meshes/" + filename).c_str());
                geometry->InsertEndChild(mesh_el);
            }

            // Collision
            if (link.collision && !link.collision->hulls.empty()) {
                auto filename = link.name + "_collision.obj";
                auto path = mesh_dir / filename;
                write_obj_multi(link.collision->hulls, path);

                auto* collision = doc.NewElement("collision");
                link_el->InsertEndChild(collision);
                auto* geometry = doc.NewElement("geometry");
                collision->InsertEndChild(geometry);
                auto* mesh_el = doc.NewElement("mesh");
                mesh_el->SetAttribute("filename", ("meshes/" + filename).c_str());
                geometry->InsertEndChild(mesh_el);
            }

            // Inertial
            if (link.physics) {
                emit_inertial(doc, link_el, *link.physics);
            }
        }

        // Emit joints
        for (const auto& joint : tree.joints) {
            auto* joint_el = doc.NewElement("joint");
            joint_el->SetAttribute("name", joint.name.c_str());
            joint_el->SetAttribute("type", joint_type_to_string(joint.type).c_str());
            robot->InsertEndChild(joint_el);

            auto* parent = doc.NewElement("parent");
            parent->SetAttribute("link", joint.parent_link.c_str());
            joint_el->InsertEndChild(parent);

            auto* child = doc.NewElement("child");
            child->SetAttribute("link", joint.child_link.c_str());
            joint_el->InsertEndChild(child);

            emit_origin(doc, joint_el, joint.origin);

            auto* axis = doc.NewElement("axis");
            axis->SetAttribute("xyz", vec3_str(joint.axis).c_str());
            joint_el->InsertEndChild(axis);

            if (joint.limits) {
                auto* limit = doc.NewElement("limit");
                limit->SetAttribute("lower", ftos(joint.limits->lower).c_str());
                limit->SetAttribute("upper", ftos(joint.limits->upper).c_str());
                limit->SetAttribute("velocity", ftos(joint.limits->velocity).c_str());
                limit->SetAttribute("effort", ftos(joint.limits->effort).c_str());
                joint_el->InsertEndChild(limit);
            }

            if (joint.dynamics) {
                auto* dyn = doc.NewElement("dynamics");
                dyn->SetAttribute("damping", ftos(joint.dynamics->damping).c_str());
                dyn->SetAttribute("friction", ftos(joint.dynamics->friction).c_str());
                joint_el->InsertEndChild(dyn);
            }
        }

        // Emit transmissions for actuators
        for (const auto& act : tree.actuators) {
            auto* trans = doc.NewElement("transmission");
            trans->SetAttribute("name", act.name.c_str());
            robot->InsertEndChild(trans);

            auto* type = doc.NewElement("type");
            type->SetText("transmission_interface/SimpleTransmission");
            trans->InsertEndChild(type);

            auto* joint = doc.NewElement("joint");
            joint->SetAttribute("name", act.joint.c_str());
            trans->InsertEndChild(joint);

            auto* actuator = doc.NewElement("actuator");
            actuator->SetAttribute("name", act.name.c_str());
            trans->InsertEndChild(actuator);

            if (act.gear_ratio != 0.0f) {
                auto* ratio = doc.NewElement("mechanicalReduction");
                ratio->SetText(ftos(act.gear_ratio).c_str());
                trans->InsertEndChild(ratio);
            }
        }

        // Emit Gazebo sensor plugins grouped by link
        for (const auto& sensor : tree.sensors) {
            auto* gz = doc.NewElement("gazebo");
            if (!sensor.link.empty()) {
                gz->SetAttribute("reference", sensor.link.c_str());
            }
            robot->InsertEndChild(gz);

            auto* sensor_el = doc.NewElement("sensor");
            sensor_el->SetAttribute("name", sensor.name.c_str());
            sensor_el->SetAttribute("type", sensor.type.c_str());
            gz->InsertEndChild(sensor_el);

            if (sensor.properties.contains("update_rate")) {
                auto* rate = doc.NewElement("update_rate");
                rate->SetText(ftos(sensor.properties["update_rate"].get<float>()).c_str());
                sensor_el->InsertEndChild(rate);
            }
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }

    bool export_single_body(const Asset& asset, const fs::path& output_path) {
        if (asset.meshes.empty()) {
            spdlog::error("URDF exporter: no meshes in asset '{}'", asset.name);
            return false;
        }

        auto mesh_dir = output_path.parent_path() / "meshes";
        fs::create_directories(mesh_dir);

        std::string visual_filename = asset.name + ".obj";
        if (!write_obj(asset.meshes[0], mesh_dir / visual_filename)) {
            spdlog::error("URDF exporter: failed to write visual mesh");
            return false;
        }

        std::string collision_filename;
        bool has_collision = asset.collision && !asset.collision->hulls.empty();
        if (has_collision) {
            collision_filename = asset.name + "_collision.obj";
            if (!write_obj_multi(asset.collision->hulls, mesh_dir / collision_filename)) {
                spdlog::error("URDF exporter: failed to write collision mesh");
                return false;
            }
        }

        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* robot = doc.NewElement("robot");
        robot->SetAttribute("name", asset.name.c_str());
        doc.InsertEndChild(robot);

        auto* link = doc.NewElement("link");
        link->SetAttribute("name", "base_link");
        robot->InsertEndChild(link);

        {
            auto* visual = doc.NewElement("visual");
            link->InsertEndChild(visual);
            auto* geometry = doc.NewElement("geometry");
            visual->InsertEndChild(geometry);
            auto* mesh_el = doc.NewElement("mesh");
            mesh_el->SetAttribute("filename", ("meshes/" + visual_filename).c_str());
            geometry->InsertEndChild(mesh_el);
        }

        if (has_collision) {
            auto* collision = doc.NewElement("collision");
            link->InsertEndChild(collision);
            auto* geometry = doc.NewElement("geometry");
            collision->InsertEndChild(geometry);
            auto* mesh_el = doc.NewElement("mesh");
            mesh_el->SetAttribute("filename", ("meshes/" + collision_filename).c_str());
            geometry->InsertEndChild(mesh_el);
        }

        if (asset.physics) {
            emit_inertial(doc, link, *asset.physics);
        }

        if (!asset.materials.empty()) {
            spdlog::warn("URDF exporter: PBR materials dropped (URDF has no material support)");
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_urdf_exporter() {
    return std::make_unique<URDFExporter>();
}

}  // namespace simforge::adapters
