// About: MJCF exporter — MuJoCo XML format with nested body tree,
// joints, actuators, and sensors. Falls back to single-body for
// non-articulated assets.
#include "simforge/adapters/exporters.h"
#include "simforge/adapters/mesh_writer.h"

#include <spdlog/spdlog.h>
#include <tinyxml2.h>
#include <unordered_map>

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

std::string quat_str(const Quaternion& q) {
    return ftos(q.w) + " " + ftos(q.x) + " " + ftos(q.y) + " " + ftos(q.z);
}

void emit_inertial(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* body,
                   const PhysicsProperties& phys) {
    auto* inertial = doc.NewElement("inertial");
    inertial->SetAttribute("pos", vec3_str(phys.center_of_mass).c_str());
    inertial->SetAttribute("mass", ftos(phys.mass).c_str());
    inertial->SetAttribute("diaginertia", vec3_str(phys.inertia_diagonal).c_str());
    body->InsertEndChild(inertial);
}

class MJCFExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "mjcf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::MJCF};
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

        auto asset_dir = output_path.parent_path() / "assets";
        fs::create_directories(asset_dir);

        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* mujoco = doc.NewElement("mujoco");
        mujoco->SetAttribute("model", asset.name.c_str());
        doc.InsertEndChild(mujoco);

        // <asset> — write and declare mesh files per link
        auto* asset_el = doc.NewElement("asset");
        mujoco->InsertEndChild(asset_el);

        for (const auto& link : tree.links) {
            for (size_t i = 0; i < link.visual_meshes.size(); i++) {
                auto filename = link.name + "_" + std::to_string(i) + ".stl";
                if (!write_binary_stl(link.visual_meshes[i], asset_dir / filename)) {
                    spdlog::warn("MJCF exporter: failed to write mesh for link '{}'", link.name);
                }

                auto* mesh = doc.NewElement("mesh");
                mesh->SetAttribute("name", (link.name + "_visual_" + std::to_string(i)).c_str());
                mesh->SetAttribute("file", ("assets/" + filename).c_str());
                asset_el->InsertEndChild(mesh);
            }
        }

        // Build parent → children map for tree traversal
        std::unordered_map<std::string, std::vector<const Joint*>> children_map;
        for (const auto& j : tree.joints) {
            children_map[j.parent_link].push_back(&j);
        }

        // <worldbody> — emit nested body tree
        auto* worldbody = doc.NewElement("worldbody");
        mujoco->InsertEndChild(worldbody);

        auto* root_link = tree.find_link(tree.root_link);
        if (root_link) {
            emit_body(doc, worldbody, *root_link, tree, children_map);
        }

        // <actuator> section
        if (!tree.actuators.empty()) {
            auto* actuator_section = doc.NewElement("actuator");
            mujoco->InsertEndChild(actuator_section);

            for (const auto& act : tree.actuators) {
                const char* tag = "motor";
                if (act.control_mode == ControlMode::Position) tag = "position";
                else if (act.control_mode == ControlMode::Velocity) tag = "velocity";

                auto* act_el = doc.NewElement(tag);
                act_el->SetAttribute("name", act.name.c_str());
                act_el->SetAttribute("joint", act.joint.c_str());

                if (act.gear_ratio != 0.0f) {
                    act_el->SetAttribute("gear", ftos(act.gear_ratio).c_str());
                }
                if (act.max_torque > 0.0f) {
                    std::string range = ftos(-act.max_torque) + " " + ftos(act.max_torque);
                    act_el->SetAttribute("forcerange", range.c_str());
                }

                actuator_section->InsertEndChild(act_el);
            }
        }

        // <sensor> section
        if (!tree.sensors.empty()) {
            auto* sensor_section = doc.NewElement("sensor");
            mujoco->InsertEndChild(sensor_section);

            for (const auto& s : tree.sensors) {
                auto* sensor_el = doc.NewElement(s.type.c_str());
                sensor_el->SetAttribute("name", s.name.c_str());

                if (!s.link.empty()) {
                    sensor_el->SetAttribute("body", s.link.c_str());
                }
                if (s.properties.contains("site")) {
                    sensor_el->SetAttribute("site",
                        s.properties["site"].get<std::string>().c_str());
                }
                if (s.properties.contains("noise")) {
                    sensor_el->SetAttribute("noise",
                        ftos(s.properties["noise"].get<float>()).c_str());
                }

                sensor_section->InsertEndChild(sensor_el);
            }
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }

    void emit_body(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* parent,
                   const Link& link, const KinematicTree& tree,
                   const std::unordered_map<std::string, std::vector<const Joint*>>& children) {
        auto* body = doc.NewElement("body");
        body->SetAttribute("name", link.name.c_str());

        body->SetAttribute("pos", vec3_str(link.origin.position).c_str());
        body->SetAttribute("quat", quat_str(link.origin.orientation).c_str());

        // Inertial
        if (link.physics) {
            emit_inertial(doc, body, *link.physics);
        }

        // Visual geoms
        for (size_t i = 0; i < link.visual_meshes.size(); i++) {
            auto* geom = doc.NewElement("geom");
            geom->SetAttribute("type", "mesh");
            geom->SetAttribute("mesh", (link.name + "_visual_" + std::to_string(i)).c_str());
            geom->SetAttribute("contype", "0");
            geom->SetAttribute("conaffinity", "0");
            body->InsertEndChild(geom);
        }

        // Recurse into child links via joints
        auto it = children.find(link.name);
        if (it != children.end()) {
            for (const auto* joint : it->second) {
                auto* child_link = tree.find_link(joint->child_link);
                if (!child_link) continue;

                // Emit the joint inside the child body element
                // First create the child body, then add joint inside it
                emit_child_body_with_joint(doc, body, *child_link, *joint, tree, children);
            }
        }

        parent->InsertEndChild(body);
    }

    void emit_child_body_with_joint(
            tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* parent_body,
            const Link& link, const Joint& joint, const KinematicTree& tree,
            const std::unordered_map<std::string, std::vector<const Joint*>>& children) {
        auto* body = doc.NewElement("body");
        body->SetAttribute("name", link.name.c_str());
        body->SetAttribute("pos", vec3_str(joint.origin.position).c_str());

        // Emit joint (unless fixed — MuJoCo has no fixed joint element)
        if (joint.type != JointType::Fixed) {
            auto* joint_el = doc.NewElement("joint");
            joint_el->SetAttribute("name", joint.name.c_str());

            const char* jtype = "hinge";
            if (joint.type == JointType::Prismatic) jtype = "slide";
            else if (joint.type == JointType::Spherical) jtype = "ball";
            else if (joint.type == JointType::Floating) jtype = "free";
            joint_el->SetAttribute("type", jtype);

            joint_el->SetAttribute("axis", vec3_str(joint.axis).c_str());

            if (joint.limits) {
                std::string range = ftos(joint.limits->lower) + " " + ftos(joint.limits->upper);
                joint_el->SetAttribute("range", range.c_str());
            }
            if (joint.dynamics) {
                if (joint.dynamics->damping != 0.0f) {
                    joint_el->SetAttribute("damping", ftos(joint.dynamics->damping).c_str());
                }
                if (joint.dynamics->friction != 0.0f) {
                    joint_el->SetAttribute("frictionloss", ftos(joint.dynamics->friction).c_str());
                }
            }

            body->InsertEndChild(joint_el);
        }

        // Inertial
        if (link.physics) {
            emit_inertial(doc, body, *link.physics);
        }

        // Visual geoms
        for (size_t i = 0; i < link.visual_meshes.size(); i++) {
            auto* geom = doc.NewElement("geom");
            geom->SetAttribute("type", "mesh");
            geom->SetAttribute("mesh", (link.name + "_visual_" + std::to_string(i)).c_str());
            geom->SetAttribute("contype", "0");
            geom->SetAttribute("conaffinity", "0");
            body->InsertEndChild(geom);
        }

        // Recurse into children
        auto it = children.find(link.name);
        if (it != children.end()) {
            for (const auto* child_joint : it->second) {
                auto* child_link = tree.find_link(child_joint->child_link);
                if (!child_link) continue;
                emit_child_body_with_joint(doc, body, *child_link, *child_joint, tree, children);
            }
        }

        parent_body->InsertEndChild(body);
    }

    bool export_single_body(const Asset& asset, const fs::path& output_path) {
        if (asset.meshes.empty()) {
            spdlog::error("MJCF exporter: no meshes in asset '{}'", asset.name);
            return false;
        }

        auto asset_dir = output_path.parent_path() / "assets";
        fs::create_directories(asset_dir);

        std::string visual_filename = asset.name + ".stl";
        if (!write_binary_stl(asset.meshes[0], asset_dir / visual_filename)) {
            spdlog::error("MJCF exporter: failed to write visual mesh");
            return false;
        }

        std::string collision_filename;
        bool has_collision = asset.collision && !asset.collision->hulls.empty();
        if (has_collision) {
            collision_filename = asset.name + "_collision.stl";
            Mesh merged;
            merged.name = asset.name + "_collision";
            uint32_t offset = 0;
            for (const auto& hull : asset.collision->hulls) {
                for (const auto& v : hull.vertices) merged.vertices.push_back(v);
                for (const auto& f : hull.faces) {
                    merged.faces.push_back({f.v0 + offset, f.v1 + offset, f.v2 + offset});
                }
                offset += static_cast<uint32_t>(hull.vertices.size());
            }
            if (!write_binary_stl(merged, asset_dir / collision_filename)) {
                spdlog::error("MJCF exporter: failed to write collision mesh");
                return false;
            }
        }

        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* mujoco = doc.NewElement("mujoco");
        mujoco->SetAttribute("model", asset.name.c_str());
        doc.InsertEndChild(mujoco);

        auto* asset_el = doc.NewElement("asset");
        mujoco->InsertEndChild(asset_el);

        auto* visual_mesh = doc.NewElement("mesh");
        visual_mesh->SetAttribute("name", (asset.name + "_visual").c_str());
        visual_mesh->SetAttribute("file", ("assets/" + visual_filename).c_str());
        asset_el->InsertEndChild(visual_mesh);

        if (has_collision) {
            auto* coll_mesh = doc.NewElement("mesh");
            coll_mesh->SetAttribute("name", (asset.name + "_collision").c_str());
            coll_mesh->SetAttribute("file", ("assets/" + collision_filename).c_str());
            asset_el->InsertEndChild(coll_mesh);
        }

        // <default> — physics material defaults
        if (asset.physics) {
            const auto& mat = asset.physics->material;
            auto* default_el = doc.NewElement("default");
            mujoco->InsertEndChild(default_el);

            auto* geom_default = doc.NewElement("geom");
            std::string friction = ftos(mat.static_friction) + " "
                                 + ftos(mat.dynamic_friction) + " 0.0005";
            geom_default->SetAttribute("friction", friction.c_str());
            geom_default->SetAttribute("solref", "-10000 -200");
            default_el->InsertEndChild(geom_default);
        }

        auto* worldbody = doc.NewElement("worldbody");
        mujoco->InsertEndChild(worldbody);

        auto* body = doc.NewElement("body");
        body->SetAttribute("name", asset.name.c_str());
        worldbody->InsertEndChild(body);

        auto* vis_geom = doc.NewElement("geom");
        vis_geom->SetAttribute("type", "mesh");
        vis_geom->SetAttribute("mesh", (asset.name + "_visual").c_str());
        vis_geom->SetAttribute("contype", "0");
        vis_geom->SetAttribute("conaffinity", "0");
        body->InsertEndChild(vis_geom);

        if (has_collision) {
            auto* coll_geom = doc.NewElement("geom");
            coll_geom->SetAttribute("type", "mesh");
            coll_geom->SetAttribute("mesh", (asset.name + "_collision").c_str());
            body->InsertEndChild(coll_geom);
        }

        if (asset.physics) {
            emit_inertial(doc, body, *asset.physics);
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_mjcf_exporter() {
    return std::make_unique<MJCFExporter>();
}

}  // namespace simforge::adapters
