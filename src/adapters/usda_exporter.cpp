// About: USDA (ASCII USD) exporter — writes text-based USD with UsdGeom mesh
// prims, UsdPhysics attributes, and PhysicsArticulationRootAPI for
// articulated assets.
#include "simforge/adapters/exporters.h"

#include <fstream>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace simforge::adapters {

namespace {

std::string indent(int level) {
    return std::string(level * 4, ' ');
}

std::string ftos(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return buf;
}

void emit_mesh_prim(std::ofstream& out, const Mesh& mesh,
                    const std::string& prim_name, int depth) {
    out << indent(depth) << "def Mesh \"" << prim_name << "\"\n";
    out << indent(depth) << "{\n";
    int d = depth + 1;

    out << indent(d) << "point3f[] points = [";
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        if (i > 0) out << ", ";
        if (i % 4 == 0 && i > 0) out << "\n" << indent(d) << "    ";
        const auto& v = mesh.vertices[i];
        out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
    out << "]\n";

    out << indent(d) << "int[] faceVertexCounts = [";
    for (size_t i = 0; i < mesh.faces.size(); i++) {
        if (i > 0) out << ", ";
        out << "3";
    }
    out << "]\n";

    out << indent(d) << "int[] faceVertexIndices = [";
    for (size_t i = 0; i < mesh.faces.size(); i++) {
        if (i > 0) out << ", ";
        if (i % 4 == 0 && i > 0) out << "\n" << indent(d) << "    ";
        const auto& f = mesh.faces[i];
        out << f.v0 << ", " << f.v1 << ", " << f.v2;
    }
    out << "]\n";

    if (!mesh.normals.empty()) {
        out << indent(d) << "normal3f[] normals = [";
        for (size_t i = 0; i < mesh.normals.size(); i++) {
            if (i > 0) out << ", ";
            if (i % 4 == 0 && i > 0) out << "\n" << indent(d) << "    ";
            const auto& n = mesh.normals[i];
            out << "(" << n.x << ", " << n.y << ", " << n.z << ")";
        }
        out << "]\n";
    }

    out << indent(depth) << "}\n";
}

void emit_physics(std::ofstream& out, const PhysicsProperties& phys, int depth) {
    out << indent(depth) << "float physics:mass = " << phys.mass << "\n";
    out << indent(depth) << "point3f physics:centerOfMass = ("
        << phys.center_of_mass.x << ", "
        << phys.center_of_mass.y << ", "
        << phys.center_of_mass.z << ")\n";
    out << indent(depth) << "float3 physics:diagonalInertia = ("
        << phys.inertia_diagonal.x << ", "
        << phys.inertia_diagonal.y << ", "
        << phys.inertia_diagonal.z << ")\n";
}

std::string usd_joint_type(JointType type) {
    switch (type) {
        case JointType::Revolute:   return "PhysicsRevoluteJoint";
        case JointType::Prismatic:  return "PhysicsPrismaticJoint";
        case JointType::Fixed:      return "PhysicsFixedJoint";
        case JointType::Spherical:  return "PhysicsSphericalJoint";
        default:                    return "PhysicsJoint";
    }
}

class USDAExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "usda_text"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::USD, SourceFormat::USDA};
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        std::ofstream out(output_path);
        if (!out) {
            spdlog::error("USDA exporter: cannot open {}", output_path.string());
            return false;
        }

        out << "#usda 1.0\n";
        out << "(\n";
        out << "    defaultPrim = \"" << asset.name << "\"\n";
        out << "    metersPerUnit = 1.0\n";
        out << "    upAxis = \"Z\"\n";
        out << ")\n\n";

        if (asset.is_articulated()) {
            export_articulated(out, asset);
        } else {
            export_single_body(out, asset);
        }

        return out.good();
    }

private:
    void export_articulated(std::ofstream& out, const Asset& asset) {
        const auto& tree = *asset.kinematic_tree;

        // Root Xform with PhysicsArticulationRootAPI
        out << "def Xform \"" << asset.name << "\" (\n";
        out << "    kind = \"component\"\n";
        out << "    prepend apiSchemas = [\"PhysicsArticulationRootAPI\"]\n";
        out << ")\n";
        out << "{\n";

        // Build parent → children map
        std::unordered_map<std::string, std::vector<const Joint*>> children_map;
        for (const auto& j : tree.joints) {
            children_map[j.parent_link].push_back(&j);
        }

        // Emit root link and recurse
        auto* root = tree.find_link(tree.root_link);
        if (root) {
            emit_link_xform(out, *root, tree, children_map, 1);
        }

        // Emit joint prims at root scope
        for (const auto& joint : tree.joints) {
            emit_joint_prim(out, joint, 1);
        }

        out << "}\n";
    }

    void emit_link_xform(std::ofstream& out, const Link& link,
                         const KinematicTree& tree,
                         const std::unordered_map<std::string, std::vector<const Joint*>>& children,
                         int depth) {
        out << indent(depth) << "def Xform \"" << link.name << "\" (\n";
        out << indent(depth) << "    prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"]\n";
        out << indent(depth) << ")\n";
        out << indent(depth) << "{\n";

        // Position
        const auto& pos = link.origin.position;
        out << indent(depth + 1) << "double3 xformOp:translate = ("
            << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        out << indent(depth + 1) << "uniform token[] xformOpOrder = [\"xformOp:translate\"]\n";

        // Physics
        if (link.physics) {
            emit_physics(out, *link.physics, depth + 1);
        }

        // Visual meshes
        for (size_t i = 0; i < link.visual_meshes.size(); i++) {
            std::string name = "Visual_" + std::to_string(i);
            emit_mesh_prim(out, link.visual_meshes[i], name, depth + 1);
        }

        // Collision meshes
        if (link.collision && !link.collision->hulls.empty()) {
            out << indent(depth + 1) << "def Scope \"Collision\"\n";
            out << indent(depth + 1) << "{\n";
            for (size_t i = 0; i < link.collision->hulls.size(); i++) {
                emit_mesh_prim(out, link.collision->hulls[i],
                             "Hull_" + std::to_string(i), depth + 2);
            }
            out << indent(depth + 1) << "}\n";
        }

        // Recurse into children
        auto it = children.find(link.name);
        if (it != children.end()) {
            for (const auto* joint : it->second) {
                auto* child = tree.find_link(joint->child_link);
                if (child) {
                    emit_link_xform(out, *child, tree, children, depth + 1);
                }
            }
        }

        out << indent(depth) << "}\n\n";
    }

    void emit_joint_prim(std::ofstream& out, const Joint& joint, int depth) {
        std::string usd_type = usd_joint_type(joint.type);
        out << indent(depth) << "def " << usd_type << " \"" << joint.name << "\"\n";
        out << indent(depth) << "{\n";

        int d = depth + 1;
        out << indent(d) << "rel physics:body0 = <" << joint.parent_link << ">\n";
        out << indent(d) << "rel physics:body1 = <" << joint.child_link << ">\n";

        // Axis
        std::string axis = "X";
        if (joint.axis.y != 0) axis = "Y";
        if (joint.axis.z != 0) axis = "Z";
        out << indent(d) << "uniform token physics:axis = \"" << axis << "\"\n";

        // Limits
        if (joint.limits) {
            out << indent(d) << "float physics:lowerLimit = " << joint.limits->lower << "\n";
            out << indent(d) << "float physics:upperLimit = " << joint.limits->upper << "\n";
        }

        // Position
        const auto& pos = joint.origin.position;
        out << indent(d) << "float3 physics:localPos0 = ("
            << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

        out << indent(depth) << "}\n\n";
    }

    void export_single_body(std::ofstream& out, const Asset& asset) {
        out << "def Xform \"" << asset.name << "\" (\n";
        out << "    kind = \"component\"\n";
        out << ")\n";
        out << "{\n";

        for (size_t i = 0; i < asset.meshes.size(); i++) {
            const auto& mesh = asset.meshes[i];
            std::string prim_name = mesh.name.empty()
                ? "Mesh_" + std::to_string(i)
                : mesh.name;
            emit_mesh_prim(out, mesh, prim_name, 1);
            out << "\n";
        }

        if (asset.collision && !asset.collision->hulls.empty()) {
            out << indent(1) << "def Scope \"Collision\"\n";
            out << indent(1) << "{\n";
            for (size_t i = 0; i < asset.collision->hulls.size(); i++) {
                emit_mesh_prim(out, asset.collision->hulls[i],
                             "Hull_" + std::to_string(i), 2);
            }
            out << indent(1) << "}\n\n";
        }

        if (asset.physics) {
            const auto& phys = *asset.physics;
            emit_physics(out, phys, 1);
            out << indent(1) << "float physics:staticFriction = "
                << phys.material.static_friction << "\n";
            out << indent(1) << "float physics:dynamicFriction = "
                << phys.material.dynamic_friction << "\n";
            out << indent(1) << "float physics:restitution = "
                << phys.material.restitution << "\n";
        }

        out << "}\n";
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_usda_exporter() {
    return std::make_unique<USDAExporter>();
}

}  // namespace simforge::adapters
