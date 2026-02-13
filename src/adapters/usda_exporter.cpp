// USDA (ASCII USD) exporter — no external SDK required.
// Writes text-based USD with UsdGeom mesh prims and UsdPhysics attributes.
#include "simforge/adapters/exporters.h"

#include <fstream>

#include <spdlog/spdlog.h>

namespace simforge::adapters {

namespace {

// Write an indentation string for the given depth.
std::string indent(int level) {
    return std::string(level * 4, ' ');
}

void emit_mesh_prim(std::ofstream& out, const Mesh& mesh,
                    const std::string& prim_name, int depth) {
    out << indent(depth) << "def Mesh \"" << prim_name << "\"\n";
    out << indent(depth) << "{\n";
    int d = depth + 1;

    // Points
    out << indent(d) << "point3f[] points = [";
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        if (i > 0) out << ", ";
        if (i % 4 == 0 && i > 0) out << "\n" << indent(d) << "    ";
        const auto& v = mesh.vertices[i];
        out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
    out << "]\n";

    // Face vertex counts (all triangles)
    out << indent(d) << "int[] faceVertexCounts = [";
    for (size_t i = 0; i < mesh.faces.size(); i++) {
        if (i > 0) out << ", ";
        out << "3";
    }
    out << "]\n";

    // Face vertex indices
    out << indent(d) << "int[] faceVertexIndices = [";
    for (size_t i = 0; i < mesh.faces.size(); i++) {
        if (i > 0) out << ", ";
        if (i % 4 == 0 && i > 0) out << "\n" << indent(d) << "    ";
        const auto& f = mesh.faces[i];
        out << f.v0 << ", " << f.v1 << ", " << f.v2;
    }
    out << "]\n";

    // Normals (if present)
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

        // Header
        out << "#usda 1.0\n";
        out << "(\n";
        out << "    defaultPrim = \"" << asset.name << "\"\n";
        out << "    metersPerUnit = 1.0\n";
        out << "    upAxis = \"Z\"\n";
        out << ")\n\n";

        // Root Xform
        out << "def Xform \"" << asset.name << "\" (\n";
        out << "    kind = \"component\"\n";
        out << ")\n";
        out << "{\n";

        // Visual meshes
        for (size_t i = 0; i < asset.meshes.size(); i++) {
            const auto& mesh = asset.meshes[i];
            std::string prim_name = mesh.name.empty()
                ? "Mesh_" + std::to_string(i)
                : mesh.name;
            emit_mesh_prim(out, mesh, prim_name, 1);
            out << "\n";
        }

        // Collision scope
        if (asset.collision && !asset.collision->hulls.empty()) {
            out << indent(1) << "def Scope \"Collision\"\n";
            out << indent(1) << "{\n";
            for (size_t i = 0; i < asset.collision->hulls.size(); i++) {
                std::string hull_name = "Hull_" + std::to_string(i);
                emit_mesh_prim(out, asset.collision->hulls[i], hull_name, 2);
            }
            out << indent(1) << "}\n\n";
        }

        // Physics properties (UsdPhysics schema attributes)
        if (asset.physics) {
            const auto& phys = *asset.physics;
            out << indent(1) << "float physics:mass = " << phys.mass << "\n";
            out << indent(1) << "point3f physics:centerOfMass = ("
                << phys.center_of_mass.x << ", "
                << phys.center_of_mass.y << ", "
                << phys.center_of_mass.z << ")\n";
            out << indent(1) << "float3 physics:diagonalInertia = ("
                << phys.inertia_diagonal.x << ", "
                << phys.inertia_diagonal.y << ", "
                << phys.inertia_diagonal.z << ")\n";
            out << indent(1) << "float physics:staticFriction = "
                << phys.material.static_friction << "\n";
            out << indent(1) << "float physics:dynamicFriction = "
                << phys.material.dynamic_friction << "\n";
            out << indent(1) << "float physics:restitution = "
                << phys.material.restitution << "\n";
        }

        out << "}\n";

        // Warnings for unsupported features
        if (!asset.materials.empty()) {
            spdlog::warn("USDA exporter: PBR materials not yet exported (Phase 5)");
        }
        if (!asset.lods.empty()) {
            spdlog::warn("USDA exporter: LODs not yet exported (Phase 5)");
        }

        return out.good();
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_usda_exporter() {
    return std::make_unique<USDAExporter>();
}

}  // namespace simforge::adapters
