// URDF exporter — single-link robot description with external OBJ meshes.
#include "simforge/adapters/exporters.h"
#include "simforge/adapters/mesh_writer.h"

#include <spdlog/spdlog.h>
#include <tinyxml2.h>

namespace simforge::adapters {

namespace {

// Format a float to a string without trailing zeros.
std::string ftos(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return buf;
}

class URDFExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "urdf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::URDF};
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        if (asset.meshes.empty()) {
            spdlog::error("URDF exporter: no meshes in asset '{}'", asset.name);
            return false;
        }

        // Create meshes/ subdirectory next to the output file
        auto mesh_dir = output_path.parent_path() / "meshes";
        fs::create_directories(mesh_dir);

        // Write visual mesh
        std::string visual_filename = asset.name + ".obj";
        auto visual_path = mesh_dir / visual_filename;
        if (!write_obj(asset.meshes[0], visual_path)) {
            spdlog::error("URDF exporter: failed to write visual mesh");
            return false;
        }

        // Write collision mesh (merge all hulls into one OBJ)
        std::string collision_filename;
        bool has_collision = asset.collision && !asset.collision->hulls.empty();
        if (has_collision) {
            collision_filename = asset.name + "_collision.obj";
            auto collision_path = mesh_dir / collision_filename;
            if (!write_obj_multi(asset.collision->hulls, collision_path)) {
                spdlog::error("URDF exporter: failed to write collision mesh");
                return false;
            }
        }

        // Build XML
        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* robot = doc.NewElement("robot");
        robot->SetAttribute("name", asset.name.c_str());
        doc.InsertEndChild(robot);

        auto* link = doc.NewElement("link");
        link->SetAttribute("name", "base_link");
        robot->InsertEndChild(link);

        // <visual>
        {
            auto* visual = doc.NewElement("visual");
            link->InsertEndChild(visual);

            auto* geometry = doc.NewElement("geometry");
            visual->InsertEndChild(geometry);

            auto* mesh_el = doc.NewElement("mesh");
            std::string mesh_ref = "meshes/" + visual_filename;
            mesh_el->SetAttribute("filename", mesh_ref.c_str());
            geometry->InsertEndChild(mesh_el);
        }

        // <collision>
        if (has_collision) {
            auto* collision = doc.NewElement("collision");
            link->InsertEndChild(collision);

            auto* geometry = doc.NewElement("geometry");
            collision->InsertEndChild(geometry);

            auto* mesh_el = doc.NewElement("mesh");
            std::string mesh_ref = "meshes/" + collision_filename;
            mesh_el->SetAttribute("filename", mesh_ref.c_str());
            geometry->InsertEndChild(mesh_el);
        }

        // <inertial>
        if (asset.physics) {
            const auto& phys = *asset.physics;

            auto* inertial = doc.NewElement("inertial");
            link->InsertEndChild(inertial);

            auto* mass = doc.NewElement("mass");
            mass->SetAttribute("value", ftos(phys.mass).c_str());
            inertial->InsertEndChild(mass);

            auto* origin = doc.NewElement("origin");
            std::string xyz = ftos(phys.center_of_mass.x) + " "
                            + ftos(phys.center_of_mass.y) + " "
                            + ftos(phys.center_of_mass.z);
            origin->SetAttribute("xyz", xyz.c_str());
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

        // Warnings for unsupported features
        if (!asset.materials.empty()) {
            spdlog::warn("URDF exporter: PBR materials dropped (URDF has no material support)");
        }
        if (!asset.lods.empty()) {
            spdlog::warn("URDF exporter: LODs dropped (URDF has no LOD support)");
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_urdf_exporter() {
    return std::make_unique<URDFExporter>();
}

}  // namespace simforge::adapters
