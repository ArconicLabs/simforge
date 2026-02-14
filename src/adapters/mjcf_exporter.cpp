// MJCF exporter — MuJoCo XML format with external STL meshes.
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

class MJCFExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "mjcf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::MJCF};
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        if (asset.meshes.empty()) {
            spdlog::error("MJCF exporter: no meshes in asset '{}'", asset.name);
            return false;
        }

        // Create assets/ subdirectory (MuJoCo convention)
        auto asset_dir = output_path.parent_path() / "assets";
        fs::create_directories(asset_dir);

        // Write visual mesh as STL
        std::string visual_filename = asset.name + ".stl";
        auto visual_path = asset_dir / visual_filename;
        if (!write_binary_stl(asset.meshes[0], visual_path)) {
            spdlog::error("MJCF exporter: failed to write visual mesh");
            return false;
        }

        // Write collision mesh as STL
        std::string collision_filename;
        bool has_collision = asset.collision && !asset.collision->hulls.empty();
        if (has_collision) {
            collision_filename = asset.name + "_collision.stl";
            auto collision_path = asset_dir / collision_filename;
            // Merge hulls into first hull for STL (STL doesn't support groups)
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
            if (!write_binary_stl(merged, collision_path)) {
                spdlog::error("MJCF exporter: failed to write collision mesh");
                return false;
            }
        }

        // Build XML
        tinyxml2::XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());

        auto* mujoco = doc.NewElement("mujoco");
        mujoco->SetAttribute("model", asset.name.c_str());
        doc.InsertEndChild(mujoco);

        // <asset> — declare mesh files
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

        // <worldbody>
        auto* worldbody = doc.NewElement("worldbody");
        mujoco->InsertEndChild(worldbody);

        auto* body = doc.NewElement("body");
        body->SetAttribute("name", asset.name.c_str());
        worldbody->InsertEndChild(body);

        // Visual geom
        auto* vis_geom = doc.NewElement("geom");
        vis_geom->SetAttribute("type", "mesh");
        vis_geom->SetAttribute("mesh", (asset.name + "_visual").c_str());
        vis_geom->SetAttribute("contype", "0");
        vis_geom->SetAttribute("conaffinity", "0");
        body->InsertEndChild(vis_geom);

        // Collision geom
        if (has_collision) {
            auto* coll_geom = doc.NewElement("geom");
            coll_geom->SetAttribute("type", "mesh");
            coll_geom->SetAttribute("mesh", (asset.name + "_collision").c_str());
            body->InsertEndChild(coll_geom);
        }

        // <inertial>
        if (asset.physics) {
            const auto& phys = *asset.physics;
            auto* inertial = doc.NewElement("inertial");
            std::string pos = ftos(phys.center_of_mass.x) + " "
                            + ftos(phys.center_of_mass.y) + " "
                            + ftos(phys.center_of_mass.z);
            inertial->SetAttribute("pos", pos.c_str());
            inertial->SetAttribute("mass", ftos(phys.mass).c_str());
            // MuJoCo inertia: diagonal of the inertia matrix
            std::string diaginertia = ftos(phys.inertia_diagonal.x) + " "
                                    + ftos(phys.inertia_diagonal.y) + " "
                                    + ftos(phys.inertia_diagonal.z);
            inertial->SetAttribute("diaginertia", diaginertia.c_str());
            body->InsertEndChild(inertial);
        }

        // Warnings for unsupported features
        if (!asset.materials.empty()) {
            spdlog::warn("MJCF exporter: PBR materials dropped (MJCF has limited material support)");
        }
        if (!asset.lods.empty()) {
            spdlog::warn("MJCF exporter: LODs dropped (MJCF has no LOD support)");
        }

        return doc.SaveFile(output_path.string().c_str()) == tinyxml2::XML_SUCCESS;
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_mjcf_exporter() {
    return std::make_unique<MJCFExporter>();
}

}  // namespace simforge::adapters
