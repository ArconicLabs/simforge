// GLTF exporter — binary GLB or text GLTF+bin via tinygltf.
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

#include "simforge/adapters/exporters.h"

#include <spdlog/spdlog.h>

namespace simforge::adapters {

namespace {

class GLTFExporter : public MeshExporter {
public:
    [[nodiscard]] std::string name() const override { return "gltf"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {SourceFormat::GLTF, SourceFormat::GLB};
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        if (asset.meshes.empty()) {
            spdlog::error("GLTF exporter: no meshes in asset '{}'", asset.name);
            return false;
        }

        tinygltf::Model model;
        model.asset.version = "2.0";
        model.asset.generator = "SimForge";

        // Build from the first mesh
        const auto& mesh = asset.meshes[0];

        // ── Buffer: pack positions, normals, UVs, indices ──
        tinygltf::Buffer buffer;

        // Positions (vec3 float)
        size_t pos_offset = buffer.data.size();
        size_t pos_size = mesh.vertices.size() * sizeof(float) * 3;
        buffer.data.resize(pos_offset + pos_size);
        for (size_t i = 0; i < mesh.vertices.size(); i++) {
            float v[3] = {mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z};
            std::memcpy(buffer.data.data() + pos_offset + i * 12, v, 12);
        }

        // Normals (vec3 float)
        size_t norm_offset = buffer.data.size();
        size_t norm_size = 0;
        if (!mesh.normals.empty()) {
            norm_size = mesh.normals.size() * sizeof(float) * 3;
            buffer.data.resize(norm_offset + norm_size);
            for (size_t i = 0; i < mesh.normals.size(); i++) {
                float n[3] = {mesh.normals[i].x, mesh.normals[i].y, mesh.normals[i].z};
                std::memcpy(buffer.data.data() + norm_offset + i * 12, n, 12);
            }
        }

        // UVs (vec2 float)
        size_t uv_offset = buffer.data.size();
        size_t uv_size = 0;
        if (!mesh.uvs.empty()) {
            uv_size = mesh.uvs.size() * sizeof(float) * 2;
            buffer.data.resize(uv_offset + uv_size);
            for (size_t i = 0; i < mesh.uvs.size(); i++) {
                float uv[2] = {mesh.uvs[i].x, mesh.uvs[i].y};
                std::memcpy(buffer.data.data() + uv_offset + i * 8, uv, 8);
            }
        }

        // Indices (uint32)
        size_t idx_offset = buffer.data.size();
        size_t idx_count = mesh.faces.size() * 3;
        size_t idx_size = idx_count * sizeof(uint32_t);
        buffer.data.resize(idx_offset + idx_size);
        for (size_t i = 0; i < mesh.faces.size(); i++) {
            uint32_t indices[3] = {mesh.faces[i].v0, mesh.faces[i].v1, mesh.faces[i].v2};
            std::memcpy(buffer.data.data() + idx_offset + i * 12, indices, 12);
        }

        int buffer_idx = static_cast<int>(model.buffers.size());
        model.buffers.push_back(std::move(buffer));

        // ── Buffer Views ──
        auto add_buffer_view = [&](size_t offset, size_t length, int target) -> int {
            tinygltf::BufferView bv;
            bv.buffer = buffer_idx;
            bv.byteOffset = offset;
            bv.byteLength = length;
            bv.target = target;
            int idx = static_cast<int>(model.bufferViews.size());
            model.bufferViews.push_back(bv);
            return idx;
        };

        int pos_bv = add_buffer_view(pos_offset, pos_size, TINYGLTF_TARGET_ARRAY_BUFFER);
        int norm_bv = -1;
        if (norm_size > 0) {
            norm_bv = add_buffer_view(norm_offset, norm_size, TINYGLTF_TARGET_ARRAY_BUFFER);
        }
        int uv_bv = -1;
        if (uv_size > 0) {
            uv_bv = add_buffer_view(uv_offset, uv_size, TINYGLTF_TARGET_ARRAY_BUFFER);
        }
        int idx_bv = add_buffer_view(idx_offset, idx_size, TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER);

        // ── Accessors ──
        // Compute min/max for positions
        float min_pos[3] = {1e30f, 1e30f, 1e30f};
        float max_pos[3] = {-1e30f, -1e30f, -1e30f};
        for (const auto& v : mesh.vertices) {
            min_pos[0] = std::min(min_pos[0], v.x);
            min_pos[1] = std::min(min_pos[1], v.y);
            min_pos[2] = std::min(min_pos[2], v.z);
            max_pos[0] = std::max(max_pos[0], v.x);
            max_pos[1] = std::max(max_pos[1], v.y);
            max_pos[2] = std::max(max_pos[2], v.z);
        }

        auto add_accessor = [&](int bv_idx, int component_type, int type,
                                size_t count) -> int {
            tinygltf::Accessor acc;
            acc.bufferView = bv_idx;
            acc.byteOffset = 0;
            acc.componentType = component_type;
            acc.type = type;
            acc.count = count;
            int idx = static_cast<int>(model.accessors.size());
            model.accessors.push_back(acc);
            return idx;
        };

        // Position accessor with min/max
        int pos_acc = add_accessor(pos_bv, TINYGLTF_COMPONENT_TYPE_FLOAT,
                                   TINYGLTF_TYPE_VEC3, mesh.vertices.size());
        model.accessors[pos_acc].minValues = {min_pos[0], min_pos[1], min_pos[2]};
        model.accessors[pos_acc].maxValues = {max_pos[0], max_pos[1], max_pos[2]};

        int norm_acc = -1;
        if (norm_bv >= 0) {
            norm_acc = add_accessor(norm_bv, TINYGLTF_COMPONENT_TYPE_FLOAT,
                                    TINYGLTF_TYPE_VEC3, mesh.normals.size());
        }

        int uv_acc = -1;
        if (uv_bv >= 0) {
            uv_acc = add_accessor(uv_bv, TINYGLTF_COMPONENT_TYPE_FLOAT,
                                  TINYGLTF_TYPE_VEC2, mesh.uvs.size());
        }

        int idx_acc = add_accessor(idx_bv, TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT,
                                   TINYGLTF_TYPE_SCALAR, idx_count);

        // ── Material (PBR metallic-roughness) ──
        tinygltf::Material mat;
        mat.name = "default";
        if (!asset.materials.empty()) {
            const auto& pbr = asset.materials[0];
            mat.name = pbr.name.empty() ? "material_0" : pbr.name;
            mat.pbrMetallicRoughness.baseColorFactor = {
                pbr.base_color.x, pbr.base_color.y, pbr.base_color.z, pbr.opacity
            };
            mat.pbrMetallicRoughness.metallicFactor = pbr.metallic;
            mat.pbrMetallicRoughness.roughnessFactor = pbr.roughness;
        } else {
            mat.pbrMetallicRoughness.baseColorFactor = {0.8, 0.8, 0.8, 1.0};
            mat.pbrMetallicRoughness.metallicFactor = 0.0;
            mat.pbrMetallicRoughness.roughnessFactor = 0.5;
        }
        int mat_idx = static_cast<int>(model.materials.size());
        model.materials.push_back(mat);

        // ── Mesh + Primitive ──
        tinygltf::Primitive prim;
        prim.attributes["POSITION"] = pos_acc;
        if (norm_acc >= 0) prim.attributes["NORMAL"] = norm_acc;
        if (uv_acc >= 0) prim.attributes["TEXCOORD_0"] = uv_acc;
        prim.indices = idx_acc;
        prim.material = mat_idx;
        prim.mode = TINYGLTF_MODE_TRIANGLES;

        tinygltf::Mesh gltf_mesh;
        gltf_mesh.name = mesh.name.empty() ? asset.name : mesh.name;
        gltf_mesh.primitives.push_back(prim);
        int mesh_idx = static_cast<int>(model.meshes.size());
        model.meshes.push_back(gltf_mesh);

        // ── Node ──
        tinygltf::Node node;
        node.name = asset.name;
        node.mesh = mesh_idx;
        int node_idx = static_cast<int>(model.nodes.size());
        model.nodes.push_back(node);

        // ── Scene ──
        tinygltf::Scene scene;
        scene.name = "Scene";
        scene.nodes.push_back(node_idx);
        model.scenes.push_back(scene);
        model.defaultScene = 0;

        // Warnings for unsupported features
        if (asset.is_articulated()) {
            spdlog::warn("GLTF exporter: articulation data dropped (GLTF has no articulation support)");
        }
        if (asset.physics) {
            spdlog::warn("GLTF exporter: physics data dropped (GLTF has no physics support)");
        }
        if (asset.collision) {
            spdlog::warn("GLTF exporter: collision data dropped (GLTF has no collision support)");
        }
        if (!asset.lods.empty()) {
            spdlog::warn("GLTF exporter: LODs dropped (Phase 5)");
        }

        // Write
        tinygltf::TinyGLTF writer;
        std::string err;
        std::string warn;

        auto ext = output_path.extension().string();
        bool is_binary = (ext == ".glb");

        bool ok = writer.WriteGltfSceneToFile(&model, output_path.string(),
                                               true,   // embedImages
                                               true,   // embedBuffers
                                               true,   // prettyPrint
                                               is_binary);
        if (!ok) {
            spdlog::error("GLTF exporter: write failed for {}", output_path.string());
        }
        return ok;
    }
};

}  // namespace

std::unique_ptr<MeshExporter> make_gltf_exporter() {
    return std::make_unique<GLTFExporter>();
}

}  // namespace simforge::adapters
