#include "simforge/adapters/adapter.h"
#include "simforge/adapters/exporters.h"

#include <fstream>

#include <spdlog/spdlog.h>

#ifdef SIMFORGE_HAS_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#endif

namespace simforge::adapters {

// ─── Assimp Importer ───────────────────────────────────────────────

#ifdef SIMFORGE_HAS_ASSIMP

class AssimpImporter : public MeshImporter {
public:
    [[nodiscard]] std::string name() const override { return "assimp"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return {
            SourceFormat::OBJ, SourceFormat::FBX, SourceFormat::GLTF,
            SourceFormat::GLB, SourceFormat::STL, SourceFormat::DAE,
        };
    }

    std::vector<Mesh> import(const fs::path& path) override {
        Assimp::Importer importer;

        unsigned int flags =
            aiProcess_Triangulate |
            aiProcess_GenNormals |
            aiProcess_JoinIdenticalVertices |
            aiProcess_SortByPType |
            aiProcess_RemoveRedundantMaterials |
            aiProcess_OptimizeMeshes;

        const aiScene* scene = importer.ReadFile(path.string(), flags);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error(
                "Assimp import failed: " + std::string(importer.GetErrorString()));
        }

        std::vector<Mesh> meshes;
        process_node(scene->mRootNode, scene, meshes);
        return meshes;
    }

private:
    void process_node(const aiNode* node, const aiScene* scene,
                      std::vector<Mesh>& meshes) {
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            const aiMesh* ai_mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(convert_mesh(ai_mesh));
        }
        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            process_node(node->mChildren[i], scene, meshes);
        }
    }

    Mesh convert_mesh(const aiMesh* ai_mesh) {
        Mesh mesh;
        mesh.name = ai_mesh->mName.C_Str();

        // Vertices
        mesh.vertices.reserve(ai_mesh->mNumVertices);
        for (unsigned int i = 0; i < ai_mesh->mNumVertices; i++) {
            mesh.vertices.push_back({
                ai_mesh->mVertices[i].x,
                ai_mesh->mVertices[i].y,
                ai_mesh->mVertices[i].z
            });
        }

        // Normals
        if (ai_mesh->HasNormals()) {
            mesh.normals.reserve(ai_mesh->mNumVertices);
            for (unsigned int i = 0; i < ai_mesh->mNumVertices; i++) {
                mesh.normals.push_back({
                    ai_mesh->mNormals[i].x,
                    ai_mesh->mNormals[i].y,
                    ai_mesh->mNormals[i].z
                });
            }
        }

        // Faces (already triangulated by flags)
        mesh.faces.reserve(ai_mesh->mNumFaces);
        for (unsigned int i = 0; i < ai_mesh->mNumFaces; i++) {
            const auto& face = ai_mesh->mFaces[i];
            if (face.mNumIndices == 3) {
                mesh.faces.push_back({
                    face.mIndices[0],
                    face.mIndices[1],
                    face.mIndices[2]
                });
            }
        }

        // UVs (first channel only)
        if (ai_mesh->HasTextureCoords(0)) {
            mesh.uvs.reserve(ai_mesh->mNumVertices);
            for (unsigned int i = 0; i < ai_mesh->mNumVertices; i++) {
                mesh.uvs.push_back({
                    ai_mesh->mTextureCoords[0][i].x,
                    ai_mesh->mTextureCoords[0][i].y,
                    0.0f
                });
            }
        }

        return mesh;
    }
};

#endif  // SIMFORGE_HAS_ASSIMP

// ─── STL Importer (minimal, no dependencies) ──────────────────────

class STLImporter : public MeshImporter {
public:
    [[nodiscard]] std::string name() const override { return "stl_builtin"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return { SourceFormat::STL };
    }

    std::vector<Mesh> import(const fs::path& path) override;
};

std::vector<Mesh> STLImporter::import(const fs::path& path) {
    // Detect binary vs ASCII STL
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open " + path.string());

    // Read header (80 bytes) + triangle count (4 bytes)
    char header[80];
    file.read(header, 80);

    uint32_t tri_count = 0;
    file.read(reinterpret_cast<char*>(&tri_count), 4);

    // Sanity check: file size should match binary STL format
    auto file_size = fs::file_size(path);
    auto expected_binary_size = 84 + tri_count * 50;  // header + count + triangles

    Mesh mesh;
    mesh.name = path.stem().string();

    if (file_size == expected_binary_size && tri_count > 0) {
        // Binary STL
        mesh.vertices.reserve(tri_count * 3);
        mesh.normals.reserve(tri_count * 3);
        mesh.faces.reserve(tri_count);

        for (uint32_t t = 0; t < tri_count; t++) {
            float normal[3], v1[3], v2[3], v3[3];
            uint16_t attr;

            file.read(reinterpret_cast<char*>(normal), 12);
            file.read(reinterpret_cast<char*>(v1), 12);
            file.read(reinterpret_cast<char*>(v2), 12);
            file.read(reinterpret_cast<char*>(v3), 12);
            file.read(reinterpret_cast<char*>(&attr), 2);

            uint32_t base = static_cast<uint32_t>(mesh.vertices.size());

            mesh.vertices.push_back({v1[0], v1[1], v1[2]});
            mesh.vertices.push_back({v2[0], v2[1], v2[2]});
            mesh.vertices.push_back({v3[0], v3[1], v3[2]});

            Vec3 n{normal[0], normal[1], normal[2]};
            mesh.normals.push_back(n);
            mesh.normals.push_back(n);
            mesh.normals.push_back(n);

            mesh.faces.push_back({base, base + 1, base + 2});
        }
    } else {
        // ASCII STL - punt to Assimp or throw
        throw std::runtime_error("ASCII STL not yet supported by builtin importer");
    }

    return {mesh};
}

// ─── OBJ Importer (minimal, no dependencies) ──────────────────────

class OBJImporter : public MeshImporter {
public:
    [[nodiscard]] std::string name() const override { return "obj_builtin"; }

    [[nodiscard]] std::vector<SourceFormat> supported_formats() const override {
        return { SourceFormat::OBJ };
    }

    std::vector<Mesh> import(const fs::path& path) override;
};

std::vector<Mesh> OBJImporter::import(const fs::path& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open " + path.string());

    Mesh mesh;
    mesh.name = path.stem().string();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            float x, y, z;
            iss >> x >> y >> z;
            mesh.vertices.push_back({x, y, z});
        } else if (prefix == "vn") {
            float x, y, z;
            iss >> x >> y >> z;
            mesh.normals.push_back({x, y, z});
        } else if (prefix == "f") {
            // Handle f v1 v2 v3 and f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
            std::vector<uint32_t> indices;
            std::string token;
            while (iss >> token) {
                auto slash = token.find('/');
                auto idx_str = (slash != std::string::npos)
                    ? token.substr(0, slash) : token;
                int idx = std::stoi(idx_str);
                // OBJ indices are 1-based; negative means relative to current count
                if (idx > 0) {
                    indices.push_back(static_cast<uint32_t>(idx - 1));
                } else if (idx < 0) {
                    indices.push_back(static_cast<uint32_t>(
                        static_cast<int>(mesh.vertices.size()) + idx));
                }
            }
            // Triangulate fan-style for quads+
            for (size_t i = 1; i + 1 < indices.size(); i++) {
                mesh.faces.push_back({indices[0], indices[i], indices[i + 1]});
            }
        }
    }

    if (file.bad()) {
        throw std::runtime_error("Read error while parsing OBJ: " + path.string());
    }

    return {mesh};
}

// ─── Adapter forward declarations ─────────────────────────────────

std::unique_ptr<LODGenerator> make_meshoptimizer_decimator();
std::unique_ptr<CollisionGenerator> make_primitive_fitter();
std::unique_ptr<ArticulatedImporter> make_urdf_importer();
std::unique_ptr<ArticulatedImporter> make_mjcf_importer();

#ifdef SIMFORGE_HAS_COACD
std::unique_ptr<CollisionGenerator> make_coacd_generator();
#endif

// ─── Adapter Registration ──────────────────────────────────────────

/// Called once at startup to register all built-in adapters.
/// Safe to call multiple times — subsequent calls are no-ops.
void register_builtin_adapters() {
    static bool registered = false;
    if (registered) return;
    registered = true;

    auto& mgr = AdapterManager::instance();

    // Always register zero-dependency importers
    mgr.register_importer(std::make_unique<STLImporter>());
    mgr.register_importer(std::make_unique<OBJImporter>());

#ifdef SIMFORGE_HAS_ASSIMP
    // Assimp handles many formats and takes priority for those it supports
    mgr.register_importer(std::make_unique<AssimpImporter>());
#endif

    spdlog::info("Registered {} importer(s)", mgr.list_importers().size());

    // Export adapters
    mgr.register_exporter(make_usda_exporter());
    mgr.register_exporter(make_urdf_exporter());
    mgr.register_exporter(make_mjcf_exporter());
    mgr.register_exporter(make_gltf_exporter());

    spdlog::info("Registered {} exporter(s)", mgr.list_exporters().size());

    // LOD generator (always available, zero deps)
    mgr.register_lod_generator(make_meshoptimizer_decimator());

    // Collision generators
    mgr.register_collision_generator(make_primitive_fitter());

#ifdef SIMFORGE_HAS_COACD
    mgr.register_collision_generator(make_coacd_generator());
#endif

    spdlog::info("Registered collision + LOD adapters");

    // Articulated importers
    mgr.register_articulated_importer(make_urdf_importer());
    mgr.register_articulated_importer(make_mjcf_importer());
}

}  // namespace simforge::adapters
