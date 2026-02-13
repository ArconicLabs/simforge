// Shared mesh file writers for OBJ and binary STL export.
#include "simforge/adapters/mesh_writer.h"

#include <fstream>

#include <spdlog/spdlog.h>

namespace simforge::adapters {

bool write_obj(const Mesh& mesh, const fs::path& path) {
    std::ofstream out(path);
    if (!out) {
        spdlog::error("mesh_writer: cannot open {} for writing", path.string());
        return false;
    }

    out << "# SimForge export: " << mesh.name << "\n";
    for (const auto& v : mesh.vertices) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (const auto& n : mesh.normals) {
        out << "vn " << n.x << " " << n.y << " " << n.z << "\n";
    }
    for (const auto& f : mesh.faces) {
        // OBJ is 1-based
        out << "f " << (f.v0 + 1) << " " << (f.v1 + 1) << " " << (f.v2 + 1) << "\n";
    }

    return out.good();
}

bool write_obj_multi(const std::vector<Mesh>& meshes, const fs::path& path) {
    std::ofstream out(path);
    if (!out) {
        spdlog::error("mesh_writer: cannot open {} for writing", path.string());
        return false;
    }

    out << "# SimForge export: multi-hull collision mesh\n";

    uint32_t vertex_offset = 0;
    for (size_t i = 0; i < meshes.size(); i++) {
        const auto& mesh = meshes[i];
        out << "g hull_" << i << "\n";
        for (const auto& v : mesh.vertices) {
            out << "v " << v.x << " " << v.y << " " << v.z << "\n";
        }
        for (const auto& f : mesh.faces) {
            out << "f " << (f.v0 + vertex_offset + 1)
                << " " << (f.v1 + vertex_offset + 1)
                << " " << (f.v2 + vertex_offset + 1) << "\n";
        }
        vertex_offset += static_cast<uint32_t>(mesh.vertices.size());
    }

    return out.good();
}

bool write_binary_stl(const Mesh& mesh, const fs::path& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        spdlog::error("mesh_writer: cannot open {} for writing", path.string());
        return false;
    }

    // 80-byte header
    char header[80] = {};
    out.write(header, 80);

    auto tri_count = static_cast<uint32_t>(mesh.faces.size());
    out.write(reinterpret_cast<const char*>(&tri_count), 4);

    for (const auto& f : mesh.faces) {
        const auto& a = mesh.vertices[f.v0];
        const auto& b = mesh.vertices[f.v1];
        const auto& c = mesh.vertices[f.v2];

        // Face normal via cross product
        Vec3 e1 = b - a;
        Vec3 e2 = c - a;
        float nx = e1.y * e2.z - e1.z * e2.y;
        float ny = e1.z * e2.x - e1.x * e2.z;
        float nz = e1.x * e2.y - e1.y * e2.x;

        float normal[3] = {nx, ny, nz};
        out.write(reinterpret_cast<const char*>(normal), 12);

        float v1[3] = {a.x, a.y, a.z};
        float v2[3] = {b.x, b.y, b.z};
        float v3[3] = {c.x, c.y, c.z};
        out.write(reinterpret_cast<const char*>(v1), 12);
        out.write(reinterpret_cast<const char*>(v2), 12);
        out.write(reinterpret_cast<const char*>(v3), 12);

        uint16_t attr = 0;
        out.write(reinterpret_cast<const char*>(&attr), 2);
    }

    return out.good();
}

}  // namespace simforge::adapters
