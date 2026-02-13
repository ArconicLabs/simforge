// Programmatic mesh builders and file writers for tests.
// Pure data construction — no external dependencies beyond simforge types.
#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "simforge/core/types.h"

namespace simforge::test {

// ─── Mesh Builders ────────────────────────────────────────────────

// 8v, 12t, watertight, 1m unit cube at origin (0-1 range)
inline Mesh make_cube() {
    Mesh m;
    m.name = "cube";
    m.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    m.faces = {
        // front / back
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        // bottom / top
        {0, 5, 1}, {0, 4, 5},
        {2, 6, 7}, {2, 7, 3},
        // left / right
        {0, 3, 7}, {0, 7, 4},
        {1, 5, 6}, {1, 6, 2},
    };
    m.recompute_bounds();
    return m;
}

// 4v, 4t, watertight, simplest closed mesh
inline Mesh make_tetrahedron() {
    Mesh m;
    m.name = "tetrahedron";
    m.vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f},
    };
    m.faces = {
        {0, 1, 2},
        {0, 1, 3},
        {1, 2, 3},
        {0, 2, 3},
    };
    m.recompute_bounds();
    return m;
}

// 8v, 10t, NOT watertight (missing top two triangles)
inline Mesh make_open_box() {
    Mesh m;
    m.name = "open_box";
    m.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    m.faces = {
        // bottom
        {0, 1, 2}, {0, 2, 3},
        // front / back
        {0, 5, 1}, {0, 4, 5},
        {2, 6, 7}, {2, 7, 3},
        // left / right
        {0, 3, 7}, {0, 7, 4},
        {1, 5, 6}, {1, 6, 2},
        // top intentionally omitted
    };
    m.recompute_bounds();
    return m;
}

// 3v, 1t, open, minimal mesh
inline Mesh make_single_triangle() {
    Mesh m;
    m.name = "triangle";
    m.vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
    };
    m.faces = {{0, 1, 2}};
    m.recompute_bounds();
    return m;
}

// Mesh with a degenerate face (duplicate vertex indices)
inline Mesh make_degenerate_mesh() {
    Mesh m;
    m.name = "degenerate";
    m.vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
    };
    m.faces = {
        {0, 1, 2},
        {0, 0, 1},  // degenerate: v0 == v1
    };
    m.recompute_bounds();
    return m;
}

// Cube at 0.0005m scale (triggers scale_sanity min_extent)
inline Mesh make_tiny_mesh() {
    Mesh m;
    m.name = "tiny";
    const float s = 0.0005f;
    m.vertices = {
        {0, 0, 0},   {s, 0, 0},   {s, s, 0},   {0, s, 0},
        {0, 0, s},   {s, 0, s},   {s, s, s},   {0, s, s},
    };
    m.faces = {
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 5, 1}, {0, 4, 5},
        {2, 6, 7}, {2, 7, 3},
        {0, 3, 7}, {0, 7, 4},
        {1, 5, 6}, {1, 6, 2},
    };
    m.recompute_bounds();
    return m;
}

// Cube at 200m scale (triggers scale_sanity max_extent)
inline Mesh make_large_mesh() {
    Mesh m;
    m.name = "large";
    const float s = 200.0f;
    m.vertices = {
        {0, 0, 0},   {s, 0, 0},   {s, s, 0},   {0, s, 0},
        {0, 0, s},   {s, 0, s},   {s, s, s},   {0, s, s},
    };
    m.faces = {
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 5, 1}, {0, 4, 5},
        {2, 6, 7}, {2, 7, 3},
        {0, 3, 7}, {0, 7, 4},
        {1, 5, 6}, {1, 6, 2},
    };
    m.recompute_bounds();
    return m;
}

// ─── File Writers ─────────────────────────────────────────────────

// Write a mesh to OBJ format at the given path.
inline void write_obj(const Mesh& mesh, const std::filesystem::path& path) {
    std::ofstream out(path);
    out << "# test mesh: " << mesh.name << "\n";
    for (const auto& v : mesh.vertices) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (const auto& f : mesh.faces) {
        // OBJ is 1-based
        out << "f " << (f.v0 + 1) << " " << (f.v1 + 1) << " " << (f.v2 + 1) << "\n";
    }
}

// Write a mesh to binary STL format at the given path.
inline void write_binary_stl(const Mesh& mesh, const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary);

    // 80-byte header
    char header[80] = {};
    out.write(header, 80);

    // Triangle count
    auto tri_count = static_cast<uint32_t>(mesh.faces.size());
    out.write(reinterpret_cast<const char*>(&tri_count), 4);

    for (const auto& f : mesh.faces) {
        const auto& a = mesh.vertices[f.v0];
        const auto& b = mesh.vertices[f.v1];
        const auto& c = mesh.vertices[f.v2];

        // Compute face normal (cross product of edges)
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
}

}  // namespace simforge::test
