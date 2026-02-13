// Tests for builtin OBJ and STL importers via round-trip through temp files.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>

#include <spdlog/spdlog.h>

#include "simforge/adapters/adapter.h"
#include "test_helpers.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;

static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    return true;
}();

namespace {

// RAII temp directory that cleans up on destruction
struct TempDir {
    std::filesystem::path path;

    TempDir() {
        path = std::filesystem::temp_directory_path() / "simforge_test_XXXXXX";
        // Create a unique directory
        path = std::filesystem::temp_directory_path()
            / ("simforge_test_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

}  // namespace

// ─── OBJ Importer Tests ──────────────────────────────────────────

TEST_CASE("OBJ import — cube round-trip", "[adapters]") {
    TempDir tmp;
    auto cube = test::make_cube();
    auto obj_path = tmp.path / "cube.obj";
    test::write_obj(cube, obj_path);

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    REQUIRE(importer != nullptr);

    auto meshes = importer->import(obj_path);
    REQUIRE(meshes.size() == 1);

    auto& m = meshes[0];
    REQUIRE(m.vertex_count() == 8);
    REQUIRE(m.triangle_count() == 12);

    m.recompute_bounds();
    REQUIRE(m.bounds.min.x == 0.0f);
    REQUIRE(m.bounds.max.x == 1.0f);
    REQUIRE(m.is_watertight());
}

TEST_CASE("OBJ import — negative indices", "[adapters]") {
    TempDir tmp;
    auto obj_path = tmp.path / "neg_idx.obj";

    // Write OBJ with negative (relative) indices
    {
        std::ofstream out(obj_path);
        out << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0.5 1 0\n"
            << "f -3 -2 -1\n";  // relative: refers to last 3 vertices
    }

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    REQUIRE(importer != nullptr);

    auto meshes = importer->import(obj_path);
    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].vertex_count() == 3);
    REQUIRE(meshes[0].triangle_count() == 1);

    // Verify the indices resolved correctly (should be 0, 1, 2)
    auto& f = meshes[0].faces[0];
    REQUIRE(f.v0 == 0);
    REQUIRE(f.v1 == 1);
    REQUIRE(f.v2 == 2);
}

TEST_CASE("OBJ import — quad triangulation", "[adapters]") {
    TempDir tmp;
    auto obj_path = tmp.path / "quad.obj";

    // Write OBJ with a quad face
    {
        std::ofstream out(obj_path);
        out << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 1 1 0\n"
            << "v 0 1 0\n"
            << "f 1 2 3 4\n";  // quad → should become 2 triangles
    }

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    REQUIRE(importer != nullptr);

    auto meshes = importer->import(obj_path);
    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].vertex_count() == 4);
    REQUIRE(meshes[0].triangle_count() == 2);
}

TEST_CASE("OBJ import — vertex/normal/texcoord format", "[adapters]") {
    TempDir tmp;
    auto obj_path = tmp.path / "vnt.obj";

    // Write OBJ with v/vt/vn format
    {
        std::ofstream out(obj_path);
        out << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0.5 1 0\n"
            << "vt 0 0\n"
            << "vt 1 0\n"
            << "vt 0.5 1\n"
            << "vn 0 0 1\n"
            << "f 1/1/1 2/2/1 3/3/1\n";
    }

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    auto meshes = importer->import(obj_path);

    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].vertex_count() == 3);
    REQUIRE(meshes[0].triangle_count() == 1);
}

// ─── STL Importer Tests ──────────────────────────────────────────

TEST_CASE("STL import — binary cube round-trip", "[adapters]") {
    TempDir tmp;
    auto cube = test::make_cube();
    auto stl_path = tmp.path / "cube.stl";
    test::write_binary_stl(cube, stl_path);

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::STL);
    REQUIRE(importer != nullptr);

    auto meshes = importer->import(stl_path);
    REQUIRE(meshes.size() == 1);

    auto& m = meshes[0];
    // STL stores 3 vertices per triangle (no sharing), so 12*3 = 36
    REQUIRE(m.triangle_count() == 12);
    REQUIRE(m.vertex_count() == 36);

    m.recompute_bounds();
    REQUIRE(m.bounds.min.x == 0.0f);
    REQUIRE(m.bounds.max.x == 1.0f);
}

TEST_CASE("STL import — tetrahedron round-trip", "[adapters]") {
    TempDir tmp;
    auto tet = test::make_tetrahedron();
    auto stl_path = tmp.path / "tet.stl";
    test::write_binary_stl(tet, stl_path);

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::STL);
    auto meshes = importer->import(stl_path);

    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].triangle_count() == 4);
}

// ─── Adapter Priority ────────────────────────────────────────────

TEST_CASE("Adapter priority — builtin importers registered", "[adapters]") {
    auto& mgr = AdapterManager::instance();

    // Both OBJ and STL should have importers
    REQUIRE(mgr.find_importer(SourceFormat::OBJ) != nullptr);
    REQUIRE(mgr.find_importer(SourceFormat::STL) != nullptr);

    auto importers = mgr.list_importers();
    REQUIRE(importers.size() >= 2);
}

#ifdef SIMFORGE_HAS_ASSIMP
TEST_CASE("Adapter priority — Assimp takes precedence", "[adapters]") {
    auto& mgr = AdapterManager::instance();

    // When Assimp is compiled in, it should be the preferred OBJ importer
    // (registered last → returned by find_importer due to reverse lookup)
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    REQUIRE(importer != nullptr);
    REQUIRE(importer->name() == "assimp");
}
#endif
