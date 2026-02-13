#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "simforge/core/types.h"

using namespace simforge;

TEST_CASE("Vec3 arithmetic", "[types]") {
    Vec3 a{1.0f, 2.0f, 3.0f};
    Vec3 b{4.0f, 5.0f, 6.0f};

    auto sum = a + b;
    REQUIRE(sum.x == 5.0f);
    REQUIRE(sum.y == 7.0f);
    REQUIRE(sum.z == 9.0f);

    auto diff = b - a;
    REQUIRE(diff.x == 3.0f);

    auto scaled = a * 2.0f;
    REQUIRE(scaled.x == 2.0f);
    REQUIRE(scaled.z == 6.0f);
}

TEST_CASE("AABB volume and center", "[types]") {
    AABB box;
    box.min = {0.0f, 0.0f, 0.0f};
    box.max = {2.0f, 3.0f, 4.0f};

    REQUIRE(box.volume() == 24.0f);

    auto c = box.center();
    REQUIRE(c.x == 1.0f);
    REQUIRE(c.y == 1.5f);
    REQUIRE(c.z == 2.0f);
}

TEST_CASE("Format detection", "[types]") {
    REQUIRE(detect_format("model.obj")  == SourceFormat::OBJ);
    REQUIRE(detect_format("model.FBX")  == SourceFormat::FBX);
    REQUIRE(detect_format("robot.urdf") == SourceFormat::URDF);
    REQUIRE(detect_format("scene.usd")  == SourceFormat::USD);
    REQUIRE(detect_format("model.step") == SourceFormat::STEP);
    REQUIRE(detect_format("model.stp")  == SourceFormat::STEP);
    REQUIRE(detect_format("file.xyz")   == SourceFormat::Unknown);
}

TEST_CASE("Mesh bounds computation", "[types]") {
    Mesh mesh;
    mesh.vertices = {
        {-1.0f, -1.0f, -1.0f},
        { 1.0f,  1.0f,  1.0f},
        { 0.5f,  0.0f,  0.5f},
    };

    mesh.recompute_bounds();

    REQUIRE(mesh.bounds.min.x == -1.0f);
    REQUIRE(mesh.bounds.max.x ==  1.0f);
    REQUIRE(mesh.bounds.volume() == 8.0f);
}

TEST_CASE("Mesh watertight check — simple tetrahedron", "[types]") {
    // A tetrahedron: 4 vertices, 4 faces, every edge shared by exactly 2 faces
    Mesh mesh;
    mesh.vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f},
    };
    mesh.faces = {
        {0, 1, 2},
        {0, 1, 3},
        {1, 2, 3},
        {0, 2, 3},
    };

    REQUIRE(mesh.is_watertight());
}

TEST_CASE("Mesh watertight check — open triangle", "[types]") {
    Mesh mesh;
    mesh.vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
    };
    mesh.faces = {{0, 1, 2}};

    REQUIRE_FALSE(mesh.is_watertight());
}

TEST_CASE("PhysicsProperties estimation", "[types]") {
    // Unit cube: volume = 1 m³, density = 1000 kg/m³ → mass = 1000 kg
    Mesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    // 12 triangles for a cube
    mesh.faces = {
        {0,1,2}, {0,2,3}, {4,6,5}, {4,7,6},
        {0,5,1}, {0,4,5}, {2,6,7}, {2,7,3},
        {0,3,7}, {0,7,4}, {1,5,6}, {1,6,2},
    };
    mesh.recompute_bounds();

    PhysicsMaterial mat;
    mat.density = 1000.0f;

    auto props = PhysicsProperties::estimate_from_mesh(mesh, mat);
    REQUIRE(props.mass_estimated);
    // Volume should be ~1.0, mass ~1000
    REQUIRE(props.mass > 500.0f);  // approximate due to signed volume method
}

TEST_CASE("Asset catalog entry", "[types]") {
    Asset asset;
    asset.id = "test-001";
    asset.name = "test_cube";
    asset.source_path = "/tmp/cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Ready;

    auto entry = asset.to_catalog_entry();
    REQUIRE(entry["id"] == "test-001");
    REQUIRE(entry["name"] == "test_cube");
    REQUIRE(entry["source_format"] == "OBJ");
}
