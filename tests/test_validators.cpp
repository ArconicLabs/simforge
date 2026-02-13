#include <catch2/catch_test_macros.hpp>

#include "simforge/validators/validator.h"

using namespace simforge;

TEST_CASE("ValidatorRegistry has built-ins", "[validators]") {
    auto& reg = ValidatorRegistry::instance();
    auto available = reg.available();

    REQUIRE(available.size() >= 5);
}

TEST_CASE("MeshIntegrityValidator — clean mesh", "[validators]") {
    Asset asset;
    Mesh mesh;
    mesh.name = "clean";
    mesh.vertices = {{0,0,0}, {1,0,0}, {0,1,0}};
    mesh.faces = {{0, 1, 2}};
    asset.meshes.push_back(mesh);

    MeshIntegrityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE(results[0].passed);
}

TEST_CASE("MeshIntegrityValidator — degenerate triangle", "[validators]") {
    Asset asset;
    Mesh mesh;
    mesh.name = "degenerate";
    mesh.vertices = {{0,0,0}, {1,0,0}, {0,1,0}};
    mesh.faces = {{0, 0, 1}};  // degenerate: v0 == v1
    asset.meshes.push_back(mesh);

    MeshIntegrityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("ScaleSanityValidator — reasonable scale", "[validators]") {
    Asset asset;
    Mesh mesh;
    mesh.name = "chair";
    mesh.bounds.min = {-0.3f, 0.0f, -0.3f};
    mesh.bounds.max = { 0.3f, 0.9f,  0.3f};
    asset.meshes.push_back(mesh);

    ScaleSanityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE(results[0].passed);  // 0.9m is reasonable
}

TEST_CASE("ScaleSanityValidator — suspiciously small (mm vs m)", "[validators]") {
    Asset asset;
    Mesh mesh;
    mesh.name = "tiny";
    mesh.bounds.min = {0.0f, 0.0f, 0.0f};
    mesh.bounds.max = {0.0005f, 0.0005f, 0.0005f};  // 0.5mm
    asset.meshes.push_back(mesh);

    ScaleSanityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("PhysicsPlausibilityValidator — missing physics", "[validators]") {
    Asset asset;
    // No physics set

    PhysicsPlausibilityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}
