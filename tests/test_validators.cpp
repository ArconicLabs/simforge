#include <catch2/catch_test_macros.hpp>

#include "simforge/validators/validator.h"
#include "test_helpers.h"

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

// ─── Additional Validator Tests ──────────────────────────────────

TEST_CASE("CollisionCorrectnessValidator — valid collision", "[validators]") {
    Asset asset;
    auto cube = test::make_cube();
    asset.meshes.push_back(cube);

    CollisionMesh coll;
    coll.type = CollisionType::ConvexHull;
    coll.hulls.push_back(cube);
    coll.total_volume = cube.compute_volume();
    asset.collision = coll;

    CollisionCorrectnessValidator validator;
    auto results = validator.validate(asset);

    // Hull count check should pass (1 hull, max 64)
    REQUIRE(!results.empty());
    REQUIRE(results[0].passed);
}

TEST_CASE("CollisionCorrectnessValidator — hull count over limit", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_cube());

    CollisionMesh coll;
    coll.type = CollisionType::ConvexDecomposition;
    // Add 65 hulls to exceed the default max of 64
    for (int i = 0; i < 65; i++) {
        coll.hulls.push_back(test::make_tetrahedron());
    }
    coll.total_volume = 1.0f;
    asset.collision = coll;

    CollisionCorrectnessValidator validator;
    auto results = validator.validate(asset);

    // Hull count check should fail
    bool hull_count_failed = false;
    for (const auto& r : results) {
        if (r.check_name == "hull_count") {
            hull_count_failed = !r.passed;
        }
    }
    REQUIRE(hull_count_failed);
}

TEST_CASE("CollisionCorrectnessValidator — volume ratio out of range", "[validators]") {
    Asset asset;
    auto cube = test::make_cube();
    asset.meshes.push_back(cube);

    CollisionMesh coll;
    coll.type = CollisionType::ConvexHull;
    coll.hulls.push_back(cube);
    // Set total_volume far larger than visual volume to trigger ratio check
    coll.total_volume = cube.compute_volume() * 5.0f;
    asset.collision = coll;

    CollisionCorrectnessValidator validator;
    auto results = validator.validate(asset);

    bool ratio_failed = false;
    for (const auto& r : results) {
        if (r.check_name == "volume_ratio") {
            ratio_failed = !r.passed;
        }
    }
    REQUIRE(ratio_failed);
}

TEST_CASE("CollisionCorrectnessValidator — no collision", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_cube());

    CollisionCorrectnessValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
    REQUIRE(results[0].check_name == "collision_exists");
}

TEST_CASE("WatertightValidator — watertight cube passes", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_cube());

    WatertightValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE(results[0].passed);
}

TEST_CASE("WatertightValidator — open box fails", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_open_box());

    WatertightValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("WatertightValidator — single triangle fails", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_single_triangle());

    WatertightValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("PhysicsPlausibilityValidator — valid physics passes", "[validators]") {
    Asset asset;
    auto cube = test::make_cube();
    asset.meshes.push_back(cube);

    PhysicsProperties props;
    props.mass = 1000.0f;  // 1m^3 cube at 1000 kg/m^3 → density = 1000
    props.material.density = 1000.0f;
    asset.physics = props;

    PhysicsPlausibilityValidator validator;
    auto results = validator.validate(asset);

    // mass_positive should pass, density_plausible should pass
    for (const auto& r : results) {
        REQUIRE(r.passed);
    }
}

TEST_CASE("PhysicsPlausibilityValidator — extreme density fails", "[validators]") {
    Asset asset;
    auto cube = test::make_cube();
    asset.meshes.push_back(cube);

    PhysicsProperties props;
    props.mass = 1000000.0f;  // 1m^3 cube → density = 1000000 kg/m^3 (way too high)
    asset.physics = props;

    PhysicsPlausibilityValidator validator;
    auto results = validator.validate(asset);

    bool density_failed = false;
    for (const auto& r : results) {
        if (r.check_name == "density_plausible") {
            density_failed = !r.passed;
        }
    }
    REQUIRE(density_failed);
}

TEST_CASE("ScaleSanityValidator — large mesh fails", "[validators]") {
    Asset asset;
    asset.meshes.push_back(test::make_large_mesh());

    ScaleSanityValidator validator;
    auto results = validator.validate(asset);

    REQUIRE(results.size() == 1);
    REQUIRE_FALSE(results[0].passed);
}
