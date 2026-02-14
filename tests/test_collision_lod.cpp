// Unit tests for collision and LOD adapters: meshoptimizer, primitive fitter, CoACD.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include <spdlog/spdlog.h>

#include "simforge/adapters/adapter.h"
#include "simforge/adapters/primitive_fitter.h"
#include "test_helpers.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;
using Catch::Matchers::WithinAbs;

static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    return true;
}();

// ─── meshoptimizer LOD adapter ────────────────────────────────────

// Build a subdivided cube with enough triangles for meaningful decimation
static Mesh make_dense_mesh() {
    Mesh m;
    m.name = "dense";

    // 5x5 grid patch on each face of a cube → plenty of triangles
    const int N = 5;  // grid subdivisions per face
    auto add_face = [&](Vec3 origin, Vec3 u, Vec3 v) {
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (int i = 0; i <= N; i++) {
            for (int j = 0; j <= N; j++) {
                float s = static_cast<float>(i) / static_cast<float>(N);
                float t = static_cast<float>(j) / static_cast<float>(N);
                m.vertices.push_back(origin + u * s + v * t);
            }
        }
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                uint32_t a = base + i * (N + 1) + j;
                uint32_t b = a + 1;
                uint32_t c = a + (N + 1);
                uint32_t d = c + 1;
                m.faces.push_back({a, c, b});
                m.faces.push_back({b, c, d});
            }
        }
    };

    // 6 faces of a unit cube
    add_face({0,0,0}, {1,0,0}, {0,1,0});  // front  z=0
    add_face({0,0,1}, {1,0,0}, {0,1,0});  // back   z=1
    add_face({0,0,0}, {1,0,0}, {0,0,1});  // bottom y=0
    add_face({0,1,0}, {1,0,0}, {0,0,1});  // top    y=1
    add_face({0,0,0}, {0,1,0}, {0,0,1});  // left   x=0
    add_face({1,0,0}, {0,1,0}, {0,0,1});  // right  x=1

    m.recompute_bounds();
    return m;
}

TEST_CASE("meshoptimizer — decimate dense mesh below budget", "[lod][meshoptimizer]") {
    auto mesh = make_dense_mesh();
    REQUIRE(mesh.triangle_count() == 300);  // 6 faces × 5×5 × 2 = 300

    auto& mgr = AdapterManager::instance();
    auto* lod_gen = mgr.find_lod_generator("meshoptimizer");
    REQUIRE(lod_gen != nullptr);
    REQUIRE(lod_gen->name() == "meshoptimizer");

    LODParams params{100, 0.5f};  // target 100 triangles from 300
    auto result = lod_gen->decimate(mesh, params);

    // Mesh should be significantly reduced (at least halved)
    REQUIRE(result.triangle_count() < 200);
    REQUIRE(result.triangle_count() > 0);
}

TEST_CASE("meshoptimizer — target larger than input returns original", "[lod][meshoptimizer]") {
    auto cube = test::make_cube();
    REQUIRE(cube.triangle_count() == 12);

    auto& mgr = AdapterManager::instance();
    auto* lod_gen = mgr.find_lod_generator("meshoptimizer");
    REQUIRE(lod_gen != nullptr);

    LODParams params{100, 0.7f};  // target more than input
    auto result = lod_gen->decimate(cube, params);

    REQUIRE(result.triangle_count() == cube.triangle_count());
}

TEST_CASE("meshoptimizer — decimated mesh has valid face indices", "[lod][meshoptimizer]") {
    auto cube = test::make_cube();

    auto& mgr = AdapterManager::instance();
    auto* lod_gen = mgr.find_lod_generator("meshoptimizer");
    REQUIRE(lod_gen != nullptr);

    LODParams params{6, 0.5f};
    auto result = lod_gen->decimate(cube, params);

    for (const auto& face : result.faces) {
        REQUIRE(face.v0 < result.vertex_count());
        REQUIRE(face.v1 < result.vertex_count());
        REQUIRE(face.v2 < result.vertex_count());
    }
}

// ─── Primitive fitter ─────────────────────────────────────────────

TEST_CASE("fit_obb — unit cube half-extents", "[collision][primitive]") {
    auto cube = test::make_cube();
    auto box = adapters::fit_obb(cube);

    // Cube is 0-1 range, so half-extents should be ~0.5 on each axis.
    // PCA may rotate axes, so sort half_extents for comparison.
    float he[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
    std::sort(he, he + 3);

    REQUIRE_THAT(he[0], WithinAbs(0.5, 0.05));
    REQUIRE_THAT(he[1], WithinAbs(0.5, 0.05));
    REQUIRE_THAT(he[2], WithinAbs(0.5, 0.05));
}

TEST_CASE("fit_sphere — unit cube bounding sphere", "[collision][primitive]") {
    auto cube = test::make_cube();
    auto sphere = adapters::fit_sphere(cube);

    // Half-diagonal of unit cube = sqrt(3)/2 ≈ 0.866
    // Ritter's algorithm gives a tight approximation
    REQUIRE(sphere.radius >= 0.85f);
    REQUIRE(sphere.radius <= 1.0f);  // shouldn't be much bigger
}

TEST_CASE("fit_capsule — elongated mesh aligns with long axis", "[collision][primitive]") {
    // Create a long thin box: 10 units along X, 1 unit along Y and Z
    Mesh elongated;
    elongated.name = "long_box";
    elongated.vertices = {
        {0, 0, 0}, {10, 0, 0}, {10, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {10, 0, 1}, {10, 1, 1}, {0, 1, 1},
    };
    elongated.faces = {
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 5, 1}, {0, 4, 5},
        {2, 6, 7}, {2, 7, 3},
        {0, 3, 7}, {0, 7, 4},
        {1, 5, 6}, {1, 6, 2},
    };
    elongated.recompute_bounds();

    auto capsule = adapters::fit_capsule(elongated);

    // Capsule axis should be roughly along X (the long dimension)
    Vec3 axis = {capsule.p1.x - capsule.p0.x,
                 capsule.p1.y - capsule.p0.y,
                 capsule.p1.z - capsule.p0.z};
    float len = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (len > 0) {
        axis.x /= len; axis.y /= len; axis.z /= len;
    }

    // X component of normalized axis should dominate
    REQUIRE(std::abs(axis.x) > 0.9f);
}

TEST_CASE("Primitive generator selects box for cube", "[collision][primitive]") {
    auto cube = test::make_cube();

    auto& mgr = AdapterManager::instance();
    auto* gen = mgr.find_collision_generator("primitive");
    REQUIRE(gen != nullptr);
    REQUIRE(gen->name() == "primitive");

    CollisionParams params;
    params.method = CollisionType::Primitive;
    auto result = gen->generate(cube, params);

    REQUIRE(result.type == CollisionType::Primitive);
    REQUIRE(result.hull_count() == 1);
    REQUIRE(result.total_volume > 0.0f);

    // For a unit cube, box should be tightest fit
    // Box volume ≈ 1.0, sphere volume ≈ 2.72, capsule even larger
    REQUIRE(result.hulls[0].name == "primitive_box");
}

TEST_CASE("Primitive generator — collision mesh has valid geometry", "[collision][primitive]") {
    auto cube = test::make_cube();

    auto& mgr = AdapterManager::instance();
    auto* gen = mgr.find_collision_generator("primitive");
    REQUIRE(gen != nullptr);

    CollisionParams params;
    params.method = CollisionType::Primitive;
    auto result = gen->generate(cube, params);

    // Verify the hull mesh is non-empty and has valid indices
    REQUIRE(!result.hulls.empty());
    const auto& hull = result.hulls[0];
    REQUIRE(!hull.vertices.empty());
    REQUIRE(!hull.faces.empty());

    for (const auto& face : hull.faces) {
        REQUIRE(face.v0 < hull.vertex_count());
        REQUIRE(face.v1 < hull.vertex_count());
        REQUIRE(face.v2 < hull.vertex_count());
    }
}

// ─── CoACD adapter (conditional) ──────────────────────────────────

#ifdef SIMFORGE_HAS_COACD

TEST_CASE("CoACD — decompose L-shaped mesh into multiple hulls", "[collision][coacd]") {
    // Build an L-shaped mesh from two boxes
    Mesh l_shape;
    l_shape.name = "l_shape";

    // Box 1: 0-1 on all axes
    l_shape.vertices = {
        {0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {2, 0, 1}, {2, 1, 1}, {0, 1, 1},
    };
    // Box 2: extends upward from x=0
    uint32_t base = 8;
    std::vector<Vec3> box2 = {
        {0, 1, 0}, {1, 1, 0}, {1, 3, 0}, {0, 3, 0},
        {0, 1, 1}, {1, 1, 1}, {1, 3, 1}, {0, 3, 1},
    };
    for (const auto& v : box2) l_shape.vertices.push_back(v);

    // Faces for box 1
    l_shape.faces = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
        {0, 5, 1}, {0, 4, 5}, {2, 6, 7}, {2, 7, 3},
        {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
    };
    // Faces for box 2
    Triangle box2_faces[] = {
        {base+0, base+1, base+2}, {base+0, base+2, base+3},
        {base+4, base+6, base+5}, {base+4, base+7, base+6},
        {base+0, base+5, base+1}, {base+0, base+4, base+5},
        {base+2, base+6, base+7}, {base+2, base+7, base+3},
        {base+0, base+3, base+7}, {base+0, base+7, base+4},
        {base+1, base+5, base+6}, {base+1, base+6, base+2},
    };
    for (const auto& f : box2_faces) l_shape.faces.push_back(f);
    l_shape.recompute_bounds();

    auto& mgr = AdapterManager::instance();
    auto* gen = mgr.find_collision_generator("coacd");
    REQUIRE(gen != nullptr);

    CollisionParams params;
    params.method = CollisionType::ConvexDecomposition;
    params.threshold = 0.05f;
    params.max_hulls = 32;
    auto result = gen->generate(l_shape, params);

    REQUIRE(result.type == CollisionType::ConvexDecomposition);
    REQUIRE(result.hull_count() >= 2);  // L-shape should decompose into ≥2 hulls
    REQUIRE(result.hull_count() <= params.max_hulls);
    REQUIRE(result.total_volume > 0.0f);

    // Each hull should be non-empty
    for (const auto& hull : result.hulls) {
        REQUIRE(!hull.vertices.empty());
        REQUIRE(!hull.faces.empty());
    }
}

#endif  // SIMFORGE_HAS_COACD
