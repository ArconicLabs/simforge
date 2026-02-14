// Per-exporter unit tests for USDA, URDF, MJCF, and GLTF export adapters.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <spdlog/spdlog.h>
#include <tinyxml2.h>
#include "tiny_gltf.h"

#include "simforge/adapters/adapter.h"
#include "simforge/core/types.h"
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

struct TempDir {
    std::filesystem::path path;

    TempDir() {
        path = std::filesystem::temp_directory_path()
            / ("simforge_export_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

// Build a fully-processed asset with meshes, collision, and physics.
Asset make_test_asset() {
    Asset asset;
    asset.id = "test-export-001";
    asset.name = "test_cube";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Validated;

    auto cube = test::make_cube();
    cube.name = "cube";
    // Add some normals for testing
    cube.normals = {
        {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1},
        {0, 0,  1}, {0, 0,  1}, {0, 0,  1}, {0, 0,  1},
    };
    asset.meshes.push_back(cube);

    // Collision
    CollisionMesh coll;
    coll.type = CollisionType::ConvexHull;
    coll.hulls.push_back(test::make_tetrahedron());
    coll.total_volume = 0.1f;
    asset.collision = coll;

    // Physics
    PhysicsProperties phys;
    phys.mass = 0.5f;
    phys.center_of_mass = {0.5f, 0.5f, 0.5f};
    phys.inertia_diagonal = {0.01f, 0.01f, 0.01f};
    phys.material.static_friction = 0.6f;
    phys.material.dynamic_friction = 0.4f;
    phys.material.restitution = 0.3f;
    asset.physics = phys;

    return asset;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream f(path);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

}  // namespace

// ─── USDA Exporter ──────────────────────────────────────────────────

TEST_CASE("USDA export — header and structure", "[exporters][usda]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto& mgr = AdapterManager::instance();
    auto* exporter = mgr.find_exporter(SourceFormat::USDA);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "usda_text");

    auto output = tmp.path / "test_cube.usda";
    REQUIRE(exporter->export_asset(asset, output));

    auto contents = read_file(output);

    // Header
    REQUIRE_THAT(contents, Catch::Matchers::StartsWith("#usda 1.0"));
    REQUIRE(contents.find("defaultPrim = \"test_cube\"") != std::string::npos);
    REQUIRE(contents.find("metersPerUnit = 1.0") != std::string::npos);
    REQUIRE(contents.find("upAxis = \"Z\"") != std::string::npos);
}

TEST_CASE("USDA export — mesh prim present", "[exporters][usda]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::USDA);
    auto output = tmp.path / "test_cube.usda";
    REQUIRE(exporter->export_asset(asset, output));

    auto contents = read_file(output);
    REQUIRE(contents.find("def Mesh \"cube\"") != std::string::npos);
    REQUIRE(contents.find("point3f[] points") != std::string::npos);
    REQUIRE(contents.find("int[] faceVertexCounts") != std::string::npos);
    REQUIRE(contents.find("int[] faceVertexIndices") != std::string::npos);
}

TEST_CASE("USDA export — collision scope present", "[exporters][usda]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::USDA);
    auto output = tmp.path / "test_cube.usda";
    REQUIRE(exporter->export_asset(asset, output));

    auto contents = read_file(output);
    REQUIRE(contents.find("def Scope \"Collision\"") != std::string::npos);
    REQUIRE(contents.find("def Mesh \"Hull_0\"") != std::string::npos);
}

TEST_CASE("USDA export — physics attributes present", "[exporters][usda]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::USDA);
    auto output = tmp.path / "test_cube.usda";
    REQUIRE(exporter->export_asset(asset, output));

    auto contents = read_file(output);
    REQUIRE(contents.find("physics:mass") != std::string::npos);
    REQUIRE(contents.find("physics:centerOfMass") != std::string::npos);
    REQUIRE(contents.find("physics:diagonalInertia") != std::string::npos);
    REQUIRE(contents.find("physics:staticFriction") != std::string::npos);
    REQUIRE(contents.find("physics:dynamicFriction") != std::string::npos);
    REQUIRE(contents.find("physics:restitution") != std::string::npos);
}

TEST_CASE("USDA export — USD format lookup works", "[exporters][usda]") {
    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::USD);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "usda_text");
}

// ─── URDF Exporter ──────────────────────────────────────────────────

TEST_CASE("URDF export — valid XML with robot and link", "[exporters][urdf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::URDF);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "urdf");

    auto output = tmp.path / "test_cube.urdf";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    REQUIRE(doc.LoadFile(output.string().c_str()) == tinyxml2::XML_SUCCESS);

    auto* robot = doc.FirstChildElement("robot");
    REQUIRE(robot != nullptr);
    REQUIRE(std::string(robot->Attribute("name")) == "test_cube");

    auto* link = robot->FirstChildElement("link");
    REQUIRE(link != nullptr);
    REQUIRE(std::string(link->Attribute("name")) == "base_link");
}

TEST_CASE("URDF export — external mesh files created", "[exporters][urdf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::URDF);
    auto output = tmp.path / "test_cube.urdf";
    REQUIRE(exporter->export_asset(asset, output));

    REQUIRE(std::filesystem::exists(tmp.path / "meshes" / "test_cube.obj"));
    REQUIRE(std::filesystem::exists(tmp.path / "meshes" / "test_cube_collision.obj"));
    REQUIRE(std::filesystem::file_size(tmp.path / "meshes" / "test_cube.obj") > 0);
}

TEST_CASE("URDF export — visual and collision geometry refs", "[exporters][urdf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::URDF);
    auto output = tmp.path / "test_cube.urdf";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    doc.LoadFile(output.string().c_str());
    auto* link = doc.FirstChildElement("robot")->FirstChildElement("link");

    auto* visual = link->FirstChildElement("visual");
    REQUIRE(visual != nullptr);
    auto* vis_mesh = visual->FirstChildElement("geometry")->FirstChildElement("mesh");
    REQUIRE(vis_mesh != nullptr);
    REQUIRE(std::string(vis_mesh->Attribute("filename")) == "meshes/test_cube.obj");

    auto* collision = link->FirstChildElement("collision");
    REQUIRE(collision != nullptr);
    auto* coll_mesh = collision->FirstChildElement("geometry")->FirstChildElement("mesh");
    REQUIRE(std::string(coll_mesh->Attribute("filename")) == "meshes/test_cube_collision.obj");
}

TEST_CASE("URDF export — inertial values correct", "[exporters][urdf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::URDF);
    auto output = tmp.path / "test_cube.urdf";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    doc.LoadFile(output.string().c_str());
    auto* link = doc.FirstChildElement("robot")->FirstChildElement("link");
    auto* inertial = link->FirstChildElement("inertial");
    REQUIRE(inertial != nullptr);

    auto* mass = inertial->FirstChildElement("mass");
    REQUIRE(mass != nullptr);
    REQUIRE(std::string(mass->Attribute("value")) == "0.5");

    auto* inertia = inertial->FirstChildElement("inertia");
    REQUIRE(inertia != nullptr);
    REQUIRE(std::string(inertia->Attribute("ixx")) == "0.01");
    REQUIRE(std::string(inertia->Attribute("ixy")) == "0");
    REQUIRE(std::string(inertia->Attribute("izz")) == "0.01");
}

// ─── MJCF Exporter ──────────────────────────────────────────────────

TEST_CASE("MJCF export — valid XML with mujoco structure", "[exporters][mjcf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::MJCF);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "mjcf");

    auto output = tmp.path / "test_cube.xml";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    REQUIRE(doc.LoadFile(output.string().c_str()) == tinyxml2::XML_SUCCESS);

    auto* mujoco = doc.FirstChildElement("mujoco");
    REQUIRE(mujoco != nullptr);
    REQUIRE(std::string(mujoco->Attribute("model")) == "test_cube");

    // Asset block with mesh references
    auto* asset_el = mujoco->FirstChildElement("asset");
    REQUIRE(asset_el != nullptr);
    auto* mesh_el = asset_el->FirstChildElement("mesh");
    REQUIRE(mesh_el != nullptr);

    // Worldbody with body
    auto* worldbody = mujoco->FirstChildElement("worldbody");
    REQUIRE(worldbody != nullptr);
    auto* body = worldbody->FirstChildElement("body");
    REQUIRE(body != nullptr);
    REQUIRE(std::string(body->Attribute("name")) == "test_cube");
}

TEST_CASE("MJCF export — STL files created", "[exporters][mjcf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::MJCF);
    auto output = tmp.path / "test_cube.xml";
    REQUIRE(exporter->export_asset(asset, output));

    REQUIRE(std::filesystem::exists(tmp.path / "assets" / "test_cube.stl"));
    REQUIRE(std::filesystem::exists(tmp.path / "assets" / "test_cube_collision.stl"));
    REQUIRE(std::filesystem::file_size(tmp.path / "assets" / "test_cube.stl") > 0);
}

TEST_CASE("MJCF export — physics defaults present", "[exporters][mjcf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::MJCF);
    auto output = tmp.path / "test_cube.xml";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    doc.LoadFile(output.string().c_str());

    auto* mujoco = doc.FirstChildElement("mujoco");
    auto* default_el = mujoco->FirstChildElement("default");
    REQUIRE(default_el != nullptr);
    auto* geom_default = default_el->FirstChildElement("geom");
    REQUIRE(geom_default != nullptr);
    REQUIRE(geom_default->Attribute("friction") != nullptr);

    // Inertial on the body
    auto* body = mujoco->FirstChildElement("worldbody")->FirstChildElement("body");
    auto* inertial = body->FirstChildElement("inertial");
    REQUIRE(inertial != nullptr);
    REQUIRE(std::string(inertial->Attribute("mass")) == "0.5");
}

// ─── GLTF Exporter ──────────────────────────────────────────────────

TEST_CASE("GLTF export — GLB file written and parseable", "[exporters][gltf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::GLB);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "gltf");

    auto output = tmp.path / "test_cube.glb";
    REQUIRE(exporter->export_asset(asset, output));
    REQUIRE(std::filesystem::exists(output));
    REQUIRE(std::filesystem::file_size(output) > 0);

    // Re-parse with tinygltf
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    bool ok = loader.LoadBinaryFromFile(&model, &err, &warn, output.string());
    REQUIRE(ok);
}

TEST_CASE("GLTF export — mesh and material nodes present", "[exporters][gltf]") {
    TempDir tmp;
    auto asset = make_test_asset();

    // Add a PBR material
    PBRMaterial mat;
    mat.name = "test_material";
    mat.base_color = {1.0f, 0.0f, 0.0f};
    mat.metallic = 0.8f;
    mat.roughness = 0.2f;
    asset.materials.push_back(mat);

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::GLB);
    auto output = tmp.path / "test_cube.glb";
    REQUIRE(exporter->export_asset(asset, output));

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    loader.LoadBinaryFromFile(&model, &err, &warn, output.string());

    REQUIRE(model.meshes.size() == 1);
    REQUIRE(model.materials.size() == 1);
    REQUIRE(model.materials[0].name == "test_material");
    REQUIRE(model.nodes.size() == 1);
    REQUIRE(model.scenes.size() == 1);
}

TEST_CASE("GLTF export — GLTF format lookup works", "[exporters][gltf]") {
    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::GLTF);
    REQUIRE(exporter != nullptr);
    REQUIRE(exporter->name() == "gltf");
}
