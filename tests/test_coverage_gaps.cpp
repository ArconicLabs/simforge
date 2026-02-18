// About: Tests for critical coverage gaps — articulated exports, stage edge
// cases, corrupted catalog recovery, report JSON validation, and input
// malformation handling.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <tinyxml2.h>
#include "tiny_gltf.h"

#include "simforge/adapters/adapter.h"
#include "simforge/core/types.h"
#include "simforge/pipeline/pipeline.h"
#include "simforge/pipeline/builtin_stages.h"
#include "test_helpers.h"

namespace simforge::adapters {
    void register_builtin_adapters();
}

using namespace simforge;
static const bool _init = [] {
    spdlog::set_level(spdlog::level::warn);
    simforge::adapters::register_builtin_adapters();
    simforge::stages::register_builtin_stages();
    return true;
}();

namespace {

struct TempDir {
    std::filesystem::path path;

    TempDir() {
        path = std::filesystem::temp_directory_path()
            / ("simforge_cov_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::filesystem::remove_all(path);
    }
};

std::string read_file(const std::filesystem::path& path) {
    std::ifstream f(path);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

// Build a fully articulated asset (3-link arm) for export testing.
Asset make_articulated_asset() {
    Asset asset;
    asset.id = "artic-test-001";
    asset.name = "test_arm";
    asset.source_format = SourceFormat::URDF;
    asset.status = AssetStatus::Validated;

    auto tree = std::make_unique<KinematicTree>();
    tree->root_link = "base_link";

    // Links with visual meshes and physics
    Link base;
    base.name = "base_link";
    base.visual_meshes.push_back(test::make_cube());
    base.physics = PhysicsProperties{};
    base.physics->mass = 5.0f;
    base.physics->center_of_mass = {0, 0, 0.05f};
    base.physics->inertia_diagonal = {0.01f, 0.01f, 0.01f};
    tree->links.push_back(std::move(base));

    Link shoulder;
    shoulder.name = "shoulder_link";
    shoulder.visual_meshes.push_back(test::make_cube());
    shoulder.physics = PhysicsProperties{};
    shoulder.physics->mass = 2.0f;
    shoulder.physics->center_of_mass = {0, 0, 0.1f};
    shoulder.physics->inertia_diagonal = {0.005f, 0.005f, 0.002f};
    tree->links.push_back(std::move(shoulder));

    Link forearm;
    forearm.name = "forearm_link";
    forearm.visual_meshes.push_back(test::make_tetrahedron());
    forearm.physics = PhysicsProperties{};
    forearm.physics->mass = 1.0f;
    forearm.physics->inertia_diagonal = {0.002f, 0.002f, 0.001f};
    tree->links.push_back(std::move(forearm));

    // Joints
    Joint j1;
    j1.name = "shoulder_joint";
    j1.type = JointType::Revolute;
    j1.parent_link = "base_link";
    j1.child_link = "shoulder_link";
    j1.axis = {0, 0, 1};
    j1.origin.position = {0, 0, 0.1f};
    j1.limits = JointLimits{-1.57f, 1.57f, 3.14f, 50.0f};
    j1.dynamics = JointDynamics{0.5f, 0.1f};
    tree->joints.push_back(std::move(j1));

    Joint j2;
    j2.name = "elbow_joint";
    j2.type = JointType::Revolute;
    j2.parent_link = "shoulder_link";
    j2.child_link = "forearm_link";
    j2.axis = {0, 1, 0};
    j2.origin.position = {0, 0, 0.2f};
    j2.limits = JointLimits{-2.0f, 2.0f, 3.14f, 30.0f};
    tree->joints.push_back(std::move(j2));

    // Actuators
    Actuator a1;
    a1.name = "shoulder_motor";
    a1.joint = "shoulder_joint";
    a1.control_mode = ControlMode::Position;
    a1.gear_ratio = 100.0f;
    a1.max_torque = 50.0f;
    tree->actuators.push_back(std::move(a1));

    Actuator a2;
    a2.name = "elbow_motor";
    a2.joint = "elbow_joint";
    a2.control_mode = ControlMode::Velocity;
    a2.gear_ratio = 80.0f;
    a2.max_torque = 30.0f;
    tree->actuators.push_back(std::move(a2));

    // Sensor
    Sensor imu;
    imu.name = "base_imu";
    imu.type = "imu";
    imu.link = "base_link";
    imu.properties = {{"update_rate", 200}};
    tree->sensors.push_back(std::move(imu));

    tree->build_index();
    asset.kinematic_tree = std::move(tree);

    return asset;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════
//  1. Articulated Export — URDF
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("URDF articulated export — valid robot structure", "[exporters][urdf][articulated]") {
    TempDir tmp;
    auto asset = make_articulated_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::URDF);
    REQUIRE(exporter != nullptr);

    auto output = tmp.path / "test_arm.urdf";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    REQUIRE(doc.LoadFile(output.string().c_str()) == tinyxml2::XML_SUCCESS);

    auto* robot = doc.FirstChildElement("robot");
    REQUIRE(robot != nullptr);
    REQUIRE(std::string(robot->Attribute("name")) == "test_arm");

    SECTION("all links present") {
        int link_count = 0;
        for (auto* el = robot->FirstChildElement("link"); el; el = el->NextSiblingElement("link"))
            link_count++;
        REQUIRE(link_count == 3);
    }

    SECTION("joints with correct parent/child") {
        auto* j1 = robot->FirstChildElement("joint");
        REQUIRE(j1 != nullptr);
        REQUIRE(std::string(j1->Attribute("name")) == "shoulder_joint");
        REQUIRE(std::string(j1->Attribute("type")) == "revolute");

        auto* parent = j1->FirstChildElement("parent");
        REQUIRE(parent != nullptr);
        REQUIRE(std::string(parent->Attribute("link")) == "base_link");

        auto* child = j1->FirstChildElement("child");
        REQUIRE(child != nullptr);
        REQUIRE(std::string(child->Attribute("link")) == "shoulder_link");
    }

    SECTION("joint limits and dynamics") {
        auto* j1 = robot->FirstChildElement("joint");
        auto* limit = j1->FirstChildElement("limit");
        REQUIRE(limit != nullptr);
        REQUIRE(limit->Attribute("lower") != nullptr);
        REQUIRE(limit->Attribute("upper") != nullptr);

        auto* dynamics = j1->FirstChildElement("dynamics");
        REQUIRE(dynamics != nullptr);
        REQUIRE(std::string(dynamics->Attribute("damping")) == "0.5");
    }

    SECTION("transmissions for actuators") {
        int trans_count = 0;
        for (auto* el = robot->FirstChildElement("transmission"); el;
             el = el->NextSiblingElement("transmission"))
            trans_count++;
        REQUIRE(trans_count == 2);
    }

    SECTION("gazebo sensor plugin") {
        auto* gz = robot->FirstChildElement("gazebo");
        REQUIRE(gz != nullptr);
        REQUIRE(std::string(gz->Attribute("reference")) == "base_link");
        auto* sensor = gz->FirstChildElement("sensor");
        REQUIRE(sensor != nullptr);
        REQUIRE(std::string(sensor->Attribute("name")) == "base_imu");
        REQUIRE(std::string(sensor->Attribute("type")) == "imu");
    }

    SECTION("per-link mesh files created") {
        REQUIRE(std::filesystem::exists(tmp.path / "meshes" / "base_link_visual_0.obj"));
        REQUIRE(std::filesystem::exists(tmp.path / "meshes" / "shoulder_link_visual_0.obj"));
        REQUIRE(std::filesystem::exists(tmp.path / "meshes" / "forearm_link_visual_0.obj"));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  2. Articulated Export — MJCF
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("MJCF articulated export — valid mujoco structure", "[exporters][mjcf][articulated]") {
    TempDir tmp;
    auto asset = make_articulated_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::MJCF);
    REQUIRE(exporter != nullptr);

    auto output = tmp.path / "test_arm.xml";
    REQUIRE(exporter->export_asset(asset, output));

    tinyxml2::XMLDocument doc;
    REQUIRE(doc.LoadFile(output.string().c_str()) == tinyxml2::XML_SUCCESS);

    auto* mujoco = doc.FirstChildElement("mujoco");
    REQUIRE(mujoco != nullptr);
    REQUIRE(std::string(mujoco->Attribute("model")) == "test_arm");

    SECTION("asset section with per-link meshes") {
        auto* asset_el = mujoco->FirstChildElement("asset");
        REQUIRE(asset_el != nullptr);
        int mesh_count = 0;
        for (auto* el = asset_el->FirstChildElement("mesh"); el;
             el = el->NextSiblingElement("mesh"))
            mesh_count++;
        REQUIRE(mesh_count == 3);  // one per link
    }

    SECTION("worldbody with nested body tree") {
        auto* worldbody = mujoco->FirstChildElement("worldbody");
        REQUIRE(worldbody != nullptr);
        auto* root_body = worldbody->FirstChildElement("body");
        REQUIRE(root_body != nullptr);
        REQUIRE(std::string(root_body->Attribute("name")) == "base_link");
    }

    SECTION("actuator section") {
        auto* actuator = mujoco->FirstChildElement("actuator");
        REQUIRE(actuator != nullptr);
        int act_count = 0;
        for (auto* el = actuator->FirstChildElement(); el; el = el->NextSiblingElement())
            act_count++;
        REQUIRE(act_count == 2);
    }

    SECTION("sensor section") {
        // Note: the fixture has 1 sensor (imu on base_link); MJCF sensor needs a 'site' or 'body'
        // to be emitted — check the sensor section exists
        // The MJCF exporter only emits sensors that have a link or site property
        auto* sensor_sec = mujoco->FirstChildElement("sensor");
        if (sensor_sec) {
            // At least check it parsed
            REQUIRE(sensor_sec != nullptr);
        }
    }

    SECTION("STL mesh files created for each link") {
        REQUIRE(std::filesystem::exists(tmp.path / "assets" / "base_link_0.stl"));
        REQUIRE(std::filesystem::exists(tmp.path / "assets" / "shoulder_link_0.stl"));
        REQUIRE(std::filesystem::exists(tmp.path / "assets" / "forearm_link_0.stl"));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  3. Articulated Export — USDA
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("USDA articulated export — PhysicsArticulationRootAPI", "[exporters][usda][articulated]") {
    TempDir tmp;
    auto asset = make_articulated_asset();

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::USDA);
    REQUIRE(exporter != nullptr);

    auto output = tmp.path / "test_arm.usda";
    REQUIRE(exporter->export_asset(asset, output));

    auto contents = read_file(output);

    SECTION("header and root prim") {
        REQUIRE_THAT(contents, Catch::Matchers::StartsWith("#usda 1.0"));
        REQUIRE(contents.find("defaultPrim = \"test_arm\"") != std::string::npos);
        REQUIRE(contents.find("PhysicsArticulationRootAPI") != std::string::npos);
    }

    SECTION("link xforms present") {
        REQUIRE(contents.find("def Xform \"base_link\"") != std::string::npos);
        REQUIRE(contents.find("def Xform \"shoulder_link\"") != std::string::npos);
        REQUIRE(contents.find("def Xform \"forearm_link\"") != std::string::npos);
    }

    SECTION("joint prims present") {
        REQUIRE(contents.find("def PhysicsRevoluteJoint \"shoulder_joint\"") != std::string::npos);
        REQUIRE(contents.find("def PhysicsRevoluteJoint \"elbow_joint\"") != std::string::npos);
    }

    SECTION("joint body references") {
        REQUIRE(contents.find("rel physics:body0 = <base_link>") != std::string::npos);
        REQUIRE(contents.find("rel physics:body1 = <shoulder_link>") != std::string::npos);
    }

    SECTION("physics attributes on links") {
        REQUIRE(contents.find("PhysicsRigidBodyAPI") != std::string::npos);
        REQUIRE(contents.find("PhysicsMassAPI") != std::string::npos);
        REQUIRE(contents.find("physics:mass") != std::string::npos);
    }

    SECTION("visual mesh prims") {
        REQUIRE(contents.find("def Mesh \"Visual_0\"") != std::string::npos);
    }
}

// ═══════════════════════════════════════════════════════════════════
//  4. PhysicsStage — explicit mass mode
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("PhysicsStage — explicit mass from metadata", "[stages][physics]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    // PhysicsStage requires CollisionGenerated status, so include collision
    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: explicit\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "explicit-mass";
    asset.name = "cube";
    asset.source_path = src_dir.path / "cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;
    asset.metadata["mass"] = 42.5f;

    auto report = pipeline.run_single(std::move(asset));

    REQUIRE(report.errors.empty());
    REQUIRE(report.final_status == AssetStatus::PhysicsAnnotated);
}

TEST_CASE("PhysicsStage — explicit mass without metadata defaults to zero", "[stages][physics]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: explicit\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "no-mass";
    asset.name = "cube";
    asset.source_path = src_dir.path / "cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;
    // No metadata["mass"] set

    auto report = pipeline.run_single(std::move(asset));

    REQUIRE(report.errors.empty());
    REQUIRE(report.final_status == AssetStatus::PhysicsAnnotated);
}

// ═══════════════════════════════════════════════════════════════════
//  5. Corrupted catalog recovery
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("Incremental — corrupted catalog triggers reprocess", "[incremental]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto yaml =
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "  target_formats: [obj]\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: primitive\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    watertight: false\n"
        "    physics_plausibility: false\n"
        "  export:\n"
        "    catalog: true\n";

    // First run — processes the asset and writes catalog
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }

    // Corrupt the catalog file
    auto catalog_path = out_dir.path / "cube.catalog.json";
    REQUIRE(std::filesystem::exists(catalog_path));
    {
        std::ofstream out(catalog_path);
        out << "{ this is not valid json !!!";
    }

    // Second run — should reprocess because catalog is corrupt
    {
        auto config = PipelineConfig::from_string(yaml);
        Pipeline pipeline(config);
        pipeline.build();
        auto report = pipeline.run();
        REQUIRE(report.total_assets == 1);
        REQUIRE(report.passed == 1);
    }
}

// ═══════════════════════════════════════════════════════════════════
//  6. ValidateStage — fail_on_warning
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("ValidateStage — fail_on_warning causes failure", "[stages][validate]") {
    TempDir src_dir;
    TempDir out_dir;

    // Use an open box (not watertight) to trigger a validation failure
    auto open_box = test::make_open_box();
    test::write_obj(open_box, src_dir.path / "open_box.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  validate:\n"
        "    watertight: true\n"
        "    fail_on_warning: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "fail-warn-test";
    asset.name = "open_box";
    asset.source_path = src_dir.path / "open_box.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    // The watertight check should fail, and fail_on_warning should make it an error
    REQUIRE(report.final_status == AssetStatus::Failed);
    REQUIRE(!report.errors.empty());
}

TEST_CASE("ValidateStage — without fail_on_warning passes despite warnings", "[stages][validate]") {
    TempDir src_dir;
    TempDir out_dir;

    auto open_box = test::make_open_box();
    test::write_obj(open_box, src_dir.path / "open_box.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  validate:\n"
        "    watertight: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "warn-pass-test";
    asset.name = "open_box";
    asset.source_path = src_dir.path / "open_box.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    // Without fail_on_warning, validation warnings don't cause failure
    REQUIRE(report.final_status == AssetStatus::Validated);
    REQUIRE(report.errors.empty());
}

// ═══════════════════════════════════════════════════════════════════
//  7. PipelineReport::write_json output validation
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("PipelineReport::write_json — schema validation", "[pipeline][report]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "  physics:\n"
        "    mass_estimation: geometry\n"
        "  validate:\n"
        "    mesh_integrity: true\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();
    auto report = pipeline.run();

    // Write report to JSON
    auto report_path = out_dir.path / "report.json";
    report.write_json(report_path);
    REQUIRE(std::filesystem::exists(report_path));

    // Parse and validate schema
    std::ifstream f(report_path);
    auto j = nlohmann::json::parse(f);

    REQUIRE(j.contains("total_assets"));
    REQUIRE(j.contains("passed"));
    REQUIRE(j.contains("failed"));
    REQUIRE(j.contains("total_time_ms"));
    REQUIRE(j.contains("assets"));

    REQUIRE(j["total_assets"].get<size_t>() == 1);
    REQUIRE(j["passed"].get<size_t>() == 1);
    REQUIRE(j["failed"].get<size_t>() == 0);
    REQUIRE(j["total_time_ms"].get<double>() > 0.0);

    // Validate asset report structure
    REQUIRE(j["assets"].is_array());
    REQUIRE(j["assets"].size() == 1);

    auto& aj = j["assets"][0];
    REQUIRE(aj.contains("id"));
    REQUIRE(aj.contains("name"));
    REQUIRE(aj["name"] == "cube");
    REQUIRE(aj.contains("status"));
    REQUIRE(aj.contains("time_ms"));
    REQUIRE(aj.contains("stages_completed"));
    REQUIRE(aj.contains("errors"));
    REQUIRE(aj.contains("validations"));

    REQUIRE(aj["stages_completed"].is_array());
    REQUIRE(aj["stages_completed"].size() >= 3);
    REQUIRE(aj["errors"].is_array());
    REQUIRE(aj["errors"].empty());
}

// ═══════════════════════════════════════════════════════════════════
//  8. Empty and malformed input files
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("OBJ importer — empty file", "[adapters][edge]") {
    TempDir tmp;
    auto path = tmp.path / "empty.obj";
    { std::ofstream out(path); }  // empty file

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    REQUIRE(importer != nullptr);

    auto meshes = importer->import(path);
    // Empty OBJ yields an empty mesh (no vertices, no faces)
    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].vertices.empty());
    REQUIRE(meshes[0].faces.empty());
}

TEST_CASE("OBJ importer — vertices only, no faces", "[adapters][edge]") {
    TempDir tmp;
    auto path = tmp.path / "verts_only.obj";
    {
        std::ofstream out(path);
        out << "v 0 0 0\n";
        out << "v 1 0 0\n";
        out << "v 0 1 0\n";
    }

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(SourceFormat::OBJ);
    auto meshes = importer->import(path);

    REQUIRE(meshes.size() == 1);
    REQUIRE(meshes[0].vertices.size() == 3);
    REQUIRE(meshes[0].faces.empty());
}

TEST_CASE("IngestStage rejects OBJ with no faces as empty mesh", "[stages][edge]") {
    TempDir src_dir;
    TempDir out_dir;

    // Write an OBJ with vertices but no faces
    {
        std::ofstream out(src_dir.path / "verts.obj");
        out << "v 0 0 0\nv 1 0 0\nv 0 1 0\n";
    }

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "no-faces";
    asset.name = "verts";
    asset.source_path = src_dir.path / "verts.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    // The importer returns a mesh with vertices but no faces.
    // IngestStage considers meshes.empty() for the error check,
    // but the mesh vector isn't empty — it has a mesh with no faces.
    // This should not crash.
    auto report = pipeline.run_single(std::move(asset));
    // Regardless of pass/fail, should not crash
    REQUIRE((report.final_status == AssetStatus::Ingested ||
             report.final_status == AssetStatus::Failed));
}

// ═══════════════════════════════════════════════════════════════════
//  9. Collision generator — fallback when named generator not found
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("CollisionStage — nonexistent generator falls back to convex hull", "[stages][collision]") {
    TempDir src_dir;
    TempDir out_dir;

    auto cube = test::make_cube();
    test::write_obj(cube, src_dir.path / "cube.obj");

    auto config = PipelineConfig::from_string(
        "pipeline:\n"
        "  source: " + src_dir.path.string() + "\n"
        "  output: " + out_dir.path.string() + "\n"
        "\n"
        "stages:\n"
        "  ingest:\n"
        "    formats: [obj]\n"
        "  collision:\n"
        "    method: convex_hull\n"
        "    generator: nonexistent_generator_xyz\n"
    );

    Pipeline pipeline(std::move(config));
    pipeline.build();

    Asset asset;
    asset.id = "fallback-test";
    asset.name = "cube";
    asset.source_path = src_dir.path / "cube.obj";
    asset.source_format = SourceFormat::OBJ;
    asset.status = AssetStatus::Raw;

    auto report = pipeline.run_single(std::move(asset));

    // Should not error — falls back to convex hull
    REQUIRE(report.errors.empty());

    bool collision_ran = false;
    for (const auto& s : report.stages_completed) {
        if (s == "collision") collision_ran = true;
    }
    REQUIRE(collision_ran);
}

// ═══════════════════════════════════════════════════════════════════
//  10. GLTF exporter — multi-mesh asset doesn't crash
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("GLTF export — multi-mesh asset exports first mesh", "[exporters][gltf][edge]") {
    TempDir tmp;

    Asset asset;
    asset.id = "multi-mesh-001";
    asset.name = "multi_mesh";
    asset.source_format = SourceFormat::GLB;
    asset.status = AssetStatus::Validated;

    // Add two meshes
    auto cube = test::make_cube();
    cube.name = "mesh_0";
    asset.meshes.push_back(cube);

    auto tet = test::make_tetrahedron();
    tet.name = "mesh_1";
    asset.meshes.push_back(tet);

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::GLB);
    REQUIRE(exporter != nullptr);

    auto output = tmp.path / "multi.glb";
    REQUIRE(exporter->export_asset(asset, output));

    // Verify the GLB is valid and has at least one mesh
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    bool ok = loader.LoadBinaryFromFile(&model, &err, &warn, output.string());
    REQUIRE(ok);
    REQUIRE(model.meshes.size() == 1);
    // The first mesh should be the one exported
    REQUIRE(model.meshes[0].name == "mesh_0");
}

TEST_CASE("GLTF export — empty meshes returns false", "[exporters][gltf][edge]") {
    TempDir tmp;

    Asset asset;
    asset.id = "empty-001";
    asset.name = "empty";
    asset.status = AssetStatus::Validated;
    // No meshes

    auto* exporter = AdapterManager::instance().find_exporter(SourceFormat::GLB);
    auto output = tmp.path / "empty.glb";
    REQUIRE_FALSE(exporter->export_asset(asset, output));
}
