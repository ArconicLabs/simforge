// About: Integration tests for URDF and MJCF ArticulatedImporters —
// verifies parsing of fixture files into correct KinematicTree structures.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>

#include <spdlog/spdlog.h>

#include "simforge/adapters/adapter.h"

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

static const std::filesystem::path fixtures_dir =
    std::filesystem::path(__FILE__).parent_path() / "fixtures";

// ─── URDF Importer ───────────────────────────────────────────────

TEST_CASE("URDF importer — parses simple_arm fixture", "[adapters][urdf]") {
    auto urdf_path = fixtures_dir / "simple_arm.urdf";
    REQUIRE(std::filesystem::exists(urdf_path));

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_articulated_importer(SourceFormat::URDF);
    REQUIRE(importer != nullptr);

    auto result = importer->import_articulated(urdf_path);
    const auto& tree = result.tree;

    SECTION("link count and names") {
        REQUIRE(tree.links.size() == 3);
        REQUIRE(tree.find_link("base_link") != nullptr);
        REQUIRE(tree.find_link("shoulder_link") != nullptr);
        REQUIRE(tree.find_link("forearm_link") != nullptr);
    }

    SECTION("root link detection") {
        REQUIRE(tree.root_link == "base_link");
    }

    SECTION("joint parsing") {
        REQUIRE(tree.joints.size() == 2);

        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder != nullptr);
        REQUIRE(shoulder->type == JointType::Revolute);
        REQUIRE(shoulder->parent_link == "base_link");
        REQUIRE(shoulder->child_link == "shoulder_link");
        REQUIRE(shoulder->axis.z == 1.0f);

        auto* elbow = tree.find_joint("elbow_joint");
        REQUIRE(elbow != nullptr);
        REQUIRE(elbow->type == JointType::Revolute);
        REQUIRE(elbow->parent_link == "shoulder_link");
        REQUIRE(elbow->child_link == "forearm_link");
    }

    SECTION("joint limits") {
        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder->limits.has_value());
        REQUIRE_THAT(shoulder->limits->lower, WithinAbs(-1.57, 0.01));
        REQUIRE_THAT(shoulder->limits->upper, WithinAbs(1.57, 0.01));
        REQUIRE_THAT(shoulder->limits->velocity, WithinAbs(3.14, 0.01));
        REQUIRE_THAT(shoulder->limits->effort, WithinAbs(50.0, 0.01));
    }

    SECTION("joint dynamics") {
        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder->dynamics.has_value());
        REQUIRE_THAT(shoulder->dynamics->damping, WithinAbs(0.5, 0.01));
        REQUIRE_THAT(shoulder->dynamics->friction, WithinAbs(0.1, 0.01));
    }

    SECTION("joint origins") {
        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE_THAT(shoulder->origin.position.z, WithinAbs(0.1, 0.001));
    }

    SECTION("actuators from transmissions") {
        REQUIRE(tree.actuators.size() == 2);

        bool found_shoulder = false, found_elbow = false;
        for (const auto& a : tree.actuators) {
            if (a.name == "shoulder_transmission") {
                found_shoulder = true;
                REQUIRE(a.joint == "shoulder_joint");
                REQUIRE_THAT(a.gear_ratio, WithinAbs(100.0, 0.01));
            }
            if (a.name == "elbow_transmission") {
                found_elbow = true;
                REQUIRE(a.joint == "elbow_joint");
                REQUIRE_THAT(a.gear_ratio, WithinAbs(80.0, 0.01));
            }
        }
        REQUIRE(found_shoulder);
        REQUIRE(found_elbow);
    }

    SECTION("sensors from gazebo plugins") {
        REQUIRE(tree.sensors.size() == 1);
        REQUIRE(tree.sensors[0].name == "base_imu");
        REQUIRE(tree.sensors[0].type == "imu");
        REQUIRE(tree.sensors[0].link == "base_link");
    }

    SECTION("inertial data") {
        auto* base = tree.find_link("base_link");
        REQUIRE(base->physics.has_value());
        REQUIRE_THAT(base->physics->mass, WithinAbs(5.0, 0.01));
        REQUIRE_THAT(base->physics->inertia_diagonal.x, WithinAbs(0.01, 0.001));
    }

    SECTION("DOF calculation") {
        REQUIRE(tree.dof() == 2);
    }

    SECTION("tree structure is valid") {
        REQUIRE(tree.is_tree());
    }
}

TEST_CASE("URDF importer — missing file throws", "[adapters][urdf]") {
    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_articulated_importer(SourceFormat::URDF);
    REQUIRE(importer != nullptr);

    REQUIRE_THROWS_AS(
        importer->import_articulated("/nonexistent/path.urdf"),
        std::runtime_error);
}

// ─── MJCF Importer ───────────────────────────────────────────────

TEST_CASE("MJCF importer — parses simple_arm fixture", "[adapters][mjcf]") {
    auto mjcf_path = fixtures_dir / "simple_arm.xml";
    REQUIRE(std::filesystem::exists(mjcf_path));

    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_articulated_importer(SourceFormat::MJCF);
    REQUIRE(importer != nullptr);

    auto result = importer->import_articulated(mjcf_path);
    const auto& tree = result.tree;

    SECTION("link count and names") {
        REQUIRE(tree.links.size() == 3);
        REQUIRE(tree.find_link("base_link") != nullptr);
        REQUIRE(tree.find_link("shoulder_link") != nullptr);
        REQUIRE(tree.find_link("forearm_link") != nullptr);
    }

    SECTION("root link is first body") {
        REQUIRE(tree.root_link == "base_link");
    }

    SECTION("joint parsing") {
        REQUIRE(tree.joints.size() == 2);

        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder != nullptr);
        REQUIRE(shoulder->type == JointType::Revolute);
        REQUIRE(shoulder->parent_link == "base_link");
        REQUIRE(shoulder->child_link == "shoulder_link");
        REQUIRE(shoulder->axis.z == 1.0f);
    }

    SECTION("joint limits from range attribute") {
        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder->limits.has_value());
        REQUIRE_THAT(shoulder->limits->lower, WithinAbs(-1.57, 0.01));
        REQUIRE_THAT(shoulder->limits->upper, WithinAbs(1.57, 0.01));
    }

    SECTION("joint dynamics") {
        auto* shoulder = tree.find_joint("shoulder_joint");
        REQUIRE(shoulder->dynamics.has_value());
        REQUIRE_THAT(shoulder->dynamics->damping, WithinAbs(0.5, 0.01));
        REQUIRE_THAT(shoulder->dynamics->friction, WithinAbs(0.1, 0.01));
    }

    SECTION("actuators") {
        REQUIRE(tree.actuators.size() == 2);

        bool found_shoulder = false, found_elbow = false;
        for (const auto& a : tree.actuators) {
            if (a.name == "shoulder_motor") {
                found_shoulder = true;
                REQUIRE(a.joint == "shoulder_joint");
                REQUIRE(a.control_mode == ControlMode::Effort);
                REQUIRE_THAT(a.gear_ratio, WithinAbs(100.0, 0.01));
                REQUIRE_THAT(a.max_torque, WithinAbs(50.0, 0.01));
            }
            if (a.name == "elbow_motor") {
                found_elbow = true;
                REQUIRE(a.joint == "elbow_joint");
                REQUIRE_THAT(a.gear_ratio, WithinAbs(80.0, 0.01));
            }
        }
        REQUIRE(found_shoulder);
        REQUIRE(found_elbow);
    }

    SECTION("sensors") {
        REQUIRE(tree.sensors.size() == 2);

        bool found_gyro = false, found_accel = false;
        for (const auto& s : tree.sensors) {
            if (s.name == "base_gyro") {
                found_gyro = true;
                REQUIRE(s.type == "gyro");
            }
            if (s.name == "base_accel") {
                found_accel = true;
                REQUIRE(s.type == "accelerometer");
            }
        }
        REQUIRE(found_gyro);
        REQUIRE(found_accel);
    }

    SECTION("inertial data") {
        auto* base = tree.find_link("base_link");
        REQUIRE(base->physics.has_value());
        REQUIRE_THAT(base->physics->mass, WithinAbs(5.0, 0.01));
    }

    SECTION("DOF calculation") {
        REQUIRE(tree.dof() == 2);
    }

    SECTION("tree structure is valid") {
        REQUIRE(tree.is_tree());
    }
}

TEST_CASE("MJCF importer — missing file throws", "[adapters][mjcf]") {
    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_articulated_importer(SourceFormat::MJCF);
    REQUIRE(importer != nullptr);

    REQUIRE_THROWS_AS(
        importer->import_articulated("/nonexistent/path.xml"),
        std::runtime_error);
}

// ─── Cross-importer consistency ──────────────────────────────────

TEST_CASE("URDF and MJCF produce equivalent tree structure", "[adapters][integration]") {
    auto& mgr = AdapterManager::instance();

    auto urdf_result = mgr.find_articulated_importer(SourceFormat::URDF)
                           ->import_articulated(fixtures_dir / "simple_arm.urdf");
    auto mjcf_result = mgr.find_articulated_importer(SourceFormat::MJCF)
                           ->import_articulated(fixtures_dir / "simple_arm.xml");

    REQUIRE(urdf_result.tree.links.size() == mjcf_result.tree.links.size());
    REQUIRE(urdf_result.tree.joints.size() == mjcf_result.tree.joints.size());
    REQUIRE(urdf_result.tree.dof() == mjcf_result.tree.dof());

    // Both should identify same root
    REQUIRE(urdf_result.tree.root_link == mjcf_result.tree.root_link);
}
