// About: Tests for articulation validators — kinematic tree structure,
// actuator plausibility, sensor configuration, and joint limits.
#include <catch2/catch_test_macros.hpp>

#include "simforge/validators/articulation_validators.h"
#include "test_helpers.h"

using namespace simforge;
using namespace simforge::test;

// ─── Helper ────────────────────────────────────────────────────────

static Asset make_articulated_asset() {
    Asset asset;
    asset.id = "test";
    asset.name = "robot_arm";
    asset.meshes.push_back(make_cube());

    KinematicTree tree;
    tree.root_link = "base_link";

    tree.links = {
        Link{"base_link", {make_cube()}, {}, {}, {}, {}, {}},
        Link{"forearm", {make_cube()}, {}, {}, {}, {}, {}},
    };

    Joint j;
    j.name = "shoulder";
    j.type = JointType::Revolute;
    j.parent_link = "base_link";
    j.child_link = "forearm";
    j.limits = JointLimits{-1.57f, 1.57f, 3.14f, 50.0f};
    tree.joints = {j};

    Actuator a;
    a.name = "motor";
    a.joint = "shoulder";
    a.gear_ratio = 100.0f;
    a.max_torque = 50.0f;
    a.max_velocity = 3.14f;
    tree.actuators = {a};

    Sensor s;
    s.name = "encoder";
    s.type = "joint_encoder";
    s.link = "forearm";
    s.properties = {{"resolution", 4096}};
    tree.sensors = {s};

    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));
    return asset;
}

// ─── KinematicTreeValidator ────────────────────────────────────────

TEST_CASE("KinematicTreeValidator — valid tree passes", "[articulation][validator]") {
    auto asset = make_articulated_asset();
    KinematicTreeValidator v;
    auto results = v.validate(asset);
    REQUIRE(results.size() == 1);
    REQUIRE(results[0].passed);
}

TEST_CASE("KinematicTreeValidator — single-body asset passes", "[articulation][validator]") {
    Asset asset;
    asset.meshes.push_back(make_cube());
    KinematicTreeValidator v;
    auto results = v.validate(asset);
    REQUIRE(results[0].passed);
}

TEST_CASE("KinematicTreeValidator — duplicate link name fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"a"}};  // duplicate
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    KinematicTreeValidator v;
    auto results = v.validate(asset);
    REQUIRE_FALSE(results[0].passed);
    REQUIRE(results[0].message.find("Duplicate link") != std::string::npos);
}

TEST_CASE("KinematicTreeValidator — bad joint reference fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"b"}};
    Joint j;
    j.name = "j1"; j.parent_link = "a"; j.child_link = "nonexistent";
    tree.joints = {j};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    KinematicTreeValidator v;
    auto results = v.validate(asset);
    REQUIRE_FALSE(results[0].passed);
}

// ─── ActuatorValidator ─────────────────────────────────────────────

TEST_CASE("ActuatorValidator — valid actuator passes", "[articulation][validator]") {
    auto asset = make_articulated_asset();
    ActuatorValidator v;
    auto results = v.validate(asset);
    REQUIRE(results.size() == 1);
    REQUIRE(results[0].passed);
}

TEST_CASE("ActuatorValidator — actuator on fixed joint fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"b"}};
    Joint j;
    j.name = "fixed_j"; j.type = JointType::Fixed;
    j.parent_link = "a"; j.child_link = "b";
    tree.joints = {j};
    Actuator a;
    a.name = "bad_motor"; a.joint = "fixed_j"; a.gear_ratio = 1.0f;
    tree.actuators = {a};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    ActuatorValidator v;
    auto results = v.validate(asset);
    bool has_failure = false;
    for (const auto& r : results) if (!r.passed) has_failure = true;
    REQUIRE(has_failure);
}

TEST_CASE("ActuatorValidator — bad joint reference fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}};
    Actuator a;
    a.name = "motor"; a.joint = "nonexistent"; a.gear_ratio = 1.0f;
    tree.actuators = {a};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    ActuatorValidator v;
    auto results = v.validate(asset);
    REQUIRE_FALSE(results[0].passed);
}

// ─── SensorValidator ───────────────────────────────────────────────

TEST_CASE("SensorValidator — valid sensor passes", "[articulation][validator]") {
    auto asset = make_articulated_asset();
    SensorValidator v;
    auto results = v.validate(asset);
    REQUIRE(results[0].passed);
}

TEST_CASE("SensorValidator — bad link reference fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}};
    Sensor s;
    s.name = "bad_sensor"; s.type = "imu"; s.link = "nonexistent";
    tree.sensors = {s};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    SensorValidator v;
    auto results = v.validate(asset);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("SensorValidator — empty type fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}};
    Sensor s;
    s.name = "typeless"; s.type = ""; s.link = "a";
    tree.sensors = {s};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    SensorValidator v;
    auto results = v.validate(asset);
    REQUIRE_FALSE(results[0].passed);
}

TEST_CASE("SensorValidator — camera missing width/height fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}};
    Sensor s;
    s.name = "cam"; s.type = "camera"; s.link = "a";
    // Missing width/height
    tree.sensors = {s};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    SensorValidator v;
    auto results = v.validate(asset);
    bool has_failure = false;
    for (const auto& r : results) if (!r.passed) has_failure = true;
    REQUIRE(has_failure);
}

// ─── JointLimitsValidator ──────────────────────────────────────────

TEST_CASE("JointLimitsValidator — valid limits pass", "[articulation][validator]") {
    auto asset = make_articulated_asset();
    JointLimitsValidator v;
    auto results = v.validate(asset);
    REQUIRE(results[0].passed);
}

TEST_CASE("JointLimitsValidator — revolute without limits fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"b"}};
    Joint j;
    j.name = "j1"; j.type = JointType::Revolute;
    j.parent_link = "a"; j.child_link = "b";
    // No limits set
    tree.joints = {j};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    JointLimitsValidator v;
    auto results = v.validate(asset);
    bool has_failure = false;
    for (const auto& r : results) if (!r.passed) has_failure = true;
    REQUIRE(has_failure);
}

TEST_CASE("JointLimitsValidator — lower >= upper fails", "[articulation][validator]") {
    Asset asset;
    asset.id = "test";
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"b"}};
    Joint j;
    j.name = "j1"; j.type = JointType::Revolute;
    j.parent_link = "a"; j.child_link = "b";
    j.limits = JointLimits{1.0f, -1.0f, 3.14f, 50.0f};  // inverted
    tree.joints = {j};
    tree.build_index();
    asset.kinematic_tree = std::make_unique<KinematicTree>(std::move(tree));

    JointLimitsValidator v;
    auto results = v.validate(asset);
    bool has_failure = false;
    for (const auto& r : results) if (!r.passed) has_failure = true;
    REQUIRE(has_failure);
}
