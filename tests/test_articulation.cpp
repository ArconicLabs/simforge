// About: Unit tests for articulation types — KinematicTree, Link, Joint,
// Actuator, Sensor, and their JSON serialization.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "simforge/core/types.h"
#include "test_helpers.h"

using namespace simforge;
using namespace simforge::test;

// ─── Helper: build a 3-link robot arm ──────────────────────────────

static KinematicTree make_simple_arm() {
    KinematicTree tree;
    tree.root_link = "base_link";

    // Links
    Link base;
    base.name = "base_link";
    base.visual_meshes.push_back(make_cube());
    tree.links.push_back(std::move(base));

    Link shoulder;
    shoulder.name = "shoulder_link";
    shoulder.visual_meshes.push_back(make_cube());
    tree.links.push_back(std::move(shoulder));

    Link forearm;
    forearm.name = "forearm_link";
    forearm.visual_meshes.push_back(make_cube());
    tree.links.push_back(std::move(forearm));

    // Joints
    Joint j1;
    j1.name = "shoulder_joint";
    j1.type = JointType::Revolute;
    j1.parent_link = "base_link";
    j1.child_link = "shoulder_link";
    j1.axis = {0, 0, 1};
    j1.limits = JointLimits{-1.57f, 1.57f, 3.14f, 50.0f};
    tree.joints.push_back(std::move(j1));

    Joint j2;
    j2.name = "elbow_joint";
    j2.type = JointType::Revolute;
    j2.parent_link = "shoulder_link";
    j2.child_link = "forearm_link";
    j2.axis = {0, 1, 0};
    j2.limits = JointLimits{-2.0f, 2.0f, 3.14f, 30.0f};
    tree.joints.push_back(std::move(j2));

    // Actuators
    Actuator a1;
    a1.name = "shoulder_motor";
    a1.joint = "shoulder_joint";
    a1.control_mode = ControlMode::Position;
    a1.gear_ratio = 100.0f;
    a1.max_torque = 50.0f;
    a1.max_velocity = 3.14f;
    tree.actuators.push_back(std::move(a1));

    Actuator a2;
    a2.name = "elbow_motor";
    a2.joint = "elbow_joint";
    a2.control_mode = ControlMode::Velocity;
    a2.gear_ratio = 80.0f;
    a2.max_torque = 30.0f;
    a2.max_velocity = 6.28f;
    tree.actuators.push_back(std::move(a2));

    // Sensors
    Sensor imu;
    imu.name = "base_imu";
    imu.type = "imu";
    imu.link = "base_link";
    imu.properties = {{"update_rate", 200}, {"noise_stddev", 0.01}};
    tree.sensors.push_back(std::move(imu));

    tree.build_index();
    return tree;
}

// ─── KinematicTree construction ────────────────────────────────────

TEST_CASE("KinematicTree — simple arm structure", "[articulation]") {
    auto tree = make_simple_arm();

    REQUIRE(tree.links.size() == 3);
    REQUIRE(tree.joints.size() == 2);
    REQUIRE(tree.actuators.size() == 2);
    REQUIRE(tree.sensors.size() == 1);
    REQUIRE(tree.root_link == "base_link");
}

TEST_CASE("KinematicTree — dof calculation", "[articulation]") {
    auto tree = make_simple_arm();
    REQUIRE(tree.dof() == 2);  // 2 revolute joints = 2 DOF
}

TEST_CASE("KinematicTree — is_tree validates structure", "[articulation]") {
    auto tree = make_simple_arm();
    REQUIRE(tree.is_tree());
}

TEST_CASE("KinematicTree — cycle detection", "[articulation]") {
    KinematicTree tree;
    tree.root_link = "a";

    Link a, b;
    a.name = "a";
    b.name = "b";
    tree.links = {a, b};

    // Create a cycle: a -> b -> a
    Joint j1, j2;
    j1.name = "j1"; j1.parent_link = "a"; j1.child_link = "b";
    j2.name = "j2"; j2.parent_link = "b"; j2.child_link = "a";
    tree.joints = {j1, j2};

    REQUIRE_FALSE(tree.is_tree());
}

TEST_CASE("KinematicTree — find_link by name", "[articulation]") {
    auto tree = make_simple_arm();

    REQUIRE(tree.find_link("base_link") != nullptr);
    REQUIRE(tree.find_link("shoulder_link") != nullptr);
    REQUIRE(tree.find_link("nonexistent") == nullptr);
}

TEST_CASE("KinematicTree — find_joint by name", "[articulation]") {
    auto tree = make_simple_arm();

    auto* j = tree.find_joint("shoulder_joint");
    REQUIRE(j != nullptr);
    REQUIRE(j->type == JointType::Revolute);
    REQUIRE(tree.find_joint("nonexistent") == nullptr);
}

TEST_CASE("KinematicTree — find_actuator_for_joint", "[articulation]") {
    auto tree = make_simple_arm();

    auto* a = tree.find_actuator_for_joint("shoulder_joint");
    REQUIRE(a != nullptr);
    REQUIRE(a->name == "shoulder_motor");
    REQUIRE(tree.find_actuator_for_joint("nonexistent") == nullptr);
}

TEST_CASE("KinematicTree — child_joints", "[articulation]") {
    auto tree = make_simple_arm();

    auto children = tree.child_joints("base_link");
    REQUIRE(children.size() == 1);
    REQUIRE(children[0]->name == "shoulder_joint");

    auto leaf_children = tree.child_joints("forearm_link");
    REQUIRE(leaf_children.empty());
}

// ─── String conversion ─────────────────────────────────────────────

TEST_CASE("JointType string round-trip", "[articulation]") {
    REQUIRE(parse_joint_type("revolute") == JointType::Revolute);
    REQUIRE(parse_joint_type("continuous") == JointType::Continuous);
    REQUIRE(parse_joint_type("prismatic") == JointType::Prismatic);
    REQUIRE(parse_joint_type("fixed") == JointType::Fixed);
    REQUIRE(parse_joint_type("floating") == JointType::Floating);
    REQUIRE(joint_type_to_string(JointType::Revolute) == "revolute");
    REQUIRE(joint_type_to_string(JointType::Fixed) == "fixed");
}

TEST_CASE("ControlMode string round-trip", "[articulation]") {
    REQUIRE(parse_control_mode("position") == ControlMode::Position);
    REQUIRE(parse_control_mode("velocity") == ControlMode::Velocity);
    REQUIRE(parse_control_mode("effort") == ControlMode::Effort);
    REQUIRE(control_mode_to_string(ControlMode::Position) == "position");
}

// ─── JSON serialization ────────────────────────────────────────────

TEST_CASE("KinematicTree JSON serialization", "[articulation]") {
    auto tree = make_simple_arm();
    nlohmann::json j;
    to_json(j, tree);

    REQUIRE(j["root_link"] == "base_link");
    REQUIRE(j["dof"] == 2);
    REQUIRE(j["links"].size() == 3);
    REQUIRE(j["joints"].size() == 2);
    REQUIRE(j["actuators"].size() == 2);
    REQUIRE(j["sensors"].size() == 1);

    // Check joint details
    REQUIRE(j["joints"][0]["name"] == "shoulder_joint");
    REQUIRE(j["joints"][0]["type"] == "revolute");

    // Check actuator details
    REQUIRE(j["actuators"][0]["name"] == "shoulder_motor");
    REQUIRE(j["actuators"][0]["gear_ratio"] == 100.0f);

    // Check sensor details
    REQUIRE(j["sensors"][0]["type"] == "imu");
    REQUIRE(j["sensors"][0]["properties"]["update_rate"] == 200);
}

// ─── Asset integration ─────────────────────────────────────────────

TEST_CASE("Asset — is_articulated", "[articulation]") {
    Asset asset;
    REQUIRE_FALSE(asset.is_articulated());

    asset.kinematic_tree = std::make_unique<KinematicTree>(make_simple_arm());
    REQUIRE(asset.is_articulated());
}

TEST_CASE("Asset — AssetStatus::Articulated exists in enum", "[articulation]") {
    Asset asset;
    asset.status = AssetStatus::Articulated;
    REQUIRE(static_cast<int>(AssetStatus::Articulated) > static_cast<int>(AssetStatus::Ingested));
    REQUIRE(static_cast<int>(AssetStatus::Articulated) < static_cast<int>(AssetStatus::CollisionGenerated));
}

// ─── DOF for various joint types ───────────────────────────────────

TEST_CASE("KinematicTree — dof for mixed joint types", "[articulation]") {
    KinematicTree tree;
    tree.root_link = "a";
    tree.links = {Link{"a"}, Link{"b"}, Link{"c"}, Link{"d"}};

    Joint fixed;
    fixed.name = "j_fixed"; fixed.type = JointType::Fixed;
    fixed.parent_link = "a"; fixed.child_link = "b";

    Joint floating;
    floating.name = "j_float"; floating.type = JointType::Floating;
    floating.parent_link = "b"; floating.child_link = "c";

    Joint spherical;
    spherical.name = "j_sphere"; spherical.type = JointType::Spherical;
    spherical.parent_link = "c"; spherical.child_link = "d";

    tree.joints = {fixed, floating, spherical};

    // Fixed=0, Floating=6, Spherical=3
    REQUIRE(tree.dof() == 9);
}
