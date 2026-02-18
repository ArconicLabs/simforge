// About: Python bindings for articulation types (joints, links, kinematics).
#include "bind_types.h"
#include "simforge/core/types.h"

#include <sstream>

using namespace simforge;

void bind_articulation(py::module_& m) {

    // ── Enums ───────────────────────────────────────────────────────

    py::enum_<JointType>(m, "JointType")
        .value("Fixed",      JointType::Fixed)
        .value("Revolute",   JointType::Revolute)
        .value("Continuous", JointType::Continuous)
        .value("Prismatic",  JointType::Prismatic)
        .value("Floating",   JointType::Floating)
        .value("Planar",     JointType::Planar)
        .value("Spherical",  JointType::Spherical);

    py::enum_<ControlMode>(m, "ControlMode")
        .value("Position", ControlMode::Position)
        .value("Velocity", ControlMode::Velocity)
        .value("Effort",   ControlMode::Effort);

    // ── Quaternion ──────────────────────────────────────────────────

    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init([](float w, float x, float y, float z) {
            return Quaternion{w, x, y, z};
        }), py::arg("w") = 1.0f, py::arg("x") = 0.0f,
            py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def("__repr__", [](const Quaternion& q) {
            std::ostringstream os;
            os << "Quaternion(w=" << q.w << ", x=" << q.x
               << ", y=" << q.y << ", z=" << q.z << ")";
            return os.str();
        });

    // ── Pose ────────────────────────────────────────────────────────

    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("position",    &Pose::position)
        .def_readwrite("orientation", &Pose::orientation)
        .def("__repr__", [](const Pose& p) {
            std::ostringstream os;
            os << "Pose(pos=(" << p.position.x << ", " << p.position.y
               << ", " << p.position.z << "))";
            return os.str();
        });

    // ── JointLimits ─────────────────────────────────────────────────

    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("lower",    &JointLimits::lower)
        .def_readwrite("upper",    &JointLimits::upper)
        .def_readwrite("velocity", &JointLimits::velocity)
        .def_readwrite("effort",   &JointLimits::effort);

    // ── JointDynamics ───────────────────────────────────────────────

    py::class_<JointDynamics>(m, "JointDynamics")
        .def(py::init<>())
        .def_readwrite("damping",  &JointDynamics::damping)
        .def_readwrite("friction", &JointDynamics::friction);

    // ── Joint ───────────────────────────────────────────────────────

    py::class_<Joint>(m, "Joint", "Kinematic joint connecting two links.")
        .def(py::init<>())
        .def_readwrite("name",        &Joint::name)
        .def_readwrite("type",        &Joint::type)
        .def_readwrite("parent_link", &Joint::parent_link)
        .def_readwrite("child_link",  &Joint::child_link)
        .def_readwrite("origin",      &Joint::origin)
        .def_readwrite("axis",        &Joint::axis)
        .def_readwrite("limits",      &Joint::limits)
        .def_readwrite("dynamics",    &Joint::dynamics)
        .def("__repr__", [](const Joint& j) {
            return "<Joint '" + j.name + "' " + joint_type_to_string(j.type)
                   + " " + j.parent_link + " -> " + j.child_link + ">";
        });

    // ── Actuator ────────────────────────────────────────────────────

    py::class_<Actuator>(m, "Actuator", "Motor or controller attached to a joint.")
        .def(py::init<>())
        .def_readwrite("name",         &Actuator::name)
        .def_readwrite("joint",        &Actuator::joint)
        .def_readwrite("control_mode", &Actuator::control_mode)
        .def_readwrite("gear_ratio",   &Actuator::gear_ratio)
        .def_readwrite("max_torque",   &Actuator::max_torque)
        .def_readwrite("max_velocity", &Actuator::max_velocity)
        .def("__repr__", [](const Actuator& a) {
            return "<Actuator '" + a.name + "' joint='" + a.joint + "'>";
        });

    // ── Sensor ──────────────────────────────────────────────────────

    py::class_<Sensor>(m, "Sensor", "Sensor attached to a link (e.g. IMU, force/torque).")
        .def(py::init<>())
        .def_readwrite("name",   &Sensor::name)
        .def_readwrite("type",   &Sensor::type)
        .def_readwrite("link",   &Sensor::link)
        .def_readwrite("origin", &Sensor::origin)
        .def("__repr__", [](const Sensor& s) {
            return "<Sensor '" + s.name + "' type='" + s.type
                   + "' link='" + s.link + "'>";
        });

    // ── Link ────────────────────────────────────────────────────────

    py::class_<Link>(m, "Link", "Rigid body in a kinematic tree with visual/collision meshes.")
        .def(py::init<>())
        .def_readwrite("name",           &Link::name)
        .def_readwrite("visual_meshes",  &Link::visual_meshes)
        .def_readwrite("collision",      &Link::collision)
        .def_readwrite("physics",        &Link::physics)
        .def_readwrite("materials",      &Link::materials)
        .def_readwrite("origin",         &Link::origin)
        .def("__repr__", [](const Link& l) {
            return "<Link '" + l.name + "' meshes="
                   + std::to_string(l.visual_meshes.size()) + ">";
        });

    // ── KinematicTree ───────────────────────────────────────────────

    py::class_<KinematicTree>(m, "KinematicTree",
             "Articulated structure with links, joints, actuators, and sensors.")
        .def(py::init<>())
        .def_readwrite("root_link",  &KinematicTree::root_link)
        .def_readwrite("links",      &KinematicTree::links)
        .def_readwrite("joints",     &KinematicTree::joints)
        .def_readwrite("actuators",  &KinematicTree::actuators)
        .def_readwrite("sensors",    &KinematicTree::sensors)
        .def("find_link", &KinematicTree::find_link,
            py::arg("name"), py::return_value_policy::reference_internal,
            "Find a link by name, or None if not found.")
        .def("find_joint", &KinematicTree::find_joint,
            py::arg("name"), py::return_value_policy::reference_internal,
            "Find a joint by name, or None if not found.")
        .def("find_actuator_for_joint", &KinematicTree::find_actuator_for_joint,
            py::arg("joint_name"), py::return_value_policy::reference_internal,
            "Find the actuator attached to a joint, or None.")
        .def("dof",         &KinematicTree::dof,
            "Degrees of freedom (non-fixed joints).")
        .def("is_tree",     &KinematicTree::is_tree,
            "True if the structure forms a valid tree (no cycles).")
        .def("build_index", &KinematicTree::build_index,
            "Rebuild the internal name-to-index lookup tables.")
        .def("__repr__", [](const KinematicTree& kt) {
            return "<KinematicTree root='" + kt.root_link + "' links="
                   + std::to_string(kt.links.size()) + " joints="
                   + std::to_string(kt.joints.size()) + " dof="
                   + std::to_string(kt.dof()) + ">";
        });
}
