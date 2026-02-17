// About: Python bindings for physics and collision types.
#include "bind_types.h"
#include "simforge/core/types.h"

using namespace simforge;

void bind_physics(py::module_& m) {

    // ── CollisionType ───────────────────────────────────────────────

    py::enum_<CollisionType>(m, "CollisionType")
        .value("ConvexHull",          CollisionType::ConvexHull)
        .value("ConvexDecomposition", CollisionType::ConvexDecomposition)
        .value("TriangleMesh",        CollisionType::TriangleMesh)
        .value("Primitive",           CollisionType::Primitive);

    // ── PhysicsMaterial ─────────────────────────────────────────────

    py::class_<PhysicsMaterial>(m, "PhysicsMaterial")
        .def(py::init<>())
        .def_readwrite("name",             &PhysicsMaterial::name)
        .def_readwrite("density",          &PhysicsMaterial::density)
        .def_readwrite("static_friction",  &PhysicsMaterial::static_friction)
        .def_readwrite("dynamic_friction", &PhysicsMaterial::dynamic_friction)
        .def_readwrite("restitution",      &PhysicsMaterial::restitution)
        .def("__repr__", [](const PhysicsMaterial& pm) {
            return "<PhysicsMaterial '" + pm.name + "' density="
                   + std::to_string(pm.density) + ">";
        });

    // ── PhysicsProperties ───────────────────────────────────────────

    py::class_<PhysicsProperties>(m, "PhysicsProperties")
        .def(py::init<>())
        .def_readwrite("mass",             &PhysicsProperties::mass)
        .def_readwrite("center_of_mass",   &PhysicsProperties::center_of_mass)
        .def_readwrite("inertia_diagonal", &PhysicsProperties::inertia_diagonal)
        .def_readwrite("material",         &PhysicsProperties::material)
        .def_readwrite("is_static",        &PhysicsProperties::is_static)
        .def_readwrite("mass_estimated",   &PhysicsProperties::mass_estimated)
        .def_static("estimate_from_mesh",  &PhysicsProperties::estimate_from_mesh,
            py::arg("mesh"), py::arg("material"))
        .def("__repr__", [](const PhysicsProperties& pp) {
            return "<PhysicsProperties mass=" + std::to_string(pp.mass)
                   + (pp.is_static ? " static" : "") + ">";
        });

    // ── CollisionMesh ───────────────────────────────────────────────

    py::class_<CollisionMesh>(m, "CollisionMesh")
        .def(py::init<>())
        .def_readwrite("type",         &CollisionMesh::type)
        .def_readwrite("hulls",        &CollisionMesh::hulls)
        .def_readwrite("total_volume", &CollisionMesh::total_volume)
        .def("hull_count",             &CollisionMesh::hull_count)
        .def("__repr__", [](const CollisionMesh& cm) {
            return "<CollisionMesh hulls=" + std::to_string(cm.hull_count()) + ">";
        });
}
