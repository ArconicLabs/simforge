// About: Python bindings for core geometry types and enums.
#include "bind_types.h"
#include "simforge/core/types.h"

#include <sstream>

using namespace simforge;

void bind_types(py::module_& m) {

    // ── Enums ───────────────────────────────────────────────────────

    py::enum_<SourceFormat>(m, "SourceFormat")
        .value("Unknown", SourceFormat::Unknown)
        .value("OBJ",     SourceFormat::OBJ)
        .value("FBX",     SourceFormat::FBX)
        .value("GLTF",    SourceFormat::GLTF)
        .value("GLB",     SourceFormat::GLB)
        .value("STL",     SourceFormat::STL)
        .value("STEP",    SourceFormat::STEP)
        .value("IGES",    SourceFormat::IGES)
        .value("URDF",    SourceFormat::URDF)
        .value("MJCF",    SourceFormat::MJCF)
        .value("USD",     SourceFormat::USD)
        .value("USDA",    SourceFormat::USDA)
        .value("USDC",    SourceFormat::USDC)
        .value("USDZ",    SourceFormat::USDZ)
        .value("DAE",     SourceFormat::DAE);

    py::enum_<AssetStatus>(m, "AssetStatus")
        .value("Raw",                AssetStatus::Raw)
        .value("Ingested",          AssetStatus::Ingested)
        .value("Articulated",       AssetStatus::Articulated)
        .value("CollisionGenerated", AssetStatus::CollisionGenerated)
        .value("PhysicsAnnotated",  AssetStatus::PhysicsAnnotated)
        .value("Optimized",         AssetStatus::Optimized)
        .value("Validated",         AssetStatus::Validated)
        .value("Ready",             AssetStatus::Ready)
        .value("Failed",            AssetStatus::Failed);

    py::enum_<LODLevel>(m, "LODLevel")
        .value("High",   LODLevel::High)
        .value("Medium", LODLevel::Medium)
        .value("Low",    LODLevel::Low);

    // ── Vec3 ────────────────────────────────────────────────────────

    py::class_<Vec3>(m, "Vec3")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def("__repr__", [](const Vec3& v) {
            std::ostringstream os;
            os << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
            return os.str();
        });

    // ── AABB ────────────────────────────────────────────────────────

    py::class_<AABB>(m, "AABB")
        .def(py::init<>())
        .def_readwrite("min", &AABB::min)
        .def_readwrite("max", &AABB::max)
        .def("center",  &AABB::center)
        .def("extents", &AABB::extents)
        .def("volume",  &AABB::volume)
        .def("__repr__", [](const AABB& b) {
            std::ostringstream os;
            os << "AABB(min=(" << b.min.x << ", " << b.min.y << ", " << b.min.z
               << "), max=(" << b.max.x << ", " << b.max.y << ", " << b.max.z << "))";
            return os.str();
        });

    // ── Triangle ────────────────────────────────────────────────────

    py::class_<Triangle>(m, "Triangle")
        .def(py::init<>())
        .def(py::init([](uint32_t v0, uint32_t v1, uint32_t v2) {
            return Triangle{v0, v1, v2};
        }), py::arg("v0"), py::arg("v1"), py::arg("v2"))
        .def_readwrite("v0", &Triangle::v0)
        .def_readwrite("v1", &Triangle::v1)
        .def_readwrite("v2", &Triangle::v2)
        .def("__repr__", [](const Triangle& t) {
            return "Triangle(" + std::to_string(t.v0) + ", "
                   + std::to_string(t.v1) + ", " + std::to_string(t.v2) + ")";
        });

    // ── Mesh ────────────────────────────────────────────────────────

    py::class_<Mesh>(m, "Mesh")
        .def(py::init<>())
        .def_readwrite("name",     &Mesh::name)
        .def_readwrite("vertices", &Mesh::vertices)
        .def_readwrite("normals",  &Mesh::normals)
        .def_readwrite("faces",    &Mesh::faces)
        .def_readwrite("uvs",      &Mesh::uvs)
        .def_readwrite("bounds",   &Mesh::bounds)
        .def("triangle_count",    &Mesh::triangle_count)
        .def("vertex_count",      &Mesh::vertex_count)
        .def("empty",             &Mesh::empty)
        .def("recompute_bounds",  &Mesh::recompute_bounds)
        .def("compute_volume",    &Mesh::compute_volume)
        .def("is_watertight",     &Mesh::is_watertight)
        .def("__repr__", [](const Mesh& mesh) {
            return "<Mesh '" + mesh.name + "' verts=" + std::to_string(mesh.vertex_count())
                   + " tris=" + std::to_string(mesh.triangle_count()) + ">";
        });

    // ── PBRMaterial ─────────────────────────────────────────────────

    py::class_<PBRMaterial>(m, "PBRMaterial")
        .def(py::init<>())
        .def_readwrite("name",          &PBRMaterial::name)
        .def_readwrite("base_color",    &PBRMaterial::base_color)
        .def_readwrite("metallic",      &PBRMaterial::metallic)
        .def_readwrite("roughness",     &PBRMaterial::roughness)
        .def_readwrite("opacity",       &PBRMaterial::opacity)
        .def_readwrite("albedo_map",    &PBRMaterial::albedo_map)
        .def_readwrite("normal_map",    &PBRMaterial::normal_map)
        .def_readwrite("roughness_map", &PBRMaterial::roughness_map)
        .def("__repr__", [](const PBRMaterial& mat) {
            return "<PBRMaterial '" + mat.name + "'>";
        });

    // ── LODMesh ─────────────────────────────────────────────────────

    py::class_<LODMesh>(m, "LODMesh")
        .def(py::init<>())
        .def_readwrite("level",         &LODMesh::level)
        .def_readwrite("mesh",          &LODMesh::mesh)
        .def_readwrite("max_triangles", &LODMesh::max_triangles);

    // ── ValidationResult ────────────────────────────────────────────

    py::class_<ValidationResult>(m, "ValidationResult")
        .def(py::init<>())
        .def_readwrite("passed",     &ValidationResult::passed)
        .def_readwrite("check_name", &ValidationResult::check_name)
        .def_readwrite("message",    &ValidationResult::message)
        .def_readwrite("score",      &ValidationResult::score)
        .def("__repr__", [](const ValidationResult& v) {
            return "<ValidationResult '" + v.check_name + "' "
                   + (v.passed ? "PASSED" : "FAILED") + ">";
        });
}
