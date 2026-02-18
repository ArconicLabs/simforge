// About: Python bindings for format detection, mesh I/O, and primitive fitting.
#include "bind_types.h"
#include "simforge/core/types.h"
#include "simforge/adapters/mesh_writer.h"
#include "simforge/adapters/primitive_fitter.h"

#include <sstream>

using namespace simforge;
namespace fitter = simforge::adapters;
namespace writer = simforge::adapters;

void bind_utilities(py::module_& m) {

    // ── Format detection ────────────────────────────────────────────

    m.def("detect_format",    &detect_format,    py::arg("path"),
          "Detect the source format of a file from its extension.");
    m.def("parse_format",     &parse_format,     py::arg("name"),
          "Parse a format name string (e.g. 'urdf') into a SourceFormat enum.");
    m.def("format_to_string", &format_to_string, py::arg("fmt"),
          "Convert a SourceFormat enum to its string name.");

    // ── Mesh I/O ────────────────────────────────────────────────────

    m.def("write_obj",        &writer::write_obj,        py::arg("mesh"), py::arg("path"),
          "Write a single mesh to an OBJ file.");
    m.def("write_obj_multi",  &writer::write_obj_multi,  py::arg("meshes"), py::arg("path"),
          "Write multiple meshes to a single OBJ file.");
    m.def("write_binary_stl", &writer::write_binary_stl, py::arg("mesh"), py::arg("path"),
          "Write a mesh to a binary STL file.");

    // ── Primitive fitting result types ──────────────────────────────

    py::class_<fitter::FittedBox>(m, "FittedBox")
        .def(py::init<>())
        .def_readwrite("center",       &fitter::FittedBox::center)
        .def_readwrite("half_extents", &fitter::FittedBox::half_extents)
        .def_property_readonly("axes", [](const fitter::FittedBox& b) {
            return std::vector<Vec3>{b.axes[0], b.axes[1], b.axes[2]};
        })
        .def("__repr__", [](const fitter::FittedBox& b) {
            std::ostringstream os;
            os << "<FittedBox half_extents=(" << b.half_extents.x << ", "
               << b.half_extents.y << ", " << b.half_extents.z << ")>";
            return os.str();
        });

    py::class_<fitter::FittedSphere>(m, "FittedSphere")
        .def(py::init<>())
        .def_readwrite("center", &fitter::FittedSphere::center)
        .def_readwrite("radius", &fitter::FittedSphere::radius)
        .def("__repr__", [](const fitter::FittedSphere& s) {
            std::ostringstream os;
            os << "<FittedSphere radius=" << s.radius << ">";
            return os.str();
        });

    py::class_<fitter::FittedCapsule>(m, "FittedCapsule")
        .def(py::init<>())
        .def_readwrite("p0",     &fitter::FittedCapsule::p0)
        .def_readwrite("p1",     &fitter::FittedCapsule::p1)
        .def_readwrite("radius", &fitter::FittedCapsule::radius)
        .def("__repr__", [](const fitter::FittedCapsule& c) {
            std::ostringstream os;
            os << "<FittedCapsule radius=" << c.radius << ">";
            return os.str();
        });

    // ── Primitive fitting functions ─────────────────────────────────

    m.def("fit_obb",     &fitter::fit_obb,     py::arg("mesh"),
          "Fit an oriented bounding box to a mesh.");
    m.def("fit_sphere",  &fitter::fit_sphere,  py::arg("mesh"),
          "Fit a bounding sphere to a mesh.");
    m.def("fit_capsule", &fitter::fit_capsule, py::arg("mesh"),
          "Fit a bounding capsule to a mesh.");
}
