// About: Shared pybind11 includes and forward declarations for binding files.
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/operators.h>

namespace py = pybind11;

void bind_types(py::module_& m);
void bind_physics(py::module_& m);
void bind_articulation(py::module_& m);
void bind_pipeline(py::module_& m);
void bind_adapters(py::module_& m);
void bind_utilities(py::module_& m);
