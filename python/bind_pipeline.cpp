// About: Python bindings for Pipeline, Asset, and reporting types.
#include "bind_types.h"
#include "simforge/core/types.h"
#include "simforge/pipeline/pipeline.h"

using namespace simforge;

// Round-trip nlohmann::json <-> Python dict via JSON string.
namespace {

py::object json_to_py(const nlohmann::json& j) {
    if (j.is_null()) return py::none();
    auto json_mod = py::module_::import("json");
    return json_mod.attr("loads")(j.dump());
}

nlohmann::json py_to_json(const py::object& obj) {
    if (obj.is_none()) return nlohmann::json();
    auto json_mod = py::module_::import("json");
    std::string s = py::cast<std::string>(json_mod.attr("dumps")(obj));
    return nlohmann::json::parse(s);
}

}  // namespace

void bind_pipeline(py::module_& m) {

    // ── StageError ──────────────────────────────────────────────────

    py::class_<StageError>(m, "StageError")
        .def(py::init<>())
        .def_readwrite("stage_name", &StageError::stage_name)
        .def_readwrite("asset_id",   &StageError::asset_id)
        .def_readwrite("message",    &StageError::message)
        .def("__repr__", [](const StageError& e) {
            return "<StageError " + e.stage_name + ": " + e.message + ">";
        });

    // ── AssetReport ─────────────────────────────────────────────────

    py::class_<AssetReport>(m, "AssetReport")
        .def(py::init<>())
        .def_readwrite("asset_id",          &AssetReport::asset_id)
        .def_readwrite("asset_name",        &AssetReport::asset_name)
        .def_readwrite("final_status",      &AssetReport::final_status)
        .def_readwrite("validations",       &AssetReport::validations)
        .def_readwrite("stages_completed",  &AssetReport::stages_completed)
        .def_readwrite("errors",            &AssetReport::errors)
        .def_readwrite("processing_time_ms", &AssetReport::processing_time_ms)
        .def("__repr__", [](const AssetReport& r) {
            return "<AssetReport '" + r.asset_name + "' stages="
                   + std::to_string(r.stages_completed.size())
                   + " errors=" + std::to_string(r.errors.size()) + ">";
        });

    // ── PipelineReport ──────────────────────────────────────────────

    py::class_<PipelineReport>(m, "PipelineReport")
        .def(py::init<>())
        .def_readwrite("total_assets",  &PipelineReport::total_assets)
        .def_readwrite("passed",        &PipelineReport::passed)
        .def_readwrite("failed",        &PipelineReport::failed)
        .def_readwrite("total_time_ms", &PipelineReport::total_time_ms)
        .def_readwrite("asset_reports", &PipelineReport::asset_reports)
        .def("print_summary",          &PipelineReport::print_summary)
        .def("write_json",             &PipelineReport::write_json, py::arg("path"))
        .def("__repr__", [](const PipelineReport& r) {
            return "<PipelineReport assets=" + std::to_string(r.total_assets)
                   + " passed=" + std::to_string(r.passed)
                   + " failed=" + std::to_string(r.failed) + ">";
        });

    // ── PipelineConfig ──────────────────────────────────────────────

    py::class_<PipelineConfig>(m, "PipelineConfig")
        .def(py::init<>())
        .def_readwrite("source_dir",      &PipelineConfig::source_dir)
        .def_readwrite("output_dir",      &PipelineConfig::output_dir)
        .def_readwrite("target_formats",  &PipelineConfig::target_formats)
        .def_readwrite("stage_order",     &PipelineConfig::stage_order)
        .def_readwrite("threads",         &PipelineConfig::threads)
        .def_static("from_file",   &PipelineConfig::from_file, py::arg("config_path"))
        .def_static("from_string", &PipelineConfig::from_string, py::arg("yaml_str"));

    // ── Asset ───────────────────────────────────────────────────────

    py::class_<Asset>(m, "Asset")
        .def(py::init<>())
        .def_readwrite("id",            &Asset::id)
        .def_readwrite("name",          &Asset::name)
        .def_readwrite("source_path",   &Asset::source_path)
        .def_readwrite("source_format", &Asset::source_format)
        .def_readwrite("meshes",        &Asset::meshes)
        .def_readwrite("lods",          &Asset::lods)
        .def_readwrite("collision",     &Asset::collision)
        .def_readwrite("physics",       &Asset::physics)
        .def_readwrite("materials",     &Asset::materials)
        .def_readwrite("status",        &Asset::status)
        .def_readwrite("validations",   &Asset::validations)
        .def_readwrite("output_path",   &Asset::output_path)
        // kinematic_tree: unique_ptr exposed as property returning reference or None
        .def_property("kinematic_tree",
            [](const Asset& a) -> const KinematicTree* {
                return a.kinematic_tree.get();
            },
            [](Asset& a, const py::object& obj) {
                if (obj.is_none())
                    a.kinematic_tree.reset();
                else
                    a.kinematic_tree = std::make_unique<KinematicTree>(
                        obj.cast<const KinematicTree&>());
            },
            py::return_value_policy::reference_internal)
        // metadata: nlohmann::json via JSON string round-trip
        .def_property("metadata",
            [](const Asset& a) { return json_to_py(a.metadata); },
            [](Asset& a, const py::object& obj) { a.metadata = py_to_json(obj); })
        .def("is_articulated",          &Asset::is_articulated)
        .def("all_validations_passed",  &Asset::all_validations_passed)
        .def("__repr__", [](const Asset& a) {
            return "<Asset '" + a.name + "' meshes="
                   + std::to_string(a.meshes.size())
                   + (a.is_articulated() ? " articulated" : "") + ">";
        });

    // ── Pipeline ────────────────────────────────────────────────────

    py::class_<Pipeline>(m, "Pipeline")
        .def(py::init<PipelineConfig>(), py::arg("config"))
        .def("build",       &Pipeline::build)
        .def("run", [](Pipeline& p) {
            py::gil_scoped_release release;
            return p.run();
        })
        .def("run_single",      &Pipeline::run_single, py::arg("asset"))
        .def("dry_run",         &Pipeline::dry_run)
        .def("discover_assets", &Pipeline::discover_assets)
        .def("stage_names",     &Pipeline::stage_names);
}
