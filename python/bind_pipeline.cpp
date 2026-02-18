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

    py::class_<StageError>(m, "StageError", "Error produced by a pipeline stage.")
        .def(py::init<>())
        .def_readwrite("stage_name", &StageError::stage_name)
        .def_readwrite("asset_id",   &StageError::asset_id)
        .def_readwrite("message",    &StageError::message)
        .def("__repr__", [](const StageError& e) {
            return "<StageError " + e.stage_name + ": " + e.message + ">";
        });

    // ── AssetReport ─────────────────────────────────────────────────

    py::class_<AssetReport>(m, "AssetReport", "Per-asset processing report with status, timing, and errors.")
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

    py::class_<PipelineReport>(m, "PipelineReport", "Aggregate report for a full pipeline run.")
        .def(py::init<>())
        .def_readwrite("total_assets",  &PipelineReport::total_assets)
        .def_readwrite("passed",        &PipelineReport::passed)
        .def_readwrite("failed",        &PipelineReport::failed)
        .def_readwrite("total_time_ms", &PipelineReport::total_time_ms)
        .def_readwrite("asset_reports", &PipelineReport::asset_reports)
        .def("print_summary",          &PipelineReport::print_summary,
             "Print a human-readable summary to the log.")
        .def("write_json",             &PipelineReport::write_json, py::arg("path"),
             "Write the report as JSON to the given file path.")
        .def("__repr__", [](const PipelineReport& r) {
            return "<PipelineReport assets=" + std::to_string(r.total_assets)
                   + " passed=" + std::to_string(r.passed)
                   + " failed=" + std::to_string(r.failed) + ">";
        });

    // ── PipelineConfig ──────────────────────────────────────────────

    py::class_<PipelineConfig>(m, "PipelineConfig",
             "Pipeline configuration parsed from YAML. Use from_file() or from_string().")
        .def(py::init<>())
        .def_readwrite("source_dir",      &PipelineConfig::source_dir)
        .def_readwrite("output_dir",      &PipelineConfig::output_dir)
        .def_readwrite("target_formats",  &PipelineConfig::target_formats)
        .def_readwrite("stage_order",     &PipelineConfig::stage_order)
        .def_readwrite("threads",         &PipelineConfig::threads)
        .def_readwrite("force",           &PipelineConfig::force)
        .def_static("from_file",   &PipelineConfig::from_file, py::arg("config_path"),
             "Load pipeline config from a YAML file.")
        .def_static("from_string", &PipelineConfig::from_string, py::arg("yaml_str"),
             "Parse pipeline config from a YAML string.");

    // ── Asset ───────────────────────────────────────────────────────

    py::class_<Asset>(m, "Asset",
             "A 3D asset with meshes, physics, collision, and optional articulation.")
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
        .def_readwrite("content_hash",  &Asset::content_hash)
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
        .def("is_articulated",          &Asset::is_articulated,
             "True if this asset has a kinematic tree (joints, links).")
        .def("all_validations_passed",  &Asset::all_validations_passed,
             "True if every validation check passed.")
        .def("__repr__", [](const Asset& a) {
            return "<Asset '" + a.name + "' meshes="
                   + std::to_string(a.meshes.size())
                   + (a.is_articulated() ? " articulated" : "") + ">";
        });

    // ── Pipeline ────────────────────────────────────────────────────

    py::class_<Pipeline>(m, "Pipeline",
             "Asset processing pipeline. Call build() then run().")
        .def(py::init<PipelineConfig>(), py::arg("config"),
             "Construct a pipeline from a PipelineConfig.")
        .def("build",       &Pipeline::build,
             "Build the stage chain from config. Must be called before run().")
        .def("run", [](Pipeline& p) {
            py::gil_scoped_release release;
            return p.run();
        }, "Run the full pipeline on all discovered assets. Releases the GIL.")
        .def("run_single",      &Pipeline::run_single, py::arg("asset"),
             "Process a single asset through all stages.")
        .def("dry_run",         &Pipeline::dry_run,
             "Preview what would be processed without running stages.")
        .def("discover_assets", &Pipeline::discover_assets,
             "Scan the source directory and return discovered assets.")
        .def("stage_names",     &Pipeline::stage_names,
             "Return the names of all built stages.");
}
