// About: Python bindings for adapter and stage registry introspection.
#include "bind_types.h"
#include "simforge/adapters/adapter.h"
#include "simforge/pipeline/stage.h"
#include "simforge/validators/validator.h"

using namespace simforge;

void bind_adapters(py::module_& m) {

    // Expose registry introspection as module-level functions rather
    // than binding the singleton classes (which hold non-copyable members).

    m.def("list_importers", []() {
        return AdapterManager::instance().list_importers();
    }, "List names of all registered mesh importers.");
    m.def("list_exporters", []() {
        return AdapterManager::instance().list_exporters();
    }, "List names of all registered mesh exporters.");

    m.def("available_stages", []() {
        return StageRegistry::instance().available();
    }, "List names of all registered pipeline stages.");
    m.def("has_stage", [](const std::string& name) {
        return StageRegistry::instance().has(name);
    }, py::arg("name"), "Check if a pipeline stage is registered.");

    m.def("available_validators", []() {
        return ValidatorRegistry::instance().available();
    }, "List names of all registered validators.");
}
