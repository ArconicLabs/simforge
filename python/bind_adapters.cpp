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
    });
    m.def("list_exporters", []() {
        return AdapterManager::instance().list_exporters();
    });

    m.def("available_stages", []() {
        return StageRegistry::instance().available();
    });
    m.def("has_stage", [](const std::string& name) {
        return StageRegistry::instance().has(name);
    }, py::arg("name"));

    m.def("available_validators", []() {
        return ValidatorRegistry::instance().available();
    });
}
