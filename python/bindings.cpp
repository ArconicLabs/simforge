// About: pybind11 module entry point. Registers all C++ builtins and
// delegates binding definitions to per-domain source files.
#include "bind_types.h"

#include "simforge/adapters/adapter.h"
#include "simforge/pipeline/builtin_stages.h"

PYBIND11_MODULE(simforge, m) {
    m.doc() = "SimForge: asset processing pipeline for simulation engineering";

    // Register all built-in adapters and stages so they're available
    // as soon as the module is imported. These are idempotent.
    simforge::adapters::register_builtin_adapters();
    simforge::stages::register_builtin_stages();

    // Bind in dependency order: types first, then things that use them.
    bind_types(m);
    bind_physics(m);
    bind_articulation(m);
    bind_pipeline(m);
    bind_adapters(m);
    bind_utilities(m);
}
