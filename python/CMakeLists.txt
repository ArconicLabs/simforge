# About: pybind11 Python extension module for simforge.

pybind11_add_module(simforge_python
    bindings.cpp
    bind_types.cpp
    bind_physics.cpp
    bind_articulation.cpp
    bind_pipeline.cpp
    bind_adapters.cpp
    bind_utilities.cpp
)

target_link_libraries(simforge_python PRIVATE simforge_core)

# Ensure all symbols from the static library are included so that
# adapter and stage registration works correctly.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(simforge_python PRIVATE
        "LINKER:--whole-archive"
        $<TARGET_FILE:simforge_core>
        "LINKER:--no-whole-archive"
    )
endif()

set_target_properties(simforge_python PROPERTIES
    OUTPUT_NAME "simforge"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)
