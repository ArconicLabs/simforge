# ─── Core Library ───────────────────────────────────────────────────

add_library(simforge_core STATIC
    core/types.cpp
    pipeline/stage.cpp
    pipeline/pipeline.cpp
    pipeline/builtin_stages.cpp
    adapters/adapter.cpp
    adapters/builtin_adapters.cpp
    adapters/mesh_writer.cpp
    adapters/usda_exporter.cpp
    adapters/urdf_exporter.cpp
    adapters/mjcf_exporter.cpp
    adapters/gltf_exporter.cpp
    adapters/meshopt_adapter.cpp
    validators/validator.cpp
)

target_include_directories(simforge_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(simforge_core PUBLIC
    yaml-cpp::yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
    meshoptimizer
)

target_include_directories(simforge_core PRIVATE
    ${tinygltf_SOURCE_DIR}
)

if(SIMFORGE_USE_ASSIMP)
    target_link_libraries(simforge_core PUBLIC assimp::assimp)
endif()

# ─── CLI Executable ─────────────────────────────────────────────────

add_executable(simforge cli/main.cpp)

target_link_libraries(simforge PRIVATE
    simforge_core
    CLI11::CLI11
)

install(TARGETS simforge RUNTIME DESTINATION bin)
install(TARGETS simforge_core ARCHIVE DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
