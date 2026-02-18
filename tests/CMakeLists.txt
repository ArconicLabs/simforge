FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

add_executable(simforge_tests
    test_types.cpp
    test_pipeline.cpp
    test_validators.cpp
    test_adapters.cpp
    test_exporters.cpp
    test_collision_lod.cpp
    test_integration.cpp
    test_articulation.cpp
    test_articulation_validators.cpp
    test_articulated_importers.cpp
    test_materials.cpp
    test_parallel.cpp
)

target_link_libraries(simforge_tests PRIVATE
    simforge_core
    Catch2::Catch2WithMain
    tinyxml2::tinyxml2
)

target_include_directories(simforge_tests PRIVATE
    ${tinygltf_SOURCE_DIR}
)

include(CTest)
include(Catch)
catch_discover_tests(simforge_tests)
