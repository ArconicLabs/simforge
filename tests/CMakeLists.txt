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
)

target_link_libraries(simforge_tests PRIVATE
    simforge_core
    Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(simforge_tests)
