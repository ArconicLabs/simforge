cmake_minimum_required(VERSION 3.20)
project(simforge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

configure_file(
    "${CMAKE_SOURCE_DIR}/include/simforge/version.h.in"
    "${CMAKE_BINARY_DIR}/include/simforge/version.h"
)

# Options
option(SIMFORGE_BUILD_TESTS    "Build unit tests"       ON)
option(SIMFORGE_BUILD_PYTHON   "Build Python bindings"  OFF)
option(SIMFORGE_USE_ASSIMP     "Enable Assimp adapter"  ON)
option(SIMFORGE_USE_OPENUSD    "Enable OpenUSD adapter"  OFF)
option(SIMFORGE_USE_COACD      "Enable CoACD adapter"   OFF)

# Static libs must be PIC to link into the Python shared module.
if(SIMFORGE_BUILD_PYTHON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# ─── Dependencies ───────────────────────────────────────────────────
include(FetchContent)

# yaml-cpp for config parsing
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
)
FetchContent_MakeAvailable(yaml-cpp)

# spdlog for logging
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.15.0
)
FetchContent_MakeAvailable(spdlog)

# CLI11 for command-line parsing
FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG        v2.4.2
)
FetchContent_MakeAvailable(cli11)

# nlohmann/json for metadata serialization
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(json)

# tinyxml2 for URDF/MJCF XML generation
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        10.0.0
)
set(tinyxml2_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tinyxml2)

# tinygltf for GLTF binary export (header-only)
FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
    GIT_TAG        v2.9.3
)
set(TINYGLTF_BUILD_LOADER_EXAMPLE OFF CACHE BOOL "" FORCE)
set(TINYGLTF_HEADER_ONLY ON CACHE BOOL "" FORCE)
set(TINYGLTF_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tinygltf)

# meshoptimizer for LOD generation (always-on, zero external deps)
FetchContent_Declare(
    meshoptimizer
    GIT_REPOSITORY https://github.com/zeux/meshoptimizer.git
    GIT_TAG        v0.22
)
FetchContent_MakeAvailable(meshoptimizer)

# Optional: Assimp for mesh I/O
if(SIMFORGE_USE_ASSIMP)
    find_package(assimp QUIET)
    if(NOT assimp_FOUND)
        message(STATUS "Assimp not found, fetching from source...")
        FetchContent_Declare(
            assimp
            GIT_REPOSITORY https://github.com/assimp/assimp.git
            GIT_TAG        v5.4.3
        )
        set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(assimp)
    endif()
    add_compile_definitions(SIMFORGE_HAS_ASSIMP)
endif()

# Optional: CoACD for convex decomposition (heavy deps: OpenVDB, Boost, TBB)
if(SIMFORGE_USE_COACD)
    find_package(coacd QUIET)
    if(NOT coacd_FOUND)
        message(STATUS "CoACD not found, fetching from source...")
        cmake_minimum_required(VERSION 3.24)
        FetchContent_Declare(
            coacd
            GIT_REPOSITORY https://github.com/SarahWeiii/CoACD.git
            GIT_TAG        v1.0.1
        )
        FetchContent_MakeAvailable(coacd)
    endif()
    add_compile_definitions(SIMFORGE_HAS_COACD)
endif()

# Optional: pybind11 for Python bindings
if(SIMFORGE_BUILD_PYTHON)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# ─── Core Library ───────────────────────────────────────────────────
add_subdirectory(src)

# ─── Python Bindings ────────────────────────────────────────────────
if(SIMFORGE_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# ─── Tests ──────────────────────────────────────────────────────────
if(SIMFORGE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
