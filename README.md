# SimForge

### A pipeline harness for simulation engineering.

SimForge takes raw 3D assets (CAD, mesh, URDF, MJCF) and produces a validated, simulation-ready asset catalog — by orchestrating existing open-source tools through a single YAML config.

```
raw_assets/               sim_ready/
├── gripper.step    →     ├── gripper.usd          (simulation-ready USD)
├── mug.obj         →     ├── gripper.catalog.json  (metadata + validation)
├── table.fbx       →     ├── mug.usd
└── robot.urdf      →     └── ...
```

## Quickstart

```bash
cmake -B build -G Ninja
cmake --build build
./build/src/simforge init                              # generate simforge.yaml
./build/src/simforge process -c simforge.yaml --dry-run # preview the pipeline
./build/src/simforge process -c simforge.yaml           # run it
```

See [QUICKSTART.md](QUICKSTART.md) for a full end-to-end walkthrough.

### Python

```bash
pip install .                    # install via pip (requires scikit-build-core)
python3 -c "import simforge as sf; print(sf.available_stages())"
```

Or build manually:

```bash
cmake -B build -G Ninja -DSIMFORGE_BUILD_PYTHON=ON && cmake --build build
PYTHONPATH=build/python python3 -c "import simforge as sf; print(sf.available_stages())"
```

See [QUICKSTART.md § 6](QUICKSTART.md#6-use-the-python-bindings) for full Python usage examples.

## Architecture

SimForge has four layers, each with a single responsibility:

1. **CLI** — Parses arguments, sets up logging, dispatches to the pipeline.
2. **Pipeline Engine** — Reads YAML config, constructs a chain of stages, feeds assets through them, collects reports.
3. **Stages** — Self-contained processing steps. Each stage implements `Stage::process(Asset) → Result<Asset>`. Stages are stateless between assets and configured once from YAML.
4. **Adapters** — Thin wrappers around external libraries (Assimp, CoACD, etc.). The `AdapterManager` singleton routes requests to the best available adapter.

See [DESIGN.md](DESIGN.md) for the full design document, [ROADMAP.md](ROADMAP.md) for planned work, and [CHANGELOG.md](CHANGELOG.md) for release history.

```
┌──────────┐  ┌─────────────┐  ┌──────────┐  ┌─────────┐  ┌────────┐  ┌─────────┐  ┌────────┐
│ Ingest   │─▶│Articulation │─▶│Collision │─▶│ Physics │─▶│Optimize│─▶│Validate │─▶│ Export │
│          │  │  (optional) │  │          │  │         │  │        │  │         │  │        │
│ Assimp   │  │ URDF/MJCF   │  │ CoACD    │  │ Density │  │meshopt │  │Watertight│  │ USDA   │
│ builtin  │  │ YAML/sidecar│  │ Primitive│  │ Explicit│  │Decimate│  │ Physics │  │ URDF   │
│ OBJ/STL  │  │ merge       │  │ConvexHull│  │ Lookup  │  │        │  │Collision│  │ MJCF   │
│ URDF/MJCF│  │             │  │          │  │         │  │        │  │Actuator │  │ GLTF   │
│          │  │             │  │          │  │         │  │        │  │ Sensor  │  │        │
└──────────┘  └─────────────┘  └──────────┘  └─────────┘  └────────┘  └─────────┘  └────────┘
```

## CLI Reference

| Subcommand      | Description                                  |
|-----------------|----------------------------------------------|
| `process`       | Run the asset pipeline                       |
| `inspect`       | Inspect a single asset file                  |
| `list-stages`   | List available pipeline stages               |
| `list-adapters` | List registered importers and exporters      |
| `init`          | Generate a default `simforge.yaml` config    |
| `validate`      | Run validators on processed assets           |

Global options: `--version`, `--log-level <trace|debug|info|warn|error>` (default: `info`)

`process` options: `-j,--threads <N>` (0=auto, 1=sequential), `--force` (reprocess all, ignore cached hashes), `--dry-run`, `--json-report`

## Configuration

See [simforge.yaml.example](simforge.yaml.example) for a fully commented example config.

A config file has two top-level keys:

- **`pipeline`** — Source/output directories, target export formats, `threads` (0=auto, 1=sequential), `force` (ignore cached hashes).
- **`stages`** — Per-stage settings:
  - `ingest` — Accepted input formats (OBJ, STL, FBX, GLTF, URDF, MJCF, ...).
  - `articulation` — Kinematic tree configuration: links, joints, actuators, sensors. Merges data from source files, sidecar metadata (`.simforge.yaml`), and inline YAML config.
  - `collision` — Decomposition method (`convex_hull`, `coacd`, `triangle_mesh`, `primitive`), concavity threshold, max hulls. Default: `convex_hull` (CoACD requires `-DSIMFORGE_USE_COACD=ON`).
  - `physics` — Mass estimation strategy (`geometry`, `explicit`, `lookup`), density, friction, restitution, `material_library` path.
  - `optimize` — LOD levels and triangle budgets.
  - `validate` — Which checks to run (watertight, physics plausibility, collision correctness, mesh integrity, scale sanity, kinematic tree, actuators, sensors, joint limits).
  - `export` — Output formats and catalog generation.

## Structure

```
simforge/
├── .github/workflows/         # CI/CD
│   ├── ci.yml                 #   Build matrix + gate job
│   └── release.yml            #   Tag-triggered release builds
├── include/simforge/          # Public headers
│   ├── adapters/              #   Adapter interfaces, mesh writer, exporter headers
│   ├── core/                  #   Core types, MaterialLibrary, hashing
│   ├── pipeline/              #   Pipeline engine, stage interface, builtins, articulation
│   └── validators/            #   Validator + articulation validator interfaces
├── src/                       # Implementation
│   ├── adapters/              #   Importers (OBJ/STL/URDF/MJCF), exporters, meshopt, CoACD
│   ├── cli/                   #   CLI entry point (main.cpp)
│   ├── core/                  #   Core type + articulation implementations
│   ├── pipeline/              #   Pipeline, stage, and articulation stage implementations
│   └── validators/            #   Validator + articulation validator implementations
├── python/                    # pybind11 Python bindings
├── tests/                     # Test suite
│   ├── fixtures/              #   Test fixture files (URDF, MJCF, sidecar YAML)
│   ├── test_helpers.h         #   Programmatic mesh builders + file writers
│   ├── test_types.cpp         #   Core type tests
│   ├── test_validators.cpp    #   Validator tests
│   ├── test_pipeline.cpp      #   Pipeline config + stage registry tests
│   ├── test_adapters.cpp      #   OBJ/STL importer round-trip tests
│   ├── test_exporters.cpp     #   USDA/URDF/MJCF/GLTF exporter unit tests
│   ├── test_collision_lod.cpp #   Collision + LOD adapter tests
│   ├── test_integration.cpp   #   End-to-end pipeline tests
│   ├── test_articulation.cpp          # Articulation type + KinematicTree tests
│   ├── test_articulation_validators.cpp # Articulation validator tests
│   ├── test_articulated_importers.cpp   # URDF/MJCF importer integration tests
│   ├── test_materials.cpp             # Material library + lookup tests
│   ├── test_parallel.cpp              # Parallel pipeline tests
│   ├── test_incremental.cpp           # Incremental processing tests
│   ├── test_coverage_gaps.cpp         # Articulated export, edge case, and stage tests
│   └── test_bindings.py               # Python binding tests (pytest)
├── data/                      # Default data files
│   └── materials.yaml         #   21 common materials (steel, rubber, ABS, ...)
├── samples/                   # Sample assets (OBJ, STL, GLTF, URDF, MJCF)
├── CMakeLists.txt             # Build configuration
├── pyproject.toml             # Python packaging (pip install .)
├── CHANGELOG.md               # Release history
├── DESIGN.md                  # Full design document
├── QUICKSTART.md              # End-to-end walkthrough
├── ROADMAP.md                 # Planned features and phases
└── simforge.yaml.example      # Example pipeline config
```

## CI/CD

GitHub Actions runs on every push to `develop` and on PRs to `main` or `develop`.

| Job | Description |
|-----|-------------|
| `Minimal / Release` | Build without Assimp, Release mode |
| `Minimal / Debug` | Build without Assimp, Debug mode |
| `Standard / Release` | Build with Assimp, Release mode |
| `Standard / Debug` | Build with Assimp, Debug mode |
| `Python Bindings` | Build with pybind11, run pytest suite (Python 3.11) |
| `Gate` | Required status check — passes only when all jobs pass |

A separate **release workflow** triggers on version tags (`v*`) — builds, tests, packages, and creates a GitHub Release with artifacts.

**Branching workflow:** develop locally on `develop` → push → CI runs → open PR to `main` → Gate must pass to merge.

## Development

### Build options

| CMake Option            | Default | Description              |
|-------------------------|---------|--------------------------|
| `SIMFORGE_BUILD_TESTS`  | `ON`    | Build unit tests         |
| `SIMFORGE_BUILD_PYTHON` | `OFF`   | Build Python bindings    |
| `SIMFORGE_USE_ASSIMP`   | `ON`    | Enable Assimp adapter    |
| `SIMFORGE_USE_OPENUSD`  | `OFF`   | Enable OpenUSD adapter   |
| `SIMFORGE_USE_COACD`    | `OFF`   | Enable CoACD adapter     |

### Running tests

```bash
cmake -B build -G Ninja -DSIMFORGE_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

### Dependencies

All dependencies are fetched automatically via CMake `FetchContent`:

- [yaml-cpp](https://github.com/jbeder/yaml-cpp) — Config parsing
- [spdlog](https://github.com/gabime/spdlog) — Logging
- [CLI11](https://github.com/CLIUtils/CLI11) — Command-line parsing
- [nlohmann/json](https://github.com/nlohmann/json) — Metadata serialization
- [Assimp](https://github.com/assimp/assimp) — Mesh I/O (optional, enabled by default)
- [tinyxml2](https://github.com/leethomason/tinyxml2) — URDF/MJCF XML generation
- [tinygltf](https://github.com/syoyo/tinygltf) — GLTF binary export
- [meshoptimizer](https://github.com/zeux/meshoptimizer) — LOD mesh decimation
- [CoACD](https://github.com/SarahWeiii/CoACD) — Convex decomposition (optional, off by default)
- [pybind11](https://github.com/pybind/pybind11) — Python bindings (optional, off by default)

## License

Copyright 2026 ArconicLabs

Apache 2.0 — see [LICENSE](LICENSE) for details.
