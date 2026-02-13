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

## Architecture

SimForge has four layers, each with a single responsibility:

1. **CLI** — Parses arguments, sets up logging, dispatches to the pipeline.
2. **Pipeline Engine** — Reads YAML config, constructs a chain of stages, feeds assets through them, collects reports.
3. **Stages** — Self-contained processing steps. Each stage implements `Stage::process(Asset) → Result<Asset>`. Stages are stateless between assets and configured once from YAML.
4. **Adapters** — Thin wrappers around external libraries (Assimp, CoACD, etc.). The `AdapterManager` singleton routes requests to the best available adapter.

See [DESIGN.md](DESIGN.md) for the full design document, [ROADMAP.md](ROADMAP.md) for planned work, and [CHANGELOG.md](CHANGELOG.md) for release history.

```
┌──────────┐   ┌────────────┐   ┌─────────┐   ┌──────────┐   ┌───────────┐   ┌────────┐
│ Ingest   │──▶│ Collision  │──▶│ Physics │──▶│ Optimize │──▶│ Validate  │──▶│ Export │
│          │   │            │   │         │   │          │   │           │   │        │
│ Assimp   │   │ CoACD      │   │ Density │   │ LOD gen  │   │ Watertight│   │ USD    │
│ builtin  │   │ V-HACD     │   │ Explicit│   │ Decimate │   │ Physics   │   │ GLTF   │
│ OBJ/STL  │   │ ConvexHull │   │ Lookup  │   │          │   │ Collision │   │        │
└──────────┘   └────────────┘   └─────────┘   └──────────┘   └───────────┘   └────────┘
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

Global option: `--log-level <trace|debug|info|warn|error>` (default: `info`)

## Configuration

See [simforge.yaml.example](simforge.yaml.example) for a fully commented example config.

A config file has two top-level keys:

- **`pipeline`** — Source/output directories and target export formats.
- **`stages`** — Per-stage settings:
  - `ingest` — Accepted input formats.
  - `collision` — Decomposition method (`coacd`, `convex_hull`, `triangle_mesh`, `primitive`), concavity threshold, max hulls.
  - `physics` — Mass estimation strategy, density, friction, restitution.
  - `optimize` — LOD levels and triangle budgets.
  - `validate` — Which checks to run (watertight, physics plausibility, collision correctness, mesh integrity, scale sanity).
  - `export` — Output formats and catalog generation.

## Structure

```
simforge/
├── .github/workflows/         # CI/CD
│   └── ci.yml                 #   Build matrix + gate job
├── include/simforge/          # Public headers
│   ├── adapters/              #   Adapter interface (MeshImporter/MeshExporter)
│   ├── core/                  #   Core types (Asset, Mesh, Vec3, Physics)
│   ├── pipeline/              #   Pipeline engine, stage interface, builtins
│   └── validators/            #   Validator interface
├── src/                       # Implementation
│   ├── adapters/              #   Builtin + Assimp adapters
│   ├── cli/                   #   CLI entry point (main.cpp)
│   ├── core/                  #   Core type implementations
│   ├── pipeline/              #   Pipeline and stage implementations
│   └── validators/            #   Validator implementations
├── tests/                     # Test suite
│   ├── test_helpers.h         #   Programmatic mesh builders + file writers
│   ├── test_types.cpp         #   Core type tests
│   ├── test_validators.cpp    #   Validator tests
│   ├── test_pipeline.cpp      #   Pipeline config + stage registry tests
│   ├── test_adapters.cpp      #   OBJ/STL importer round-trip tests
│   └── test_integration.cpp   #   End-to-end pipeline tests
├── samples/                   # Sample assets (OBJ, STL, GLTF, URDF, MJCF)
├── CMakeLists.txt             # Build configuration
├── CHANGELOG.md               # Release history
├── DESIGN.md                  # Full design document
├── QUICKSTART.md              # End-to-end walkthrough
├── ROADMAP.md                 # Planned features and phases
└── simforge.yaml.example      # Example pipeline config
```

## CI/CD

GitHub Actions runs on every push to `develop` and on PRs to `main`.

| Job | Description |
|-----|-------------|
| `Minimal / Release` | Build without Assimp, Release mode |
| `Minimal / Debug` | Build without Assimp, Debug mode |
| `Standard / Release` | Build with Assimp, Release mode |
| `Standard / Debug` | Build with Assimp, Debug mode |
| `Gate` | Required status check — passes only when all matrix builds pass |

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

## License

Copyright 2026 ArconicLabs

Apache 2.0 — see [LICENSE](LICENSE) for details.
