# SimForge — Quickstart Guide

A step-by-step walkthrough from building the tool to processing your first assets.

## Prerequisites

- C++20 compiler (GCC 11+, Clang 14+, MSVC 2022+)
- CMake 3.20+
- Ninja (recommended) or Make

## 1. Build

```bash
cmake -B build -G Ninja
cmake --build build
```

The `simforge` binary is built to `build/src/simforge`.

## 2. Generate a config

```bash
./build/src/simforge init
```
```
[info] Generated config at simforge.yaml
```

This writes a `simforge.yaml` with sensible defaults — source directory, output directory, target formats, and all six stage blocks. Open it to customize paths and parameters.

The generated config looks like this (abbreviated):

```yaml
pipeline:
  source: ./raw_assets/
  output: ./sim_ready/
  target_formats: [usd]

stages:
  ingest:
    formats: [obj, fbx, gltf, glb, stl, step, iges, urdf, mjcf, dae]
  collision:
    method: coacd
    threshold: 0.05
    max_hulls: 32
  physics:
    mass_estimation: geometry
    density: 1000.0
  optimize:
    lod_levels: [high, medium, low]
    max_triangles: [50000, 10000, 2000]
  validate:
    watertight: true
    physics_plausibility: true
  export:
    catalog: true
```

See [simforge.yaml.example](simforge.yaml.example) for the fully commented version with all options.

## 3. Preview the pipeline

Use `--dry-run` to see what would be processed without actually running anything:

```bash
./build/src/simforge process -c simforge.yaml --dry-run
```
```
[info] === DRY RUN ===
[info] Source:  ./raw_assets/
[info] Output:  ./sim_ready/
[info] Targets: USD
[info] Stages:
[info]   ingest → collision → physics → optimize → validate → export
[info] Discovered 3 assets in ./raw_assets/
[info] Would process 3 assets
[info]   gripper (STEP)
[info]   mug (OBJ)
[info]   table (FBX)
```

This tells you exactly which assets were discovered and which stages will run, without touching any files.

## 4. Run the pipeline

```bash
./build/src/simforge process -c simforge.yaml
```

SimForge walks each asset through the configured stages — ingest, collision generation, physics annotation, optimization, validation, and export — then prints a summary report.

Add `--json-report` to write a machine-readable report:

```bash
./build/src/simforge process -c simforge.yaml --json-report --report-path results.json
```

You can also override config paths from the command line:

```bash
./build/src/simforge process -c simforge.yaml -s ./my_assets/ -o ./output/
```

## 5. Inspect individual assets

Check geometry details of any asset file without running the full pipeline:

```bash
./build/src/simforge inspect samples/primitives/cube.obj
```
```
[info] File:   samples/primitives/cube.obj
[info] Format: OBJ
[info] Importer: obj_builtin
[info]   Mesh 'cube': 8 verts, 12 tris
[info]     Bounds: [-0.500, -0.500, -0.500] → [0.500, 0.500, 0.500]
[info]     Watertight: yes
[info]     Volume: 1.000000 m³
[info] Total: 1 mesh(es), 8 verts, 12 tris
```

This is useful for sanity-checking assets before feeding them into the pipeline.

## 6. Validate processed assets

After processing, run standalone validation on the output directory:

```bash
./build/src/simforge validate ./sim_ready/
```

This runs the same checks as the `validate` pipeline stage (watertight, physics plausibility, collision correctness, mesh integrity, scale sanity) but as a standalone pass.

## 7. Explore available stages and adapters

```bash
./build/src/simforge list-stages
```
```
[info] Available pipeline stages:
[info]   ingest
[info]   collision
[info]   physics
[info]   optimize
[info]   validate
[info]   export
```

```bash
./build/src/simforge list-adapters
```
```
[info] Importers:
[info]   obj_builtin
[info]   stl_builtin
[info] Exporters:
```

The available adapters depend on which optional dependencies are enabled. Build with `-DSIMFORGE_USE_ASSIMP=ON` (the default) to get Assimp-backed importers for GLTF, FBX, DAE, and other formats.

## Next steps

- Edit `simforge.yaml` to point `source` at your own asset directory
- Adjust stage parameters (collision method, physics density, LOD budgets) for your use case
- See the [CLI Reference](README.md#cli-reference) for all subcommands and flags
- See the [Configuration](README.md#configuration) section for details on each stage block
- Read [DESIGN.md](DESIGN.md) for the full architecture and design rationale
