# SimForge — Roadmap

> Last updated: 2026-02-17

SimForge is an open-source pipeline harness that takes raw 3D assets and produces validated, simulation-ready output. This roadmap tracks what's been shipped, what's in progress, and what's planned. For architecture and design rationale, see [DESIGN.md](DESIGN.md).

## Current Status

**Phase 4 (Articulated Asset Support + Python) is complete.** SimForge supports articulated bodies via a KinematicTree type system, URDF/MJCF importers, per-link collision and physics processing, articulation validators, and articulated export across all four formats. Python bindings (pybind11) expose the full API for programmatic use. Parallel asset processing is the next planned item.

---

## Phase 2 — Export Adapters

The highest-leverage work. Without exporters, SimForge is an analysis tool. With them, it's a production pipeline.

| Task | Description | Status |
|------|-------------|--------|
| USDA text exporter | Write ASCII USD without the OpenUSD SDK. Emit visual mesh, collision mesh, and physics properties using UsdGeom + UsdPhysics schemas. | Done |
| URDF exporter | Emit `<robot>` XML with visual/collision meshes as external OBJ files and `<inertial>` blocks. | Done |
| MJCF exporter | Emit `<mujoco>` XML with mesh references, STL assets, and physics material defaults. | Done |
| GLTF exporter | Binary GLTF via tinygltf with PBR materials. No physics data (warn on drop). | Done |
| Regression tests | Per-exporter unit tests and multi-format integration tests. | Done |

## Phase 3 — Collision + LOD Adapters

Replace placeholder fallbacks with real implementations.

| Task | Description | Status |
|------|-------------|--------|
| CoACD adapter | High-quality convex decomposition via the CoACD library. Gated by `SIMFORGE_USE_COACD`. | Done |
| meshoptimizer LOD adapter | Mesh decimation via `meshopt_simplify()`. Always-on, zero external deps. Replaces planned Open3D adapter. | Done |
| Primitive fitting | Fit box/sphere/capsule collision shapes to visual meshes via PCA. No external dependencies. | Done |
| CollisionStage routing | Default generator auto-selected from collision method (primitive → "primitive", coacd → "coacd"). | Done |

## Phase 4 — Articulated Asset Support + Python

| Task | Description | Status |
|------|-------------|--------|
| URDF/MJCF importers | Parse description XML, resolve mesh references, preserve joint hierarchies. | Done |
| KinematicTree type system | Link, Joint, Actuator, Sensor types with `KinematicTree` on Asset for articulated body support. | Done |
| Python bindings | pybind11 wrappers for `Pipeline`, `Asset`, and programmatic stage configuration. | Done |
| Parallel asset processing | Thread pool for per-asset parallelism. Stages must be stateless (they already are). | Planned |

## Phase 5 — Pipeline Polish

| Task | Description | Status |
|------|-------------|--------|
| Incremental processing | Hash-based skip for unchanged assets (make-style dependency tracking). | Planned |
| Material library | Lookup table mapping material names to density/friction values (wood, steel, ceramic, etc.). | Planned |
| Headless thumbnail rendering | Generate a turntable PNG per asset for catalog preview. | Planned |
| CI-friendly output | JUnit/TAP format validation results, structured exit codes. | Planned |
| Importer polish | ASCII STL support, OBJ UV/material parsing, Assimp material extraction. | Planned |

---

## Out of Scope (Public)

The following features are intentionally outside the scope of this open-source project:

- Scene composition and environment generation
- Procedural generation and domain randomization
- Game engine environment importers (Unity/Unreal scene round-tripping)
- Batch variant generation
- Vendor onboarding workflows and acceptance profiles
- Visual diff validation

---

## Resolved Design Questions

Decisions from [DESIGN.md §14](DESIGN.md#14-open-questions) that have been settled:

| Question | Decision |
|----------|----------|
| Kinematic trees | Add a `KinematicTree` struct to `Asset` (option b) for type safety. Phase 4 — Done. |
| Material library | Separate YAML file referenced from main config. Phase 5. |
| Incremental processing | Hash-based (SHA-256 of source file). Phase 5. |
| Plugin loading | Not planned. Adapters are compiled in via CMake flags. |
| Asset identity | Content hash (SHA-256) for deduplication. Implement alongside incremental processing. |
