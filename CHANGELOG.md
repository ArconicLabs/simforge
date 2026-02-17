# Changelog

All notable changes to this project will be documented in this file.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **meshoptimizer LOD adapter**: Always-on mesh decimation via `meshopt_simplify()` quadric-error simplification — zero external dependencies, replaces planned Open3D adapter
- **Primitive fitting collision generator**: PCA-based box/sphere/capsule fitting with automatic tightest-volume selection — zero external dependencies
- **CoACD collision adapter**: High-quality convex decomposition via the CoACD library, gated by `SIMFORGE_USE_COACD` (OFF by default)
- **Primitive fitter header**: `fit_obb()`, `fit_sphere()`, `fit_capsule()` public API in `simforge/adapters/primitive_fitter.h`
- **Dependencies**: meshoptimizer v0.22 via FetchContent (always-on), CoACD v1.0.1 via find_package/FetchContent (optional)
- **Collision + LOD tests**: 8 unit tests covering meshoptimizer decimation, primitive fitting (OBB, sphere, capsule), and primitive generator selection
- **Integration tests**: LOD pipeline test (optimize stage with meshoptimizer) and primitive collision pipeline test
- **USDA exporter**: ASCII USD output with visual meshes, collision scope, and UsdPhysics schema attributes — no OpenUSD SDK required
- **URDF exporter**: Single-link `<robot>` XML with external OBJ meshes in `meshes/` and `<inertial>` blocks via tinyxml2
- **MJCF exporter**: MuJoCo `<mujoco>` XML with STL mesh assets in `assets/`, physics material defaults, and inertial properties via tinyxml2
- **GLTF exporter**: Binary GLB (or text GLTF) output via tinygltf with PBR metallic-roughness materials, buffer packing, and scene graph
- **Shared mesh writers**: `write_obj`, `write_obj_multi`, `write_binary_stl` utilities in `simforge/adapters/mesh_writer.h` for exporter use
- **Exporter factory functions**: `make_usda_exporter()`, `make_urdf_exporter()`, `make_mjcf_exporter()`, `make_gltf_exporter()` in `simforge/adapters/exporters.h`
- **Dependencies**: tinyxml2 (URDF/MJCF XML generation) and tinygltf (GLTF binary export) via FetchContent
- **Exporter tests**: 15 unit tests covering output structure, external mesh files, physics attributes, and round-trip parsing for all 4 formats
- **Integration test**: Full pipeline test exporting to all 4 formats simultaneously with catalog verification
- **KinematicTree type system**: Link, Joint, Actuator, Sensor, and KinematicTree types in `simforge/core/articulation.h` with JSON serialization, index-based lookup, and DOF/tree-structure helpers
- **ArticulatedImporter interface**: New adapter interface for importers that produce a KinematicTree, with factory registration in AdapterManager
- **ArticulationStage**: New pipeline stage for sidecar YAML loading and three-source merge (source file > sidecar > YAML config), placed between ingest and collision
- **URDF importer**: Parses URDF XML — links, joints, transmissions, Gazebo sensor plugins — into KinematicTree via tinyxml2
- **MJCF importer**: Parses MuJoCo XML — nested body hierarchy, actuators, sensors — into KinematicTree via tinyxml2
- **Articulation validators**: KinematicTreeValidator (acyclic, one root, no orphans), ActuatorValidator, SensorValidator, JointLimitsValidator
- **Articulation tests**: 33 tests across 3 files — type unit tests, validator tests, URDF/MJCF importer integration tests with cross-format consistency check
- **Test fixtures**: `simple_arm.urdf`, `simple_arm.xml` (MJCF), `mug.simforge.yaml` (sidecar metadata)

### Changed

- **CollisionStage**: Default generator now auto-routes based on collision method — `primitive` method uses "primitive" generator, `convex_decomposition` uses "coacd" generator
- **OptimizeStage**: LOD generation now uses meshoptimizer instead of copying the original mesh when a generator is available
- **CollisionStage / PhysicsStage**: Process per-link when asset is articulated, single-body path unchanged
- **URDF exporter**: Emits multi-link `<robot>` with `<joint>`, `<transmission>`, and Gazebo `<sensor>` plugins for articulated assets; single-body fallback preserved
- **MJCF exporter**: Emits nested `<body>` tree with `<joint>`, `<actuator>`, and `<sensor>` sections for articulated assets; single-body fallback preserved
- **USDA exporter**: Emits `PhysicsArticulationRootAPI` with per-link Xforms and `PhysicsJoint` prims for articulated assets; also ASCII USD output with visual meshes, collision scope, and UsdPhysics schema attributes — no OpenUSD SDK required
- **GLTF exporter**: Warns when articulation data is dropped (GLTF has no native articulation support)

## [0.1.0] — 2026-02-13

Initial public release. Phase 1 (Core Pipeline + Builtins) is complete.

### Added

- **Core types**: `Asset`, `Mesh`, `Vec3`, `AABB`, `PhysicsProperties`, `CollisionMesh`, `LODMesh`, `PBRMaterial` with full type system
- **Pipeline engine**: YAML config parsing, stage chain construction, sequential execution, JSON report output
- **Stage system**: Stage interface with auto-registration macro (`SIMFORGE_REGISTER_STAGE`) and 6 built-in stages (ingest, collision, physics, optimize, validate, export)
- **Adapter system**: `MeshImporter`, `MeshExporter`, `CollisionGenerator`, `LODGenerator` interfaces with `AdapterManager` singleton routing
- **Importers**: Builtin OBJ importer (vertices, normals, faces, negative index handling, quad triangulation), builtin binary STL importer, optional Assimp adapter for FBX/GLTF/GLB/DAE (gated by `SIMFORGE_USE_ASSIMP`)
- **Validators**: Watertight check, physics plausibility, collision correctness, mesh integrity, scale sanity — all with structured `ValidationResult` output including continuous quality scores
- **CLI**: 6 subcommands (`process`, `inspect`, `init`, `list-stages`, `list-adapters`, `validate`) with `--dry-run`, `--json-report`, `--log-level`, and source/output path overrides
- **Export stage**: Catalog JSON output with full asset metadata, validation results, and export target paths
- **Config system**: `simforge.yaml` with pipeline-level and per-stage configuration, `simforge init` generator, example config at `simforge.yaml.example`
- **Test suite**: Unit tests (core types, validators), adapter tests (OBJ/STL round-trips), pipeline tests (config parsing, stage registry, format detection), integration tests (end-to-end pipeline runs)
- **Sample assets**: OBJ and STL primitives for pipeline testing
- **CI/CD**: GitHub Actions workflow with 4-configuration build matrix (Minimal/Standard x Debug/Release) and gate job
- **Documentation**: DESIGN.md (architecture + design rationale), QUICKSTART.md (end-to-end walkthrough), README.md (CLI reference, config guide, file tree)

### Fixed

- Negative OBJ vertex indices now handled as relative offsets
- Adapter lookup order reversed so last-registered wins (Assimp takes priority over builtins)
- Bounds computed before inertia estimation in `estimate_from_mesh`
- Stages own their status transitions instead of unconditionally overriding to Ready
- Map-type validator configs parsed correctly instead of silently dropped
- Partial asset data preserved through `Result::err` on stage failure
- Pipeline-level `target_formats` injected into export stage config
- `AdapterManager::reset()` added for test isolation
- Missing `<fstream>` includes for `ExportStage` and builtin adapters
- Adapter tests made importer-agnostic for Assimp compatibility

### Changed

- `CollisionMesh::hull_count` replaced with method derived from `hulls.size()`
- Shared `parse_format()` extracted to remove duplicated format parsing logic

[Unreleased]: https://github.com/AeronicLabs/simforge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/AeronicLabs/simforge/releases/tag/v0.1.0
