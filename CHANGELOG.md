# Changelog

All notable changes to this project will be documented in this file.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
