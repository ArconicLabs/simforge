# simforge — Design Document

## 1. Problem Statement

Robotics teams building sim-to-real pipelines face a fragmented tooling landscape. The raw inputs are diverse (CAD files from mechanical engineers, mesh exports from 3D artists, URDF/MJCF from roboticists), the target simulators each expect different formats (USD for Isaac Sim, URDF for Gazebo, MJCF for MuJoCo), and the intermediate processing steps — collision mesh generation, physics annotation, LOD optimization, quality validation — each require separate tools with incompatible interfaces.

No single tool orchestrates this end-to-end. Teams either build bespoke scripts that rot, or they manually push assets through a chain of GUIs. Both approaches are unreproducible, error-prone, and don't scale.

simforge is a declarative pipeline harness that takes `raw assets in, sim-ready catalog out` by wiring together existing open-source tools through a single YAML config. Think of it as **Make for 3D assets** — you declare what you want, it figures out how to get there.

## 2. Design Principles

**Harness, not engine.** simforge doesn't reimplement mesh processing, physics simulation, or format conversion. It wraps the best available tool for each job behind a thin adapter interface. When CoACD is the best collision decomposer, we call CoACD. When Assimp is the best mesh importer, we call Assimp. The value is in orchestration, configuration, and validation — not in reinventing libraries.

**Declarative over imperative.** Users describe the desired outcome in YAML. The pipeline resolves which tools to use, in what order, with what parameters. This makes pipelines reproducible, diffable, and version-controllable.

**Format-agnostic.** simforge processes everything through its internal `Asset` representation. Input format and output format are independent concerns handled by adapter layers. A single pipeline run can ingest OBJ+FBX+STEP and export USD+URDF+MJCF simultaneously.

**Fail informatively.** Every stage produces structured diagnostics. When a mesh fails the watertight check or a collision decomposition produces implausible volumes, the report tells you *what* failed, *why*, and *how far off* the measurement was. No silent corruption.

**C++ core, Python surface.** Performance-critical mesh operations run in C++. The CLI is the primary interface. Python bindings (pybind11) expose the pipeline for notebook workflows and integration into existing Python-based robotics toolchains.

## 3. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              CLI (main.cpp)                             │
│          process | inspect | init | list-stages | list-adapters         │
└───────────────────────────────┬──────────────────────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │    Pipeline Engine     │
                    │  (config → DAG → run)  │
                    └───────────┬───────────┘
                                │
        ┌───────────┬───────────┼───────────┬───────────┬───────────┐
        ▼           ▼           ▼           ▼           ▼           ▼
   ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐
   │ Ingest  │ │Collision│ │ Physics │ │Optimize │ │Validate │ │ Export  │
   │  Stage  │ │  Stage  │ │  Stage  │ │  Stage  │ │  Stage  │ │  Stage  │
   └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘
        │           │           │           │           │           │
   ┌────▼────────────▼───────────▼───────────▼───────────▼───────────▼────┐
   │                        Adapter Manager                               │
   │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
   │  │Importers │  │Exporters │  │Collision │  │   LOD    │            │
   │  │          │  │          │  │Generators│  │Generators│            │
   │  │• Assimp  │  │• USDA    │  │• CoACD   │  │• meshopt │            │
   │  │• OBJ     │  │• URDF    │  │• Primitive│  │          │            │
   │  │• STL     │  │• MJCF    │  │• Builtin │  │          │            │
   │  │• URDF    │  │• GLTF    │  │          │  │          │            │
   │  │• MJCF    │  │• OBJ     │  │          │  │          │            │
   │  └──────────┘  └──────────┘  └──────────┘  └──────────┘            │
   └──────────────────────────────────────────────────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │     Core Types        │
                    │  Asset, Mesh, Vec3,   │
                    │  Physics, Collision   │
                    └───────────────────────┘
```

The architecture has four layers, each with a single responsibility:

1. **CLI** — Parses arguments, sets up logging, dispatches to pipeline.
2. **Pipeline Engine** — Reads YAML config, constructs a chain of stages, feeds assets through them, collects reports.
3. **Stages** — Self-contained processing steps. Each stage implements `Stage::process(Asset) → Result<Asset>`. Stages are stateless between assets and configured once from YAML.
4. **Adapters** — Thin wrappers around external libraries. Each adapter implements one of four interfaces: `MeshImporter`, `MeshExporter`, `CollisionGenerator`, or `LODGenerator`. The `AdapterManager` singleton routes requests to the best available adapter.

## 4. Core Data Model

The `Asset` struct is the single unit of work that flows through the pipeline. Every stage reads from it and writes back to it. This avoids intermediate file I/O between stages — everything stays in memory until the final export.

```
Asset
├── identity:    id, name, source_path, source_format
├── geometry:    meshes[]           ← populated by Ingest
│                lods[]             ← populated by Optimize
├── collision:   CollisionMesh?     ← populated by Collision
├── physics:     PhysicsProperties? ← populated by Physics
├── appearance:  materials[]        ← populated by Ingest (when available)
├── status:      AssetStatus enum   ← advanced by each stage
├── validations: ValidationResult[] ← populated by Validate
├── metadata:    json               ← extensible key-value bag
└── output_path: fs::path           ← set by Export
```

### Status progression

Each stage advances the asset through a defined status sequence. Stages check `should_run()` against the current status to ensure correct ordering and idempotency:

```
Raw → Ingested → CollisionGenerated → PhysicsAnnotated → Optimized → Validated → Ready
                                                                                   │
                                                                        (any stage can →) Failed
```

### Why not an ECS / component model?

We considered decomposing assets into separate components (geometry component, physics component, etc.) managed by an entity system. We chose a flat struct instead because:

- The pipeline is strictly sequential, not parallel across components.
- Every stage needs read access to multiple "components" (e.g., Collision reads geometry, Physics reads geometry + collision).
- The asset count per run is hundreds to low thousands, not millions. Cache coherence doesn't matter at this scale.
- A flat struct is simpler to serialize, debug, and pass through the `Result<Asset>` return type.

If we later need parallel processing across *assets* (not components), we parallelize at the pipeline level, not the data model level.

## 5. Adapter System

The adapter layer is where simforge's value as a harness lives. Each adapter is a thin wrapper that translates between simforge's internal `Mesh`/`Asset` types and an external library's API.

### 5.1 Adapter Interfaces

Four interfaces cover all external tool interactions:

```cpp
// Import: file on disk → simforge Mesh objects
class MeshImporter {
    virtual std::string name() const = 0;
    virtual std::vector<SourceFormat> supported_formats() const = 0;
    virtual std::vector<Mesh> import(const fs::path& path) = 0;
};

// Export: simforge Asset → file on disk in target format
class MeshExporter {
    virtual std::string name() const = 0;
    virtual std::vector<SourceFormat> supported_formats() const = 0;
    virtual bool export_asset(const Asset& asset, const fs::path& output_path) = 0;
};

// Collision: visual Mesh → CollisionMesh (set of convex hulls)
class CollisionGenerator {
    virtual std::string name() const = 0;
    virtual CollisionMesh generate(const Mesh& visual, const CollisionParams& params) = 0;
};

// LOD: high-poly Mesh → decimated Mesh
class LODGenerator {
    virtual std::string name() const = 0;
    virtual Mesh decimate(const Mesh& source, const LODParams& params) = 0;
};
```

### 5.2 Adapter Registration

Adapters register with the `AdapterManager` singleton at startup. The manager resolves which adapter to use for a given format or operation:

```cpp
void register_builtin_adapters() {
    auto& mgr = AdapterManager::instance();
    mgr.register_importer(std::make_unique<STLImporter>());
    mgr.register_importer(std::make_unique<OBJImporter>());
#ifdef SIMFORGE_HAS_ASSIMP
    mgr.register_importer(std::make_unique<AssimpImporter>());
#endif
#ifdef SIMFORGE_HAS_COACD
    mgr.register_collision_generator(std::make_unique<CoACDGenerator>());
#endif
    // ...
}
```

The `#ifdef` guards are important. simforge compiles and runs with zero external dependencies beyond its vendored headers (yaml-cpp, spdlog, CLI11, nlohmann/json). Each optional adapter is gated by a CMake flag:

| CMake Flag | Library | What it enables |
|---|---|---|
| `SIMFORGE_USE_ASSIMP` | Assimp 5.4+ | FBX, GLTF, GLB, DAE import |
| `SIMFORGE_USE_COACD` | CoACD | High-quality convex decomposition |
| `SIMFORGE_USE_OPENUSD` | OpenUSD | USD/USDA/USDC import + export |
| (always-on) | meshoptimizer | LOD mesh decimation via quadric-error simplification |
| (always-on) | (built-in) | PCA-based primitive fitting (box/sphere/capsule) |

### 5.3 Adapter Priority & Fallback

When multiple adapters support the same format, the manager uses last-registered-wins priority. Assimp is registered after the builtin OBJ/STL importers, so it takes precedence when available. If Assimp isn't compiled in, the builtins still work for their supported formats.

For collision generation, if no adapter is registered, the CollisionStage falls back to a naive convex hull of the visual mesh. This is wrong for concave objects but ensures the pipeline never hard-fails due to a missing optional dependency.

### 5.4 Writing a New Adapter

To add support for a new external tool, implement one of the four interfaces and register it. Example — the meshoptimizer LOD adapter (actual implementation in `src/adapters/meshopt_adapter.cpp`):

```cpp
// src/adapters/meshopt_adapter.cpp
#include "simforge/adapters/adapter.h"
#include <meshoptimizer.h>

namespace simforge::adapters {

class MeshoptimizerDecimator : public LODGenerator {
public:
    std::string name() const override { return "meshoptimizer"; }

    Mesh decimate(const Mesh& source, const LODParams& params) override {
        // Pack faces into flat index buffer
        std::vector<uint32_t> indices(source.faces.size() * 3);
        for (size_t i = 0; i < source.faces.size(); i++) {
            indices[i * 3 + 0] = source.faces[i].v0;
            indices[i * 3 + 1] = source.faces[i].v1;
            indices[i * 3 + 2] = source.faces[i].v2;
        }

        // Call meshopt_simplify
        std::vector<uint32_t> dest(indices.size());
        float result_error = 0.0f;
        size_t result_count = meshopt_simplify(
            dest.data(), indices.data(), indices.size(),
            /* positions */, source.vertices.size(), sizeof(float) * 3,
            params.target_triangles * 3, 1.0f - params.quality,
            0, &result_error);

        // Rebuild Mesh from simplified index buffer
        Mesh result;
        result.vertices = source.vertices;
        for (size_t i = 0; i + 2 < result_count; i += 3)
            result.faces.push_back({dest[i], dest[i+1], dest[i+2]});
        result.recompute_bounds();
        return result;
    }
};

}  // namespace simforge::adapters
```

The pattern is always: convert in → call library → convert out. The adapter owns the translation; the stage never sees library-specific types.

## 6. Stage System

### 6.1 Stage Interface

Every stage implements three methods:

```cpp
class Stage {
    virtual std::string name() const = 0;
    virtual void configure(const YAML::Node& config) = 0;
    virtual Result<Asset> process(Asset asset) = 0;
    virtual bool should_run(const Asset& asset) const;  // default: true
};
```

The `Result<Asset>` return type forces explicit error handling. A stage either returns the modified asset or a structured error — no exceptions crossing stage boundaries.

### 6.2 Built-in Stages

| Stage | Input Status | Output Status | What It Does |
|---|---|---|---|
| `ingest` | Raw | Ingested | Reads source file via adapter, populates `meshes[]` |
| `collision` | Ingested | CollisionGenerated | Generates collision hulls from visual geometry |
| `physics` | CollisionGenerated | PhysicsAnnotated | Estimates mass, inertia, friction from geometry + material |
| `optimize` | PhysicsAnnotated | Optimized | Generates LOD variants via mesh decimation |
| `validate` | (any) | Validated | Runs all configured validators, populates `validations[]` |
| `export` | Validated | Ready | Writes to one or more target formats + catalog JSON |

### 6.3 Stage Registration

Stages auto-register via a macro that runs at static initialization:

```cpp
SIMFORGE_REGISTER_STAGE(IngestStage,    "ingest")
SIMFORGE_REGISTER_STAGE(CollisionStage, "collision")
// etc.
```

This means adding a new stage requires zero changes to the pipeline engine — just define the class, use the macro, and reference it by name in the YAML config.

### 6.4 Writing a Custom Stage

Example — a stage that centers meshes at the origin (useful for assets exported with arbitrary world-space offsets):

```cpp
class CenterOriginStage : public Stage {
public:
    std::string name() const override { return "center_origin"; }

    void configure(const YAML::Node& config) override {
        center_mode_ = config["mode"].as<std::string>("bounds");
        // "bounds" = center of AABB, "centroid" = average vertex position
    }

    Result<Asset> process(Asset asset) override {
        for (auto& mesh : asset.meshes) {
            mesh.recompute_bounds();
            Vec3 offset = (center_mode_ == "centroid")
                ? compute_centroid(mesh)
                : mesh.bounds.center();

            for (auto& v : mesh.vertices) {
                v = v - offset;
            }
            mesh.recompute_bounds();
        }
        return Result<Asset>::ok(std::move(asset));
    }

private:
    std::string center_mode_;

    Vec3 compute_centroid(const Mesh& m) {
        Vec3 sum{};
        for (const auto& v : m.vertices) {
            sum.x += v.x; sum.y += v.y; sum.z += v.z;
        }
        float n = static_cast<float>(m.vertices.size());
        return n > 0 ? sum * (1.0f / n) : Vec3{};
    }
};

SIMFORGE_REGISTER_STAGE(CenterOriginStage, "center_origin")
```

Then in YAML:

```yaml
stages:
  ingest:
    formats: [obj, stl]
  center_origin:
    mode: bounds
  collision:
    method: coacd
  # ...
```

The pipeline will run `center_origin` between `ingest` and `collision` automatically based on key ordering.

## 7. Validation System

Validators are separate from stages. The `ValidateStage` is a meta-stage that runs all configured validators and collects results. This separation means validators can also be run standalone via `simforge validate <dir>`.

### 7.1 Built-in Validators

| Validator | What It Checks |
|---|---|
| `watertight` | Every edge shared by exactly 2 triangles (manifold mesh) |
| `physics_plausibility` | Mass > 0 for non-static objects; implied density within sane range (10–25,000 kg/m³) |
| `collision_correctness` | Hull count ≤ max; collision/visual volume ratio within tolerance |
| `mesh_integrity` | No degenerate triangles (duplicate indices, out-of-bounds); no empty meshes |
| `scale_sanity` | Bounding box within expected real-world dimensions (catches mm-vs-m unit errors) |

### 7.2 Validation Result

Each check produces a structured result:

```cpp
struct ValidationResult {
    bool        passed;         // hard pass/fail
    std::string check_name;     // "watertight:gripper_mesh"
    std::string message;        // human-readable diagnostic
    float       score;          // 0.0–1.0 continuous quality metric
};
```

The `score` field enables quality thresholds beyond binary pass/fail. A collision volume ratio of 0.95 scores higher than 0.52, even though both might "pass" the configured tolerance.

### 7.3 Writing a Custom Validator

```cpp
class TextureResolutionValidator : public Validator {
public:
    std::string name() const override { return "texture_resolution"; }

    void configure(const YAML::Node& config) override {
        min_resolution_ = config["min_pixels"].as<uint32_t>(512);
    }

    std::vector<ValidationResult> validate(const Asset& asset) override {
        std::vector<ValidationResult> results;
        for (const auto& mat : asset.materials) {
            if (!mat.albedo_map.empty()) {
                auto res = get_image_resolution(mat.albedo_map);
                results.push_back({
                    res >= min_resolution_,
                    "texture_resolution:" + mat.name,
                    std::to_string(res) + "px (min: " + std::to_string(min_resolution_) + "px)",
                    std::clamp(static_cast<float>(res) / static_cast<float>(min_resolution_), 0.f, 1.f)
                });
            }
        }
        return results;
    }

private:
    uint32_t min_resolution_{512};
};
```

## 8. Export System

The export stage supports writing each asset to multiple target formats simultaneously. This is the key to simforge's format-agnostic positioning — one pipeline run produces assets ready for every simulator your team uses.

### 8.1 Multi-Format Output

```yaml
pipeline:
  target_formats: [usd, urdf, mjcf]
```

Produces:

```
sim_ready/
├── usd/
│   ├── gripper.usd
│   └── mug.usd
├── urdf/
│   ├── gripper.urdf
│   └── mug.urdf
├── mjcf/
│   ├── gripper.xml
│   └── mug.xml
├── gripper.catalog.json
└── mug.catalog.json
```

### 8.2 Format Capability Matrix

Not all formats can represent all data. URDF has no concept of PBR materials. MJCF doesn't support LODs. The export adapters must handle this gracefully — emit what the format supports, warn about what's dropped.

```
                        USD     URDF    MJCF    GLTF    OBJ
Visual mesh              ✓       ✓       ✓       ✓      ✓
Collision mesh           ✓       ✓       ✓       ✗      ✗
Physics (mass/inertia)   ✓       ✓       ✓       ✗      ✗
Joint/kinematic tree     ✓       ✓       ✓       ✗      ✗
PBR materials            ✓       ✗       ✗       ✓      partial
LODs                     ✓       ✗       ✗       ✗      ✗
Texture references       ✓       ✓*      ✓*      ✓      ✓
Layered composition      ✓       ✗       ✗       ✗      ✗

* limited to simple texture paths
```

Each exporter should log warnings for data it cannot represent:

```
[warn] [urdf_exporter] Dropping PBR materials for 'mug' (URDF has no material support)
[warn] [mjcf_exporter] Dropping 2 LOD levels for 'gripper' (MJCF has no LOD concept)
```

### 8.3 Exporter Implementation Pattern

Each exporter follows the same structure: iterate the Asset's fields, emit what the format supports, skip what it doesn't.

Example skeleton for the URDF exporter:

```cpp
class URDFExporter : public MeshExporter {
public:
    std::string name() const override { return "urdf"; }

    std::vector<SourceFormat> supported_formats() const override {
        return { SourceFormat::URDF };
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        // URDF is XML. A single-link asset produces:
        //
        // <robot name="gripper">
        //   <link name="base_link">
        //     <visual>
        //       <geometry><mesh filename="meshes/gripper.obj"/></geometry>
        //     </visual>
        //     <collision>
        //       <geometry><mesh filename="meshes/gripper_collision.obj"/></geometry>
        //     </collision>
        //     <inertial>
        //       <mass value="0.5"/>
        //       <inertia ixx="..." ixy="0" ixz="0" iyy="..." iyz="0" izz="..."/>
        //     </inertial>
        //   </link>
        // </robot>

        auto meshes_dir = output_path.parent_path() / "meshes";
        fs::create_directories(meshes_dir);

        // 1. Write visual mesh as OBJ (URDF references external mesh files)
        auto visual_path = meshes_dir / (asset.name + ".obj");
        write_obj(asset.meshes, visual_path);

        // 2. Write collision mesh as OBJ
        if (asset.collision) {
            auto coll_path = meshes_dir / (asset.name + "_collision.obj");
            write_obj(asset.collision->hulls, coll_path);
        }

        // 3. Emit URDF XML
        XMLDocument doc;
        auto* robot = doc.NewElement("robot");
        robot->SetAttribute("name", asset.name.c_str());
        doc.InsertFirstChild(robot);

        auto* link = doc.NewElement("link");
        link->SetAttribute("name", "base_link");
        robot->InsertEndChild(link);

        // Visual
        emit_visual(doc, link, visual_path, asset);

        // Collision
        if (asset.collision)
            emit_collision(doc, link, meshes_dir / (asset.name + "_collision.obj"));

        // Inertial
        if (asset.physics)
            emit_inertial(doc, link, *asset.physics);

        doc.SaveFile(output_path.string().c_str());

        // Warn about dropped data
        if (!asset.materials.empty())
            spdlog::warn("[urdf] Dropping PBR materials for '{}'", asset.name);
        if (!asset.lods.empty())
            spdlog::warn("[urdf] Dropping {} LOD(s) for '{}'", asset.lods.size(), asset.name);

        return true;
    }
};
```

### 8.4 USDA Exporter

USDA (ASCII USD) can be written without the full OpenUSD SDK — it's a human-readable text format. This is valuable because it removes the heaviest optional dependency for the most common output format.

```cpp
class USDAExporter : public MeshExporter {
public:
    std::string name() const override { return "usda_text"; }

    std::vector<SourceFormat> supported_formats() const override {
        return { SourceFormat::USD, SourceFormat::USDA };
    }

    bool export_asset(const Asset& asset, const fs::path& output_path) override {
        std::ofstream out(output_path);

        out << "#usda 1.0\n";
        out << "(\n";
        out << "    defaultPrim = \"" << asset.name << "\"\n";
        out << "    metersPerUnit = 1.0\n";
        out << "    upAxis = \"Z\"\n";
        out << ")\n\n";

        out << "def Xform \"" << asset.name << "\" (\n";
        out << "    kind = \"component\"\n";
        out << ")\n{\n";

        // Visual mesh
        for (size_t i = 0; i < asset.meshes.size(); i++) {
            emit_mesh_prim(out, asset.meshes[i], "visual_" + std::to_string(i), 1);
        }

        // Collision (USD Physics schema)
        if (asset.collision) {
            out << "\n    def Scope \"Collision\"\n    {\n";
            for (size_t i = 0; i < asset.collision->hulls.size(); i++) {
                emit_mesh_prim(out, asset.collision->hulls[i],
                               "hull_" + std::to_string(i), 2);
            }
            out << "    }\n";
        }

        // Physics properties (UsdPhysics schema)
        if (asset.physics) {
            emit_physics(out, *asset.physics);
        }

        out << "}\n";
        return true;
    }

private:
    void emit_mesh_prim(std::ofstream& out, const Mesh& mesh,
                        const std::string& name, int indent_level) {
        std::string indent(indent_level * 4, ' ');

        out << indent << "def Mesh \"" << name << "\"\n";
        out << indent << "{\n";

        // Points
        out << indent << "    point3f[] points = [";
        for (size_t i = 0; i < mesh.vertices.size(); i++) {
            if (i > 0) out << ", ";
            const auto& v = mesh.vertices[i];
            out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        }
        out << "]\n";

        // Face vertex counts (all triangles)
        out << indent << "    int[] faceVertexCounts = [";
        for (size_t i = 0; i < mesh.faces.size(); i++) {
            if (i > 0) out << ", ";
            out << "3";
        }
        out << "]\n";

        // Face vertex indices
        out << indent << "    int[] faceVertexIndices = [";
        for (size_t i = 0; i < mesh.faces.size(); i++) {
            if (i > 0) out << ", ";
            out << mesh.faces[i].v0 << ", " << mesh.faces[i].v1 << ", " << mesh.faces[i].v2;
        }
        out << "]\n";

        // Normals
        if (!mesh.normals.empty()) {
            out << indent << "    normal3f[] normals = [";
            for (size_t i = 0; i < mesh.normals.size(); i++) {
                if (i > 0) out << ", ";
                const auto& n = mesh.normals[i];
                out << "(" << n.x << ", " << n.y << ", " << n.z << ")";
            }
            out << "]\n";
        }

        out << indent << "}\n";
    }

    void emit_physics(std::ofstream& out, const PhysicsProperties& p) {
        // UsdPhysics API
        out << "\n    # Physics\n";
        out << "    float physics:mass = " << p.mass << "\n";
        out << "    point3f physics:centerOfMass = ("
            << p.center_of_mass.x << ", "
            << p.center_of_mass.y << ", "
            << p.center_of_mass.z << ")\n";
        out << "    float physics:diagonalInertia = ("
            << p.inertia_diagonal.x << ", "
            << p.inertia_diagonal.y << ", "
            << p.inertia_diagonal.z << ")\n";

        // Material
        out << "    float physics:staticFriction = " << p.material.static_friction << "\n";
        out << "    float physics:dynamicFriction = " << p.material.dynamic_friction << "\n";
        out << "    float physics:restitution = " << p.material.restitution << "\n";
    }
};
```

## 9. Roadmap

Implementation phases, priorities, and planned features are tracked in [ROADMAP.md](ROADMAP.md).

The sections below document implementation patterns for upcoming work. They are kept here as design reference for contributors — the roadmap is the source of truth for what's planned and when.

## 10. Build & Dependency Management

### 10.1 Dependency Strategy

simforge uses a layered dependency model:

**Always present (vendored via FetchContent):**
- yaml-cpp 0.8 — config parsing
- spdlog 1.15 — structured logging
- CLI11 2.4 — argument parsing
- nlohmann/json 3.11 — metadata serialization
- Catch2 3.7 — testing

These are header-only or small libraries that compile in seconds. They are fetched automatically by CMake.

**Optional (gated by CMake flags, found via `find_package` or FetchContent):**
- Assimp 5.4+ — multi-format mesh I/O
- CoACD — convex decomposition
- Open3D — mesh processing, decimation
- OpenUSD — full USD read/write (heavy dependency, ~2GB)
- tinyxml2 — URDF/MJCF XML writing (lightweight alternative to pulling in full XML parsers)
- tinygltf — GLTF export

**Never a dependency:**
- Boost (too heavy, everything we need is in C++20 stdlib)
- Qt (no GUI)
- NVIDIA CUDA (simforge runs on CPU; simulation runtimes use GPU)

### 10.2 Build Matrix

CI should test these configurations:

| Config | Assimp | CoACD | OpenUSD | Description |
|---|---|---|---|---|
| Minimal | ✗ | ✗ | ✗ | Builtin importers only (OBJ, STL) |
| Standard | ✓ | ✗ | ✗ | Most users — Assimp covers common formats |
| Full | ✓ | ✓ | ✓ | All adapters enabled |

### 10.3 Platform Targets

- Linux x86_64 (primary — this is what robotics teams run)
- macOS arm64 (developer machines)
- Windows x64 (nice to have, not priority)

## 11. Testing Strategy

### 11.1 Test Levels

**Unit tests (Catch2):** Per-function/per-class tests for core types, validators, format detection, config parsing. These run without any test fixtures and complete in <1 second. Currently in `tests/test_types.cpp`, `tests/test_pipeline.cpp`, `tests/test_validators.cpp`.

**Adapter tests:** Per-adapter tests that import a known reference mesh, verify vertex counts, triangle counts, bounds, and watertight status. These require test fixture files checked into `tests/fixtures/`:

```
tests/fixtures/
├── cube.obj          # 8 verts, 12 tris, watertight
├── cube.stl          # same geometry as binary STL
├── open_box.obj      # not watertight (missing bottom face)
├── mug.obj           # concave, good test for collision decomposition
└── tiny_chair.obj    # 0.5mm scale, triggers scale_sanity validator
```

**Integration tests:** End-to-end pipeline runs with a YAML config and fixture directory. Verify that the output directory contains expected files, catalog JSON has expected fields, and the exit code reflects pass/fail correctly.

```cpp
TEST_CASE("End-to-end: OBJ cube → USD + catalog", "[integration]") {
    auto config = PipelineConfig::from_string(R"(
pipeline:
  source: ../tests/fixtures/
  output: /tmp/simforge_test_output/
  target_formats: [usd]
stages:
  ingest:
    formats: [obj]
  collision:
    method: convex_hull
  physics:
    mass_estimation: geometry
    density: 1000.0
  validate:
    watertight: true
    mesh_integrity: true
  export:
    catalog: true
)");

    Pipeline pipeline(std::move(config));
    pipeline.build();
    auto report = pipeline.run();

    REQUIRE(report.passed > 0);
    REQUIRE(report.failed == 0);
    REQUIRE(fs::exists("/tmp/simforge_test_output/cube.catalog.json"));
}
```

**Regression tests:** Golden-file comparisons for export adapters. Export a known asset, diff against a checked-in reference file. Catches unintended format changes.

### 11.2 Test Fixture Assets

We need a small set of reference meshes that exercise different edge cases. These should be created or sourced from permissively-licensed collections:

| Fixture | Properties | Tests |
|---|---|---|
| `cube.obj` | 8v, 12t, watertight, 1m³ | Basic pipeline, volume calc, physics estimation |
| `cube.stl` | Same as cube.obj in binary STL | STL importer correctness |
| `sphere.obj` | ~500v, watertight | Smooth normals, volume ≈ 4.19m³ |
| `mug.obj` | Concave, handle with hole | CoACD decomposition, non-trivial collision |
| `open_box.obj` | Not watertight | Watertight validator failure |
| `tiny_chair.obj` | 0.5mm extents | Scale sanity validator failure |
| `heavy_cube.obj` | Cube + metadata `mass: 50000` | Physics plausibility failure |
| `robot_arm.urdf` | Multi-link, 6 joints | URDF import (Phase 4) |

## 12. Error Handling Philosophy

Errors in simforge fall into three categories:

**Configuration errors** (wrong YAML, missing source directory, unknown stage name): Fail fast with a clear message before any processing starts. Caught during `Pipeline::build()`.

**Per-asset errors** (corrupt file, unsupported format variant, importer crash): Captured in the `Result<Asset>` return type. The pipeline logs the error, marks the asset as `Failed`, and continues to the next asset. One bad file doesn't poison the batch.

**Validation failures** (mesh not watertight, suspicious mass, scale mismatch): These are *not* errors — they're diagnostic results. The asset continues through the pipeline and gets exported with its validation report attached. The `fail_on_warning` config flag controls whether validation failures should be promoted to hard errors.

This three-tier model means simforge is aggressive about reporting problems but conservative about stopping work. The default behavior is: process everything, report everything, let the human decide what to fix.

## 13. Catalog Format

Each exported asset produces a `.catalog.json` sidecar file. This is the machine-readable record of what simforge did to the asset and what it found.

```json
{
  "id": "0",
  "name": "mug",
  "source_path": "./raw_assets/mug.obj",
  "source_format": "OBJ",
  "status": 6,
  "mesh_count": 1,
  "total_triangles": 4820,
  "collision": {
    "type": 1,
    "hull_count": 8,
    "volume": 0.000342
  },
  "physics": {
    "mass": 0.342,
    "center_of_mass": {"x": 0.0, "y": 0.045, "z": 0.0},
    "inertia_diagonal": {"x": 0.000012, "y": 0.000008, "z": 0.000012},
    "material": {
      "name": "ceramic",
      "density": 2300.0,
      "static_friction": 0.6,
      "dynamic_friction": 0.5,
      "restitution": 0.1
    },
    "mass_estimated": true,
    "is_static": false
  },
  "export_targets": {
    "usd": "./sim_ready/usd/mug.usd",
    "urdf": "./sim_ready/urdf/mug.urdf",
    "mjcf": "./sim_ready/mjcf/mug.xml"
  },
  "validations": [
    {"passed": true,  "check": "watertight:mug",         "message": "Mesh is watertight",     "score": 1.0},
    {"passed": true,  "check": "mesh_integrity:mug",     "message": "No degenerate triangles", "score": 1.0},
    {"passed": true,  "check": "scale_sanity:mug",       "message": "Max extent: 0.12m",       "score": 1.0},
    {"passed": true,  "check": "mass_positive",          "message": "Mass is positive",         "score": 1.0},
    {"passed": true,  "check": "density_plausible",      "message": "Implied density: 2300 kg/m³", "score": 1.0},
    {"passed": true,  "check": "hull_count",             "message": "8 hull(s)",                "score": 1.0},
    {"passed": true,  "check": "volume_ratio",           "message": "Ratio: 1.02",              "score": 1.0}
  ]
}
```

Downstream tools can parse these catalogs to:
- Auto-discover sim-ready assets by format
- Filter assets by validation score
- Build asset databases for large-scale simulation farms
- Generate sim environment configs that reference validated assets

## 14. Open Questions

**Asset identity.** Currently `id` is a sequential counter. Should it be a content hash (SHA-256 of source file)? A UUID? A user-provided string? Content hashing enables deduplication and cache invalidation but adds I/O overhead.

**Kinematic trees.** The current `Asset` struct represents individual objects, not articulated robots with joints. URDF and MJCF import/export needs a joint tree representation. Options: (a) store in `metadata` as JSON, (b) add a `JointTree` struct to `Asset`, (c) treat each link as a separate Asset with parent references. Leaning toward (b) for type safety.

**Material library.** The physics stage uses a single `default_material` from config. Real pipelines need a material lookup table (wood, steel, plastic, rubber, ceramic) that maps to density + friction values. Should this be a separate YAML file, embedded in the main config, or an adapter that reads from a database?

**Incremental processing.** Currently the pipeline processes all discovered assets every run. Should we support only processing assets newer than their catalog entry? This is essentially `make`-style dependency tracking. Important for large asset libraries but adds complexity.

**Plugin loading.** Currently adapters are compiled in via CMake flags. Should we support dynamic plugin loading (`dlopen` / shared libraries) so users can add adapters without recompiling simforge? This enables a richer ecosystem but complicates distribution.
