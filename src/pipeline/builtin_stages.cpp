#include "simforge/pipeline/builtin_stages.h"

#include <fstream>

#include <spdlog/spdlog.h>

namespace simforge::stages {

// ─── Ingest ────────────────────────────────────────────────────────

void IngestStage::configure(const YAML::Node& config) {
    if (auto fmts = config["formats"]) {
        for (const auto& f : fmts) {
            auto s = f.as<std::string>();
            std::transform(s.begin(), s.end(), s.begin(), ::toupper);
            // Map string to SourceFormat
            if      (s == "OBJ")  accepted_formats_.push_back(SourceFormat::OBJ);
            else if (s == "FBX")  accepted_formats_.push_back(SourceFormat::FBX);
            else if (s == "GLTF") accepted_formats_.push_back(SourceFormat::GLTF);
            else if (s == "GLB")  accepted_formats_.push_back(SourceFormat::GLB);
            else if (s == "STL")  accepted_formats_.push_back(SourceFormat::STL);
            else if (s == "STEP") accepted_formats_.push_back(SourceFormat::STEP);
            else if (s == "IGES") accepted_formats_.push_back(SourceFormat::IGES);
            else if (s == "URDF") accepted_formats_.push_back(SourceFormat::URDF);
            else if (s == "MJCF") accepted_formats_.push_back(SourceFormat::MJCF);
            else if (s == "DAE")  accepted_formats_.push_back(SourceFormat::DAE);
            else spdlog::warn("Unknown format in ingest config: {}", s);
        }
    }
}

Result<Asset> IngestStage::process(Asset asset) {
    // Check format filter
    if (!accepted_formats_.empty()) {
        bool accepted = false;
        for (auto fmt : accepted_formats_) {
            if (asset.source_format == fmt) { accepted = true; break; }
        }
        if (!accepted) {
            return Result<Asset>::err({
                name(), asset.id,
                "Format " + format_to_string(asset.source_format) + " not in accepted list"
            }, std::move(asset));
        }
    }

    // Find an importer
    auto& mgr = AdapterManager::instance();
    auto* importer = mgr.find_importer(asset.source_format);
    if (!importer) {
        return Result<Asset>::err({
            name(), asset.id,
            "No importer available for " + format_to_string(asset.source_format)
        }, std::move(asset));
    }

    spdlog::info("  Using importer: {}", importer->name());

    try {
        asset.meshes = importer->import(asset.source_path);
    } catch (const std::exception& e) {
        return Result<Asset>::err({name(), asset.id, e.what()}, std::move(asset));
    }

    if (asset.meshes.empty()) {
        return Result<Asset>::err({
            name(), asset.id, "Importer returned no meshes"
        }, std::move(asset));
    }

    // Recompute bounds
    for (auto& mesh : asset.meshes) {
        mesh.recompute_bounds();
    }

    size_t total_verts = 0, total_tris = 0;
    for (const auto& m : asset.meshes) {
        total_verts += m.vertex_count();
        total_tris  += m.triangle_count();
    }

    spdlog::info("  Ingested {} mesh(es): {} verts, {} tris",
                 asset.meshes.size(), total_verts, total_tris);

    asset.status = AssetStatus::Ingested;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Collision ─────────────────────────────────────────────────────

void CollisionStage::configure(const YAML::Node& config) {
    if (auto m = config["method"]) {
        auto method = m.as<std::string>("coacd");
        if      (method == "coacd" || method == "convex_decomposition")
            params_.method = CollisionType::ConvexDecomposition;
        else if (method == "convex_hull")
            params_.method = CollisionType::ConvexHull;
        else if (method == "triangle_mesh")
            params_.method = CollisionType::TriangleMesh;
        else if (method == "primitive")
            params_.method = CollisionType::Primitive;
    }

    params_.threshold  = config["threshold"].as<float>(params_.threshold);
    params_.max_hulls  = config["max_hulls"].as<uint32_t>(params_.max_hulls);
    params_.resolution = config["resolution"].as<uint32_t>(params_.resolution);
    generator_name_    = config["generator"].as<std::string>(generator_name_);
}

Result<Asset> CollisionStage::process(Asset asset) {
    // Merge all visual meshes into one for collision generation
    // (many assets have multiple sub-meshes)
    Mesh combined;
    combined.name = asset.name + "_combined";
    uint32_t vertex_offset = 0;

    for (const auto& mesh : asset.meshes) {
        for (const auto& v : mesh.vertices)
            combined.vertices.push_back(v);
        for (const auto& n : mesh.normals)
            combined.normals.push_back(n);
        for (const auto& tri : mesh.faces) {
            combined.faces.push_back({
                tri.v0 + vertex_offset,
                tri.v1 + vertex_offset,
                tri.v2 + vertex_offset
            });
        }
        vertex_offset += static_cast<uint32_t>(mesh.vertices.size());
    }
    combined.recompute_bounds();

    // Try adapter-based generator first
    auto& mgr = AdapterManager::instance();
    auto* gen = mgr.find_collision_generator(generator_name_);

    if (gen) {
        spdlog::info("  Using collision generator: {}", gen->name());
        try {
            asset.collision = gen->generate(combined, params_);
        } catch (const std::exception& e) {
            return Result<Asset>::err({name(), asset.id, e.what()}, std::move(asset));
        }
    } else {
        // Fallback: use the visual mesh as a convex hull (naive but functional)
        spdlog::warn("  No collision generator '{}' found, using convex hull fallback",
                     generator_name_);
        CollisionMesh coll;
        coll.type = CollisionType::ConvexHull;
        coll.hulls.push_back(combined);
        coll.total_volume = combined.compute_volume();
        asset.collision = coll;
    }

    spdlog::info("  Generated {} collision hull(s)",
                 asset.collision->hull_count());

    asset.status = AssetStatus::CollisionGenerated;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Physics ───────────────────────────────────────────────────────

void PhysicsStage::configure(const YAML::Node& config) {
    mass_mode_ = config["mass_estimation"].as<std::string>(mass_mode_);

    if (auto mat = config["default_material"]) {
        default_material_.name             = mat["name"].as<std::string>("default");
        default_material_.density          = mat["density"].as<float>(1000.0f);
        default_material_.static_friction  = mat["static_friction"].as<float>(0.5f);
        default_material_.dynamic_friction = mat["dynamic_friction"].as<float>(0.4f);
        default_material_.restitution      = mat["restitution"].as<float>(0.3f);
    } else {
        // Shorthand properties at stage level
        default_material_.density     = config["density"].as<float>(default_material_.density);
        default_material_.static_friction = config["friction"].as<float>(default_material_.static_friction);
        default_material_.dynamic_friction = default_material_.static_friction * 0.8f;
        default_material_.restitution = config["restitution"].as<float>(default_material_.restitution);
    }
}

Result<Asset> PhysicsStage::process(Asset asset) {
    if (asset.meshes.empty()) {
        return Result<Asset>::err({name(), asset.id, "No meshes to annotate"}, std::move(asset));
    }

    if (mass_mode_ == "geometry") {
        // Estimate from the first mesh (or combined)
        auto& primary = asset.meshes[0];
        asset.physics = PhysicsProperties::estimate_from_mesh(primary, default_material_);
        spdlog::info("  Estimated mass: {:.3f} kg (density: {:.0f} kg/m³)",
                     asset.physics->mass, default_material_.density);
    } else if (mass_mode_ == "explicit") {
        // Use values from metadata if present
        PhysicsProperties props;
        props.material = default_material_;
        if (asset.metadata.contains("mass")) {
            props.mass = asset.metadata["mass"].get<float>();
        }
        asset.physics = props;
    } else {
        // Default: just attach material properties
        PhysicsProperties props;
        props.material = default_material_;
        asset.physics = props;
    }

    asset.status = AssetStatus::PhysicsAnnotated;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Optimize ──────────────────────────────────────────────────────

void OptimizeStage::configure(const YAML::Node& config) {
    if (auto levels = config["lod_levels"]) {
        lod_configs_.clear();
        // Expect parallel arrays: lod_levels and max_triangles
        auto max_tris = config["max_triangles"];
        for (size_t i = 0; i < levels.size(); i++) {
            auto level_str = levels[i].as<std::string>();
            LODLevel level = LODLevel::High;
            if      (level_str == "high")   level = LODLevel::High;
            else if (level_str == "medium") level = LODLevel::Medium;
            else if (level_str == "low")    level = LODLevel::Low;

            uint32_t tris = 10000;
            if (max_tris && i < max_tris.size()) {
                tris = max_tris[i].as<uint32_t>();
            }

            lod_configs_.push_back({level, tris});
        }
    }

    generator_name_ = config["generator"].as<std::string>(generator_name_);
}

Result<Asset> OptimizeStage::process(Asset asset) {
    if (lod_configs_.empty()) {
        spdlog::info("  No LOD levels configured, skipping optimization");
        asset.status = AssetStatus::Optimized;
        return Result<Asset>::ok(std::move(asset));
    }

    auto& mgr = AdapterManager::instance();
    auto* lod_gen = mgr.find_lod_generator(generator_name_);

    for (const auto& lod_cfg : lod_configs_) {
        for (const auto& mesh : asset.meshes) {
            LODMesh lod;
            lod.level = lod_cfg.level;
            lod.max_triangles = lod_cfg.max_triangles;

            if (mesh.triangle_count() <= lod_cfg.max_triangles) {
                // Already under budget, use as-is
                lod.mesh = mesh;
            } else if (lod_gen) {
                LODParams params{lod_cfg.max_triangles};
                lod.mesh = lod_gen->decimate(mesh, params);
            } else {
                // No generator available, just copy
                spdlog::warn("  No LOD generator available, copying original mesh");
                lod.mesh = mesh;
            }

            asset.lods.push_back(std::move(lod));
        }
    }

    spdlog::info("  Generated {} LOD variant(s)", asset.lods.size());
    asset.status = AssetStatus::Optimized;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Validate ──────────────────────────────────────────────────────

void ValidateStage::configure(const YAML::Node& config) {
    auto& registry = ValidatorRegistry::instance();

    // Enable specific validators from config
    for (auto it = config.begin(); it != config.end(); ++it) {
        auto key = it->first.as<std::string>();
        if (key == "fail_on_warning") {
            fail_on_warning_ = it->second.as<bool>(false);
            continue;
        }

        // A map value means enabled with sub-config; a scalar is a bool toggle
        bool enabled = it->second.IsMap() || it->second.as<bool>(true);
        if (!enabled) continue;

        try {
            auto validator = registry.create(key);
            if (it->second.IsMap()) {
                validator->configure(it->second);
            }
            validators_.push_back(std::move(validator));
        } catch (...) {
            spdlog::warn("Validator '{}' not found, skipping", key);
        }
    }

    // If no validators configured, use defaults
    if (validators_.empty()) {
        validators_.push_back(std::make_unique<WatertightValidator>());
        validators_.push_back(std::make_unique<MeshIntegrityValidator>());
        validators_.push_back(std::make_unique<PhysicsPlausibilityValidator>());
        validators_.push_back(std::make_unique<CollisionCorrectnessValidator>());
        validators_.push_back(std::make_unique<ScaleSanityValidator>());
    }
}

Result<Asset> ValidateStage::process(Asset asset) {
    bool any_failed = false;

    for (auto& validator : validators_) {
        auto results = validator->validate(asset);
        for (auto& r : results) {
            spdlog::info("  [{}] {} - {}",
                         r.passed ? "PASS" : "FAIL", r.check_name, r.message);
            if (!r.passed) any_failed = true;
            asset.validations.push_back(std::move(r));
        }
    }

    if (any_failed && fail_on_warning_) {
        return Result<Asset>::err({
            name(), asset.id, "One or more validations failed"
        }, std::move(asset));
    }

    asset.status = AssetStatus::Validated;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Export ────────────────────────────────────────────────────────

static std::string format_extension(SourceFormat fmt) {
    switch (fmt) {
        case SourceFormat::USD:  return ".usd";
        case SourceFormat::USDA: return ".usda";
        case SourceFormat::USDC: return ".usdc";
        case SourceFormat::URDF: return ".urdf";
        case SourceFormat::MJCF: return ".xml";
        case SourceFormat::GLTF: return ".gltf";
        case SourceFormat::GLB:  return ".glb";
        case SourceFormat::OBJ:  return ".obj";
        case SourceFormat::STL:  return ".stl";
        case SourceFormat::FBX:  return ".fbx";
        default:                 return ".bin";
    }
}

void ExportStage::configure(const YAML::Node& config) {
    output_dir_       = config["output_dir"].as<std::string>("./sim_ready/");
    write_catalog_    = config["catalog"].as<bool>(true);
    unified_catalog_  = config["unified_catalog"].as<bool>(true);

    targets_.clear();

    // New multi-format config: formats: [usd, urdf, mjcf, gltf]
    if (auto formats = config["formats"]) {
        for (const auto& f : formats) {
            if (f.IsScalar()) {
                // Simple: formats: [usd, urdf]
                auto fmt = parse_format(f.as<std::string>());
                if (fmt != SourceFormat::Unknown) {
                    targets_.push_back({fmt, {}});
                } else {
                    spdlog::warn("Unknown export format: {}", f.as<std::string>());
                }
            } else if (f.IsMap()) {
                // Detailed: formats: [{format: usd, subdir: usd/}, ...]
                auto fmt = parse_format(f["format"].as<std::string>("usd"));
                auto subdir = f["subdir"].as<std::string>("");
                if (fmt != SourceFormat::Unknown) {
                    targets_.push_back({fmt, subdir});
                }
            }
        }
    }

    // Legacy single-format config: format: usd
    if (targets_.empty()) {
        if (auto fmt = config["format"]) {
            auto parsed = parse_format(fmt.as<std::string>("usd"));
            if (parsed != SourceFormat::Unknown) {
                targets_.push_back({parsed, {}});
            }
        }
    }

    // Default: USD if nothing specified
    if (targets_.empty()) {
        targets_.push_back({SourceFormat::USD, {}});
    }

    // Log what we're exporting to
    std::string fmt_list;
    for (const auto& t : targets_) {
        if (!fmt_list.empty()) fmt_list += ", ";
        fmt_list += format_to_string(t.format);
    }
    spdlog::info("Export targets: {}", fmt_list);
}

Result<Asset> ExportStage::export_single(const Asset& asset, const ExportTarget& target) {
    // Determine output path
    fs::path out_dir = output_dir_;
    if (!target.subdir.empty()) {
        out_dir /= target.subdir;
    } else if (targets_.size() > 1) {
        // Auto-subdirectory when multiple formats: sim_ready/usd/, sim_ready/urdf/
        auto fmt_str = format_to_string(target.format);
        std::transform(fmt_str.begin(), fmt_str.end(), fmt_str.begin(), ::tolower);
        out_dir /= fmt_str;
    }
    fs::create_directories(out_dir);

    auto output_path = out_dir / (asset.name + format_extension(target.format));

    auto& mgr = AdapterManager::instance();
    auto* exporter = mgr.find_exporter(target.format);

    if (!exporter) {
        spdlog::warn("  No exporter for {}, writing catalog entry only",
                     format_to_string(target.format));
        return Result<Asset>::ok(asset);
    }

    if (exporter->export_asset(asset, output_path)) {
        spdlog::info("  [{}] → {}", format_to_string(target.format), output_path.string());
        return Result<Asset>::ok(asset);
    } else {
        return Result<Asset>::err({
            name(), asset.id,
            "Export to " + format_to_string(target.format) + " failed for " + asset.name
        }, asset);
    }
}

Result<Asset> ExportStage::process(Asset asset) {
    fs::create_directories(output_dir_);

    // Track all output paths for this asset
    nlohmann::json outputs = nlohmann::json::object();
    bool any_succeeded = false;
    bool any_failed = false;

    for (const auto& target : targets_) {
        auto result = export_single(asset, target);

        if (result.is_ok()) {
            // Record output path
            auto fmt_str = format_to_string(target.format);
            std::transform(fmt_str.begin(), fmt_str.end(), fmt_str.begin(), ::tolower);

            fs::path out_dir = output_dir_;
            if (!target.subdir.empty()) {
                out_dir /= target.subdir;
            } else if (targets_.size() > 1) {
                out_dir /= fmt_str;
            }
            outputs[fmt_str] = (out_dir / (asset.name + format_extension(target.format))).string();
            any_succeeded = true;
        } else {
            spdlog::warn("  Export to {} failed: {}", format_to_string(target.format),
                         result.error().message);
            any_failed = true;
        }
    }

    // Store output paths in asset metadata
    asset.metadata["outputs"] = outputs;

    // Use the first successful output as the primary output_path
    if (!outputs.empty()) {
        asset.output_path = outputs.begin().value().get<std::string>();
    }

    // Write catalog entry
    if (write_catalog_) {
        auto catalog_path = output_dir_ / (asset.name + ".catalog.json");
        auto entry = asset.to_catalog_entry();
        entry["export_targets"] = outputs;

        std::ofstream out(catalog_path);
        out << entry.dump(2);
        spdlog::info("  Catalog → {}", catalog_path.string());
    }

    if (!any_succeeded) {
        return Result<Asset>::err({
            name(), asset.id, "All export formats failed for " + asset.name
        }, std::move(asset));
    }

    asset.status = AssetStatus::Ready;
    return Result<Asset>::ok(std::move(asset));
}

// ─── Auto-register all built-in stages ─────────────────────────────

SIMFORGE_REGISTER_STAGE(IngestStage,    "ingest")
SIMFORGE_REGISTER_STAGE(CollisionStage, "collision")
SIMFORGE_REGISTER_STAGE(PhysicsStage,   "physics")
SIMFORGE_REGISTER_STAGE(OptimizeStage,  "optimize")
SIMFORGE_REGISTER_STAGE(ValidateStage,  "validate")
SIMFORGE_REGISTER_STAGE(ExportStage,    "export")

}  // namespace simforge::stages
