// CoACD convex decomposition adapter for collision generation.
// Only compiled when SIMFORGE_HAS_COACD is defined.
#ifdef SIMFORGE_HAS_COACD

#include "simforge/adapters/adapter.h"

#include <coacd/coacd.h>
#include <spdlog/spdlog.h>

namespace simforge::adapters {

class CoACDGenerator : public CollisionGenerator {
public:
    [[nodiscard]] std::string name() const override { return "coacd"; }

    CollisionMesh generate(const Mesh& visual_mesh,
                           const CollisionParams& params) override {
        // Convert simforge::Mesh → coacd::Mesh (float→double, uint32→int)
        coacd::Mesh input;
        input.vertices.resize(visual_mesh.vertices.size());
        for (size_t i = 0; i < visual_mesh.vertices.size(); i++) {
            input.vertices[i] = {
                static_cast<double>(visual_mesh.vertices[i].x),
                static_cast<double>(visual_mesh.vertices[i].y),
                static_cast<double>(visual_mesh.vertices[i].z)
            };
        }

        input.indices.resize(visual_mesh.faces.size());
        for (size_t i = 0; i < visual_mesh.faces.size(); i++) {
            input.indices[i] = {
                static_cast<int>(visual_mesh.faces[i].v0),
                static_cast<int>(visual_mesh.faces[i].v1),
                static_cast<int>(visual_mesh.faces[i].v2)
            };
        }

        // Run decomposition
        std::vector<coacd::Mesh> parts = coacd::CoACD(
            input,
            static_cast<double>(params.threshold),
            static_cast<int>(params.max_hulls),
            params.preprocess,
            static_cast<int>(params.resolution),
            100  // sample resolution
        );

        // Convert back to CollisionMesh
        CollisionMesh result;
        result.type = CollisionType::ConvexDecomposition;
        result.total_volume = 0.0f;

        for (const auto& part : parts) {
            Mesh hull;
            hull.name = "coacd_hull_" + std::to_string(result.hulls.size());

            hull.vertices.reserve(part.vertices.size());
            for (const auto& v : part.vertices) {
                hull.vertices.push_back({
                    static_cast<float>(v[0]),
                    static_cast<float>(v[1]),
                    static_cast<float>(v[2])
                });
            }

            hull.faces.reserve(part.indices.size());
            for (const auto& idx : part.indices) {
                hull.faces.push_back({
                    static_cast<uint32_t>(idx[0]),
                    static_cast<uint32_t>(idx[1]),
                    static_cast<uint32_t>(idx[2])
                });
            }

            hull.recompute_bounds();
            result.total_volume += hull.compute_volume();
            result.hulls.push_back(std::move(hull));
        }

        spdlog::info("CoACD: decomposed into {} hull(s), total volume: {:.6f}",
                     result.hulls.size(), result.total_volume);

        return result;
    }
};

// Factory function for registration
std::unique_ptr<CollisionGenerator> make_coacd_generator() {
    return std::make_unique<CoACDGenerator>();
}

}  // namespace simforge::adapters

#endif  // SIMFORGE_HAS_COACD
