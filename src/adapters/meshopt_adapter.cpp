// LOD generation adapter using meshoptimizer's quadric-error mesh decimation.
#include "simforge/adapters/adapter.h"

#include <meshoptimizer.h>
#include <spdlog/spdlog.h>

namespace simforge::adapters {

class MeshoptimizerDecimator : public LODGenerator {
public:
    [[nodiscard]] std::string name() const override { return "meshoptimizer"; }

    Mesh decimate(const Mesh& source, const LODParams& params) override {
        if (source.faces.empty() || source.vertices.empty()) {
            return source;
        }

        size_t index_count = source.faces.size() * 3;
        size_t vertex_count = source.vertices.size();

        // Pack faces into flat index buffer
        std::vector<uint32_t> indices(index_count);
        for (size_t i = 0; i < source.faces.size(); i++) {
            indices[i * 3 + 0] = source.faces[i].v0;
            indices[i * 3 + 1] = source.faces[i].v1;
            indices[i * 3 + 2] = source.faces[i].v2;
        }

        // Pack vertices into flat float buffer (positions only)
        std::vector<float> positions(vertex_count * 3);
        for (size_t i = 0; i < vertex_count; i++) {
            positions[i * 3 + 0] = source.vertices[i].x;
            positions[i * 3 + 1] = source.vertices[i].y;
            positions[i * 3 + 2] = source.vertices[i].z;
        }

        size_t target_index_count = static_cast<size_t>(params.target_triangles) * 3;

        // If already at or below budget, return as-is
        if (index_count <= target_index_count) {
            return source;
        }

        float target_error = 1.0f - params.quality;

        // Simplify
        std::vector<uint32_t> dest(index_count);
        float result_error = 0.0f;

        size_t result_count = meshopt_simplify(
            dest.data(),
            indices.data(),
            index_count,
            positions.data(),
            vertex_count,
            sizeof(float) * 3,
            target_index_count,
            target_error,
            0,
            &result_error
        );

        dest.resize(result_count);

        // Rebuild mesh from simplified index buffer, reusing source vertices
        Mesh result;
        result.name = source.name + "_lod";
        result.vertices = source.vertices;
        result.normals = source.normals;
        result.uvs = source.uvs;

        result.faces.reserve(result_count / 3);
        for (size_t i = 0; i + 2 < result_count; i += 3) {
            result.faces.push_back({dest[i], dest[i + 1], dest[i + 2]});
        }

        result.recompute_bounds();

        spdlog::debug("meshoptimizer: {} → {} triangles (error: {:.4f})",
                      source.triangle_count(), result.triangle_count(), result_error);

        return result;
    }
};

// Factory function for registration
std::unique_ptr<LODGenerator> make_meshoptimizer_decimator() {
    return std::make_unique<MeshoptimizerDecimator>();
}

}  // namespace simforge::adapters
