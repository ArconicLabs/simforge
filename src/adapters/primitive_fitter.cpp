// PCA-based primitive shape fitting and collision generator.
// Fits box, sphere, and capsule to meshes, selects the tightest fit.
#include "simforge/adapters/primitive_fitter.h"
#include "simforge/adapters/adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <spdlog/spdlog.h>

namespace simforge::adapters {

// ─── Math helpers ─────────────────────────────────────────────────

static constexpr float PI = 3.14159265358979323846f;

static Vec3 compute_centroid(const std::vector<Vec3>& vertices) {
    Vec3 sum{};
    for (const auto& v : vertices) {
        sum.x += v.x;
        sum.y += v.y;
        sum.z += v.z;
    }
    float n = static_cast<float>(vertices.size());
    if (n == 0.0f) return {};
    return {sum.x / n, sum.y / n, sum.z / n};
}

static float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static float length(const Vec3& v) {
    return std::sqrt(dot(v, v));
}

static Vec3 normalize(const Vec3& v) {
    float len = length(v);
    if (len < 1e-10f) return {1, 0, 0};
    return v * (1.0f / len);
}

/// 3x3 symmetric covariance matrix stored as 6 unique values.
struct Cov3 {
    float xx, xy, xz, yy, yz, zz;
};

static Cov3 compute_covariance(const std::vector<Vec3>& vertices, const Vec3& centroid) {
    Cov3 c{};
    for (const auto& v : vertices) {
        float dx = v.x - centroid.x;
        float dy = v.y - centroid.y;
        float dz = v.z - centroid.z;
        c.xx += dx * dx;
        c.xy += dx * dy;
        c.xz += dx * dz;
        c.yy += dy * dy;
        c.yz += dy * dz;
        c.zz += dz * dz;
    }
    float n = static_cast<float>(vertices.size());
    if (n > 1.0f) {
        float inv = 1.0f / n;
        c.xx *= inv; c.xy *= inv; c.xz *= inv;
        c.yy *= inv; c.yz *= inv; c.zz *= inv;
    }
    return c;
}

/// Classic Jacobi eigenvalue algorithm for 3x3 symmetric matrices.
/// Returns eigenvalues in vals[3] and column eigenvectors in vecs[3][3].
static void jacobi_eigen_3x3(const Cov3& cov, float vals[3], float vecs[3][3]) {
    // Initialize matrix A
    float a[3][3] = {
        {cov.xx, cov.xy, cov.xz},
        {cov.xy, cov.yy, cov.yz},
        {cov.xz, cov.yz, cov.zz}
    };

    // Initialize eigenvector matrix to identity
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            vecs[i][j] = (i == j) ? 1.0f : 0.0f;

    for (int iter = 0; iter < 50; iter++) {
        // Find largest off-diagonal element
        int p = 0, q = 1;
        float max_val = std::abs(a[0][1]);
        if (std::abs(a[0][2]) > max_val) { p = 0; q = 2; max_val = std::abs(a[0][2]); }
        if (std::abs(a[1][2]) > max_val) { p = 1; q = 2; }

        if (std::abs(a[p][q]) < 1e-10f) break;

        // Compute rotation angle
        float theta = 0.5f * std::atan2(2.0f * a[p][q], a[p][p] - a[q][q]);
        float c = std::cos(theta);
        float s = std::sin(theta);

        // Apply Givens rotation: A' = G^T * A * G
        float a_pp = c * c * a[p][p] + 2 * s * c * a[p][q] + s * s * a[q][q];
        float a_qq = s * s * a[p][p] - 2 * s * c * a[p][q] + c * c * a[q][q];

        a[p][q] = 0.0f;
        a[q][p] = 0.0f;
        a[p][p] = a_pp;
        a[q][q] = a_qq;

        // Update remaining elements
        for (int r = 0; r < 3; r++) {
            if (r == p || r == q) continue;
            float a_rp = c * a[r][p] + s * a[r][q];
            float a_rq = -s * a[r][p] + c * a[r][q];
            a[r][p] = a_rp; a[p][r] = a_rp;
            a[r][q] = a_rq; a[q][r] = a_rq;
        }

        // Accumulate eigenvectors
        for (int r = 0; r < 3; r++) {
            float v_rp = c * vecs[r][p] + s * vecs[r][q];
            float v_rq = -s * vecs[r][p] + c * vecs[r][q];
            vecs[r][p] = v_rp;
            vecs[r][q] = v_rq;
        }
    }

    vals[0] = a[0][0];
    vals[1] = a[1][1];
    vals[2] = a[2][2];

    // Sort by descending eigenvalue
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (vals[j] > vals[i]) {
                std::swap(vals[i], vals[j]);
                for (int k = 0; k < 3; k++)
                    std::swap(vecs[k][i], vecs[k][j]);
            }
        }
    }
}

// ─── Fitting functions ────────────────────────────────────────────

FittedBox fit_obb(const Mesh& mesh) {
    FittedBox box{};
    if (mesh.vertices.empty()) return box;

    Vec3 centroid = compute_centroid(mesh.vertices);
    Cov3 cov = compute_covariance(mesh.vertices, centroid);

    float eigenvalues[3];
    float eigenvectors[3][3];
    jacobi_eigen_3x3(cov, eigenvalues, eigenvectors);

    // Extract axes (columns of eigenvector matrix)
    for (int i = 0; i < 3; i++) {
        box.axes[i] = normalize({eigenvectors[0][i], eigenvectors[1][i], eigenvectors[2][i]});
    }

    // Project vertices onto principal axes to find extents
    float min_proj[3] = { std::numeric_limits<float>::max(),
                          std::numeric_limits<float>::max(),
                          std::numeric_limits<float>::max() };
    float max_proj[3] = { std::numeric_limits<float>::lowest(),
                          std::numeric_limits<float>::lowest(),
                          std::numeric_limits<float>::lowest() };

    for (const auto& v : mesh.vertices) {
        Vec3 d = v - centroid;
        for (int i = 0; i < 3; i++) {
            float proj = dot(d, box.axes[i]);
            min_proj[i] = std::min(min_proj[i], proj);
            max_proj[i] = std::max(max_proj[i], proj);
        }
    }

    box.half_extents = {
        (max_proj[0] - min_proj[0]) * 0.5f,
        (max_proj[1] - min_proj[1]) * 0.5f,
        (max_proj[2] - min_proj[2]) * 0.5f
    };

    // Center is the midpoint of projections in world space
    box.center = centroid;
    for (int i = 0; i < 3; i++) {
        float mid = (min_proj[i] + max_proj[i]) * 0.5f;
        box.center = box.center + box.axes[i] * mid;
    }

    return box;
}

FittedSphere fit_sphere(const Mesh& mesh) {
    FittedSphere sphere{};
    if (mesh.vertices.empty()) return sphere;

    // Ritter's bounding sphere: find farthest point from first vertex,
    // then farthest from that, use those as initial diameter, then expand.

    // Step 1: Find two widely separated points
    const auto& v0 = mesh.vertices[0];
    size_t far1_idx = 0;
    float max_dist2 = 0.0f;
    for (size_t i = 1; i < mesh.vertices.size(); i++) {
        Vec3 d = mesh.vertices[i] - v0;
        float dist2 = dot(d, d);
        if (dist2 > max_dist2) {
            max_dist2 = dist2;
            far1_idx = i;
        }
    }

    size_t far2_idx = 0;
    max_dist2 = 0.0f;
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        Vec3 d = mesh.vertices[i] - mesh.vertices[far1_idx];
        float dist2 = dot(d, d);
        if (dist2 > max_dist2) {
            max_dist2 = dist2;
            far2_idx = i;
        }
    }

    // Step 2: Initial sphere from diameter of far1 → far2
    Vec3 p1 = mesh.vertices[far1_idx];
    Vec3 p2 = mesh.vertices[far2_idx];
    sphere.center = (p1 + p2) * 0.5f;
    sphere.radius = length(p2 - p1) * 0.5f;

    // Step 3: Expand to include all points
    for (const auto& v : mesh.vertices) {
        Vec3 d = v - sphere.center;
        float dist = length(d);
        if (dist > sphere.radius) {
            float new_radius = (sphere.radius + dist) * 0.5f;
            float shift = dist - sphere.radius;
            sphere.center = sphere.center + normalize(d) * (shift * 0.5f);
            sphere.radius = new_radius;
        }
    }

    return sphere;
}

FittedCapsule fit_capsule(const Mesh& mesh) {
    FittedCapsule capsule{};
    if (mesh.vertices.empty()) return capsule;

    Vec3 centroid = compute_centroid(mesh.vertices);
    Cov3 cov = compute_covariance(mesh.vertices, centroid);

    float eigenvalues[3];
    float eigenvectors[3][3];
    jacobi_eigen_3x3(cov, eigenvalues, eigenvectors);

    // Principal axis = eigenvector with largest eigenvalue (already sorted)
    Vec3 axis = normalize({eigenvectors[0][0], eigenvectors[1][0], eigenvectors[2][0]});

    // Project vertices onto principal axis
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    float max_perp_dist = 0.0f;

    for (const auto& v : mesh.vertices) {
        Vec3 d = v - centroid;
        float proj = dot(d, axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);

        // Perpendicular distance from axis
        Vec3 along = axis * proj;
        Vec3 perp = d - along;
        float perp_dist = length(perp);
        max_perp_dist = std::max(max_perp_dist, perp_dist);
    }

    capsule.p0 = centroid + axis * min_proj;
    capsule.p1 = centroid + axis * max_proj;
    capsule.radius = max_perp_dist;

    return capsule;
}

// ─── Volume computations ──────────────────────────────────────────

static float box_volume(const FittedBox& box) {
    return 8.0f * box.half_extents.x * box.half_extents.y * box.half_extents.z;
}

static float sphere_volume(const FittedSphere& sphere) {
    return (4.0f / 3.0f) * PI * sphere.radius * sphere.radius * sphere.radius;
}

static float capsule_volume(const FittedCapsule& capsule) {
    float h = length(capsule.p1 - capsule.p0);
    float r = capsule.radius;
    // Cylinder + two hemispheres = sphere
    return PI * r * r * h + (4.0f / 3.0f) * PI * r * r * r;
}

// ─── Primitive mesh generation ────────────────────────────────────

/// Generate a box mesh from fitted OBB parameters.
static Mesh box_to_mesh(const FittedBox& box) {
    Mesh m;
    m.name = "primitive_box";

    // 8 corners of OBB
    for (int i = 0; i < 8; i++) {
        float sx = (i & 1) ? 1.0f : -1.0f;
        float sy = (i & 2) ? 1.0f : -1.0f;
        float sz = (i & 4) ? 1.0f : -1.0f;
        Vec3 corner = box.center
            + box.axes[0] * (sx * box.half_extents.x)
            + box.axes[1] * (sy * box.half_extents.y)
            + box.axes[2] * (sz * box.half_extents.z);
        m.vertices.push_back(corner);
    }

    // 12 triangles (6 faces × 2)
    m.faces = {
        {0, 2, 1}, {1, 2, 3},  // face 1
        {4, 5, 6}, {5, 7, 6},  // face 2
        {0, 1, 4}, {1, 5, 4},  // face 3
        {2, 6, 3}, {3, 6, 7},  // face 4
        {0, 4, 2}, {2, 4, 6},  // face 5
        {1, 3, 5}, {3, 7, 5},  // face 6
    };

    m.recompute_bounds();
    return m;
}

/// Generate a sphere mesh approximation.
static Mesh sphere_to_mesh(const FittedSphere& sphere) {
    Mesh m;
    m.name = "primitive_sphere";

    const int stacks = 8;
    const int slices = 12;

    // Generate vertices
    m.vertices.push_back({sphere.center.x, sphere.center.y + sphere.radius, sphere.center.z});

    for (int i = 1; i < stacks; i++) {
        float phi = PI * static_cast<float>(i) / static_cast<float>(stacks);
        float y = sphere.radius * std::cos(phi);
        float ring_r = sphere.radius * std::sin(phi);
        for (int j = 0; j < slices; j++) {
            float theta = 2.0f * PI * static_cast<float>(j) / static_cast<float>(slices);
            m.vertices.push_back({
                sphere.center.x + ring_r * std::cos(theta),
                sphere.center.y + y,
                sphere.center.z + ring_r * std::sin(theta)
            });
        }
    }

    m.vertices.push_back({sphere.center.x, sphere.center.y - sphere.radius, sphere.center.z});

    // Top cap
    for (int j = 0; j < slices; j++) {
        m.faces.push_back({0, static_cast<uint32_t>(1 + j),
                           static_cast<uint32_t>(1 + (j + 1) % slices)});
    }

    // Middle bands
    for (int i = 0; i < stacks - 2; i++) {
        for (int j = 0; j < slices; j++) {
            uint32_t a = 1 + i * slices + j;
            uint32_t b = 1 + i * slices + (j + 1) % slices;
            uint32_t c = 1 + (i + 1) * slices + j;
            uint32_t d = 1 + (i + 1) * slices + (j + 1) % slices;
            m.faces.push_back({a, c, b});
            m.faces.push_back({b, c, d});
        }
    }

    // Bottom cap
    uint32_t bot = static_cast<uint32_t>(m.vertices.size() - 1);
    uint32_t last_ring = 1 + (stacks - 2) * slices;
    for (int j = 0; j < slices; j++) {
        m.faces.push_back({bot, static_cast<uint32_t>(last_ring + (j + 1) % slices),
                           static_cast<uint32_t>(last_ring + j)});
    }

    m.recompute_bounds();
    return m;
}

/// Generate a capsule mesh approximation.
static Mesh capsule_to_mesh(const FittedCapsule& capsule) {
    Mesh m;
    m.name = "primitive_capsule";

    Vec3 axis = normalize(capsule.p1 - capsule.p0);
    float h = length(capsule.p1 - capsule.p0);

    // Build local coordinate frame
    Vec3 up = (std::abs(axis.y) < 0.9f) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    Vec3 side1 = normalize(cross(axis, up));
    Vec3 side2 = cross(axis, side1);

    const int slices = 12;
    const int hemi_stacks = 4;

    Vec3 mid0 = capsule.p0;
    Vec3 mid1 = capsule.p1;

    // Top hemisphere
    m.vertices.push_back(mid1 + axis * capsule.radius);

    for (int i = 1; i <= hemi_stacks; i++) {
        float phi = PI * 0.5f * static_cast<float>(i) / static_cast<float>(hemi_stacks);
        float y_off = capsule.radius * std::cos(phi);
        float ring_r = capsule.radius * std::sin(phi);
        for (int j = 0; j < slices; j++) {
            float theta = 2.0f * PI * static_cast<float>(j) / static_cast<float>(slices);
            Vec3 pt = mid1 + axis * y_off
                + side1 * (ring_r * std::cos(theta))
                + side2 * (ring_r * std::sin(theta));
            m.vertices.push_back(pt);
        }
    }

    // Bottom hemisphere
    for (int i = 1; i <= hemi_stacks; i++) {
        float phi = PI * 0.5f + PI * 0.5f * static_cast<float>(i) / static_cast<float>(hemi_stacks);
        float y_off = capsule.radius * std::cos(phi);
        float ring_r = capsule.radius * std::sin(phi);
        for (int j = 0; j < slices; j++) {
            float theta = 2.0f * PI * static_cast<float>(j) / static_cast<float>(slices);
            Vec3 pt = mid0 + axis * y_off
                + side1 * (ring_r * std::cos(theta))
                + side2 * (ring_r * std::sin(theta));
            m.vertices.push_back(pt);
        }
    }

    m.vertices.push_back(mid0 - axis * capsule.radius);

    // Top cap
    for (int j = 0; j < slices; j++) {
        m.faces.push_back({0, static_cast<uint32_t>(1 + j),
                           static_cast<uint32_t>(1 + (j + 1) % slices)});
    }

    // Bands
    int total_rings = hemi_stacks * 2;
    for (int i = 0; i < total_rings - 1; i++) {
        for (int j = 0; j < slices; j++) {
            uint32_t a = 1 + i * slices + j;
            uint32_t b = 1 + i * slices + (j + 1) % slices;
            uint32_t c = 1 + (i + 1) * slices + j;
            uint32_t d = 1 + (i + 1) * slices + (j + 1) % slices;
            m.faces.push_back({a, c, b});
            m.faces.push_back({b, c, d});
        }
    }

    // Bottom cap
    uint32_t bot = static_cast<uint32_t>(m.vertices.size() - 1);
    uint32_t last_ring = 1 + (total_rings - 1) * slices;
    for (int j = 0; j < slices; j++) {
        m.faces.push_back({bot, static_cast<uint32_t>(last_ring + (j + 1) % slices),
                           static_cast<uint32_t>(last_ring + j)});
    }

    m.recompute_bounds();
    return m;
}

// ─── PrimitiveFitter CollisionGenerator ───────────────────────────

class PrimitiveFitter : public CollisionGenerator {
public:
    [[nodiscard]] std::string name() const override { return "primitive"; }

    CollisionMesh generate(const Mesh& visual_mesh,
                           const CollisionParams& /*params*/) override {
        FittedBox box = fit_obb(visual_mesh);
        FittedSphere sphere = fit_sphere(visual_mesh);
        FittedCapsule capsule = fit_capsule(visual_mesh);

        float vol_box = box_volume(box);
        float vol_sphere = sphere_volume(sphere);
        float vol_capsule = capsule_volume(capsule);

        CollisionMesh result;
        result.type = CollisionType::Primitive;

        // Pick tightest fit (smallest volume)
        if (vol_box <= vol_sphere && vol_box <= vol_capsule) {
            result.hulls.push_back(box_to_mesh(box));
            result.total_volume = vol_box;
            spdlog::debug("primitive fitter: selected box (volume: {:.6f})", vol_box);
        } else if (vol_sphere <= vol_capsule) {
            result.hulls.push_back(sphere_to_mesh(sphere));
            result.total_volume = vol_sphere;
            spdlog::debug("primitive fitter: selected sphere (volume: {:.6f})", vol_sphere);
        } else {
            result.hulls.push_back(capsule_to_mesh(capsule));
            result.total_volume = vol_capsule;
            spdlog::debug("primitive fitter: selected capsule (volume: {:.6f})", vol_capsule);
        }

        return result;
    }
};

// Factory function for registration
std::unique_ptr<CollisionGenerator> make_primitive_fitter() {
    return std::make_unique<PrimitiveFitter>();
}

}  // namespace simforge::adapters
