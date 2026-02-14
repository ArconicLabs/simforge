// PCA-based primitive shape fitting for collision generation.
// Fits box, sphere, and capsule shapes to meshes with zero external dependencies.
#pragma once

#include "simforge/core/types.h"

namespace simforge::adapters {

struct FittedBox {
    Vec3 center;
    Vec3 axes[3];        // orthonormal principal axes
    Vec3 half_extents;   // half-size along each axis
};

struct FittedSphere {
    Vec3  center;
    float radius;
};

struct FittedCapsule {
    Vec3  p0, p1;        // endpoints of the capsule axis
    float radius;        // perpendicular radius
};

/// Fit an oriented bounding box via PCA eigenvectors.
FittedBox     fit_obb(const Mesh& mesh);

/// Fit a bounding sphere via Ritter's algorithm.
FittedSphere  fit_sphere(const Mesh& mesh);

/// Fit a capsule aligned to the principal axis.
FittedCapsule fit_capsule(const Mesh& mesh);

}  // namespace simforge::adapters
