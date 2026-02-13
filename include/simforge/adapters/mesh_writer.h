// Shared mesh file writers for export adapters (OBJ, STL).
#pragma once

#include <filesystem>
#include <vector>

#include "simforge/core/types.h"

namespace simforge::adapters {

namespace fs = std::filesystem;

/// Write a single mesh to Wavefront OBJ format.
/// Returns false on write failure.
bool write_obj(const Mesh& mesh, const fs::path& path);

/// Write multiple meshes to a single OBJ file, adjusting vertex index
/// offsets between meshes. Used for multi-hull collision export.
/// Returns false on write failure.
bool write_obj_multi(const std::vector<Mesh>& meshes, const fs::path& path);

/// Write a mesh to binary STL format with computed face normals.
/// Returns false on write failure.
bool write_binary_stl(const Mesh& mesh, const fs::path& path);

}  // namespace simforge::adapters
