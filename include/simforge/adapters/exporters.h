// Factory functions for export adapters.
#pragma once

#include <memory>

#include "simforge/adapters/adapter.h"

namespace simforge::adapters {

std::unique_ptr<MeshExporter> make_usda_exporter();
std::unique_ptr<MeshExporter> make_urdf_exporter();
std::unique_ptr<MeshExporter> make_mjcf_exporter();
std::unique_ptr<MeshExporter> make_gltf_exporter();

}  // namespace simforge::adapters
