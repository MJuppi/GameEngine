#pragma once

// =============================================================================
// ObjMeshLoader.h — Load Wavefront .obj + companion .mtl materials
// =============================================================================

#include "engine/mesh/MeshData.h"

#include <string>

namespace ge {

[[nodiscard]] MeshData loadObjFile(const std::string& path);

} // namespace ge
