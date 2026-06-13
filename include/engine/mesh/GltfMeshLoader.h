#pragma once

// =============================================================================
// GltfMeshLoader.h — Load glTF 2.0 files (.gltf, .glb)
// =============================================================================
// Uses cgltf library to parse glTF files and convert to MeshData.
// Supports positions, normals, materials, and multiple primitives.
// =============================================================================

#include "engine/mesh/MeshData.h"

#include <string>

namespace ge {

/// Loads a glTF file (both .gltf JSON and .glb binary formats) and returns
/// the mesh data. Throws std::runtime_error on parse or I/O failure.
[[nodiscard]] MeshData loadGltfFile(const std::string& path);

} // namespace ge
