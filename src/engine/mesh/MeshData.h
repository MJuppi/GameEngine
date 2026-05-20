#pragma once

#include "engine/math/Types.h"
#include "engine/mesh/Material.h"

#include <cstdint>
#include <vector>

namespace ge {

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<Material> materials;
};

struct MeshBounds {
    float radius = 1.0f;
};

[[nodiscard]] MeshData makeUnitCubeMesh();

void centerMesh(MeshData& mesh);
void orientMeshYUpToZUp(MeshData& mesh);
void flipMeshWinding(MeshData& mesh);
[[nodiscard]] MeshBounds computeMeshBounds(const MeshData& mesh);

} // namespace ge
