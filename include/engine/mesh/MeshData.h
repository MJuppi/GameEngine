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

// Create a large cube used as a simple skybox. Normals are inverted so the
// inside of the cube is visible when rendered. `materialIndex` sets the
// `Vertex::materialIndex` used for this mesh's vertices.
[[nodiscard]] MeshData makeSkyboxMesh(uint32_t materialIndex = 0, float size = 50.0f);

// Create a flat ground plane centered at origin on the given `y` height.
// `size` is the plane width/length, and `materialIndex` selects the material
// index assigned to generated vertices.
[[nodiscard]] MeshData makeGroundPlaneMesh(uint32_t materialIndex = 0, float size = 20.0f, float y = -1.0f);

void centerMesh(MeshData& mesh);
void orientMeshYUpToZUp(MeshData& mesh);
void flipMeshWinding(MeshData& mesh);
[[nodiscard]] MeshBounds computeMeshBounds(const MeshData& mesh);

} // namespace ge
