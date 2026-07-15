#pragma once

#include "engine/math/Types.h"
#include "engine/mesh/Material.h"
#include <glm/glm.hpp>
#include <vector>

namespace ge {

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<Material> materials;
};

struct MeshBounds {
    float radius = 1.0f;
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};

    [[nodiscard]] glm::vec3 getHalfExtents() const { return (max - min) * 0.5f; }
    [[nodiscard]] glm::vec3 getCenter() const { return (max + min) * 0.5f; }
};

[[nodiscard]] MeshData makeUnitCubeMesh();
[[nodiscard]] MeshData makeSkyboxMesh(uint32_t materialIndex = 0, float size = 50.0f);
[[nodiscard]] MeshData makeGroundPlaneMesh(uint32_t materialIndex = 0, float size = 20.0f, float y = -1.0f);

void centerMesh(MeshData& mesh);
void orientMeshYUpToZUp(MeshData& mesh);
void flipMeshWinding(MeshData& mesh);
[[nodiscard]] MeshBounds computeMeshBounds(const MeshData& mesh);

} // namespace ge
