#include "engine/mesh/MeshData.h"
#include <algorithm>
#include <limits>

namespace ge {

namespace {
Vertex makeV(float x, float y, float z, float nx, float ny, float nz, float u, float v, uint32_t matIdx) {
    return {{x, y, z}, {nx, ny, nz}, {u, v}, matIdx};
}
} // namespace

MeshData makeUnitCubeMesh() {
    MeshData mesh;
    mesh.materials = {
        makeDefaultMaterial("red"),   makeDefaultMaterial("green"),
        makeDefaultMaterial("blue"),  makeDefaultMaterial("yellow"),
        makeDefaultMaterial("cyan"),   makeDefaultMaterial("magenta")
    };
    mesh.materials[0].diffuse = {1, 0, 0};
    mesh.materials[1].diffuse = {0, 1, 0};
    mesh.materials[2].diffuse = {0, 0, 1};
    mesh.materials[3].diffuse = {1, 1, 0};
    mesh.materials[4].diffuse = {0, 1, 1};
    mesh.materials[5].diffuse = {1, 0, 1};

    // 24 vertices, 4 per face for distinct normals
    mesh.vertices = {
        // Front (+Z)
        makeV( 1,  1,  1,  0,  0,  1, 1, 1, 0), makeV(-1,  1,  1,  0,  0,  1, 0, 1, 0),
        makeV(-1, -1,  1,  0,  0,  1, 0, 0, 0), makeV( 1, -1,  1,  0,  0,  1, 1, 0, 0),
        // Back (-Z)
        makeV(-1,  1, -1,  0,  0, -1, 1, 1, 1), makeV( 1,  1, -1,  0,  0, -1, 0, 1, 1),
        makeV( 1, -1, -1,  0,  0, -1, 0, 0, 1), makeV(-1, -1, -1,  0,  0, -1, 1, 0, 1),
        // Top (+Y)
        makeV( 1,  1, -1,  0,  1,  0, 1, 1, 2), makeV(-1,  1, -1,  0,  1,  0, 0, 1, 2),
        makeV(-1,  1,  1,  0,  1,  0, 0, 0, 2), makeV( 1,  1,  1,  0,  1,  0, 1, 0, 2),
        // Bottom (-Y)
        makeV( 1, -1,  1,  0, -1,  0, 1, 1, 3), makeV(-1, -1,  1,  0, -1,  0, 0, 1, 3),
        makeV(-1, -1, -1,  0, -1,  0, 0, 0, 3), makeV( 1, -1, -1,  0, -1,  0, 1, 0, 3),
        // Right (+X)
        makeV( 1,  1, -1,  1,  0,  0, 1, 1, 4), makeV( 1,  1,  1,  1,  0,  0, 0, 1, 4),
        makeV( 1, -1,  1,  1,  0,  0, 0, 0, 4), makeV( 1, -1, -1,  1,  0,  0, 1, 0, 4),
        // Left (-X)
        makeV(-1,  1,  1, -1,  0,  0, 1, 1, 5), makeV(-1,  1, -1, -1,  0,  0, 0, 1, 5),
        makeV(-1, -1, -1, -1,  0,  0, 0, 0, 5), makeV(-1, -1,  1, -1,  0,  0, 1, 0, 5)
    };

    mesh.indices.reserve(36);
    for (uint32_t i = 0; i < 6; ++i) {
        uint32_t base = i * 4;
        mesh.indices.push_back(base + 0);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 3);
        mesh.indices.push_back(base + 0);
    }
    return mesh;
}

MeshData makeSkyboxMesh(uint32_t materialIndex, float size) {
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial()};
    const float h = size * 0.5f;
    mesh.vertices = {
        makeV(-h,-h,-h, 0,0, 1, 0,0, materialIndex), makeV( h,-h,-h, 0,0, 1, 1,0, materialIndex),
        makeV( h, h,-h, 0,0, 1, 1,1, materialIndex), makeV(-h, h,-h, 0,0, 1, 0,1, materialIndex),
        makeV(-h,-h, h, 0,0,-1, 0,0, materialIndex), makeV( h,-h, h, 0,0,-1, 1,0, materialIndex),
        makeV( h, h, h, 0,0,-1, 1,1, materialIndex), makeV(-h, h, h, 0,0,-1, 0,1, materialIndex)
    };
    mesh.indices = { 0,2,1, 2,0,3, 4,6,5, 6,4,7, 0,7,4, 7,0,3, 1,6,5, 6,1,2, 3,6,2, 6,3,7, 0,5,1, 5,0,4 };
    return mesh;
}

MeshData makeGroundPlaneMesh(uint32_t materialIndex, float size, float y) {
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial()};
    const float h = size * 0.5f;
    mesh.vertices = {
        makeV(-h, y, -h, 0, 1, 0, 0, 0, materialIndex), makeV( h, y, -h, 0, 1, 0, 1, 0, materialIndex),
        makeV( h, y,  h, 0, 1, 0, 1, 1, materialIndex), makeV(-h, y,  h, 0, 1, 0, 0, 1, materialIndex)
    };
    mesh.indices = {0, 1, 2, 2, 3, 0};
    return mesh;
}

void centerMesh(MeshData& mesh) {
    if (mesh.vertices.empty()) return;
    glm::vec3 minV{std::numeric_limits<float>::max()}, maxV{std::numeric_limits<float>::lowest()};
    for (const auto& v : mesh.vertices) {
        for (int i=0; i<3; ++i) {
            minV[i] = std::min(minV[i], v.position[i]);
            maxV[i] = std::max(maxV[i], v.position[i]);
        }
    }
    glm::vec3 center = (minV + maxV) * 0.5f;
    for (auto& v : mesh.vertices) for (int i=0; i<3; ++i) v.position[i] -= center[i];
}

void orientMeshYUpToZUp(MeshData& mesh) {
    for (auto& v : mesh.vertices) {
        float y = v.position[1], z = v.position[2], ny = v.normal[1], nz = v.normal[2];
        v.position[1] = z; v.position[2] = -y;
        v.normal[1] = nz; v.normal[2] = -ny;
    }
}

void flipMeshWinding(MeshData& mesh) {
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
}

MeshBounds computeMeshBounds(const MeshData& mesh) {
    MeshBounds bounds;
    if (mesh.vertices.empty()) return bounds;
    float maxDistSq = 0.0f;
    bounds.min = glm::vec3(std::numeric_limits<float>::max());
    bounds.max = glm::vec3(std::numeric_limits<float>::lowest());
    for (const auto& v : mesh.vertices) {
        float dSq = v.position[0]*v.position[0] + v.position[1]*v.position[1] + v.position[2]*v.position[2];
        maxDistSq = std::max(maxDistSq, dSq);
        for (int i=0; i<3; ++i) {
            bounds.min[i] = std::min(bounds.min[i], v.position[i]);
            bounds.max[i] = std::max(bounds.max[i], v.position[i]);
        }
    }
    bounds.radius = std::sqrt(maxDistSq);
    if (bounds.radius < 1e-4f) bounds.radius = 1.0f;
    return bounds;
}

} // namespace ge
