#include "engine/mesh/MeshData.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ge {

namespace {

Vertex makeVertex(float x, float y, float z, float nx, float ny, float nz, uint32_t materialIndex) {
    // Helper to create a Vertex value with position, normal and material index.
    Vertex v{};
    v.position[0] = x;
    v.position[1] = y;
    v.position[2] = z;
    v.normal[0] = nx;
    v.normal[1] = ny;
    v.normal[2] = nz;
    v.materialIndex = materialIndex;
    return v;
}

} // namespace

MeshData makeUnitCubeMesh() {
    // Produce a simple unit cube mesh centered at origin with one default material.
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial("default")};

    mesh.vertices = {
        makeVertex(-0.5f, -0.5f, -0.5f, 0, 0, -1, 0),
        makeVertex(0.5f, -0.5f, -0.5f, 0, 0, -1, 0),
        makeVertex(0.5f, 0.5f, -0.5f, 0, 0, -1, 0),
        makeVertex(-0.5f, 0.5f, -0.5f, 0, 0, -1, 0),
        makeVertex(-0.5f, -0.5f, 0.5f, 0, 0, 1, 0),
        makeVertex(0.5f, -0.5f, 0.5f, 0, 0, 1, 0),
        makeVertex(0.5f, 0.5f, 0.5f, 0, 0, 1, 0),
        makeVertex(-0.5f, 0.5f, 0.5f, 0, 0, 1, 0),
    };

    mesh.indices = {
        0, 1, 2, 2, 3, 0,
        4, 5, 6, 6, 7, 4,
        0, 4, 7, 7, 3, 0,
        1, 5, 6, 6, 2, 1,
        3, 2, 6, 6, 7, 3,
        0, 1, 5, 5, 4, 0,
    };

    return mesh;
}

MeshData makeSkyboxMesh(uint32_t materialIndex, float size) {
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial("default")};

    const float h = size * 0.5f;

    auto makeV = [&](float x, float y, float z, float nx, float ny, float nz) {
        return makeVertex(x, y, z, nx, ny, nz, materialIndex);
    };

    // Create a cube but invert normals so the inside is visible.
    mesh.vertices = {
        makeV(-h, -h, -h, 0, 0, 1),
        makeV( h, -h, -h, 0, 0, 1),
        makeV( h,  h, -h, 0, 0, 1),
        makeV(-h,  h, -h, 0, 0, 1),
        makeV(-h, -h,  h, 0, 0, -1),
        makeV( h, -h,  h, 0, 0, -1),
        makeV( h,  h,  h, 0, 0, -1),
        makeV(-h,  h,  h, 0, 0, -1),
    };

    // Use same index order but winding will be flipped later if needed.
    mesh.indices = {
        0, 2, 1, 2, 0, 3,
        4, 6, 5, 6, 4, 7,
        0, 7, 4, 7, 0, 3,
        1, 6, 5, 6, 1, 2,
        3, 6, 2, 6, 3, 7,
        0, 5, 1, 5, 0, 4,
    };

    return mesh;
}

MeshData makeGroundPlaneMesh(uint32_t materialIndex, float size, float y) {
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial("default")};

    const float h = size * 0.5f;

    // Simple quad (two triangles) lying on XZ plane at height y.
    Vertex v0 = makeVertex(-h, y, -h, 0, 1, 0, materialIndex);
    Vertex v1 = makeVertex( h, y, -h, 0, 1, 0, materialIndex);
    Vertex v2 = makeVertex( h, y,  h, 0, 1, 0, materialIndex);
    Vertex v3 = makeVertex(-h, y,  h, 0, 1, 0, materialIndex);

    mesh.vertices = {v0, v1, v2, v3};
    mesh.indices = {0, 1, 2, 2, 3, 0};

    return mesh;
}

void centerMesh(MeshData& mesh) {
    // Translate mesh so its bounding box is centered at the origin.
    if (mesh.vertices.empty()) {
        return;
    }

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (const Vertex& v : mesh.vertices) {
        minX = std::min(minX, v.position[0]);
        minY = std::min(minY, v.position[1]);
        minZ = std::min(minZ, v.position[2]);
        maxX = std::max(maxX, v.position[0]);
        maxY = std::max(maxY, v.position[1]);
        maxZ = std::max(maxZ, v.position[2]);
    }

    const float cx = 0.5f * (minX + maxX);
    const float cy = 0.5f * (minY + maxY);
    const float cz = 0.5f * (minZ + maxZ);

    for (Vertex& v : mesh.vertices) {
        v.position[0] -= cx;
        v.position[1] -= cy;
        v.position[2] -= cz;
    }
}

void orientMeshYUpToZUp(MeshData& mesh) {
    // Convert vertex coordinates/normals from Y-up convention to Z-up used by renderer.
    for (Vertex& v : mesh.vertices) {
        const float x = v.position[0];
        const float y = v.position[1];
        const float z = v.position[2];
        v.position[0] = x;
        v.position[1] = z;
        v.position[2] = -y;

        const float nx = v.normal[0];
        const float ny = v.normal[1];
        const float nz = v.normal[2];
        v.normal[0] = nx;
        v.normal[1] = nz;
        v.normal[2] = -ny;
    }
}

void flipMeshWinding(MeshData& mesh) {
    // Flip triangle winding order for each triangle (swap second and third index).
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
    }
}

MeshBounds computeMeshBounds(const MeshData& mesh) {
    // Compute bounding radius from vertex positions; fall back to 1.0 if too small.
    MeshBounds bounds;
    float maxDistSq = 0.0f;

    for (const Vertex& v : mesh.vertices) {
        const float dSq = v.position[0] * v.position[0] + v.position[1] * v.position[1] +
                          v.position[2] * v.position[2];
        maxDistSq = std::max(maxDistSq, dSq);
    }

    bounds.radius = std::sqrt(maxDistSq);
    if (bounds.radius < 1e-4f) {
        bounds.radius = 1.0f;
    }
    return bounds;
}

} // namespace ge
