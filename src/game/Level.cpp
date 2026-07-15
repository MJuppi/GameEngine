#include "game/Level.h"
#include "engine/asset/AssetManager.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>
#include <algorithm>

namespace ge {
namespace {

void postProcessMesh(MeshData& mesh) {
    centerMesh(mesh);
    flipMeshWinding(mesh);
}

MeshData makeFallbackMesh(const glm::vec3& halfExtents) {
    MeshData mesh = makeUnitCubeMesh();
    for (auto& vertex : mesh.vertices) {
        vertex.position[0] *= halfExtents.x;
        vertex.position[1] *= halfExtents.y;
        vertex.position[2] *= halfExtents.z;
    }
    return mesh;
}

bool loadObjectMesh(PhysicsMeshObject& object,
                    AssetManager& assetManager,
                    const std::string& levelName) {
    if (object.name.empty()) {
        object.name = levelName;
    }

    const bool isAuto = (object.halfExtents.x < 0.0f);

    if (object.meshPath.empty()) {
        if (isAuto) {
            object.halfExtents = {0.5f, 0.5f, 0.5f};
        }
        object.mesh = makeFallbackMesh(object.halfExtents);
        postProcessMesh(object.mesh);
        return true;
    }

    try {
        object.mesh = assetManager.loadMesh(object.meshPath);
        postProcessMesh(object.mesh);

        if (isAuto) {
            auto bounds = computeMeshBounds(object.mesh);
            object.halfExtents = bounds.getHalfExtents();
            // Ensure non-zero extents
            object.halfExtents.x = std::max(object.halfExtents.x, 0.01f);
            object.halfExtents.y = std::max(object.halfExtents.y, 0.01f);
            object.halfExtents.z = std::max(object.halfExtents.z, 0.01f);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load object '" << object.name << "' for level '" << levelName << "' ("
                  << object.meshPath << "): " << e.what() << "\nFalling back to unit cube.\n";

        if (isAuto) {
            object.halfExtents = {0.5f, 0.5f, 0.5f};
        }

        object.mesh = makeFallbackMesh(object.halfExtents);
        postProcessMesh(object.mesh);
        return true;
    }
}

} // namespace

Level::Level(std::string name)
    : name_(std::move(name)) {}

MeshData Level::buildCombinedMesh() const {
    MeshData combinedMesh;

    // 1. Collect all unique materials from all objects (including active ones).
    // Prioritize materials with textures to ensure they are at the beginning of the buffer.
    auto addUniqueMaterials = [&](bool onlyWithTextures) {
        for (const auto& object : objects_) {
            for (const auto& mat : object.mesh.materials) {
                const bool hasTex = !mat.texturePath.empty();
                if (hasTex != onlyWithTextures) continue;

                auto it = std::find_if(combinedMesh.materials.begin(), combinedMesh.materials.end(),
                    [&](const Material& m) { return m.name == mat.name; });

                if (it == combinedMesh.materials.end()) {
                    combinedMesh.materials.push_back(mat);
                }
            }
        }
    };

    addUniqueMaterials(true);  // First pass: materials with textures
    addUniqueMaterials(false); // Second pass: materials without textures

    // 2. Bake static and visual objects into the combined mesh.
    for (const auto& object : objects_) {
        if (object.type == ObjectType::Active || object.mesh.vertices.empty()) {
            continue;
        }

        const glm::mat4 objectTransform = object.getWorldTransform();
        const glm::mat3 normalTransform = glm::mat3(glm::transpose(glm::inverse(objectTransform)));
        const auto vertexOffset = static_cast<uint32_t>(combinedMesh.vertices.size());

        for (const auto& vertex : object.mesh.vertices) {
            Vertex transformedVertex{};

            // Map the local material index to the global material list
            if (vertex.materialIndex < object.mesh.materials.size()) {
                const std::string& matName = object.mesh.materials[vertex.materialIndex].name;
                auto it = std::find_if(combinedMesh.materials.begin(), combinedMesh.materials.end(),
                    [&](const Material& m) { return m.name == matName; });

                if (it != combinedMesh.materials.end()) {
                    transformedVertex.materialIndex = static_cast<uint32_t>(std::distance(combinedMesh.materials.begin(), it));
                }
            }

            const glm::vec4 transformedPosition = objectTransform * glm::vec4(
                vertex.position[0], vertex.position[1], vertex.position[2], 1.0f);
            transformedVertex.position[0] = transformedPosition.x;
            transformedVertex.position[1] = transformedPosition.y;
            transformedVertex.position[2] = transformedPosition.z;

            const glm::vec3 transformedNormal = normalTransform * glm::vec3(
                vertex.normal[0], vertex.normal[1], vertex.normal[2]);
            transformedVertex.normal[0] = transformedNormal.x;
            transformedVertex.normal[1] = transformedNormal.y;
            transformedVertex.normal[2] = transformedNormal.z;

            transformedVertex.texCoord[0] = vertex.texCoord[0];
            transformedVertex.texCoord[1] = vertex.texCoord[1];

            combinedMesh.vertices.push_back(transformedVertex);
        }

        for (const auto index : object.mesh.indices) {
            combinedMesh.indices.push_back(vertexOffset + index);
        }
    }

    return combinedMesh;
}

const MeshData& Level::getMesh() const {
    if (combinedMeshDirty_) {
        combinedMesh_ = buildCombinedMesh();

        // If no static geometry was baked, provide a fallback cube but keep the collected materials
        if (combinedMesh_.vertices.empty()) {
            MeshData fallback = makeUnitCubeMesh();
            combinedMesh_.vertices = std::move(fallback.vertices);
            combinedMesh_.indices = std::move(fallback.indices);

            // If we found no materials from any object, use the fallback materials
            if (combinedMesh_.materials.empty()) {
                combinedMesh_.materials = std::move(fallback.materials);
            }
        }

        combinedMeshDirty_ = false;
    }
    return combinedMesh_;
}

void Level::addObject(PhysicsMeshObject object) {
    objects_.push_back(std::move(object));
    combinedMeshDirty_ = true;
}

void Level::addVisual(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents) {
    addObject(ObjectBuilder::createVisual(std::move(name), std::move(meshPath), location, halfExtents));
}

void Level::addStatic(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents, const RigidBodyProps& props) {
    addObject(ObjectBuilder::createStatic(std::move(name), std::move(meshPath), location, halfExtents, props));
}

void Level::addActive(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents, const RigidBodyProps& props) {
    addObject(ObjectBuilder::createActive(std::move(name), std::move(meshPath), location, halfExtents, props));
}

void Level::load(AssetManager& assetManager) {
    if (loaded_) return;

    bool loadedAny = false;
    for (auto& object : objects_) {
        loadedAny |= loadObjectMesh(object, assetManager, name_);
    }

    combinedMeshDirty_ = true;
    std::cout << "Loaded level '" << name_ << "' with " << objects_.size() << " object(s).\n";
    loaded_ = loadedAny;
}

void Level::unload() {
    if (!loaded_) return;
    for (auto& object : objects_) {
        object.mesh = MeshData{};
    }
    combinedMesh_ = MeshData{};
    combinedMeshDirty_ = true;
    loaded_ = false;
}

} // namespace ge
