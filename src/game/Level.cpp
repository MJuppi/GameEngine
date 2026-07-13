#include "game/Level.h"
#include "engine/asset/AssetManager.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>

namespace ge {
namespace {

void postProcessMesh(MeshData& mesh) {
    centerMesh(mesh);
    orientMeshYUpToZUp(mesh);
    flipMeshWinding(mesh);
}

MeshData makeFallbackMesh() {
    return makeUnitCubeMesh();
}

bool loadObjectMesh(PhysicsMeshObject& object,
                    AssetManager& assetManager,
                    const std::string& levelName,
                    const std::string& levelMeshPath,
                    size_t index) {
    if (object.name.empty()) {
        object.name = levelName;
    }

    const std::string resolvedPath = object.meshPath.empty() ? (index == 0 ? levelMeshPath : std::string{}) : object.meshPath;
    if (resolvedPath.empty()) {
        return false;
    }

    try {
        object.mesh = assetManager.loadMesh(resolvedPath);
        postProcessMesh(object.mesh);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load object '" << object.name << "' for level '" << levelName << "' ("
                  << resolvedPath << "): " << e.what() << "\nFalling back to unit cube.\n";
        object.mesh = makeFallbackMesh();
        postProcessMesh(object.mesh);
        return true;
    }
}

} // namespace

Level::Level(std::string name, std::string meshPath, PhysicsMeshObject object)
    : name_(std::move(name)), meshPath_(std::move(meshPath)) {
    if (!object.meshPath.empty() || meshPath_.empty()) {
        objects_.push_back(std::move(object));
    } else {
        object.meshPath = meshPath_;
        objects_.push_back(std::move(object));
    }
}

/// @brief Gets the mesh for the level.
/// @return
MeshData Level::buildCombinedMesh() const {
    MeshData combinedMesh;

    for (const auto& object : objects_) {
        if (object.mesh.vertices.empty() || object.mesh.indices.empty()) {
            continue;
        }

        // Only bake Static and Visual objects into the level's static mesh.
        // Active (dynamic) objects are rendered individually.
        if (object.type == ObjectType::Active) {
            continue;
        }

        const size_t materialBaseIndex = combinedMesh.materials.size();
        if (object.mesh.materials.empty()) {
            combinedMesh.materials.push_back(makeDefaultMaterial("default"));
        } else {
            combinedMesh.materials.insert(
                combinedMesh.materials.end(),
                object.mesh.materials.begin(),
                object.mesh.materials.end());
        }

        const glm::mat4 objectTransform = object.getWorldTransform();
        const glm::mat3 normalTransform = glm::mat3(glm::transpose(glm::inverse(objectTransform)));
        const auto vertexOffset = static_cast<uint32_t>(combinedMesh.vertices.size());

        for (const auto& vertex : object.mesh.vertices) {
            Vertex transformedVertex{};
            transformedVertex.materialIndex = static_cast<uint32_t>(materialBaseIndex + vertex.materialIndex);

            const glm::vec4 transformedPosition = objectTransform * glm::vec4(
                vertex.position[0],
                vertex.position[1],
                vertex.position[2],
                1.0f);
            transformedVertex.position[0] = transformedPosition.x;
            transformedVertex.position[1] = transformedPosition.y;
            transformedVertex.position[2] = transformedPosition.z;

            const glm::vec3 transformedNormal = normalTransform * glm::vec3(
                vertex.normal[0],
                vertex.normal[1],
                vertex.normal[2]);
            transformedVertex.normal[0] = transformedNormal.x;
            transformedVertex.normal[1] = transformedNormal.y;
            transformedVertex.normal[2] = transformedNormal.z;

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
        if (!objects_.empty()) {
            combinedMesh_ = buildCombinedMesh();
            if (combinedMesh_.vertices.empty() || combinedMesh_.indices.empty()) {
                combinedMesh_ = objects_.front().mesh;
            }
        } else {
            combinedMesh_ = fallbackObject_.mesh;
        }
        combinedMeshDirty_ = false;
    }
    return combinedMesh_;
}

const PhysicsMeshObject& Level::getObject() const {
    if (!objects_.empty()) {
        return objects_.front();
    }
    return fallbackObject_;
}

PhysicsMeshObject& Level::getObject() {
    if (objects_.empty()) {
        objects_.push_back(PhysicsMeshObject{});
    }
    return objects_.front();
}

/// @brief Adds an object to the level.
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

/// @brief Adds an object to the level with specified parameters.
void Level::addObject(std::string name,
                      const glm::mat4& transform,
                      const glm::vec3& location,
                      const glm::vec3& halfExtents,
                      const RigidBodyProps& props,
                      std::string meshPath) {
    PhysicsMeshObject object = ObjectBuilder::createObject(
        std::move(name),
        MeshData{},
        transform,
        location,
        halfExtents,
        props);

    object.meshPath = std::move(meshPath);
    if (object.meshPath.empty() && !meshPath_.empty() && objects_.empty()) {
        object.meshPath = meshPath_;
    }

    objects_.push_back(std::move(object));
    combinedMeshDirty_ = true;
}

/// @brief Loads the level's assets.
/// @param assetManager
void Level::load(AssetManager& assetManager) {
    if (loaded_) return;

    if (objects_.empty()) {
        PhysicsMeshObject fallbackObject;
        fallbackObject.name = name_;
        fallbackObject.meshPath = meshPath_;
        objects_.push_back(std::move(fallbackObject));
    }

    bool loadedAny = false;
    for (size_t index = 0; index < objects_.size(); ++index) {
        auto& object = objects_[index];
        loadedAny |= loadObjectMesh(object, assetManager, name_, meshPath_, index);
    }

    if (!loadedAny && !meshPath_.empty()) {
        try {
            fallbackObject_.mesh = assetManager.loadMesh(meshPath_);
            postProcessMesh(fallbackObject_.mesh);
            loadedAny = true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load level '" << name_ << "' (" << meshPath_
                      << "): " << e.what() << "\nFalling back to unit cube.\n";
            fallbackObject_.mesh = makeFallbackMesh();
            postProcessMesh(fallbackObject_.mesh);
            loadedAny = true;
        }
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
    fallbackObject_.mesh = MeshData{};
    combinedMesh_ = MeshData{};
    combinedMeshDirty_ = true;
    loaded_ = false;
}

} // namespace ge
