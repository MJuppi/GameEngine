#pragma once
#include "engine/mesh/MeshData.h"
#include "engine/scene/Light.h"
#include "engine/scene/ObjectBuilder.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace ge {

class AssetManager; // forward

class Level {
public:
    explicit Level(std::string name);
    ~Level() = default;

    const std::string& getName() const { return name_; }
    const MeshData& getMesh() const;
    MeshData buildCombinedMesh() const;
    const std::vector<PhysicsMeshObject>& getObjects() const { return objects_; }
    std::vector<PhysicsMeshObject>& getObjects() { return objects_; }
    const PointLight& getPointLight() const { return pointLight_; }
    PointLight& getPointLight() { return pointLight_; }
    bool isLoaded() const { return loaded_; }

    void addObject(PhysicsMeshObject object);

    void addVisual(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f});
    void addStatic(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f}, const RigidBodyProps& props = {});
    void addActive(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f}, const RigidBodyProps& props = {1.0f});

    void load(AssetManager& assetManager);
    void unload();

private:
    std::string name_;
    std::vector<PhysicsMeshObject> objects_;
    PointLight pointLight_{};
    mutable MeshData combinedMesh_{};
    mutable bool combinedMeshDirty_ = true;
    bool loaded_ = false;
};

} // namespace ge
