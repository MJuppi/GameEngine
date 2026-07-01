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
    explicit Level(std::string name, std::string meshPath = {}, PhysicsMeshObject object = {});
    ~Level() = default;

    const std::string& getName() const { return name_; }
    const std::string& getMeshPath() const { return meshPath_; }
    const MeshData& getMesh() const;
    MeshData buildCombinedMesh() const;
    const std::vector<PhysicsMeshObject>& getObjects() const { return objects_; }
    std::vector<PhysicsMeshObject>& getObjects() { return objects_; }
    const PointLight& getPointLight() const { return pointLight_; }
    PointLight& getPointLight() { return pointLight_; }
    const PhysicsMeshObject& getObject() const;
    PhysicsMeshObject& getObject();
    bool isLoaded() const { return loaded_; }

    void addObject(PhysicsMeshObject object);
    void addObject(std::string name,
                   const glm::mat4& transform,
                   const glm::vec3& translation,
                   const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f},
                   const RigidBodyProps& props = {},
                   std::string meshPath = {});
    void load(AssetManager& assetManager);
    void unload();

private:
    std::string name_;
    std::string meshPath_;
    std::vector<PhysicsMeshObject> objects_;
    PointLight pointLight_{};
    mutable PhysicsMeshObject fallbackObject_{};
    mutable MeshData combinedMesh_{};
    mutable bool combinedMeshDirty_ = true;
    bool loaded_ = false;
};

} // namespace ge
