#pragma once
#include "engine/mesh/MeshData.h"
#include "engine/scene/Light.h"
#include "engine/scene/ObjectBuilder.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace ge {

class AssetManager;

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

    struct ObjectBuilderProxy {
        Level& level;
        PhysicsMeshObject object;

        ObjectBuilderProxy& at(const glm::vec3& location) { object.spawnLocation = location; return *this; }
        ObjectBuilderProxy& at(float x, float y, float z) { object.spawnLocation = {x, y, z}; return *this; }
        ObjectBuilderProxy& extents(const glm::vec3& ext) { object.halfExtents = ext; return *this; }
        ObjectBuilderProxy& name(std::string n) { object.name = std::move(n); return *this; }
        ObjectBuilderProxy& material(const Material& mat) { object.mesh.materials = {mat}; return *this; }

        void asVisual() { object.type = ObjectType::Visual; level.addObject(std::move(object)); }
        void asStatic(const RigidBodyProps& props = {}) {
            object.type = ObjectType::Static;
            object.physicsProps = props;
            // Default to static behavior if mass is not explicitly set to non-zero/dynamic
            if (object.physicsProps.mass == 1.0f && !object.physicsProps.isKinematic) {
                object.physicsProps.mass = 0.0f;
                object.physicsProps.isKinematic = true;
                object.physicsProps.useGravity = false;
            }
            level.addObject(std::move(object));
        }
        void asActive(const RigidBodyProps& props = {1.0f}) {
            object.type = ObjectType::Active;
            object.physicsProps = props;
            level.addObject(std::move(object));
        }
    };

    ObjectBuilderProxy add(std::string meshPath, std::string name = "") {
        if (name.empty()) name = meshPath;
        return {*this, PhysicsMeshObject(std::move(name), std::move(meshPath), {0,0,0}, kAutoExtents, ObjectType::Static)};
    }

    void addVisual(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = kAutoExtents);
    void addStatic(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = kAutoExtents, const RigidBodyProps& props = {});
    void addActive(std::string name, std::string meshPath, const glm::vec3& location, const glm::vec3& halfExtents = kAutoExtents, const RigidBodyProps& props = {1.0f});

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
