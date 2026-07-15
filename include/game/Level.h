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
        /// @brief Sets the spawn location of the object in world space.
        /// @param location
        /// @param x 
        /// @param y 
        /// @param z 
        /// @return 
        ObjectBuilderProxy& at(float x, float y, float z) { object.spawnLocation = {x, y, z}; return *this; }
        /// @brief Sets the half extents of the object. If the extents are set to kAutoExtents, the engine will attempt to automatically calculate the bounds based on the mesh data.
        /// @param ext 
        /// @return 
        ObjectBuilderProxy& extents(const glm::vec3& ext) { object.halfExtents = ext; return *this; }
        /// @brief Sets the name of the object.
        /// @param n 
        /// @return 
        ObjectBuilderProxy& name(std::string n) { object.name = std::move(n); return *this; }
        /// @brief Sets the material of the object. This will override any materials defined in the mesh data.
        /// @param mat 
        /// @return 
        ObjectBuilderProxy& material(const Material& mat) { object.mesh.materials = {mat}; return *this; }

        /// @brief Sets the object type to visual (not physical).
        void asVisual() { object.type = ObjectType::Visual; level.addObject(std::move(object)); }
        /// @brief Sets the object type to static (immovable).
        /// @param props 
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
        /// @brief Sets the object type to active (physically simulated).
        /// @param props 
        void asActive(const RigidBodyProps& props = {1.0f}) {
            object.type = ObjectType::Active;
            object.physicsProps = props;
            level.addObject(std::move(object));
        }
    };

    /// @brief Creates a new object in the level with the given mesh path and optional name. If the name is not provided, it will default to the mesh path. The returned ObjectBuilderProxy allows for further configuration of the object, such as setting its location, extents, and type (visual, static, or active).
    /// @param meshPath 
    /// @param name 
    /// @return 
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
