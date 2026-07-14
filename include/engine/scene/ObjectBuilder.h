#pragma once

#include "engine/mesh/MeshData.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/RigidBody.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>

namespace ge {

enum class ObjectType {
    Visual, // No physics, baked into level mesh
    Static, // Static physics (mass 0), baked into level mesh
    Active  // Dynamic physics (mass > 0), rendered individually
};

struct PhysicsMeshObject {
    std::string name;
    std::string meshPath;
    MeshData mesh;
    glm::mat4 transform{1.0f};
    glm::vec3 halfExtents{0.5f, 0.5f, 0.5f};
    glm::vec3 spawnLocation{0.0f, 0.0f, 0.0f};
    RigidBodyProps physicsProps;
    ObjectType type = ObjectType::Static;

    PhysicsMeshObject() = default;

    PhysicsMeshObject(std::string n, std::string path, glm::vec3 loc, glm::vec3 ext, ObjectType t, RigidBodyProps props = {})
            : name(std::move(n)), meshPath(std::move(path)), halfExtents(ext), spawnLocation(loc), physicsProps(props), type(t) {}

    [[nodiscard]] glm::mat4 getWorldTransform() const {
        return glm::translate(glm::mat4(1.0f), spawnLocation) * transform;
    }

    [[nodiscard]] bool hasPhysicsBody() const noexcept {
        return type != ObjectType::Visual;
    }
};

class ObjectBuilder {
public:
    static PhysicsMeshObject createVisual(
        std::string name,
        std::string meshPath,
        const glm::vec3& location = {0.0f, 0.0f, 0.0f},
        const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f});

    static PhysicsMeshObject createStatic(
        std::string name,
        std::string meshPath,
        const glm::vec3& location = {0.0f, 0.0f, 0.0f},
        const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f},
        const RigidBodyProps& props = {});

    static PhysicsMeshObject createActive(
        std::string name,
        std::string meshPath,
        const glm::vec3& location = {0.0f, 0.0f, 0.0f},
        const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f},
        const RigidBodyProps& props = {1.0f});

    // Legacy support / General purpose
    static PhysicsMeshObject createObject(
        std::string name,
        MeshData mesh,
        const glm::mat4& transform = glm::mat4(1.0f),
        const glm::vec3& location = {0.0f, 0.0f, 0.0f},
        const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f},
        const RigidBodyProps& physicsProps = {});

    static RigidBody* attachPhysics(
        PhysicsEngine& physicsEngine,
        PhysicsMeshObject& object);
};

} // namespace ge
