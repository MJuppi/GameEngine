#pragma once

#include "engine/mesh/MeshData.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>

namespace ge {

struct PhysicsMeshObject {
    std::string name;
    std::string meshPath;
    MeshData mesh;
    glm::mat4 transform{1.0f};
    glm::vec3 halfExtents{0.5f, 0.5f, 0.5f};
    glm::vec3 spawnLocation{0.0f, 0.0f, 0.0f};
    RigidBodyProps physicsProps{};
    bool createPhysicsBody = true;

    [[nodiscard]] glm::mat4 getWorldTransform() const {
        return glm::translate(glm::mat4(1.0f), spawnLocation) * transform;
    }

    [[nodiscard]] bool hasPhysicsBody() const noexcept {
        return createPhysicsBody;
    }
};

class ObjectBuilder {
public:
    static PhysicsMeshObject createObject(
        std::string name,
        MeshData mesh,
        const glm::mat4& transform = glm::mat4(1.0f),
        const glm::vec3& halfExtents = {0.5f, 0.5f, 0.5f},
        const glm::vec3& location = {0.0f, 0.0f, 0.0f},
        const RigidBodyProps& physicsProps = {});

    static RigidBody* attachPhysics(
        PhysicsEngine& physicsEngine,
        PhysicsMeshObject& object);
};

} // namespace ge
