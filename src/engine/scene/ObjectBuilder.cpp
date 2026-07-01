#include "engine/scene/ObjectBuilder.h"

#include <utility>

namespace ge {

/// @brief Creates a PhysicsMeshObject with the specified parameters.
/// @param name 
/// @param mesh 
/// @param transform 
/// @param translation
/// @param halfExtents 
/// @param physicsProps 
/// @return 
PhysicsMeshObject ObjectBuilder::createObject(
    std::string name,
    MeshData mesh,
    const glm::mat4& transform,
    const glm::vec3& halfExtents,
    const glm::vec3& location,
    const RigidBodyProps& physicsProps) {
    PhysicsMeshObject object;
    object.name = std::move(name);
    object.mesh = std::move(mesh);
    object.transform = transform;
    object.halfExtents = halfExtents;
    object.spawnLocation = location;
    object.physicsProps = physicsProps;
    object.createPhysicsBody = true;
    return object;
}

RigidBody* ObjectBuilder::attachPhysics(PhysicsEngine& physicsEngine, PhysicsMeshObject& object) {
    if (!object.hasPhysicsBody()) {
        return nullptr;
    }

    return physicsEngine.createBoxBody(
        object.halfExtents,
        object.getWorldTransform(),
        object.physicsProps);
}

} // namespace ge
