#include "engine/scene/ObjectBuilder.h"

#include <utility>

namespace ge {

PhysicsMeshObject ObjectBuilder::createVisual(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents) {
    PhysicsMeshObject object;
    object.name = std::move(name);
    object.meshPath = std::move(meshPath);
    object.spawnLocation = location;
    object.halfExtents = halfExtents;
    object.type = ObjectType::Visual;
    return object;
}

PhysicsMeshObject ObjectBuilder::createStatic(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& props) {
    PhysicsMeshObject object;
    object.name = std::move(name);
    object.meshPath = std::move(meshPath);
    object.spawnLocation = location;
    object.halfExtents = halfExtents;
    object.physicsProps = props;
    object.physicsProps.mass = 0.0f; // Force static
    object.physicsProps.isKinematic = true;
    object.type = ObjectType::Static;
    return object;
}

PhysicsMeshObject ObjectBuilder::createActive(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& props) {
    PhysicsMeshObject object;
    object.name = std::move(name);
    object.meshPath = std::move(meshPath);
    object.spawnLocation = location;
    object.halfExtents = halfExtents;
    object.physicsProps = props;
    if (object.physicsProps.mass <= 0.0f) object.physicsProps.mass = 1.0f;
    object.type = ObjectType::Active;
    return object;
}

PhysicsMeshObject ObjectBuilder::createObject(
    std::string name,
    MeshData mesh,
    const glm::mat4& transform,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& physicsProps) {
    PhysicsMeshObject object;
    object.name = std::move(name);
    object.mesh = std::move(mesh);
    object.transform = transform;
    object.spawnLocation = location;
    object.halfExtents = halfExtents;
    object.physicsProps = physicsProps;

    if (physicsProps.mass > 0.0f && !physicsProps.isKinematic) {
        object.type = ObjectType::Active;
    } else if (physicsProps.mass == 0.0f && physicsProps.isKinematic) {
        object.type = ObjectType::Static;
    } else {
        object.type = ObjectType::Visual;
    }

    return object;
}

RigidBody* ObjectBuilder::attachPhysics(PhysicsEngine& physicsEngine, PhysicsMeshObject& object) {
    if (!object.hasPhysicsBody()) {
        return nullptr;
    }

    auto* body = physicsEngine.createBoxBody(
        object.halfExtents,
        object.getWorldTransform(),
        object.physicsProps);

    if (body && !object.mesh.materials.empty()) {
        body->setMaterial(std::make_shared<Material>(object.mesh.materials[0]));
    }

    return body;
}

} // namespace ge
