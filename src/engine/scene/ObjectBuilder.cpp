#include "engine/scene/ObjectBuilder.h"

#include <utility>
#include <iostream>

namespace ge {

PhysicsMeshObject ObjectBuilder::createVisual(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents) {

    return PhysicsMeshObject(std::move(name), std::move(meshPath), location, halfExtents, ObjectType::Visual);
}

PhysicsMeshObject ObjectBuilder::createStatic(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& props) {

    RigidBodyProps staticProps = props;
    // Default to static behavior if mass is default 1.0 or mass 0 without kinematic
    if (staticProps.mass == 1.0f && !staticProps.isKinematic) {
        staticProps.mass = 0.0f;
        staticProps.isKinematic = true;
        staticProps.useGravity = false;
    }

    return PhysicsMeshObject(std::move(name), std::move(meshPath), location, halfExtents, ObjectType::Static, staticProps);
}

PhysicsMeshObject ObjectBuilder::createActive(
    std::string name,
    std::string meshPath,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& props) {
    
    return PhysicsMeshObject(std::move(name), std::move(meshPath), location, halfExtents, ObjectType::Active, props);
}

PhysicsMeshObject ObjectBuilder::createObject(
    std::string name,
    MeshData mesh,
    const glm::mat4& transform,
    const glm::vec3& location,
    const glm::vec3& halfExtents,
    const RigidBodyProps& physicsProps) {

    ObjectType type;
    // Determine the object type based on physics properties
    if (physicsProps.mass > 0.0f && !physicsProps.isKinematic) {
        type = ObjectType::Active;
    } else if (physicsProps.mass == 0.0f && physicsProps.isKinematic) {
        type = ObjectType::Static;
    } else {
        type = ObjectType::Visual;
    }

    PhysicsMeshObject obj(std::move(name), "", location, halfExtents, type, physicsProps);
    obj.mesh = std::move(mesh);
    obj.transform = transform;
    return obj;
}

RigidBody* ObjectBuilder::attachPhysics(PhysicsEngine& physicsEngine, PhysicsMeshObject& object) {
    if (!object.hasPhysicsBody()) {
        return nullptr;
    }

    glm::mat4 initialTransform = object.getWorldTransform();

    if (object.type == ObjectType::Active) {
        // Compute a local center-of-mass offset from the active mesh bounds.
        // This allows tipping behavior to use the actual object COM rather than the shape origin.
        if (!object.mesh.vertices.empty()) {
            auto bounds = computeMeshBounds(object.mesh);
            object.physicsProps.centerOfMassOffset = bounds.getCenter();
        } else {
            object.physicsProps.centerOfMassOffset = glm::vec3(0.0f);
        }

        // Active bodies use the same creation convention as projectiles:
        // scaled transform with a unit cube collider.
    }

    RigidBody* body = nullptr;
    if (object.type == ObjectType::Active) {
        body = physicsEngine.createActiveBoxBody(
            object.halfExtents,
            initialTransform,
            object.physicsProps);
    } else {
        body = physicsEngine.createBoxBody(
            object.halfExtents,
            initialTransform,
            object.physicsProps);
    }

    if (body) {
        body->setName(object.name);
        std::cout << "[Physics] Attached body to '" << object.name << "' at ("
                  << object.spawnLocation.x << ", " << object.spawnLocation.y << ", " << object.spawnLocation.z
                  << ") extents=(" << object.halfExtents.x << ", " << object.halfExtents.y << ", " << object.halfExtents.z << ")\n";

        if (!object.mesh.materials.empty()) {
            body->setMaterial(std::make_shared<Material>(object.mesh.materials[0]));
        }

        if (object.type == ObjectType::Active && !object.mesh.vertices.empty()) {
            body->setMesh(object.mesh);
        }
    }

    return body;
}

} // namespace ge
