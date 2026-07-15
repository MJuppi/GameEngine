#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/gtc/matrix_transform.hpp>

namespace ge {

void SceneFactory::configureTestLevel(Level& level) {
    // Configure lighting
    auto& light = level.getPointLight();
    light.position = {0.0f, 2.0f, 0.0f, 1.0f};
    light.color = {1.0f, 0.95f, 0.8f, 1.0f};
    light.parameters = {1.0f, 0.09f, 0.032f, 8.0f};

    // Simplify adding models using the new fluent API and auto-extents
    level.add("TestCube.obj").at(0.0f, 4.0f, 0.0f).asActive();
    level.add("TestCube.obj").at(0.0f, 10.0f, 0.0f).asActive();

    // Example of using the new material refactor
    Material redMat = makeDefaultMaterial("RedCube");
    redMat.diffuse = {1.0f, 0.0f, 0.0f};
    redMat.shininess = 64.0f;
    level.add("suomi").name("Suomi").at(2.0f, 6.0f, 0.0f).extents({5.0f, 5.0f, 5.0f}).asVisual();

    // Add static ground (empty path uses fallback unit cube)
    level.add("").name("Ground").at(0.0f, -2.0f, 0.0f).extents({50.0f, 0.5f, 50.0f}).asStatic();

    // Example: Add a visual-only prop from an OBJ file with automatic bounds calculation
    // level.add("models/Mountain.obj").at(0.0f, 0.0f, -100.0f).asVisual();
}

RigidBodyProps SceneFactory::makeDynamicBoxProps(float mass, float friction, float restitution) {
    RigidBodyProps props;
    props.mass = mass;
    props.friction = friction;
    props.restitution = restitution;
    props.linearDamping = 0.01f;
    props.angularDamping = 0.02f;
    return props;
}

RigidBodyProps SceneFactory::makeGroundProps() {
    RigidBodyProps props;
    props.mass = 0.0f;
    props.isKinematic = true;
    props.useGravity = false;
    return props;
}

RigidBodyProps SceneFactory::makeProjectileProps() {
    return makeDynamicBoxProps(1.0f, 0.2f, 0.6f);
}

/// @brief Spawns a projectile in the physics engine with the specified properties. The projectile is created as a box-shaped rigid body with the given half extents, spawn position, and initial velocity based on the fire direction and optional velocity offset. The function returns a pointer to the created RigidBody, or nullptr if the half extents are invalid (non-positive).
/// @param engine 
/// @param spawnPosition 
/// @param fireDirection 
/// @param velocityOffset 
/// @param halfExtents 
/// @return 
RigidBody* SceneFactory::spawnProjectile(Engine& engine, const glm::vec3& spawnPosition, const glm::vec3& fireDirection, const glm::vec3& velocityOffset, const glm::vec3& halfExtents) {
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f) {
        return nullptr;
    }

    const glm::mat4 spawnTransform = glm::translate(glm::mat4(1.0f), spawnPosition);
    auto* projectile = engine.getPhysicsEngine().createBoxBody(halfExtents * 0.1f, spawnTransform, makeProjectileProps());

    if (projectile) {
        projectile->setVelocity(fireDirection * 15.0f + velocityOffset);
        projectile->setAngularVelocity(glm::vec3(0.5f, 1.0f, 0.2f));
    }

    return projectile;
}

} // namespace ge
