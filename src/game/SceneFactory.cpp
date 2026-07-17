#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/gtc/matrix_transform.hpp>

namespace ge {

void SceneFactory::configureTestLevel(Level& level) {
    // Configure lighting
    auto& lights = level.getSceneLights();

    // Ambient
    lights.ambient.color = {1.0f, 1.0f, 1.0f, 1.0f};
    lights.ambient.intensity = 0.1f;

    // Directional
    lights.directional.direction = glm::normalize(glm::vec4(0.5f, 1.0f, 0.5f, 0.0f));
    lights.directional.color = {1.0f, 0.95f, 0.8f, 1.0f};
    lights.directional.intensity = 0.5f;

    // Point Lights
    lights.pointLightCount = 2;

    // Light 1: Warm orange
    lights.pointLights[0].position = {0.0f, 4.0f, 0.0f, 1.0f};
    lights.pointLights[0].color = {1.0f, 0.6f, 0.2f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.09f, 0.032f, 2.0f};

    // Light 2: Cool blue
    lights.pointLights[1].position = {5.0f, 2.0f, 5.0f, 1.0f};
    lights.pointLights[1].color = {0.2f, 0.4f, 1.0f, 1.0f};
    lights.pointLights[1].parameters = {1.0f, 0.09f, 0.032f, 1.5f};

    // Stacking test for stability
    for (int i = 0; i < 5; ++i) {
        level.add("TestCube.obj").name("Stack_" + std::to_string(i))
             .at(0.0f, 2.0f + i * 2.1f, 0.0f).asActive();
    }

    // Trigger test: A ghost cube that doesn't block
    RigidBodyProps triggerProps;
    triggerProps.isTrigger = true;
    triggerProps.isKinematic = true;
    level.add("TestCube.obj").name("GhostCube").at(5.0f, 2.0f, 0.0f)
         .props(triggerProps).asActive();

    // Example of using the new material refactor
    Material redMat = makeDefaultMaterial("RedCube");
    redMat.diffuse = {1.0f, 0.0f, 0.0f};
    redMat.shininess = 64.0f;
    level.add("suomi").name("Suomi").at(2.0f, 6.0f, 0.0f).extents({5.0f, 5.0f, 5.0f}).asVisual();

    // Add static ground
    level.add("").name("Ground").at(0.0f, -2.0f, 0.0f).extents({50.0f, 0.5f, 50.0f}).asStatic();
}

RigidBodyProps SceneFactory::makeDynamicBoxProps(float mass, float friction, float restitution) {
    RigidBodyProps props;
    props.mass = mass;
    props.friction = friction;
    props.restitution = restitution;
    props.linearDamping = 0.05f;
    props.angularDamping = 0.1f;
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
    return makeDynamicBoxProps(1.0f, 0.5f, 0.4f);
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
