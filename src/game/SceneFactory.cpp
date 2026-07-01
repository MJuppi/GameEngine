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

    // Add test cubes
    const glm::vec3 cubeHalfExtents{0.5f, 0.5f, 0.5f};
    const std::string modelPath = "assets/models/TestCube.obj";

    level.addObject("TestCube",
                    glm::mat4(1.0f),
                    {0.0f, 0.0f, 0.0f},
                    cubeHalfExtents,
                    makeDynamicBoxProps(),
                    modelPath);

    level.addObject("TestCube2",
                    glm::mat4(1.0f),
                    {0.0f, 0.0f, 5.0f},
                    cubeHalfExtents,
                    makeDynamicBoxProps(),
                    modelPath);

    // Add ground
    const glm::vec3 groundHalfExtents{50.0f, 0.5f, 50.0f};
    level.addObject("Ground",
                    glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}),
                    groundHalfExtents,
                    {0.0f, 0.0f, 0.0f},
                    makeGroundProps());
}

void SceneFactory::setupTestPhysics(Engine& engine) {
    // Create test cubes
    const glm::vec3 cubeHalfExtents{0.5f, 0.5f, 0.5f};
    const glm::mat4 cubeTransform = glm::translate(glm::mat4(1.0f), {0.0f, 5.0f, 0.0f});
    
    engine.getPhysicsEngine().createBoxBody(
        cubeHalfExtents,
        cubeTransform,
        makeDynamicBoxProps());

    // Create ground
    const glm::vec3 groundHalfExtents{50.0f, 0.5f, 50.0f};
    const glm::mat4 groundTransform = glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f});
    
    engine.getPhysicsEngine().createBoxBody(
        groundHalfExtents,
        groundTransform,
        makeGroundProps());
}

RigidBodyProps SceneFactory::makeDynamicBoxProps(float mass,
                                                 float friction,
                                                 float restitution) {
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

RigidBody* SceneFactory::spawnProjectile(Engine& engine,
                                         const glm::vec3& spawnPosition,
                                         const glm::vec3& fireDirection,
                                         const glm::vec3& velocityOffset,
                                         const glm::vec3& halfExtents) {
    // Validate input parameters
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f) {
        return nullptr;
    }

    const glm::mat4 spawnTransform = glm::translate(glm::mat4(1.0f), spawnPosition);
    auto* projectile = engine.getPhysicsEngine().createBoxBody(
        halfExtents,
        spawnTransform,
        makeProjectileProps());

    if (projectile) {
        projectile->setVelocity(fireDirection * 15.0f + velocityOffset);
        projectile->setAngularVelocity(glm::vec3(0.5f, 1.0f, 0.2f));
    }
    
    return projectile;
}

} // namespace ge
