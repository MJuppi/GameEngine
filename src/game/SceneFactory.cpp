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

    const std::string modelPath = "assets/models/TestCube.obj";
    const glm::vec3 cubeHalfExtents{0.5f, 0.5f, 0.5f};

    // Add active physics objects
    level.addActive("TestCube", modelPath, {0.0f, 0.0f, 0.0f}, cubeHalfExtents);
    level.addActive("TestCube2", modelPath, {0.0f, 4.0f, 0.0f}, cubeHalfExtents);

    // Add static ground
    level.addStatic("Ground", "", {0.0f, -2.0f, 0.0f}, {50.0f, 0.5f, 50.0f});

    // Example: Add a visual-only prop (no physics)
    // level.addVisual("BackgroundMountain", "assets/models/Mountain.obj", {0.0f, 0.0f, -100.0f}, {10.0f, 10.0f, 10.0f});
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
