#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/gtc/matrix_transform.hpp>

namespace ge {

namespace {

constexpr glm::vec3 kDefaultSpawnPosition{0.0f, 1.0f, 0.0f};
constexpr glm::vec3 kDefaultFireDirection{0.0f, 0.0f, -1.0f};

} // namespace

void SceneFactory::configureTestLevel(Level& level) {
    level.getPointLight().position = {0.0f, 2.0f, 0.0f, 1.0f};
    level.getPointLight().color = {1.0f, 0.95f, 0.8f, 1.0f};
    level.getPointLight().parameters = {1.0f, 0.09f, 0.032f, 8.0f};

    level.addObject("TestCube",
                    glm::mat4(1.0f),
                    {0.0f, 0.0f, 0.0f},
                    {0.5f, 0.5f, 0.5f},
                    makeDynamicBoxProps(),
                    "assets/models/TestCube.obj");

    level.addObject("TestCube2",
                    glm::mat4(1.0f),
                    {0.0f, 0.0f, 5.0f},
                    {0.5f, 0.5f, 0.5f},
                    makeDynamicBoxProps(),
                    "assets/models/TestCube.obj");

    level.addObject("Ground",
                    glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}),
                    {50.0f, 0.5f, 50.0f},
                    {0.0f, 0.0f, 0.0f},
                    makeGroundProps());
}

void SceneFactory::setupTestPhysics(Engine& engine) {
    engine.getPhysicsEngine().createBoxBody(
        {0.5f, 0.5f, 0.5f},
        glm::translate(glm::mat4(1.0f), {0.0f, 5.0f, 0.0f}),
        makeDynamicBoxProps());

    engine.getPhysicsEngine().createBoxBody(
        {50.0f, 0.5f, 50.0f},
        glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}),
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
    const glm::mat4 spawnTransform = glm::translate(glm::mat4(1.0f), spawnPosition);
    auto* projectile = engine.getPhysicsEngine().createBoxBody(
        halfExtents,
        spawnTransform,
        makeProjectileProps());

    projectile->setVelocity(fireDirection * 15.0f + velocityOffset);
    projectile->setAngularVelocity(glm::vec3(0.5f, 1.0f, 0.2f));
    return projectile;
}

} // namespace ge
