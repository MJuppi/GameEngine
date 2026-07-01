#pragma once

#include "game/Level.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/glm.hpp>

namespace ge {

class Engine;

class SceneFactory {
public:
    static void configureTestLevel(Level& level);
    static void setupTestPhysics(Engine& engine);

    static RigidBodyProps makeDynamicBoxProps(float mass = 1.0f,
                                              float friction = 0.3f,
                                              float restitution = 0.7f);
    static RigidBodyProps makeGroundProps();
    static RigidBodyProps makeProjectileProps();

    static RigidBody* spawnProjectile(Engine& engine,
                                      const glm::vec3& spawnPosition,
                                      const glm::vec3& fireDirection,
                                      const glm::vec3& velocityOffset = {0.0f, 0.2f, 0.0f},
                                      const glm::vec3& halfExtents = {0.25f, 0.25f, 0.25f});
};

} // namespace ge
