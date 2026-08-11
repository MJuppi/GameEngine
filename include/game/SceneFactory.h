#pragma once

#include "game/Level.h"
#include "game/LevelManager.h"
#include "engine/physics/PhysicsEngine.h"

#include <glm/glm.hpp>
#include <functional>

namespace ge {

class Engine;

class SceneFactory {
public:
    struct PauseMenuBindings {
        std::function<void(bool)> setVisible;
        std::function<bool()> isVisible;
    };

    static void configureTestLevel(Level& level);
    static void configureConstraintParityLevel(Level& level);
    static void configurePairOrderingLevel(Level& level);
    static void setupTestPhysics(Engine& engine);
    static void setupConstraintParityPhysics(Engine& engine);
    static void setupPairOrderingPhysics(Engine& engine);
    static PauseMenuBindings setupUI(Engine& engine,
                                     const LevelManager& levelManager,
                                     size_t currentLevelIndex,
                                     std::function<void(size_t)> onLevelSelected);

    static RigidBodyProps makeDynamicBoxProps(float mass = 1.0f,
                                              float friction = 0.3f,
                                              float restitution = 0.7f);
    static RigidBodyProps makeGroundProps();
    static RigidBodyProps makeProjectileProps();

    static RigidBody* spawnProjectile(Engine& engine,
                                      const glm::vec3& spawnPosition,
                                      const glm::vec3& fireDirection,
                                      const glm::vec3& velocityOffset = {0.0f, 0.2f, 0.0f},
                                      const glm::vec3& halfExtents = {1.0f, 1.0f, 1.0f});
};

} // namespace ge
