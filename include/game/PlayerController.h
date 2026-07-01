#pragma once

#include "engine/physics/PhysicsEngine.h"
#include <glm/glm.hpp>

namespace ge {

class Engine;

class PlayerController {
public:
    explicit PlayerController(Engine& engine);
    ~PlayerController() = default;

    void update(float deltaTime);

private:
    void fireProjectile();
    void updateScoredProjectiles();
    bool isProjectileInTargetZone(const RigidBody& projectile) const;

    Engine& engine_;
    bool leftMouseDown_ = false;
    static constexpr std::size_t kBoxesToShoot = 8;
    std::size_t boxesShot_ = 0;
    std::size_t boxesScored_ = 0;
    std::vector<RigidBody*> spawnedProjectiles_;
    std::vector<RigidBody*> scoredProjectiles_;
    glm::vec3 targetMin_{-2.5f, 0.0f, 3.5f};
    glm::vec3 targetMax_{2.5f, 2.5f, 8.5f};
};

} // namespace ge
