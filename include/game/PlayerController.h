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
    void updateCamera(float deltaTime);
    void updateCameraVectors();
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

    // Camera state
    glm::vec3 cameraPosition_{2.0f, 2.0f, 5.0f};
    glm::vec3 cameraFront_{0.0f, 0.0f, -1.0f};
    glm::vec3 cameraUp_{0.0f, 1.0f, 0.0f};
    glm::vec3 cameraRight_{1.0f, 0.0f, 0.0f};
    glm::vec3 cameraWorldUp_{0.0f, 1.0f, 0.0f};

    float cameraYaw_ = -90.0f;
    float cameraPitch_ = 0.0f;
    float cameraSpeed_ = 5.0f;
    float mouseSensitivity_ = 0.15f;

    double lastCursorX_ = 0.0;
    double lastCursorY_ = 0.0;
    bool firstMouse_ = true;
    bool mouseCaptured_ = false;
};

} // namespace ge
