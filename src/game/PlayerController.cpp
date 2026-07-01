#include "game/PlayerController.h"

#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <iostream>
#include <vector>

namespace ge {

namespace {

class ProjectileHandle {
public:
    explicit ProjectileHandle(RigidBody* body) : body(body) {}
    RigidBody* body;
};

} // namespace

PlayerController::PlayerController(Engine& engine)
    : engine_(engine) {}

/// @brief Updates the player controller state, handling input and projectile firing.
/// @param deltaTime 
void PlayerController::update(float deltaTime) {
    (void)deltaTime;

    auto* window = engine_.getWindowHandle();
    if (!window) {
        return;
    }

    const bool firePressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (firePressed && !leftMouseDown_ && boxesShot_ < kBoxesToShoot) {
        const glm::vec3 spawnPosition = glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 fireDirection = glm::vec3(0.0f, 0.0f, -1.0f);
        const glm::mat4 spawnTransform = glm::translate(glm::mat4(1.0f), spawnPosition);

        RigidBodyProps projectileProps;
        projectileProps.mass = 1.0f;
        projectileProps.friction = 0.2f;
        projectileProps.restitution = 0.6f;
        projectileProps.linearDamping = 0.01f;
        projectileProps.angularDamping = 0.02f;

        auto* projectile = engine_.getPhysicsEngine().createBoxBody(
            {0.25f, 0.25f, 0.25f},
            spawnTransform,
            projectileProps);
        projectile->setVelocity(fireDirection * 15.0f + glm::vec3(0.0f, 0.2f, 0.0f));
        projectile->setAngularVelocity(glm::vec3(0.5f, 1.0f, 0.2f));
        spawnedProjectiles_.push_back(projectile);
        ++boxesShot_;
        std::cout << "Fired box " << boxesShot_ << '/' << kBoxesToShoot << "\n";
    }
    leftMouseDown_ = firePressed;

    for (void* projectilePtr : spawnedProjectiles_) {
        auto* projectile = static_cast<RigidBody*>(projectilePtr);
        if (!projectile) {
            continue;
        }

        const glm::vec3 position = projectile->getPosition();
        const bool inTargetZone = position.x >= targetMin_.x && position.x <= targetMax_.x &&
                                  position.y >= targetMin_.y && position.y <= targetMax_.y &&
                                  position.z >= targetMin_.z && position.z <= targetMax_.z;
        if (inTargetZone && std::find(scoredProjectiles_.begin(), scoredProjectiles_.end(), projectilePtr) == scoredProjectiles_.end()) {
            scoredProjectiles_.push_back(projectilePtr);
            ++boxesScored_;
            std::cout << "Box hit target zone! " << boxesScored_ << '/' << kBoxesToShoot << "\n";
        }
    }
}

} // namespace ge
