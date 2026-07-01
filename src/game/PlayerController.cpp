#include "game/PlayerController.h"

#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"
#include "game/SceneFactory.h"

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <set>

namespace ge {

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
        fireProjectile();
    }
    leftMouseDown_ = firePressed;
    updateScoredProjectiles();
}

void PlayerController::fireProjectile() {
    const glm::vec3 spawnPosition = glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 fireDirection = glm::vec3(0.0f, 0.0f, -1.0f);

    auto* projectile = SceneFactory::spawnProjectile(engine_, spawnPosition, fireDirection);
    if (projectile) {
        spawnedProjectiles_.push_back(projectile);
        ++boxesShot_;
        std::cout << "Fired box " << boxesShot_ << '/' << kBoxesToShoot << "\n";
    } else {
        std::cerr << "Failed to spawn projectile\n";
    }
}

void PlayerController::updateScoredProjectiles() {
    // Use a set for faster lookup instead of linear search
    std::set<RigidBody*> scoredSet(scoredProjectiles_.begin(), scoredProjectiles_.end());

    for (RigidBody* projectile : spawnedProjectiles_) {
        if (!projectile || !isProjectileInTargetZone(*projectile)) {
            continue;
        }

        // Check if projectile has already been scored using set for O(log n) lookup
        if (scoredSet.find(projectile) == scoredSet.end()) {
            scoredProjectiles_.push_back(projectile);
            scoredSet.insert(projectile);  // Add to set for future checks
            ++boxesScored_;
            std::cout << "Box hit target zone! " << boxesScored_ << '/' << kBoxesToShoot << "\n";
        }
    }
}

bool PlayerController::isProjectileInTargetZone(const RigidBody& projectile) const {
    const glm::vec3 position = projectile.getPosition();
    return position.x >= targetMin_.x && position.x <= targetMax_.x &&
           position.y >= targetMin_.y && position.y <= targetMax_.y &&
           position.z >= targetMin_.z && position.z <= targetMax_.z;
}

} // namespace ge
