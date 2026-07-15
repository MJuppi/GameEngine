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
    updateCamera(deltaTime);

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

void PlayerController::updateCamera(float deltaTime) {
    auto* window = engine_.getWindowHandle();
    if (!window) return;

    const float velocity = cameraSpeed_ * deltaTime;
    glm::vec3 horizontalFront = glm::normalize(glm::vec3(cameraFront_.x, 0.0f, cameraFront_.z));
    glm::vec3 horizontalRight = glm::normalize(glm::vec3(cameraRight_.x, 0.0f, cameraRight_.z));

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        cameraPosition_ += horizontalFront * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        cameraPosition_ -= horizontalFront * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        cameraPosition_ -= horizontalRight * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        cameraPosition_ += horizontalRight * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        cameraPosition_ += cameraWorldUp_ * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cameraPosition_ -= cameraWorldUp_ * velocity;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        cameraSpeed_ = 10.0f; // Sprint speed
    } else {
        cameraSpeed_ = 5.0f; // Normal speed
    }

    // Toggle mouse capture
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (!mouseCaptured_) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            firstMouse_ = true;
            mouseCaptured_ = true;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (mouseCaptured_) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            mouseCaptured_ = false;
        }
    }

    if (mouseCaptured_) {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        if (firstMouse_) {
            lastCursorX_ = xpos;
            lastCursorY_ = ypos;
            firstMouse_ = false;
        }

        float xoffset = static_cast<float>(xpos - lastCursorX_);
        float yoffset = static_cast<float>(ypos - lastCursorY_);
        lastCursorX_ = xpos;
        lastCursorY_ = ypos;

        xoffset *= mouseSensitivity_;
        yoffset *= mouseSensitivity_;

        cameraYaw_ += xoffset;
        cameraPitch_ += yoffset;
        cameraPitch_ = glm::clamp(cameraPitch_, -89.0f, 89.0f);

        updateCameraVectors();
    }

    engine_.setCamera(cameraPosition_, cameraFront_, cameraUp_);
}

void PlayerController::updateCameraVectors() {
    glm::vec3 front;
    front.x = std::cos(glm::radians(cameraYaw_)) * std::cos(glm::radians(cameraPitch_));
    front.y = std::sin(glm::radians(cameraPitch_));
    front.z = std::sin(glm::radians(cameraYaw_)) * std::cos(glm::radians(cameraPitch_));
    cameraFront_ = glm::normalize(front);
    cameraRight_ = glm::normalize(glm::cross(cameraFront_, cameraWorldUp_));
    cameraUp_ = glm::normalize(glm::cross(cameraRight_, cameraFront_));
}

void PlayerController::fireProjectile() {
    const glm::vec3 spawnPosition = {cameraPosition_.x, cameraPosition_.y * -1.0f, cameraPosition_.z};
    const glm::vec3 fireDirection = {cameraFront_.x, cameraFront_.y * -1.0f, cameraFront_.z};

    auto* projectile = SceneFactory::spawnProjectile(engine_, spawnPosition, fireDirection, {0.0f, 1.0f, 0.0f}, {0.1f, 0.1f, 0.1f});
    if (projectile) {
        spawnedProjectiles_.push_back(projectile);
        ++boxesShot_;
        std::cout << "Fired box at: " << spawnPosition.x << ", " << spawnPosition.y << ", " << spawnPosition.z << " | Boxes shot: " << boxesShot_ << '/' << kBoxesToShoot << "\n";
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
