#include "game/PlayerController.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/RigidBody.h"
#include "engine/ui/Label.h"
#include "engine/ui/UIManager.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

namespace ge {
namespace {
RigidBodyProps aircraftProps() {
    RigidBodyProps props;
    props.mass = 1.0f;
    props.useGravity = false;
    props.linearDamping = 0.02f;
    props.angularDamping = 0.9f;
    props.restitution = 0.1f;
    return props;
}

bool isAircraftName(const std::string& name) {
    return name == "PlayerJet" || name.rfind("EnemyJet", 0) == 0;
}
} // namespace

PlayerController::PlayerController(Engine& engine) : engine_(engine) {
    player_ = findBody("PlayerJet");
    for (int i = 0; i < 4; ++i) {
        if (auto* enemy = findBody("EnemyJet" + std::to_string(i))) enemies_.push_back(enemy);
    }

    auto hud = std::make_shared<Label>();
    hud->setPosition({0.025f, 0.035f});
    hud->setSize({0.52f, 0.27f});
    hud->setColor({0.82f, 0.94f, 1.0f, 1.0f});
    hud->setFontSize(0.82f);
    hud->setOnUpdate([this](Label& label, float deltaTime) {
        updateHud(deltaTime);
        std::ostringstream text;
        const float speed = player_ ? glm::length(player_->getVelocity()) : 0.0f;
        const float altitude = player_ ? std::max(0.0f, player_->getPosition().y) : 0.0f;
        text << "SKY RAID  //  " << (missionComplete_ ? "MISSION COMPLETE" : missionFailed_ ? "AIRCRAFT LOST" : "INTERCEPT")
             << "\nSPD " << std::fixed << std::setprecision(0) << speed << " m/s   ALT " << altitude << " m"
             << "\nHULL " << std::setprecision(0) << std::max(0.0f, playerHealth_) << "%   HEAT " << heat_ * 100.0f << "%"
             << "\nHOSTILES " << std::max(0, static_cast<int>(enemies_.size()) - destroyedEnemies_) << "/" << enemies_.size()
             << "   MISSILES " << missiles_ << "   " << (lockedTarget_ ? "[LOCK]" : "[ SEARCH ]")
             << "\nW/S throttle  A/D roll  ARROWS pitch/yaw  SPACE guns  RMB missile  C camera  TAB loadout";
        label.setText(text.str());
    });
    engine_.getUIManager().addElement(hud);

    engine_.getPhysicsEngine().getWorld().setCollisionCallback([this](const PhysicsWorld::CollisionEvent& event) {
        if (!event.bodyA || !event.bodyB) return;
        for (auto& projectile : projectiles_) {
            if (projectile.body != event.bodyA && projectile.body != event.bodyB) continue;
            RigidBody* target = projectile.body == event.bodyA ? event.bodyB : event.bodyA;
            if (!isAircraftName(target->getName())) continue;
            if (target == player_) playerHealth_ -= projectile.damage;
            else target->setMass(0.0f);
            projectile.lifetime = -1.0f;
        }
    });
}

void PlayerController::setInputEnabled(bool enabled) {
    inputEnabled_ = enabled;
    if (!enabled) {
        if (auto* window = engine_.getWindowHandle()) glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        mouseCaptured_ = false;
        firstMouse_ = true;
    }
}

RigidBody* PlayerController::findBody(const std::string& name) const {
    for (const auto& body : engine_.getPhysicsEngine().getWorld().getBodies()) {
        if (body && body->getName() == name) return body.get();
    }
    return nullptr;
}

glm::vec3 PlayerController::forward() const { return glm::normalize(cameraFront_); }
glm::vec3 PlayerController::right() const { return glm::normalize(glm::cross(forward(), cameraWorldUp_)); }

void PlayerController::setAircraftTransform(RigidBody& body, const glm::vec3& position, const glm::vec3& direction) {
    const glm::vec3 nose = glm::normalize(direction);
    const glm::vec3 baseSide = glm::normalize(glm::cross(nose, cameraWorldUp_));
    const glm::vec3 baseUp = glm::normalize(glm::cross(baseSide, nose));
    const float bank = body.getName() == "PlayerJet" ? roll_ : 0.0f;
    const glm::vec3 side = baseSide * std::cos(bank) + baseUp * std::sin(bank);
    const glm::vec3 up = baseUp * std::cos(bank) - baseSide * std::sin(bank);
    glm::mat4 transform(1.0f);
    transform[0] = glm::vec4(side, 0.0f);
    transform[1] = glm::vec4(up, 0.0f);
    transform[2] = glm::vec4(-nose, 0.0f);
    transform[3] = glm::vec4(position, 1.0f);
    body.setTransform(transform);
}

void PlayerController::fixedUpdate(float deltaTime) {
    if (!inputEnabled_ || !player_ || missionComplete_ || missionFailed_) return;
    updateFlight(deltaTime);
    updateEnemies(deltaTime);
    updateWeapons(deltaTime);
}

void PlayerController::updateFlight(float deltaTime) {
    auto* window = engine_.getWindowHandle();
    if (!window) return;
    prevCameraPosition_ = player_->getPosition();
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) throttle_ += deltaTime * 0.35f;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) throttle_ -= deltaTime * 0.35f;
    throttle_ = glm::clamp(throttle_, 0.12f, 1.0f);
    const bool afterburner = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS && heat_ < 0.98f;
    const float targetSpeed = (afterburner ? 95.0f : 62.0f) * throttle_;
    const float speed = glm::mix(glm::length(player_->getVelocity()), targetSpeed, deltaTime * 2.8f);
    const float pitch = (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ? 1.0f : 0.0f) - (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ? 1.0f : 0.0f);
    const float yaw = (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS ? 1.0f : 0.0f) - (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS ? 1.0f : 0.0f);
    const float rollInput = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 1.0f : 0.0f) - (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 1.0f : 0.0f);
    cameraPitch_ = glm::clamp(cameraPitch_ + pitch * deltaTime * 42.0f, -65.0f, 65.0f);
    cameraYaw_ += yaw * deltaTime * 55.0f;
    roll_ = glm::clamp(roll_ + rollInput * deltaTime * 2.5f, -1.1f, 1.1f);
    if (rollInput == 0.0f) roll_ = glm::mix(roll_, 0.0f, deltaTime * 2.0f);
    updateCameraVectors();
    player_->setVelocity(forward() * speed);
    glm::vec3 position = player_->getPosition() + player_->getVelocity() * deltaTime;
    position.y = std::max(position.y, 5.0f);
    setAircraftTransform(*player_, position, forward());
    heat_ = afterburner ? std::min(1.0f, heat_ + deltaTime * 0.22f) : std::max(0.0f, heat_ - deltaTime * 0.12f);
    if (position.y <= 5.1f) playerHealth_ -= deltaTime * 5.0f;
    if (playerHealth_ <= 0.0f) missionFailed_ = true;
}

void PlayerController::updateEnemies(float deltaTime) {
    for (auto* enemy : enemies_) {
        if (!enemy || enemy->getProps().mass <= 0.0f) continue;
        const glm::vec3 toPlayer = player_->getPosition() - enemy->getPosition();
        const float distance = glm::length(toPlayer);
        const glm::vec3 direction = glm::normalize(toPlayer);
        const float speed = 32.0f + static_cast<float>((enemy->getName().back() - '0') * 3);
        enemy->setVelocity(direction * speed);
        setAircraftTransform(*enemy, enemy->getPosition() + direction * speed * deltaTime, direction);
        if (distance < 18.0f) playerHealth_ -= deltaTime * 4.0f;
    }
    lockedTarget_ = nullptr;
    float nearest = 1000.0f;
    for (auto* enemy : enemies_) {
        if (!enemy || enemy->getProps().mass <= 0.0f) continue;
        const glm::vec3 toEnemy = enemy->getPosition() - player_->getPosition();
        const float distance = glm::length(toEnemy);
        if (distance < nearest && glm::dot(forward(), glm::normalize(toEnemy)) > 0.65f) { nearest = distance; lockedTarget_ = enemy; }
    }
    targetLock_ = lockedTarget_ ? std::min(1.0f, targetLock_ + deltaTime * 0.8f) : 0.0f;
}

void PlayerController::updateWeapons(float deltaTime) {
    auto* window = engine_.getWindowHandle();
    if (!window) return;
    gunCooldown_ -= deltaTime;
    missileCooldown_ -= deltaTime;
    const bool gunDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool missileDown = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (gunDown && gunCooldown_ <= 0.0f && heat_ < 0.99f) fireGun();
    if (missileDown && !missileWasDown_ && missileCooldown_ <= 0.0f && missiles_ > 0 && targetLock_ > 0.65f) fireMissile();
    gunWasDown_ = gunDown;
    missileWasDown_ = missileDown;
    for (auto& projectile : projectiles_) {
        projectile.lifetime -= deltaTime;
        if (projectile.body && projectile.missile && lockedTarget_ && projectile.lifetime > 0.0f) {
            projectile.body->setVelocity(glm::normalize(lockedTarget_->getPosition() - projectile.body->getPosition()) * 105.0f);
        }
    }
    for (auto it = projectiles_.begin(); it != projectiles_.end();) {
        if (it->lifetime > 0.0f && it->body) { ++it; continue; }
        destroyProjectile(*it);
        it = projectiles_.erase(it);
    }
    destroyedEnemies_ = 0;
    for (auto* enemy : enemies_) if (enemy && enemy->getProps().mass <= 0.0f) ++destroyedEnemies_;
    if (destroyedEnemies_ >= static_cast<int>(enemies_.size())) missionComplete_ = true;
}

void PlayerController::fireGun() {
    const glm::mat4 transform = glm::translate(glm::mat4(1.0f), player_->getPosition() + forward() * 3.0f);
    auto* projectile = engine_.getPhysicsEngine().createActiveBoxBody({0.12f, 0.12f, 0.35f}, transform, aircraftProps());
    if (!projectile) return;
    projectile->setName("GunRound");
    projectile->setVelocity(forward() * 120.0f);
    projectiles_.push_back({projectile, 1.8f, selectedLoadout_ == 0 ? 8.0f : 11.0f, false});
    gunCooldown_ = selectedLoadout_ == 0 ? 0.10f : 0.18f;
    heat_ = std::min(1.0f, heat_ + 0.035f);
}

void PlayerController::fireMissile() {
    const glm::mat4 transform = glm::translate(glm::mat4(1.0f), player_->getPosition() + forward() * 3.0f);
    auto* projectile = engine_.getPhysicsEngine().createActiveBoxBody({0.25f, 0.25f, 0.8f}, transform, aircraftProps());
    if (!projectile) return;
    projectile->setName("Missile");
    projectile->setVelocity(forward() * 70.0f);
    projectiles_.push_back({projectile, 7.0f, 65.0f, true});
    --missiles_;
    missileCooldown_ = 1.0f;
}

void PlayerController::destroyProjectile(Projectile& projectile) {
    if (projectile.body) engine_.getPhysicsEngine().destroyBody(projectile.body);
    projectile.body = nullptr;
}

void PlayerController::variableUpdate(float deltaTime, float alpha) {
    auto* window = engine_.getWindowHandle();
    if (!window || !inputEnabled_) return;
    const bool cameraDown = glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS;
    const bool loadoutDown = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;
    if (cameraDown && !cameraToggleWasDown_) cameraFirstPerson_ = !cameraFirstPerson_;
    if (loadoutDown && !loadoutToggleWasDown_) selectedLoadout_ = (selectedLoadout_ + 1) % 2;
    cameraToggleWasDown_ = cameraDown;
    loadoutToggleWasDown_ = loadoutDown;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !mouseCaptured_) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        mouseCaptured_ = true;
        firstMouse_ = true;
    }
    if (mouseCaptured_) {
        double x, y;
        glfwGetCursorPos(window, &x, &y);
        if (firstMouse_) { lastCursorX_ = x; lastCursorY_ = y; firstMouse_ = false; }
        cameraYaw_ += static_cast<float>(x - lastCursorX_) * mouseSensitivity_;
        cameraPitch_ = glm::clamp(cameraPitch_ - static_cast<float>(y - lastCursorY_) * mouseSensitivity_, -65.0f, 65.0f);
        lastCursorX_ = x;
        lastCursorY_ = y;
        updateCameraVectors();
    }
    updateCamera(deltaTime, alpha);
}

void PlayerController::updateCamera(float, float alpha) {
    if (!player_) return;
    const glm::vec3 position = glm::mix(prevCameraPosition_, player_->getPosition(), alpha);
    cameraPosition_ = cameraFirstPerson_ ? position + glm::vec3(0.0f, 0.8f, 0.0f) : position - forward() * 13.0f + glm::vec3(0.0f, 5.0f, 0.0f);
    engine_.setCamera(cameraPosition_, forward(), cameraWorldUp_);
}

void PlayerController::updateHud(float) {}

void PlayerController::updateCameraVectors() {
    glm::vec3 front;
    front.x = std::cos(glm::radians(cameraYaw_)) * std::cos(glm::radians(cameraPitch_));
    front.y = std::sin(glm::radians(cameraPitch_));
    front.z = std::sin(glm::radians(cameraYaw_)) * std::cos(glm::radians(cameraPitch_));
    cameraFront_ = glm::normalize(front);
    cameraRight_ = glm::normalize(glm::cross(cameraFront_, cameraWorldUp_));
    cameraUp_ = glm::normalize(glm::cross(cameraRight_, cameraFront_));
}

} // namespace ge
