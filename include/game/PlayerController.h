#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace ge {

class Engine;
class RigidBody;

class PlayerController {
public:
    explicit PlayerController(Engine& engine);
    ~PlayerController() = default;

    void setInputEnabled(bool enabled);
    bool isInputEnabled() const { return inputEnabled_; }

    void fixedUpdate(float deltaTime);
    void variableUpdate(float deltaTime, float alpha);

private:
    struct Projectile {
        RigidBody* body = nullptr;
        float lifetime = 0.0f;
        float damage = 0.0f;
        bool missile = false;
    };

    void updateFlight(float deltaTime);
    void updateEnemies(float deltaTime);
    void updateWeapons(float deltaTime);
    void updateCamera(float deltaTime, float alpha);
    void updateCameraVectors();
    void updateHud(float deltaTime);
    void fireGun();
    void fireMissile();
    void destroyProjectile(Projectile& projectile);
    RigidBody* findBody(const std::string& name) const;
    glm::vec3 forward() const;
    glm::vec3 right() const;
    void setAircraftTransform(RigidBody& body, const glm::vec3& position, const glm::vec3& direction);

    Engine& engine_;
    RigidBody* player_ = nullptr;
    std::vector<RigidBody*> enemies_;
    std::vector<Projectile> projectiles_;
    float playerHealth_ = 100.0f;
    int destroyedEnemies_ = 0;
    int missiles_ = 6;
    int selectedLoadout_ = 0;
    float throttle_ = 0.45f;
    float roll_ = 0.0f;
    float heat_ = 0.0f;
    float gunCooldown_ = 0.0f;
    float missileCooldown_ = 0.0f;
    float targetLock_ = 0.0f;
    RigidBody* lockedTarget_ = nullptr;
    bool missionComplete_ = false;
    bool missionFailed_ = false;
    bool cameraFirstPerson_ = false;
    bool cameraToggleWasDown_ = false;
    bool loadoutToggleWasDown_ = false;
    bool gunWasDown_ = false;
    bool missileWasDown_ = false;

    // Camera state
    glm::vec3 cameraPosition_{0.0f, 30.0f, 26.0f};
    glm::vec3 prevCameraPosition_{0.0f, 30.0f, 26.0f};
    glm::vec3 cameraFront_{0.0f, 0.0f, -1.0f};
    glm::vec3 cameraUp_{0.0f, 1.0f, 0.0f};
    glm::vec3 cameraRight_{1.0f, 0.0f, 0.0f};
    glm::vec3 cameraWorldUp_{0.0f, 1.0f, 0.0f};

    float cameraYaw_ = -90.0f;
    float cameraPitch_ = 0.0f;
    float mouseSensitivity_ = 0.15f;

    double lastCursorX_ = 0.0;
    double lastCursorY_ = 0.0;
    bool firstMouse_ = true;
    bool mouseCaptured_ = false;
    bool inputEnabled_ = true;
};

} // namespace ge
