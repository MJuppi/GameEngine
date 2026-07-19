#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <string>
#include <cstdint>

namespace ge {

enum class ColliderType {
    Box,
    Sphere
};

struct Contact;

class RigidBody;

struct Contact {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    glm::vec3 normal{0.0f};
    float depth = 0.0f;
    glm::vec3 point{0.0f};

    // Persistent ID for warm starting (e.g., hash of feature indices)
    uint32_t persistentId = 0;

    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
};

struct ContactManifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    glm::vec3 normal{0.0f};
    std::vector<Contact> contacts;

    bool isColliding = false;
};

class Collider {
public:
    explicit Collider(ColliderType type) : type_(type) {}
    virtual ~Collider() = default;

    ColliderType getType() const { return type_; }
    virtual void getLocalBounds(glm::vec3& min, glm::vec3& max) const = 0;
    virtual void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const = 0;

private:
    ColliderType type_;
};

} // namespace ge
