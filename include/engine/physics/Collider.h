#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <string>

namespace ge {

struct Contact;

class RigidBody;

struct CollisionResult {
    bool isColliding = false;
    std::vector<Contact> contacts;
};

struct Contact {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    glm::vec3 normal{0.0f};
    float depth = 0.0f;
    glm::vec3 point{0.0f};
};

class Collider {
public:
    virtual ~Collider() = default;

    virtual std::string getType() const = 0;
    virtual void getLocalBounds(glm::vec3& min, glm::vec3& max) const = 0;
    virtual CollisionResult checkCollision(
        const Collider& other,
        const glm::mat4& transformA,
        const glm::mat4& transformB
    ) const = 0;
    virtual void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const = 0;
};

} // namespace ge
