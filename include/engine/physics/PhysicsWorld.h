#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "engine/physics/Collider.h"

namespace ge {

class RigidBody;

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    void setGravity(const glm::vec3& gravity);
    RigidBody* addBody(std::unique_ptr<RigidBody> body);
    void removeBody(RigidBody* body);
    void clearBodies();

    void step(float deltaTime, int maxSubSteps = 1);

    struct RaycastResult {
        bool hit = false;
        RigidBody* body = nullptr;
        glm::vec3 point{0.0f};
        glm::vec3 normal{0.0f};
        float fraction = 1.0f;
    };

    RaycastResult raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance);

    void setSolverIterations(int iterations) { solverIterations_ = iterations; }
    int getSolverIterations() const { return solverIterations_; }

    const std::vector<std::unique_ptr<RigidBody>>& getBodies() const { return bodies_; }

private:
    void applyGravity();
    void resolveContacts(std::vector<ContactManifold>& manifolds);
    void warmStart(std::vector<ContactManifold>& manifolds);

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<ContactManifold> manifoldCache_;

    glm::vec3 gravity_{0.0f, -9.81f, 0.0f};
    int solverIterations_ = 8;
};

} // namespace ge
