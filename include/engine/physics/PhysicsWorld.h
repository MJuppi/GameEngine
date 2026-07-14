#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

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

    const std::vector<std::unique_ptr<RigidBody>>& getBodies() const { return bodies_; }

private:
    void applyGravity();

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    glm::vec3 gravity_{0.0f, -9.81f, 0.0f};
    int solverIterations_ = 4;
};

} // namespace ge
