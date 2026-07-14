#pragma once

#include "engine/physics/PhysicsWorld.h"
#include <glm/glm.hpp>

namespace ge {

class RigidBody;
struct RigidBodyProps;
class PhysicsWorld;

class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine();

    RigidBody* createBoxBody(const glm::vec3& halfExtents,
                             const glm::mat4& transform,
                             const RigidBodyProps& props);
    RigidBody* createSphereBody(float radius,
                               const glm::mat4& transform,
                               const RigidBodyProps& props);

    void destroyBody(RigidBody* body);
    void setGravity(const glm::vec3& gravity);
    void setFixedTimeStep(float step) { fixedTimeStep_ = step; }
    void update(float deltaTime, int maxSubSteps = 1);
    void clear();

    PhysicsWorld& getWorld() { return world_; }
    const PhysicsWorld& getWorld() const { return world_; }

    bool paused_ = false;
    float fixedTimeStep_ = 0.0f;
    float accumulator_ = 0.0f;

private:
    PhysicsWorld world_;
};

} // namespace ge
