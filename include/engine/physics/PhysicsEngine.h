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
    RigidBody* createActiveBoxBody(const glm::vec3& visualHalfExtents,
                                  const glm::mat4& transform,
                                  const RigidBodyProps& props);
    RigidBody* createSphereBody(float radius,
                               const glm::mat4& transform,
                               const RigidBodyProps& props);

    void destroyBody(RigidBody* body);
    cannon::PointToPointConstraint* addPointToPointConstraint(RigidBody* bodyA,
                                                               const glm::vec3& pivotA,
                                                               RigidBody* bodyB,
                                                               const glm::vec3& pivotB,
                                                               float maxForce = 1e6f,
                                                               bool collideConnected = true);
    cannon::DistanceConstraint* addDistanceConstraint(RigidBody* bodyA,
                                                      RigidBody* bodyB,
                                                      float distance,
                                                      float maxForce = 1e6f,
                                                      bool collideConnected = true);
    cannon::LockConstraint* addLockConstraint(RigidBody* bodyA,
                                              RigidBody* bodyB,
                                              float maxForce = 1e6f,
                                              bool collideConnected = true);
    cannon::HingeConstraint* addHingeConstraint(RigidBody* bodyA,
                                                RigidBody* bodyB,
                                                const glm::vec3& pivotA,
                                                const glm::vec3& pivotB,
                                                const glm::vec3& axisA,
                                                const glm::vec3& axisB,
                                                float maxForce = 1e6f,
                                                bool collideConnected = true);
    cannon::ConeTwistConstraint* addConeTwistConstraint(RigidBody* bodyA,
                                                         RigidBody* bodyB,
                                                         const glm::vec3& pivotA,
                                                         const glm::vec3& pivotB,
                                                         const glm::vec3& axisA,
                                                         const glm::vec3& axisB,
                                                         float angle = 0.0f,
                                                         float twistAngle = 0.0f,
                                                         float maxForce = 1e6f,
                                                         bool collideConnected = true);
    void removeConstraint(cannon::Constraint* constraint);
    void clearConstraints();
    void setGravity(const glm::vec3& gravity);
    void setFixedTimeStep(float step) { fixedTimeStep_ = step; }
    float getFixedTimeStep() const { return fixedTimeStep_; }
    void setPaused(bool paused) { paused_ = paused; }
    bool isPaused() const { return paused_; }
    void setSolverIterations(int iterations) { world_.setSolverIterations(iterations); }

    // Returns interpolation alpha (0 when fixed-stepping; caller owns accumulator)
    float update(float deltaTime, int maxSubSteps = 1);
    void clear();

    PhysicsWorld& getWorld() { return world_; }
    const PhysicsWorld& getWorld() const { return world_; }

private:
    PhysicsWorld world_;
    bool paused_ = false;
    float fixedTimeStep_ = 0.0f;
};

} // namespace ge
