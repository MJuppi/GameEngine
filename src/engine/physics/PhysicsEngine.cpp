#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/PhysicsWorld.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"

namespace ge {

PhysicsEngine::PhysicsEngine() : fixedTimeStep_(1.0f / 60.0f) {}
PhysicsEngine::~PhysicsEngine() = default;

RigidBody* PhysicsEngine::createBoxBody(const glm::vec3& halfExtents,
                                       const glm::mat4& transform,
                                       const RigidBodyProps& props) {
    auto collider = std::make_unique<BoxCollider>(halfExtents);
    auto body = std::make_unique<RigidBody>(std::move(collider), transform, props);
    return world_.addBody(std::move(body));
}

RigidBody* PhysicsEngine::createActiveBoxBody(const glm::vec3& visualHalfExtents,
                                              const glm::mat4& transform,
                                              const RigidBodyProps& props) {
    glm::mat4 scaledTransform = glm::scale(transform, visualHalfExtents);
    // Active cube visuals use a unit mesh (-0.5..0.5). Keep physics in sync
    // by scaling collider half extents with the same factor used for rendering.
    auto collider = std::make_unique<BoxCollider>(glm::vec3(0.5f) * visualHalfExtents);
    auto body = std::make_unique<RigidBody>(std::move(collider), scaledTransform, props);
    return world_.addBody(std::move(body));
}

RigidBody* PhysicsEngine::createSphereBody(float radius,
                                           const glm::mat4& transform,
                                           const RigidBodyProps& props) {
    auto collider = std::make_unique<SphereCollider>(radius);
    auto body = std::make_unique<RigidBody>(std::move(collider), transform, props);
    return world_.addBody(std::move(body));
}

void PhysicsEngine::destroyBody(RigidBody* body) {
    world_.removeBody(body);
}

cannon::PointToPointConstraint* PhysicsEngine::addPointToPointConstraint(RigidBody* bodyA,
                                                                         const glm::vec3& pivotA,
                                                                         RigidBody* bodyB,
                                                                         const glm::vec3& pivotB,
                                                                         float maxForce,
                                                                         bool collideConnected) {
    return world_.addPointToPointConstraint(bodyA, pivotA, bodyB, pivotB, maxForce, collideConnected);
}

cannon::DistanceConstraint* PhysicsEngine::addDistanceConstraint(RigidBody* bodyA,
                                                                 RigidBody* bodyB,
                                                                 float distance,
                                                                 float maxForce,
                                                                 bool collideConnected) {
    return world_.addDistanceConstraint(bodyA, bodyB, distance, maxForce, collideConnected);
}

cannon::LockConstraint* PhysicsEngine::addLockConstraint(RigidBody* bodyA,
                                                         RigidBody* bodyB,
                                                         float maxForce,
                                                         bool collideConnected) {
    return world_.addLockConstraint(bodyA, bodyB, maxForce, collideConnected);
}

cannon::HingeConstraint* PhysicsEngine::addHingeConstraint(RigidBody* bodyA,
                                                          RigidBody* bodyB,
                                                          const glm::vec3& pivotA,
                                                          const glm::vec3& pivotB,
                                                          const glm::vec3& axisA,
                                                          const glm::vec3& axisB,
                                                          float maxForce,
                                                          bool collideConnected) {
    return world_.addHingeConstraint(bodyA, bodyB, pivotA, pivotB, axisA, axisB, maxForce, collideConnected);
}

cannon::ConeTwistConstraint* PhysicsEngine::addConeTwistConstraint(RigidBody* bodyA,
                                                                   RigidBody* bodyB,
                                                                   const glm::vec3& pivotA,
                                                                   const glm::vec3& pivotB,
                                                                   const glm::vec3& axisA,
                                                                   const glm::vec3& axisB,
                                                                   float angle,
                                                                   float twistAngle,
                                                                   float maxForce,
                                                                   bool collideConnected) {
    return world_.addConeTwistConstraint(bodyA, bodyB, pivotA, pivotB, axisA, axisB, angle, twistAngle, maxForce, collideConnected);
}

void PhysicsEngine::removeConstraint(cannon::Constraint* constraint) {
    world_.removeConstraint(constraint);
}

void PhysicsEngine::clearConstraints() {
    world_.clearConstraints();
}

void PhysicsEngine::setGravity(const glm::vec3& gravity) {
    world_.setGravity(gravity);
}

float PhysicsEngine::update(float deltaTime, int maxSubSteps) {
    if (paused_ || deltaTime <= 0.0f) {
        return 0.0f;
    }

    const float stepSize = fixedTimeStep_ > 0.0f ? fixedTimeStep_ : deltaTime;
    world_.step(stepSize, maxSubSteps);
    return fixedTimeStep_ > 0.0f ? 0.0f : 1.0f;
}

void PhysicsEngine::clear() {
    world_.clearBodies();
}

} // namespace ge
