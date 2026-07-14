#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/PhysicsWorld.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"

namespace ge {

PhysicsEngine::PhysicsEngine() = default;

PhysicsEngine::~PhysicsEngine() = default;

RigidBody* PhysicsEngine::createBoxBody(const glm::vec3& halfExtents,
                                       const glm::mat4& transform,
                                       const RigidBodyProps& props) {
    auto collider = std::make_unique<BoxCollider>(halfExtents);
    auto body = std::make_unique<RigidBody>(std::move(collider), transform, props);
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

void PhysicsEngine::setGravity(const glm::vec3& gravity) {
    world_.setGravity(gravity);
}

void PhysicsEngine::update(float deltaTime, int maxSubSteps) {
    if (paused_ || deltaTime <= 0.0f) {
        return;
    }

    if (fixedTimeStep_ > 0.0f) {
        // Fixed time stepping
        accumulator_ += deltaTime;

        while (accumulator_ >= fixedTimeStep_) {
            world_.step(fixedTimeStep_, maxSubSteps);
            accumulator_ -= fixedTimeStep_;
        }
    } else {
        // Variable time stepping
        world_.step(deltaTime, maxSubSteps);
    }
}

void PhysicsEngine::clear() {
    world_.clearBodies();
}

} // namespace ge
