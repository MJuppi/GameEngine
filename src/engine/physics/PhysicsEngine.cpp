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
