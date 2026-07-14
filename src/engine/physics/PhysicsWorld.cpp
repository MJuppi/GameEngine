#include "engine/physics/PhysicsWorld.h"

#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"

#include <algorithm>

namespace ge {

PhysicsWorld::PhysicsWorld() {

}

PhysicsWorld::~PhysicsWorld() {
    clearBodies();
}

void PhysicsWorld::setGravity(const glm::vec3& gravity) {
    gravity_ = gravity;
}

RigidBody* PhysicsWorld::addBody(std::unique_ptr<RigidBody> body) {
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

void PhysicsWorld::removeBody(RigidBody* body) {
    auto it = std::find_if(bodies_.begin(), bodies_.end(),
        [body](const std::unique_ptr<RigidBody>& ptr) { return ptr.get() == body; });

    if (it != bodies_.end()) {
        bodies_.erase(it);
    }
}

void PhysicsWorld::clearBodies() {
    bodies_.clear();
}

void PhysicsWorld::applyGravity() {
    for (auto& body : bodies_) {
        if (body->getProps().useGravity && !body->getProps().isKinematic) {
            body->addForce(gravity_ * body->getProps().mass);
        }
    }
}

void PhysicsWorld::step(float deltaTime, int /*maxSubSteps*/) {
    if (deltaTime <= 0.0f) {
        return;
    }

    applyGravity();

    for (auto& body : bodies_) {
        body->integrate(deltaTime);
    }

    std::vector<Contact> contacts;
    for (int iteration = 0; iteration < solverIterations_; ++iteration) {
        CollisionDetection::detectCollisions(bodies_, contacts);
        if (contacts.empty()) {
            break;
        }
        CollisionDetection::resolveCollisions(contacts);
    }
}

} // namespace ge
