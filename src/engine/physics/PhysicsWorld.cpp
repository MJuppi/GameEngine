#include "engine/physics/PhysicsWorld.h"

#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/BroadPhase.h"
#include "engine/physics/BoxCollider.h"

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

PhysicsWorld::RaycastResult PhysicsWorld::raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) {
    RaycastResult bestResult;
    bestResult.fraction = 1.0f;
    glm::vec3 dir = glm::normalize(direction);

    for (const auto& body : bodies_) {
        const auto& collider = body->getCollider();
        if (collider.getType() == ColliderType::Box) {
            const auto& box = static_cast<const BoxCollider&>(collider);
            glm::mat4 invTransform = glm::inverse(body->getWorldTransform());
            glm::vec3 localOrigin = glm::vec3(invTransform * glm::vec4(origin, 1.0f));
            glm::vec3 localDir = glm::normalize(glm::vec3(invTransform * glm::vec4(dir, 0.0f)));
            glm::vec3 h = box.getHalfExtents();

            float tMin = 0.0f;
            float tMax = maxDistance;

            for (int i = 0; i < 3; ++i) {
                float invD = 1.0f / localDir[i];
                float t0 = (-h[i] - localOrigin[i]) * invD;
                float t1 = (h[i] - localOrigin[i]) * invD;
                if (invD < 0.0f) std::swap(t0, t1);
                tMin = std::max(tMin, t0);
                tMax = std::min(tMax, t1);
                if (tMax <= tMin) break;
            }

            if (tMax > tMin && tMin < bestResult.fraction * maxDistance && tMin > 0.0f) {
                bestResult.hit = true;
                bestResult.body = body.get();
                bestResult.fraction = tMin / maxDistance;
                bestResult.point = origin + dir * tMin;
                // Normal would be one of the box axes
                bestResult.normal = body->getWorldTransform()[0]; // Simplified
            }
        }
    }
    return bestResult;
}

void PhysicsWorld::warmStart(std::vector<ContactManifold>& manifolds) {
    for (auto& manifold : manifolds) {
        for (const auto& cachedManifold : manifoldCache_) {
            if ((manifold.bodyA == cachedManifold.bodyA && manifold.bodyB == cachedManifold.bodyB) ||
                (manifold.bodyA == cachedManifold.bodyB && manifold.bodyB == cachedManifold.bodyA)) {

                for (auto& contact : manifold.contacts) {
                    for (const auto& cachedContact : cachedManifold.contacts) {
                        if (contact.persistentId == cachedContact.persistentId) {
                            contact.normalImpulse = cachedContact.normalImpulse;
                            contact.tangentImpulse = cachedContact.tangentImpulse;

                            float invMassA = (contact.bodyA->getProps().isKinematic || contact.bodyA->getProps().mass <= 0.0f) ? 0.0f : 1.0f / contact.bodyA->getProps().mass;
                            float invMassB = (contact.bodyB->getProps().isKinematic || contact.bodyB->getProps().mass <= 0.0f) ? 0.0f : 1.0f / contact.bodyB->getProps().mass;

                            glm::vec3 rA = contact.point - contact.bodyA->getPosition();
                            glm::vec3 rB = contact.point - contact.bodyB->getPosition();

                            glm::vec3 impulseNormal = contact.normal * contact.normalImpulse;
                            contact.bodyA->setVelocity(contact.bodyA->getVelocity() - impulseNormal * invMassA);
                            contact.bodyA->setAngularVelocity(contact.bodyA->getAngularVelocity() - contact.bodyA->getInverseInertiaTensor() * glm::cross(rA, impulseNormal));
                            contact.bodyB->setVelocity(contact.bodyB->getVelocity() + impulseNormal * invMassB);
                            contact.bodyB->setAngularVelocity(contact.bodyB->getAngularVelocity() - contact.bodyB->getInverseInertiaTensor() * glm::cross(rB, -impulseNormal));
                        }
                    }
                }
            }
        }
    }
}

void PhysicsWorld::resolveContacts(std::vector<ContactManifold>& manifolds) {
    if (manifolds.empty()) return;
    CollisionDetection::resolveCollisions(manifolds, solverIterations_);
}

void PhysicsWorld::step(float deltaTime, int /*maxSubSteps*/) {
    if (deltaTime <= 0.0f) return;

    applyGravity();

    for (auto& body : bodies_) {
        float speed = glm::length(body->getVelocity());
        if (speed * deltaTime > 0.5f) {
             auto res = raycast(body->getPosition(), body->getVelocity(), speed * deltaTime);
             if (res.hit && res.body != body.get()) {
                 body->setPosition(res.point - glm::normalize(body->getVelocity()) * 0.01f);
                 body->setVelocity(glm::vec3(0.0f));
             }
        }
        body->integrate(deltaTime);
    }

    auto potentialPairs = BroadPhase::findPairs(bodies_);
    std::vector<ContactManifold> manifolds;
    CollisionDetection::detectCollisions(potentialPairs, manifolds);

    warmStart(manifolds);
    resolveContacts(manifolds);

    manifoldCache_ = std::move(manifolds);
}

} // namespace ge
