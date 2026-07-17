#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <iostream>

namespace ge {

void CollisionDetection::detectCollisions(
    const std::vector<PotentialPair>& pairs,
    std::vector<ContactManifold>& manifolds) {
    manifolds.clear();

    for (const auto& pair : pairs) {
        RigidBody* bodyA = pair.bodyA;
        RigidBody* bodyB = pair.bodyB;

        ContactManifold manifold = bodyA->getCollider().checkCollision(
            bodyB->getCollider(),
            bodyA->getWorldTransform(),
            bodyB->getWorldTransform());

        if (manifold.isColliding) {
            manifold.bodyA = bodyA;
            manifold.bodyB = bodyB;
            for (auto& contact : manifold.contacts) {
                contact.bodyA = bodyA;
                contact.bodyB = bodyB;
                contact.normal = manifold.normal;
            }
            manifolds.push_back(std::move(manifold));
        }
    }
}

void CollisionDetection::resolveCollisions(std::vector<ContactManifold>& manifolds, int iterations) {
    for (int i = 0; i < iterations; ++i) {
        for (auto& manifold : manifolds) {
            resolveManifold(manifold);
        }
    }
}

void CollisionDetection::resolveManifold(ContactManifold& manifold) {
    for (auto& contact : manifold.contacts) {
        RigidBody* bodyA = contact.bodyA;
        RigidBody* bodyB = contact.bodyB;

        const RigidBodyProps& propsA = bodyA->getProps();
        const RigidBodyProps& propsB = bodyB->getProps();

        if (propsA.isTrigger || propsB.isTrigger) continue;

        const float invMassA = (propsA.isKinematic || propsA.mass <= 0.0f) ? 0.0f : 1.0f / propsA.mass;
        const float invMassB = (propsB.isKinematic || propsB.mass <= 0.0f) ? 0.0f : 1.0f / propsB.mass;
        const float totalInvMass = invMassA + invMassB;

        if (totalInvMass <= 0.0f) continue;

        const glm::vec3 rA = contact.point - bodyA->getPosition();
        const glm::vec3 rB = contact.point - bodyB->getPosition();

        const glm::vec3 vA = bodyA->getVelocity() + glm::cross(bodyA->getAngularVelocity(), rA);
        const glm::vec3 vB = bodyB->getVelocity() + glm::cross(bodyB->getAngularVelocity(), rB);
        const glm::vec3 relativeVelocity = vB - vA;

        const float velocityAlongNormal = glm::dot(relativeVelocity, contact.normal);

        // Positional correction (Baumgarte)
        const float biasFactor = 0.2f;
        const float slack = 0.02f;
        float bias = (biasFactor / 1.0f) * std::max(0.0f, contact.depth - slack);

        const glm::vec3 crossA = glm::cross(rA, contact.normal);
        const glm::vec3 crossB = glm::cross(rB, contact.normal);
        float angularTermA = glm::dot(crossA, bodyA->getInverseInertiaTensor() * crossA);
        float angularTermB = glm::dot(crossB, bodyB->getInverseInertiaTensor() * crossB);

        float massNormal = 1.0f / (totalInvMass + angularTermA + angularTermB);

        // Sequential Impulse: calculate delta impulse and accumulate
        float j = (-(1.0f + std::min(propsA.restitution, propsB.restitution)) * velocityAlongNormal + bias) * massNormal;

        float oldNormalImpulse = contact.normalImpulse;
        contact.normalImpulse = std::max(oldNormalImpulse + j, 0.0f);
        j = contact.normalImpulse - oldNormalImpulse;

        glm::vec3 impulseVec = contact.normal * j;

        bodyA->setVelocity(bodyA->getVelocity() - impulseVec * invMassA);
        bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, impulseVec));

        bodyB->setVelocity(bodyB->getVelocity() + impulseVec * invMassB);
        bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, impulseVec));

        // Friction
        const glm::vec3 vA_f = bodyA->getVelocity() + glm::cross(bodyA->getAngularVelocity(), rA);
        const glm::vec3 vB_f = bodyB->getVelocity() + glm::cross(bodyB->getAngularVelocity(), rB);
        const glm::vec3 relVel_f = vB_f - vA_f;

        glm::vec3 tangent = relVel_f - contact.normal * glm::dot(relVel_f, contact.normal);
        if (glm::length2(tangent) > 0.0001f) {
            tangent = glm::normalize(tangent);

            const glm::vec3 crossATan = glm::cross(rA, tangent);
            const glm::vec3 crossBTan = glm::cross(rB, tangent);
            float angularTermATan = glm::dot(crossATan, bodyA->getInverseInertiaTensor() * crossATan);
            float angularTermBTan = glm::dot(crossBTan, bodyB->getInverseInertiaTensor() * crossBTan);

            float massTangent = 1.0f / (totalInvMass + angularTermATan + angularTermBTan);
            float jt = -glm::dot(relVel_f, tangent) * massTangent;

            const float friction = std::min(propsA.friction, propsB.friction);
            float maxFriction = contact.normalImpulse * friction;

            float oldTangentImpulse = contact.tangentImpulse;
            contact.tangentImpulse = std::clamp(oldTangentImpulse + jt, -maxFriction, maxFriction);
            jt = contact.tangentImpulse - oldTangentImpulse;

            glm::vec3 frictionImpulseVec = tangent * jt;

            bodyA->setVelocity(bodyA->getVelocity() - frictionImpulseVec * invMassA);
            bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, frictionImpulseVec));

            bodyB->setVelocity(bodyB->getVelocity() + frictionImpulseVec * invMassB);
            bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, frictionImpulseVec));
        }
    }
}

} // namespace ge
