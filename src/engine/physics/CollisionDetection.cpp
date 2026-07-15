#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <iostream>

namespace ge {

void CollisionDetection::detectCollisions(
    const std::vector<std::unique_ptr<RigidBody>>& bodies,
    std::vector<Contact>& contacts) {
    contacts.clear();

    const std::size_t bodyCount = bodies.size();
    if (bodyCount < 2) {
        return;
    }

    for (std::size_t i = 0; i < bodyCount; ++i) {
        for (std::size_t j = i + 1; j < bodyCount; ++j) {
            RigidBody* bodyA = bodies[i].get();
            RigidBody* bodyB = bodies[j].get();

            CollisionResult collision = bodyA->getCollider().checkCollision(
                bodyB->getCollider(),
                bodyA->getWorldTransform(),
                bodyB->getWorldTransform());

            if (collision.isColliding) {
                for (auto& contact : collision.contacts) {
                    contact.bodyA = bodyA;
                    contact.bodyB = bodyB;
                    contacts.push_back(contact);
                }
            }
        }
    }
}

void CollisionDetection::resolveCollisions(std::vector<Contact>& contacts) {
    for (auto& contact : contacts) {
        resolveContact(contact);
    }
}

void CollisionDetection::resolveContact(Contact& contact) {
    RigidBody* bodyA = contact.bodyA;
    RigidBody* bodyB = contact.bodyB;
    if (bodyA == nullptr || bodyB == nullptr) {
        return;
    }

    const RigidBodyProps& propsA = bodyA->getProps();
    const RigidBodyProps& propsB = bodyB->getProps();

    const float invMassA = (propsA.isKinematic || propsA.mass <= 0.0f) ? 0.0f : 1.0f / propsA.mass;
    const float invMassB = (propsB.isKinematic || propsB.mass <= 0.0f) ? 0.0f : 1.0f / propsB.mass;
    const float totalInvMass = invMassA + invMassB;

    if (totalInvMass <= 0.0f) {
        return;
    }

    // Vectors from center of mass to contact point
    const glm::vec3 rA = contact.point - bodyA->getPosition();
    const glm::vec3 rB = contact.point - bodyB->getPosition();

    // Relative velocity at contact point
    const glm::vec3 vA = bodyA->getVelocity() + glm::cross(bodyA->getAngularVelocity(), rA);
    const glm::vec3 vB = bodyB->getVelocity() + glm::cross(bodyB->getAngularVelocity(), rB);
    const glm::vec3 relativeVelocity = vB - vA;

    const float velocityAlongNormal = glm::dot(relativeVelocity, contact.normal);

    // Do not resolve if velocities are separating
    if (velocityAlongNormal > 0.0f) {
        return;
    }

    const float restitution = std::min(propsA.restitution, propsB.restitution);

    // Calculate impulse scalar with rotational terms
    const glm::vec3 crossA = glm::cross(rA, contact.normal);
    const glm::vec3 crossB = glm::cross(rB, contact.normal);

    float angularTermA = glm::dot(crossA, bodyA->getInverseInertiaTensor() * crossA);
    float angularTermB = glm::dot(crossB, bodyB->getInverseInertiaTensor() * crossB);

    const float impulse = -(1.0f + restitution) * velocityAlongNormal / (totalInvMass + angularTermA + angularTermB);

    // Apply impulse
    const glm::vec3 impulseVec = contact.normal * impulse;

    bodyA->setVelocity(bodyA->getVelocity() - impulseVec * invMassA);
    bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, impulseVec));

    bodyB->setVelocity(bodyB->getVelocity() + impulseVec * invMassB);
    bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, impulseVec));

    // Friction impulse
    glm::vec3 tangent = relativeVelocity - contact.normal * velocityAlongNormal;
    if (glm::length(tangent) > 1e-4f) {
        tangent = glm::normalize(tangent);

        const glm::vec3 crossATan = glm::cross(rA, tangent);
        const glm::vec3 crossBTan = glm::cross(rB, tangent);
        float angularTermATan = glm::dot(crossATan, bodyA->getInverseInertiaTensor() * crossATan);
        float angularTermBTan = glm::dot(crossBTan, bodyB->getInverseInertiaTensor() * crossBTan);

        float frictionImpulse = -glm::dot(relativeVelocity, tangent) / (totalInvMass + angularTermATan + angularTermBTan);

        const float friction = std::min(propsA.friction, propsB.friction);
        const float maxFriction = std::abs(impulse) * friction;
        frictionImpulse = std::clamp(frictionImpulse, -maxFriction, maxFriction);

        const glm::vec3 frictionImpulseVec = tangent * frictionImpulse;

        bodyA->setVelocity(bodyA->getVelocity() - frictionImpulseVec * invMassA);
        bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, frictionImpulseVec));

        bodyB->setVelocity(bodyB->getVelocity() + frictionImpulseVec * invMassB);
        bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, frictionImpulseVec));
    }

    // Positional correction (Slack/Baumgarte)
    const float correctionFactor = 0.4f; // Reduced from 0.8 for stability
    const float correctionMargin = 0.01f; // Increased from 0.001
    if (contact.depth > correctionMargin) {
        const glm::vec3 correction =
            contact.normal * ((contact.depth - correctionMargin) * correctionFactor / totalInvMass);

        if (invMassA > 0.0f) {
            bodyA->setPosition(bodyA->getPosition() - correction * invMassA);
            // We don't updateTransform here to avoid redundant calculations,
            // the physics step will do it at the end if needed,
            // but for simple resolution it's fine.
        }

        if (invMassB > 0.0f) {
            bodyB->setPosition(bodyB->getPosition() + correction * invMassB);
        }
    }
}

} // namespace ge
