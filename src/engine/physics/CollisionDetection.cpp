#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <iostream>

namespace ge {

namespace {

struct OBB {
    glm::vec3 center;
    glm::vec3 axes[3];
    glm::vec3 halfExtents;
};

OBB getOBB(const glm::vec3& halfExtents, const glm::mat4& transform) {
    OBB obb;
    obb.center = glm::vec3(transform[3]);
    obb.axes[0] = glm::normalize(glm::vec3(transform[0]));
    obb.axes[1] = glm::normalize(glm::vec3(transform[1]));
    obb.axes[2] = glm::normalize(glm::vec3(transform[2]));

    glm::vec3 scale = glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
    obb.halfExtents = halfExtents * scale;
    return obb;
}

float getOverlap(const OBB& a, const OBB& b, const glm::vec3& axis) {
    if (glm::length2(axis) < 0.0001f) return std::numeric_limits<float>::max();
    glm::vec3 n = glm::normalize(axis);
    float radiusA = 0.0f;
    for (int i = 0; i < 3; ++i) radiusA += std::abs(glm::dot(a.axes[i] * a.halfExtents[i], n));
    float radiusB = 0.0f;
    for (int i = 0; i < 3; ++i) radiusB += std::abs(glm::dot(b.axes[i] * b.halfExtents[i], n));
    float distance = std::abs(glm::dot(b.center - a.center, n));
    return (radiusA + radiusB) - distance;
}

struct ClipVertex {
    glm::vec3 v;
    uint32_t id;
};

void clip(std::vector<ClipVertex>& vIn, const glm::vec3& n, float offset, std::vector<ClipVertex>& vOut) {
    vOut.clear();
    if (vIn.empty()) return;
    ClipVertex vA = vIn.back();
    float distA = glm::dot(vA.v, n) - offset;
    for (const auto& vB : vIn) {
        float distB = glm::dot(vB.v, n) - offset;
        if (distA * distB < 0.0f) {
            float t = distA / (distA - distB);
            vOut.push_back({vA.v + t * (vB.v - vA.v), vA.id});
        }
        if (distB >= 0.0f) vOut.push_back(vB);
        vA = vB;
        distA = distB;
    }
}

ContactManifold intersectBoxBox(const BoxCollider& boxA, const glm::mat4& transformA,
                                const BoxCollider& boxB, const glm::mat4& transformB) {
    ContactManifold result;
    OBB obbA = getOBB(boxA.getHalfExtents(), transformA);
    OBB obbB = getOBB(boxB.getHalfExtents(), transformB);

    glm::vec3 axes[15];
    for (int i = 0; i < 3; ++i) axes[i] = obbA.axes[i];
    for (int i = 0; i < 3; ++i) axes[i + 3] = obbB.axes[i];
    int axisIdx = 6;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            axes[axisIdx++] = glm::cross(obbA.axes[i], obbB.axes[j]);
        }
    }

    float minOverlap = std::numeric_limits<float>::max();
    int bestAxisIdx = -1;
    for (int i = 0; i < 15; ++i) {
        float overlap = getOverlap(obbA, obbB, axes[i]);
        if (overlap <= 0.0f) return result;
        float bias = (i >= 6) ? 0.95f : 1.0f;
        if (overlap * bias < minOverlap) {
            minOverlap = overlap;
            bestAxisIdx = i;
        }
    }

    result.isColliding = true;
    glm::vec3 normal = glm::normalize(axes[bestAxisIdx]);
    if (glm::dot(obbB.center - obbA.center, normal) < 0.0f) normal = -normal;
    result.normal = normal;

    const OBB* ref = &obbA;
    const OBB* inc = &obbB;
    bool flipped = false;
    if (bestAxisIdx >= 3 && bestAxisIdx < 6) {
        ref = &obbB;
        inc = &obbA;
        flipped = true;
    }

    glm::vec3 incNormal = flipped ? normal : -normal;
    int incAxisIdx = 0;
    float minDot = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        float d = glm::dot(incNormal, inc->axes[i]);
        if (d < minDot) { minDot = d; incAxisIdx = i; }
        if (-d < minDot) { minDot = -d; incAxisIdx = i + 3; }
    }

    glm::vec3 faceNormal = (incAxisIdx < 3) ? inc->axes[incAxisIdx] : -inc->axes[incAxisIdx - 3];
    std::vector<ClipVertex> incidentVertices;
    glm::vec3 incAxes[2];
    int incA0 = (incAxisIdx % 3 + 1) % 3;
    int incA1 = (incAxisIdx % 3 + 2) % 3;
    incAxes[0] = inc->axes[incA0];
    incAxes[1] = inc->axes[incA1];

    glm::vec3 faceCenter = inc->center + faceNormal * inc->halfExtents[incAxisIdx % 3];
    incidentVertices.push_back({faceCenter + incAxes[0] * inc->halfExtents[incA0] + incAxes[1] * inc->halfExtents[incA1], 0});
    incidentVertices.push_back({faceCenter - incAxes[0] * inc->halfExtents[incA0] + incAxes[1] * inc->halfExtents[incA1], 1});
    incidentVertices.push_back({faceCenter - incAxes[0] * inc->halfExtents[incA0] - incAxes[1] * inc->halfExtents[incA1], 2});
    incidentVertices.push_back({faceCenter + incAxes[0] * inc->halfExtents[incA0] - incAxes[1] * inc->halfExtents[incA1], 3});

    int refAxisIdx = bestAxisIdx % 3;
    glm::vec3 refAxes[2];
    int refA0 = (refAxisIdx + 1) % 3;
    int refA1 = (refAxisIdx + 2) % 3;
    refAxes[0] = ref->axes[refA0];
    refAxes[1] = ref->axes[refA1];

    std::vector<ClipVertex> vOut;
    clip(incidentVertices, refAxes[0], glm::dot(ref->center, refAxes[0]) - ref->halfExtents[refA0], vOut);
    clip(vOut, -refAxes[0], -glm::dot(ref->center, refAxes[0]) - ref->halfExtents[refA0], incidentVertices);
    clip(incidentVertices, refAxes[1], glm::dot(ref->center, refAxes[1]) - ref->halfExtents[refA1], vOut);
    clip(vOut, -refAxes[1], -glm::dot(ref->center, refAxes[1]) - ref->halfExtents[refA1], incidentVertices);

    float refOffset = glm::dot(ref->center, normal * (flipped ? -1.0f : 1.0f)) + ref->halfExtents[refAxisIdx];
    for (const auto& v : incidentVertices) {
        float depth = refOffset - glm::dot(v.v, normal * (flipped ? -1.0f : 1.0f));
        if (depth >= 0.0f) {
            Contact contact;
            contact.point = v.v;
            contact.normal = normal;
            contact.depth = depth;
            contact.persistentId = v.id;
            result.contacts.push_back(contact);
        }
    }
    return result;
}

ContactManifold intersectSphereSphere(const SphereCollider& sphereA, const glm::mat4& transformA,
                                      const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    glm::vec3 centerA = glm::vec3(transformA[3]);
    glm::vec3 centerB = glm::vec3(transformB[3]);
    float distance = glm::distance(centerA, centerB);
    float radiusSum = sphereA.getRadius() + sphereB.getRadius();
    if (distance <= radiusSum) {
        result.isColliding = true;
        Contact contact;
        contact.depth = radiusSum - distance;
        contact.normal = (distance > 0.001f) ? glm::normalize(centerB - centerA) : glm::vec3(0, 1, 0);
        contact.point = centerA + contact.normal * sphereA.getRadius();
        result.contacts.push_back(contact);
        result.normal = contact.normal;
    }
    return result;
}

ContactManifold intersectBoxSphere(const BoxCollider& boxA, const glm::mat4& transformA,
                                   const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    glm::mat4 invTransformA = glm::inverse(transformA);
    glm::vec3 localSphereCenter = glm::vec3(invTransformA * glm::vec4(glm::vec3(transformB[3]), 1.0f));
    float sphereRadius = sphereB.getRadius();
    glm::vec3 halfExtents = boxA.getHalfExtents();
    glm::vec3 localClosestPoint = glm::clamp(localSphereCenter, -halfExtents, halfExtents);
    float distanceSq = glm::distance2(localSphereCenter, localClosestPoint);
    if (distanceSq <= sphereRadius * sphereRadius) {
        result.isColliding = true;
        float distance = glm::sqrt(distanceSq);
        Contact contact;
        contact.depth = sphereRadius - distance;
        contact.normal = (distance > 0.001f) ?
            glm::normalize(glm::vec3(transformA * glm::vec4(glm::normalize(localSphereCenter - localClosestPoint), 0.0f))) :
            glm::normalize(glm::vec3(transformA[1]));
        contact.point = glm::vec3(transformA * glm::vec4(localClosestPoint, 1.0f));
        result.contacts.push_back(contact);
        result.normal = contact.normal;
    }
    return result;
}

} // namespace

ContactManifold CollisionDetection::checkCollision(
    const Collider& colliderA, const glm::mat4& transformA,
    const Collider& colliderB, const glm::mat4& transformB) {

    ColliderType typeA = colliderA.getType();
    ColliderType typeB = colliderB.getType();

    if (typeA == ColliderType::Box && typeB == ColliderType::Box) {
        return intersectBoxBox(static_cast<const BoxCollider&>(colliderA), transformA,
                               static_cast<const BoxCollider&>(colliderB), transformB);
    } else if (typeA == ColliderType::Sphere && typeB == ColliderType::Sphere) {
        return intersectSphereSphere(static_cast<const SphereCollider&>(colliderA), transformA,
                                     static_cast<const SphereCollider&>(colliderB), transformB);
    } else if (typeA == ColliderType::Box && typeB == ColliderType::Sphere) {
        return intersectBoxSphere(static_cast<const BoxCollider&>(colliderA), transformA,
                                  static_cast<const SphereCollider&>(colliderB), transformB);
    } else if (typeA == ColliderType::Sphere && typeB == ColliderType::Box) {
        ContactManifold manifold = intersectBoxSphere(static_cast<const BoxCollider&>(colliderB), transformB,
                                                      static_cast<const SphereCollider&>(colliderA), transformA);
        manifold.normal = -manifold.normal;
        for (auto& contact : manifold.contacts) contact.normal = -contact.normal;
        return manifold;
    }
    return {};
}

void CollisionDetection::detectCollisions(
    const std::vector<PotentialPair>& pairs,
    std::vector<ContactManifold>& manifolds) {
    manifolds.clear();

    for (const auto& pair : pairs) {
        RigidBody* bodyA = pair.bodyA;
        RigidBody* bodyB = pair.bodyB;

        ContactManifold manifold = checkCollision(
            bodyA->getCollider(),
            bodyA->getWorldTransform(),
            bodyB->getCollider(),
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
