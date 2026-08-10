#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "engine/physics/BoxCollider.h"
#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"

namespace ge {
namespace {

uint32_t hashCombine(uint32_t seed, uint32_t value) {
    return seed ^ (value + 0x9e3779b9u + (seed << 6) + (seed >> 2));
}

glm::vec3 extractScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

struct OBB {
    glm::vec3 center;
    glm::vec3 axes[3];
    glm::vec3 halfExtents;
};

OBB makeOBB(const glm::vec3& halfExtents, const glm::mat4& transform) {
    OBB obb;
    obb.center = glm::vec3(transform[3]);

    const glm::vec3 axisX = glm::vec3(transform[0]);
    const glm::vec3 axisY = glm::vec3(transform[1]);
    const glm::vec3 axisZ = glm::vec3(transform[2]);
    const glm::vec3 scale = extractScale(transform);

    obb.axes[0] = glm::length2(axisX) > 1e-8f ? axisX / scale.x : glm::vec3(1.0f, 0.0f, 0.0f);
    obb.axes[1] = glm::length2(axisY) > 1e-8f ? axisY / scale.y : glm::vec3(0.0f, 1.0f, 0.0f);
    obb.axes[2] = glm::length2(axisZ) > 1e-8f ? axisZ / scale.z : glm::vec3(0.0f, 0.0f, 1.0f);
    obb.halfExtents = halfExtents * scale;
    return obb;
}

float projectRadius(const OBB& obb, const glm::vec3& axis) {
    return std::abs(glm::dot(obb.axes[0] * obb.halfExtents.x, axis)) +
           std::abs(glm::dot(obb.axes[1] * obb.halfExtents.y, axis)) +
           std::abs(glm::dot(obb.axes[2] * obb.halfExtents.z, axis));
}

ContactManifold intersectBoxBox(const BoxCollider& boxA, const glm::mat4& transformA,
                                const BoxCollider& boxB, const glm::mat4& transformB) {
    ContactManifold result;
    const OBB a = makeOBB(boxA.getHalfExtents(), transformA);
    const OBB b = makeOBB(boxB.getHalfExtents(), transformB);

    glm::vec3 axes[15];
    int axisCount = 0;
    for (int i = 0; i < 3; ++i) axes[axisCount++] = a.axes[i];
    for (int i = 0; i < 3; ++i) axes[axisCount++] = b.axes[i];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 cross = glm::cross(a.axes[i], b.axes[j]);
            const float len2 = glm::length2(cross);
            if (len2 > 1e-8f) {
                axes[axisCount++] = cross / std::sqrt(len2);
            }
        }
    }

    float minPenetration = std::numeric_limits<float>::max();
    glm::vec3 bestNormal(0.0f);
    for (int i = 0; i < axisCount; ++i) {
        const glm::vec3 axis = axes[i];
        if (glm::length2(axis) < 1e-8f) {
            continue;
        }

        const float ra = projectRadius(a, axis);
        const float rb = projectRadius(b, axis);
        const float distance = std::abs(glm::dot(b.center - a.center, axis));
        const float penetration = (ra + rb) - distance;
        if (penetration <= 0.0f) {
            return result;
        }

        if (penetration < minPenetration) {
            minPenetration = penetration;
            bestNormal = axis;
        }
    }

    if (glm::dot(b.center - a.center, bestNormal) < 0.0f) {
        bestNormal = -bestNormal;
    }

    result.isColliding = true;
    result.normal = glm::normalize(bestNormal);
    Contact contact;
    contact.normal = result.normal;
    contact.point = 0.5f * (a.center + b.center);
    contact.depth = minPenetration;
    result.contacts.push_back(contact);
    return result;
}

ContactManifold intersectSphereSphere(const SphereCollider& sphereA, const glm::mat4& transformA,
                                      const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    const glm::vec3 centerA = glm::vec3(transformA[3]);
    const glm::vec3 centerB = glm::vec3(transformB[3]);
    const glm::vec3 scaleA = extractScale(transformA);
    const glm::vec3 scaleB = extractScale(transformB);
    const float radiusA = sphereA.getRadius() * std::max({scaleA.x, scaleA.y, scaleA.z});
    const float radiusB = sphereB.getRadius() * std::max({scaleB.x, scaleB.y, scaleB.z});
    const float distance = glm::distance(centerA, centerB);
    const float radiusSum = radiusA + radiusB;
    if (distance <= radiusSum) {
        result.isColliding = true;
        Contact contact;
        contact.depth = radiusSum - distance;
        contact.normal = (distance > 1e-4f) ? glm::normalize(centerB - centerA) : glm::vec3(0.0f, 1.0f, 0.0f);
        contact.point = centerA + contact.normal * radiusA;
        result.contacts.push_back(contact);
        result.normal = contact.normal;
    }
    return result;
}

ContactManifold intersectBoxSphere(const BoxCollider& boxA, const glm::mat4& transformA,
                                   const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    const glm::mat4 invTransform = glm::inverse(transformA);
    const glm::vec3 localSphereCenter = glm::vec3(invTransform * glm::vec4(glm::vec3(transformB[3]), 1.0f));
    const glm::vec3 scaleB = extractScale(transformB);
    const float sphereRadius = sphereB.getRadius() * std::max({scaleB.x, scaleB.y, scaleB.z});
    const glm::vec3 halfExtents = boxA.getHalfExtents();
    const glm::vec3 localClosestPoint = glm::clamp(localSphereCenter, -halfExtents, halfExtents);
    const float distanceSq = glm::distance2(localSphereCenter, localClosestPoint);
    if (distanceSq <= sphereRadius * sphereRadius) {
        result.isColliding = true;
        const float distance = std::sqrt(distanceSq);
        Contact contact;
        contact.depth = sphereRadius - distance;
        contact.normal = (distance > 1e-4f) ?
            glm::normalize(glm::vec3(transformA * glm::vec4(glm::normalize(localSphereCenter - localClosestPoint), 0.0f))) :
            glm::normalize(glm::vec3(transformA[1]));
        contact.point = glm::vec3(transformA * glm::vec4(localClosestPoint, 1.0f));
        result.contacts.push_back(contact);
        result.normal = contact.normal;
    }
    return result;
}
} // namespace

uint32_t makeContactId(RigidBody* bodyA, RigidBody* bodyB, const glm::vec3& point) {
    uint32_t hash = 0;
    hash = hashCombine(hash, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bodyA) >> 4));
    hash = hashCombine(hash, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bodyB) >> 4));
    hash = hashCombine(hash, static_cast<uint32_t>(point.x * 100.0f));
    hash = hashCombine(hash, static_cast<uint32_t>(point.y * 100.0f));
    hash = hashCombine(hash, static_cast<uint32_t>(point.z * 100.0f));
    return hash;
}

ContactManifold CollisionDetection::checkCollision(const Collider& colliderA, const glm::mat4& transformA,
                                                  const Collider& colliderB, const glm::mat4& transformB) {
    if (colliderA.getType() == ColliderType::Box && colliderB.getType() == ColliderType::Box) {
        return intersectBoxBox(static_cast<const BoxCollider&>(colliderA), transformA,
                               static_cast<const BoxCollider&>(colliderB), transformB);
    }
    if (colliderA.getType() == ColliderType::Sphere && colliderB.getType() == ColliderType::Sphere) {
        return intersectSphereSphere(static_cast<const SphereCollider&>(colliderA), transformA,
                                     static_cast<const SphereCollider&>(colliderB), transformB);
    }
    if (colliderA.getType() == ColliderType::Box && colliderB.getType() == ColliderType::Sphere) {
        return intersectBoxSphere(static_cast<const BoxCollider&>(colliderA), transformA,
                                  static_cast<const SphereCollider&>(colliderB), transformB);
    }
    if (colliderA.getType() == ColliderType::Sphere && colliderB.getType() == ColliderType::Box) {
        ContactManifold manifold = intersectBoxSphere(static_cast<const BoxCollider&>(colliderB), transformB,
                                                      static_cast<const SphereCollider&>(colliderA), transformA);
        manifold.normal = -manifold.normal;
        for (auto& contact : manifold.contacts) {
            contact.normal = -contact.normal;
        }
        return manifold;
    }
    return {};
}

void CollisionDetection::detectCollisions(const std::vector<PotentialPair>& pairs,
                                          std::vector<ContactManifold>& manifolds) {
    manifolds.clear();
    for (const auto& pair : pairs) {
        ContactManifold manifold = checkCollision(pair.bodyA->getCollider(), pair.bodyA->getWorldTransform(),
                                                  pair.bodyB->getCollider(), pair.bodyB->getWorldTransform());
        if (!manifold.isColliding) {
            continue;
        }

        manifold.bodyA = pair.bodyA;
        manifold.bodyB = pair.bodyB;
        for (auto& contact : manifold.contacts) {
            contact.bodyA = pair.bodyA;
            contact.bodyB = pair.bodyB;
            contact.normal = manifold.normal;
            contact.persistentId = makeContactId(pair.bodyA, pair.bodyB, contact.point);
        }
        manifolds.push_back(std::move(manifold));
    }
}

void CollisionDetection::resolveCollisions(std::vector<ContactManifold>& manifolds, int iterations, float deltaTime) {
    for (int i = 0; i < iterations; ++i) {
        for (auto& manifold : manifolds) {
            resolveManifold(manifold, deltaTime);
        }
    }
}

void CollisionDetection::resolveManifold(ContactManifold& manifold, float deltaTime) {
    for (auto& contact : manifold.contacts) {
        RigidBody* bodyA = contact.bodyA;
        RigidBody* bodyB = contact.bodyB;
        const RigidBodyProps& propsA = bodyA->getProps();
        const RigidBodyProps& propsB = bodyB->getProps();

        if (propsA.isTrigger || propsB.isTrigger) {
            continue;
        }

        const float invMassA = (propsA.isKinematic || propsA.mass <= 0.0f) ? 0.0f : 1.0f / propsA.mass;
        const float invMassB = (propsB.isKinematic || propsB.mass <= 0.0f) ? 0.0f : 1.0f / propsB.mass;
        const float totalInvMass = invMassA + invMassB;
        if (totalInvMass <= 0.0f) {
            continue;
        }

        const glm::vec3 rA = contact.point - bodyA->getPosition();
        const glm::vec3 rB = contact.point - bodyB->getPosition();
        const glm::vec3 vA = bodyA->getVelocity() + glm::cross(bodyA->getAngularVelocity(), rA);
        const glm::vec3 vB = bodyB->getVelocity() + glm::cross(bodyB->getAngularVelocity(), rB);
        const glm::vec3 relativeVelocity = vB - vA;

        const float angularTermA = glm::dot(glm::cross(rA, contact.normal), bodyA->getInverseInertiaTensor() * glm::cross(rA, contact.normal));
        const float angularTermB = glm::dot(glm::cross(rB, contact.normal), bodyB->getInverseInertiaTensor() * glm::cross(rB, contact.normal));
        const float massNormal = 1.0f / (totalInvMass + angularTermA + angularTermB);
        const float velocityAlongNormal = glm::dot(relativeVelocity, contact.normal);

        float restitution = std::min(propsA.restitution, propsB.restitution);
        if (std::abs(velocityAlongNormal) < 0.2f) {
            restitution = 0.0f;
        }

        const float biasFactor = 0.2f;
        const float slop = 0.005f;
        const float bias = (biasFactor / std::max(deltaTime, 1e-4f)) * std::max(0.0f, contact.depth - slop);
        float j = (-(1.0f + restitution) * velocityAlongNormal + bias) * massNormal;

        const float oldNormalImpulse = contact.normalImpulse;
        contact.normalImpulse = std::max(oldNormalImpulse + j, 0.0f);
        j = contact.normalImpulse - oldNormalImpulse;

        const glm::vec3 impulse = contact.normal * j;
        bodyA->setVelocity(bodyA->getVelocity() - impulse * invMassA);
        bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, impulse));
        bodyB->setVelocity(bodyB->getVelocity() + impulse * invMassB);
        bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, impulse));

        const glm::vec3 vA_f = bodyA->getVelocity() + glm::cross(bodyA->getAngularVelocity(), rA);
        const glm::vec3 vB_f = bodyB->getVelocity() + glm::cross(bodyB->getAngularVelocity(), rB);
        const glm::vec3 relVelF = vB_f - vA_f;
        glm::vec3 tangent = relVelF - contact.normal * glm::dot(relVelF, contact.normal);
        if (glm::length2(tangent) > 1e-6f) {
            tangent = glm::normalize(tangent);
            const glm::vec3 crossATan = glm::cross(rA, tangent);
            const glm::vec3 crossBTan = glm::cross(rB, tangent);
            const float angularTermATan = glm::dot(crossATan, bodyA->getInverseInertiaTensor() * crossATan);
            const float angularTermBTan = glm::dot(crossBTan, bodyB->getInverseInertiaTensor() * crossBTan);
            const float massTangent = 1.0f / (totalInvMass + angularTermATan + angularTermBTan);
            float jt = -glm::dot(relVelF, tangent) * massTangent;

            const float staticFriction = std::min(propsA.staticFriction, propsB.staticFriction);
            const float dynamicFriction = std::min(propsA.friction, propsB.friction);
            const float maxStatic = std::abs(contact.normalImpulse) * staticFriction;
            const float maxDynamic = std::abs(contact.normalImpulse) * dynamicFriction;
            const float newTangent = contact.tangentImpulse + jt;
            if (std::abs(newTangent) <= maxStatic) {
                contact.tangentImpulse = newTangent;
            } else {
                contact.tangentImpulse = glm::clamp(newTangent, -maxDynamic, maxDynamic);
            }
            jt = contact.tangentImpulse - (contact.tangentImpulse - jt);

            const glm::vec3 frictionImpulse = tangent * jt;
            bodyA->setVelocity(bodyA->getVelocity() - frictionImpulse * invMassA);
            bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, frictionImpulse));
            bodyB->setVelocity(bodyB->getVelocity() + frictionImpulse * invMassB);
            bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, frictionImpulse));
        }
    }
}

void CollisionDetection::correctPositions(std::vector<ContactManifold>& manifolds) {
    constexpr float percent = 0.8f;
    constexpr float slop = 0.001f;

    for (auto& manifold : manifolds) {
        for (auto& contact : manifold.contacts) {
            RigidBody* bodyA = contact.bodyA;
            RigidBody* bodyB = contact.bodyB;
            if (bodyA->getProps().isTrigger || bodyB->getProps().isTrigger) {
                continue;
            }

            const float invMassA = (bodyA->getProps().isKinematic || bodyA->getProps().mass <= 0.0f) ? 0.0f : 1.0f / bodyA->getProps().mass;
            const float invMassB = (bodyB->getProps().isKinematic || bodyB->getProps().mass <= 0.0f) ? 0.0f : 1.0f / bodyB->getProps().mass;
            const float totalInvMass = invMassA + invMassB;
            if (totalInvMass <= 0.0f) {
                continue;
            }

            const float correction = std::max(contact.depth - slop, 0.0f) / totalInvMass * percent;
            const glm::vec3 correctionVec = contact.normal * correction;
            if (invMassA > 0.0f) {
                bodyA->movePosition(-correctionVec * invMassA);
            }
            if (invMassB > 0.0f) {
                bodyB->movePosition(correctionVec * invMassB);
            }
        }
    }
}

} // namespace ge
