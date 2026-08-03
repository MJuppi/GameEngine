#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <glm/glm.hpp>

namespace ge {

uint32_t hashCombine(uint32_t seed, uint32_t value) {
    return seed ^ (value + 0x9e3779b9u + (seed << 6) + (seed >> 2));
}

uint32_t makeContactId(RigidBody* bodyA, RigidBody* bodyB, const glm::vec3& point) {
    uint32_t hash = 0;
    hash = hashCombine(hash, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bodyA) >> 4));
    hash = hashCombine(hash, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bodyB) >> 4));
    hash = hashCombine(hash, static_cast<uint32_t>(point.x * 100.0f));
    hash = hashCombine(hash, static_cast<uint32_t>(point.y * 100.0f));
    hash = hashCombine(hash, static_cast<uint32_t>(point.z * 100.0f));
    return hash;
}

glm::vec3 getScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

struct OBB {
    glm::vec3 center;
    glm::vec3 axes[3];   // unit vectors
    glm::vec3 halfExtents;
};

static OBB makeOBB(const glm::vec3& localHalfExtents, const glm::mat4& transform) {
    OBB obb;
    obb.center = glm::vec3(transform[3]);

    // Extract rotation + scale
    glm::vec3 sx = glm::vec3(transform[0]);
    glm::vec3 sy = glm::vec3(transform[1]);
    glm::vec3 sz = glm::vec3(transform[2]);

    float lenX = glm::length(sx);
    float lenY = glm::length(sy);
    float lenZ = glm::length(sz);

    obb.axes[0] = (lenX > 1e-6f) ? sx / lenX : glm::vec3(1,0,0);
    obb.axes[1] = (lenY > 1e-6f) ? sy / lenY : glm::vec3(0,1,0);
    obb.axes[2] = (lenZ > 1e-6f) ? sz / lenZ : glm::vec3(0,0,1);

    // Apply scale to half-extents (handles non-uniform scale correctly)
    obb.halfExtents = localHalfExtents * glm::vec3(lenX, lenY, lenZ);
    return obb;
}

static float projectRadius(const OBB& obb, const glm::vec3& axis) {
    return std::abs(glm::dot(obb.axes[0] * obb.halfExtents.x, axis)) +
           std::abs(glm::dot(obb.axes[1] * obb.halfExtents.y, axis)) +
           std::abs(glm::dot(obb.axes[2] * obb.halfExtents.z, axis));
}

ContactManifold intersectBoxBox(const BoxCollider& boxA, const glm::mat4& transformA,
                                const BoxCollider& boxB, const glm::mat4& transformB) {
    ContactManifold result;

    OBB a = makeOBB(boxA.getHalfExtents(), transformA);
    OBB b = makeOBB(boxB.getHalfExtents(), transformB);

    // 15 potential separating axes
    glm::vec3 axes[15];
    int axisCount = 0;

    // Face normals of A
    for (int i = 0; i < 3; ++i) axes[axisCount++] = a.axes[i];
    // Face normals of B
    for (int i = 0; i < 3; ++i) axes[axisCount++] = b.axes[i];
    // Edge cross products
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 cross = glm::cross(a.axes[i], b.axes[j]);
            float len2 = glm::length2(cross);
            if (len2 > 1e-6f)
                axes[axisCount++] = cross / std::sqrt(len2);
            else
                axes[axisCount++] = glm::vec3(0.0f); // will be skipped
        }
    }

    float minPenetration = std::numeric_limits<float>::max();
    glm::vec3 bestNormal(0.0f);
    int bestAxis = -1;

    for (int i = 0; i < axisCount; ++i) {
        glm::vec3 axis = axes[i];
        if (glm::length2(axis) < 1e-6f) continue;

        float ra = projectRadius(a, axis);
        float rb = projectRadius(b, axis);
        float dist = std::abs(glm::dot(b.center - a.center, axis));
        float penetration = (ra + rb) - dist;

        if (penetration <= 0.0f)
            return result; // separating axis found

        // Prefer face axes slightly (reduces jitter)
        float bias = (i < 6) ? 1.0f : 0.98f;
        if (penetration * bias < minPenetration) {
            minPenetration = penetration;
            bestNormal = axis;
            bestAxis = i;
        }
    }

    // Ensure normal points from A to B
    if (glm::dot(b.center - a.center, bestNormal) < 0.0f)
        bestNormal = -bestNormal;

    result.isColliding = true;
    result.normal = bestNormal;

    // ---- Contact generation (reference face + clip) ----
    // Determine which body owns the reference face
    bool aIsRef = (bestAxis < 3);          // face of A
    const OBB& ref = aIsRef ? a : b;
    const OBB& inc = aIsRef ? b : a;
    glm::vec3 refNormal = aIsRef ? bestNormal : -bestNormal;

    // Find the most anti-parallel face on the incident OBB
    int incFace = 0;
    float minDot = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        float d = glm::dot(inc.axes[i], -refNormal);
        if (d < minDot) { minDot = d; incFace = i; }
        if (-d < minDot) { minDot = -d; incFace = i + 3; }
    }

    // Build the 4 vertices of the incident face
    glm::vec3 faceNormal = (incFace < 3) ? inc.axes[incFace] : -inc.axes[incFace - 3];
    int axis0 = (incFace % 3 + 1) % 3;
    int axis1 = (incFace % 3 + 2) % 3;

    glm::vec3 center = inc.center + faceNormal * inc.halfExtents[incFace % 3];
    glm::vec3 e0 = inc.axes[axis0] * inc.halfExtents[axis0];
    glm::vec3 e1 = inc.axes[axis1] * inc.halfExtents[axis1];

    std::vector<glm::vec3> incident = {
        center + e0 + e1,
        center - e0 + e1,
        center - e0 - e1,
        center + e0 - e1
    };

    // Clip against the side planes of the reference face
    int refAxis = bestAxis % 3;
    int side0 = (refAxis + 1) % 3;
    int side1 = (refAxis + 2) % 3;

    auto clipPolygon = [](const std::vector<glm::vec3>& poly,
                          const glm::vec3& planeNormal, float planeOffset) {
        std::vector<glm::vec3> out;
        if (poly.empty()) return out;

        glm::vec3 prev = poly.back();
        float prevDist = glm::dot(prev, planeNormal) - planeOffset;

        for (const glm::vec3& curr : poly) {
            float currDist = glm::dot(curr, planeNormal) - planeOffset;
            if (prevDist * currDist < 0.0f) {
                float t = prevDist / (prevDist - currDist);
                out.push_back(prev + t * (curr - prev));
            }
            if (currDist >= 0.0f)
                out.push_back(curr);
            prev = curr;
            prevDist = currDist;
        }
        return out;
    };

    // Four side planes
    float plane0 = glm::dot(ref.center, ref.axes[side0]) + ref.halfExtents[side0];
    float plane1 = glm::dot(ref.center, -ref.axes[side0]) + ref.halfExtents[side0];
    float plane2 = glm::dot(ref.center, ref.axes[side1]) + ref.halfExtents[side1];
    float plane3 = glm::dot(ref.center, -ref.axes[side1]) + ref.halfExtents[side1];

    incident = clipPolygon(incident,  ref.axes[side0],  plane0);
    incident = clipPolygon(incident, -ref.axes[side0],  plane1);
    incident = clipPolygon(incident,  ref.axes[side1],  plane2);
    incident = clipPolygon(incident, -ref.axes[side1],  plane3);

    // Keep only points that penetrate the reference face
    float refPlane = glm::dot(ref.center, refNormal) + ref.halfExtents[refAxis];

    for (const glm::vec3& p : incident) {
        float depth = refPlane - glm::dot(p, refNormal);
        if (depth >= 0.0f) {
            Contact c;
            c.point  = p;
            c.normal = bestNormal;          // always from A → B
            c.depth  = depth;
            c.persistentId = 0;             // can improve later with feature IDs
            result.contacts.push_back(c);
        }
    }

    // Fallback: if clipping produced nothing (rare), still report the manifold with a center contact
    if (result.contacts.empty()) {
        Contact c;
        c.point  = 0.5f * (a.center + b.center);
        c.normal = bestNormal;
        c.depth  = minPenetration;
        result.contacts.push_back(c);
    }

    return result;
}

ContactManifold intersectSphereSphere(const SphereCollider& sphereA, const glm::mat4& transformA,
                                      const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    const glm::vec3 centerA = glm::vec3(transformA[3]);
    const glm::vec3 centerB = glm::vec3(transformB[3]);
    const glm::vec3 scaleA = getScale(transformA);
    const glm::vec3 scaleB = getScale(transformB);
    const float radiusA = sphereA.getRadius() * std::max({scaleA.x, scaleA.y, scaleA.z});
    const float radiusB = sphereB.getRadius() * std::max({scaleB.x, scaleB.y, scaleB.z});
    const float distance = glm::distance(centerA, centerB);
    const float radiusSum = radiusA + radiusB;
    if (distance <= radiusSum) {
        result.isColliding = true;
        Contact contact;
        contact.depth = radiusSum - distance;
        contact.normal = (distance > 0.001f) ? glm::normalize(centerB - centerA) : glm::vec3(0, 1, 0);
        contact.point = centerA + contact.normal * radiusA;
        result.contacts.push_back(contact);
        result.normal = contact.normal;
    }
    return result;
}

ContactManifold intersectBoxSphere(const BoxCollider& boxA, const glm::mat4& transformA,
                                   const SphereCollider& sphereB, const glm::mat4& transformB) {
    ContactManifold result;
    const glm::mat4 invTransformA = glm::inverse(transformA);
    const glm::vec3 localSphereCenter = glm::vec3(invTransformA * glm::vec4(glm::vec3(transformB[3]), 1.0f));
    const glm::vec3 scaleB = getScale(transformB);
    const float sphereRadius = sphereB.getRadius() * std::max({scaleB.x, scaleB.y, scaleB.z});
    const glm::vec3 halfExtents = boxA.getHalfExtents();
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
                contact.persistentId = makeContactId(bodyA, bodyB, contact.point);
            }
            manifolds.push_back(std::move(manifold));
        }
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

        // Positional correction (Baumgarte) - now delta-time aware
        const float angularTermA = glm::dot(glm::cross(rA, contact.normal), bodyA->getInverseInertiaTensor() * glm::cross(rA, contact.normal));
        const float angularTermB = glm::dot(glm::cross(rB, contact.normal), bodyB->getInverseInertiaTensor() * glm::cross(rB, contact.normal));

        float massNormal = 1.0f / (totalInvMass + angularTermA + angularTermB);

        // Sequential Impulse: calculate delta impulse and accumulate
        const float velocityAlongNormal = glm::dot(relativeVelocity, contact.normal);

        // Only bounce if closing speed is significant (lowered threshold for realism)
        constexpr float kRestitutionThreshold = 0.2f; // m/s
        float e = std::min(propsA.restitution, propsB.restitution);
        if (std::abs(velocityAlongNormal) < kRestitutionThreshold) {
            e = 0.0f;
        }

        const float biasFactor = 0.2f;
        const float slop = 0.005f;
        float bias = (biasFactor / deltaTime) * std::max(0.0f, contact.depth - slop);

        float j = (-(1.0f + e) * velocityAlongNormal + bias) * massNormal;

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

            // Use static vs dynamic friction: prefer static if tangential impulse stays within limit
            const float staticF = std::min(propsA.staticFriction, propsB.staticFriction);
            const float dynamicF = std::min(propsA.friction, propsB.friction);
            float maxStatic = contact.normalImpulse * staticF;
            float maxDynamic = contact.normalImpulse * dynamicF;

            float oldTangentImpulse = contact.tangentImpulse;
            float newTangent = oldTangentImpulse + jt;

            if (std::abs(newTangent) <= maxStatic) {
                // static friction holds
                contact.tangentImpulse = newTangent;
            } else {
                // sliding: limit using dynamic friction
                contact.tangentImpulse = glm::clamp(newTangent, -maxDynamic, maxDynamic);
            }

            jt = contact.tangentImpulse - oldTangentImpulse;

            glm::vec3 frictionImpulseVec = tangent * jt;

            bodyA->setVelocity(bodyA->getVelocity() - frictionImpulseVec * invMassA);
            bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * glm::cross(rA, frictionImpulseVec));

            bodyB->setVelocity(bodyB->getVelocity() + frictionImpulseVec * invMassB);
            bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * glm::cross(rB, frictionImpulseVec));
        }

        // Torsional spin friction: reduce relative angular velocity about the contact normal.
        // This helps stop cubes from spinning in place when they are supported by a contact patch.
        const float rollingFriction = std::min(propsA.rollingFriction, propsB.rollingFriction);
        if (rollingFriction > 0.0f) {
            const float wA = glm::dot(bodyA->getAngularVelocity(), contact.normal);
            const float wB = glm::dot(bodyB->getAngularVelocity(), contact.normal);
            const float relSpin = wB - wA;

            const float invInertiaA = glm::dot(contact.normal, bodyA->getInverseInertiaTensor() * contact.normal);
            const float invInertiaB = glm::dot(contact.normal, bodyB->getInverseInertiaTensor() * contact.normal);
            const float spinMass = invInertiaA + invInertiaB;

            if (spinMass > 0.0f) {
                float js = -relSpin / spinMass;
                const float maxSpinImpulse = contact.normalImpulse * rollingFriction;
                js = glm::clamp(js, -maxSpinImpulse, maxSpinImpulse);

                const glm::vec3 spinImpulse = contact.normal * js;
                bodyA->setAngularVelocity(bodyA->getAngularVelocity() - bodyA->getInverseInertiaTensor() * spinImpulse);
                bodyB->setAngularVelocity(bodyB->getAngularVelocity() + bodyB->getInverseInertiaTensor() * spinImpulse);
            }
        }
    }
}

void CollisionDetection::correctPositions(std::vector<ContactManifold>& manifolds) {
    constexpr float kPercent = 0.95f;
    constexpr float kSlop    = 0.001f;

    for (auto& manifold : manifolds) {
        for (auto& contact : manifold.contacts) {
            RigidBody* bodyA = contact.bodyA;
            RigidBody* bodyB = contact.bodyB;

            if (bodyA->getProps().isTrigger || bodyB->getProps().isTrigger)
                continue;

            float invMassA = (bodyA->getProps().isKinematic || bodyA->getProps().mass <= 0.0f)
                                 ? 0.0f : 1.0f / bodyA->getProps().mass;
            float invMassB = (bodyB->getProps().isKinematic || bodyB->getProps().mass <= 0.0f)
                                 ? 0.0f : 1.0f / bodyB->getProps().mass;
            float totalInv = invMassA + invMassB;
            if (totalInv <= 0.0f)
                continue;

            float correctionMag = std::max(contact.depth - kSlop, 0.0f) / totalInv * kPercent;
            glm::vec3 correction = contact.normal * correctionMag;

            if (invMassA > 0.0f)
                bodyA->movePosition(-correction * invMassA);
            if (invMassB > 0.0f)
                bodyB->movePosition( correction * invMassB);
        }
    }
}

} // namespace ge
