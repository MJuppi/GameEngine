#include "engine/physics/PhysicsWorld.h"

#include "engine/physics/BoxCollider.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ge {
namespace {

glm::vec3 extractWorldScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

bool raycastBox(const BoxCollider& box,
                const glm::mat4& transform,
                const glm::vec3& origin,
                const glm::vec3& direction,
                float maxDistance,
                float& outDistance,
                glm::vec3& outNormal) {
    const glm::mat4 invTransform = glm::inverse(transform);
    const glm::vec3 localOrigin = glm::vec3(invTransform * glm::vec4(origin, 1.0f));
    const glm::vec3 localDirection = glm::vec3(invTransform * glm::vec4(direction, 0.0f));
    const float directionLength = glm::length(localDirection);
    if (directionLength < 1e-6f) {
        return false;
    }

    const glm::vec3 localDir = localDirection / directionLength;
    const glm::vec3 halfExtents = box.getHalfExtents();

    float tMin = 0.0f;
    float tMax = maxDistance * directionLength;
    int hitAxis = -1;
    float hitSign = 1.0f;

    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(localDir[axis]) < 1e-6f) {
            if (localOrigin[axis] < -halfExtents[axis] || localOrigin[axis] > halfExtents[axis]) {
                return false;
            }
            continue;
        }

        const float invD = 1.0f / localDir[axis];
        float t0 = (-halfExtents[axis] - localOrigin[axis]) * invD;
        float t1 = (halfExtents[axis] - localOrigin[axis]) * invD;
        float sign = 1.0f;
        if (invD < 0.0f) {
            std::swap(t0, t1);
            sign = -1.0f;
        }

        if (t0 > tMin) {
            tMin = t0;
            hitAxis = axis;
            hitSign = sign;
        }
        tMax = std::min(tMax, t1);
        if (tMax <= tMin) {
            return false;
        }
    }

    if (tMax <= tMin || tMin <= 0.0f || tMin > maxDistance * directionLength) {
        return false;
    }

    outDistance = tMin / directionLength;
    glm::vec3 localNormal(0.0f);
    if (hitAxis >= 0) {
        localNormal[hitAxis] = hitSign;
    } else {
        localNormal = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    outNormal = glm::normalize(glm::vec3(transform * glm::vec4(localNormal, 0.0f)));
    return true;
}

bool raycastSphere(const SphereCollider& sphere,
                   const glm::mat4& transform,
                   const glm::vec3& origin,
                   const glm::vec3& direction,
                   float maxDistance,
                   float& outDistance,
                   glm::vec3& outNormal) {
    const glm::vec3 center = glm::vec3(transform[3]);
    const glm::vec3 scale = extractWorldScale(transform);
    const float radius = sphere.getRadius() * std::max({scale.x, scale.y, scale.z});

    const glm::vec3 oc = origin - center;
    const float a = glm::dot(direction, direction);
    const float b = 2.0f * glm::dot(oc, direction);
    const float c = glm::dot(oc, oc) - radius * radius;
    const float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }

    const float sqrtDisc = std::sqrt(discriminant);
    float t = (-b - sqrtDisc) / (2.0f * a);
    if (t <= 0.0f) {
        t = (-b + sqrtDisc) / (2.0f * a);
    }
    if (t <= 0.0f || t > maxDistance) {
        return false;
    }

    outDistance = t;
    const glm::vec3 hitPoint = origin + direction * t;
    outNormal = glm::normalize(hitPoint - center);
    return true;
}

bool canCollidePair(const RigidBody& bodyA, const RigidBody& bodyB) {
    const bool bodyADynamic = !bodyA.getProps().isKinematic && bodyA.getProps().mass > 0.0f;
    const bool bodyBDynamic = !bodyB.getProps().isKinematic && bodyB.getProps().mass > 0.0f;
    return (bodyADynamic || bodyBDynamic) && !bodyA.getProps().isTrigger && !bodyB.getProps().isTrigger;
}

float getInverseMass(const RigidBody& body) {
    if (body.getProps().isKinematic || body.getProps().mass <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / body.getProps().mass;
}

ContactManifold makeBoxBoxManifold(RigidBody& bodyA, RigidBody& bodyB) {
    ContactManifold manifold;
    manifold.bodyA = &bodyA;
    manifold.bodyB = &bodyB;

    const auto& boxA = static_cast<const BoxCollider&>(bodyA.getCollider());
    const auto& boxB = static_cast<const BoxCollider&>(bodyB.getCollider());
    const glm::vec3 centerA = bodyA.getPosition();
    const glm::vec3 centerB = bodyB.getPosition();
    const glm::vec3 halfA = boxA.getHalfExtents();
    const glm::vec3 halfB = boxB.getHalfExtents();

    const glm::vec3 delta = centerB - centerA;
    const glm::vec3 overlap = glm::vec3(
        halfA.x + halfB.x - std::abs(delta.x),
        halfA.y + halfB.y - std::abs(delta.y),
        halfA.z + halfB.z - std::abs(delta.z));

    if (overlap.x < 0.0f || overlap.y < 0.0f || overlap.z < 0.0f) {
        return manifold;
    }

    float penetration = std::numeric_limits<float>::max();
    glm::vec3 normal(0.0f);

    if (overlap.x < penetration) {
        penetration = overlap.x;
        normal = delta.x >= 0.0f ? glm::vec3(-1.0f, 0.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    }
    if (overlap.y < penetration) {
        penetration = overlap.y;
        normal = delta.y >= 0.0f ? glm::vec3(0.0f, -1.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    if (overlap.z < penetration) {
        penetration = overlap.z;
        normal = delta.z >= 0.0f ? glm::vec3(0.0f, 0.0f, -1.0f) : glm::vec3(0.0f, 0.0f, 1.0f);
    }

    Contact contact;
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;
    contact.normal = normal;
    contact.depth = penetration;
    contact.point = centerA + (delta * 0.5f);
    manifold.contacts.push_back(contact);
    manifold.isColliding = true;
    manifold.normal = normal;
    return manifold;
}

ContactManifold makeSphereBoxManifold(RigidBody& bodyA, RigidBody& bodyB) {
    ContactManifold manifold;
    manifold.bodyA = &bodyA;
    manifold.bodyB = &bodyB;

    const glm::vec3 centerA = bodyA.getPosition();
    const glm::vec3 centerB = bodyB.getPosition();
    const glm::vec3 delta = centerB - centerA;
    const auto& sphere = static_cast<const SphereCollider&>(bodyA.getCollider());
    const auto& box = static_cast<const BoxCollider&>(bodyB.getCollider());
    const float radius = sphere.getRadius();
    const glm::vec3 half = box.getHalfExtents();

    const glm::vec3 clamped = glm::clamp(delta, -half, half);
    const glm::vec3 closestPoint = centerA + clamped;
    const glm::vec3 diff = centerB - closestPoint;
    const float distSq = glm::dot(diff, diff);
    if (distSq > radius * radius) {
        return manifold;
    }

    glm::vec3 normal = glm::dot(diff, diff) > 1e-6f ? glm::normalize(diff) : glm::vec3(0.0f, 1.0f, 0.0f);
    const float penetration = radius - std::sqrt(std::max(0.0f, distSq));

    Contact contact;
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;
    contact.normal = normal;
    contact.depth = penetration;
    contact.point = closestPoint;
    manifold.contacts.push_back(contact);
    manifold.isColliding = true;
    manifold.normal = normal;
    return manifold;
}

ContactManifold makeSphereSphereManifold(RigidBody& bodyA, RigidBody& bodyB) {
    ContactManifold manifold;
    manifold.bodyA = &bodyA;
    manifold.bodyB = &bodyB;

    const auto& sphereA = static_cast<const SphereCollider&>(bodyA.getCollider());
    const auto& sphereB = static_cast<const SphereCollider&>(bodyB.getCollider());
    const glm::vec3 delta = bodyB.getPosition() - bodyA.getPosition();
    const float distance = glm::length(delta);
    const float minDistance = sphereA.getRadius() + sphereB.getRadius();
    if (distance >= minDistance) {
        return manifold;
    }

    glm::vec3 normal = distance > 1e-6f ? delta / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    const float penetration = minDistance - distance;

    Contact contact;
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;
    contact.normal = normal;
    contact.depth = penetration;
    contact.point = bodyA.getPosition() + normal * sphereA.getRadius();
    manifold.contacts.push_back(contact);
    manifold.isColliding = true;
    manifold.normal = normal;
    return manifold;
}

ContactManifold buildManifold(RigidBody& bodyA, RigidBody& bodyB) {
    const auto typeA = bodyA.getCollider().getType();
    const auto typeB = bodyB.getCollider().getType();

    if (bodyA.getProps().isTrigger || bodyB.getProps().isTrigger) {
        return {};
    }

    if (typeA == ColliderType::Box && typeB == ColliderType::Box) {
        return makeBoxBoxManifold(bodyA, bodyB);
    }
    if (typeA == ColliderType::Sphere && typeB == ColliderType::Box) {
        return makeSphereBoxManifold(bodyA, bodyB);
    }
    if (typeA == ColliderType::Box && typeB == ColliderType::Sphere) {
        auto manifold = makeSphereBoxManifold(bodyB, bodyA);
        if (manifold.isColliding) {
            manifold.normal = -manifold.normal;
        }
        return manifold;
    }
    if (typeA == ColliderType::Sphere && typeB == ColliderType::Sphere) {
        return makeSphereSphereManifold(bodyA, bodyB);
    }
    return {};
}
} // namespace

PhysicsWorld::PhysicsWorld() = default;

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
    const auto it = std::find_if(bodies_.begin(), bodies_.end(),
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
    RaycastResult result;
    result.fraction = 1.0f;

    const float directionLength = glm::length(direction);
    if (directionLength < 1e-6f || maxDistance <= 0.0f) {
        return result;
    }

    const glm::vec3 dir = direction / directionLength;
    float closestDistance = maxDistance;

    for (const auto& body : bodies_) {
        const auto& collider = body->getCollider();
        const glm::mat4& transform = body->getWorldTransform();
        float hitDistance = 0.0f;
        glm::vec3 hitNormal(0.0f);
        bool hit = false;

        if (collider.getType() == ColliderType::Box) {
            hit = raycastBox(static_cast<const BoxCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        } else if (collider.getType() == ColliderType::Sphere) {
            hit = raycastSphere(static_cast<const SphereCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        }

        if (hit && hitDistance > 0.0f && hitDistance < closestDistance) {
            closestDistance = hitDistance;
            result.hit = true;
            result.body = body.get();
            result.fraction = hitDistance / maxDistance;
            result.point = origin + dir * hitDistance;
            result.normal = hitNormal;
        }
    }

    return result;
}

void PhysicsWorld::warmStart(std::vector<ContactManifold>& manifolds) {
    (void)manifolds;
}

void PhysicsWorld::resolveContacts(std::vector<ContactManifold>& manifolds, float deltaTime) {
    (void)deltaTime;
    for (auto& manifold : manifolds) {
        if (!manifold.isColliding || manifold.contacts.empty()) {
            continue;
        }

        for (auto& contact : manifold.contacts) {
            auto* bodyA = contact.bodyA;
            auto* bodyB = contact.bodyB;
            if (!bodyA || !bodyB) {
                continue;
            }

            const glm::vec3 normal = glm::normalize(contact.normal);
            if (glm::dot(normal, normal) <= 1e-6f) {
                continue;
            }

            const float invMassA = getInverseMass(*bodyA);
            const float invMassB = getInverseMass(*bodyB);
            const float invMassSum = invMassA + invMassB;
            if (invMassSum <= 1e-6f) {
                continue;
            }

            const glm::vec3 relativeVelocity = bodyA->getVelocity() - bodyB->getVelocity();
            const float velocityAlongNormal = glm::dot(relativeVelocity, normal);
            if (velocityAlongNormal > 0.0f) {
                continue;
            }

            const float restitution = std::min(bodyA->getProps().restitution, bodyB->getProps().restitution);
            const float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / invMassSum;
            const glm::vec3 impulse = normal * impulseMagnitude;

            glm::vec3 velocityA = bodyA->getVelocity();
            glm::vec3 velocityB = bodyB->getVelocity();
            if (invMassA > 0.0f) {
                velocityA += impulse * invMassA;
            }
            if (invMassB > 0.0f) {
                velocityB -= impulse * invMassB;
            }
            bodyA->setVelocity(velocityA);
            bodyB->setVelocity(velocityB);

            const float penetration = std::max(0.0f, contact.depth + 0.01f);
            const glm::vec3 correction = normal * (penetration / std::max(invMassSum, 1e-6f)) * 0.75f;
            if (invMassA > 0.0f) {
                bodyA->movePosition(correction);
            }
            if (invMassB > 0.0f) {
                bodyB->movePosition(-correction);
            }
        }
    }
}

void PhysicsWorld::sweepCCD(float deltaTime) {
    (void)deltaTime;
}

void PhysicsWorld::stepInternal(float deltaTime) {
    applyGravity();

    for (auto& body : bodies_) {
        body->integrateVelocity(deltaTime);
    }

    std::vector<ContactManifold> manifolds;
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
        for (auto jt = std::next(it); jt != bodies_.end(); ++jt) {
            if (!canCollidePair(**it, **jt)) {
                continue;
            }

            auto manifold = buildManifold(**it, **jt);
            if (manifold.isColliding) {
                manifolds.push_back(std::move(manifold));
            }
        }
    }

    resolveContacts(manifolds, deltaTime);

    for (auto& body : bodies_) {
        body->integratePosition(deltaTime);
    }

    manifoldCache_ = std::move(manifolds);
}

void PhysicsWorld::step(float deltaTime, int maxSubSteps) {
    if (deltaTime <= 0.0f) {
        return;
    }

    const int subSteps = std::max(1, maxSubSteps);
    const float subDeltaTime = deltaTime / static_cast<float>(subSteps);
    for (int i = 0; i < subSteps; ++i) {
        stepInternal(subDeltaTime);
    }
}

} // namespace ge
