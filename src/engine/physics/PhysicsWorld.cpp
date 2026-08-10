#include "engine/physics/PhysicsWorld.h"

#include "engine/physics/BroadPhase.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/CollisionDetection.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
    if (!manifolds.empty()) {
        CollisionDetection::resolveCollisions(manifolds, solverIterations_, deltaTime);
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

    const auto potentialPairs = BroadPhase::findPairs(bodies_);
    std::vector<ContactManifold> manifolds;
    CollisionDetection::detectCollisions(potentialPairs, manifolds);

    resolveContacts(manifolds, deltaTime);
    CollisionDetection::correctPositions(manifolds);

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
