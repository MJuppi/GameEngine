#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <limits>

namespace ge {

BoxCollider::BoxCollider(const glm::vec3& halfExtents)
    : halfExtents_(halfExtents) {
}

std::string BoxCollider::getType() const {
    return "Box";
}

void BoxCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = -halfExtents_;
    max = halfExtents_;
}

CollisionResult BoxCollider::checkCollision(
    const Collider& other,
    const glm::mat4& transformA,
    const glm::mat4& transformB
) const {
    CollisionResult result;

    // Check if the other collider is a BoxCollider
    if (other.getType() == std::string("Box")) {
        const BoxCollider& boxB = static_cast<const BoxCollider&>(other);

        // Extract OBB information from both boxes
        glm::vec3 aMin, aMax;
        getWorldBounds(aMin, aMax, transformA);

        glm::vec3 bMin, bMax;
        boxB.getWorldBounds(bMin, bMax, transformB);

        // Simple AABB collision detection for now
        // TODO: Implement proper OBB collision detection
        bool colliding = true;
        colliding &= (aMin.x <= bMax.x && aMax.x >= bMin.x);
        colliding &= (aMin.y <= bMax.y && aMax.y >= bMin.y);
        colliding &= (aMin.z <= bMax.z && aMax.z >= bMin.z);

        if (colliding) {
            result.isColliding = true;

            const float overlapX1 = aMax.x - bMin.x;
            const float overlapX2 = bMax.x - aMin.x;
            const float overlapY1 = aMax.y - bMin.y;
            const float overlapY2 = bMax.y - aMin.y;
            const float overlapZ1 = aMax.z - bMin.z;
            const float overlapZ2 = bMax.z - aMin.z;

            float penetration = overlapX1;
            glm::vec3 normal{1.0f, 0.0f, 0.0f};

            if (overlapX2 < penetration) { penetration = overlapX2; normal = {-1.0f, 0.0f, 0.0f}; }
            if (overlapY1 < penetration) { penetration = overlapY1; normal = {0.0f, 1.0f, 0.0f}; }
            if (overlapY2 < penetration) { penetration = overlapY2; normal = {0.0f, -1.0f, 0.0f}; }
            if (overlapZ1 < penetration) { penetration = overlapZ1; normal = {0.0f, 0.0f, 1.0f}; }
            if (overlapZ2 < penetration) { penetration = overlapZ2; normal = {0.0f, 0.0f, -1.0f}; }

            const glm::vec3 centerA = (aMin + aMax) * 0.5f;
            const glm::vec3 centerB = (bMin + bMax) * 0.5f;
            if (glm::dot(normal, centerB - centerA) < 0.0f) {
                normal = -normal;
            }

            Contact contact;
            contact.depth = penetration;
            contact.normal = normal;
            contact.point = {
                (std::max(aMin.x, bMin.x) + std::min(aMax.x, bMax.x)) * 0.5f,
                (std::max(aMin.y, bMin.y) + std::min(aMax.y, bMax.y)) * 0.5f,
                (std::max(aMin.z, bMin.z) + std::min(aMax.z, bMax.z)) * 0.5f
            };

            result.contacts.push_back(contact);
        }

    } else if (other.getType() == std::string("Sphere")) {
        const SphereCollider& sphereB = static_cast<const SphereCollider&>(other);

        // Proper OBB-Sphere collision
        // Transform sphere center into Box's local space
        const glm::mat4 invTransformA = glm::inverse(transformA);
        const glm::vec3 localSphereCenter = glm::vec3(invTransformA * glm::vec4(glm::vec3(transformB[3]), 1.0f));
        const float sphereRadius = sphereB.getRadius();

        // Find closest point on AABB to local sphere center
        glm::vec3 localClosestPoint = localSphereCenter;
        localClosestPoint.x = std::clamp(localClosestPoint.x, -halfExtents_.x, halfExtents_.x);
        localClosestPoint.y = std::clamp(localClosestPoint.y, -halfExtents_.y, halfExtents_.y);
        localClosestPoint.z = std::clamp(localClosestPoint.z, -halfExtents_.z, halfExtents_.z);

        const float distanceSq = glm::distance2(localSphereCenter, localClosestPoint);

        if (distanceSq <= sphereRadius * sphereRadius) {
            result.isColliding = true;

            const float distance = glm::sqrt(distanceSq);
            Contact contact;
            contact.depth = sphereRadius - distance;

            if (distance > 0.001f) {
                // Transform local normal back to world space
                glm::vec3 localNormal = glm::normalize(localSphereCenter - localClosestPoint);
                contact.normal = glm::normalize(glm::vec3(transformA * glm::vec4(localNormal, 0.0f)));
            } else {
                contact.normal = glm::normalize(glm::vec3(transformA[1])); // Up vector of box as fallback
            }

            contact.point = glm::vec3(transformA * glm::vec4(localClosestPoint, 1.0f));
            result.contacts.push_back(contact);
        }
    }

    return result;
}

void BoxCollider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    // Transform the 8 corners and find the min/max
    glm::vec3 corners[8] = {
        {-halfExtents_.x, -halfExtents_.y, -halfExtents_.z},
        { halfExtents_.x, -halfExtents_.y, -halfExtents_.z},
        {-halfExtents_.x,  halfExtents_.y, -halfExtents_.z},
        { halfExtents_.x,  halfExtents_.y, -halfExtents_.z},
        {-halfExtents_.x, -halfExtents_.y,  halfExtents_.z},
        { halfExtents_.x, -halfExtents_.y,  halfExtents_.z},
        {-halfExtents_.x,  halfExtents_.y,  halfExtents_.z},
        { halfExtents_.x,  halfExtents_.y,  halfExtents_.z}
    };

    min = glm::vec3(std::numeric_limits<float>::max());
    max = glm::vec3(std::numeric_limits<float>::lowest());

    for (const auto& corner : corners) {
        glm::vec3 worldCorner = glm::vec3(transform * glm::vec4(corner, 1.0f));
        min = glm::min(min, worldCorner);
        max = glm::max(max, worldCorner);
    }
}

} // namespace ge
