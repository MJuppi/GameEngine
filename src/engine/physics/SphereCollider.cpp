#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/SphereCollider.h"
#include "engine/physics/BoxCollider.h"
#include <algorithm>

namespace ge {

SphereCollider::SphereCollider(float radius)
    : radius_(radius) {
}

std::string SphereCollider::getType() const {
    return "Sphere";
}

void SphereCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = {-radius_, -radius_, -radius_};
    max = {radius_, radius_, radius_};
}

ContactManifold SphereCollider::checkCollision(
    const Collider& other,
    const glm::mat4& transformA,
    const glm::mat4& transformB
) const {
    ContactManifold result;

    if (other.getType() == std::string("Sphere")) {
        const SphereCollider& sphereB = static_cast<const SphereCollider&>(other);

        glm::vec3 centerA = glm::vec3(transformA[3]);
        glm::vec3 centerB = glm::vec3(transformB[3]);

        const float distance = glm::distance(centerA, centerB);
        const float radiusSum = radius_ + sphereB.getRadius();

        if (distance <= radiusSum) {
            result.isColliding = true;

            Contact contact;
            contact.depth = radiusSum - distance;

            if (distance > 0.001f) {
                contact.normal = glm::normalize(centerB - centerA);
            } else {
                // Spheres are overlapping completely, use arbitrary normal
                contact.normal = { 0, 1, 0 };
            }

            contact.point = centerA + contact.normal * radius_;
            result.contacts.push_back(contact);
            result.normal = contact.normal;
        }

    } else if (other.getType() == std::string("Box")) {
        // Sphere-Box collision
        const BoxCollider& boxB = static_cast<const BoxCollider&>(other);

        // Transform sphere center into Box's local space
        const glm::mat4 invTransformB = glm::inverse(transformB);
        const glm::vec3 localSphereCenter = glm::vec3(invTransformB * glm::vec4(glm::vec3(transformA[3]), 1.0f));
        const glm::vec3 halfExtents = boxB.getHalfExtents();

        // Find closest point on AABB to local sphere center
        glm::vec3 localClosestPoint = localSphereCenter;
        localClosestPoint.x = std::clamp(localClosestPoint.x, -halfExtents.x, halfExtents.x);
        localClosestPoint.y = std::clamp(localClosestPoint.y, -halfExtents.y, halfExtents.y);
        localClosestPoint.z = std::clamp(localClosestPoint.z, -halfExtents.z, halfExtents.z);

        const float distanceSq = glm::distance2(localSphereCenter, localClosestPoint);

        if (distanceSq <= radius_ * radius_) {
            result.isColliding = true;

            const float distance = glm::sqrt(distanceSq);
            Contact contact;
            contact.depth = radius_ - distance;

            if (distance > 0.001f) {
                glm::vec3 localNormal = glm::normalize(localClosestPoint - localSphereCenter);
                contact.normal = glm::normalize(glm::vec3(transformB * glm::vec4(localNormal, 0.0f)));
            } else {
                contact.normal = { 0, 1, 0 };
            }

            contact.point = glm::vec3(transformB * glm::vec4(localClosestPoint, 1.0f));
            result.contacts.push_back(contact);
            result.normal = contact.normal;
        }
    }

    return result;
}

void SphereCollider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    glm::vec3 center = glm::vec3(transform[3]);
    min = center - glm::vec3(radius_);
    max = center + glm::vec3(radius_);
}

} // namespace ge
