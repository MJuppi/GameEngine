#include "engine/physics/SphereCollider.h"
#include <algorithm>

namespace ge {
namespace {

glm::vec3 extractScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

} // namespace

SphereCollider::SphereCollider(float radius)
    : Collider(ColliderType::Sphere), radius_(radius) {
}

void SphereCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = {-radius_, -radius_, -radius_};
    max = {radius_, radius_, radius_};
}

void SphereCollider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    const glm::vec3 center = glm::vec3(transform[3]);
    const glm::vec3 scale = extractScale(transform);
    const glm::vec3 worldRadius = radius_ * scale;
    min = center - worldRadius;
    max = center + worldRadius;
}

} // namespace ge
