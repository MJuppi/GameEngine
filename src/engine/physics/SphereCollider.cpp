#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/SphereCollider.h"
#include "engine/physics/BoxCollider.h"
#include <algorithm>

namespace ge {

SphereCollider::SphereCollider(float radius)
    : Collider(ColliderType::Sphere), radius_(radius) {
}

void SphereCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = {-radius_, -radius_, -radius_};
    max = {radius_, radius_, radius_};
}

void SphereCollider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    glm::vec3 center = glm::vec3(transform[3]);
    min = center - glm::vec3(radius_);
    max = center + glm::vec3(radius_);
}

} // namespace ge
