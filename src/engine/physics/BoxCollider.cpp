#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <limits>

namespace ge {

BoxCollider::BoxCollider(const glm::vec3& halfExtents)
    : Collider(ColliderType::Box), halfExtents_(halfExtents) {
}

void BoxCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = -halfExtents_;
    max = halfExtents_;
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
