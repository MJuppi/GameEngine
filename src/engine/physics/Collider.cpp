#include "engine/physics/Collider.h"

namespace ge {

void Collider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    glm::vec3 localMin, localMax;
    getLocalBounds(localMin, localMax);

    // Transform all 8 corners of the bounding box
    min = max = glm::vec3(transform * glm::vec4(localMin, 1.0f));

    glm::vec4 corners[] = {
        {localMin.x, localMin.y, localMin.z, 1.0f},
        {localMax.x, localMax.y, localMax.z, 1.0f},
        {localMin.x, localMax.y, localMin.z, 1.0f},
        {localMax.x, localMin.y, localMin.z, 1.0f},
        {localMin.x, localMin.y, localMax.z, 1.0f},
        {localMax.x, localMin.y, localMax.z, 1.0f},
        {localMin.x, localMax.y, localMax.z, 1.0f},
        {localMax.x, localMax.y, localMax.z, 1.0f}
    };

    for (const auto& corner : corners) {
        glm::vec3 transformed = glm::vec3(transform * corner);
        min = glm::min(min, transformed);
        max = glm::max(max, transformed);
    }
}

} // namespace ge
