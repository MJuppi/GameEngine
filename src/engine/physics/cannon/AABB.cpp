#include "engine/physics/cannon/AABB.h"
#include <algorithm>

namespace ge {
namespace cannon {

AABB::AABB() : lowerBound(), upperBound() {}

void AABB::setFromPoints(const std::vector<Vec3>& points, const Vec3& position, const Quaternion* quaternion, float skinSize) {
    if (points.empty()) {
        lowerBound.setZero();
        upperBound.setZero();
        return;
    }

    Vec3 first = points[0];

    if (quaternion) {
        first = quaternion->vmult(first);
    }

    lowerBound = first;
    upperBound = first;

    for (size_t i = 1; i < points.size(); ++i) {
        Vec3 p = points[i];
        if (quaternion) {
            p = quaternion->vmult(p);
        }
        lowerBound.x = std::min(lowerBound.x, p.x);
        lowerBound.y = std::min(lowerBound.y, p.y);
        lowerBound.z = std::min(lowerBound.z, p.z);
        upperBound.x = std::max(upperBound.x, p.x);
        upperBound.y = std::max(upperBound.y, p.y);
        upperBound.z = std::max(upperBound.z, p.z);
    }

    lowerBound.vadd(position);
    upperBound.vadd(position);

    if (skinSize != 0.0f) {
        lowerBound.x -= skinSize;
        lowerBound.y -= skinSize;
        lowerBound.z -= skinSize;
        upperBound.x += skinSize;
        upperBound.y += skinSize;
        upperBound.z += skinSize;
    }
}

bool AABB::overlaps(const AABB& other) const {
    return lowerBound.x <= other.upperBound.x && upperBound.x >= other.lowerBound.x &&
           lowerBound.y <= other.upperBound.y && upperBound.y >= other.lowerBound.y &&
           lowerBound.z <= other.upperBound.z && upperBound.z >= other.lowerBound.z;
}

void AABB::copy(const AABB& other) {
    lowerBound.copy(other.lowerBound);
    upperBound.copy(other.upperBound);
}

} // namespace cannon
} // namespace ge
