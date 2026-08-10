#pragma once

#include "engine/physics/cannon/Vec3.h"
#include "engine/physics/cannon/Quaternion.h"
#include <vector>

namespace ge {
namespace cannon {

class AABB {
public:
    Vec3 lowerBound;
    Vec3 upperBound;

    AABB();
    void setFromPoints(const std::vector<Vec3>& points, const Vec3& position, const Quaternion* quaternion = nullptr, float skinSize = 0.0f);
    bool overlaps(const AABB& other) const;
    void copy(const AABB& other);
};

} // namespace cannon
} // namespace ge
