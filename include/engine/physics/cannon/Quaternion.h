#pragma once

#include "engine/physics/cannon/Vec3.h"

namespace ge {
namespace cannon {

class Quaternion {
public:
    float x;
    float y;
    float z;
    float w;

    Quaternion();
    Quaternion(float x, float y, float z, float w);

    Quaternion& set(float nx, float ny, float nz, float nw);
    Quaternion& setFromAxisAngle(const Vec3& axis, float angle);
    Quaternion& normalize();
    Quaternion& conjugate();
    Vec3 vmult(const Vec3& v) const;
    Quaternion mult(const Quaternion& other) const;
    Quaternion& copy(const Quaternion& other);
    float dot(const Quaternion& other) const;
    Quaternion slerp(const Quaternion& other, float t) const;
};

} // namespace cannon
} // namespace ge
