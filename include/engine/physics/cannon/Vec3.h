#pragma once

#include <cmath>

namespace ge {
namespace cannon {

class Vec3 {
public:
    float x;
    float y;
    float z;

    Vec3();
    Vec3(float x, float y, float z);

    Vec3& set(float newX, float newY, float newZ);
    Vec3& setZero();
    Vec3& copy(const Vec3& other);
    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;
    Vec3& vadd(const Vec3& other);
    Vec3& vsub(const Vec3& other);
    Vec3& scale(float scalar);
    Vec3& normalize();
    float length() const;
    float lengthSquared() const;
    Vec3& addScaledVector(float scalar, const Vec3& other);
    Vec3& add(const Vec3& other);
    Vec3& sub(const Vec3& other);
};

} // namespace cannon
} // namespace ge
