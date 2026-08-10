#include "engine/physics/cannon/Vec3.h"

namespace ge {
namespace cannon {

Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
Vec3::Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

Vec3& Vec3::set(float newX, float newY, float newZ) {
    x = newX;
    y = newY;
    z = newZ;
    return *this;
}

Vec3& Vec3::setZero() {
    x = y = z = 0.0f;
    return *this;
}

Vec3& Vec3::copy(const Vec3& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

float Vec3::dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vec3 Vec3::cross(const Vec3& other) const {
    return Vec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

Vec3& Vec3::vadd(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3& Vec3::vsub(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vec3& Vec3::scale(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vec3& Vec3::normalize() {
    const float len = length();
    if (len > 0.0f) {
        const float invLen = 1.0f / len;
        x *= invLen;
        y *= invLen;
        z *= invLen;
    }
    return *this;
}

float Vec3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

float Vec3::lengthSquared() const {
    return dot(*this);
}

Vec3& Vec3::addScaledVector(float scalar, const Vec3& other) {
    x += scalar * other.x;
    y += scalar * other.y;
    z += scalar * other.z;
    return *this;
}

Vec3& Vec3::add(const Vec3& other) {
    return vadd(other);
}

Vec3& Vec3::sub(const Vec3& other) {
    return vsub(other);
}

} // namespace cannon
} // namespace ge
