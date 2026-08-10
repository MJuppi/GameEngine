#include "engine/physics/cannon/Quaternion.h"
#include <cmath>

namespace ge {
namespace cannon {

Quaternion::Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
Quaternion::Quaternion(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

Quaternion& Quaternion::set(float nx, float ny, float nz, float nw) {
    x = nx;
    y = ny;
    z = nz;
    w = nw;
    return *this;
}

Quaternion& Quaternion::setFromAxisAngle(const Vec3& axis, float angle) {
    const float s = std::sin(angle * 0.5f);
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
    w = std::cos(angle * 0.5f);
    return *this;
}

Quaternion& Quaternion::normalize() {
    const float len = std::sqrt(x*x + y*y + z*z + w*w);
    if (len > 0.0f) {
        const float invLen = 1.0f / len;
        x *= invLen;
        y *= invLen;
        z *= invLen;
        w *= invLen;
    }
    return *this;
}

Quaternion& Quaternion::conjugate() {
    x = -x;
    y = -y;
    z = -z;
    return *this;
}

Vec3 Quaternion::vmult(const Vec3& v) const {
    const float ix =  w * v.x + y * v.z - z * v.y;
    const float iy =  w * v.y + z * v.x - x * v.z;
    const float iz =  w * v.z + x * v.y - y * v.x;
    const float iw = -x * v.x - y * v.y - z * v.z;
    return Vec3(
        ix * w + iw * -x + iy * -z - iz * -y,
        iy * w + iw * -y + iz * -x - ix * -z,
        iz * w + iw * -z + ix * -y - iy * -x
    );
}

Quaternion Quaternion::mult(const Quaternion& other) const {
    return Quaternion(
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y + y * other.w + z * other.x - x * other.z,
        w * other.z + z * other.w + x * other.y - y * other.x,
        w * other.w - x * other.x - y * other.y - z * other.z
    );
}

Quaternion& Quaternion::copy(const Quaternion& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    w = other.w;
    return *this;
}

float Quaternion::dot(const Quaternion& other) const {
    return x * other.x + y * other.y + z * other.z + w * other.w;
}

Quaternion Quaternion::slerp(const Quaternion& other, float t) const {
    float cosHalfTheta = dot(other);
    Quaternion target = other;

    if (cosHalfTheta < 0.0f) {
        target.x = -target.x;
        target.y = -target.y;
        target.z = -target.z;
        target.w = -target.w;
        cosHalfTheta = -cosHalfTheta;
    }

    if (std::abs(cosHalfTheta) >= 1.0f) {
        return *this;
    }

    const float halfTheta = std::acos(cosHalfTheta);
    const float sinHalfTheta = std::sqrt(1.0f - cosHalfTheta * cosHalfTheta);

    if (std::fabs(sinHalfTheta) < 1e-6f) {
        return Quaternion(
            0.5f * (x + target.x),
            0.5f * (y + target.y),
            0.5f * (z + target.z),
            0.5f * (w + target.w)
        );
    }

    const float ratioA = std::sin((1.0f - t) * halfTheta) / sinHalfTheta;
    const float ratioB = std::sin(t * halfTheta) / sinHalfTheta;
    return Quaternion(
        x * ratioA + target.x * ratioB,
        y * ratioA + target.y * ratioB,
        z * ratioA + target.z * ratioB,
        w * ratioA + target.w * ratioB
    );
}

} // namespace cannon
} // namespace ge
