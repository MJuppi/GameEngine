#include "engine/physics/cannon/Mat3.h"

namespace ge {
namespace cannon {

Mat3::Mat3() {
    identity();
}

void Mat3::identity() {
    elements = {1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
}

void Mat3::setZero() {
    elements.fill(0.0f);
}

Vec3 Mat3::vmult(const Vec3& v) const {
    return Vec3(
        elements[0] * v.x + elements[1] * v.y + elements[2] * v.z,
        elements[3] * v.x + elements[4] * v.y + elements[5] * v.z,
        elements[6] * v.x + elements[7] * v.y + elements[8] * v.z
    );
}

Mat3 Mat3::mmult(const Mat3& other) const {
    Mat3 result;
    const auto& A = elements;
    const auto& B = other.elements;
    auto& T = result.elements;

    T[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
    T[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
    T[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];
    T[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
    T[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
    T[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];
    T[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
    T[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
    T[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];

    return result;
}

Mat3& Mat3::scale(const Vec3& v) {
    Mat3 result;
    result.elements[0] = elements[0] * v.x;
    result.elements[1] = elements[1] * v.y;
    result.elements[2] = elements[2] * v.z;
    result.elements[3] = elements[3] * v.x;
    result.elements[4] = elements[4] * v.y;
    result.elements[5] = elements[5] * v.z;
    result.elements[6] = elements[6] * v.x;
    result.elements[7] = elements[7] * v.y;
    result.elements[8] = elements[8] * v.z;
    *this = result;
    return *this;
}

} // namespace cannon
} // namespace ge
