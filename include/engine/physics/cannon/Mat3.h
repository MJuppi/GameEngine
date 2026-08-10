#pragma once

#include "engine/physics/cannon/Vec3.h"
#include <array>

namespace ge {
namespace cannon {

class Mat3 {
public:
    std::array<float, 9> elements;

    Mat3();
    void identity();
    void setZero();
    Vec3 vmult(const Vec3& v) const;
    Mat3 mmult(const Mat3& other) const;
    Mat3& scale(const Vec3& v);
};

} // namespace cannon
} // namespace ge
