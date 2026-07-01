#pragma once

#include <glm/glm.hpp>

namespace ge {

struct PointLight {
    alignas(16) glm::vec4 position{0.0f, 1.5f, 2.0f, 1.0f};
    alignas(16) glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
    alignas(16) glm::vec4 parameters{1.0f, 0.09f, 0.032f, 8.0f};
};

} // namespace ge
