#pragma once

#include <glm/glm.hpp>

namespace ge {

struct PointLight {
    alignas(16) glm::vec4 position{0.0f, 1.5f, 2.0f, 1.0f};
    alignas(16) glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
    alignas(16) glm::vec4 parameters{1.0f, 0.09f, 0.032f, 1.0f}; // x: constant, y: linear, z: quadratic, w: intensity
};

struct DirectionalLight {
    alignas(16) glm::vec4 direction{0.35f, 0.55f, 0.75f, 0.0f};
    alignas(16) glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
    alignas(16) float intensity = 1.0f;
};

struct AmbientLight {
    alignas(16) glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
    alignas(16) float intensity = 0.15f;
};

struct SceneLights {
    DirectionalLight directional;
    AmbientLight ambient;
    PointLight pointLights[8];
    uint32_t pointLightCount = 0;
};

} // namespace ge
