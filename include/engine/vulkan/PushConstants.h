#pragma once

#include <glm/glm.hpp>
#include <cstdint>

namespace ge {

struct ScenePushConstants {
    glm::mat4 model;
    glm::mat4 normal;
};

static_assert(sizeof(ScenePushConstants) == 128, "ScenePushConstants must be 128 bytes to match pipeline push constant range");

struct UIPushConstants {
    glm::vec2 uiPosition;
    glm::vec2 uiSize;
    glm::vec4 uiColor;
    glm::vec4 uiUVRect;
    int32_t hasTexture;
    int32_t _pad[3];
};

static_assert(sizeof(UIPushConstants) <= 128, "UIPushConstants must fit within the pipeline push constant range");

} // namespace ge
