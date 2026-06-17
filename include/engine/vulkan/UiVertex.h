#pragma once

#include <glm/glm.hpp>

// UiVertex is used for both UI rendering and font rendering
struct UiVertex {
    glm::vec2 position;
    glm::vec2 texCoord;
    glm::vec4 color;
};