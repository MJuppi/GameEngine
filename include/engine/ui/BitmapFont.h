#pragma once

#include "engine/asset/TextureData.h"
#include <glm/glm.hpp>

namespace ge {

class BitmapFont {
public:
    // Generates a simple 8x8 ASCII bitmap (32-126) packed into a 128x128 texture
    static TextureData generateDefaultFontTexture();
    // Returns UV rect (umin, vmin, umax, vmax) for given ASCII character
    static glm::vec4 getCharUV(char c);
};

} // namespace ge
