#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace ge {

/// Glyph metrics for a single character
struct GlyphMetrics {
    uint32_t codepoint = 0;
    float advance = 0.0f;
    float bearingX = 0.0f;
    float bearingY = 0.0f;
    int width = 0;
    int height = 0;
    float uvLeft = 0.0f;
    float uvTop = 0.0f;
    float uvRight = 0.0f;
    float uvBottom = 0.0f;
};

/// Font metrics (ascender, descender, line height, etc.)
struct FontMetrics {
    float ascender = 0.0f;
    float descender = 0.0f;
    float lineHeight = 0.0f;
    float lineGap = 0.0f;
    float scale = 0.0f;
};

/// Font data loaded from a TTF file
struct FontData {
    std::string path;
    FontMetrics metrics;
    std::vector<GlyphMetrics> glyphs;
    std::vector<uint8_t> atlasPixels;
    int atlasWidth = 0;
    int atlasHeight = 0;
};

/// Font rendering vertex for text rendering
struct FontVertex {
    glm::vec2 position;
    glm::vec2 texCoord;
    glm::vec4 color;
};

} // namespace ge
