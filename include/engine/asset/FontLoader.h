#pragma once

#include "engine/asset/FontData.h"

#include <string>
#include <vector>

namespace ge {

class FontLoader {
public:
    FontLoader() = delete;
    ~FontLoader() = delete;

    FontLoader(const FontLoader&) = delete;
    FontLoader& operator=(const FontLoader&) = delete;

    /// Load a TTF font file and bake glyphs into a texture atlas
    /// @param path Path to the TTF font file
    /// @param fontHeight Height of the font in pixels
    /// @param glyphRange Range of Unicode codepoints to bake (e.g., 32-126 for ASCII)
    /// @return FontData containing glyph metrics and atlas texture
    [[nodiscard]] static FontData loadFont(
        const std::string& path,
        float fontHeight = 32.0f,
        uint32_t glyphRangeStart = 32,
        uint32_t glyphRangeEnd = 126);

    /// Load a TTF font file with specific glyphs
    /// @param path Path to the TTF font file
    /// @param fontHeight Height of the font in pixels
    /// @param glyphs List of Unicode codepoints to bake
    /// @return FontData containing glyph metrics and atlas texture
    [[nodiscard]] static FontData loadFontWithGlyphs(
        const std::string& path,
        float fontHeight,
        const std::vector<uint32_t>& glyphs);
};

} // namespace ge
