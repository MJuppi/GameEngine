#pragma once

#include "engine/ui/UIElement.h"
#include <string>
#include <functional>
#include <glm/glm.hpp>
#include "engine/ui/BitmapFont.h"

namespace ge {

/**
 * @brief A UI element for displaying text.
 */
class Label : public UIElement {
public:
    void render() override {
        // Rendering logic is handled by the renderer.
    }

    void update(float deltaTime) override {
        if (m_onUpdate) {
            m_onUpdate(*this, deltaTime);
        }
    }

    void setText(const std::string& text) { m_text = text; }
    const std::string& getText() const { return m_text; }

    void setFontSize(float size) { m_fontSize = size; }
    float getFontSize() const { return m_fontSize; }

    /**
     * @brief Set a callback for updating the label text or state.
     */
    void setOnUpdate(std::function<void(Label&, float)> callback) {
        m_onUpdate = std::move(callback);
    }

    // Iterate glyphs of the current text and invoke callback with glyph position, size, and UV rect
    void forEachGlyph(std::function<void(const glm::vec2& pos, const glm::vec2& size, const glm::vec4& uv)> cb) const {
        // Compute glyph sizes to fit inside element size when possible
        int glyphCount = 0;
        for (char c : m_text) if (c >= 32 && c <= 126) ++glyphCount;
        if (glyphCount == 0) return;

        // Use the element's size as the basis for glyph layout. Respect m_fontSize as a multiplier.
        float totalWidth = m_size.x;
        float glyphWidth = (totalWidth / static_cast<float>(glyphCount)) * 0.9f * (m_fontSize);
        float glyphHeight = m_size.y * (m_fontSize);

        // Start at the element's position (left-aligned). Future: add alignment options.
        float xOffset = 0.0f;
        for (char c : m_text) {
            if (c < 32 || c > 126) continue;
            glm::vec2 glyphPos = m_position + glm::vec2(xOffset, 0.0f);
            glm::vec2 glyphSize = glm::vec2(glyphWidth, glyphHeight);
            glm::vec4 uv = BitmapFont::getCharUV(c);
            cb(glyphPos, glyphSize, uv);
            xOffset += glyphWidth;
        }
    }

private:
    std::string m_text;
    float m_fontSize = 1.0f;
    std::function<void(Label&, float)> m_onUpdate;
};

} // namespace ge
