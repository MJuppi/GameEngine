#pragma once

#include "engine/ui/UIElement.h"
#include <string>
#include <functional>

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

private:
    std::string m_text;
    float m_fontSize = 1.0f;
    std::function<void(Label&, float)> m_onUpdate;
};

} // namespace ge
