#pragma once

#include "engine/ui/UIElement.h"
#include <functional>

namespace ge {

/**
 * @brief A clickable button UI element.
 */
class Button : public UIElement {
public:
    using ClickCallback = std::function<void()>;

    void render() override {
        if (!m_visible) return;
        // Rendering implementation would use a renderer to draw the button
    }

    bool handleInput(double xpos, double ypos, int button, int action, int mods) override {
        if (!m_visible) return false;

        // Simple hit test (AABB)
        bool isInside = xpos >= m_position.x && xpos <= m_position.x + m_size.x &&
                       ypos >= m_position.y && ypos <= m_position.y + m_size.y;

        if (isInside && action == 1) { // action == 1 typically means PRESS (e.g. GLFW_PRESS)
            if (m_onClick) {
                m_onClick();
            }
            return true;
        }

        return false;
    }

    void setOnClick(ClickCallback callback) { m_onClick = std::move(callback); }

private:
    ClickCallback m_onClick;
};

} // namespace ge
