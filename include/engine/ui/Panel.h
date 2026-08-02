#pragma once

#include "engine/ui/UIElement.h"
#include <vector>
#include <memory>

namespace ge {

/**
 * @brief A container UI element that can hold other UI elements.
 */
class Panel : public UIElement {
public:
    void render() override {
        if (!m_visible) return;

        // Panel rendering could include drawing a background quad

        for (auto& child : m_children) {
            child->render();
        }
    }

    bool handleInput(double xpos, double ypos, int button, int action, int mods) override {
        if (!m_visible) return false;

        // Handle input for children in reverse order (topmost first)
        for (auto it = m_children.rbegin(); it != m_children.rend(); ++it) {
            if ((*it)->handleInput(xpos, ypos, button, action, mods)) {
                return true;
            }
        }
        return false;
    }

    void addChild(std::shared_ptr<UIElement> child) {
        m_children.push_back(std::move(child));
    }

    const std::vector<std::shared_ptr<UIElement>>& getChildren() const { return m_children; }

private:
    std::vector<std::shared_ptr<UIElement>> m_children;
};

} // namespace ge
