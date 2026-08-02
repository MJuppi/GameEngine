#include "engine/ui/UIManager.h"
#include <algorithm>

namespace ge {

void UIManager::addElement(std::shared_ptr<UIElement> element) {
    m_elements.push_back(element);
}

void UIManager::removeElement(std::shared_ptr<UIElement> element) {
    m_elements.erase(std::remove(m_elements.begin(), m_elements.end(), element), m_elements.end());
}

void UIManager::processInput(double xpos, double ypos, int button, int action) {
    // Traverse in reverse order to handle top-most elements first (Z-order)
    for (auto it = m_elements.rbegin(); it != m_elements.rend(); ++it) {
        if ((*it)->isVisible()) {
            // UIElement::handleInput takes mods, passing 0 as it's not provided in processInput
            if ((*it)->handleInput(xpos, ypos, button, action, 0)) {
                break; // Event consumed
            }
        }
    }
}

void UIManager::update(float deltaTime) {
    for (auto& element : m_elements) {
        element->update(deltaTime);
    }
}

std::vector<std::shared_ptr<UIElement>> UIManager::getVisibleElements() const {
    std::vector<std::shared_ptr<UIElement>> visibleElements;
    for (const auto& element : m_elements) {
        if (element->isVisible()) {
            visibleElements.push_back(element);
        }
    }
    return visibleElements;
}

} // namespace ge
