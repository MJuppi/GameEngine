#pragma once

#include "engine/ui/UIElement.h"
#include <vector>
#include <memory>

namespace ge {

/**
 * @brief Manages UI elements and their interactions.
 */
class UIManager {
public:
    /**
     * @brief Adds a UI element to the manager.
     * @param element The element to add.
     */
    void addElement(std::shared_ptr<UIElement> element);

    /**
     * @brief Removes a UI element from the manager.
     * @param element The element to remove.
     */
    void removeElement(std::shared_ptr<UIElement> element);

    /**
     * @brief Propagates mouse events to UI elements.
     * @param xpos Mouse X position (normalized 0..1 or pixel).
     * @param ypos Mouse Y position (normalized 0..1 or pixel).
     * @param button Mouse button.
     * @param action Action (press/release).
     */
    void processInput(double xpos, double ypos, int button, int action);

    /**
     * @brief Updates all UI elements.
     */
    void update(float deltaTime);

    /**
     * @brief Returns a list of all visible UI elements for rendering.
     * @return List of visible elements.
     */
    std::vector<std::shared_ptr<UIElement>> getVisibleElements() const;

private:
    std::vector<std::shared_ptr<UIElement>> m_elements;
};

} // namespace ge
