#pragma once

#include <glm/glm.hpp>

namespace ge {

/**
 * @brief Base class for all UI elements.
 */
class UIElement {
public:
    virtual ~UIElement() = default;

    /**
     * @brief Renders the UI element.
     */
    virtual void render() = 0;

    /**
     * @brief Updates the UI element.
     */
    virtual void update(float /*deltaTime*/) {}

    /**
     * @brief Handles input events.
     * @return true if the event was consumed, false otherwise.
     */
    virtual bool handleInput(double xpos, double ypos, int button, int action, int mods) { return false; }

    void setPosition(const glm::vec2& pos) { m_position = pos; }
    void setSize(const glm::vec2& size) { m_size = size; }
    void setColor(const glm::vec4& color) { m_color = color; }
    void setVisible(bool visible) { m_visible = visible; }

    const glm::vec2& getPosition() const { return m_position; }
    const glm::vec2& getSize() const { return m_size; }
    const glm::vec4& getColor() const { return m_color; }
    bool isVisible() const { return m_visible; }

protected:
    glm::vec2 m_position{0.0f};
    glm::vec2 m_size{100.0f};
    glm::vec4 m_color{1.0f};
    bool m_visible = true;
};

} // namespace ge
