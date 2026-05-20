#pragma once

// =============================================================================
// Window.h — OS window via GLFW
// =============================================================================
// Vulkan does not create windows. GLFW gives us:
//   - A native window handle
//   - Input events (keyboard, resize)
//   - VkSurfaceKHR creation via glfwCreateWindowSurface
// =============================================================================

struct GLFWwindow;

namespace ge {

class Window {
public:
    /// width/height in pixels; title shown in the title bar.
    Window(int width, int height, const char* title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    [[nodiscard]] GLFWwindow* handle() const { return m_window; }
    [[nodiscard]] int width() const { return m_width; }
    [[nodiscard]] int height() const { return m_height; }
    [[nodiscard]] bool shouldClose() const;
    void pollEvents();

private:
    GLFWwindow* m_window = nullptr;
    int m_width = 0;
    int m_height = 0;
};

} // namespace ge
