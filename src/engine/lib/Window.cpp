#include "engine/Window.h"

#include <GLFW/glfw3.h>
#include <stdexcept>

namespace ge {

Window::Window(int width, int height, const char* title)
    : m_width(width)
    , m_height(height)
{
    // Initialize GLFW and create a native window for Vulkan presentation.
    // Throws on failure so callers can abort cleanly.

    // glfwInit must be called once per process before any other GLFW function.
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Tell GLFW not to create an OpenGL context — we use Vulkan only.
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
}

Window::~Window() {
    // Destroy the GLFW window and terminate GLFW on teardown.
    if (m_window) {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

bool Window::shouldClose() const {
    // Query whether the OS or the app requested window close.
    return glfwWindowShouldClose(m_window);
}

void Window::pollEvents() {
    // Process pending window/input events (non-blocking).
    glfwPollEvents();
}

} // namespace ge
