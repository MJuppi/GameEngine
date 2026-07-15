#include "engine/Window.h"

#include <GLFW/glfw3.h>
#include <stdexcept>

namespace {

void initializeGlfw() {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }
}

} // namespace

namespace ge {

Window::Window(int width, int height, const char* title)
    : m_width(width)
    , m_height(height)
{
    initializeGlfw();

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    if (glfwRawMouseMotionSupported()) {
        glfwSetInputMode(m_window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    }
}

Window::~Window() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}

bool Window::shouldClose() const {
    return m_window != nullptr && glfwWindowShouldClose(m_window);
}

void Window::pollEvents() {
    glfwPollEvents();
}

} // namespace ge
