#include "engine/Engine.h"

#include "engine/Window.h"
#include "engine/vulkan/VulkanRenderer.h"

#include <GLFW/glfw3.h>
#include <memory>

namespace ge {

class Engine::Impl {
public:
    static constexpr int kWidth = 1280;
    static constexpr int kHeight = 720;
    static constexpr const char* kTitle = "GameEngine — Vulkan";

    Window window{kWidth, kHeight, kTitle};
    VulkanRenderer renderer;

    explicit Impl(MeshData mesh)
        : renderer(window, std::move(mesh))
    {
    }

    void run() {
        while (!window.shouldClose()) {
            window.pollEvents();

            if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.handle(), GLFW_TRUE);
            }

            renderer.drawFrame();
        }
        // VulkanRenderer destructor calls vkDeviceWaitIdle before teardown.
    }
};

Engine::Engine(MeshData mesh)
    : m_impl(new Impl(std::move(mesh)))
{
}

Engine::~Engine() {
    delete m_impl;
}

void Engine::run() {
    m_impl->run();
}

} // namespace ge
