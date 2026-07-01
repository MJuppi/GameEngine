#include "engine/Engine.h"
#include "engine/FrameTimer.h"
#include "engine/physics/PhysicsEngine.h"

#include "engine/Window.h"
#include "engine/vulkan/VulkanRenderer.h"

#include <GLFW/glfw3.h>

namespace ge {

class Engine::Impl {
public:
    static constexpr int kWidth = 1280;
    static constexpr int kHeight = 720;
    static constexpr const char* kTitle = "GameEngine — Vulkan";

    Window window{kWidth, kHeight, kTitle};
    VulkanRenderer renderer;
    PhysicsEngine physicsEngine;
    FrameTimer frameTimer;
    FrameUpdateCallback frameUpdateCallback;

    explicit Impl(MeshData mesh, PointLight pointLight)
        : renderer(window, std::move(mesh))
    {
        renderer.setPointLight(pointLight);

        physicsEngine.setGravity({0.0f, -9.81f, 0.0f});
        physicsEngine.setFixedTimeStep(1.0f / 60.0f);
        frameTimer.reset();
    }

    void run() {
        while (!window.shouldClose()) {
            const float deltaTime = frameTimer.beginFrame();
            window.pollEvents();
            handleWindowInput();

            if (frameUpdateCallback) {
                frameUpdateCallback(deltaTime);
            }

            physicsEngine.update(deltaTime);
            renderer.drawFrame();
        }
    }

public:
    PhysicsEngine& getPhysicsEngine() { return physicsEngine; }

private:
    void handleWindowInput() {
        if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window.handle(), GLFW_TRUE);
        }
    }
};

Engine::Engine(MeshData mesh, PointLight pointLight)
    : m_impl(new Impl(std::move(mesh), std::move(pointLight)))
{
}

Engine::~Engine() {
    delete m_impl;
}

void Engine::run() {
    m_impl->run();
}

void Engine::setPointLight(const PointLight& pointLight) {
    m_impl->renderer.setPointLight(pointLight);
}

void Engine::setFrameUpdateCallback(FrameUpdateCallback callback) {
    m_impl->frameUpdateCallback = std::move(callback);
}

GLFWwindow* Engine::getWindowHandle() const {
    return m_impl->window.handle();
}

PhysicsEngine& Engine::getPhysicsEngine() {
    return m_impl->getPhysicsEngine();
}

const PhysicsEngine& Engine::getPhysicsEngine() const {
    return m_impl->getPhysicsEngine();
}

} // namespace ge
