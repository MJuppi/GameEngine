#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"

#include "engine/Window.h"
#include "engine/vulkan/VulkanRenderer.h"

#include <GLFW/glfw3.h>
#include <memory>
#include <chrono>

namespace ge {

class Engine::Impl {
public:
    static constexpr int kWidth = 1280;
    static constexpr int kHeight = 720;
    static constexpr const char* kTitle = "GameEngine — Vulkan";

    Window window{kWidth, kHeight, kTitle};
    VulkanRenderer renderer;
    PhysicsEngine physicsEngine;
    
    // Time tracking for delta time calculation
    std::chrono::high_resolution_clock::time_point lastFrameTime;
    float deltaTime = 0.0f;
    bool firstFrame = true;

    explicit Impl(MeshData mesh)
        : renderer(window, std::move(mesh)),
          lastFrameTime(std::chrono::high_resolution_clock::now())
    {
        // Set up physics engine with default gravity
        physicsEngine.setGravity({0.0f, -9.81f, 0.0f});
        physicsEngine.setFixedTimeStep(1.0f / 60.0f);
    }

    void run() {
        // Main loop: poll OS events, handle escape key, and render each frame.
        while (!window.shouldClose()) {
            // Calculate delta time
            auto currentTime = std::chrono::high_resolution_clock::now();
            if (!firstFrame) {
                auto duration = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - lastFrameTime);
                deltaTime = duration.count();
            } else {
                deltaTime = 1.0f / 60.0f; // Default for first frame
                firstFrame = false;
            }
            lastFrameTime = currentTime;

            window.pollEvents();

            if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.handle(), GLFW_TRUE);
            }

            // Update physics simulation
            physicsEngine.update(deltaTime);

            // Sync the first dynamic body's transform to the renderer
            for (const auto& body : physicsEngine.getWorld().getBodies()) {
                if (!body->getProps().isKinematic) {
                    renderer.setModelMatrix(body->getWorldTransform());
                    break;
                }
            }

            renderer.drawFrame();
        }
        // VulkanRenderer destructor calls vkDeviceWaitIdle before teardown.
    }
    
    // Public access to physics engine for the Game class
    PhysicsEngine& getPhysicsEngine() { return physicsEngine; }
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

PhysicsEngine& Engine::getPhysicsEngine() {
    return m_impl->getPhysicsEngine();
}

const PhysicsEngine& Engine::getPhysicsEngine() const {
    return m_impl->getPhysicsEngine();
}

} // namespace ge
