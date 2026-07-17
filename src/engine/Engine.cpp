#include "engine/Engine.h"
#include "engine/FrameTimer.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/RigidBody.h"

#include "engine/Window.h"
#include "engine/vulkan/VulkanRenderer.h"

#include <GLFW/glfw3.h>
#include <iostream>

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
    FixedUpdateCallback fixedUpdateCallback;
    VariableUpdateCallback variableUpdateCallback;

    explicit Impl(MeshData mesh, SceneLights sceneLights)
        : renderer(window, std::move(mesh))
    {
        renderer.setSceneLights(sceneLights);

        physicsEngine.setGravity({0.0f, -9.81f, 0.0f});
        physicsEngine.setFixedTimeStep(1.0f / 60.0f);
        physicsEngine.setSolverIterations(8);
        frameTimer.reset();
    }

    void run() {
        const float fixedStep = 1.0f / 60.0f;
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            const float deltaTime = frameTimer.beginFrame();
            window.pollEvents();

            accumulator += deltaTime;
            if (accumulator > 0.25f) accumulator = 0.25f;

            while (accumulator >= fixedStep) {
                if (fixedUpdateCallback) {
                    fixedUpdateCallback(fixedStep);
                }

                physicsEngine.update(fixedStep); // This internal update handles its own internal accumulator too,
                                                 // but we want to call fixedUpdate logic here.
                                                 // Actually, it's better if we let physicsEngine.update handle the loop
                                                 // but we need to hook into each step.

                accumulator -= fixedStep;
            }

            const float alpha = accumulator / fixedStep;

            if (variableUpdateCallback) {
                variableUpdateCallback(deltaTime, alpha);
            }

            // Sync physics transforms to renderer with interpolation
            const auto& bodies = physicsEngine.getWorld().getBodies();
            renderer.clearDynamicObjects();
            for (const auto& body : bodies) {
                if (!body->getProps().isKinematic && body->getProps().mass > 0.0f) {
                    const auto& mesh = body->getMesh();
                    glm::mat4 interpolatedTransform = body->getInterpolatedTransform(alpha);
                    renderer.addDynamicObject(interpolatedTransform, body->getMaterial(), mesh.has_value() ? &mesh.value() : nullptr);
                }
            }

            renderer.drawFrame();
        }
    }

public:
    PhysicsEngine& getPhysicsEngine() { return physicsEngine; }

private:
    void handleWindowInput() {
    }
};

Engine::Engine(MeshData mesh, SceneLights sceneLights)
    : m_impl(new Impl(std::move(mesh), std::move(sceneLights)))
{
}

Engine::~Engine() {
    delete m_impl;
}

void Engine::run() {
    m_impl->run();
}

void Engine::setSceneLights(const SceneLights& sceneLights) {
    m_impl->renderer.setSceneLights(sceneLights);
}

void Engine::setCamera(const glm::vec3& position, const glm::vec3& front, const glm::vec3& up) {
    m_impl->renderer.setCamera(position, front, up);
}

void Engine::setFixedUpdateCallback(FixedUpdateCallback callback) {
    m_impl->fixedUpdateCallback = std::move(callback);
}

void Engine::setVariableUpdateCallback(VariableUpdateCallback callback) {
    m_impl->variableUpdateCallback = std::move(callback);
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
