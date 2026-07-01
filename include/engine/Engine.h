#pragma once

// =============================================================================
// Engine.h — Top-level game loop owner
// =============================================================================
// Engine wires Window + VulkanRenderer + PhysicsEngine. main.cpp only constructs Engine and
// calls run() — keeps bootstrap code tiny and readable.
// =============================================================================

#include "engine/mesh/MeshData.h"
#include "engine/scene/Light.h"

namespace ge {

class PhysicsEngine;

class Engine {
public:
    /// Pass mesh data from disk (e.g. loadObjFile) or use makeUnitCubeMesh().
    explicit Engine(MeshData mesh, PointLight pointLight = PointLight{});
    ~Engine();

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    /// Blocks until the window is closed. One frame = poll input + render.
    void run();

    /// Get the physics engine
    PhysicsEngine& getPhysicsEngine();
    const PhysicsEngine& getPhysicsEngine() const;
    void setPointLight(const PointLight& pointLight);

private:
    class Impl;
    Impl* m_impl = nullptr;
};

} // namespace ge
