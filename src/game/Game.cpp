#include "game/Game.h"
#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/physics/PhysicsEngine.h"
#include <iostream>
#include <glm/glm.hpp>

namespace ge {

Game::Game() = default;

Game::~Game() {
    shutdown();
}

void Game::initialize() {
    if (state_ != GameState::Uninitialized) {
        return;
    }

    std::cout << "Initializing game...\n";
    state_ = GameState::Loading;

    initializeLevels();

    // Load current level
    Level* currentLevel = levelManager_.getCurrentLevel();
    if (currentLevel) {
        std::cout << "Loading level: " << currentLevel->getName() << '\n';
        currentLevel->load(assetManager_);

        engine_ = std::make_unique<Engine>(currentLevel->getMesh());
        setupTestPhysics();
    } else {
        // Fallback
        std::cout << "No level available, using fallback unit cube.\n";
        engine_ = std::make_unique<Engine>(makeUnitCubeMesh());

        RigidBodyProps testCubeProps;
        testCubeProps.mass = 1.0f;
        testCubeProps.restitution = 0.7f;
        testCubeProps.friction = 0.3f;

        engine_->getPhysicsEngine().createBoxBody({0.5f, 0.5f, 0.5f},
                                                  glm::mat4(1.0f),
                                                  testCubeProps);
    }

    std::cout << "Game initialized successfully.\n";
    state_ = GameState::Running;
}

void Game::run() {
    if (!engine_) {
        std::cerr << "Engine not initialized. Call initialize() first.\n";
        return;
    }

    if (state_ != GameState::Running) {
        state_ = GameState::Running;
    }

    engine_->run();
}

void Game::shutdown() {
    if (state_ == GameState::Shutdown) {
        return;
    }

    std::cout << "Shutting down game...\n";

    engine_.reset();
    levelManager_.unloadAll();

    state_ = GameState::Shutdown;
}

void Game::initializeLevels() {
    levelManager_.addLevel("TestCube", "assets/models/TestCube.obj");
    
    // Add more levels here as needed:
    // levelManager_.addLevel("Level2", "assets/models/Level2.obj");
    
    levelManager_.setCurrentLevel("TestCube");
}

void Game::setupTestPhysics() {
    // Falling test cube
    RigidBodyProps testCubeProps;
    testCubeProps.mass = 1.0f;
    testCubeProps.restitution = 0.7f;
    testCubeProps.friction = 0.3f;

    engine_->getPhysicsEngine().createBoxBody({0.5f, 0.5f, 0.5f},
                                              glm::translate(glm::mat4(1.0f), {0.0f, 5.0f, 0.0f}),
                                              testCubeProps);

    // Ground plane
    RigidBodyProps groundProps;
    groundProps.mass = 0.0f;
    groundProps.isKinematic = true;
    groundProps.useGravity = false;

    engine_->getPhysicsEngine().createBoxBody({50.0f, 0.5f, 50.0f},
                                              glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}),
                                              groundProps);
}

} // namespace ge