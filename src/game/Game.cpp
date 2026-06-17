#include "game/Game.h"
#include "engine/mesh/MeshData.h"

#include <iostream>
#include <filesystem>
#include <glm/glm.hpp>

namespace fs = std::filesystem;
namespace ge {

Game::Game()
    : state_(GameState::Uninitialized), engine_(nullptr) {
}

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
    
    // Create engine with the current level's mesh
    auto currentLevel = levelManager_.getCurrentLevel();
    if (currentLevel) {
        currentLevel->load();
        engine_ = std::make_unique<Engine>(currentLevel->getMesh());
        
        // Create physics bodies using the engine's physics engine
        RigidBodyProps testCubeProps;
        testCubeProps.mass = 1.0f;
        testCubeProps.restitution = 0.7f;
        testCubeProps.friction = 0.3f;
        
        // Create a test cube body
        engine_->getPhysicsEngine().createBoxBody({0.5f, 0.5f, 0.5f}, 
                                                  glm::translate(glm::mat4(1.0f), {0.0f, 5.0f, 0.0f}), 
                                                  testCubeProps);
        
        // Create a ground plane (kinematic body)
        RigidBodyProps groundProps;
        groundProps.mass = 0.0f; // Infinite mass (kinematic)
        groundProps.isKinematic = true;
        groundProps.useGravity = false;
        
        // Large box as ground
        engine_->getPhysicsEngine().createBoxBody({50.0f, 0.5f, 50.0f}, 
                                                   glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}), 
                                                   groundProps);
    } else {
        // Fallback to unit cube if no levels
        engine_ = std::make_unique<Engine>(makeUnitCubeMesh());
        
        // Create a test physics body
        RigidBodyProps testCubeProps;
        testCubeProps.mass = 1.0f;
        testCubeProps.restitution = 0.7f;
        
        engine_->getPhysicsEngine().createBoxBody({0.5f, 0.5f, 0.5f}, glm::mat4(1.0f), testCubeProps);
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
    levelManager_.unloadAllLevels();
    state_ = GameState::Shutdown;
}

void Game::initializeLevels() {
    // Create a level with the test cube
    auto testCubeLevel = std::make_shared<Level>("TestCube", "assets/models/TestCube.obj");
    levelManager_.addLevel(testCubeLevel);
    
    // Set the test cube level as current
    levelManager_.setCurrentLevel("TestCube");
}

}  // namespace ge
