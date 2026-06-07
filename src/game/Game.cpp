#include "game/Game.h"
#include "engine/mesh/MeshData.h"

#include <iostream>
#include <filesystem>

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
    } else {
        // Fallback to unit cube if no levels
        engine_ = std::make_unique<Engine>(makeUnitCubeMesh());
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
    // Create a single default level. The level itself will choose a model from
    // assets/models if one is available, otherwise it will fall back to the
    // built-in scene.
    auto defaultLevel = std::make_shared<Level>("Default");
    levelManager_.addLevel(defaultLevel);
}


}  // namespace ge
