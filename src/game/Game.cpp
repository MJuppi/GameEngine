#include "game/Game.h"
#include "engine/mesh/MeshData.h"

#include <iostream>
#include <filesystem>
#include <algorithm>

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
    loadModelsFromDirectory();
    
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
    // Create default levels
    // TODO: Load level definitions from configuration or files
    
    auto defaultLevel = std::make_shared<Level>("Default");
    const fs::path modelDir = "assets/models";
    
    // Try to find first available model
    if (fs::exists(modelDir) && fs::is_directory(modelDir)) {
        for (const auto& entry : fs::directory_iterator(modelDir)) {
            if (!entry.is_regular_file()) {
                continue;
            }

            const auto ext = entry.path().extension();
            if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
                defaultLevel->setMeshPath(entry.path().string());
                break;
            }
        }
    }
    
    levelManager_.addLevel(defaultLevel);
}

void Game::loadModelsFromDirectory() {
    const fs::path modelDir = "assets/models";
    std::vector<fs::path> modelFiles;

    if (fs::exists(modelDir) && fs::is_directory(modelDir)) {
        for (const auto& entry : fs::directory_iterator(modelDir)) {
            if (!entry.is_regular_file()) {
                continue;
            }

            const auto ext = entry.path().extension();
            if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
                modelFiles.push_back(entry.path());
            }
        }
    }

    if (!modelFiles.empty()) {
        std::sort(modelFiles.begin(), modelFiles.end());
        std::cout << "Found " << modelFiles.size() << " model(s) in " << modelDir.string() << '\n';
        
        // Log available models
        for (const auto& model : modelFiles) {
            std::cout << "  - " << model.filename().string() << '\n';
        }
    } else {
        std::cout << "No models found in " << modelDir.string() << '\n';
    }
}

}  // namespace ge
