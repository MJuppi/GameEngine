#include "game/Game.h"
#include "game/LevelBuilder.h"
#include "game/PlayerController.h"
#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/scene/ObjectBuilder.h"
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

    std::cout << "Initializing game engine...\n";
    state_ = GameState::Loading;

    assetManager_.loadManifest("assets/manifest.json");

    initializeLevels();

    Level* currentLevel = levelManager_.getCurrentLevel();
    bool success = false;
    if (currentLevel) {
        success = loadLevel(*currentLevel);
    }

    if (!success) {
        loadFallbackLevel();
    }

    std::cout << "Game initialized successfully.\n";
    state_ = GameState::Running;
}

void Game::run() {
    if (!engine_) {
        std::cerr << "Engine not initialized. Call initialize() first.\n";
        return;
    }

    state_ = GameState::Running;
    engine_->run();
}

void Game::updateGameplay(float deltaTime) {
    if (playerController_) {
        playerController_->update(deltaTime);
    }
}

bool Game::loadLevel(Level& level) {
    std::cout << "Loading level: " << level.getName() << '\n';
    level.load(assetManager_);

    if (!level.isLoaded()) {
        std::cerr << "Failed to load level assets for: " << level.getName() << '\n';
        return false;
    }

    engine_ = std::make_unique<Engine>(level.getMesh(), level.getPointLight());
    for (auto& levelObject : level.getObjects()) {
        ObjectBuilder::attachPhysics(engine_->getPhysicsEngine(), levelObject);
    }

    playerController_ = std::make_unique<PlayerController>(*engine_);
    engine_->setFrameUpdateCallback([this](float deltaTime) {
        updateGameplay(deltaTime);
    });

    return true;
}

void Game::loadFallbackLevel() {
    std::cout << "Using fallback unit cube level.\n";
    auto fallbackObject = ObjectBuilder::createActive(
        "FallbackCube",
        "", // use default cube
        {0.0f, 0.0f, 0.0f},
        {0.5f, 0.5f, 0.5f},
        RigidBodyProps{1.0f, 0.3f, 0.7f});

    engine_ = std::make_unique<Engine>(fallbackObject.mesh);
    ObjectBuilder::attachPhysics(engine_->getPhysicsEngine(), fallbackObject);
}

void Game::shutdown() {
    if (state_ == GameState::Shutdown) {
        return;
    }

    std::cout << "Shutting down game...\n";

    playerController_.reset();
    engine_.reset();
    levelManager_.unloadAll();

    state_ = GameState::Shutdown;
}

void Game::initializeLevels() {
    LevelBuilder::registerDefaultLevels(levelManager_);
    levelManager_.setCurrentLevel("TestCube");
}

} // namespace ge