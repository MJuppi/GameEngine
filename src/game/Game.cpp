#include "game/Game.h"
#include "game/LevelBuilder.h"
#include "game/PlayerController.h"
#include "game/SceneFactory.h"
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

    std::cout << "Initializing game...\n";
    state_ = GameState::Loading;

    initializeLevels();

    Level* currentLevel = levelManager_.getCurrentLevel();
    if (currentLevel) {
        initializeLevel(*currentLevel);
    } else {
        initializeFallbackLevel();
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

void Game::updateGameplay(float deltaTime) {
    if (playerController_) {
        playerController_->update(deltaTime);
    }
}

void Game::initializeLevel(Level& level) {
    std::cout << "Loading level: " << level.getName() << '\n';
    level.load(assetManager_);

    engine_ = std::make_unique<Engine>(level.getMesh(), level.getPointLight());
    for (auto& levelObject : level.getObjects()) {
        ObjectBuilder::attachPhysics(engine_->getPhysicsEngine(), levelObject);
    }

    playerController_ = std::make_unique<PlayerController>(*engine_);
    engine_->setFrameUpdateCallback([this](float deltaTime) {
        updateGameplay(deltaTime);
    });
    setupTestPhysics();
}

void Game::initializeFallbackLevel() {
    std::cout << "No level available, using fallback unit cube.\n";
    auto fallbackObject = ObjectBuilder::createObject(
        "FallbackCube",
        makeUnitCubeMesh(),
        glm::mat4(1.0f),
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

void Game::setupTestPhysics() {
    SceneFactory::setupTestPhysics(*engine_);
}

} // namespace ge