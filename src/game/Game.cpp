#include "game/Game.h"
#include "game/LevelBuilder.h"
#include "game/PlayerController.h"
#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/ui/UIManager.h"
#include "engine/mesh/MeshData.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/RigidBody.h"
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

bool Game::loadLevel(Level& level) {
    std::cout << "Loading level: " << level.getName() << '\n';
    level.load(assetManager_);

    if (!level.isLoaded()) {
        std::cerr << "Failed to load level assets for: " << level.getName() << '\n';
        return false;
    }

    engine_ = std::make_unique<Engine>(level.getMesh(), level.getSceneLights());
    for (auto& levelObject : level.getObjects()) {
        ObjectBuilder::attachPhysics(engine_->getPhysicsEngine(), levelObject);
    }

    if (level.getName() == "PairOrderingParity") {
        SceneFactory::setupPairOrderingPhysics(*engine_);
    } else {
        engine_->getPhysicsEngine().getWorld().setContactManifoldCallback(
            [](const ContactManifold& manifold) {
                const std::string bodyNameA = (manifold.bodyA && !manifold.bodyA->getName().empty())
                    ? manifold.bodyA->getName()
                    : std::string("BodyA");
                const std::string bodyNameB = (manifold.bodyB && !manifold.bodyB->getName().empty())
                    ? manifold.bodyB->getName()
                    : std::string("BodyB");

                std::cout << "[manifold] " << bodyNameA << " <-> " << bodyNameB
                          << " contacts=" << manifold.contacts.size()
                          << " normal=(" << manifold.normal.x << ", " << manifold.normal.y << ", " << manifold.normal.z << ")\n";
            });
    }

    if (level.getName() == "ConstraintParity") {
        SceneFactory::setupConstraintParityPhysics(*engine_);
    }

    playerController_ = std::make_unique<PlayerController>(*engine_);

    SceneFactory::setupUI(*engine_);

    engine_->setFixedUpdateCallback([this](float deltaTime) {
        if (playerController_) {
            playerController_->fixedUpdate(deltaTime);
        }
    });

    engine_->setVariableUpdateCallback([this](float deltaTime, float alpha) {
        engine_->getUIManager().update(deltaTime);
        if (playerController_) {
            playerController_->variableUpdate(deltaTime, alpha);
        }
    });

    return true;
}

void Game::loadFallbackLevel() {
    std::cout << "Using fallback unit cube level.\n";
    auto fallbackObject = ObjectBuilder::createActive(
        "FallbackCube",
        "test_cube",
        {0.0f, 0.0f, 0.0f},
        {0.5f, 0.5f, 0.5f},
        RigidBodyProps{1.0f, 0.3f, 0.7f});

    try {
        fallbackObject.mesh = assetManager_.loadMesh(fallbackObject.meshPath);
    } catch (...) {
        fallbackObject.mesh = makeUnitCubeMesh();
    }

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
    if (!levelManager_.setCurrentLevel("PairOrderingParity")) {
        if (!levelManager_.setCurrentLevel("ConstraintParity")) {
            levelManager_.setCurrentLevel("TestCube");
        }
    }
}

} // namespace ge