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
#include <iomanip>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

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
    while (engine_) {
        escWasDown_ = false;
        pendingLevelIndex_.reset();
        setMenuVisible(false);
        engine_->run();

        if (!pendingLevelIndex_.has_value()) {
            break;
        }

        const size_t requestedIndex = *pendingLevelIndex_;
        pendingLevelIndex_.reset();

        if (!levelManager_.setCurrentLevel(requestedIndex)) {
            std::cerr << "Requested level index is out of range: " << requestedIndex << '\n';
            break;
        }

        playerController_.reset();
        engine_.reset();

        Level* nextLevel = levelManager_.getCurrentLevel();
        if (!nextLevel || !loadLevel(*nextLevel)) {
            loadFallbackLevel();
            break;
        }
    }
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

                const bool probeStack0 = bodyNameA == "Stack_0" || bodyNameB == "Stack_0";
                if (probeStack0) {
                    std::cout << "[probe] Stack_0 manifold " << bodyNameA << " <-> " << bodyNameB
                              << " contacts=" << manifold.contacts.size()
                              << " normal=(" << std::fixed << std::setprecision(3)
                              << manifold.normal.x << ", " << manifold.normal.y << ", " << manifold.normal.z << ")";

                    if (!manifold.contacts.empty()) {
                        const Contact& contact = manifold.contacts.front();
                        std::cout << " point=(" << contact.point.x << ", " << contact.point.y << ", " << contact.point.z << ")"
                                  << " depth=" << contact.depth
                                  << " nImpulse=" << contact.normalImpulse
                                  << " tImpulse=" << contact.tangentImpulse;
                    }

                    if (manifold.bodyA && bodyNameA == "Stack_0") {
                        const glm::vec3 pos = manifold.bodyA->getPosition();
                        const glm::vec3 vel = manifold.bodyA->getVelocity();
                        std::cout << " posA=(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                                  << " velA=(" << vel.x << ", " << vel.y << ", " << vel.z << ")";
                    } else if (manifold.bodyB && bodyNameB == "Stack_0") {
                        const glm::vec3 pos = manifold.bodyB->getPosition();
                        const glm::vec3 vel = manifold.bodyB->getVelocity();
                        std::cout << " posB=(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                                  << " velB=(" << vel.x << ", " << vel.y << ", " << vel.z << ")";
                    }

                    std::cout << '\n';
                }

                std::cout << "[manifold] " << bodyNameA << " <-> " << bodyNameB
                          << " contacts=" << manifold.contacts.size()
                          << " normal=(" << manifold.normal.x << ", " << manifold.normal.y << ", " << manifold.normal.z << ")\n";
            });
    }

    if (level.getName() == "ConstraintParity") {
        SceneFactory::setupConstraintParityPhysics(*engine_);
    }

    playerController_ = std::make_unique<PlayerController>(*engine_);

    const int currentIndex = levelManager_.getCurrentLevelIndex();
    const size_t safeIndex = currentIndex < 0 ? 0u : static_cast<size_t>(currentIndex);
    pauseMenu_ = SceneFactory::setupUI(
        *engine_,
        levelManager_,
        safeIndex,
        [this](size_t levelIndex) {
            pendingLevelIndex_ = levelIndex;
            if (engine_) {
                engine_->requestStop();
            }
        });
    setMenuVisible(false);

    engine_->setFixedUpdateCallback([this](float deltaTime) {
        if (playerController_ && !isMenuVisible()) {
            playerController_->fixedUpdate(deltaTime);
        }
    });

    engine_->setVariableUpdateCallback([this](float deltaTime, float alpha) {
        if (engine_) {
            auto* window = engine_->getWindowHandle();
            if (window) {
                const bool escDown = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
                if (escDown && !escWasDown_) {
                    setMenuVisible(!isMenuVisible());
                }
                escWasDown_ = escDown;
            }
        }

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
    if (!levelManager_.setCurrentLevel("TestCube")) {
        levelManager_.setCurrentLevel(0);
    }
}

void Game::setMenuVisible(bool visible) {
    if (pauseMenu_.setVisible) {
        pauseMenu_.setVisible(visible);
    }

    if (playerController_) {
        playerController_->setInputEnabled(!visible);
    }
}

bool Game::isMenuVisible() const {
    return pauseMenu_.isVisible ? pauseMenu_.isVisible() : false;
}

} // namespace ge