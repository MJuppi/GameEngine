#pragma once
#include "game/GameState.h"
#include "game/LevelManager.h"
#include "game/SceneFactory.h"
#include "engine/asset/AssetManager.h"
#include <memory>
#include <optional>

namespace ge {

class Engine;
class AssetManager;
class PlayerController;

class Game {
public:
    Game();
    ~Game();

    void initialize();
    void run();
    void shutdown();

    LevelManager& getLevelManager() { return levelManager_; }
    const LevelManager& getLevelManager() const { return levelManager_; }

    AssetManager& getAssetManager() { return assetManager_; }
    const AssetManager& getAssetManager() const { return assetManager_; }

    GameState getState() const { return state_; }

private:
    void initializeLevels();
    bool loadLevel(Level& level);
    void loadFallbackLevel();
    void updateGameplay(float deltaTime);
    void setMenuVisible(bool visible);
    bool isMenuVisible() const;

    GameState state_ = GameState::Uninitialized;
    std::unique_ptr<Engine> engine_;
    std::unique_ptr<PlayerController> playerController_;
    LevelManager levelManager_;
    AssetManager assetManager_;
    SceneFactory::PauseMenuBindings pauseMenu_{};
    std::optional<size_t> pendingLevelIndex_;
    bool escWasDown_ = false;
};

} // namespace ge