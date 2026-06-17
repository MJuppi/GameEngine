#pragma once

#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"
#include "LevelManager.h"
#include <memory>

namespace ge {

enum class GameState {
    Uninitialized,
    Loading,
    Running,
    Paused,
    Shutdown
};

class Game {
public:
    Game();
    ~Game();
    
    // Lifecycle
    void initialize();
    void run();
    void shutdown();
    
    // State management
    GameState getState() const { return state_; }
    void setState(GameState state) { state_ = state; }
    
    // Level management
    LevelManager& getLevelManager() { return levelManager_; }
    const LevelManager& getLevelManager() const { return levelManager_; }
    
    // Engine access
    Engine* getEngine() { return engine_.get(); }
    const Engine* getEngine() const { return engine_.get(); }
    
    // Physics engine access
    PhysicsEngine& getPhysicsEngine() { return engine_->getPhysicsEngine(); }
    const PhysicsEngine& getPhysicsEngine() const { return engine_->getPhysicsEngine(); }
    
private:
    void initializeLevels();
    
    GameState state_;
    LevelManager levelManager_;
    std::unique_ptr<Engine> engine_;
};

}  // namespace ge
