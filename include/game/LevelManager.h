#pragma once
#include "game/Level.h"
#include "engine/scene/ObjectBuilder.h"
#include <functional>
#include <vector>
#include <memory>
#include <string>

namespace ge {

class AssetManager;

class LevelManager {
public:
    using LevelConfigurator = std::function<void(Level&)>;

    LevelManager() = default;
    ~LevelManager();

    Level* addLevel(std::unique_ptr<Level> level);
    Level* addLevel(const std::string& name, const std::string& meshPath = {});
    Level* addLevel(const std::string& name, const std::string& meshPath, const PhysicsMeshObject& object);
    Level* addLevel(const std::string& name, const std::string& meshPath, LevelConfigurator configurator);

    // === Recommended: Return raw pointer (non-owning) ===
    Level* getCurrentLevel() const;
    Level* getLevel(const std::string& name) const;
    Level* getLevel(size_t index) const;

    bool setCurrentLevel(const std::string& name);
    bool setCurrentLevel(size_t index);
    bool nextLevel();
    bool previousLevel();

    void loadAll(AssetManager& assetManager);
    void unloadAll();

    size_t getLevelCount() const { return levels_.size(); }

private:
    std::vector<std::unique_ptr<Level>> levels_;
    int currentLevelIndex_ = -1;
};

} // namespace ge