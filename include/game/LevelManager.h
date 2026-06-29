#pragma once
#include "game/Level.h"
#include <vector>
#include <memory>
#include <string>

namespace ge {

class AssetManager;

class LevelManager {
public:
    LevelManager() = default;
    ~LevelManager();

    void addLevel(std::unique_ptr<Level> level);
    void addLevel(const std::string& name, const std::string& meshPath = {});

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