#include "game/LevelManager.h"
#include "engine/asset/AssetManager.h"
#include <algorithm>

namespace ge {

LevelManager::~LevelManager() {
    unloadAll();
}

Level* LevelManager::addLevel(std::unique_ptr<Level> level) {
    if (!level) return nullptr;

    Level* rawLevel = level.get();
    levels_.push_back(std::move(level));

    if (currentLevelIndex_ < 0) {
        currentLevelIndex_ = 0;
    }

    return rawLevel;
}

Level* LevelManager::addLevel(const std::string& name, const std::string& meshPath) {
    return addLevel(std::make_unique<Level>(name, meshPath));
}

Level* LevelManager::addLevel(const std::string& name, const std::string& meshPath, const PhysicsMeshObject& object) {
    return addLevel(std::make_unique<Level>(name, meshPath, object));
}

Level* LevelManager::addLevel(const std::string& name, const std::string& meshPath, LevelConfigurator configurator) {
    auto level = std::make_unique<Level>(name, meshPath);
    if (configurator) {
        configurator(*level);
    }
    return addLevel(std::move(level));
}

// ====================== Getters ======================

Level* LevelManager::getCurrentLevel() const {
    if (currentLevelIndex_ < 0 || currentLevelIndex_ >= static_cast<int>(levels_.size())) {
        return nullptr;
    }
    return levels_[currentLevelIndex_].get();
}

Level* LevelManager::getLevel(const std::string& name) const {
    auto it = std::find_if(levels_.begin(), levels_.end(),
        [&](const auto& lvl) { return lvl && lvl->getName() == name; });
    
    return (it != levels_.end()) ? it->get() : nullptr;
}

Level* LevelManager::getLevel(size_t index) const {
    if (index >= levels_.size()) return nullptr;
    return levels_[index].get();
}

// ====================== Setters ======================

bool LevelManager::setCurrentLevel(const std::string& name) {
    auto level = getLevel(name);
    if (!level) return false;

    auto it = std::find_if(levels_.begin(), levels_.end(),
        [level](const auto& ptr) { return ptr.get() == level; });
    
    currentLevelIndex_ = static_cast<int>(std::distance(levels_.begin(), it));
    return true;
}

bool LevelManager::setCurrentLevel(size_t index) {
    if (index >= levels_.size()) return false;
    currentLevelIndex_ = static_cast<int>(index);
    return true;
}

bool LevelManager::nextLevel() {
    if (levels_.empty()) return false;
    currentLevelIndex_ = (currentLevelIndex_ + 1) % static_cast<int>(levels_.size());
    return true;
}

bool LevelManager::previousLevel() {
    if (levels_.empty()) return false;
    currentLevelIndex_ = (currentLevelIndex_ + static_cast<int>(levels_.size()) - 1) 
                          % static_cast<int>(levels_.size());
    return true;
}

void LevelManager::loadAll(AssetManager& assetManager) {
    for (auto& level : levels_) {
        if (level) level->load(assetManager);
    }
}

void LevelManager::unloadAll() {
    for (auto& level : levels_) {
        if (level) level->unload();
    }
}

} // namespace ge