#include "game/LevelManager.h"
#include "engine/asset/AssetManager.h"

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

// ====================== Getters ======================

Level* LevelManager::getCurrentLevel() const {
    if (currentLevelIndex_ < 0 || currentLevelIndex_ >= static_cast<int>(levels_.size())) {
        return nullptr;
    }
    return levels_[currentLevelIndex_].get();
}

Level* LevelManager::getLevel(const std::string& name) const {
    for (const auto& level : levels_) {
        if (level && level->getName() == name) {
            return level.get();
        }
    }
    return nullptr;
}

Level* LevelManager::getLevel(size_t index) const {
    if (index >= levels_.size()) return nullptr;
    return levels_[index].get();
}

// ====================== Setters ======================

bool LevelManager::setCurrentLevel(const std::string& name) {
    for (int i = 0; i < static_cast<int>(levels_.size()); ++i) {
        if (levels_[i] && levels_[i]->getName() == name) {
            currentLevelIndex_ = i;
            return true;
        }
    }
    return false;
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
