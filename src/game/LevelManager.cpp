#include "game/LevelManager.h"
#include <algorithm>
#include <iostream>

namespace ge {

LevelManager::LevelManager()
    : currentLevelIndex_(-1) {
}

LevelManager::~LevelManager() {
    unloadAllLevels();
}

void LevelManager::addLevel(const std::shared_ptr<Level>& level) {
    if (!level) {
        return;
    }
    
    levels_.push_back(level);
    
    // Auto-select first level
    if (currentLevelIndex_ < 0) {
        currentLevelIndex_ = 0;
    }
}

std::shared_ptr<Level> LevelManager::getLevel(size_t index) const {
    if (index >= levels_.size()) {
        return nullptr;
    }
    return levels_[index];
}

std::shared_ptr<Level> LevelManager::getLevelByName(const std::string& name) const {
    auto it = std::find_if(levels_.begin(), levels_.end(),
        [&name](const std::shared_ptr<Level>& level) {
            return level && level->getName() == name;
        });
    
    return (it != levels_.end()) ? *it : nullptr;
}

std::shared_ptr<Level> LevelManager::getCurrentLevel() const {
    if (currentLevelIndex_ < 0 || currentLevelIndex_ >= static_cast<int>(levels_.size())) {
        return nullptr;
    }
    return levels_[currentLevelIndex_];
}

bool LevelManager::setCurrentLevel(size_t index) {
    if (index >= levels_.size()) {
        return false;
    }
    currentLevelIndex_ = static_cast<int>(index);
    return true;
}

bool LevelManager::setCurrentLevel(const std::string& name) {
    auto level = getLevelByName(name);
    if (!level) {
        return false;
    }
    
    auto it = std::find(levels_.begin(), levels_.end(), level);
    currentLevelIndex_ = static_cast<int>(std::distance(levels_.begin(), it));
    return true;
}

bool LevelManager::nextLevel() {
    return setCurrentLevel(currentLevelIndex_ + 1);
}

bool LevelManager::previousLevel() {
    return setCurrentLevel(currentLevelIndex_ - 1);
}

void LevelManager::loadAllLevels() {
    for (auto& level : levels_) {
        if (level) {
            level->load();
        }
    }
}

void LevelManager::unloadAllLevels() {
    for (auto& level : levels_) {
        if (level) {
            level->unload();
        }
    }
}

}  // namespace ge
