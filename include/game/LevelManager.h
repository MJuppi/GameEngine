#pragma once

#include "Level.h"
#include <memory>
#include <vector>
#include <string>

namespace ge {

class LevelManager {
public:
    LevelManager();
    ~LevelManager();
    
    // Level management
    void addLevel(const std::shared_ptr<Level>& level);
    std::shared_ptr<Level> getLevel(size_t index) const;
    std::shared_ptr<Level> getLevelByName(const std::string& name) const;
    std::shared_ptr<Level> getCurrentLevel() const;
    
    // Level transitions
    bool setCurrentLevel(size_t index);
    bool setCurrentLevel(const std::string& name);
    bool nextLevel();
    bool previousLevel();
    
    // Level count
    size_t getLevelCount() const { return levels_.size(); }
    int getCurrentLevelIndex() const { return currentLevelIndex_; }
    
    // Batch operations
    void loadAllLevels();
    void unloadAllLevels();
    
private:
    std::vector<std::shared_ptr<Level>> levels_;
    int currentLevelIndex_;
};

}  // namespace ge
