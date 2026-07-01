#pragma once

#include "game/Level.h"
#include "game/LevelManager.h"

namespace ge {

class LevelBuilder {
public:
    static void registerDefaultLevels(LevelManager& levelManager);
};

} // namespace ge
