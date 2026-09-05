#include "game/LevelBuilder.h"
#include "game/SceneFactory.h"
#include <memory>

namespace ge {

void LevelBuilder::registerDefaultLevels(LevelManager& levelManager) {
    auto mission = std::make_unique<Level>("CombatMission");
    SceneFactory::configureCombatMission(*mission);
    levelManager.addLevel(std::move(mission));
}

} // namespace ge
