#include "game/LevelBuilder.h"
#include "game/SceneFactory.h"
#include <memory>

namespace ge {

void LevelBuilder::registerDefaultLevels(LevelManager& levelManager) {
    auto testLevel = std::make_unique<Level>("TestCube");
    SceneFactory::configureTestLevel(*testLevel);
    levelManager.addLevel(std::move(testLevel));

    auto constraintLevel = std::make_unique<Level>("ConstraintParity");
    SceneFactory::configureConstraintParityLevel(*constraintLevel);
    levelManager.addLevel(std::move(constraintLevel));

    auto pairOrderingLevel = std::make_unique<Level>("PairOrderingParity");
    SceneFactory::configurePairOrderingLevel(*pairOrderingLevel);
    levelManager.addLevel(std::move(pairOrderingLevel));

    // Add more levels here as needed:
    // auto level2 = std::make_unique<Level>("Level2");
    // SceneFactory::configureLevel2(*level2);
    // levelManager.addLevel(std::move(level2));
}

} // namespace ge
