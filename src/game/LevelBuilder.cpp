#include "game/LevelBuilder.h"
#include "game/SceneFactory.h"

namespace ge {

void LevelBuilder::registerDefaultLevels(LevelManager& levelManager) {
    levelManager.addLevel("TestCube", "assets/models/TestCubeTexture.obj", [](Level& level) {
        SceneFactory::configureTestLevel(level);
    });

    // Add more levels here as needed:
    // levelManager.addLevel("Level2", "assets/models/Level2.obj", [](Level& level) {
    //     SceneFactory::configureLevel2(level);
    // });
}

} // namespace ge
