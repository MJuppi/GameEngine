#include "game/LevelBuilder.h"

#include <glm/gtc/matrix_transform.hpp>

namespace ge {

void LevelBuilder::registerDefaultLevels(LevelManager& levelManager) {
    levelManager.addLevel("TestCube", "assets/models/TestCube.obj", [](Level& level) {
        level.getPointLight().position = {0.0f, 2.0f, 0.0f, 1.0f};
        level.getPointLight().color = {1.0f, 0.95f, 0.8f, 1.0f};
        level.getPointLight().parameters = {1.0f, 0.09f, 0.032f, 8.0f};

        level.addObject("TestCube",
                        glm::mat4(1.0f),
                        {0.0f, 0.0f, 0.0f},
                        {0.5f, 0.5f, 0.5f},
                        RigidBodyProps{1.0f, 0.3f, 0.7f},
                        "assets/models/TestCube.obj");
        
        level.addObject("TestCube2",
                        glm::mat4(1.0f),
                        {0.0f, 0.0f, 5.0f},
                        {0.5f, 0.5f, 0.5f},
                        RigidBodyProps{1.0f, 0.3f, 0.7f},
                        "assets/models/TestCube.obj");

        RigidBodyProps groundProps;
        groundProps.mass = 0.0f;
        groundProps.isKinematic = true;
        groundProps.useGravity = false;

        level.addObject("Ground",
                        glm::translate(glm::mat4(1.0f), {0.0f, -2.0f, 0.0f}),
                        {50.0f, 0.5f, 50.0f},
                        {0.0f, 0.0f, 0.0f},
                        groundProps);
    });

    // Add more levels here as needed:
    // levelManager.addLevel("Level2", "assets/models/Level2.obj", [](Level& level) {
    //     level.addObject("Platform", glm::mat4(1.0f), {1.0f, 1.0f, 1.0f});
    // });
}

} // namespace ge
