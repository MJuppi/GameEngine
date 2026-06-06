// Game entry point using refactored Game and Level system.
#include "game/Game.h"
#include <iostream>

int main(int /*argc*/, char** /*argv*/) {
    using namespace ge;
    
    try {
        Game game;
        game.initialize();
        game.run();
        game.shutdown();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
}
