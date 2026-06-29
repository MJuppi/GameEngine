#pragma once

namespace ge {

enum class GameState {
    Uninitialized,
    Loading,
    Running,
    Paused,
    Shutdown
};

} // namespace ge