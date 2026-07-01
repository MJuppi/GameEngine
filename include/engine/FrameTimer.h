#pragma once

#include <chrono>

namespace ge {

class FrameTimer {
public:
    FrameTimer() = default;

    void reset();
    float beginFrame();

    [[nodiscard]] float deltaTime() const noexcept { return m_deltaTime; }

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

    TimePoint m_lastFrameTime{};
    bool m_isFirstFrame = true;
    float m_deltaTime = 0.0f;
};

} // namespace ge
