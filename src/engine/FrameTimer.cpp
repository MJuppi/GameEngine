#include "engine/FrameTimer.h"

namespace ge {

void FrameTimer::reset() {
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
    m_isFirstFrame = true;
    m_deltaTime = 0.0f;
}

float FrameTimer::beginFrame() {
    const auto now = std::chrono::high_resolution_clock::now();

    if (m_isFirstFrame) {
        m_deltaTime = 1.0f / 60.0f;
        m_isFirstFrame = false;
    } else {
        const auto duration = std::chrono::duration_cast<std::chrono::duration<float>>(now - m_lastFrameTime);
        m_deltaTime = duration.count();
    }

    m_lastFrameTime = now;
    return m_deltaTime;
}

} // namespace ge
