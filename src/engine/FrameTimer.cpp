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

    // Update FPS
    m_fpsAccumulator += m_deltaTime;
    m_fpsFrameCount++;
    if (m_fpsAccumulator >= 1.0f) {
        m_fps = static_cast<float>(m_fpsFrameCount) / m_fpsAccumulator;
        m_fpsAccumulator = 0.0f;
        m_fpsFrameCount = 0;
    }

    return m_deltaTime;
}

} // namespace ge
