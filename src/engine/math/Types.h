#pragma once

// =============================================================================
// Types.h — Vertex layout shared by CPU and GPU
// =============================================================================

#include <cstdint>

namespace ge {

/// One vertex: position, shading normal, index into the material uniform array.
struct Vertex {
    float position[3];
    float normal[3];
    uint32_t materialIndex = 0;
    float _pad = 0.0f;
};

} // namespace ge
