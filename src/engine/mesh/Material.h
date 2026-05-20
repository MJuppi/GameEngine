#pragma once

// =============================================================================
// Material.h — Wavefront .mtl material properties
// =============================================================================
// MTL files define surface appearance: diffuse (Kd), specular (Ks), shininess
// (Ns), etc. The GPU receives a packed GpuMaterial array in a uniform buffer.
// =============================================================================

#include <cstdint>
#include <string>
#include <vector>

namespace ge {

/// Parsed from a .mtl file (one newmtl block per instance).
struct Material {
    std::string name;
    float ambient[3] = {0.2f, 0.2f, 0.2f};
    float diffuse[3] = {0.8f, 0.8f, 0.8f};
    float specular[3] = {0.5f, 0.5f, 0.5f};
    float shininess = 32.0f;
    float alpha = 1.0f;
};

/// Maximum materials in one draw (shader uniform array size).
inline constexpr uint32_t kMaxGpuMaterials = 16;

/// std140 layout — must match GLSL struct Material in basic.frag.
struct GpuMaterial {
    float diffuse[4] = {0.8f, 0.8f, 0.8f, 1.0f};  // rgb + alpha
    float specular[4] = {0.5f, 0.5f, 0.5f, 32.0f}; // rgb + Ns
};

/// GPU uniform block: array of 16 materials (std140, 512 bytes).
struct MaterialBufferObject {
    GpuMaterial materials[kMaxGpuMaterials]{};
};

[[nodiscard]] Material makeDefaultMaterial(const std::string& name = "default");

void fillMaterialBuffer(const std::vector<Material>& materials, MaterialBufferObject& out);

} // namespace ge
