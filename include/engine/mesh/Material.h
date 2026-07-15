#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace ge {

class ShaderEffect;

struct Material {
    std::string name;
    glm::vec3 diffuse = {0.8f, 0.8f, 0.8f};
    glm::vec3 specular = {0.5f, 0.5f, 0.5f};
    float shininess = 32.0f;
    float alpha = 1.0f;
    std::string texturePath;

    std::shared_ptr<ShaderEffect> effect;
};

inline constexpr uint32_t kMaxGpuMaterials = 16;

struct GpuMaterial {
    glm::vec4 diffuse = {0.8f, 0.8f, 0.8f, 1.0f};
    glm::vec4 specular = {0.5f, 0.5f, 0.5f, 32.0f};
    uint32_t hasTexture = 0;
    float _pad[3] = {0.0f, 0.0f, 0.0f};
};

struct MaterialBufferObject {
    GpuMaterial materials[kMaxGpuMaterials]{};
};

[[nodiscard]] Material makeDefaultMaterial(const std::string& name = "default");
void fillMaterialBuffer(const std::vector<Material>& materials, MaterialBufferObject& out);

} // namespace ge
