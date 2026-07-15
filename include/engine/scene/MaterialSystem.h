#pragma once

#include "engine/mesh/Material.h"
#include "engine/scene/ShaderEffect.h"
#include <map>
#include <string>
#include <memory>

namespace ge {

class VulkanDevice;
class VulkanSwapchain;

/**
 * @brief Manages ShaderEffects and creates Materials with assigned effects.
 */
class MaterialSystem {
public:
    MaterialSystem(VulkanDevice& device, VulkanSwapchain& swapchain);
    ~MaterialSystem();

    /// Returns a cached effect or creates a new one if it doesn't exist.
    std::shared_ptr<ShaderEffect> getOrCreateEffect(const std::string& name, const ShaderEffect::Config& config);

    /// Creates a material with default settings and optional effect.
    Material createMaterial(const std::string& name, std::shared_ptr<ShaderEffect> effect = nullptr);

    /// Helper to create a basic lit material with a solid color.
    Material createLitMaterial(const std::string& name, const glm::vec3& color, float shininess = 32.0f);

    /// Helper to create a material with a texture.
    Material createTexturedMaterial(const std::string& name, const std::string& texturePath, float shininess = 32.0f);

    /// Returns the default lit effect.
    [[nodiscard]] std::shared_ptr<ShaderEffect> getDefaultEffect() const { return defaultEffect_; }

private:
    VulkanDevice& device_;
    VulkanSwapchain& swapchain_;
    std::map<std::string, std::shared_ptr<ShaderEffect>> effects_;
    std::shared_ptr<ShaderEffect> defaultEffect_;
};

} // namespace ge
