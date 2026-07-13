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

    std::shared_ptr<ShaderEffect> getOrCreateEffect(const std::string& name, const ShaderEffect::Config& config);

    // Creates a material and assigns a default effect if none is provided
    Material createMaterial(const std::string& name, std::shared_ptr<ShaderEffect> effect = nullptr);

private:
    VulkanDevice& device_;
    VulkanSwapchain& swapchain_;
    std::map<std::string, std::shared_ptr<ShaderEffect>> effects_;
    std::shared_ptr<ShaderEffect> defaultEffect_;
};

} // namespace ge
