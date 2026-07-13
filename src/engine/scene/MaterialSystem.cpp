#include "engine/scene/MaterialSystem.h"

#ifndef SHADER_DIR
#define SHADER_DIR "shaders"
#endif

namespace ge {

MaterialSystem::MaterialSystem(VulkanDevice& device, VulkanSwapchain& swapchain)
    : device_(device), swapchain_(swapchain) {

    // Create a default lit effect
    ShaderEffect::Config defaultConfig;
    std::string shaderDir = SHADER_DIR;
    defaultConfig.vertexShaderPath = shaderDir + "/basic.vert.spv";
    defaultConfig.fragmentShaderPath = shaderDir + "/basic.frag.spv";
    defaultEffect_ = getOrCreateEffect("default_lit", defaultConfig);
}

MaterialSystem::~MaterialSystem() = default;

std::shared_ptr<ShaderEffect> MaterialSystem::getOrCreateEffect(const std::string& name, const ShaderEffect::Config& config) {
    auto it = effects_.find(name);
    if (it != effects_.end()) {
        return it->second;
    }

    auto effect = std::make_shared<ShaderEffect>(device_, swapchain_, config);
    effects_[name] = effect;
    return effect;
}

Material MaterialSystem::createMaterial(const std::string& name, std::shared_ptr<ShaderEffect> effect) {
    Material mat = makeDefaultMaterial(name);
    mat.effect = effect ? effect : defaultEffect_;
    return mat;
}

} // namespace ge
