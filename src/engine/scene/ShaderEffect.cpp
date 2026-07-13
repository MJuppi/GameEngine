#include "engine/scene/ShaderEffect.h"
#include "engine/vulkan/VulkanPipeline.h"

namespace ge {

ShaderEffect::ShaderEffect(VulkanDevice& device, VulkanSwapchain& swapchain, const Config& config)
    : config_(config) {
    // Note: We need to adapt VulkanPipeline to accept these configuration flags.
    // For now, we use the paths. The 'transparent' flag can map to the 'useUi'
    // flag in VulkanPipeline as a temporary measure since useUi enables blending.
    pipeline_ = std::make_unique<VulkanPipeline>(
        device,
        swapchain,
        config.vertexShaderPath,
        config.fragmentShaderPath,
        config.transparent);
}

ShaderEffect::~ShaderEffect() = default;

} // namespace ge
