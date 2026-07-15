#include "engine/scene/ShaderEffect.h"
#include "engine/vulkan/VulkanPipeline.h"

namespace ge {

ShaderEffect::ShaderEffect(VulkanDevice& device, VulkanSwapchain& swapchain, const Config& config)
    : config_(config) {

    VulkanPipeline::PipelineConfig pipelineConfig;
    pipelineConfig.vertPath = config.vertexShaderPath;
    pipelineConfig.fragPath = config.fragmentShaderPath;
    pipelineConfig.depthTest = config.depthTest;
    pipelineConfig.depthWrite = config.depthWrite;
    pipelineConfig.blendEnable = config.transparent;
    pipelineConfig.wireframe = config.wireframe;

    switch (config.cullMode) {
        case Config::CullMode::None:  pipelineConfig.cullMode = VK_CULL_MODE_NONE; break;
        case Config::CullMode::Front: pipelineConfig.cullMode = VK_CULL_MODE_FRONT_BIT; break;
        case Config::CullMode::Back:  pipelineConfig.cullMode = VK_CULL_MODE_BACK_BIT; break;
        case Config::CullMode::Both:  pipelineConfig.cullMode = VK_CULL_MODE_FRONT_AND_BACK; break;
    }

    pipeline_ = std::make_unique<VulkanPipeline>(device, swapchain, pipelineConfig);
}

ShaderEffect::~ShaderEffect() = default;

} // namespace ge
