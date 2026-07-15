#pragma once

// =============================================================================
// VulkanPipeline.h — Graphics pipeline + descriptor layout
// =============================================================================
// The pipeline bundles shader stages, vertex input, rasterization, blending, etc.
// Descriptor set layout tells Vulkan how shaders access buffers (e.g. MVP matrix).
// =============================================================================

#include <vulkan/vulkan.h>
#include <string>
#include <vector>

namespace ge {

class VulkanDevice;
class VulkanSwapchain;

class VulkanPipeline {
public:
    struct PipelineConfig {
        std::string vertPath;
        std::string fragPath;
        bool depthTest = true;
        bool depthWrite = true;
        bool blendEnable = false;
        bool wireframe = false;
        VkCullModeFlags cullMode = VK_CULL_MODE_BACK_BIT;
    };

    VulkanPipeline(VulkanDevice& device, VulkanSwapchain& swapchain, const PipelineConfig& config);
    ~VulkanPipeline();

    VulkanPipeline(const VulkanPipeline&) = delete;
    VulkanPipeline& operator=(const VulkanPipeline&) = delete;

    void recreate(VulkanSwapchain& swapchain);

    [[nodiscard]] VkPipeline handle() const { return m_pipeline; }
    [[nodiscard]] VkPipelineLayout layout() const { return m_pipelineLayout; }
    [[nodiscard]] VkDescriptorSetLayout descriptorSetLayout() const {
        return m_descriptorSetLayout;
    }

    /// Describes how vertex buffer data maps to vertex shader inputs.
    VkVertexInputBindingDescription vertexBindingDescription();
    std::vector<VkVertexInputAttributeDescription> vertexAttributeDescriptions();

private:
    void createDescriptorSetLayout();
    void createGraphicsPipeline(VulkanSwapchain& swapchain);
    void destroyPipeline();

    static VkShaderModule createShaderModule(VulkanDevice& device, const std::vector<char>& code);

    VulkanDevice& m_device;
    PipelineConfig m_config;

    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_pipeline = VK_NULL_HANDLE;
};

} // namespace ge
