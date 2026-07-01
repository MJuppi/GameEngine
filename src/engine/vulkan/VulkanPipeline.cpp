#include "engine/vulkan/VulkanPipeline.h"
#include "engine/vulkan/VulkanBuffer.h"
#include "engine/vulkan/VulkanDevice.h"
#include "engine/vulkan/VulkanSwapchain.h"
#include "engine/math/Types.h"

#include <array>
#include <cstddef>
#include <stdexcept>

namespace {

struct UiVertex {
    float position[2];
    float color[4];
};

} // namespace

namespace ge {

VulkanPipeline::VulkanPipeline(
    VulkanDevice& device,
    VulkanSwapchain& swapchain,
    const std::string& vertSpvPath,
    const std::string& fragSpvPath,
    bool useUi)
    : m_device(device)
    , m_vertPath(vertSpvPath)
    , m_fragPath(fragSpvPath)
    , m_useUi(useUi)
{
    if (!m_useUi) {
        createDescriptorSetLayout();
    }
    createGraphicsPipeline(swapchain);
}

VulkanPipeline::~VulkanPipeline() {
    destroyPipeline();
    if (m_descriptorSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(m_device.logical(), m_descriptorSetLayout, nullptr);
    }
}

void VulkanPipeline::recreate(VulkanSwapchain& swapchain) {
    destroyPipeline();
    createGraphicsPipeline(swapchain);
}

VkVertexInputBindingDescription VulkanPipeline::vertexBindingDescription() {
    // Describe how vertex data is laid out in memory: stride and input rate.
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    binding.stride = m_useUi ? sizeof(UiVertex) : sizeof(Vertex);
    return binding;
}

std::vector<VkVertexInputAttributeDescription> VulkanPipeline::vertexAttributeDescriptions() {
    if (m_useUi) {
        std::vector<VkVertexInputAttributeDescription> attrs(2);
        attrs[0].binding = 0;
        attrs[0].location = 0;
        attrs[0].format = VK_FORMAT_R32G32_SFLOAT;
        attrs[0].offset = offsetof(UiVertex, position);

        attrs[1].binding = 0;
        attrs[1].location = 1;
        attrs[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
        attrs[1].offset = offsetof(UiVertex, color);
        return attrs;
    }

    std::vector<VkVertexInputAttributeDescription> attrs(3);
    attrs[0].binding = 0;
    attrs[0].location = 0;
    attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attrs[0].offset = offsetof(Vertex, position);

    attrs[1].binding = 0;
    attrs[1].location = 1;
    attrs[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attrs[1].offset = offsetof(Vertex, normal);

    attrs[2].binding = 0;
    attrs[2].location = 2;
    attrs[2].format = VK_FORMAT_R32_UINT;
    attrs[2].offset = offsetof(Vertex, materialIndex);

    return attrs;
}

void VulkanPipeline::createDescriptorSetLayout() {
    if (m_useUi) {
        return;
    }

    // Create a descriptor set layout describing two uniform buffers:
    // binding 0 = scene UBO, binding 1 = material UBO.
    VkDescriptorSetLayoutBinding bindings[2]{};

    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(m_device.logical(), &layoutInfo, nullptr, &m_descriptorSetLayout) !=
        VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor set layout");
    }
}

VkShaderModule VulkanPipeline::createShaderModule(VulkanDevice& device, const std::vector<char>& code) {
    // Create a VkShaderModule from SPIR-V bytecode.
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    VkShaderModule module;
    if (vkCreateShaderModule(device.logical(), &createInfo, nullptr, &module) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create shader module");
    }
    return module;
}

void VulkanPipeline::createGraphicsPipeline(VulkanSwapchain& swapchain) {
    // Load SPIR-V, create shader modules and assemble the fixed-function pipeline state.
    auto vertCode = readSpvFile(m_vertPath);
    auto fragCode = readSpvFile(m_fragPath);

    VkShaderModule vertModule = createShaderModule(m_device, vertCode);
    VkShaderModule fragModule = createShaderModule(m_device, fragCode);

    VkPipelineShaderStageCreateInfo vertStage{};
    vertStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertModule;
    vertStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragStage{};
    fragStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragModule;
    fragStage.pName = "main";

    VkPipelineShaderStageCreateInfo stages[] = {vertStage, fragStage};

    auto bindingDescription = vertexBindingDescription();
    auto attributeDescriptions = vertexAttributeDescriptions();

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &bindingDescription;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
    vertexInput.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(swapchain.extent().width);
    viewport.height = static_cast<float>(swapchain.extent().height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = swapchain.extent();

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = m_useUi ? VK_CULL_MODE_NONE : VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;

    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    if (m_useUi) {
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
    }

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = m_useUi ? VK_FALSE : VK_TRUE;
    depthStencil.depthWriteEnable = m_useUi ? VK_FALSE : VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = m_useUi ? 0 : 1;
    pipelineLayoutInfo.pSetLayouts = m_useUi ? nullptr : &m_descriptorSetLayout;

    if (vkCreatePipelineLayout(m_device.logical(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) !=
        VK_SUCCESS) {
        throw std::runtime_error("Failed to create pipeline layout");
    }

    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = stages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.layout = m_pipelineLayout;
    pipelineInfo.renderPass = swapchain.renderPass();
    pipelineInfo.subpass = 0;

    if (vkCreateGraphicsPipelines(
            m_device.logical(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipeline) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create graphics pipeline");
    }

    vkDestroyShaderModule(m_device.logical(), fragModule, nullptr);
    vkDestroyShaderModule(m_device.logical(), vertModule, nullptr);
}

void VulkanPipeline::destroyPipeline() {
    if (m_pipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(m_device.logical(), m_pipeline, nullptr);
        m_pipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(m_device.logical(), m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
}

} // namespace ge
