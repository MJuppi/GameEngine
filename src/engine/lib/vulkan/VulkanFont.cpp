#include "engine/vulkan/VulkanFont.h"
#include "engine/vulkan/VulkanBuffer.h"
#include "engine/vulkan/VulkanDevice.h"
#include "engine/asset/FontLoader.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>

namespace {

// Convert pixel coordinates to NDC
glm::vec2 pixelToNdc(float x, float y, uint32_t width, uint32_t height) {
    return {
        (x / static_cast<float>(width)) * 2.0f - 1.0f,
        (y / static_cast<float>(height)) * 2.0f - 1.0f,
    };
}

} // namespace

ge::VulkanFont::VulkanFont(VulkanDevice& device, VkCommandPool commandPool, VkQueue queue)
    : m_device(&device), m_commandPool(commandPool), m_queue(queue) {
    createDescriptorSetLayout(device);
}

ge::VulkanFont::~VulkanFont() {
    if (m_device) {
        destroyTextureResources(*m_device);

        if (m_descriptorSetLayout != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(m_device->logical(), m_descriptorSetLayout, nullptr);
        }
    }
}

void ge::VulkanFont::loadFont(
    const std::string& path,
    float fontHeight,
    uint32_t glyphRangeStart,
    uint32_t glyphRangeEnd) {

    // Load font data
    m_fontData = FontLoader::loadFont(path, fontHeight, glyphRangeStart, glyphRangeEnd);

    // Create GPU resources
    createTextureImage(*m_device, m_commandPool, m_queue);
    createTextureImageView(*m_device);
    createTextureSampler(*m_device);
}

void ge::VulkanFont::loadFontWithGlyphs(
    const std::string& path,
    float fontHeight,
    const std::vector<uint32_t>& glyphs) {

    // Load font data
    m_fontData = FontLoader::loadFontWithGlyphs(path, fontHeight, glyphs);

    // Create GPU resources
    createTextureImage(*m_device, m_commandPool, m_queue);
    createTextureImageView(*m_device);
    createTextureSampler(*m_device);
}

const ge::GlyphMetrics* ge::VulkanFont::findGlyph(uint32_t codepoint) const {
    auto it = std::lower_bound(
        m_fontData.glyphs.begin(),
        m_fontData.glyphs.end(),
        codepoint,
        [](const GlyphMetrics& glyph, uint32_t cp) {
            return glyph.codepoint < cp;
        });

    if (it != m_fontData.glyphs.end() && it->codepoint == codepoint) {
        return &(*it);
    }

    return nullptr;
}

float ge::VulkanFont::getTextWidth(const std::string& text) const {
    float width = 0.0f;

    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const GlyphMetrics* glyph = findGlyph(static_cast<uint32_t>(c));
        if (glyph) {
            width += glyph->advance;
        }
    }

    return width;
}

float ge::VulkanFont::getTextHeight() const {
    return m_fontData.metrics.lineHeight;
}

std::vector<ge::FontVertex> ge::VulkanFont::createTextVertices(
    const std::string& text,
    float x,
    float y,
    glm::vec4 color,
    uint32_t screenWidth,
    uint32_t screenHeight) const {

    std::vector<FontVertex> vertices;

    float cursorX = x;
    float cursorY = y;

    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const GlyphMetrics* glyph = findGlyph(static_cast<uint32_t>(c));

        if (!glyph) {
            continue;
        }

        // Calculate glyph position
        float left = cursorX + glyph->bearingX;
        float top = cursorY + glyph->bearingY - m_fontData.metrics.ascender;
        float right = left + glyph->width;
        float bottom = top + glyph->height;

        // Convert to NDC
        const glm::vec2 tl = pixelToNdc(left, top, screenWidth, screenHeight);
        const glm::vec2 bl = pixelToNdc(left, bottom, screenWidth, screenHeight);
        const glm::vec2 br = pixelToNdc(right, bottom, screenWidth, screenHeight);
        const glm::vec2 tr = pixelToNdc(right, top, screenWidth, screenHeight);

        // Create vertices with texture coordinates
        // Note: Vulkan uses Y-down coordinate system for textures
        vertices.push_back({tl, {glyph->uvLeft, glyph->uvTop}, color});    // Top-left
        vertices.push_back({bl, {glyph->uvLeft, glyph->uvBottom}, color}); // Bottom-left
        vertices.push_back({br, {glyph->uvRight, glyph->uvBottom}, color}); // Bottom-right

        vertices.push_back({tl, {glyph->uvLeft, glyph->uvTop}, color});    // Top-left
        vertices.push_back({br, {glyph->uvRight, glyph->uvBottom}, color}); // Bottom-right
        vertices.push_back({tr, {glyph->uvRight, glyph->uvTop}, color});   // Top-right

        // Advance cursor
        cursorX += glyph->advance;
    }

    return vertices;
}

void ge::VulkanFont::createDescriptorSetLayout(VulkanDevice& device) {
    VkDescriptorSetLayoutBinding uboLayoutBinding{};
    uboLayoutBinding.binding = 0;
    uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBinding.descriptorCount = 1;
    uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    uboLayoutBinding.pImmutableSamplers = nullptr;

    VkDescriptorSetLayoutBinding samplerLayoutBinding{};
    samplerLayoutBinding.binding = 1;
    samplerLayoutBinding.descriptorCount = 1;
    samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    samplerLayoutBinding.pImmutableSamplers = nullptr;
    samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::array<VkDescriptorSetLayoutBinding, 2> bindings = {
        uboLayoutBinding, samplerLayoutBinding};

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(device.logical(), &layoutInfo, nullptr, &m_descriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create font descriptor set layout");
    }
}

std::vector<VkDescriptorSet> ge::VulkanFont::createDescriptorSets(
    VkDescriptorPool descriptorPool,
    int setCount) const {

    std::vector<VkDescriptorSetLayout> layouts(setCount, m_descriptorSetLayout);
    
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(setCount);
    allocInfo.pSetLayouts = layouts.data();

    std::vector<VkDescriptorSet> descriptorSets(setCount);
    if (vkAllocateDescriptorSets(m_device->logical(), &allocInfo, descriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate font descriptor sets");
    }

    // Update descriptor sets with font texture info
    for (int i = 0; i < setCount; ++i) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imageInfo.imageView = m_textureImageView;
        imageInfo.sampler = m_textureSampler;

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = descriptorSets[i];
        descriptorWrite.dstBinding = 1; // Font atlas texture is at binding 1
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pImageInfo = &imageInfo;

        vkUpdateDescriptorSets(m_device->logical(), 1, &descriptorWrite, 0, nullptr);
    }

    return descriptorSets;
}

std::vector<VkDescriptorSet> ge::VulkanFont::createDescriptorSetsWithLayout(
    VkDescriptorPool descriptorPool,
    VkDescriptorSetLayout layout,
    int setCount) const {

    std::vector<VkDescriptorSetLayout> layouts(setCount, layout);
    
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(setCount);
    allocInfo.pSetLayouts = layouts.data();

    std::vector<VkDescriptorSet> descriptorSets(setCount);
    if (vkAllocateDescriptorSets(m_device->logical(), &allocInfo, descriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate font descriptor sets");
    }

    // Update descriptor sets with font texture info
    for (int i = 0; i < setCount; ++i) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imageInfo.imageView = m_textureImageView;
        imageInfo.sampler = m_textureSampler;

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = descriptorSets[i];
        descriptorWrite.dstBinding = 1; // Font atlas texture is at binding 1
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pImageInfo = &imageInfo;

        vkUpdateDescriptorSets(m_device->logical(), 1, &descriptorWrite, 0, nullptr);
    }

    return descriptorSets;
}

void ge::VulkanFont::createTextureImage(VulkanDevice& device, VkCommandPool commandPool, VkQueue queue) {
    if (m_fontData.atlasPixels.empty() || m_fontData.atlasWidth == 0 || m_fontData.atlasHeight == 0) {
        throw std::runtime_error("FontLoader: No atlas data available for texture creation");
    }

    const VkDeviceSize imageSize = static_cast<VkDeviceSize>(m_fontData.atlasWidth) * 
                                   static_cast<VkDeviceSize>(m_fontData.atlasHeight) * 4; // RGBA = 4 bytes per pixel

    // Convert grayscale atlas to RGBA
    // For font rendering, use intensity as alpha channel and white for RGB
    std::vector<uint8_t> rgbaPixels(m_fontData.atlasWidth * m_fontData.atlasHeight * 4);
    for (int y = 0; y < m_fontData.atlasHeight; ++y) {
        for (int x = 0; x < m_fontData.atlasWidth; ++x) {
            uint8_t intensity = m_fontData.atlasPixels[y * m_fontData.atlasWidth + x];
            int dstIdx = (y * m_fontData.atlasWidth + x) * 4;
            rgbaPixels[dstIdx + 0] = 255;       // R (white)
            rgbaPixels[dstIdx + 1] = 255;       // G
            rgbaPixels[dstIdx + 2] = 255;       // B
            rgbaPixels[dstIdx + 3] = intensity; // A (glyph mask)
        }
    }

    // Create staging buffer
    VulkanBuffer stagingBuffer;
    stagingBuffer.create(
        device,
        imageSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    // Copy RGBA pixel data to staging buffer
    VulkanBuffer::write(
        device,
        stagingBuffer.handle(),
        stagingBuffer.memory(),
        rgbaPixels.data(),
        imageSize);

    // Create texture image
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = static_cast<uint32_t>(m_fontData.atlasWidth);
    imageInfo.extent.height = static_cast<uint32_t>(m_fontData.atlasHeight);
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(device.logical(), &imageInfo, nullptr, &m_textureImage) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create font texture image");
    }

    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(device.logical(), m_textureImage, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = device.findMemoryType(
        memRequirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (vkAllocateMemory(device.logical(), &allocInfo, nullptr, &m_textureImageMemory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate font texture image memory");
    }

    vkBindImageMemory(device.logical(), m_textureImage, m_textureImageMemory, 0);

    // Copy from staging buffer to image
    VkCommandBuffer commandBuffer;
    VkCommandBufferAllocateInfo allocInfoCB{};
    allocInfoCB.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfoCB.commandPool = commandPool;
    allocInfoCB.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfoCB.commandBufferCount = 1;

    if (vkAllocateCommandBuffers(device.logical(), &allocInfoCB, &commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate command buffer for font texture upload");
    }

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    // Transition image layout to transfer destination
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = m_textureImage;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0,
        nullptr,
        0,
        nullptr,
        1,
        &barrier);

    // Copy buffer to image
    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {
        static_cast<uint32_t>(m_fontData.atlasWidth),
        static_cast<uint32_t>(m_fontData.atlasHeight),
        1};

    vkCmdCopyBufferToImage(
        commandBuffer,
        stagingBuffer.handle(),
        m_textureImage,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1,
        &region);

    // Transition image layout to shader read only
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0,
        0,
        nullptr,
        0,
        nullptr,
        1,
        &barrier);

    vkEndCommandBuffer(commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(queue);

    vkFreeCommandBuffers(device.logical(), commandPool, 1, &commandBuffer);
    stagingBuffer.destroy(device);
}

void ge::VulkanFont::createTextureImageView(VulkanDevice& device) {
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = m_textureImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device.logical(), &viewInfo, nullptr, &m_textureImageView) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create font texture image view");
    }
}

void ge::VulkanFont::createTextureSampler(VulkanDevice& device) {
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.anisotropyEnable = VK_FALSE;
    samplerInfo.maxAnisotropy = 1.0f;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_WHITE;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = 1.0f;

    if (vkCreateSampler(device.logical(), &samplerInfo, nullptr, &m_textureSampler) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create font texture sampler");
    }
}

void ge::VulkanFont::destroyTextureResources(VulkanDevice& device) {
    if (m_textureSampler != VK_NULL_HANDLE) {
        vkDestroySampler(device.logical(), m_textureSampler, nullptr);
        m_textureSampler = VK_NULL_HANDLE;
    }

    if (m_textureImageView != VK_NULL_HANDLE) {
        vkDestroyImageView(device.logical(), m_textureImageView, nullptr);
        m_textureImageView = VK_NULL_HANDLE;
    }

    if (m_textureImage != VK_NULL_HANDLE) {
        vkDestroyImage(device.logical(), m_textureImage, nullptr);
        m_textureImage = VK_NULL_HANDLE;
    }

    if (m_textureImageMemory != VK_NULL_HANDLE) {
        vkFreeMemory(device.logical(), m_textureImageMemory, nullptr);
        m_textureImageMemory = VK_NULL_HANDLE;
    }
}

