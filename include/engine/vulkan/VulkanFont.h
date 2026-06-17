#pragma once

// =============================================================================
// VulkanFont.h — GPU font resources and text rendering
// =============================================================================
// Manages font atlas texture and provides text rendering capabilities.
// =============================================================================

#include "engine/asset/FontData.h"
#include <vulkan/vulkan.h>
#include <memory>
#include <string>
#include <vector>

namespace ge {

class VulkanDevice;
class VulkanBuffer;

class VulkanFont {
public:
    VulkanFont(VulkanDevice& device, VkCommandPool commandPool, VkQueue queue);
    ~VulkanFont();

    VulkanFont(const VulkanFont&) = delete;
    VulkanFont& operator=(const VulkanFont&) = delete;

    /// Load a font from a TTF file
    void loadFont(
        const std::string& path,
        float fontHeight = 32.0f,
        uint32_t glyphRangeStart = 32,
        uint32_t glyphRangeEnd = 126);

    /// Load a font with specific glyphs
    void loadFontWithGlyphs(
        const std::string& path,
        float fontHeight,
        const std::vector<uint32_t>& glyphs);

    /// Generate vertices for rendering text
    /// @param text The text to render
    /// @param x Starting X position in pixels
    /// @param y Starting Y position in pixels
    /// @param color Text color
    /// @param screenWidth Screen width in pixels
    /// @param screenHeight Screen height in pixels
    /// @return Vector of FontVertex for rendering
    [[nodiscard]] std::vector<FontVertex> createTextVertices(
        const std::string& text,
        float x,
        float y,
        glm::vec4 color,
        uint32_t screenWidth,
        uint32_t screenHeight) const;

    /// Get the font atlas texture view
    [[nodiscard]] VkImageView textureView() const { return m_textureImageView; }

    /// Get the font atlas sampler
    [[nodiscard]] VkSampler sampler() const { return m_textureSampler; }

    /// Get the descriptor set layout for font rendering
    [[nodiscard]] VkDescriptorSetLayout descriptorSetLayout() const { return m_descriptorSetLayout; }

    /// Create descriptor sets for font rendering
    /// @param descriptorPool The descriptor pool to allocate from
    /// @return Vector of descriptor sets (one per frame in flight)
    [[nodiscard]] std::vector<VkDescriptorSet> createDescriptorSets(
        VkDescriptorPool descriptorPool,
        int setCount = 2) const;

    /// Create descriptor sets for font rendering with a custom layout
    /// @param descriptorPool The descriptor pool to allocate from
    /// @param layout The descriptor set layout to use
    /// @param setCount Number of descriptor sets to create
    /// @return Vector of descriptor sets (one per frame in flight)
    [[nodiscard]] std::vector<VkDescriptorSet> createDescriptorSetsWithLayout(
        VkDescriptorPool descriptorPool,
        VkDescriptorSetLayout layout,
        int setCount = 2) const;

    /// Get the font metrics
    [[nodiscard]] const FontMetrics& metrics() const { return m_fontData.metrics; }

    /// Get the glyph for a codepoint (returns nullptr if not found)
    [[nodiscard]] const GlyphMetrics* findGlyph(uint32_t codepoint) const;

    /// Get text width for a given string
    [[nodiscard]] float getTextWidth(const std::string& text) const;

    /// Get text height for the font
    [[nodiscard]] float getTextHeight() const;

private:
    void createTextureImage(VulkanDevice& device, VkCommandPool commandPool, VkQueue queue);
    void createTextureImageView(VulkanDevice& device);
    void createTextureSampler(VulkanDevice& device);
    void createDescriptorSetLayout(VulkanDevice& device);

    void destroyTextureResources(VulkanDevice& device);

    VulkanDevice* m_device = nullptr;
    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    VkQueue m_queue = VK_NULL_HANDLE;
    FontData m_fontData;

    VkImage m_textureImage = VK_NULL_HANDLE;
    VkDeviceMemory m_textureImageMemory = VK_NULL_HANDLE;
    VkImageView m_textureImageView = VK_NULL_HANDLE;
    VkSampler m_textureSampler = VK_NULL_HANDLE;

    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
};

} // namespace ge
