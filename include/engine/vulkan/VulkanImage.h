#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include "engine/asset/TextureData.h"

namespace ge {

class VulkanDevice;

class VulkanImage {
public:
    VulkanImage() = default;
    ~VulkanImage();

    void create(
        VulkanDevice& device,
        uint32_t width,
        uint32_t height,
        VkFormat format,
        VkImageTiling tiling,
        VkImageUsageFlags usage,
        VkMemoryPropertyFlags properties);

    void destroy(VulkanDevice& device);

    void transitionLayout(
        VulkanDevice& device,
        VkCommandPool commandPool,
        VkQueue queue,
        VkImageLayout oldLayout,
        VkImageLayout newLayout);

    void copyFromBuffer(
        VulkanDevice& device,
        VkCommandPool commandPool,
        VkQueue queue,
        VkBuffer buffer,
        uint32_t width,
        uint32_t height);

    void createView(VulkanDevice& device, VkFormat format, VkImageAspectFlags aspectFlags);
    void createSampler(VulkanDevice& device);

    [[nodiscard]] VkImage handle() const { return m_image; }
    [[nodiscard]] VkImageView view() const { return m_view; }
    [[nodiscard]] VkSampler sampler() const { return m_sampler; }

    static VulkanImage fromTextureData(
        VulkanDevice& device,
        VkCommandPool commandPool,
        VkQueue queue,
        const TextureData& data);

private:
    VkImage m_image = VK_NULL_HANDLE;
    VkDeviceMemory m_memory = VK_NULL_HANDLE;
    VkImageView m_view = VK_NULL_HANDLE;
    VkSampler m_sampler = VK_NULL_HANDLE;
};

} // namespace ge
