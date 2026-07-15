#include "engine/vulkan/VulkanImage.h"
#include "engine/vulkan/VulkanDevice.h"
#include "engine/vulkan/VulkanBuffer.h"
#include <stdexcept>

namespace ge {

VulkanImage::~VulkanImage() {}

void VulkanImage::create(VulkanDevice& device, uint32_t w, uint32_t h, VkFormat f, VkImageTiling t, VkImageUsageFlags u, VkMemoryPropertyFlags p) {
    VkImageCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    info.imageType = VK_IMAGE_TYPE_2D;
    info.extent = {w, h, 1};
    info.mipLevels = 1;
    info.arrayLayers = 1;
    info.format = f;
    info.tiling = t;
    info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    info.usage = u;
    info.samples = VK_SAMPLE_COUNT_1_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(device.logical(), &info, nullptr, &m_image) != VK_SUCCESS) throw std::runtime_error("No image");
    VkMemoryRequirements mem; vkGetImageMemoryRequirements(device.logical(), m_image, &mem);
    VkMemoryAllocateInfo alloc{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, mem.size, device.findMemoryType(mem.memoryTypeBits, p) };
    if (vkAllocateMemory(device.logical(), &alloc, nullptr, &m_memory) != VK_SUCCESS) throw std::runtime_error("No memory");
    vkBindImageMemory(device.logical(), m_image, m_memory, 0);
}

void VulkanImage::destroy(VulkanDevice& device) {
    if (m_sampler) vkDestroySampler(device.logical(), m_sampler, nullptr);
    if (m_view) vkDestroyImageView(device.logical(), m_view, nullptr);
    if (m_image) vkDestroyImage(device.logical(), m_image, nullptr);
    if (m_memory) vkFreeMemory(device.logical(), m_memory, nullptr);
    m_sampler = 0; m_view = 0; m_image = 0; m_memory = 0;
}

void VulkanImage::transitionLayout(VulkanDevice& device, VkCommandPool pool, VkQueue queue, VkImageLayout oldL, VkImageLayout newL) {
    VkCommandBufferAllocateInfo aInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, pool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1 };
    VkCommandBuffer cb; vkAllocateCommandBuffers(device.logical(), &aInfo, &cb);

    VkCommandBufferBeginInfo bInfo{};
    bInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &bInfo);

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = oldL;
    barrier.newLayout = newL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = m_image;
    barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

    VkPipelineStageFlags src, dst;
    if (oldL == VK_IMAGE_LAYOUT_UNDEFINED && newL == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) { barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; src = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT; dst = VK_PIPELINE_STAGE_TRANSFER_BIT; }
    else if (oldL == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newL == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) { barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; src = VK_PIPELINE_STAGE_TRANSFER_BIT; dst = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT; }
    else throw std::runtime_error("No transition");
    vkCmdPipelineBarrier(cb, src, dst, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    vkEndCommandBuffer(cb);

    VkSubmitInfo sInfo{};
    sInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    sInfo.commandBufferCount = 1;
    sInfo.pCommandBuffers = &cb;

    if (vkQueueSubmit(queue, 1, &sInfo, VK_NULL_HANDLE) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit command buffer");
    }
    vkQueueWaitIdle(queue);
    vkFreeCommandBuffers(device.logical(), pool, 1, &cb);
}

void VulkanImage::copyFromBuffer(VulkanDevice& device, VkCommandPool pool, VkQueue queue, VkBuffer buffer, uint32_t w, uint32_t h) {
    VkCommandBufferAllocateInfo aInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, pool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1 };
    VkCommandBuffer cb; vkAllocateCommandBuffers(device.logical(), &aInfo, &cb);

    VkCommandBufferBeginInfo bInfo{};
    bInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &bInfo);

    VkBufferImageCopy region{};
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageExtent = {w, h, 1};
    vkCmdCopyBufferToImage(cb, buffer, m_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
    vkEndCommandBuffer(cb);

    VkSubmitInfo sInfo{};
    sInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    sInfo.commandBufferCount = 1;
    sInfo.pCommandBuffers = &cb;

    if (vkQueueSubmit(queue, 1, &sInfo, VK_NULL_HANDLE) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit command buffer");
    }
    vkQueueWaitIdle(queue);
    vkFreeCommandBuffers(device.logical(), pool, 1, &cb);
}

void VulkanImage::createView(VulkanDevice& device, VkFormat f, VkImageAspectFlags a) {
    VkImageViewCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    info.image = m_image;
    info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    info.format = f;
    info.subresourceRange = {a, 0, 1, 0, 1};
    vkCreateImageView(device.logical(), &info, nullptr, &m_view);
}

void VulkanImage::createSampler(VulkanDevice& device) {
    VkSamplerCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    info.magFilter = VK_FILTER_LINEAR;
    info.minFilter = VK_FILTER_LINEAR;
    info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    info.anisotropyEnable = VK_TRUE;
    info.maxAnisotropy = device.properties().limits.maxSamplerAnisotropy;
    info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    vkCreateSampler(device.logical(), &info, nullptr, &m_sampler);
}

VulkanImage VulkanImage::fromTextureData(VulkanDevice& device, VkCommandPool pool, VkQueue queue, const TextureData& data) {
    VulkanBuffer staging; staging.create(device, data.pixels.size(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(device, staging.handle(), staging.memory(), data.pixels.data(), data.pixels.size());
    VulkanImage img; img.create(device, data.width, data.height, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    img.transitionLayout(device, pool, queue, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    img.copyFromBuffer(device, pool, queue, staging.handle(), data.width, data.height);
    img.transitionLayout(device, pool, queue, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    staging.destroy(device);
    img.createView(device, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT);
    img.createSampler(device);
    return img;
}

} // namespace ge
