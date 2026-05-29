#include "engine/vulkan/VulkanBuffer.h"
#include "engine/vulkan/VulkanDevice.h"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace ge {

VulkanBuffer::~VulkanBuffer() {
    // Destructor intentionally empty: caller must call destroy() before the VulkanDevice is torn down.
}

void VulkanBuffer::create(
    VulkanDevice& device,
    VkDeviceSize size,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties)
{
    // Create a VkBuffer and allocate device memory for it.
    // Memory type is selected via VulkanDevice::findMemoryType.
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device.logical(), &bufferInfo, nullptr, &m_buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create buffer");
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(device.logical(), m_buffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex =
        device.findMemoryType(memRequirements.memoryTypeBits, properties);

    if (vkAllocateMemory(device.logical(), &allocInfo, nullptr, &m_memory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate buffer memory");
    }

    vkBindBufferMemory(device.logical(), m_buffer, m_memory, 0);
}

void VulkanBuffer::destroy(VulkanDevice& device) {
    // Free buffer and associated memory from the logical device.
    if (m_buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device.logical(), m_buffer, nullptr);
        m_buffer = VK_NULL_HANDLE;
    }
    if (m_memory != VK_NULL_HANDLE) {
        vkFreeMemory(device.logical(), m_memory, nullptr);
        m_memory = VK_NULL_HANDLE;
    }
}

void VulkanBuffer::write(
    VulkanDevice& device,
    VkBuffer buffer,
    VkDeviceMemory memory,
    const void* data,
    VkDeviceSize size)
{
    // Map host-visible memory and copy CPU-side data into it.
    void* mapped = nullptr;
    vkMapMemory(device.logical(), memory, 0, size, 0, &mapped);
    std::memcpy(mapped, data, static_cast<size_t>(size));
    vkUnmapMemory(device.logical(), memory);
    (void)buffer;
}

void VulkanBuffer::copyBuffer(
    VulkanDevice& device,
    VkCommandPool commandPool,
    VkQueue queue,
    VkBuffer src,
    VkBuffer dst,
    VkDeviceSize size)
{
    // Record and submit a short-lived command buffer to copy buffer contents on the GPU.
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = commandPool;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(device.logical(), &allocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    VkBufferCopy copyRegion{};
    copyRegion.size = size;
    vkCmdCopyBuffer(commandBuffer, src, dst, 1, &copyRegion);

    vkEndCommandBuffer(commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(queue);

    vkFreeCommandBuffers(device.logical(), commandPool, 1, &commandBuffer);
}

std::vector<char> readSpvFile(const std::string& path) {
    // Read a SPIR-V binary file entirely into a byte vector.
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    size_t fileSize = static_cast<size_t>(file.tellg());
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), static_cast<std::streamsize>(fileSize));
    return buffer;
}

} // namespace ge
