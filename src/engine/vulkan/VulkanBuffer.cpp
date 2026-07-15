#include "engine/vulkan/VulkanBuffer.h"
#include "engine/vulkan/VulkanDevice.h"
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace ge {

VulkanBuffer::~VulkanBuffer() {}

void VulkanBuffer::create(VulkanDevice& device, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags props) {
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = size;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device.logical(), &info, nullptr, &m_buffer) != VK_SUCCESS) throw std::runtime_error("No buffer");
    VkMemoryRequirements mem; vkGetBufferMemoryRequirements(device.logical(), m_buffer, &mem);
    VkMemoryAllocateInfo alloc{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, mem.size, device.findMemoryType(mem.memoryTypeBits, props) };
    if (vkAllocateMemory(device.logical(), &alloc, nullptr, &m_memory) != VK_SUCCESS) throw std::runtime_error("No memory");
    vkBindBufferMemory(device.logical(), m_buffer, m_memory, 0);
}

void VulkanBuffer::destroy(VulkanDevice& device) {
    if (m_buffer) vkDestroyBuffer(device.logical(), m_buffer, nullptr);
    if (m_memory) vkFreeMemory(device.logical(), m_memory, nullptr);
    m_buffer = VK_NULL_HANDLE; m_memory = VK_NULL_HANDLE;
}

void VulkanBuffer::write(VulkanDevice& device, VkBuffer, VkDeviceMemory memory, const void* data, VkDeviceSize size) {
    void* mapped; vkMapMemory(device.logical(), memory, 0, size, 0, &mapped);
    std::memcpy(mapped, data, (size_t)size);
    vkUnmapMemory(device.logical(), memory);
}

void VulkanBuffer::copyBuffer(VulkanDevice& device, VkCommandPool pool, VkQueue queue, VkBuffer src, VkBuffer dst, VkDeviceSize size) {
    VkCommandBufferAllocateInfo aInfo{};
    aInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    aInfo.commandPool = pool;
    aInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    aInfo.commandBufferCount = 1;

    VkCommandBuffer cb; vkAllocateCommandBuffers(device.logical(), &aInfo, &cb);

    VkCommandBufferBeginInfo bInfo{};
    bInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &bInfo);

    VkBufferCopy copy{ 0, 0, size }; vkCmdCopyBuffer(cb, src, dst, 1, &copy);
    vkEndCommandBuffer(cb);

    VkSubmitInfo sInfo{};
    sInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    sInfo.commandBufferCount = 1;
    sInfo.pCommandBuffers = &cb;

    if (vkQueueSubmit(queue, 1, &sInfo, VK_NULL_HANDLE) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit copy command buffer");
    }
    vkQueueWaitIdle(queue);
    vkFreeCommandBuffers(device.logical(), pool, 1, &cb);
}

std::vector<char> readSpvFile(const std::string& path) {
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) throw std::runtime_error("No file: " + path);
    size_t size = (size_t)file.tellg(); std::vector<char> buf(size);
    file.seekg(0); file.read(buf.data(), size);
    return buf;
}

} // namespace ge
