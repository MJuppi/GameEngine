#pragma once

// =============================================================================
// VulkanBuffer.h — GPU buffers and memory
// =============================================================================
// Buffers hold vertex/index/uniform data. Device-local memory is fast for the GPU;
// host-visible memory lets the CPU map and write (e.g. per-frame MVP updates).
// =============================================================================

#include <vulkan/vulkan.h>
#include <string>
#include <vector>

namespace ge {

class VulkanDevice;

class VulkanBuffer {
public:
    VulkanBuffer() = default;
    ~VulkanBuffer();

    VulkanBuffer(const VulkanBuffer&) = delete;
    VulkanBuffer& operator=(const VulkanBuffer&) = delete;

    [[nodiscard]] VkBuffer handle() const { return m_buffer; }
    [[nodiscard]] VkDeviceMemory memory() const { return m_memory; }

    void create(
        VulkanDevice& device,
        VkDeviceSize size,
        VkBufferUsageFlags usage,
        VkMemoryPropertyFlags properties);

    void destroy(VulkanDevice& device);

    /// Copy host data into a host-visible buffer (map/unmap).
    static void write(
        VulkanDevice& device,
        VkBuffer buffer,
        VkDeviceMemory memory,
        const void* data,
        VkDeviceSize size);

    /// Staging buffer -> device-local buffer (typical upload path).
    static void copyBuffer(
        VulkanDevice& device,
        VkCommandPool commandPool,
        VkQueue queue,
        VkBuffer src,
        VkBuffer dst,
        VkDeviceSize size);

private:
    VkBuffer m_buffer = VK_NULL_HANDLE;
    VkDeviceMemory m_memory = VK_NULL_HANDLE;
};

/// Loads SPIR-V from disk (used by pipeline and kept here for reuse).
std::vector<char> readSpvFile(const std::string& path);

} // namespace ge
