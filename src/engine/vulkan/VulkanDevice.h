#pragma once

// =============================================================================
// VulkanDevice.h — Physical GPU + logical VkDevice + queues
// =============================================================================
// You pick a VkPhysicalDevice (GPU), then create VkDevice with the queue
// families and extensions needed for swapchain presentation.
// =============================================================================

#include <vulkan/vulkan.h>
#include <optional>
#include <vector>

struct GLFWwindow;

namespace ge {

class VulkanContext;

/// Queue families are not always the same index; graphics and present may differ.
struct QueueFamilyIndices {
    std::optional<uint32_t> graphics;
    std::optional<uint32_t> present;

    [[nodiscard]] bool isComplete() const {
        return graphics.has_value() && present.has_value();
    }
};

class VulkanDevice {
public:
    VulkanDevice(VulkanContext& context, GLFWwindow* window);
    ~VulkanDevice();

    VulkanDevice(const VulkanDevice&) = delete;
    VulkanDevice& operator=(const VulkanDevice&) = delete;

    [[nodiscard]] VkPhysicalDevice physical() const { return m_physicalDevice; }
    [[nodiscard]] VkDevice logical() const { return m_device; }
    [[nodiscard]] VkSurfaceKHR surface() const { return m_surface; }
    [[nodiscard]] QueueFamilyIndices queueFamilies() const { return m_queueFamilies; }
    [[nodiscard]] VkQueue graphicsQueue() const { return m_graphicsQueue; }
    [[nodiscard]] VkQueue presentQueue() const { return m_presentQueue; }

    [[nodiscard]] VkFormat findSupportedFormat(
        const std::vector<VkFormat>& candidates,
        VkImageTiling tiling,
        VkFormatFeatureFlags features) const;

    [[nodiscard]] uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

private:
    void createSurface(GLFWwindow* window);
    void pickPhysicalDevice();
    void createLogicalDevice();

    bool isDeviceSuitable(VkPhysicalDevice device) const;
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) const;
    bool checkDeviceExtensionSupport(VkPhysicalDevice device) const;

    VulkanContext& m_context;
    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;

    QueueFamilyIndices m_queueFamilies;
    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
    VkQueue m_presentQueue = VK_NULL_HANDLE;

    const std::vector<const char*> m_deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
};

} // namespace ge
