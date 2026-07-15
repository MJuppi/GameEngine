#include "engine/vulkan/VulkanDevice.h"
#include "engine/vulkan/VulkanContext.h"
#include <GLFW/glfw3.h>
#include <set>
#include <iostream>
#include <stdexcept>

namespace ge {

VulkanDevice::VulkanDevice(VulkanContext& context, GLFWwindow* window) : m_context(context) {
    if (glfwCreateWindowSurface(m_context.instance(), window, nullptr, &m_surface) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create surface");
    }
    pickPhysicalDevice();
    createLogicalDevice();
}

VulkanDevice::~VulkanDevice() {
    if (m_device != VK_NULL_HANDLE) vkDestroyDevice(m_device, nullptr);
    if (m_surface != VK_NULL_HANDLE) vkDestroySurfaceKHR(m_context.instance(), m_surface, nullptr);
}

void VulkanDevice::pickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(m_context.instance(), &count, nullptr);
    if (count == 0) throw std::runtime_error("No Vulkan GPUs");
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(m_context.instance(), &count, devices.data());

    std::cout << "[VulkanDevice] Available GPUs (" << count << "):" << std::endl;
    for (auto d : devices) {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(d, &props);
        std::cout << "  - " << props.deviceName << " (Type: " << props.deviceType << ")" << std::endl;
    }

    for (auto d : devices) {
        if (isDeviceSuitable(d)) {
            m_physicalDevice = d;
            VkPhysicalDeviceProperties props;
            vkGetPhysicalDeviceProperties(d, &props);
            std::cout << "[VulkanDevice] Selected GPU: " << props.deviceName << std::endl;
            return;
        }
    }
    throw std::runtime_error("No suitable GPU");
}

bool VulkanDevice::isDeviceSuitable(VkPhysicalDevice device) const {
    auto indices = findQueueFamilies(device);
    bool extSupported = checkDeviceExtensionSupport(device);
    bool swapchainAdequate = false;
    if (extSupported) {
        uint32_t f, p;
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &f, nullptr);
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &p, nullptr);
        swapchainAdequate = f != 0 && p != 0;
    }
    VkPhysicalDeviceFeatures features;
    vkGetPhysicalDeviceFeatures(device, &features);
    return indices.isComplete() && extSupported && swapchainAdequate && features.samplerAnisotropy;
}

QueueFamilyIndices VulkanDevice::findQueueFamilies(VkPhysicalDevice device) const {
    QueueFamilyIndices indices;
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
    for (uint32_t i = 0; i < count; ++i) {
        if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) indices.graphics = i;
        VkBool32 present = false;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &present);
        if (present) indices.present = i;
        if (indices.isComplete()) break;
    }
    return indices;
}

bool VulkanDevice::checkDeviceExtensionSupport(VkPhysicalDevice device) const {
    uint32_t count;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> available(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, available.data());
    std::set<std::string> required(m_deviceExtensions.begin(), m_deviceExtensions.end());
    for (const auto& ext : available) required.erase(ext.extensionName);
    return required.empty();
}

void VulkanDevice::createLogicalDevice() {
    m_queueFamilies = findQueueFamilies(m_physicalDevice);
    std::set<uint32_t> unique = {m_queueFamilies.graphics.value(), m_queueFamilies.present.value()};
    std::vector<VkDeviceQueueCreateInfo> qInfos;
    float priority = 1.0f;
    for (auto q : unique) qInfos.push_back({ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, nullptr, 0, q, 1, &priority });

    VkPhysicalDeviceFeatures features{};
    features.samplerAnisotropy = VK_TRUE;
    VkDeviceCreateInfo info{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO, nullptr, 0, (uint32_t)qInfos.size(), qInfos.data(), 0, nullptr, (uint32_t)m_deviceExtensions.size(), m_deviceExtensions.data(), &features };

    if (vkCreateDevice(m_physicalDevice, &info, nullptr, &m_device) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create logical device");
    }
    vkGetDeviceQueue(m_device, m_queueFamilies.graphics.value(), 0, &m_graphicsQueue);
    vkGetDeviceQueue(m_device, m_queueFamilies.present.value(), 0, &m_presentQueue);
}

VkFormat VulkanDevice::findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features) const {
    for (auto format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(m_physicalDevice, format, &props);
        if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features) return format;
        if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features) return format;
    }
    throw std::runtime_error("No supported format");
}

uint32_t VulkanDevice::findMemoryType(uint32_t filter, VkMemoryPropertyFlags props) const {
    VkPhysicalDeviceMemoryProperties mem;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &mem);
    for (uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
        if ((filter & (1 << i)) && (mem.memoryTypes[i].propertyFlags & props) == props) return i;
    }
    throw std::runtime_error("No suitable memory type");
}

} // namespace ge
