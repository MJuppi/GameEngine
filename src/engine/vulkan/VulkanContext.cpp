#include "engine/vulkan/VulkanContext.h"
#include <GLFW/glfw3.h>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace ge {

namespace {

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT /*type*/,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
    void* /*userData*/)
{
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        std::cerr << "[Vulkan ERROR] " << callbackData->pMessage << std::endl;
    } else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        std::cerr << "[Vulkan WARNING] " << callbackData->pMessage << std::endl;
    } else {
        // Optionally log verbose/info here
    }
    return VK_FALSE;
}

} // namespace

VulkanContext::VulkanContext(bool enableValidation) : m_enableValidation(enableValidation) {
    if (m_enableValidation && !checkValidationLayerSupport()) {
        throw std::runtime_error("Validation layers requested but not available");
    }
}

VulkanContext::~VulkanContext() {
    destroyDebugMessenger();
    if (m_instance != VK_NULL_HANDLE) vkDestroyInstance(m_instance, nullptr);
}

void VulkanContext::init(GLFWwindow* window) {
    std::cout << "[VulkanContext] Initializing Vulkan Instance..." << std::endl;
    createInstance(window);
    createDebugMessenger();
}

std::vector<const char*> VulkanContext::requiredExtensions(GLFWwindow* /*window*/) const {
    uint32_t count = 0;
    const char** glfwExts = glfwGetRequiredInstanceExtensions(&count);
    std::vector<const char*> exts(glfwExts, glfwExts + count);
    if (m_enableValidation) exts.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    return exts;
}

void VulkanContext::createInstance(GLFWwindow* window) {
    VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO, nullptr, "GameEngine", 1, "GameEngine", 1, VK_API_VERSION_1_0 };
    auto exts = requiredExtensions(window);

    VkInstanceCreateInfo info{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, nullptr, 0, &app, 0, nullptr, (uint32_t)exts.size(), exts.data() };
    VkDebugUtilsMessengerCreateInfoEXT debug{ VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT, nullptr, 0,
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
        debugCallback, nullptr };

    if (m_enableValidation) {
        info.enabledLayerCount = (uint32_t)m_validationLayers.size();
        info.ppEnabledLayerNames = m_validationLayers.data();
        info.pNext = &debug;
    }

    if (vkCreateInstance(&info, nullptr, &m_instance) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan instance");
    }
    std::cout << "[VulkanContext] Vulkan Instance created." << std::endl;
}

void VulkanContext::createDebugMessenger() {
    if (!m_enableValidation) return;
    VkDebugUtilsMessengerCreateInfoEXT info{ VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT, nullptr, 0,
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT,
        debugCallback, nullptr };

    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT");
    if (!func || func(m_instance, &info, nullptr, &m_debugMessenger) != VK_SUCCESS) {
        throw std::runtime_error("Failed to set up debug messenger");
    }
}

void VulkanContext::destroyDebugMessenger() {
    if (m_debugMessenger == VK_NULL_HANDLE) return;
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT");
    if (func) func(m_instance, m_debugMessenger, nullptr);
    m_debugMessenger = VK_NULL_HANDLE;
}

bool VulkanContext::checkValidationLayerSupport() const {
    uint32_t count;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> available(count);
    vkEnumerateInstanceLayerProperties(&count, available.data());
    for (const char* name : m_validationLayers) {
        bool found = false;
        for (const auto& layer : available) if (strcmp(name, layer.layerName) == 0) { found = true; break; }
        if (!found) return false;
    }
    return true;
}

} // namespace ge
