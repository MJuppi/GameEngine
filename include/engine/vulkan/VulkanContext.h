#pragma once

// =============================================================================
// VulkanContext.h — VkInstance and (debug) validation
// =============================================================================
// VkInstance is the connection between your app and the Vulkan loader/library.
// Almost every Vulkan object is created from or tied to the instance.
// =============================================================================

#include <vulkan/vulkan.h>
#include <vector>

struct GLFWwindow;

namespace ge {

class VulkanContext {
public:
    explicit VulkanContext(bool enableValidation);
    ~VulkanContext();

    /// Creates VkInstance (needs GLFW for required instance extensions).
    void init(GLFWwindow* window);

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;

    [[nodiscard]] VkInstance instance() const { return m_instance; }
    [[nodiscard]] bool validationEnabled() const { return m_enableValidation; }
    [[nodiscard]] const std::vector<const char*>& validationLayers() const {
        return m_validationLayers;
    }

    /// Required when validation is on: logs errors/warnings from the driver.
    void createDebugMessenger();
    void destroyDebugMessenger();

    /// Extensions required for GLFW + debug utils (if enabled).
    [[nodiscard]] std::vector<const char*> requiredExtensions(GLFWwindow* window) const;

private:
    void createInstance(GLFWwindow* window);
    bool checkValidationLayerSupport() const;

    VkInstance m_instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
    bool m_enableValidation = false;

    const std::vector<const char*> m_validationLayers = {"VK_LAYER_KHRONOS_validation"};
};

} // namespace ge
