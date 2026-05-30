#pragma once

// =============================================================================
// VulkanSwapchain.h — Swapchain, render pass, framebuffers
// =============================================================================
// The swapchain is a queue of images the GPU renders into; the OS presents them
// to the screen. A render pass describes *how* we draw (attachments, load/store).
// Framebuffers bind swapchain image views to that render pass.
// =============================================================================

#include <vulkan/vulkan.h>
#include <vector>

struct GLFWwindow;

namespace ge {

class VulkanDevice;

struct SwapchainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities{};
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

class VulkanSwapchain {
public:
    VulkanSwapchain(VulkanDevice& device, GLFWwindow* window);
    ~VulkanSwapchain();

    VulkanSwapchain(const VulkanSwapchain&) = delete;
    VulkanSwapchain& operator=(const VulkanSwapchain&) = delete;

    void recreate(GLFWwindow* window);

    [[nodiscard]] VkSwapchainKHR handle() const { return m_swapchain; }
    [[nodiscard]] VkFormat imageFormat() const { return m_imageFormat; }
    [[nodiscard]] VkExtent2D extent() const { return m_extent; }
    [[nodiscard]] VkRenderPass renderPass() const { return m_renderPass; }
    [[nodiscard]] const std::vector<VkFramebuffer>& framebuffers() const { return m_framebuffers; }
    [[nodiscard]] const std::vector<VkImageView>& imageViews() const { return m_imageViews; }
    [[nodiscard]] size_t imageCount() const { return m_images.size(); }

private:
    void createSwapchain(GLFWwindow* window);
    void createImageViews();
    void createDepthResources();
    void createRenderPass();
    void createFramebuffers();
    void cleanupSwapchain();
    void destroyDepthResources();

    [[nodiscard]] VkFormat findDepthFormat() const;

    SwapchainSupportDetails querySupport() const;
    VkSurfaceFormatKHR chooseFormat(const std::vector<VkSurfaceFormatKHR>& formats) const;
    VkPresentModeKHR choosePresentMode(const std::vector<VkPresentModeKHR>& modes) const;
    VkExtent2D chooseExtent(const VkSurfaceCapabilitiesKHR& capabilities, GLFWwindow* window) const;

    VulkanDevice& m_device;
    VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
    std::vector<VkImage> m_images;
    VkFormat m_imageFormat{};
    VkExtent2D m_extent{};
    std::vector<VkImageView> m_imageViews;

    VkFormat m_depthFormat = VK_FORMAT_D32_SFLOAT;
    std::vector<VkImage> m_depthImages;
    std::vector<VkDeviceMemory> m_depthImageMemory;
    std::vector<VkImageView> m_depthImageViews;

    VkRenderPass m_renderPass = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> m_framebuffers;
};

} // namespace ge
