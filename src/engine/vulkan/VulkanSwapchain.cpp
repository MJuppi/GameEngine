#include "engine/vulkan/VulkanSwapchain.h"
#include "engine/vulkan/VulkanDevice.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace ge {

VulkanSwapchain::VulkanSwapchain(VulkanDevice& device, GLFWwindow* window)
    : m_device(device)
{
    createSwapchain(window);
    createImageViews();
    createRenderPass();
    createDepthResources();
    createFramebuffers();
}

VulkanSwapchain::~VulkanSwapchain() {
    cleanupSwapchain();
    if (m_renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(m_device.logical(), m_renderPass, nullptr);
    }
}

void VulkanSwapchain::recreate(GLFWwindow* window) {
    vkDeviceWaitIdle(m_device.logical());
    cleanupSwapchain();
    createSwapchain(window);
    createImageViews();
    createDepthResources();
    createFramebuffers();
}

SwapchainSupportDetails VulkanSwapchain::querySupport() const {
    SwapchainSupportDetails details;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
        m_device.physical(), m_device.surface(), &details.capabilities);

    uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(
        m_device.physical(), m_device.surface(), &formatCount, nullptr);
    if (formatCount != 0) {
        details.formats.resize(formatCount);
        vkGetPhysicalDeviceSurfaceFormatsKHR(
            m_device.physical(), m_device.surface(), &formatCount, details.formats.data());
    }

    uint32_t presentModeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(
        m_device.physical(), m_device.surface(), &presentModeCount, nullptr);
    if (presentModeCount != 0) {
        details.presentModes.resize(presentModeCount);
        vkGetPhysicalDeviceSurfacePresentModesKHR(
            m_device.physical(), m_device.surface(), &presentModeCount, details.presentModes.data());
    }
    return details;
}

VkSurfaceFormatKHR VulkanSwapchain::chooseFormat(const std::vector<VkSurfaceFormatKHR>& formats) const {
    for (const auto& available : formats) {
        if (available.format == VK_FORMAT_B8G8R8A8_SRGB &&
            available.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return available;
        }
    }
    return formats[0];
}

VkPresentModeKHR VulkanSwapchain::choosePresentMode(const std::vector<VkPresentModeKHR>& modes) const {
    // MAILBOX = low latency (may drop frames). FIFO = vsync. Prefer MAILBOX if present.
    for (VkPresentModeKHR mode : modes) {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return mode;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D VulkanSwapchain::chooseExtent(
    const VkSurfaceCapabilitiesKHR& capabilities,
    GLFWwindow* window) const
{
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    VkExtent2D actual{};
    actual.width = std::clamp(
        static_cast<uint32_t>(width),
        capabilities.minImageExtent.width,
        capabilities.maxImageExtent.width);
    actual.height = std::clamp(
        static_cast<uint32_t>(height),
        capabilities.minImageExtent.height,
        capabilities.maxImageExtent.height);
    return actual;
}

void VulkanSwapchain::createSwapchain(GLFWwindow* window) {
    auto support = querySupport();
    auto surfaceFormat = chooseFormat(support.formats);
    auto presentMode = choosePresentMode(support.presentModes);
    m_extent = chooseExtent(support.capabilities, window);
    m_imageFormat = surfaceFormat.format;

    uint32_t imageCount = support.capabilities.minImageCount + 1;
    if (support.capabilities.maxImageCount > 0 &&
        imageCount > support.capabilities.maxImageCount) {
        imageCount = support.capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = m_device.surface();
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = m_extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    const auto& families = m_device.queueFamilies();
    uint32_t queueFamilyIndices[] = {families.graphics.value(), families.present.value()};

    if (families.graphics != families.present) {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    } else {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    createInfo.preTransform = support.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(m_device.logical(), &createInfo, nullptr, &m_swapchain) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create swapchain");
    }

    vkGetSwapchainImagesKHR(m_device.logical(), m_swapchain, &imageCount, nullptr);
    m_images.resize(imageCount);
    vkGetSwapchainImagesKHR(m_device.logical(), m_swapchain, &imageCount, m_images.data());
}

void VulkanSwapchain::createImageViews() {
    m_imageViews.resize(m_images.size());
    for (size_t i = 0; i < m_images.size(); ++i) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_images[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_imageFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device.logical(), &viewInfo, nullptr, &m_imageViews[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create image views");
        }
    }
}

VkFormat VulkanSwapchain::findDepthFormat() const {
    return m_device.findSupportedFormat(
        {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT},
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
}

void VulkanSwapchain::createRenderPass() {
    m_depthFormat = findDepthFormat();

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = m_imageFormat;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = m_depthFormat;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthRef{};
    depthRef.attachment = 1;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                              VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                              VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                               VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};

    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(m_device.logical(), &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create render pass");
    }
}

void VulkanSwapchain::createDepthResources() {
    m_depthImages.resize(m_imageViews.size());
    m_depthImageMemory.resize(m_imageViews.size());
    m_depthImageViews.resize(m_imageViews.size());

    for (size_t i = 0; i < m_imageViews.size(); ++i) {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = m_extent.width;
        imageInfo.extent.height = m_extent.height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = m_depthFormat;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(m_device.logical(), &imageInfo, nullptr, &m_depthImages[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create depth image");
        }

        VkMemoryRequirements memRequirements;
        vkGetImageMemoryRequirements(m_device.logical(), m_depthImages[i], &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = m_device.findMemoryType(
            memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        if (vkAllocateMemory(m_device.logical(), &allocInfo, nullptr, &m_depthImageMemory[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to allocate depth image memory");
        }

        vkBindImageMemory(m_device.logical(), m_depthImages[i], m_depthImageMemory[i], 0);

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_depthImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_depthFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device.logical(), &viewInfo, nullptr, &m_depthImageViews[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create depth image view");
        }
    }
}

void VulkanSwapchain::destroyDepthResources() {
    for (auto view : m_depthImageViews) {
        if (view != VK_NULL_HANDLE) {
            vkDestroyImageView(m_device.logical(), view, nullptr);
        }
    }
    m_depthImageViews.clear();

    for (size_t i = 0; i < m_depthImages.size(); ++i) {
        if (m_depthImages[i] != VK_NULL_HANDLE) {
            vkDestroyImage(m_device.logical(), m_depthImages[i], nullptr);
        }
        if (i < m_depthImageMemory.size() && m_depthImageMemory[i] != VK_NULL_HANDLE) {
            vkFreeMemory(m_device.logical(), m_depthImageMemory[i], nullptr);
        }
    }
    m_depthImages.clear();
    m_depthImageMemory.clear();
}

void VulkanSwapchain::createFramebuffers() {
    m_framebuffers.resize(m_imageViews.size());
    for (size_t i = 0; i < m_imageViews.size(); ++i) {
        std::array<VkImageView, 2> attachments = {m_imageViews[i], m_depthImageViews[i]};

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = m_renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = m_extent.width;
        framebufferInfo.height = m_extent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(m_device.logical(), &framebufferInfo, nullptr, &m_framebuffers[i]) !=
            VK_SUCCESS) {
            throw std::runtime_error("Failed to create framebuffer");
        }
    }
}

void VulkanSwapchain::cleanupSwapchain() {
    for (auto framebuffer : m_framebuffers) {
        vkDestroyFramebuffer(m_device.logical(), framebuffer, nullptr);
    }
    m_framebuffers.clear();

    destroyDepthResources();

    for (auto view : m_imageViews) {
        vkDestroyImageView(m_device.logical(), view, nullptr);
    }
    m_imageViews.clear();

    if (m_swapchain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(m_device.logical(), m_swapchain, nullptr);
        m_swapchain = VK_NULL_HANDLE;
    }
}

} // namespace ge
