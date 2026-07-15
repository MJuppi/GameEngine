#include "engine/vulkan/VulkanSwapchain.h"
#include "engine/vulkan/VulkanDevice.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <stdexcept>

namespace ge {

VulkanSwapchain::VulkanSwapchain(VulkanDevice& device, GLFWwindow* window) : m_device(device) {
    std::cout << "[VulkanSwapchain] Creating swapchain..." << std::endl;
    createSwapchain(window);
    createImageViews();
    createRenderPass();
    createDepthResources();
    createFramebuffers();
    std::cout << "[VulkanSwapchain] Swapchain ready. Extent: " << m_extent.width << "x" << m_extent.height << std::endl;
}

VulkanSwapchain::~VulkanSwapchain() {
    cleanupSwapchain();
    if (m_renderPass != VK_NULL_HANDLE) vkDestroyRenderPass(m_device.logical(), m_renderPass, nullptr);
}

void VulkanSwapchain::recreate(GLFWwindow* window) {
    vkDeviceWaitIdle(m_device.logical());
    cleanupSwapchain();
    createSwapchain(window);
    createImageViews();
    createDepthResources();
    createFramebuffers();
}

void VulkanSwapchain::createSwapchain(GLFWwindow* window) {
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_device.physical(), m_device.surface(), &caps);
    uint32_t fCount, pCount;
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_device.physical(), m_device.surface(), &fCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_device.physical(), m_device.surface(), &fCount, formats.data());
    vkGetPhysicalDeviceSurfacePresentModesKHR(m_device.physical(), m_device.surface(), &pCount, nullptr);
    std::vector<VkPresentModeKHR> modes(pCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(m_device.physical(), m_device.surface(), &pCount, modes.data());

    auto format = formats[0];
    for (auto& f : formats) if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) format = f;
    auto mode = VK_PRESENT_MODE_FIFO_KHR;
    for (auto& m : modes) if (m == VK_PRESENT_MODE_MAILBOX_KHR) mode = m;

    if (caps.currentExtent.width != 0xFFFFFFFF) m_extent = caps.currentExtent;
    else {
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        m_extent = { std::clamp((uint32_t)w, caps.minImageExtent.width, caps.maxImageExtent.width),
                     std::clamp((uint32_t)h, caps.minImageExtent.height, caps.maxImageExtent.height) };
    }

    uint32_t count = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && count > caps.maxImageCount) count = caps.maxImageCount;

    m_imageFormat = format.format;
    VkSwapchainCreateInfoKHR info{};
    info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    info.surface = m_device.surface();
    info.minImageCount = count;
    info.imageFormat = m_imageFormat;
    info.imageColorSpace = format.colorSpace;
    info.imageExtent = m_extent;
    info.imageArrayLayers = 1;
    info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    auto q = m_device.queueFamilies();
    uint32_t indices[] = { q.graphics.value(), q.present.value() };
    if (q.graphics != q.present) { info.imageSharingMode = VK_SHARING_MODE_CONCURRENT; info.queueFamilyIndexCount = 2; info.pQueueFamilyIndices = indices; }
    else info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;

    info.preTransform = caps.currentTransform;
    info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    info.presentMode = mode;
    info.clipped = VK_TRUE;

    if (vkCreateSwapchainKHR(m_device.logical(), &info, nullptr, &m_swapchain) != VK_SUCCESS) throw std::runtime_error("No swapchain");
    vkGetSwapchainImagesKHR(m_device.logical(), m_swapchain, &count, nullptr);
    m_images.resize(count);
    vkGetSwapchainImagesKHR(m_device.logical(), m_swapchain, &count, m_images.data());
}

void VulkanSwapchain::createImageViews() {
    m_imageViews.resize(m_images.size());
    for (size_t i = 0; i < m_images.size(); i++) {
        VkImageViewCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        info.image = m_images[i];
        info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        info.format = m_imageFormat;
        info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        if (vkCreateImageView(m_device.logical(), &info, nullptr, &m_imageViews[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create swapchain image view " + std::to_string(i));
        }
    }
}

void VulkanSwapchain::createRenderPass() {
    m_depthFormat = m_device.findSupportedFormat({VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT}, VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);

    VkAttachmentDescription attachments[2]{};
    attachments[0].format = m_imageFormat;
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    attachments[1].format = m_depthFormat;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference cRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    VkAttachmentReference dRef{ 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

    VkSubpassDescription sub{};
    sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sub.colorAttachmentCount = 1;
    sub.pColorAttachments = &cRef;
    sub.pDepthStencilAttachment = &dRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    info.attachmentCount = 2;
    info.pAttachments = attachments;
    info.subpassCount = 1;
    info.pSubpasses = &sub;
    info.dependencyCount = 1;
    info.pDependencies = &dep;

    if (vkCreateRenderPass(m_device.logical(), &info, nullptr, &m_renderPass) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create render pass");
    }
}

void VulkanSwapchain::createDepthResources() {
    m_depthImages.resize(m_images.size()); m_depthImageMemory.resize(m_images.size()); m_depthImageViews.resize(m_images.size());
    for (size_t i = 0; i < m_images.size(); i++) {
        VkImageCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = m_depthFormat;
        info.extent = {m_extent.width, m_extent.height, 1};
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = VK_SAMPLE_COUNT_1_BIT;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        vkCreateImage(m_device.logical(), &info, nullptr, &m_depthImages[i]);
        VkMemoryRequirements mem; vkGetImageMemoryRequirements(m_device.logical(), m_depthImages[i], &mem);
        VkMemoryAllocateInfo alloc{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, mem.size, m_device.findMemoryType(mem.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) };
        vkAllocateMemory(m_device.logical(), &alloc, nullptr, &m_depthImageMemory[i]);
        vkBindImageMemory(m_device.logical(), m_depthImages[i], m_depthImageMemory[i], 0);

        VkImageViewCreateInfo vInfo{};
        vInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vInfo.image = m_depthImages[i];
        vInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vInfo.format = m_depthFormat;
        vInfo.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
        vkCreateImageView(m_device.logical(), &vInfo, nullptr, &m_depthImageViews[i]);
    }
}

void VulkanSwapchain::cleanupSwapchain() {
    for (auto f : m_framebuffers) vkDestroyFramebuffer(m_device.logical(), f, nullptr);
    for (size_t i = 0; i < m_depthImages.size(); i++) {
        vkDestroyImageView(m_device.logical(), m_depthImageViews[i], nullptr);
        vkDestroyImage(m_device.logical(), m_depthImages[i], nullptr);
        vkFreeMemory(m_device.logical(), m_depthImageMemory[i], nullptr);
    }
    for (auto v : m_imageViews) vkDestroyImageView(m_device.logical(), v, nullptr);
    if (m_swapchain != VK_NULL_HANDLE) vkDestroySwapchainKHR(m_device.logical(), m_swapchain, nullptr);
    m_framebuffers.clear(); m_depthImages.clear(); m_depthImageMemory.clear(); m_depthImageViews.clear(); m_imageViews.clear();
}

void VulkanSwapchain::createFramebuffers() {
    m_framebuffers.resize(m_imageViews.size());
    for (size_t i = 0; i < m_imageViews.size(); i++) {
        VkImageView att[] = { m_imageViews[i], m_depthImageViews[i] };
        VkFramebufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        info.renderPass = m_renderPass;
        info.attachmentCount = 2;
        info.pAttachments = att;
        info.width = m_extent.width;
        info.height = m_extent.height;
        info.layers = 1;
        if (vkCreateFramebuffer(m_device.logical(), &info, nullptr, &m_framebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create framebuffer " + std::to_string(i));
        }
    }
}

} // namespace ge
