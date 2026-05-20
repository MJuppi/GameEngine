#include "engine/vulkan/VulkanRenderer.h"

#include "engine/Window.h"
#include "engine/mesh/Material.h"
#include "engine/mesh/MeshData.h"
#include "engine/vulkan/VulkanBuffer.h"
#include "engine/vulkan/VulkanContext.h"
#include "engine/vulkan/VulkanDevice.h"
#include "engine/vulkan/VulkanPipeline.h"
#include "engine/vulkan/VulkanSwapchain.h"

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <array>
#include <cstring>
#include <stdexcept>
#include <vector>

#ifndef SHADER_DIR
#define SHADER_DIR "shaders"
#endif

namespace ge {

namespace {

void framebufferResizeCallback(GLFWwindow* window, int /*width*/, int /*height*/) {
    auto* renderer = reinterpret_cast<VulkanRenderer*>(glfwGetWindowUserPointer(window));
    if (renderer) {
        renderer->onFramebufferResize();
    }
}

} // namespace

VulkanRenderer::VulkanRenderer(Window& window, MeshData mesh)
    : m_window(window)
    , m_mesh(std::move(mesh))
    , m_shaderDir(SHADER_DIR)
{
    const MeshBounds bounds = computeMeshBounds(m_mesh);
    m_meshRadius = bounds.radius;
    glfwSetWindowUserPointer(m_window.handle(), this);
    glfwSetFramebufferSizeCallback(m_window.handle(), framebufferResizeCallback);
    initVulkan();
}

VulkanRenderer::~VulkanRenderer() {
    if (m_device) {
        vkDeviceWaitIdle(m_device->logical());
    }

    for (size_t i = 0; i < m_sceneBuffers.size(); ++i) {
        if (m_sceneBuffersMapped[i]) {
            vkUnmapMemory(m_device->logical(), m_sceneBuffers[i]->memory());
        }
    }

    if (m_indexBuffer) {
        m_indexBuffer->destroy(*m_device);
        m_indexBuffer.reset();
    }
    if (m_vertexBuffer) {
        m_vertexBuffer->destroy(*m_device);
        m_vertexBuffer.reset();
    }
    for (auto& ub : m_sceneBuffers) {
        if (ub) {
            ub->destroy(*m_device);
        }
    }
    if (m_materialBuffer) {
        m_materialBuffer->destroy(*m_device);
        m_materialBuffer.reset();
    }

    cleanupSwapchain();

    if (m_descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device->logical(), m_descriptorPool, nullptr);
    }

    destroySyncObjects();

    if (m_commandPool != VK_NULL_HANDLE) {
        vkDestroyCommandPool(m_device->logical(), m_commandPool, nullptr);
    }
}

void VulkanRenderer::initVulkan() {
#ifdef GE_DEBUG
    const bool enableValidation = true;
#else
    const bool enableValidation = false;
#endif

    m_context = std::make_unique<VulkanContext>(enableValidation);
    m_context->init(m_window.handle());

    m_device = std::make_unique<VulkanDevice>(*m_context, m_window.handle());
    m_swapchain = std::make_unique<VulkanSwapchain>(*m_device, m_window.handle());

    m_pipeline = std::make_unique<VulkanPipeline>(
        *m_device,
        *m_swapchain,
        m_shaderDir + "/basic.vert.spv",
        m_shaderDir + "/basic.frag.spv");

    createCommandPool();
    createMeshBuffers();
    createSceneBuffers();
    createMaterialBuffer();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();
}

void VulkanRenderer::createCommandPool() {
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_device->queueFamilies().graphics.value();

    if (vkCreateCommandPool(m_device->logical(), &poolInfo, nullptr, &m_commandPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create command pool");
    }
}

void VulkanRenderer::createMeshBuffers() {
    if (m_mesh.vertices.empty() || m_mesh.indices.empty()) {
        throw std::runtime_error("Mesh has no vertices or indices");
    }

    VkDeviceSize vertexSize = sizeof(Vertex) * m_mesh.vertices.size();
    VkDeviceSize indexSize = sizeof(uint32_t) * m_mesh.indices.size();
    m_indexCount = static_cast<uint32_t>(m_mesh.indices.size());

    // Staging (CPU-visible) -> copy -> device-local (GPU-fast)
    VulkanBuffer stagingVertex;
    stagingVertex.create(
        *m_device,
        vertexSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(
        *m_device, stagingVertex.handle(), stagingVertex.memory(), m_mesh.vertices.data(), vertexSize);

    m_vertexBuffer = std::make_unique<VulkanBuffer>();
    m_vertexBuffer->create(
        *m_device,
        vertexSize,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VulkanBuffer::copyBuffer(
        *m_device,
        m_commandPool,
        m_device->graphicsQueue(),
        stagingVertex.handle(),
        m_vertexBuffer->handle(),
        vertexSize);
    stagingVertex.destroy(*m_device);

    VulkanBuffer stagingIndex;
    stagingIndex.create(
        *m_device,
        indexSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(
        *m_device, stagingIndex.handle(), stagingIndex.memory(), m_mesh.indices.data(), indexSize);

    m_indexBuffer = std::make_unique<VulkanBuffer>();
    m_indexBuffer->create(
        *m_device,
        indexSize,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VulkanBuffer::copyBuffer(
        *m_device,
        m_commandPool,
        m_device->graphicsQueue(),
        stagingIndex.handle(),
        m_indexBuffer->handle(),
        indexSize);
    stagingIndex.destroy(*m_device);
}

void VulkanRenderer::createSceneBuffers() {
    const VkDeviceSize bufferSize = sizeof(SceneUbo);
    m_sceneBuffers.resize(kMaxFramesInFlight);
    m_sceneBuffersMapped.resize(kMaxFramesInFlight);

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_sceneBuffers[i] = std::make_unique<VulkanBuffer>();
        m_sceneBuffers[i]->create(
            *m_device,
            bufferSize,
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        vkMapMemory(
            m_device->logical(),
            m_sceneBuffers[i]->memory(),
            0,
            bufferSize,
            0,
            &m_sceneBuffersMapped[i]);
    }
}

void VulkanRenderer::createMaterialBuffer() {
    fillMaterialBuffer(m_mesh.materials, m_materialGpu);

    m_materialBuffer = std::make_unique<VulkanBuffer>();
    m_materialBuffer->create(
        *m_device,
        sizeof(MaterialBufferObject),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    VulkanBuffer::write(
        *m_device,
        m_materialBuffer->handle(),
        m_materialBuffer->memory(),
        &m_materialGpu,
        sizeof(MaterialBufferObject));
}

void VulkanRenderer::createDescriptorPool() {
    std::array<VkDescriptorPoolSize, 2> poolSizes{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = static_cast<uint32_t>(kMaxFramesInFlight);
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[1].descriptorCount = static_cast<uint32_t>(kMaxFramesInFlight);

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = static_cast<uint32_t>(kMaxFramesInFlight);

    if (vkCreateDescriptorPool(m_device->logical(), &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor pool");
    }
}

void VulkanRenderer::createDescriptorSets() {
    std::vector<VkDescriptorSetLayout> layouts(
        kMaxFramesInFlight, m_pipeline->descriptorSetLayout());
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = m_descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(kMaxFramesInFlight);
    allocInfo.pSetLayouts = layouts.data();

    m_descriptorSets.resize(kMaxFramesInFlight);
    if (vkAllocateDescriptorSets(m_device->logical(), &allocInfo, m_descriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate descriptor sets");
    }

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        VkDescriptorBufferInfo sceneInfo{};
        sceneInfo.buffer = m_sceneBuffers[i]->handle();
        sceneInfo.offset = 0;
        sceneInfo.range = sizeof(SceneUbo);

        VkDescriptorBufferInfo materialInfo{};
        materialInfo.buffer = m_materialBuffer->handle();
        materialInfo.offset = 0;
        materialInfo.range = sizeof(MaterialBufferObject);

        std::array<VkWriteDescriptorSet, 2> writes{};

        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = m_descriptorSets[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].descriptorCount = 1;
        writes[0].pBufferInfo = &sceneInfo;

        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = m_descriptorSets[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[1].descriptorCount = 1;
        writes[1].pBufferInfo = &materialInfo;

        vkUpdateDescriptorSets(m_device->logical(), static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanRenderer::createCommandBuffers() {
    m_commandBuffers.resize(m_swapchain->imageCount());

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<uint32_t>(m_commandBuffers.size());

    if (vkAllocateCommandBuffers(m_device->logical(), &allocInfo, m_commandBuffers.data()) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate command buffers");
    }
}

void VulkanRenderer::createSyncObjects() {
    const size_t imageCount = m_swapchain->imageCount();

    m_imageAvailableSemaphores.resize(kMaxFramesInFlight);
    m_renderFinishedSemaphores.resize(imageCount);
    m_inFlightFences.resize(kMaxFramesInFlight);
    m_imagesInFlight.assign(imageCount, VK_NULL_HANDLE);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        if (vkCreateSemaphore(
                m_device->logical(), &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]) != VK_SUCCESS ||
            vkCreateFence(m_device->logical(), &fenceInfo, nullptr, &m_inFlightFences[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create frame sync objects");
        }
    }

    for (size_t i = 0; i < imageCount; ++i) {
        if (vkCreateSemaphore(
                m_device->logical(), &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create present sync objects");
        }
    }
}

void VulkanRenderer::destroySyncObjects() {
    for (VkSemaphore sem : m_imageAvailableSemaphores) {
        if (sem != VK_NULL_HANDLE) {
            vkDestroySemaphore(m_device->logical(), sem, nullptr);
        }
    }
    m_imageAvailableSemaphores.clear();

    for (VkSemaphore sem : m_renderFinishedSemaphores) {
        if (sem != VK_NULL_HANDLE) {
            vkDestroySemaphore(m_device->logical(), sem, nullptr);
        }
    }
    m_renderFinishedSemaphores.clear();

    for (VkFence fence : m_inFlightFences) {
        if (fence != VK_NULL_HANDLE) {
            vkDestroyFence(m_device->logical(), fence, nullptr);
        }
    }
    m_inFlightFences.clear();
    m_imagesInFlight.clear();
}

void VulkanRenderer::onFramebufferResize() {
    m_framebufferResized = true;
}

void VulkanRenderer::recreateSwapchain() {
    int width = 0, height = 0;
    glfwGetFramebufferSize(m_window.handle(), &width, &height);
    while (width == 0 || height == 0) {
        glfwGetFramebufferSize(m_window.handle(), &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(m_device->logical());

    destroySyncObjects();

    m_swapchain->recreate(m_window.handle());
    m_pipeline->recreate(*m_swapchain);

    vkDestroyCommandPool(m_device->logical(), m_commandPool, nullptr);
    createCommandPool();
    createCommandBuffers();
    createSyncObjects();
}

void VulkanRenderer::cleanupSwapchain() {
    m_pipeline.reset();
    m_swapchain.reset();
}

void VulkanRenderer::drawFrame() {
    vkWaitForFences(
        m_device->logical(), 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex = 0;
    VkResult acquireResult = vkAcquireNextImageKHR(
        m_device->logical(),
        m_swapchain->handle(),
        UINT64_MAX,
        m_imageAvailableSemaphores[m_currentFrame],
        VK_NULL_HANDLE,
        &imageIndex);

    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR || m_framebufferResized) {
        m_framebufferResized = false;
        recreateSwapchain();
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swapchain image");
    }

    // Do not reuse this swapchain image until the previous submission using it has finished.
    if (m_imagesInFlight[imageIndex] != VK_NULL_HANDLE) {
        vkWaitForFences(m_device->logical(), 1, &m_imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
    }

    // Camera fits the mesh bounding sphere (no vertex scaling — proportions stay true).
    m_rotation += 0.01f;
    const float r = m_meshRadius;
    const float aspect =
        m_swapchain->extent().width / static_cast<float>(m_swapchain->extent().height);
    const float cameraDistance = r * 2.8f;

    const glm::mat4 model = glm::rotate(glm::mat4(1.0f), m_rotation, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::mat4 view = glm::lookAt(
        glm::vec3(cameraDistance * 0.6f, cameraDistance * 0.35f, cameraDistance * 0.75f),
        glm::vec3(0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, r * 0.05f, r * 50.0f);
    proj[1][1] *= -1.0f;

    SceneUbo scene{};
    scene.model = model;
    scene.viewProj = proj * view;
    scene.lightDir = glm::vec4(glm::normalize(glm::vec3(0.35f, 0.55f, 0.75f)), 0.0f);

    std::memcpy(m_sceneBuffersMapped[m_currentFrame], &scene, sizeof(scene));

    vkResetFences(m_device->logical(), 1, &m_inFlightFences[m_currentFrame]);
    vkResetCommandBuffer(m_commandBuffers[imageIndex], 0);
    recordCommandBuffer(m_commandBuffers[imageIndex], imageIndex);

    VkSemaphore waitSemaphores[] = {m_imageAvailableSemaphores[m_currentFrame]};
    VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffers[imageIndex];

    VkSemaphore signalSemaphores[] = {m_renderFinishedSemaphores[imageIndex]};
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    if (vkQueueSubmit(m_device->graphicsQueue(), 1, &submitInfo, m_inFlightFences[m_currentFrame]) !=
        VK_SUCCESS) {
        throw std::runtime_error("Failed to submit draw command buffer");
    }

    m_imagesInFlight[imageIndex] = m_inFlightFences[m_currentFrame];

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    VkSwapchainKHR swapchain = m_swapchain->handle();
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain;
    presentInfo.pImageIndices = &imageIndex;

    VkResult presentResult = vkQueuePresentKHR(m_device->presentQueue(), &presentInfo);

    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR ||
        m_framebufferResized) {
        m_framebufferResized = false;
        recreateSwapchain();
    } else if (presentResult != VK_SUCCESS) {
        throw std::runtime_error("Failed to present swapchain image");
    }

    m_currentFrame = (m_currentFrame + 1) % kMaxFramesInFlight;
}

void VulkanRenderer::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("Failed to begin recording command buffer");
    }

    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = m_swapchain->renderPass();
    renderPassInfo.framebuffer = m_swapchain->framebuffers()[imageIndex];
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = m_swapchain->extent();

    VkClearValue clearValues[2]{};
    clearValues[0].color = {{0.05f, 0.05f, 0.08f, 1.0f}};
    clearValues[1].depthStencil = {1.0f, 0};
    renderPassInfo.clearValueCount = 2;
    renderPassInfo.pClearValues = clearValues;

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline->handle());

    VkBuffer vertexBuffers[] = {m_vertexBuffer->handle()};
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
    vkCmdBindIndexBuffer(commandBuffer, m_indexBuffer->handle(), 0, VK_INDEX_TYPE_UINT32);

    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        m_pipeline->layout(),
        0,
        1,
        &m_descriptorSets[m_currentFrame],
        0,
        nullptr);

    vkCmdDrawIndexed(commandBuffer, m_indexCount, 1, 0, 0, 0);
    vkCmdEndRenderPass(commandBuffer);

    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to record command buffer");
    }
}

} // namespace ge
