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
#include <iostream>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include "engine/asset/TextureLoader.h"
#include "engine/asset/AssetLoader.h"

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
    , m_boxMesh(makeUnitCubeMesh())
{
    flipMeshWinding(m_boxMesh);
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

    if (m_indexBuffer) m_indexBuffer->destroy(*m_device);
    if (m_vertexBuffer) m_vertexBuffer->destroy(*m_device);

    for (auto& ub : m_sceneBuffers) {
        if (ub) ub->destroy(*m_device);
    }
    if (m_materialBuffer) m_materialBuffer->destroy(*m_device);

    for (auto& [mesh, buffers] : m_dynamicMeshCache) {
        if (buffers.vb) buffers.vb->destroy(*m_device);
        if (buffers.ib) buffers.ib->destroy(*m_device);
    }
    m_dynamicMeshCache.clear();

    m_texture.destroy(*m_device);
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

    m_materialSystem = std::make_unique<MaterialSystem>(*m_device, *m_swapchain);

    createCommandPool();
    createMeshBuffers();
    createSceneBuffers();
    createMaterialBuffer();
    createTextureImage();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();
    std::cout << "[VulkanRenderer] initVulkan completed." << std::endl;
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

    VulkanBuffer stagingVertex;
    stagingVertex.create(*m_device, vertexSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(*m_device, stagingVertex.handle(), stagingVertex.memory(), m_mesh.vertices.data(), vertexSize);

    m_vertexBuffer = std::make_unique<VulkanBuffer>();
    m_vertexBuffer->create(*m_device, vertexSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VulkanBuffer::copyBuffer(*m_device, m_commandPool, m_device->graphicsQueue(), stagingVertex.handle(), m_vertexBuffer->handle(), vertexSize);
    stagingVertex.destroy(*m_device);

    VulkanBuffer stagingIndex;
    stagingIndex.create(*m_device, indexSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(*m_device, stagingIndex.handle(), stagingIndex.memory(), m_mesh.indices.data(), indexSize);

    m_indexBuffer = std::make_unique<VulkanBuffer>();
    m_indexBuffer->create(*m_device, indexSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VulkanBuffer::copyBuffer(*m_device, m_commandPool, m_device->graphicsQueue(), stagingIndex.handle(), m_indexBuffer->handle(), indexSize);
    stagingIndex.destroy(*m_device);
}

void VulkanRenderer::createSceneBuffers() {
    const VkDeviceSize bufferSize = sizeof(SceneUbo);
    m_sceneBuffers.resize(kMaxFramesInFlight);
    m_sceneBuffersMapped.resize(kMaxFramesInFlight);

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_sceneBuffers[i] = std::make_unique<VulkanBuffer>();
        m_sceneBuffers[i]->create(*m_device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkMapMemory(m_device->logical(), m_sceneBuffers[i]->memory(), 0, bufferSize, 0, &m_sceneBuffersMapped[i]);
    }
}

void VulkanRenderer::createMaterialBuffer() {
    fillMaterialBuffer(m_mesh.materials, m_materialGpu);
    m_materialBuffer = std::make_unique<VulkanBuffer>();
    m_materialBuffer->create(*m_device, sizeof(MaterialBufferObject), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VulkanBuffer::write(*m_device, m_materialBuffer->handle(), m_materialBuffer->memory(), &m_materialGpu, sizeof(MaterialBufferObject));
}

void VulkanRenderer::createTextureImage() {
    std::string texturePath = "textures/grid_texture.png";
    for (const auto& mat : m_mesh.materials) {
        if (!mat.texturePath.empty()) {
            texturePath = mat.texturePath;
            break;
        }
    }

    try {
        const auto resolved = AssetLoader::resolveAssetPath(texturePath);
        TextureData data = loadTextureFile(resolved.string());
        m_texture = VulkanImage::fromTextureData(*m_device, m_commandPool, m_device->graphicsQueue(), data);
    } catch (...) {
        TextureData dummy;
        dummy.width = 2; dummy.height = 2; dummy.channels = 4;
        dummy.pixels = { 255, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255, 255 };
        m_texture = VulkanImage::fromTextureData(*m_device, m_commandPool, m_device->graphicsQueue(), dummy);
    }
}

void VulkanRenderer::createDescriptorPool() {
    std::array<VkDescriptorPoolSize, 3> poolSizes{};
    poolSizes[0] = { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, static_cast<uint32_t>(kMaxFramesInFlight) };
    poolSizes[1] = { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, static_cast<uint32_t>(kMaxFramesInFlight) };
    poolSizes[2] = { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, static_cast<uint32_t>(kMaxFramesInFlight) };

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 3;
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = static_cast<uint32_t>(kMaxFramesInFlight);

    if (vkCreateDescriptorPool(m_device->logical(), &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor pool");
    }
}

void VulkanRenderer::createDescriptorSets() {
    auto defaultEffect = m_materialSystem->createMaterial("temp").effect;
    std::vector<VkDescriptorSetLayout> layouts(kMaxFramesInFlight, defaultEffect->getPipeline().descriptorSetLayout());

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
        VkDescriptorBufferInfo sceneInfo{ m_sceneBuffers[i]->handle(), 0, sizeof(SceneUbo) };
        VkDescriptorBufferInfo materialInfo{ m_materialBuffer->handle(), 0, sizeof(MaterialBufferObject) };
        VkDescriptorImageInfo imageInfo{ m_texture.sampler(), m_texture.view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

        std::array<VkWriteDescriptorSet, 3> writes{};
        writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, m_descriptorSets[i], 0, 0, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, nullptr, &sceneInfo, nullptr };
        writes[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, m_descriptorSets[i], 1, 0, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, nullptr, &materialInfo, nullptr };
        writes[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, m_descriptorSets[i], 2, 0, 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &imageInfo, nullptr, nullptr };
        vkUpdateDescriptorSets(m_device->logical(), 3, writes.data(), 0, nullptr);
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
        vkCreateSemaphore(m_device->logical(), &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]);
        vkCreateFence(m_device->logical(), &fenceInfo, nullptr, &m_inFlightFences[i]);
    }
    for (size_t i = 0; i < imageCount; ++i) {
        vkCreateSemaphore(m_device->logical(), &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]);
    }
}

void VulkanRenderer::destroySyncObjects() {
    for (auto s : m_imageAvailableSemaphores) vkDestroySemaphore(m_device->logical(), s, nullptr);
    for (auto s : m_renderFinishedSemaphores) vkDestroySemaphore(m_device->logical(), s, nullptr);
    for (auto f : m_inFlightFences) vkDestroyFence(m_device->logical(), f, nullptr);
    m_imageAvailableSemaphores.clear();
    m_renderFinishedSemaphores.clear();
    m_inFlightFences.clear();
    m_imagesInFlight.clear();
}

void VulkanRenderer::onFramebufferResize() { m_framebufferResized = true; }

void VulkanRenderer::recreateSwapchain() {
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_window.handle(), &w, &h);
    while (w == 0 || h == 0) {
        glfwGetFramebufferSize(m_window.handle(), &w, &h);
        glfwWaitEvents();
    }
    vkDeviceWaitIdle(m_device->logical());
    destroySyncObjects();
    m_swapchain->recreate(m_window.handle());
    vkDestroyCommandPool(m_device->logical(), m_commandPool, nullptr);
    createCommandPool();
    createCommandBuffers();
    createSyncObjects();
}

void VulkanRenderer::cleanupSwapchain() { m_swapchain.reset(); }

void VulkanRenderer::drawFrame() {
    VkResult res;

    res = vkWaitForFences(m_device->logical(), 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);
    if (res != VK_SUCCESS) {
         std::cerr << "[VulkanRenderer] vkWaitForFences failed: " << res << std::endl;
    }

    uint32_t imageIndex = 0;
    res = vkAcquireNextImageKHR(m_device->logical(), m_swapchain->handle(), UINT64_MAX, m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

    if (res == VK_ERROR_OUT_OF_DATE_KHR || m_framebufferResized) {
        m_framebufferResized = false;
        recreateSwapchain();
        return;
    } else if (res != VK_SUCCESS && res != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swapchain image");
    }

    if (m_imagesInFlight[imageIndex] != VK_NULL_HANDLE) {
        vkWaitForFences(m_device->logical(), 1, &m_imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
    }
    m_imagesInFlight[imageIndex] = m_inFlightFences[m_currentFrame];

    const float aspect = m_swapchain->extent().width / static_cast<float>(m_swapchain->extent().height);

    SceneUbo scene{};
    scene.model = m_modelMatrix;
    glm::mat4 proj = glm::perspective(glm::radians(90.0f), aspect, 0.1f, 1000.0f);
    proj[1][1] *= -1.0f;
    scene.viewProj = proj * glm::lookAt(m_cameraPosition, m_cameraPosition + m_cameraFront, m_cameraUp);
    scene.normalMatrix = glm::transpose(glm::inverse(m_modelMatrix));
    scene.lightDir = m_sceneLights.directional.direction;
    scene.cameraPos = glm::vec4(m_cameraPosition, 1.0f);
    scene.lights = m_sceneLights;

    if (m_sceneBuffersMapped[m_currentFrame] != nullptr) {
        std::memcpy(m_sceneBuffersMapped[m_currentFrame], &scene, sizeof(scene));
    }

    vkResetFences(m_device->logical(), 1, &m_inFlightFences[m_currentFrame]);
    vkResetCommandBuffer(m_commandBuffers[imageIndex], 0);
    recordCommandBuffer(m_commandBuffers[imageIndex], imageIndex);

    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &m_imageAvailableSemaphores[m_currentFrame];
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffers[imageIndex];
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = &m_renderFinishedSemaphores[imageIndex];

    if (vkQueueSubmit(m_device->graphicsQueue(), 1, &submitInfo, m_inFlightFences[m_currentFrame]) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit draw command buffer");
    }

    VkSwapchainKHR swapChains[] = { m_swapchain->handle() };
    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &m_renderFinishedSemaphores[imageIndex];
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &imageIndex;

    res = vkQueuePresentKHR(m_device->presentQueue(), &presentInfo);

    if (res == VK_ERROR_OUT_OF_DATE_KHR || res == VK_SUBOPTIMAL_KHR || m_framebufferResized) {
        std::cout << "[VulkanRenderer] Present result: " << res << ", recreating swapchain..." << std::endl;
        m_framebufferResized = false;
        recreateSwapchain();
    } else if (res != VK_SUCCESS) {
        throw std::runtime_error("Failed to present swapchain image");
    }

    m_currentFrame = (m_currentFrame + 1) % kMaxFramesInFlight;
}

void VulkanRenderer::addDynamicObject(const glm::mat4& transform, std::shared_ptr<Material> material, const MeshData* mesh) {
    m_dynamicObjects.push_back({transform, material, mesh});
}

void VulkanRenderer::recordCommandBuffer(VkCommandBuffer cb, uint32_t idx) {
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    if (vkBeginCommandBuffer(cb, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("Failed to begin recording command buffer");
    }

    VkClearValue clears[2]{};
    clears[0].color = {{0.05f, 0.05f, 0.08f, 1.0f}};
    clears[1].depthStencil = {1.0f, 0};

    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = m_swapchain->renderPass();
    rpInfo.framebuffer = m_swapchain->framebuffers()[idx];
    rpInfo.renderArea.offset = {0, 0};
    rpInfo.renderArea.extent = m_swapchain->extent();
    rpInfo.clearValueCount = 2;
    rpInfo.pClearValues = clears;

    vkCmdBeginRenderPass(cb, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    auto defaultEffect = m_materialSystem->createMaterial("temp").effect;
    VkPipelineLayout layout = defaultEffect->getPipeline().layout();

    vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &m_descriptorSets[m_currentFrame], 0, nullptr);

    // Draw Static
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, defaultEffect->getPipeline().handle());
    if (m_vertexBuffer && m_indexBuffer) {
        VkBuffer vbs[] = { m_vertexBuffer->handle() };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(cb, 0, 1, vbs, offsets);
        vkCmdBindIndexBuffer(cb, m_indexBuffer->handle(), 0, VK_INDEX_TYPE_UINT32);

        struct { glm::mat4 m; glm::mat4 n; } push { m_modelMatrix, glm::transpose(glm::inverse(m_modelMatrix)) };
        vkCmdPushConstants(cb, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);
        vkCmdDrawIndexed(cb, m_indexCount, 1, 0, 0, 0);
    }

    // Draw Dynamic
    for (size_t i = 0; i < m_dynamicObjects.size(); ++i) {
        const auto& obj = m_dynamicObjects[i];
        const MeshData* mesh = obj.mesh ? obj.mesh : &m_boxMesh;

        // Ensure mesh is in cache
        auto it = m_dynamicMeshCache.find(mesh);
        if (it == m_dynamicMeshCache.end()) {
            if (mesh->vertices.empty() || mesh->indices.empty()) continue;

            MeshBuffers buffers;
            VkDeviceSize vSize = sizeof(Vertex) * mesh->vertices.size();
            VkDeviceSize iSize = sizeof(uint32_t) * mesh->indices.size();
            buffers.indexCount = (uint32_t)mesh->indices.size();

            VulkanBuffer sVb;
            sVb.create(*m_device, vSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            VulkanBuffer::write(*m_device, sVb.handle(), sVb.memory(), mesh->vertices.data(), vSize);
            buffers.vb = std::make_unique<VulkanBuffer>();
            buffers.vb->create(*m_device, vSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            VulkanBuffer::copyBuffer(*m_device, m_commandPool, m_device->graphicsQueue(), sVb.handle(), buffers.vb->handle(), vSize);
            sVb.destroy(*m_device);

            VulkanBuffer sIb;
            sIb.create(*m_device, iSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            VulkanBuffer::write(*m_device, sIb.handle(), sIb.memory(), mesh->indices.data(), iSize);
            buffers.ib = std::make_unique<VulkanBuffer>();
            buffers.ib->create(*m_device, iSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            VulkanBuffer::copyBuffer(*m_device, m_commandPool, m_device->graphicsQueue(), sIb.handle(), buffers.ib->handle(), iSize);
            sIb.destroy(*m_device);

            it = m_dynamicMeshCache.emplace(mesh, std::move(buffers)).first;
        }

        const auto& buf = it->second;
        auto effect = (obj.material && obj.material->effect) ? obj.material->effect : defaultEffect;
        if (!effect) continue;

        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, effect->getPipeline().handle());
        VkBuffer vbs[] = { buf.vb->handle() };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(cb, 0, 1, vbs, offsets);
        vkCmdBindIndexBuffer(cb, buf.ib->handle(), 0, VK_INDEX_TYPE_UINT32);

        struct { glm::mat4 m; glm::mat4 n; } push { obj.transform, glm::transpose(glm::inverse(obj.transform)) };
        vkCmdPushConstants(cb, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);
        vkCmdDrawIndexed(cb, buf.indexCount, 1, 0, 0, 0);
    }

    vkCmdEndRenderPass(cb);
    if (vkEndCommandBuffer(cb) != VK_SUCCESS) {
        throw std::runtime_error("Failed to record command buffer");
    }
}

} // namespace ge
