#pragma once

// =============================================================================
// VulkanRenderer.h — Frame loop: sync, command buffers, draw calls
// =============================================================================
// Owns everything needed to render one frame: acquire swapchain image, record
// commands, submit to GPU, present. Also holds mesh buffers and descriptor sets.
// =============================================================================

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "engine/mesh/Material.h"
#include "engine/mesh/MeshData.h"
#include "engine/asset/FontData.h"
#include "engine/vulkan/UiVertex.h"

struct GLFWwindow;

namespace ge {

class Window;
class VulkanContext;
class VulkanDevice;
class VulkanSwapchain;
class VulkanPipeline;
class VulkanBuffer;
class VulkanFont;

/// Scene uniform — must match GLSL SceneUbo in basic.vert / basic.frag.
struct SceneUbo {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 viewProj;
    alignas(16) glm::mat4 normalMatrix;
    alignas(16) glm::vec4 lightDir;
};

class VulkanRenderer {
public:
    VulkanRenderer(Window& window, MeshData mesh);
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;

    void drawFrame();
    void onFramebufferResize();

private:
    void initVulkan();
    void createCommandPool();
    void createCommandBuffers();
    void createSyncObjects();
    void destroySyncObjects();
    void createMeshBuffers();
    void createSceneBuffers();
    void createMaterialBuffer();
    void createDescriptorPool();
    void createDescriptorSets();
    void createUiPipeline();
    void createFontSystem();
    void createUiBuffers();
    void updateUiVertexBuffer(
        uint32_t width,
        uint32_t height,
        float fps,
        float minFrameTimeMs,
        float maxFrameTimeMs);
    void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);

    void recreateSwapchain();
    void cleanupSwapchain();
    void updateCamera(float deltaTime);
    void resetCamera(const MeshBounds& bounds);
    void updateCameraVectors();

    Window& m_window;
    MeshData m_mesh;
    std::string m_shaderDir;

    std::unique_ptr<VulkanContext> m_context;
    std::unique_ptr<VulkanDevice> m_device;
    std::unique_ptr<VulkanSwapchain> m_swapchain;
    std::unique_ptr<VulkanPipeline> m_pipeline;
    std::unique_ptr<VulkanPipeline> m_uiPipeline;
    std::unique_ptr<VulkanFont> m_font;

    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> m_commandBuffers;

    std::unique_ptr<VulkanBuffer> m_vertexBuffer;
    std::unique_ptr<VulkanBuffer> m_indexBuffer;
    uint32_t m_indexCount = 0;

    static constexpr uint32_t kMaxUiVertices = 16384;

    std::unique_ptr<VulkanBuffer> m_uiVertexBuffer;
    uint32_t m_uiVertexCount = 0;

    std::vector<std::unique_ptr<VulkanBuffer>> m_sceneBuffers;
    std::vector<void*> m_sceneBuffersMapped;

    std::unique_ptr<VulkanBuffer> m_materialBuffer;
    MaterialBufferObject m_materialGpu{};

    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    std::vector<VkDescriptorSet> m_descriptorSets;
    std::vector<VkDescriptorSet> m_fontDescriptorSets;

    // Per in-flight frame: acquire + fence. Per swapchain image: present semaphore.
    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
    std::vector<VkFence> m_imagesInFlight;

    static constexpr int kMaxFramesInFlight = 2;
    size_t m_currentFrame = 0;
    bool m_framebufferResized = false;
    float m_meshRadius = 1.0f;

    float m_minFrameTimeMs = std::numeric_limits<float>::infinity();
    float m_maxFrameTimeMs = 0.0f;
    double m_frameTimeWindowStart = 0.0;

    glm::vec3 m_cameraPosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 m_cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 m_cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 m_cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 m_cameraWorldUp = glm::vec3(0.0f, 1.0f, 0.0f);

    float m_cameraYaw = -90.0f;
    float m_cameraPitch = 0.0f;
    float m_cameraSpeed = 5.0f;
    float m_mouseSensitivity = 0.15f;

    double m_lastCursorX = 0.0;
    double m_lastCursorY = 0.0;
    bool m_firstMouse = true;
    bool m_mouseCaptured = false;
    double m_lastFrameTime = 0.0;

    double m_lastUiUpdateTime = 0.0;
    float m_lastRenderedFps = 0.0f;
    float m_lastRenderedMinFrameTimeMs = 0.0f;
    float m_lastRenderedMaxFrameTimeMs = 0.0f;
};

} // namespace ge
