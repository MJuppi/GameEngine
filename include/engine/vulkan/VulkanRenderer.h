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
#include <map>

#include "engine/mesh/Material.h"
#include "engine/mesh/MeshData.h"
#include "engine/scene/Light.h"
#include "engine/scene/MaterialSystem.h"
#include "engine/vulkan/VulkanImage.h"

struct GLFWwindow;

namespace ge {

class Window;
class VulkanContext;
class VulkanDevice;
class VulkanSwapchain;
class VulkanPipeline;
class VulkanBuffer;
class MaterialSystem;

struct RenderObject {
    glm::mat4 transform;
    std::shared_ptr<Material> material;
    const MeshData* mesh = nullptr;
};

/// Scene uniform — must match GLSL SceneUbo in basic.vert / basic.frag.
struct SceneUbo {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 viewProj;
    alignas(16) glm::mat4 normalMatrix;
    alignas(16) glm::vec4 lightDir;
    alignas(16) glm::vec4 cameraPos;
    alignas(16) PointLight pointLight;
};

class VulkanRenderer {
public:
    VulkanRenderer(Window& window, MeshData mesh);
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;

    void drawFrame();
    void onFramebufferResize();
    void setModelMatrix(const glm::mat4& model) { m_modelMatrix = model; }

    void setCamera(const glm::vec3& position, const glm::vec3& front, const glm::vec3& up) {
        m_cameraPosition = position;
        m_cameraFront = front;
        m_cameraUp = up;
    }

    void addDynamicObject(const glm::mat4& transform, std::shared_ptr<Material> material, const MeshData* mesh = nullptr);
    void clearDynamicObjects() { m_dynamicObjects.clear(); }

    void setPointLight(const PointLight& pointLight) { m_pointLight = pointLight; }
    [[nodiscard]] glm::vec3 getCameraPosition() const { return m_cameraPosition; }
    [[nodiscard]] glm::vec3 getCameraFront() const { return m_cameraFront; }

    MaterialSystem& getMaterialSystem() { return *m_materialSystem; }

private:
    void initVulkan();
    void createCommandPool();
    void createCommandBuffers();
    void createSyncObjects();
    void destroySyncObjects();
    void createMeshBuffers();
    void createBoxMeshBuffers();
    void createSceneBuffers();
    void createMaterialBuffer();
    void createTextureImage();
    void createDescriptorPool();
    void createDescriptorSets();
    void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);

    struct MeshBuffers {
        std::unique_ptr<VulkanBuffer> vb;
        std::unique_ptr<VulkanBuffer> ib;
        uint32_t indexCount = 0;
    };

    void recreateSwapchain();
    void cleanupSwapchain();

    Window& m_window;
    MeshData m_mesh;
    std::string m_shaderDir;

    std::unique_ptr<VulkanContext> m_context;
    std::unique_ptr<VulkanDevice> m_device;
    std::unique_ptr<VulkanSwapchain> m_swapchain;

    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> m_commandBuffers;

    std::unique_ptr<VulkanBuffer> m_vertexBuffer;
    std::unique_ptr<VulkanBuffer> m_indexBuffer;
    uint32_t m_indexCount = 0;
    std::unique_ptr<VulkanBuffer> m_boxVertexBuffer;
    std::unique_ptr<VulkanBuffer> m_boxIndexBuffer;
    uint32_t m_boxIndexCount = 0;

    std::vector<std::unique_ptr<VulkanBuffer>> m_sceneBuffers;
    std::vector<void*> m_sceneBuffersMapped;

    std::unique_ptr<VulkanBuffer> m_materialBuffer;
    MaterialBufferObject m_materialGpu{};

    VulkanImage m_texture;

    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    std::vector<VkDescriptorSet> m_descriptorSets;

    // Per in-flight frame: acquire + fence. Per swapchain image: present semaphore.
    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
    std::vector<VkFence> m_imagesInFlight;

    static constexpr int kMaxFramesInFlight = 2;
    size_t m_currentFrame = 0;
    bool m_framebufferResized = false;

    PointLight m_pointLight{};

    glm::vec3 m_cameraPosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 m_cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 m_cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

    glm::mat4 m_modelMatrix = glm::mat4(1.0f);
    std::vector<RenderObject> m_dynamicObjects;
    std::map<const MeshData*, MeshBuffers> m_dynamicMeshCache;
    MeshData m_boxMesh{};
    std::unique_ptr<MaterialSystem> m_materialSystem;
};

} // namespace ge
