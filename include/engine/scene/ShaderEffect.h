#pragma once

#include <string>
#include <memory>
#include <vector>

namespace ge {

class VulkanPipeline;
class VulkanDevice;
class VulkanSwapchain;

/**
 * @brief Bundles shaders and pipeline state configurations.
 */
class ShaderEffect {
public:
    struct Config {
        std::string vertexShaderPath;
        std::string fragmentShaderPath;
        bool depthTest = true;
        bool depthWrite = true;
        bool transparent = false;
        // Add more state like cull mode here later
    };

    ShaderEffect(VulkanDevice& device, VulkanSwapchain& swapchain, const Config& config);
    ~ShaderEffect();

    [[nodiscard]] VulkanPipeline& getPipeline() const { return *pipeline_; }
    [[nodiscard]] const Config& getConfig() const { return config_; }

private:
    std::unique_ptr<VulkanPipeline> pipeline_;
    Config config_;
};

} // namespace ge
