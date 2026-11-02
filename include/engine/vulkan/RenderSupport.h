#pragma once

#include "engine/asset/TextureData.h"
#include "engine/mesh/Material.h"

#include <vulkan/vulkan.h>
#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace ge::render {

inline constexpr uint32_t kMaterialTextureSlots = kMaxGpuMaterials;

inline std::array<VkDescriptorPoolSize, 2> rendererDescriptorPoolSizes(uint32_t framesInFlight) {
    return {{
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, framesInFlight * 4},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, framesInFlight * 2 * kMaterialTextureSlots}
    }};
}

inline glm::mat4 vulkanPerspective(float fovRadians, float aspect, float nearPlane, float farPlane) {
    glm::mat4 projection = glm::perspectiveRH_ZO(fovRadians, aspect, nearPlane, farPlane);
    projection[1][1] *= -1.0f;
    return projection;
}

inline bool fontPixel(uint8_t row, int column) {
    return column >= 0 && column < 5 && (row & (1u << (4 - column))) != 0;
}

inline size_t textureByteCount(const TextureData& data) {
    if (data.width <= 0 || data.height <= 0 || data.channels < 1 || data.channels > 4) {
        throw std::invalid_argument("Texture dimensions and channel count must be valid");
    }

    const size_t bytesPerChannel = data.hdr ? sizeof(float) : sizeof(uint8_t);
    const size_t width = static_cast<size_t>(data.width);
    const size_t height = static_cast<size_t>(data.height);
    const size_t channels = static_cast<size_t>(data.channels);
    if (width > SIZE_MAX / height || width * height > SIZE_MAX / channels ||
        width * height * channels > SIZE_MAX / bytesPerChannel) {
        throw std::invalid_argument("Texture byte count overflows size_t");
    }

    const size_t expectedSize = width * height * channels * bytesPerChannel;
    if (data.pixels.size() != expectedSize) {
        throw std::invalid_argument("Texture pixel data size does not match its dimensions");
    }
    return expectedSize;
}

inline VkFormat textureFormat(const TextureData& data) {
    (void)textureByteCount(data);
    if (data.hdr) {
        constexpr VkFormat formats[] = {
            VK_FORMAT_UNDEFINED,
            VK_FORMAT_R32_SFLOAT,
            VK_FORMAT_R32G32_SFLOAT,
            VK_FORMAT_R32G32B32_SFLOAT,
            VK_FORMAT_R32G32B32A32_SFLOAT
        };
        return formats[data.channels];
    }

    constexpr VkFormat formats[] = {
        VK_FORMAT_UNDEFINED,
        VK_FORMAT_R8_UNORM,
        VK_FORMAT_R8G8_UNORM,
        VK_FORMAT_R8G8B8_UNORM,
        VK_FORMAT_R8G8B8A8_SRGB
    };
    return formats[data.channels];
}

} // namespace ge::render
