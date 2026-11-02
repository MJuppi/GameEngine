#include "engine/mesh/Material.h"
#include "engine/vulkan/RenderSupport.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testDescriptorPoolSizesAreAggregated() {
    const auto sizes = ge::render::rendererDescriptorPoolSizes(2);
    require(sizes.size() == 2, "Descriptor pool entries must have unique descriptor types");
    require(sizes[0].type == VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            "The first descriptor pool entry must be uniform buffers");
    require(sizes[0].descriptorCount == 8,
            "Two uniform buffers per set are required for each of four sets");
    require(sizes[1].type == VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            "The second descriptor pool entry must be combined image samplers");
    require(sizes[1].descriptorCount == 64,
            "Each of four sets must reserve all sixteen texture slots");
}

void testProjectionUsesVulkanDepthRange() {
    const glm::mat4 projection = ge::render::vulkanPerspective(
        glm::radians(70.0f), 1.5f, 0.1f, 100.0f);
    const glm::vec4 nearClip = projection * glm::vec4(0.0f, 0.0f, -0.1f, 1.0f);
    const glm::vec4 farClip = projection * glm::vec4(0.0f, 0.0f, -100.0f, 1.0f);

    require(std::fabs(nearClip.z / nearClip.w) < 1e-5f,
            "The near plane must map to Vulkan NDC depth zero");
    require(std::fabs(farClip.z / farClip.w - 1.0f) < 1e-5f,
            "The far plane must map to Vulkan NDC depth one");
}

void testFontPixelLookupStaysWithinFiveGlyphColumns() {
    require(ge::render::fontPixel(0x10, 0), "Font bit 4 must map to the first glyph column");
    require(ge::render::fontPixel(0x01, 4), "Font bit 0 must map to the fifth glyph column");
    require(!ge::render::fontPixel(0xFF, 5), "The sixth glyph column must remain blank");
    require(!ge::render::fontPixel(0xFF, 7), "The eighth glyph column must remain blank");
}

void testTextureFormatAndByteCountSupportHdr() {
    ge::TextureData hdr;
    hdr.width = 2;
    hdr.height = 1;
    hdr.channels = 3;
    hdr.hdr = true;
    hdr.pixels.resize(2 * 1 * 3 * sizeof(float));

    require(ge::render::textureFormat(hdr) == VK_FORMAT_R32G32B32_SFLOAT,
            "RGB HDR data must use a 32-bit floating-point Vulkan format");
    require(ge::render::textureByteCount(hdr) == hdr.pixels.size(),
            "HDR staging size must account for four bytes per channel");

    hdr.pixels.pop_back();
    bool rejected = false;
    try {
        (void)ge::render::textureFormat(hdr);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Texture data with a mismatched byte count must be rejected");
}

void testMaterialBufferPreservesEveryMaterial() {
    std::vector<ge::Material> materials(2);
    materials[0].diffuse = {1.0f, 0.0f, 0.0f};
    materials[0].alpha = 0.5f;
    materials[1].diffuse = {0.0f, 1.0f, 0.0f};
    materials[1].specular = {0.1f, 0.2f, 0.3f};
    materials[1].shininess = 64.0f;
    materials[1].texturePath = "second.png";

    ge::MaterialBufferObject gpuMaterials{};
    ge::fillMaterialBuffer(materials, gpuMaterials);
    require(gpuMaterials.materials[0].diffuse.a == 0.5f,
            "Material alpha must be uploaded");
    require(gpuMaterials.materials[1].diffuse.g == 1.0f,
            "Every material must retain its own diffuse color");
    require(gpuMaterials.materials[1].hasTexture == 1,
            "Textured materials must retain their texture flag");
}

} // namespace

int main() {
    try {
        testDescriptorPoolSizesAreAggregated();
        testProjectionUsesVulkanDepthRange();
        testFontPixelLookupStaysWithinFiveGlyphColumns();
        testTextureFormatAndByteCountSupportHdr();
        testMaterialBufferPreservesEveryMaterial();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
