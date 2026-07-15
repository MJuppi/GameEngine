#include "engine/mesh/Material.h"
#include <algorithm>

namespace ge {

Material makeDefaultMaterial(const std::string& name) {
    Material m;
    m.name = name;
    return m;
}

void fillMaterialBuffer(const std::vector<Material>& materials, MaterialBufferObject& out) {
    const uint32_t count = static_cast<uint32_t>(std::min(materials.size(), (size_t)kMaxGpuMaterials));

    for (uint32_t i = 0; i < count; ++i) {
        const auto& src = materials[i];
        auto& dst = out.materials[i];
        dst.diffuse = glm::vec4(src.diffuse, src.alpha);
        dst.specular = glm::vec4(src.specular, src.shininess);
        dst.hasTexture = src.texturePath.empty() ? 0 : 1;
    }

    for (uint32_t i = count; i < kMaxGpuMaterials; ++i) {
        out.materials[i] = out.materials[0];
    }
}

} // namespace ge
