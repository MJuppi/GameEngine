#include "engine/mesh/Material.h"

#include <algorithm>

namespace ge {

Material makeDefaultMaterial(const std::string& name) {
    Material m;
    m.name = name;
    return m;
}

void fillMaterialBuffer(const std::vector<Material>& materials, MaterialBufferObject& out) {
    const uint32_t count =
        static_cast<uint32_t>(std::min(materials.size(), size_t{kMaxGpuMaterials}));

    for (uint32_t i = 0; i < count; ++i) {
        const Material& src = materials[i];
        GpuMaterial& dst = out.materials[i];
        dst.diffuse[0] = src.diffuse[0];
        dst.diffuse[1] = src.diffuse[1];
        dst.diffuse[2] = src.diffuse[2];
        dst.diffuse[3] = src.alpha;
        dst.specular[0] = src.specular[0];
        dst.specular[1] = src.specular[1];
        dst.specular[2] = src.specular[2];
        dst.specular[3] = src.shininess;
    }

    for (uint32_t i = count; i < kMaxGpuMaterials; ++i) {
        out.materials[i] = out.materials[0];
    }
}

} // namespace ge
