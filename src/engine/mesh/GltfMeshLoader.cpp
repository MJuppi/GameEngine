#define CGLTF_IMPLEMENTATION
#include "engine/mesh/GltfMeshLoader.h"
#include <cgltf.h>
#include <numeric>
#include <stdexcept>

namespace ge {

namespace {
std::vector<float> unpack(const cgltf_accessor* a) {
    if (!a) return {};
    std::vector<float> r(a->count * cgltf_num_components(a->type));
    cgltf_accessor_unpack_floats(a, r.data(), r.size());
    return r;
}

void processPrimitive(const cgltf_primitive* p, const cgltf_data* d, MeshData& mesh) {
    auto pos = unpack(cgltf_find_accessor(p, cgltf_attribute_type_position, 0));
    auto nrm = unpack(cgltf_find_accessor(p, cgltf_attribute_type_normal, 0));
    auto uv = unpack(cgltf_find_accessor(p, cgltf_attribute_type_texcoord, 0));

    uint32_t matIdx = 0;
    if (p->material) {
        matIdx = (uint32_t)cgltf_material_index(d, p->material);
        while (mesh.materials.size() <= matIdx) mesh.materials.push_back(makeDefaultMaterial());
        auto& m = mesh.materials[matIdx];
        if (p->material->name) m.name = p->material->name;
        if (p->material->has_pbr_metallic_roughness) {
            auto& pbr = p->material->pbr_metallic_roughness;
            m.diffuse = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
        }
    }

    uint32_t offset = (uint32_t)mesh.vertices.size();
    for (size_t i = 0; i < pos.size() / 3; ++i) {
        Vertex v{};
        for (int j=0; j<3; ++j) v.position[j] = pos[i*3+j];
        if (!nrm.empty()) for (int j=0; j<3; ++j) v.normal[j] = nrm[i*3+j];
        if (!uv.empty()) { v.texCoord[0] = uv[i*2]; v.texCoord[1] = uv[i*2+1]; }
        v.materialIndex = matIdx;
        mesh.vertices.push_back(v);
    }

    if (p->indices) {
        std::vector<uint32_t> idx(p->indices->count);
        cgltf_accessor_unpack_indices(p->indices, idx.data(), sizeof(uint32_t), idx.size());
        for (auto i : idx) mesh.indices.push_back(offset + i);
    } else {
        for (uint32_t i = 0; i < (uint32_t)pos.size() / 3; ++i) mesh.indices.push_back(offset + i);
    }
}

void processNode(const cgltf_node* n, const cgltf_data* d, MeshData& mesh) {
    if (n->mesh) for (size_t i=0; i<n->mesh->primitives_count; ++i) processPrimitive(&n->mesh->primitives[i], d, mesh);
    for (size_t i=0; i<n->children_count; ++i) processNode(n->children[i], d, mesh);
}
} // namespace

MeshData loadGltfFile(const std::string& path) {
    cgltf_options opt = {};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&opt, path.c_str(), &data) != cgltf_result_success) throw std::runtime_error("No glTF: " + path);
    if (cgltf_load_buffers(&opt, data, path.c_str()) != cgltf_result_success) { cgltf_free(data); throw std::runtime_error("No buffers: " + path); }

    MeshData mesh;
    mesh.materials = {makeDefaultMaterial()};
    auto* scene = data->scene ? data->scene : &data->scenes[0];
    for (size_t i=0; i<scene->nodes_count; ++i) processNode(scene->nodes[i], data, mesh);
    cgltf_free(data);
    return mesh;
}

} // namespace ge
