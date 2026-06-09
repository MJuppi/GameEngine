#include "engine/mesh/GltfMeshLoader.h"

#include "engine/mesh/Material.h"

#include <cgltf.h>
#include <glm/glm.hpp>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <numeric>

namespace ge {

namespace {

std::string cgltfResultToString(cgltf_result result) {
    switch (result) {
        case cgltf_result_success: return "success";
        case cgltf_result_data_too_short: return "data too short";
        case cgltf_result_unknown_format: return "unknown format";
        case cgltf_result_invalid_json: return "invalid JSON";
        case cgltf_result_invalid_gltf: return "invalid glTF";
        case cgltf_result_invalid_options: return "invalid options";
        case cgltf_result_file_not_found: return "file not found";
        case cgltf_result_io_error: return "I/O error";
        case cgltf_result_out_of_memory: return "out of memory";
        case cgltf_result_legacy_gltf: return "legacy glTF";
        default: return "unknown cgltf error";
    }
}

std::vector<glm::vec3> extractVec3Attribute(const cgltf_accessor* accessor) {
    if (accessor == nullptr) {
        throw std::runtime_error("Missing accessor for vec3 attribute");
    }

    if (cgltf_num_components(accessor->type) != 3) {
        throw std::runtime_error("Expected vec3 accessor data");
    }

    const cgltf_size elementCount = accessor->count * 3;
    std::vector<float> rawData(elementCount);
    const cgltf_size readCount = cgltf_accessor_unpack_floats(accessor, rawData.data(), elementCount);
    if (readCount != elementCount) {
        throw std::runtime_error("Failed to unpack vec3 accessor data");
    }

    std::vector<glm::vec3> result;
    result.reserve(accessor->count);
    for (cgltf_size index = 0; index < accessor->count; ++index) {
        const cgltf_size offset = index * 3;
        result.emplace_back(rawData[offset], rawData[offset + 1], rawData[offset + 2]);
    }

    return result;
}

std::vector<uint32_t> extractIndices(const cgltf_accessor* accessor) {
    if (accessor == nullptr) {
        throw std::runtime_error("Missing accessor for index data");
    }

    std::vector<uint32_t> result(accessor->count);
    const cgltf_size readCount = cgltf_accessor_unpack_indices(accessor, result.data(), sizeof(uint32_t), accessor->count);
    if (readCount != accessor->count) {
        throw std::runtime_error("Failed to unpack index accessor data");
    }

    return result;
}

std::vector<glm::vec3> computeNormals(
    const std::vector<glm::vec3>& positions,
    const std::vector<uint32_t>& indices)
{
    std::vector<glm::vec3> normals(positions.size(), glm::vec3(0.0f));

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const uint32_t i0 = indices[i];
        const uint32_t i1 = indices[i + 1];
        const uint32_t i2 = indices[i + 2];

        const glm::vec3& p0 = positions[i0];
        const glm::vec3& p1 = positions[i1];
        const glm::vec3& p2 = positions[i2];

        const glm::vec3 faceNormal = glm::cross(p1 - p0, p2 - p0);
        normals[i0] += faceNormal;
        normals[i1] += faceNormal;
        normals[i2] += faceNormal;
    }

    for (auto& normal : normals) {
        const float length = glm::length(normal);
        if (length > 0.0001f) {
            normal /= length;
        } else {
            normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    return normals;
}

void ensureMaterialIndex(MeshData& result, uint32_t index) {
    while (result.materials.size() <= index) {
        result.materials.push_back(makeDefaultMaterial("material_" + std::to_string(result.materials.size())));
    }
}

void fillMaterialFromCgltf(MeshData& result, uint32_t materialIndex, const cgltf_material* material) {
    if (!material) {
        return;
    }

    ensureMaterialIndex(result, materialIndex);
    if (material->name && result.materials[materialIndex].name.empty()) {
        result.materials[materialIndex].name = material->name;
    }

    if (material->has_pbr_metallic_roughness) {
        const auto& pbr = material->pbr_metallic_roughness;
        result.materials[materialIndex].diffuse[0] = pbr.base_color_factor[0];
        result.materials[materialIndex].diffuse[1] = pbr.base_color_factor[1];
        result.materials[materialIndex].diffuse[2] = pbr.base_color_factor[2];
    }
}

void processPrimitive(const cgltf_primitive* primitive, const cgltf_data* data, MeshData& result) {
    if (!primitive) {
        return;
    }

    const cgltf_accessor* posAccessor = cgltf_find_accessor(primitive, cgltf_attribute_type_position, 0);
    if (!posAccessor) {
        return;
    }

    std::vector<glm::vec3> positions = extractVec3Attribute(posAccessor);
    std::vector<glm::vec3> normals;
    const cgltf_accessor* normalsAccessor = cgltf_find_accessor(primitive, cgltf_attribute_type_normal, 0);
    if (normalsAccessor) {
        normals = extractVec3Attribute(normalsAccessor);
    } else {
        normals.resize(positions.size(), glm::vec3(0.0f, 1.0f, 0.0f));
    }

    std::vector<uint32_t> indices;
    if (primitive->indices) {
        indices = extractIndices(primitive->indices);
    } else {
        indices.resize(positions.size());
        std::iota(indices.begin(), indices.end(), 0u);
    }

    if (!normalsAccessor && !indices.empty()) {
        normals = computeNormals(positions, indices);
    }

    if (normals.size() != positions.size()) {
        normals.resize(positions.size(), glm::vec3(0.0f, 1.0f, 0.0f));
    }

    uint32_t materialIndex = 0;
    if (primitive->material) {
        materialIndex = static_cast<uint32_t>(cgltf_material_index(data, primitive->material));
        fillMaterialFromCgltf(result, materialIndex, primitive->material);
    }

    const uint32_t vertexOffset = static_cast<uint32_t>(result.vertices.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        Vertex vertex;
        vertex.position[0] = positions[i].x;
        vertex.position[1] = positions[i].y;
        vertex.position[2] = positions[i].z;
        vertex.normal[0] = normals[i].x;
        vertex.normal[1] = normals[i].y;
        vertex.normal[2] = normals[i].z;
        vertex.materialIndex = materialIndex;
        result.vertices.push_back(vertex);
    }

    for (uint32_t index : indices) {
        result.indices.push_back(vertexOffset + index);
    }
}

void processNode(const cgltf_node* node, const cgltf_data* data, MeshData& result) {
    if (!node) {
        return;
    }

    if (node->mesh) {
        for (cgltf_size primitiveIndex = 0; primitiveIndex < node->mesh->primitives_count; ++primitiveIndex) {
            processPrimitive(&node->mesh->primitives[primitiveIndex], data, result);
        }
    }

    for (cgltf_size childIndex = 0; childIndex < node->children_count; ++childIndex) {
        processNode(node->children[childIndex], data, result);
    }
}

} // namespace

MeshData loadGltfFile(const std::string& path) {
    cgltf_options options = {};
    cgltf_data* data = nullptr;

    cgltf_result parseResult = cgltf_parse_file(&options, path.c_str(), &data);
    if (parseResult != cgltf_result_success) {
        throw std::runtime_error("Failed to parse glTF file: " + path + " (" + cgltfResultToString(parseResult) + ")");
    }

    cgltf_result bufferResult = cgltf_load_buffers(&options, data, path.c_str());
    if (bufferResult != cgltf_result_success) {
        cgltf_free(data);
        throw std::runtime_error("Failed to load glTF buffers: " + path + " (" + cgltfResultToString(bufferResult) + ")");
    }

    if (!data->scene) {
        if (data->scenes_count == 0) {
            cgltf_free(data);
            throw std::runtime_error("glTF file contains no scenes: " + path);
        }
        data->scene = &data->scenes[0];
    }

    MeshData result;
    result.materials = {makeDefaultMaterial("default")};

    for (cgltf_size nodeIndex = 0; nodeIndex < data->scene->nodes_count; ++nodeIndex) {
        processNode(data->scene->nodes[nodeIndex], data, result);
    }

    cgltf_free(data);

    if (result.vertices.empty() || result.indices.empty()) {
        throw std::runtime_error("glTF file contains no drawable geometry: " + path);
    }

    return result;
}

} // namespace ge
