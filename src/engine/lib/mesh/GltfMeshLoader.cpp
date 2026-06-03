#include "engine/mesh/GltfMeshLoader.h"

#include "engine/mesh/Material.h"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYGLTF_IMPLEMENTATION
#include <tiny_gltf.h>

#include <glm/glm.hpp>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace ge {

namespace {

/// Extract a float attribute from a glTF accessor.
std::vector<glm::vec3> extractVec3Attribute(
    const tinygltf::Model& model,
    const tinygltf::Accessor& accessor)
{
    std::vector<glm::vec3> result;
    
    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
    
    const auto* data = reinterpret_cast<const float*>(
        buffer.data.data() + bufferView.byteOffset + accessor.byteOffset);
    
    const size_t stride = bufferView.byteStride > 0 
        ? bufferView.byteStride / sizeof(float)
        : 3;
    
    result.reserve(accessor.count);
    for (size_t i = 0; i < accessor.count; ++i) {
        const size_t offset = i * stride;
        result.push_back(glm::vec3(data[offset], data[offset + 1], data[offset + 2]));
    }
    
    return result;
}

/// Extract indices from a glTF accessor (supports uint8, uint16, uint32).
std::vector<uint32_t> extractIndices(
    const tinygltf::Model& model,
    const tinygltf::Accessor& accessor)
{
    std::vector<uint32_t> result;
    result.reserve(accessor.count);
    
    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
    const auto* basePtr = buffer.data.data() + bufferView.byteOffset + accessor.byteOffset;
    
    if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
        const auto* data = reinterpret_cast<const uint32_t*>(basePtr);
        const size_t stride = bufferView.byteStride > 0 
            ? bufferView.byteStride / sizeof(uint32_t)
            : 1;
        for (size_t i = 0; i < accessor.count; ++i) {
            result.push_back(data[i * stride]);
        }
    } else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
        const auto* data = reinterpret_cast<const uint16_t*>(basePtr);
        const size_t stride = bufferView.byteStride > 0 
            ? bufferView.byteStride / sizeof(uint16_t)
            : 1;
        for (size_t i = 0; i < accessor.count; ++i) {
            result.push_back(static_cast<uint32_t>(data[i * stride]));
        }
    } else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
        const auto* data = reinterpret_cast<const uint8_t*>(basePtr);
        const size_t stride = bufferView.byteStride > 0 
            ? bufferView.byteStride / sizeof(uint8_t)
            : 1;
        for (size_t i = 0; i < accessor.count; ++i) {
            result.push_back(static_cast<uint32_t>(data[i * stride]));
        }
    } else {
        throw std::runtime_error("Unsupported index component type");
    }
    
    return result;
}

/// Compute smooth normals if not provided by the glTF file.
std::vector<glm::vec3> computeNormals(
    const std::vector<glm::vec3>& positions,
    const std::vector<uint32_t>& indices)
{
    std::vector<glm::vec3> normals(positions.size(), glm::vec3(0.0f));
    
    // Accumulate face normals for each vertex
    for (size_t i = 0; i < indices.size(); i += 3) {
        const uint32_t i0 = indices[i];
        const uint32_t i1 = indices[i + 1];
        const uint32_t i2 = indices[i + 2];
        
        const glm::vec3& p0 = positions[i0];
        const glm::vec3& p1 = positions[i1];
        const glm::vec3& p2 = positions[i2];
        
        const glm::vec3 edge1 = p1 - p0;
        const glm::vec3 edge2 = p2 - p0;
        const glm::vec3 faceNormal = glm::cross(edge1, edge2);
        
        normals[i0] += faceNormal;
        normals[i1] += faceNormal;
        normals[i2] += faceNormal;
    }
    
    // Normalize all normals
    for (auto& n : normals) {
        const float length = glm::length(n);
        if (length > 0.0001f) {
            n /= length;
        } else {
            n = glm::vec3(0.0f, 1.0f, 0.0f); // Default normal
        }
    }
    
    return normals;
}

} // namespace

MeshData loadGltfFile(const std::string& path) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    
    bool loaded = false;
    if (path.size() >= 4 && path.compare(path.size() - 4, 4, ".glb") == 0) {
        loaded = loader.LoadBinaryFromFile(&model, &err, &warn, path);
    } else if (path.size() >= 5 && path.compare(path.size() - 5, 5, ".gltf") == 0) {
        loaded = loader.LoadASCIIFromFile(&model, &err, &warn, path);
    } else {
        throw std::runtime_error("Unsupported file format: " + path + " (expected .gltf or .glb)");
    }
    
    if (!loaded) {
        throw std::runtime_error("Failed to load glTF file: " + path + "\nError: " + err);
    }
    
    if (!warn.empty()) {
        // Warning messages are logged but don't fail loading
    }
    
    MeshData result;
    result.materials = {makeDefaultMaterial("default")};
    
    // Parse the first scene (if available)
    if (model.scenes.empty()) {
        throw std::runtime_error("glTF file contains no scenes: " + path);
    }
    
    const tinygltf::Scene& scene = model.scenes[0];
    
    // Process all nodes in the scene
    std::vector<glm::vec3> allPositions;
    std::vector<glm::vec3> allNormals;
    std::vector<uint32_t> allIndices;
    
    for (int nodeIndex : scene.nodes) {
        if (nodeIndex < 0 || nodeIndex >= static_cast<int>(model.nodes.size())) {
            continue;
        }
        
        const tinygltf::Node& node = model.nodes[nodeIndex];
        
        if (node.mesh < 0 || node.mesh >= static_cast<int>(model.meshes.size())) {
            continue;
        }
        
        const tinygltf::Mesh& mesh = model.meshes[node.mesh];
        
        // Process each primitive in the mesh
        for (const auto& primitive : mesh.primitives) {
            // Get position attribute (required)
            auto posIter = primitive.attributes.find("POSITION");
            if (posIter == primitive.attributes.end()) {
                continue; // Skip primitives without positions
            }
            
            const tinygltf::Accessor& posAccessor = model.accessors[posIter->second];
            std::vector<glm::vec3> positions = extractVec3Attribute(model, posAccessor);
            
            // Get normal attribute, or compute if missing
            std::vector<glm::vec3> normals;
            auto normIter = primitive.attributes.find("NORMAL");
            if (normIter != primitive.attributes.end()) {
                const tinygltf::Accessor& normAccessor = model.accessors[normIter->second];
                normals = extractVec3Attribute(model, normAccessor);
            } else {
                // Will compute normals after we have indices
                normals.resize(positions.size(), glm::vec3(0.0f, 1.0f, 0.0f));
            }
            
            // Get indices if available
            std::vector<uint32_t> indices;
            if (primitive.indices >= 0 && primitive.indices < static_cast<int>(model.accessors.size())) {
                const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
                indices = extractIndices(model, indexAccessor);
            } else {
                // No indices — create index list for triangle fan/strip
                indices.resize(positions.size());
                for (size_t i = 0; i < indices.size(); ++i) {
                    indices[i] = static_cast<uint32_t>(i);
                }
            }
            
            // Compute normals if missing
            auto normIter2 = primitive.attributes.find("NORMAL");
            if (normIter2 == primitive.attributes.end() && !indices.empty()) {
                normals = computeNormals(positions, indices);
            }
            
            if (normals.size() != positions.size()) {
                normals.resize(positions.size(), glm::vec3(0.0f, 1.0f, 0.0f));
            }
            
            // Extract material index if available
            uint32_t materialIndex = 0;
            if (primitive.material >= 0 && primitive.material < static_cast<int>(model.materials.size())) {
                materialIndex = static_cast<uint32_t>(primitive.material);
                
                // Ensure we have enough materials
                while (result.materials.size() <= materialIndex) {
                    result.materials.push_back(makeDefaultMaterial("material_" + std::to_string(result.materials.size())));
                }
                
                // Fill in material data from glTF
                const tinygltf::Material& gltfMat = model.materials[primitive.material];
                if (!gltfMat.name.empty() && result.materials[materialIndex].name.empty()) {
                    result.materials[materialIndex].name = gltfMat.name;
                }
                
                // Extract diffuse color from base color factor
                if (!gltfMat.values.empty()) {
                    auto colorIter = gltfMat.values.find("baseColorFactor");
                    if (colorIter != gltfMat.values.end() && colorIter->second.ColorFactor().size() >= 3) {
                        const auto& color = colorIter->second.ColorFactor();
                        result.materials[materialIndex].diffuse[0] = static_cast<float>(color[0]);
                        result.materials[materialIndex].diffuse[1] = static_cast<float>(color[1]);
                        result.materials[materialIndex].diffuse[2] = static_cast<float>(color[2]);
                    }
                }
            }
            
            // Add vertices with material index
            const uint32_t vertexOffset = static_cast<uint32_t>(allPositions.size());
            for (size_t i = 0; i < positions.size(); ++i) {
                Vertex v;
                v.position[0] = positions[i].x;
                v.position[1] = positions[i].y;
                v.position[2] = positions[i].z;
                v.normal[0] = normals[i].x;
                v.normal[1] = normals[i].y;
                v.normal[2] = normals[i].z;
                v.materialIndex = materialIndex;
                result.vertices.push_back(v);
            }
            
            // Add indices with offset
            for (uint32_t idx : indices) {
                result.indices.push_back(vertexOffset + idx);
            }
        }
    }
    
    if (result.vertices.empty() || result.indices.empty()) {
        throw std::runtime_error("glTF file contains no drawable geometry: " + path);
    }
    
    return result;
}

} // namespace ge
