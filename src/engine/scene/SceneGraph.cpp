#include "engine/scene/SceneGraph.h"

#include <cstdint>
#include <utility>

namespace ge {
namespace {

void appendNodeToMesh(MeshData& mesh, const SceneNode& node) {
    const auto materialOffset = static_cast<uint32_t>(mesh.materials.size());

    mesh.materials.insert(mesh.materials.end(), node.mesh.materials.begin(), node.mesh.materials.end());

    for (const auto& vertex : node.mesh.vertices) {
        auto adjusted = vertex;
        adjusted.materialIndex += materialOffset;
        mesh.vertices.push_back(adjusted);
    }

    const auto vertexOffset = static_cast<uint32_t>(mesh.vertices.size()) - static_cast<uint32_t>(node.mesh.vertices.size());
    for (const auto index : node.mesh.indices) {
        mesh.indices.push_back(vertexOffset + index);
    }
}

} // namespace

void SceneGraph::clear() {
    nodes_.clear();
}

void SceneGraph::addNode(SceneNode node) {
    nodes_.push_back(std::move(node));
}

MeshData SceneGraph::buildMesh() const {
    MeshData mesh;

    std::size_t totalMaterials = 0;
    std::size_t totalVertices = 0;
    std::size_t totalIndices = 0;
    for (const auto& node : nodes_) {
        totalMaterials += node.mesh.materials.size();
        totalVertices += node.mesh.vertices.size();
        totalIndices += node.mesh.indices.size();
    }

    mesh.materials.reserve(totalMaterials);
    mesh.vertices.reserve(totalVertices);
    mesh.indices.reserve(totalIndices);

    for (const auto& node : nodes_) {
        appendNodeToMesh(mesh, node);
    }

    return mesh;
}

bool SceneGraph::empty() const noexcept {
    return nodes_.empty();
}

std::size_t SceneGraph::nodeCount() const noexcept {
    return nodes_.size();
}

} // namespace ge
