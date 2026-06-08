#include "engine/scene/SceneGraph.h"

namespace ge {

void SceneGraph::clear() {
    nodes_.clear();
}

void SceneGraph::addNode(SceneNode node) {
    nodes_.push_back(std::move(node));
}

MeshData SceneGraph::buildMesh() const {
    MeshData mesh;

    for (const auto& node : nodes_) {
        const auto materialOffset = static_cast<uint32_t>(mesh.materials.size());

        // Append mesh materials
        mesh.materials.insert(mesh.materials.end(), node.mesh.materials.begin(), node.mesh.materials.end());

        // Append vertices with remapped material indices
        for (const auto& vertex : node.mesh.vertices) {
            auto adjusted = vertex;
            adjusted.materialIndex += materialOffset;
            mesh.vertices.push_back(adjusted);
        }

        // Append indices with rewritten vertex offsets
        const auto vertexOffset = static_cast<uint32_t>(mesh.vertices.size()) - static_cast<uint32_t>(node.mesh.vertices.size());
        for (const auto index : node.mesh.indices) {
            mesh.indices.push_back(vertexOffset + index);
        }
    }

    return mesh;
}

bool SceneGraph::empty() const {
    return nodes_.empty();
}

} // namespace ge
