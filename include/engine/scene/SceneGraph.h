#pragma once

#include "engine/mesh/MeshData.h"

#include <string>
#include <vector>

namespace ge {

struct SceneNode {
    std::string name;
    MeshData mesh;
};

class SceneGraph {
public:
    SceneGraph() = default;
    ~SceneGraph() = default;

    SceneGraph(const SceneGraph&) = delete;
    SceneGraph& operator=(const SceneGraph&) = delete;

    void clear();
    void addNode(SceneNode node);
    [[nodiscard]] MeshData buildMesh() const;
    [[nodiscard]] bool empty() const;

private:
    std::vector<SceneNode> nodes_;
};

} // namespace ge
