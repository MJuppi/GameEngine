#pragma once

#include "engine/asset/AssetManager.h"
#include "engine/mesh/MeshData.h"
#include "engine/scene/SceneGraph.h"
#include <string>
#include <vector>

namespace ge {

class Level {
public:
    Level(const std::string& name, const std::string& meshPath = "");
    
    // Lifecycle
    void load();
    void unload();
    bool isLoaded() const { return loaded_; }
    
    // Getters
    const std::string& getName() const { return name_; }
    const std::string& getMeshPath() const { return meshPath_; }
    const MeshData& getMesh() const { return mesh_; }
    MeshData& getMesh() { return mesh_; }
    
    // Setters
    void setMeshPath(const std::string& path) { meshPath_ = path; }
    void setMesh(const MeshData& mesh) { mesh_ = mesh; }
    
private:
    std::string name_;
    std::string meshPath_;
    MeshData mesh_;
    bool loaded_;
    AssetManager assetManager_;
};

}  // namespace ge
