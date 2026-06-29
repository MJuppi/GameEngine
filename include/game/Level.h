#pragma once
#include "engine/mesh/MeshData.h"
#include <string>

namespace ge {

class AssetManager; // forward

class Level {
public:
    explicit Level(std::string name, std::string meshPath = {});
    ~Level() = default;

    const std::string& getName() const { return name_; }
    const std::string& getMeshPath() const { return meshPath_; }
    const MeshData& getMesh() const { return mesh_; }
    bool isLoaded() const { return loaded_; }

    void load(AssetManager& assetManager);
    void unload();

private:
    std::string name_;
    std::string meshPath_;
    MeshData mesh_;
    bool loaded_ = false;
};

} // namespace ge
