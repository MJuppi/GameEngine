#pragma once

#include "engine/asset/TextureData.h"
#include "engine/mesh/MeshData.h"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

namespace ge {

class AssetManager {
public:
    AssetManager() = default;
    ~AssetManager() = default;

    AssetManager(const AssetManager&) = delete;
    AssetManager& operator=(const AssetManager&) = delete;

    [[nodiscard]] const MeshData& loadMesh(const std::string& path);
    [[nodiscard]] bool hasMesh(const std::string& path) const;
    [[nodiscard]] const MeshData& getMesh(const std::string& path) const;

    [[nodiscard]] const TextureData& loadTexture(const std::string& path);
    [[nodiscard]] bool hasTexture(const std::string& path) const;
    [[nodiscard]] const TextureData& getTexture(const std::string& path) const;

private:
    std::unordered_map<std::string, MeshData> meshCache_;
    std::unordered_map<std::string, TextureData> textureCache_;
};

} // namespace ge
