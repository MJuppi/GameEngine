#pragma once

#include "engine/asset/TextureData.h"
#include "engine/mesh/MeshData.h"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <future>
#include "engine/asset/AssetManifest.h"

namespace ge {

class AssetManager {
public:
    AssetManager() = default;
    ~AssetManager() = default;

    AssetManager(const AssetManager&) = delete;
    AssetManager& operator=(const AssetManager&) = delete;

    [[nodiscard]] const MeshData& loadMesh(const std::string& path);
    [[nodiscard]] std::future<std::shared_ptr<MeshData>> loadMeshAsync(const std::string& path);
    [[nodiscard]] bool hasMesh(const std::string& path) const;
    [[nodiscard]] const MeshData& getMesh(const std::string& path) const;

    [[nodiscard]] const TextureData& loadTexture(const std::string& path);
    [[nodiscard]] std::future<std::shared_ptr<TextureData>> loadTextureAsync(const std::string& path);
    [[nodiscard]] bool hasTexture(const std::string& path) const;
    [[nodiscard]] const TextureData& getTexture(const std::string& path) const;

    void loadManifest(const std::filesystem::path& manifestPath);

private:
    std::unordered_map<std::string, std::shared_ptr<MeshData>> meshCache_;
    std::unordered_map<std::string, std::shared_ptr<TextureData>> textureCache_;

    AssetManifest manifest_{};
};

} // namespace ge
